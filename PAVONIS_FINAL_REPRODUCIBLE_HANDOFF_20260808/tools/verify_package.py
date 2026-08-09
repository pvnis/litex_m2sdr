#!/usr/bin/env python3
"""Verify package integrity, source bundles, privacy, and script syntax."""

from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile


PACKAGE = Path(__file__).resolve().parents[1]
MANIFEST = PACKAGE / "manifests" / "SHA256SUMS"
HISTORY = PACKAGE / "history" / "sanitized-checkpoints-final"
HISTORY_MANIFEST_ORDER = ("HASH_MANIFEST.tsv",)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_manifest() -> list[str]:
    failures: list[str] = []
    expected: set[str] = set()
    for line in MANIFEST.read_text(encoding="ascii").splitlines():
        value, relative = line.split(maxsplit=1)
        relative = relative.removeprefix("./")
        expected.add(relative)
        path = PACKAGE / relative
        if not path.is_file() or digest(path) != value:
            failures.append(f"manifest:{relative}")
    actual = {
        path.relative_to(PACKAGE).as_posix()
        for path in PACKAGE.rglob("*")
        if path.is_file()
        and path != MANIFEST
        and "work" not in path.relative_to(PACKAGE).parts
        and "__pycache__" not in path.parts
    }
    if expected != actual:
        for relative in sorted(expected ^ actual):
            failures.append(f"manifest-membership:{relative}")
    return failures


def verify_sanitized_history() -> list[str]:
    failures: list[str] = []
    actual_manifests = {path.name for path in HISTORY.glob("HASH_MANIFEST*.tsv")}
    if actual_manifests != set(HISTORY_MANIFEST_ORDER):
        failures.append("history-manifest-set")
    final_hashes: dict[str, tuple[str, str]] = {}
    malformed: list[str] = []
    for name in HISTORY_MANIFEST_ORDER:
        manifest = HISTORY / name
        if not manifest.is_file():
            malformed.append(name)
            continue
        with manifest.open(encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle, delimiter="\t")
            if reader.fieldnames != [
                "sanitized_file", "source_sha256", "sanitized_sha256"
            ]:
                malformed.append(manifest.name)
                continue
            for row in reader:
                relative = row["sanitized_file"]
                final_hashes[relative] = (manifest.name, row["sanitized_sha256"])
    if malformed:
        failures.append(
            f"history-manifest-malformed:{len(malformed)}:first={malformed[0]}"
        )

    mismatches = []
    for relative, (manifest_name, expected) in final_hashes.items():
        path = HISTORY / relative
        if not path.is_file() or digest(path) != expected:
            mismatches.append(f"{manifest_name}:{relative}")
    if mismatches:
        failures.append(
            f"history-current-hash:{len(mismatches)}:first={mismatches[0]}"
        )

    reports = {path.name for path in HISTORY.glob("CP*.md")}
    uncovered = sorted(reports - set(final_hashes))
    if uncovered:
        failures.append(
            f"history-uncovered:{len(uncovered)}:first={uncovered[0]}"
        )
    checkpoint_ids = []
    for name in reports:
        match = re.match(r"CP(\d+)", name)
        if match:
            checkpoint_ids.append(int(match.group(1)))
    if (
        len(reports) != 3587
        or len(set(checkpoint_ids)) != 3586
        or min(checkpoint_ids, default=0) != 17
        or max(checkpoint_ids, default=0) != 3637
        or 3633 in checkpoint_ids
    ):
        failures.append("history-coverage:expected-CP17-through-CP3637-with-declared-gaps")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--skip-manifest", action="store_true")
    args = parser.parse_args()
    failures: list[str] = []
    if not args.skip_manifest:
        if not MANIFEST.is_file():
            failures.append("manifest:missing")
        else:
            failures.extend(verify_manifest())
    failures.extend(verify_sanitized_history())
    with tempfile.TemporaryDirectory(prefix="pavonis-bundle-verify.") as temporary:
        subprocess.run(["git", "init", "--quiet", temporary], check=True)
        for bundle in sorted((PACKAGE / "source" / "bundles").glob("*.bundle")):
            check = subprocess.run(
                ["git", "-C", temporary, "bundle", "verify", str(bundle)],
                capture_output=True,
            )
            if check.returncode:
                failures.append(f"bundle:{bundle.name}")
    privacy = subprocess.run([sys.executable, str(PACKAGE / "tools" / "privacy_scan.py")])
    if privacy.returncode:
        failures.append("privacy")
    for script in sorted((PACKAGE / "harness" / "templates").rglob("*.sh")):
        if subprocess.run(["bash", "-n", str(script)]).returncode:
            failures.append(f"bash-syntax:{script.relative_to(PACKAGE)}")
    python_files = list((PACKAGE / "tools").glob("*.py"))
    python_files += list((PACKAGE / "harness" / "templates").rglob("*.py"))
    for script in sorted(python_files):
        try:
            ast.parse(script.read_text(encoding="utf-8"), filename=str(script))
        except (OSError, SyntaxError, UnicodeDecodeError):
            failures.append(f"python-syntax:{script.relative_to(PACKAGE)}")
    selftest_env = os.environ.copy()
    selftest_env["PYTHONDONTWRITEBYTECODE"] = "1"
    selftest = subprocess.run(
        [sys.executable, str(PACKAGE / "tools" / "selftest.py")],
        env=selftest_env,
    )
    if selftest.returncode:
        failures.append("portable-controller-selftest")
    if failures:
        for failure in failures:
            print(f"VERIFY_FAIL: {failure}", file=sys.stderr)
        return 1
    print("PAVONIS_FINAL_PACKAGE_VERIFY=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
