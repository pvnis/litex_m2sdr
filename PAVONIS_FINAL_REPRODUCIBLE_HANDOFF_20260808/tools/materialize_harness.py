#!/usr/bin/env python3
"""Render site paths and regenerate the proven harness dependency hash chain."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import re
import sys

from remote_common import load_site


PACKAGE = Path(__file__).resolve().parents[1]
TEMPLATES = PACKAGE / "harness" / "templates"
PROVENANCE = PACKAGE / "harness" / "provenance" / "ORIGINAL_TEMPLATE_SHA256SUMS"
SHA256 = re.compile(r"^[0-9a-f]{64}$")


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def normalized_path(value: str) -> str:
    return value.removeprefix("./")


def source_hash_map() -> dict[str, str]:
    result: dict[str, str] = {}
    for line in PROVENANCE.read_text(encoding="utf-8").splitlines():
        old_hash, path = line.split(maxsplit=1)
        path = normalized_path(path)
        if path in {"ssh_pexpect_run.py", "scp_pexpect_put.py", "scp_pexpect_get.py"}:
            result[old_hash] = path
        elif (TEMPLATES / path).is_file():
            result[old_hash] = path
    return result


def render(site_file: str, output: Path) -> None:
    site = load_site(site_file)
    network = site.raw.get("network", {})
    core_n2_ip = str(network.get("core_n2_ip", ""))
    primer_s1_ip = str(network.get("primer_s1_ip", ""))
    ran_interface = str(network.get("ran_interface", ""))
    external_interface = str(network.get("external_interface", ""))
    for label, value in {
        "network.core_n2_ip": core_n2_ip,
        "network.primer_s1_ip": primer_s1_ip,
        "network.ran_interface": ran_interface,
        "network.external_interface": external_interface,
    }.items():
        if not value or value.startswith("REPLACE_"):
            raise ValueError(f"{label} is incomplete")

    if output.exists():
        raise ValueError(f"output already exists: {output}")
    output.mkdir(parents=True)
    replacements = {
        b"@REMOTE_HOME@": site.roles["ran"].home.encode(),
        b"@CONTROLLER_HARNESS@": str(output).encode(),
        b"@CONTROLLER_WORKSPACE@": str(PACKAGE).encode(),
        b"@CORE_N2_IP@": core_n2_ip.encode(),
        b"@PRIMER_S1_IP@": primer_s1_ip.encode(),
        b"@RAN_INTERFACE@": ran_interface.encode(),
        b"@EXTERNAL_INTERFACE@": external_interface.encode(),
    }
    runtime_overrides = site.raw.get("runtime", {}).get("pin_overrides", {})
    for old, new in runtime_overrides.items():
        if not SHA256.fullmatch(str(old)) or not SHA256.fullmatch(str(new)):
            raise ValueError("runtime.pin_overrides keys and values must be SHA-256")
        replacements[str(old).encode()] = str(new).encode()

    base: dict[str, bytes] = {}
    modes: dict[str, int] = {}
    for source in sorted(TEMPLATES.rglob("*")):
        if not source.is_file():
            continue
        relative = source.relative_to(TEMPLATES).as_posix()
        data = source.read_bytes()
        for old, new in replacements.items():
            data = data.replace(old, new)
        unresolved = re.findall(rb"@[A-Z0-9_]+@", data)
        if unresolved:
            raise ValueError(f"unresolved placeholders in {relative}: {unresolved[:3]}")
        base[relative] = data
        modes[relative] = source.stat().st_mode

    old_to_path = source_hash_map()
    state = dict(base)
    for _ in range(len(state) + 2):
        current_hashes = {path: digest(data) for path, data in state.items()}
        next_state: dict[str, bytes] = {}
        for path, original in base.items():
            data = original
            for old_hash, target in old_to_path.items():
                data = data.replace(old_hash.encode(), current_hashes[target].encode())
            next_state[path] = data
        if next_state == state:
            state = next_state
            break
        state = next_state
    else:
        raise ValueError("included dependency hash graph did not converge")

    for relative, data in state.items():
        destination = output / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(data)
        mode = modes[relative] & 0o777
        if relative.endswith((".sh", ".py")):
            mode |= 0o700
        destination.chmod(mode)

    baseline_names = [
        "cp3121_run_continuous_srsenb_to_ocudu_metrics_rf.sh",
        "cp3121_run_filtered_phone_metrics.sh",
        "cp3121_run_metric_al8_phone_rf.sh",
        "cp3121_execute_metric_al8_phone_rf.sh",
    ]
    manifest = output / "portable_baseline.sha256"
    manifest.write_text(
        "".join(f"{digest(state[name])}  {name}\n" for name in baseline_names),
        encoding="ascii",
    )
    rendered = output / "RENDERED_SHA256SUMS"
    rows = []
    for path in sorted(output.rglob("*")):
        if path.is_file() and path != rendered:
            rows.append(f"{digest(path.read_bytes())}  {path.relative_to(output).as_posix()}\n")
    rendered.write_text("".join(rows), encoding="ascii")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--site", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    try:
        render(args.site, args.output.resolve())
    except (OSError, ValueError) as exc:
        print(f"MATERIALIZE_FAIL: {exc}", file=sys.stderr)
        return 1
    print(f"PAVONIS_HARNESS_MATERIALIZED={args.output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
