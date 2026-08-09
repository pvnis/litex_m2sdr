#!/usr/bin/env python3
"""Run one proven phone profile and emit a compact stage receipt."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import selectors
import subprocess
import sys
import time

from remote_common import load_site, role


FAST = Path(
    "cp3612_phone_15mhz_all_mcs17_no_rf_gate_20260802/"
    "cp3612h_execute_phone_15mhz_all_mcs17_attempt.sh"
)
SLOW = Path("cp3121h_execute_metric_al8_phone_rf.sh")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_rendered(harness: Path) -> None:
    manifest = harness / "RENDERED_SHA256SUMS"
    if not manifest.is_file():
        raise SystemExit("rendered harness manifest is missing")
    for line in manifest.read_text(encoding="ascii").splitlines():
        expected, relative = line.split(maxsplit=1)
        path = harness / relative.removeprefix("./")
        if not path.is_file() or digest(path) != expected:
            raise SystemExit(f"rendered harness hash mismatch: {relative}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("profile", choices=("slow", "fast"))
    parser.add_argument("--site", required=True)
    parser.add_argument("--harness", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--stamp", required=True)
    parser.add_argument("--hold", action="store_true")
    parser.add_argument("--public-proof", action="store_true")
    args = parser.parse_args()
    if os.environ.get("PAVONIS_RF_APPROVED") != "1":
        print("PAVONIS_PHONE_RF_GATE=CLOSED")
        return 2
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", args.stamp):
        raise SystemExit("invalid phone-stage stamp")
    harness = args.harness.resolve()
    verify_rendered(harness)
    runner = harness / (FAST if args.profile == "fast" else SLOW)
    if not runner.is_file():
        raise SystemExit(f"phone runner is missing: {runner}")
    output = args.output_dir.resolve()
    if output.exists():
        raise SystemExit(f"phone output already exists: {output}")
    output.mkdir(mode=0o700, parents=True)
    result_root = harness / "stage-results" / f"phone-{args.stamp}"
    result_root.mkdir(mode=0o700, parents=True)
    log = output / "controller.log"
    public_log = output / "public-egress.log"
    env = os.environ.copy()
    env.update(
        {
            "ART": str(harness),
            "PAVONIS_SITE_CONFIG": str(Path(args.site).resolve()),
            "PAVONIS_HOLD_BEFORE_TEARDOWN": "1" if (args.hold or args.public_proof) else "0",
            "STAMP": args.stamp,
        }
    )
    if args.profile == "fast":
        env["PAVONIS_CP3612_EXECUTE_RF_APPROVED"] = "1"
        env["PAVONIS_CP3612_RESULT_ROOT"] = str(result_root)
    public_rc: int | None = None
    with log.open("w", encoding="utf-8") as handle:
        if not args.public_proof:
            run = subprocess.run(
                [str(runner)], env=env, stdout=handle, stderr=subprocess.STDOUT,
                check=False,
            )
        else:
            process = subprocess.Popen(
                [str(runner)], env=env, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1,
            )
            assert process.stdout is not None
            selector = selectors.DefaultSelector()
            selector.register(process.stdout, selectors.EVENT_READ)
            deadline = time.monotonic() + 1000
            hold_seen = False
            while time.monotonic() < deadline:
                for key, _ in selector.select(timeout=1):
                    line = key.fileobj.readline()
                    if line:
                        handle.write(line)
                        handle.flush()
                        if "HOLDING BEFORE TEARDOWN" in line:
                            hold_seen = True
                            break
                if hold_seen:
                    break
                if process.poll() is not None:
                    break
            selector.close()
            if hold_seen:
                try:
                    site = load_site(args.site)
                    remote_helper = (
                        role(site, "ue").home + "/pavonis_phone_public_egress_remote.sh"
                    )
                    public_rc = subprocess.run(
                        [
                            sys.executable, str(harness / "ssh_pexpect_run.py"),
                            "--site", str(Path(args.site).resolve()),
                            "--host", "ue", "--out", str(public_log), "--timeout", "120",
                            "--", remote_helper,
                        ],
                        check=False,
                    ).returncode
                except (OSError, ValueError):
                    public_rc = 1
                finally:
                    if not args.hold:
                        prefix = "cp3612h" if args.profile == "fast" else "cp3121h"
                        (harness / f"{prefix}_{args.stamp}_RELEASE").touch(mode=0o600)
            elif process.poll() is None:
                process.terminate()
            try:
                remainder, _ = process.communicate(timeout=3700 if args.hold else 180)
                handle.write(remainder)
                runner_rc = process.returncode
            except subprocess.TimeoutExpired:
                process.terminate()
                remainder, _ = process.communicate(timeout=30)
                handle.write(remainder)
                runner_rc = 124
            run = subprocess.CompletedProcess([str(runner)], runner_rc)
    reasons: list[str] = []
    artifacts = [log]
    metrics: dict = {}
    if run.returncode != 0:
        reasons.append(f"runner_rc={run.returncode}")
    if args.public_proof:
        if public_rc != 0:
            reasons.append(f"public_proof_rc={public_rc}")
        elif not public_log.is_file() or "PAVONIS_PHONE_PUBLIC_EGRESS=PASS" not in public_log.read_text(
            encoding="utf-8", errors="replace"
        ):
            reasons.append("public_egress_marker_missing")
        else:
            artifacts.append(public_log)
    if args.profile == "fast":
        analysis = result_root / f"attempt_{args.stamp}" / "sustained_analysis.json"
        if not analysis.is_file():
            reasons.append("missing_sustained_analysis")
        else:
            metrics = json.loads(analysis.read_text(encoding="utf-8"))
            artifacts.append(analysis)
            if metrics.get("verdict") != "pass":
                reasons.append("sustained_verdict_not_pass")
    else:
        text = log.read_text(encoding="utf-8", errors="replace")
        if "PAVONIS_FINAL_DATA_GATE_PASS=1" not in text:
            reasons.append("final_data_marker_missing")
    result = "PASS" if not reasons else "FAIL"
    summary = {
        "schema": 1,
        "stage": f"phone-{args.profile}",
        "stamp": args.stamp,
        "result": result,
        "reference": "CP3631",
        "reasons": reasons,
        "runner_rc": run.returncode,
        "public_egress_proven": args.public_proof and not any(
            reason.startswith("public_") for reason in reasons
        ),
        "metrics": metrics,
        "artifact_sha256": {
            path.name: digest(path) for path in artifacts if path.is_file()
        },
    }
    summary_path = output / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"PAVONIS_PHONE_SUMMARY={summary_path}")
    print(f"PAVONIS_PHONE_STAGE={result}")
    return 0 if result == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
