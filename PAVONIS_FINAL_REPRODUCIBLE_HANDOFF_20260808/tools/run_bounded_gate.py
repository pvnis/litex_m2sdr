#!/usr/bin/env python3
"""Run and validate the CP3424-class M2SDR/bladeRF bounded-data gate."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys


RUNNER = Path(
    "cp3422_lead11_mcs19_robust_no_rf_gate_20260731/"
    "cp3423_run_lead11_mcs19_preconnected.sh"
)


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
    parser.add_argument("--site", required=True)
    parser.add_argument("--harness", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--stamp", required=True)
    args = parser.parse_args()
    if os.environ.get("PAVONIS_RF_APPROVED") != "1":
        print("PAVONIS_BOUNDED_RF_GATE=CLOSED")
        return 2
    if not re.fullmatch(r"[A-Za-z0-9_]+", args.stamp):
        raise SystemExit("bounded-stage stamp must contain only letters, digits, or underscore")
    harness = args.harness.resolve()
    verify_rendered(harness)
    runner = harness / RUNNER
    if not runner.is_file():
        raise SystemExit(f"bounded runner is missing: {runner}")
    output = args.output_dir.resolve()
    if output.exists():
        raise SystemExit(f"bounded output already exists: {output}")
    output.mkdir(mode=0o700, parents=True)
    log = output / "controller.log"
    env = os.environ.copy()
    env.update(
        {
            "ART": str(harness),
            "PAVONIS_SITE_CONFIG": str(Path(args.site).resolve()),
            "PAVONIS_BOUNDED_RESULT_ROOT": str(output / "results"),
            "PAVONIS_CP3423_RF_APPROVED": "1",
            "STAMP": args.stamp,
        }
    )
    with log.open("w", encoding="utf-8") as handle:
        run = subprocess.run(
            [str(runner)], env=env, stdout=handle, stderr=subprocess.STDOUT,
            check=False,
        )
    attempt = output / "results" / f"attempt_{args.stamp}"
    execution = attempt / "execution_summary.json"
    sustained = attempt / "sustained_analysis.json"
    result = "FAIL"
    reasons: list[str] = []
    execution_data: dict = {}
    sustained_data: dict = {}
    if run.returncode != 0:
        reasons.append(f"runner_rc={run.returncode}")
    for label, path in (("execution", execution), ("sustained", sustained)):
        if not path.is_file():
            reasons.append(f"missing_{label}_summary")
    if execution.is_file():
        execution_data = json.loads(execution.read_text(encoding="utf-8"))
        if execution_data.get("verdict") != "pass":
            reasons.append("execution_verdict_not_pass")
        health = execution_data.get("radio_health", {})
        if health.get("cell_select_ok") is not True:
            reasons.append("cell_selection_not_pass")
        if int(health.get("pusch_crc_ok") or 0) < 1:
            reasons.append("no_crc_ok_pusch")
        if int(health.get("soapy_tx_timeout_count") or 0) != 0:
            reasons.append("soapy_tx_timeout")
        if int(health.get("downlink_late_count") or 0) != 0:
            reasons.append("downlink_late")
    if sustained.is_file():
        sustained_data = json.loads(sustained.read_text(encoding="utf-8"))
        if sustained_data.get("verdict") != "pass":
            reasons.append("sustained_verdict_not_pass")
    if not reasons:
        result = "PASS"
    summary = {
        "schema": 1,
        "stage": "bounded-m2sdr-bladerf",
        "stamp": args.stamp,
        "result": result,
        "reference": "CP3424",
        "reasons": reasons,
        "runner_rc": run.returncode,
        "radio_health": execution_data.get("radio_health", {}),
        "throughput": sustained_data,
        "artifact_sha256": {
            path.name: digest(path)
            for path in (log, execution, sustained)
            if path.is_file()
        },
    }
    summary_path = output / "summary.json"
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"PAVONIS_BOUNDED_SUMMARY={summary_path}")
    print(f"PAVONIS_BOUNDED_STAGE={result}")
    return 0 if result == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
