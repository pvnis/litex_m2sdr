#!/usr/bin/env python3
"""Join CP3155 deadline-bounded endpoint measurements."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


CP3133_DOWNLINK_BPS = 786_864
CP3133_UPLINK_BPS = 12_012


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def ratio(value: float, baseline: float) -> float:
    return value / baseline if baseline else 0.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--phone", type=Path, required=True)
    parser.add_argument("--core", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    phone = load(args.phone)
    core = load(args.core)
    phone_complete = phone.get("complete") is True
    core_complete = core.get("complete") is True

    downlink_bps = float(phone["downlink"]["bytes_per_second"])
    uplink_bps = float(core["directions"]["uplink"]["bytes_per_second"])
    requested = int(phone["requested_seconds_per_direction"])
    duration_ok = (
        float(phone["downlink"]["seconds"]) >= requested - 1
        and float(phone["uplink"]["seconds"]) >= requested - 1
        and float(core["directions"]["downlink"]["seconds"]) >= requested - 1
        and requested - 1
        <= float(core["directions"]["uplink"]["seconds"])
        <= requested + 2
    )
    crosscheck = {
        "downlink_phone_bytes_per_second": downlink_bps,
        "downlink_core_bytes_per_second": float(
            core["directions"]["downlink"]["bytes_per_second"]
        ),
        "uplink_phone_bytes_per_second": float(
            phone["uplink"]["bytes_per_second"]
        ),
        "uplink_core_bytes_per_second": uplink_bps,
    }
    output = {
        "schema": 1,
        "verdict": (
            "pass" if phone_complete and core_complete and duration_ok else "invalid"
        ),
        "method": {
            "transport": "HTTP over the proven private phone PDU session",
            "seconds_per_direction": requested,
            "directions": "sequential",
            "rf_or_scheduler_config_changed": False,
            "uplink_deadline_enforced_by_receiver": True,
        },
        "sustained": {
            "downlink_bytes_per_second": downlink_bps,
            "downlink_megabits_per_second": downlink_bps * 8 / 1_000_000,
            "uplink_bytes_per_second": uplink_bps,
            "uplink_megabits_per_second": uplink_bps * 8 / 1_000_000,
        },
        "cp3133_one_shot_median": {
            "downlink_bytes_per_second": CP3133_DOWNLINK_BPS,
            "uplink_bytes_per_second": CP3133_UPLINK_BPS,
        },
        "ratio_to_cp3133": {
            "downlink": ratio(downlink_bps, CP3133_DOWNLINK_BPS),
            "uplink": ratio(uplink_bps, CP3133_UPLINK_BPS),
        },
        "duration_gate_pass": duration_ok,
        "endpoint_crosscheck": crosscheck,
    }
    args.output.write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0 if output["verdict"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
