#!/usr/bin/env python3
"""Validate receiver-side parallel TCP results and emit a compact verdict."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--core", type=Path, required=True)
    parser.add_argument("--ue", type=Path, required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--streams", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    core = json.loads(args.core.read_text(encoding="utf-8"))
    ue = json.loads(args.ue.read_text(encoding="utf-8"))
    downlink = ue["downlink"]
    uplink = core["directions"]["uplink"]
    low = args.duration - 1
    high = args.duration + 2
    duration_gate = (
        low <= float(downlink["seconds"]) <= high
        and low <= float(uplink["seconds"]) <= high
    )
    stream_gate = (
        int(core.get("streams", 0)) == args.streams
        and int(ue.get("streams", 0)) == args.streams
        and len(downlink.get("per_stream", [])) == args.streams
        and len(uplink.get("per_stream", [])) == args.streams
    )
    complete = bool(core.get("complete")) and bool(ue.get("complete"))
    positive = int(downlink["bytes"]) > 0 and int(uplink["bytes"]) > 0
    result = {
        "schema": 2,
        "verdict": (
            "pass" if complete and duration_gate and stream_gate and positive else "fail"
        ),
        "method": {
            "transport": "parallel TCP over the private srsUE data session",
            "streams": args.streams,
            "directions": "sequential",
            "seconds_per_direction": args.duration,
            "downlink_authority": "ue_receiver",
            "uplink_authority": "core_receiver",
        },
        "duration_gate_pass": duration_gate,
        "stream_gate_pass": stream_gate,
        "sustained": {
            "downlink_bytes": int(downlink["bytes"]),
            "downlink_seconds": float(downlink["seconds"]),
            "downlink_megabits_per_second": float(downlink["bytes_per_second"]) * 8 / 1e6,
            "uplink_bytes": int(uplink["bytes"]),
            "uplink_seconds": float(uplink["seconds"]),
            "uplink_megabits_per_second": float(uplink["bytes_per_second"]) * 8 / 1e6,
        },
        "endpoint_crosscheck": {
            "downlink_core_bytes": int(core["directions"]["downlink"]["bytes"]),
            "uplink_ue_bytes": int(ue["uplink"]["bytes"]),
        },
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return 0 if result["verdict"] == "pass" else 1


if __name__ == "__main__":
    raise SystemExit(main())
