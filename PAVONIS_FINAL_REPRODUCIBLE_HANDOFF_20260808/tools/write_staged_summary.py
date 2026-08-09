#!/usr/bin/env python3
"""Validate stage receipts and write the final staged-run receipt."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stamp", required=True)
    parser.add_argument("--profile", required=True, choices=("slow", "fast"))
    parser.add_argument("--zmq", required=True, type=Path)
    parser.add_argument("--bounded", required=True, type=Path)
    parser.add_argument("--phone", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    stages: dict[str, dict] = {}
    for name, path in (("zmq", args.zmq), ("bounded", args.bounded), ("phone", args.phone)):
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("result") not in {"PASS", "pass"}:
            raise SystemExit(f"{name} stage receipt is not PASS")
        stages[name] = {
            "summary": str(path.resolve()),
            "sha256": digest(path),
            "result": data["result"],
        }
    summary = {
        "schema": 1,
        "result": "PASS",
        "stamp": args.stamp,
        "profile": args.profile,
        "ordered_stages": ["zmq", "bounded", "phone"],
        "stages": stages,
    }
    args.output.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"PAVONIS_STAGED_SUMMARY={args.output.resolve()}")
    print("PAVONIS_STAGED_RUN=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
