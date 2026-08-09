#!/usr/bin/env python3
"""Wait incrementally for a new qcore user-plane activation."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()

    deadline = time.monotonic() + args.timeout
    with args.log.open("r", encoding="utf-8", errors="replace") as stream:
        stream.seek(0, 2)
        print("CP2883_TARGET_ACTIVATION_WATCHER=ARMED", flush=True)
        while time.monotonic() < deadline:
            line = stream.readline()
            if not line:
                time.sleep(0.05)
                continue
            if "Activate userplane session UE IP" in line:
                print("CP2883_TARGET_ACTIVATION=PASS", flush=True)
                return 0

    print("CP2883_TARGET_ACTIVATION=TIMEOUT", flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main())
