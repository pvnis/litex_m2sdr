#!/usr/bin/env python3
"""Count selected Qualcomm Diag log codes without exposing packet payloads."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from collections import Counter
from pathlib import Path
import struct


EXPECTED = {0xB821, 0xB889, 0xB88A}
HEADER = struct.Struct("<HHQ")
DIAG_EPOCH_UNIX = datetime(1980, 1, 6, tzinfo=timezone.utc).timestamp()


def decode_timestamp(raw: int) -> str | None:
    seconds = (raw >> 20) / 50 + DIAG_EPOCH_UNIX
    seconds += (raw & 0xFFFFF) / 0x100000
    if not (datetime(2010, 1, 1, tzinfo=timezone.utc).timestamp() <= seconds <=
            datetime(2050, 1, 1, tzinfo=timezone.utc).timestamp()):
        return None
    return datetime.fromtimestamp(seconds, timezone.utc).isoformat()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("dlf", type=Path)
    args = parser.parse_args()

    data = args.dlf.read_bytes()
    offset = 0
    counts: Counter[int] = Counter()
    first_timestamp: dict[int, str] = {}
    last_timestamp: dict[int, str] = {}
    malformed = 0
    while offset + HEADER.size <= len(data):
        length, code, raw_timestamp = HEADER.unpack_from(data, offset)
        if length < HEADER.size or offset + length > len(data):
            malformed += 1
            break
        counts[code] += 1
        timestamp = decode_timestamp(raw_timestamp)
        if timestamp is not None:
            first_timestamp.setdefault(code, timestamp)
            last_timestamp[code] = timestamp
        offset += length

    result = {
        "bytes": len(data),
        "records": sum(counts.values()),
        "counts": {f"0x{code:04x}": counts[code] for code in sorted(EXPECTED)},
        "first_timestamp_utc": {
            f"0x{code:04x}": first_timestamp.get(code)
            for code in sorted(EXPECTED)
        },
        "last_timestamp_utc": {
            f"0x{code:04x}": last_timestamp.get(code)
            for code in sorted(EXPECTED)
        },
        "unexpected_codes": {
            f"0x{code:04x}": count
            for code, count in sorted(counts.items())
            if code not in EXPECTED
        },
        "trailing_bytes": len(data) - offset,
        "malformed": malformed,
    }
    print(json.dumps(result, sort_keys=True))
    return 0 if not result["unexpected_codes"] and malformed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
