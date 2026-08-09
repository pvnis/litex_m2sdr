#!/usr/bin/env python3
"""Classify qcore registration identities without emitting credential values."""

from __future__ import annotations

import argparse
import re
import sys
import tomllib
from pathlib import Path


REGISTERING_RE = re.compile(r"Registering imsi-([0-9]+)")
SIM_KEY_RE = re.compile(r"imsi-([0-9]+)")


def deletes_one_to_match(value: str, target: str) -> bool:
    return len(value) == len(target) + 1 and any(
        value[:index] + value[index + 1 :] == target for index in range(len(value))
    )


def three_digit_mnc_reordered(value: str) -> str:
    if len(value) < 6:
        return value
    return value[:3] + value[4:6] + value[3] + value[6:]


def relation(value: str, configured: set[str]) -> str:
    if value in configured:
        return "exact"
    if value.endswith("15") and value[:-2] in configured:
        return "trailing_filler_rendered_decimal"
    if any(item.endswith("15") and item[:-2] == value for item in configured):
        return "configured_has_trailing_filler_rendered_decimal"
    if three_digit_mnc_reordered(value) in configured:
        return "three_digit_mnc_order"
    if any(deletes_one_to_match(value, item) for item in configured):
        return "one_extra_digit"
    if any(deletes_one_to_match(item, value) for item in configured):
        return "one_missing_digit"
    return "different"


def common_prefix_length(left: str, right: str) -> int:
    count = 0
    for left_digit, right_digit in zip(left, right):
        if left_digit != right_digit:
            break
        count += 1
    return count


def common_suffix_length(left: str, right: str) -> int:
    return common_prefix_length(left[::-1], right[::-1])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True)
    parser.add_argument("--sim", required=True)
    args = parser.parse_args()

    try:
        with Path(args.sim).open("rb") as handle:
            table = tomllib.load(handle)
        configured = {
            match.group(1)
            for key in table
            if (match := SIM_KEY_RE.fullmatch(str(key))) is not None
        }
        if len(configured) != len(table) or not configured:
            raise ValueError("invalid subscriber-key shape")

        text = Path(args.log).read_text(encoding="utf-8", errors="replace")
        identities = REGISTERING_RE.findall(text)
    except Exception as error:
        print(f"QCORE_IDENTITY_AUDIT=ERROR class={type(error).__name__}")
        return 1

    print(f"QCORE_IDENTITY_CONFIG_COUNT={len(configured)}")
    print(
        "QCORE_IDENTITY_CONFIG_LENGTHS="
        + ",".join(str(length) for length in sorted({len(item) for item in configured}))
    )
    print(f"QCORE_IDENTITY_REGISTERING_COUNT={len(identities)}")
    print(f"QCORE_IDENTITY_UNIQUE_COUNT={len(set(identities))}")
    for index, identity in enumerate(identities, start=1):
        same_as_first = int(identity == identities[0])
        nearest = max(
            configured,
            key=lambda item: (
                common_prefix_length(identity, item) + common_suffix_length(identity, item),
                -abs(len(identity) - len(item)),
            ),
        )
        print(
            f"QCORE_IDENTITY_EVENT_{index}="
            f"length:{len(identity)},"
            f"length_delta:{len(identity) - len(nearest)},"
            f"matches_config:{int(identity in configured)},"
            f"same_as_first:{same_as_first},"
            f"common_prefix:{common_prefix_length(identity, nearest)},"
            f"common_suffix:{common_suffix_length(identity, nearest)},"
            f"relation:{relation(identity, configured)}"
        )
    if len(identities) >= 2:
        print(f"QCORE_IDENTITY_FIRST_LAST_EQUAL={int(identities[0] == identities[-1])}")
    print("QCORE_IDENTITY_AUDIT=PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
