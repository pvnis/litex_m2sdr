#!/usr/bin/env python3
"""Create a site-config pin override fragment from rebuilt runtime files."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import tomllib


PACKAGE = Path(__file__).resolve().parents[1]


def digest(path: Path) -> str:
    if not path.is_file():
        raise ValueError(f"not a file: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--qcore", type=Path, required=True)
    parser.add_argument("--gnb", type=Path, required=True)
    parser.add_argument("--srsenb", type=Path, required=True)
    parser.add_argument("--rf-soapy", type=Path, required=True)
    parser.add_argument("--soapy-module", type=Path, required=True)
    parser.add_argument("--soapy-source", type=Path, required=True)
    parser.add_argument("--m2sdr-module", type=Path, required=True)
    parser.add_argument("--enb-conf", type=Path, required=True)
    parser.add_argument("--rr-conf", type=Path, required=True)
    parser.add_argument("--sib-conf", type=Path, required=True)
    parser.add_argument("--rb-conf", type=Path, required=True)
    args = parser.parse_args()
    with (PACKAGE / "config" / "runtime-pin-map.toml").open("rb") as handle:
        historical = tomllib.load(handle)["artifacts"]
    replacements = {
        historical["qcore"]: digest(args.qcore),
        historical["gnb_stage"]: digest(args.gnb),
        historical["gnb_slow"]: digest(args.gnb),
        historical["gnb_fast"]: digest(args.gnb),
        historical["srsenb"]: digest(args.srsenb),
        historical["rf_soapy"]: digest(args.rf_soapy),
        historical["soapy_module_recorded"]: digest(args.soapy_module),
        historical["soapy_module_legacy_default"]: digest(args.soapy_module),
        historical["soapy_source_recorded"]: digest(args.soapy_source),
        historical["soapy_source_legacy_default"]: digest(args.soapy_source),
        historical["m2sdr_kernel_module"]: digest(args.m2sdr_module),
        historical["srsenb_enb_conf"]: digest(args.enb_conf),
        historical["srsenb_rr_conf"]: digest(args.rr_conf),
        historical["srsenb_sib_conf"]: digest(args.sib_conf),
        historical["srsenb_rb_conf"]: digest(args.rb_conf),
    }
    print("[runtime.pin_overrides]")
    for old, new in replacements.items():
        print(f'"{old}" = "{new}"')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

