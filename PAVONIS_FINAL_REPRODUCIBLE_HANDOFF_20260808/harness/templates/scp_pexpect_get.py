#!/usr/bin/env python3
"""Compatibility CLI for account-neutral role-based SCP download."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from remote_common import load_site, role, run_scp


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--site")
    parser.add_argument("--host", required=True, choices=("ran", "ue"))
    parser.add_argument("--remote", required=True)
    parser.add_argument("--local", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--timeout", type=int, default=180)
    args = parser.parse_args()
    local = Path(args.local)
    local.parent.mkdir(parents=True, exist_ok=True)
    site = load_site(args.site)
    return run_scp(
        site, role(site, args.host), args.remote, str(local),
        Path(args.out), args.timeout, "get"
    )


if __name__ == "__main__":
    sys.exit(main())

