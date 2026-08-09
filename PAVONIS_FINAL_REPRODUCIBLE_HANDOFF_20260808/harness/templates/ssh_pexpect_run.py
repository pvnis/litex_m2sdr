#!/usr/bin/env python3
"""Compatibility CLI for account-neutral role-based SSH execution."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from remote_common import load_site, role, run_ssh


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--site")
    parser.add_argument("--host", required=True, choices=("ran", "ue"))
    parser.add_argument("--out", required=True)
    parser.add_argument("--timeout", type=int, default=120)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("missing remote command")
    site = load_site(args.site)
    return run_ssh(site, role(site, args.host), command, Path(args.out), args.timeout)


if __name__ == "__main__":
    sys.exit(main())

