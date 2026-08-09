#!/usr/bin/env python3
"""Create the mode-600 private ZMQ input skeleton beside an unpacked tree."""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil


PACKAGE = Path(__file__).resolve().parents[1]
TEMPLATES = PACKAGE / "config" / "private-templates"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True, type=Path)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    if not (workspace / "SOURCE_REVISIONS").is_file():
        raise SystemExit("workspace was not created by tools/unpack_source.sh")
    target = workspace / "pavonis-local"
    if target.exists():
        raise SystemExit(f"private input directory already exists: {target}")
    target.mkdir(mode=0o700)
    files = {
        "ue-zmq.local.conf": "ue-zmq-netns.template.conf",
        "sims-zmq.local.toml": "sims-zmq.template.toml",
    }
    for destination, source in files.items():
        path = target / destination
        shutil.copyfile(TEMPLATES / source, path)
        path.chmod(0o600)
    print(f"PAVONIS_PRIVATE_SKELETON={target}")
    print("Populate both files with the same private test-subscriber role, then rerun.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
