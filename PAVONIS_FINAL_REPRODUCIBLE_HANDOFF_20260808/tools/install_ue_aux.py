#!/usr/bin/env python3
"""Install pinned handset-side observer prerequisites on the UE role."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import shlex

from remote_common import load_site, role, run_scp, run_ssh


PACKAGE = Path(__file__).resolve().parents[1]
JARS = {
    "pavonis_fplmn_repair.cp2661.jar":
        "cc6c2c8ea415b504a2cdbfa9620eeb656b25fcbe9091663972ffcfbba77aa21e",
    "pavonis_sim_power_cycle.jar":
        "6ee9fee93c89a183bd385802824404747280b473fcf1b72402fa7eacca599da2",
}
PACKAGES = (
    "crcmod==1.7",
    "pycrate==0.8.1",
    "pyserial==3.5",
    "pyusb==1.3.1",
    "qcsuper==2.1.0.post4",
)


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--site", required=True)
    parser.add_argument("--log-dir", required=True, type=Path)
    args = parser.parse_args()
    site = load_site(args.site)
    target = role(site, "ue")
    args.log_dir.mkdir(parents=True, exist_ok=True)
    for name, expected in JARS.items():
        source = PACKAGE / "runtime" / "ue-aux" / name
        if digest(source) != expected:
            raise SystemExit(f"packaged UE helper hash mismatch: {name}")
        temporary = f"/tmp/{name}.{expected[:12]}"
        rc = run_scp(
            site, target, str(source), temporary,
            args.log_dir / f"put-{name}.log", 60, "put",
        )
        if rc:
            return rc
        install = (
            "set -euo pipefail; "
            f"test \"$(sha256sum {shlex.quote(temporary)} | cut -d ' ' -f1)\" = {expected}; "
            f"install -m 600 {shlex.quote(temporary)} {shlex.quote(target.home + '/' + name)}; "
            f"rm -f {shlex.quote(temporary)}"
        )
        rc = run_ssh(
            site, target, ["bash", "-lc", install],
            args.log_dir / f"install-{name}.log", 60,
        )
        if rc:
            return rc
    venv = f"{target.home}/pavonis_qcsuper_cp2741"
    package_args = " ".join(shlex.quote(item) for item in PACKAGES)
    setup = (
        "set -euo pipefail; "
        f"if test ! -x {shlex.quote(venv + '/bin/python3')}; then "
        f"python3 -m venv {shlex.quote(venv)}; fi; "
        f"{shlex.quote(venv + '/bin/python3')} -m pip install --disable-pip-version-check "
        f"{package_args}; "
        f"test \"$({shlex.quote(venv + '/bin/python3')} -c "
        "'import importlib.metadata; print(importlib.metadata.version(\"qcsuper\"))')\" "
        "= 2.1.0.post4; echo PAVONIS_UE_AUX_INSTALL=PASS"
    )
    rc = run_ssh(
        site, target, ["bash", "-lc", setup], args.log_dir / "qcsuper.log", 600
    )
    if rc == 0:
        print("PAVONIS_UE_AUX_INSTALL=PASS")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
