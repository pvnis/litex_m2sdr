#!/usr/bin/env python3
"""Deploy a rendered, hash-recorded harness to both role hosts without RF."""

from __future__ import annotations

import argparse
import hashlib
import io
from pathlib import Path
import tarfile
import tempfile

from remote_common import load_site, role, run_scp, run_ssh


PACKAGE = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def payload_files(harness: Path, role_name: str | None = None) -> dict[str, Path]:
    result: dict[str, Path] = {}
    for path in sorted(harness.rglob("*")):
        if not path.is_file() or (
            path.suffix not in {".sh", ".py"}
            and not path.name.endswith("_launcher")
        ):
            continue
        archive_name = f"scripts/{path.name}"
        if archive_name in result:
            raise ValueError(f"duplicate deploy basename: {path.name}")
        result[archive_name] = path
    config_root = harness / "runtime-config" / "srsenb"
    for path in sorted(config_root.glob("*.conf")):
        result[f"runtime-config/srsenb/{path.name}"] = path
    if role_name in {None, "ue"}:
        for path in sorted((PACKAGE / "runtime" / "ue-aux").glob("*.jar")):
            result[f"runtime/ue-aux/{path.name}"] = path
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--site", required=True)
    parser.add_argument("--harness", required=True, type=Path)
    parser.add_argument("--log-dir", required=True, type=Path)
    args = parser.parse_args()
    site = load_site(args.site)
    harness = args.harness.resolve()
    expected = harness / "RENDERED_SHA256SUMS"
    if not expected.is_file():
        raise SystemExit("rendered harness manifest is missing")
    args.log_dir.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="pavonis-deploy.") as temporary:
        for role_name in ("ran", "ue"):
            files = payload_files(harness, role_name)
            archive = Path(temporary) / f"harness-{role_name}.tar.gz"
            manifest = "".join(
                f"{sha256(path)}  {name}\n" for name, path in sorted(files.items())
            ).encode("ascii")
            with tarfile.open(archive, "w:gz", format=tarfile.PAX_FORMAT) as tar:
                for name, path in sorted(files.items()):
                    info = tar.gettarinfo(str(path), arcname=name)
                    info.uid = info.gid = 0
                    info.uname = info.gname = "root"
                    info.mtime = 0
                    with path.open("rb") as source:
                        tar.addfile(info, source)
                info = tarfile.TarInfo("PAVONIS_HARNESS_SHA256SUMS")
                info.size = len(manifest)
                info.mode = 0o600
                info.mtime = 0
                tar.addfile(info, io.BytesIO(manifest))
            archive_hash = sha256(archive)
            target = role(site, role_name)
            remote_archive = f"/tmp/pavonis-harness-{archive_hash[:16]}.tar.gz"
            remote_root = f"{target.home}/.pavonis-harness-{archive_hash[:16]}"
            put_rc = run_scp(
                site, target, str(archive), remote_archive,
                args.log_dir / f"{role_name}-put.log", 120, "put"
            )
            if put_rc:
                return put_rc
            aux_install = (
                f"install -m 600 runtime/ue-aux/*.jar {target.home}/; "
                if role_name == "ue" else ""
            )
            command = [
                "bash", "-lc",
                "set -euo pipefail; "
                f"test \"$(sha256sum {remote_archive} | cut -d ' ' -f1)\" = {archive_hash}; "
                f"test ! -e {remote_root}; mkdir -m 700 {remote_root}; "
                f"tar -xzf {remote_archive} -C {remote_root}; "
                f"cd {remote_root}; sha256sum -c PAVONIS_HARNESS_SHA256SUMS >/dev/null; "
                f"install -m 700 scripts/*.sh scripts/*.py scripts/*_launcher {target.home}/; "
                f"mkdir -p {target.home}/pavonis_cp2924_srsenb_m2sdr/config; "
                f"install -m 600 runtime-config/srsenb/*.conf {target.home}/pavonis_cp2924_srsenb_m2sdr/config/; "
                f"{aux_install}"
                f"rm -f {remote_archive}; echo PAVONIS_HARNESS_DEPLOY=PASS",
            ]
            rc = run_ssh(
                site, target, command,
                args.log_dir / f"{role_name}-verify.log", 120
            )
            if rc:
                return rc
    print(
        "PAVONIS_HARNESS_DEPLOY=PASS "
        f"ran_files={len(payload_files(harness, 'ran'))} "
        f"ue_files={len(payload_files(harness, 'ue'))}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
