#!/usr/bin/env python3
"""No-network, no-RF end-to-end test of the portable controller layer."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from unittest import mock

from deploy_harness import payload_files
import install_ue_aux
from materialize_harness import render
from prepare_private import main as prepare_private_main
import remote_common
import run_phone_gate


PACKAGE = Path(__file__).resolve().parents[1]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    with tempfile.TemporaryDirectory(prefix="pavonis-selftest.") as temporary:
        root = Path(temporary)
        site_path = root / "site.toml"
        site_path.write_text(
            """[transport]
known_hosts = "/dev/null"
strict_host_key_checking = "yes"
connect_timeout_seconds = 8
password_file = ""

[roles.ran]
host = "ran.example.invalid"
user = "pavonis"
home = "/home/pavonis"
identity_file = ""
port = 22

[roles.ue]
host = "ue.example.invalid"
user = "pavonis"
home = "/home/pavonis"
identity_file = ""
port = 22

[network]
core_n2_ip = "192.0.2.10"
primer_s1_ip = "192.0.2.12"
ran_interface = "lab0"
external_interface = "uplink0"

[runtime]
[runtime.pin_overrides]
""",
            encoding="utf-8",
        )
        site_path.chmod(0o600)
        output = root / "harness"
        render(str(site_path), output)

        build_tool = (PACKAGE / "tools" / "build_from_source.sh").read_text()
        assert "-DENABLE_SOAPYSDR=ON" in build_tool
        assert "--target srsue srsenb srsran_rf_soapy" in build_tool
        assert build_tool.count("-DENABLE_SOAPY=ON") == 1

        for line in (output / "RENDERED_SHA256SUMS").read_text().splitlines():
            expected, relative = line.split(maxsplit=1)
            assert digest(output / relative) == expected
        for line in (output / "portable_baseline.sha256").read_text().splitlines():
            expected, relative = line.split(maxsplit=1)
            assert digest(output / relative) == expected

        env = os.environ.copy()
        env.update({
            "ART": str(output),
            "STAMP": "SELFTEST_FRESH",
            "PAVONIS_SITE_CONFIG": str(site_path),
        })
        slow = subprocess.run(
            [str(output / "cp3121h_run_metric_al8_phone_rf.sh")],
            env=env, capture_output=True, text=True, check=False,
        )
        assert slow.returncode == 2, slow.stderr
        fast = subprocess.run(
            [str(output / "cp3612_phone_15mhz_all_mcs17_no_rf_gate_20260802" /
                 "cp3612h_execute_phone_15mhz_all_mcs17_attempt.sh")],
            env=env, capture_output=True, text=True, check=False,
        )
        assert fast.returncode == 2, fast.stderr

        bounded_runner = (
            output / "cp3422_lead11_mcs19_robust_no_rf_gate_20260731" /
            "cp3423_run_lead11_mcs19_preconnected.sh"
        )
        bounded_env = env.copy()
        bounded_env["CP3422_VALIDATE_ONLY"] = "1"
        bounded_validate = subprocess.run(
            [str(bounded_runner)], env=bounded_env,
            capture_output=True, text=True, check=False,
        )
        assert bounded_validate.returncode == 0, bounded_validate.stderr
        assert "CP3159_CONDUCTED_VALIDATE_ONLY=PASS" in bounded_validate.stdout
        bounded_closed = subprocess.run(
            [str(bounded_runner)], env=env,
            capture_output=True, text=True, check=False,
        )
        assert bounded_closed.returncode == 2, bounded_closed.stderr
        assert "CP3423_RF_GATE=CLOSED" in bounded_closed.stdout

        files = payload_files(output)
        assert len(files) >= 99
        assert sum(name.endswith(".conf") for name in files) == 4
        assert {
            name for name in files if name.startswith("runtime/ue-aux/")
        } == {
            "runtime/ue-aux/pavonis_fplmn_repair.cp2661.jar",
            "runtime/ue-aux/pavonis_sim_power_cycle.jar",
        }
        assert {
            "scripts/pavonis_cp3422_lead11_deadline_inside_guard_mcs19_launcher",
            "scripts/pavonis_cp3397_pusch_mcs19_logwarning_config_launcher",
        }.issubset(files)

        with (
            mock.patch("sys.argv", [
                "install_ue_aux.py", "--site", str(site_path),
                "--log-dir", str(root / "ue-aux-install"),
            ]),
            mock.patch.object(install_ue_aux, "run_scp", return_value=0) as put,
            mock.patch.object(install_ue_aux, "run_ssh", return_value=0) as remote,
        ):
            assert install_ue_aux.main() == 0
            assert put.call_count == 2
            assert remote.call_count == 3
            assert "qcsuper==2.1.0.post4" in remote.call_args.args[2][2]

        source_workspace = root / "source-workspace"
        source_workspace.mkdir()
        (source_workspace / "SOURCE_REVISIONS").write_text("selftest\n")
        with mock.patch("sys.argv", ["prepare_private.py", "--workspace", str(source_workspace)]):
            assert prepare_private_main() == 0
        local = source_workspace / "pavonis-local"
        assert (local / "ue-zmq.local.conf").stat().st_mode & 0o777 == 0o600
        assert (local / "sims-zmq.local.toml").stat().st_mode & 0o777 == 0o600

        build_paths = (
            "litex_m2sdr/.keep",
            "ocudu/build-pavonis/apps/gnb/gnb",
            "qcore/target/release/qcore",
            "srsRAN_4G/build-pavonis/srsenb/src/srsenb",
            "srsRAN_4G/build-pavonis/srsue/src/srsue",
            "srsRAN_4G/build-pavonis/lib/src/phy/rf/libsrsran_rf_soapy.so",
        )
        for relative in build_paths:
            path = source_workspace / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f"selftest {relative}\n", encoding="ascii")
            path.chmod(0o700)
        layout = PACKAGE / "tools" / "install_runtime_layout.sh"
        for role_name in ("ran", "ue"):
            home = root / f"{role_name}-home"
            home.mkdir()
            layout_env = os.environ.copy()
            layout_env["HOME"] = str(home)
            for _ in range(2):
                installed = subprocess.run(
                    [str(layout), role_name, str(source_workspace)],
                    env=layout_env, capture_output=True, text=True, check=False,
                )
                assert installed.returncode == 0, installed.stderr
                assert f"PAVONIS_RUNTIME_LAYOUT=PASS role={role_name}" in installed.stdout
            manifest = home / f"PAVONIS_RUNTIME_LAYOUT_{role_name}_SHA256SUMS"
            assert manifest.stat().st_mode & 0o777 == 0o600
            checked = subprocess.run(
                ["sha256sum", "-c", str(manifest)],
                capture_output=True, text=True, check=False,
            )
            assert checked.returncode == 0, checked.stderr

        stage_inputs = []
        for name in ("zmq", "bounded", "phone"):
            path = root / f"{name}.json"
            path.write_text('{"result":"PASS"}\n', encoding="utf-8")
            stage_inputs.append(path)
        staged = root / "staged.json"
        receipt = subprocess.run(
            [
                sys.executable, str(PACKAGE / "tools" / "write_staged_summary.py"),
                "--stamp", "SELFTEST", "--profile", "fast",
                "--zmq", str(stage_inputs[0]),
                "--bounded", str(stage_inputs[1]),
                "--phone", str(stage_inputs[2]),
                "--output", str(staged),
            ],
            capture_output=True, text=True, check=False,
        )
        assert receipt.returncode == 0, receipt.stderr
        assert '"result": "PASS"' in staged.read_text(encoding="utf-8")

        fake_harness = root / "fake-phone-harness"
        fake_harness.mkdir()
        fake_runner = fake_harness / "fake_runner.sh"
        fake_runner.write_text(
            """#!/usr/bin/env bash
set -euo pipefail
echo 'HOLDING BEFORE TEARDOWN'
release="$ART/cp3612h_${STAMP}_RELEASE"
for _ in $(seq 1 100); do test -e "$release" && break; sleep 0.05; done
test -e "$release"
attempt="$PAVONIS_CP3612_RESULT_ROOT/attempt_${STAMP}"
mkdir -p "$attempt"
printf '{"verdict":"pass"}\\n' >"$attempt/sustained_analysis.json"
""",
            encoding="utf-8",
        )
        fake_runner.chmod(0o700)
        fake_ssh = fake_harness / "ssh_pexpect_run.py"
        fake_ssh.write_text(
            """#!/usr/bin/env python3
import pathlib, sys
out = pathlib.Path(sys.argv[sys.argv.index('--out') + 1])
out.write_text('PAVONIS_PHONE_PUBLIC_EGRESS=PASS\\n')
""",
            encoding="utf-8",
        )
        fake_ssh.chmod(0o700)
        rendered = fake_harness / "RENDERED_SHA256SUMS"
        rendered.write_text(
            f"{digest(fake_runner)}  fake_runner.sh\n"
            f"{digest(fake_ssh)}  ssh_pexpect_run.py\n",
            encoding="ascii",
        )
        fake_output = root / "fake-phone-output"
        with (
            mock.patch.object(run_phone_gate, "FAST", Path("fake_runner.sh")),
            mock.patch.dict(os.environ, {"PAVONIS_RF_APPROVED": "1"}),
            mock.patch("sys.argv", [
                "run_phone_gate.py", "fast", "--site", str(site_path),
                "--harness", str(fake_harness), "--output-dir", str(fake_output),
                "--stamp", "SELFTEST_PHONE", "--public-proof",
            ]),
        ):
            assert run_phone_gate.main() == 0
        phone_receipt = (fake_output / "summary.json").read_text(encoding="utf-8")
        assert '"public_egress_proven": true' in phone_receipt

        site = remote_common.load_site(str(site_path))
        with mock.patch.object(remote_common, "_run", return_value=0) as runner:
            rc = remote_common.run_ssh(
                site, remote_common.role(site, "ran"), ["true"],
                root / "ssh.log", 10,
            )
            assert rc == 0
            argv = runner.call_args.args[0]
            assert argv[0] == "ssh"
            assert argv[-2] == "pavonis@ran.example.invalid"
            assert argv[-1] == "true"
        with mock.patch.object(remote_common, "_run", return_value=0) as runner:
            rc = remote_common.run_scp(
                site, remote_common.role(site, "ue"), "source", "destination",
                root / "scp.log", 10, "put",
            )
            assert rc == 0
            argv = runner.call_args.args[0]
            assert argv[0] == "scp"
            assert argv[-1] == "pavonis@ue.example.invalid:destination"

    print("PAVONIS_PORTABLE_CONTROLLER_SELFTEST=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
