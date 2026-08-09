#!/usr/bin/env python3
"""Run the staged Pavonis reproducibility campaign with resumable manifests."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import tarfile
import time
from typing import Any


ART = Path(__file__).resolve().parent
SSH = ART / "ssh_pexpect_run.py"
SCP_PUT = ART / "scp_pexpect_put.py"
SCP_GET = ART / "scp_pexpect_get.py"
COLD_LAB = ART / "pavonis_poc_cold_lab.sh"
ZMQ_UE = ART / "cp2317_zmq_stage8_control_20260714/ue-zmq-cp2317.conf"
BOUNDED_RUNNER = ART / "cp2389_run_conres_mcs2_fallback_dl_al8_packet_proof.sh"
PACKET_PROBE = ART / "cp2380_prearmed_netlink_userplane_probe.sh"
NETNS_PREPARE = ART / "cp2378_prepare_ue_netns.sh"
PHONE_RUNNER = ART / "cp3121_execute_metric_al8_phone_rf.sh"
PHONE_ANALYZER = ART / "cp3121_analyze_phone_metric_run.py"
PHONE_SEED = ART / "cp3121d_metric_run_summary.json"

REMOTE_VALIDATION = Path(
    "@REMOTE_HOME@/CLionProjects/m2sdr/litex_m2sdr/software/validation/ocudu"
)
REMOTE_ZMQ_RUNNER = REMOTE_VALIDATION / "scripts/run_zmq_ocudu_dataplane.sh"
REMOTE_ZMQ_SIM = REMOTE_VALIDATION / "configs/sims-ocudu-zmq.example.toml"
REMOTE_OTA_SIM = REMOTE_VALIDATION / "configs/sims-pavonis-ota.private.toml"
REMOTE_ZMQ_GNB = REMOTE_VALIDATION / "configs/gnb_zmq_tdd_n78_20mhz.yml"
REMOTE_BOUNDED_GNB = Path(
    "@REMOTE_HOME@/CLionProjects/ocudu/build-clion/apps/gnb/gnb"
)
REMOTE_PACKET_PROBE = Path("@REMOTE_HOME@/pavonis_cp2380_prearmed_netlink_userplane_probe.sh")
REMOTE_NETNS_PREPARE = Path("@REMOTE_HOME@/pavonis_cp2378_prepare_ue_netns.sh")

BOUNDED_STAGE1_CANDIDATE_LIST = (
    "-448:-0.5:30000,"
    "80:0.5:-6500,-64:0.5:-6500,-448:-0.5:31000,-448:-0.5:29000,"
    "16:-0.5:-25000,64:-0.5:-25000,120:0:0,-224:-0.5:60000,"
    "-96:0.5:24000,-448:0.5:48000,-224:0.5:49000,"
    "-192:-0.5:20000,-192:-0.5:35000,-192:-0.5:45000,"
    "64:-0.5:5000,64:-0.5:10000,64:-0.5:15000,64:-0.5:20000"
)

PINS = {
    "cold_lab": "17fcb94a7c8e46a860a95dccd1fd2d6514a6301f1eb5bfb8899ca82dec2ee01d",
    "zmq_ue": "c766a533eb08e913d48fc15bc5c62d29620736f0f5037a1cf161d11e355b67f8",
    "bounded_runner": "ed34033fd9bc011dcd7c77bc7e670c902544a81a37a89cba5c3439bc29dbf2f4",
    "packet_probe": "5d03170657b30a3f1aedac6969b457f1f72714ab33b430bf2a41fabdb99190e7",
    "netns_prepare": "49c1cb5d129c7a985dcc29534d1c28704278517c1ea124857b6d0f17c97931c4",
    "phone_runner": "b94d1d9566ad2b71600e9575565f00c6072606f5eb9a0c3e66e16e4336c0af7f",
    "phone_analyzer": "6090eb5d2a976bc8d6cfc1b990117506e94e562c68e116238eaed6a6fc1a2ef7",
    "phone_seed": "59074450f938ea4d90f53a228e5fc211da934d2a44a49d587252cf52a9d857cb",
    "remote_zmq_runner": "2d23a4659b7b31b7e3017b905fb58f71991b1d75db97a0f69b924faf54057fdd",
    "remote_zmq_sim": "701378afdb271318c6100c4229929cd17a573d9b309df48ba1cfd271ae4df56f",
    "remote_ota_sim": "17ff95bcf0417e3816a7b9775349b3fe15ac83f3fa7f58ebc4949448f879dcdb",
    "remote_zmq_gnb": "4dc477f4d678aa2db7a47ea66fa8e0aec382a189fecffe336335b917a82f44e7",
    "remote_bounded_gnb": "90a5db019d52b3f893572d96a8338eae67a81b1e4169e8eb12c4afab20375f8a",
    "phone_fresh": "8ef3fe48c1bfa90bfb34c1989315b3a0b9ef908fb4cdc75f552f78c96ec0870c",
    "phone_sim": "f37becd7da794647cd889178818325a01173f1021350de6ea41d578871b6de0d",
    "phone_policy": "ac24833190ae12277ec9600b1fa5ab9bcc9e04689d6fd56872ab952d21ef86d2",
    "phone_echo_helper": "425f08cedae5ab7557313b04408b6f305b4b00cde54fb4e688a63768477638a6",
    "phone_http_helper": "ebded899eacd4f67b9b787ee0fb0a1ddc3188229c3ba464f669e88518f662f2b",
    "core_http_helper": "3c286e009408af22d65aa8b3755a6e44083068038c7dcdb1b0a1785cabaebf53",
}

LOCAL_PIN_PATHS = {
    "cold_lab": COLD_LAB,
    "zmq_ue": ZMQ_UE,
    "bounded_runner": BOUNDED_RUNNER,
    "packet_probe": PACKET_PROBE,
    "netns_prepare": NETNS_PREPARE,
    "phone_runner": PHONE_RUNNER,
    "phone_analyzer": PHONE_ANALYZER,
    "phone_seed": PHONE_SEED,
}


def utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    os.chmod(temporary, 0o600)
    os.replace(temporary, path)


def run_logged(
    command: list[str],
    log: Path,
    *,
    env: dict[str, str] | None = None,
    timeout: int | None = None,
) -> int:
    log.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    with log.open("wb") as output:
        try:
            completed = subprocess.run(
                command,
                stdout=output,
                stderr=subprocess.STDOUT,
                env=env,
                timeout=timeout,
                check=False,
            )
            return completed.returncode
        except subprocess.TimeoutExpired:
            output.write(b"\nPAVONIS_CAMPAIGN_LOCAL_TIMEOUT=1\n")
            return 124


def ssh(host: str, output: Path, timeout: int, *remote: str) -> int:
    return subprocess.run(
        [
            sys.executable,
            str(SSH),
            "--host",
            host,
            "--out",
            str(output),
            "--timeout",
            str(timeout),
            "--",
            *remote,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    ).returncode


def scp_put(host: str, local: Path, remote: str, output: Path, timeout: int = 120) -> int:
    return subprocess.run(
        [
            sys.executable,
            str(SCP_PUT),
            "--host",
            host,
            "--local",
            str(local),
            "--remote",
            remote,
            "--out",
            str(output),
            "--timeout",
            str(timeout),
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    ).returncode


def scp_get(host: str, remote: str, local: Path, output: Path, timeout: int = 180) -> int:
    return subprocess.run(
        [
            sys.executable,
            str(SCP_GET),
            "--host",
            host,
            "--remote",
            remote,
            "--local",
            str(local),
            "--out",
            str(output),
            "--timeout",
            str(timeout),
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    ).returncode


def artifact_hashes(paths: dict[str, Path]) -> dict[str, dict[str, Any]]:
    return {
        name: {
            "file": path.name,
            "sha256": sha256(path),
            "bytes": path.stat().st_size,
        }
        for name, path in paths.items()
        if path.is_file()
    }


def parse_bounded_gnb(path: Path) -> dict[str, int]:
    metrics = {
        "pusch_results": 0,
        "pusch_crc_ok": 0,
        "pusch_crc_ko": 0,
        "msg3_crc_ok_tbs11": 0,
    }
    if not path.is_file():
        return metrics
    field_pattern = re.compile(r"([A-Za-z0-9_]+)=([^\s]+)")
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if "PAVONIS_PUSCH_CSI_TRACE event=result" not in line:
                continue
            row = dict(field_pattern.findall(line))
            metrics["pusch_results"] += 1
            if row.get("crc") == "OK":
                metrics["pusch_crc_ok"] += 1
                if row.get("tbs_bytes") == "11":
                    metrics["msg3_crc_ok_tbs11"] += 1
            else:
                metrics["pusch_crc_ko"] += 1
    return metrics


def load_state(root: Path) -> dict[str, Any]:
    return json.loads((root / "campaign_state.json").read_text(encoding="utf-8"))


def save_state(root: Path, state: dict[str, Any]) -> None:
    state["updated_utc"] = datetime.now(timezone.utc).isoformat()
    write_json(root / "campaign_state.json", state)


def stage_counts(state: dict[str, Any], stage: str) -> tuple[int, int]:
    attempts = [item for item in state["attempts"] if item["stage"] == stage]
    valid = sum(item["classification"] == "valid_success" for item in attempts)
    return len(attempts), valid


def record_attempt(
    root: Path,
    state: dict[str, Any],
    summary: dict[str, Any],
    summary_path: Path,
) -> None:
    write_json(summary_path, summary)
    state["attempts"].append(
        {
            "stage": summary["stage"],
            "index": summary["index"],
            "stamp": summary["stamp"],
            "classification": summary["classification"],
            "summary": str(summary_path.relative_to(root)),
            "summary_sha256": sha256(summary_path),
            "seed": bool(summary.get("seed", False)),
        }
    )
    save_state(root, state)


def recover_orphan_attempts(root: Path, state: dict[str, Any]) -> None:
    recorded = {
        (item["stage"], int(item["index"])) for item in state.get("attempts", [])
    }
    changed = False
    for stage in ["zmq", "bounded", "phone"]:
        stage_dir = root / stage
        if not stage_dir.is_dir():
            continue
        for attempt_dir in sorted(stage_dir.glob("raw_*")):
            match = re.match(r"raw_([0-9]+)_(.+)$", attempt_dir.name)
            if not match:
                continue
            index = int(match.group(1))
            if (stage, index) in recorded:
                continue
            summary_path = attempt_dir / "attempt_summary.json"
            if summary_path.is_file():
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
            else:
                summary = {
                    "checkpoint": 3122,
                    "stage": stage,
                    "index": index,
                    "stamp": match.group(2),
                    "classification": "interrupted_void",
                    "pass": False,
                    "recovered_after_controller_interruption": True,
                    "artifacts": artifact_hashes(
                        {
                            path.name: path
                            for path in attempt_dir.iterdir()
                            if path.is_file()
                        }
                    ),
                }
                write_json(summary_path, summary)
            state["attempts"].append(
                {
                    "stage": stage,
                    "index": index,
                    "stamp": summary["stamp"],
                    "classification": summary["classification"],
                    "summary": str(summary_path.relative_to(root)),
                    "summary_sha256": sha256(summary_path),
                    "seed": bool(summary.get("seed", False)),
                }
            )
            recorded.add((stage, index))
            changed = True
    if changed:
        state["attempts"].sort(key=lambda item: (item["stage"], int(item["index"])))
        save_state(root, state)


def ensure_local_pins() -> None:
    for name, path in LOCAL_PIN_PATHS.items():
        if not path.is_file() or sha256(path) != PINS[name]:
            raise RuntimeError(f"local pin mismatch: {name}")


def host_postcheck(attempt_dir: Path) -> bool:
    for check_index in range(2):
        suffix = "" if check_index == 0 else "_retry"
        nuc_rc = ssh(
            "ran",
            attempt_dir / f"ran_postcheck{suffix}.log",
            60,
            "bash",
            "-lc",
            (
                "set -euo pipefail; "
                'test "$(ps -C gnb -C qcore -C srsenb -C srsue --no-headers | wc -l)" -eq 0; '
                'test "$(cat /sys/module/m2sdr/parameters/dma_reader_program_mode)" = N; '
                "echo PAVONIS_CAMPAIGN_NUC4_POSTCHECK=PASS"
            ),
        )
        sens_rc = ssh(
            "ue",
            attempt_dir / f"sens_postcheck{suffix}.log",
            60,
            "bash",
            "-lc",
            (
                "set -euo pipefail; "
                'test "$(ps -C gnb -C qcore -C srsenb -C srsue --no-headers | wc -l)" -eq 0; '
                'test -z "$(pgrep -f \'[/]qcsuper\' || true)"; '
                "test ! -e /run/netns/ue1; "
                'test "$(adb get-state)" = device; '
                "echo PAVONIS_CAMPAIGN_SENS_POSTCHECK=PASS"
            ),
        )
        if nuc_rc == 0 and sens_rc == 0:
            return True
        if check_index == 0:
            time.sleep(2)
    return False


def do_probe(root: Path, state: dict[str, Any]) -> None:
    ensure_local_pins()
    probe_dir = root / "probe" / utc_stamp()
    probe_dir.mkdir(mode=0o700, parents=True)

    cold_output = probe_dir / "cold_lab"
    cold_env = os.environ.copy()
    cold_env.update(
        {
            "PAVONIS_POC_BASE_STAMP": f"{state['campaign']}_CP3122_PROBE",
            "PAVONIS_POC_OUTPUT_DIR": str(cold_output),
        }
    )
    cold_rc = run_logged(
        [str(COLD_LAB), "probe"],
        probe_dir / "cold_lab_console.log",
        env=cold_env,
        timeout=300,
    )

    sequence_env = os.environ.copy()
    sequence_env.update(
        {
            "STAMP": f"{state['campaign']}_CP3122_SEQUENCE",
            "TAG": "cp3122_campaign_sequence_probe",
            "PAVONIS_CP3121_SEQUENCE_PROBE": "1",
        }
    )
    sequence_rc = run_logged(
        [str(PHONE_RUNNER)],
        probe_dir / "phone_sequence_probe.log",
        env=sequence_env,
        timeout=90,
    )

    packet_put_rc = scp_put(
        "ue",
        PACKET_PROBE,
        str(REMOTE_PACKET_PROBE),
        probe_dir / "packet_probe_put.log",
    )
    netns_put_rc = scp_put(
        "ue",
        NETNS_PREPARE,
        str(REMOTE_NETNS_PREPARE),
        probe_dir / "netns_prepare_put.log",
    )
    nuc_rc = ssh(
        "ran",
        probe_dir / "ran_campaign_pin_gate.log",
        60,
        "bash",
        "-lc",
        (
            "set -euo pipefail; "
            f"sha256sum {shlex.quote(str(REMOTE_ZMQ_RUNNER))} | grep -q '^{PINS['remote_zmq_runner']}  '; "
            f"sha256sum {shlex.quote(str(REMOTE_ZMQ_GNB))} | grep -q '^{PINS['remote_zmq_gnb']}  '; "
            f"sha256sum {shlex.quote(str(REMOTE_ZMQ_SIM))} | grep -q '^{PINS['remote_zmq_sim']}  '; "
            f"sha256sum {shlex.quote(str(REMOTE_OTA_SIM))} | grep -q '^{PINS['remote_ota_sim']}  '; "
            f"sha256sum {shlex.quote(str(REMOTE_BOUNDED_GNB))} | grep -q '^{PINS['remote_bounded_gnb']}  '; "
            f"grep -a -q 'PAVONIS_PUSCH_CSI_TRACE' {shlex.quote(str(REMOTE_BOUNDED_GNB))}; "
            f"sha256sum @REMOTE_HOME@/pavonis_cp3121_core_http_metrics_remote.py | grep -q '^{PINS['core_http_helper']}  '; "
            'test "$(cat /sys/module/m2sdr/parameters/dma_reader_program_mode)" = N; '
            "echo PAVONIS_CP3122_NUC4_PIN_GATE=PASS"
        ),
    )
    sens_rc = ssh(
        "ue",
        probe_dir / "sens_campaign_pin_gate.log",
        60,
        "bash",
        "-lc",
        (
            "set -euo pipefail; "
            f"sha256sum {shlex.quote(str(REMOTE_PACKET_PROBE))} | grep -q '^{PINS['packet_probe']}  '; "
            f"sha256sum {shlex.quote(str(REMOTE_NETNS_PREPARE))} | grep -q '^{PINS['netns_prepare']}  '; "
            f"{shlex.quote(str(REMOTE_NETNS_PREPARE))} --self-test | grep -q '^SELF_TEST=PASS$'; "
            f"sha256sum @REMOTE_HOME@/pavonis_cp2969_fresh_phone_epoch_remote.sh | grep -q '^{PINS['phone_fresh']}  '; "
            f"sha256sum @REMOTE_HOME@/cp3086_phone_sim_power_cycle_remote.sh | grep -q '^{PINS['phone_sim']}  '; "
            f"sha256sum @REMOTE_HOME@/cp3087_phone_policy_audit_remote.sh | grep -q '^{PINS['phone_policy']}  '; "
            f"sha256sum @REMOTE_HOME@/pavonis_cp3121_phone_data_metrics_remote.sh | grep -q '^{PINS['phone_echo_helper']}  '; "
            f"sha256sum @REMOTE_HOME@/pavonis_cp3121_phone_http_metrics_remote.sh | grep -q '^{PINS['phone_http_helper']}  '; "
            'test "$(adb get-state)" = device; '
            "echo PAVONIS_CP3122_SENS_PIN_GATE=PASS"
        ),
    )

    passed = all(
        rc == 0
        for rc in [
            cold_rc,
            sequence_rc,
            packet_put_rc,
            netns_put_rc,
            nuc_rc,
            sens_rc,
        ]
    )
    proof = {
        "checkpoint": 3122,
        "campaign": state["campaign"],
        "probe_utc": datetime.now(timezone.utc).isoformat(),
        "pass": passed,
        "bounded_fixture": {
            "prach_timing_compensation_samples": 237336,
            "prach_report_slot_offset": 0,
            "ra_response_window_slots": 40,
            "stage1_stop_on_accept": True,
            "stage1_ta_compensation": True,
            "stage1_max_candidates": 30,
            "stage1_candidate_list": BOUNDED_STAGE1_CANDIDATE_LIST,
        },
        "return_codes": {
            "cold_lab": cold_rc,
            "phone_sequence": sequence_rc,
            "packet_deploy": packet_put_rc,
            "netns_prepare_deploy": netns_put_rc,
            "ran_pin_gate": nuc_rc,
            "sens_pin_gate": sens_rc,
        },
        "pins": PINS,
        "artifacts": artifact_hashes(
            {
                "cold_console": probe_dir / "cold_lab_console.log",
                "phone_sequence": probe_dir / "phone_sequence_probe.log",
                "netns_prepare_deploy": probe_dir / "netns_prepare_put.log",
                "ran_gate": probe_dir / "ran_campaign_pin_gate.log",
                "sens_gate": probe_dir / "sens_campaign_pin_gate.log",
            }
        ),
    }
    write_json(probe_dir / "probe_summary.json", proof)
    if not passed:
        raise RuntimeError("campaign probe failed")
    state["probe_pass"] = True
    state["probe_summary"] = str((probe_dir / "probe_summary.json").relative_to(root))
    state["probe_summary_sha256"] = sha256(probe_dir / "probe_summary.json")
    save_state(root, state)
    print("CP3122_CAMPAIGN_PROBE=PASS", flush=True)


def next_index(state: dict[str, Any], stage: str) -> int:
    indices = [int(item["index"]) for item in state["attempts"] if item["stage"] == stage]
    return max(indices, default=0) + 1


def run_zmq_attempt(root: Path, state: dict[str, Any]) -> dict[str, Any]:
    stage = "zmq"
    index = next_index(state, stage)
    stamp = f"{utc_stamp()}CP3122_ZMQ_{index:02d}"
    attempt_dir = root / stage / f"raw_{index:03d}_{stamp}"
    attempt_dir.mkdir(mode=0o700, parents=True)
    remote_ue = f"/tmp/pavonis_cp3122_{state['campaign']}_{index:03d}.ue.private.conf"
    remote_tar = f"/tmp/pavonis_cp3122_{state['campaign']}_{index:03d}_zmq.tar.gz"
    local_tar = attempt_dir / "zmq_run.tar.gz"

    put_rc = scp_put("ran", ZMQ_UE, remote_ue, attempt_dir / "put_private.log")
    runner_rc = 1
    direct_rc = 1
    package_rc = 1
    get_rc = 1
    run_dir = ""
    try:
        if put_rc == 0:
            remote_script = (
                "set -euo pipefail; "
                f"chmod 600 {shlex.quote(remote_ue)}; "
                f"sha256sum {shlex.quote(remote_ue)} | grep -q '^{PINS['zmq_ue']}  '; "
                "export QCORE_BIN=@REMOTE_HOME@/CLionProjects/qcore/target/debug/qcore; "
                "export OCUDU_GNB_BIN=@REMOTE_HOME@/pavonis_cp3109_scheduler_conres_rebase_gnb/gnb; "
                "export SRSUE_BIN=@REMOTE_HOME@/CLionProjects/srsRAN_4G/build-clion/srsue/src/srsue; "
                f"{shlex.quote(str(REMOTE_ZMQ_RUNNER))} "
                f"--ue-conf {shlex.quote(remote_ue)} --sim-file {shlex.quote(str(REMOTE_ZMQ_SIM))} "
                f"--gnb-conf {shlex.quote(str(REMOTE_ZMQ_GNB))} --external-iface @RAN_INTERFACE@ "
                f"--ping-target 10.255.0.1 --attach-timeout 120 --allowed-loss 0 "
                f"--tag {shlex.quote(stamp)}"
            )
            runner_rc = ssh(
                "ran",
                attempt_dir / "runner.log",
                300,
                "bash",
                "-lc",
                remote_script,
            )
            text = (attempt_dir / "runner.log").read_text(
                encoding="utf-8", errors="replace"
            )
            matches = re.findall(r"^Run directory: (.+)$", text, flags=re.MULTILINE)
            if matches and matches[-1].startswith(str(REMOTE_VALIDATION / "runs") + "/"):
                run_dir = matches[-1].strip()
                gate_script = (
                    "set -euo pipefail; "
                    f"d={shlex.quote(run_dir)}; "
                    'grep -q "^VALIDATION_PING_OK=1 " "$d/dataplane.txt"; '
                    'grep -q "Random Access Complete" "$d/srsue.log"; '
                    'grep -q "RRC Connected" "$d/srsue.log"; '
                    "echo PAVONIS_CP3122_ZMQ_DIRECT_GATE=PASS"
                )
                direct_rc = ssh(
                    "ran",
                    attempt_dir / "direct_gate.log",
                    60,
                    "bash",
                    "-lc",
                    gate_script,
                )
                package_script = (
                    "set -euo pipefail; "
                    f"d={shlex.quote(run_dir)}; out={shlex.quote(remote_tar)}; "
                    'tar -C "$(dirname "$d")" -czf "$out" "$(basename "$d")"; '
                    'sha256sum "$out"'
                )
                package_rc = ssh(
                    "ran",
                    attempt_dir / "package.log",
                    120,
                    "bash",
                    "-lc",
                    package_script,
                )
                if package_rc == 0:
                    get_rc = scp_get(
                        "ran",
                        remote_tar,
                        local_tar,
                        attempt_dir / "get_archive.log",
                    )
    finally:
        ssh(
            "ran",
            attempt_dir / "remote_cleanup.log",
            45,
            "bash",
            "-lc",
            f"rm -f {shlex.quote(remote_ue)} {shlex.quote(remote_tar)}",
        )

    postcheck = host_postcheck(attempt_dir)
    passed = direct_rc == 0 and package_rc == 0 and get_rc == 0 and postcheck
    summary = {
        "checkpoint": 3122,
        "stage": stage,
        "index": index,
        "stamp": stamp,
        "classification": "valid_success" if passed else "valid_failure",
        "pass": passed,
        "runner_rc": runner_rc,
        "direct_gate_rc": direct_rc,
        "package_rc": package_rc,
        "archive_get_rc": get_rc,
        "postcheck": postcheck,
        "runner_pipefail_tolerated_only_by_direct_gate": runner_rc != 0 and direct_rc == 0,
        "artifacts": artifact_hashes(
            {
                "runner_log": attempt_dir / "runner.log",
                "direct_gate": attempt_dir / "direct_gate.log",
                "archive": local_tar,
            }
        ),
    }
    record_attempt(root, state, summary, attempt_dir / "attempt_summary.json")
    print(
        f"CP3122_ATTEMPT stage=zmq raw={index} class={summary['classification']} "
        f"runner_rc={runner_rc}",
        flush=True,
    )
    return summary


def run_bounded_attempt(root: Path, state: dict[str, Any]) -> dict[str, Any]:
    stage = "bounded"
    index = next_index(state, stage)
    stamp = f"{utc_stamp()}CP3122_BOUNDED_{index:02d}"
    tag = f"cp3122_{state['campaign']}_bounded_{index:02d}"
    attempt_dir = root / stage / f"raw_{index:03d}_{stamp}"
    attempt_dir.mkdir(mode=0o700, parents=True)
    remote_log = f"/tmp/pavonis_cp3122_{state['campaign']}_{index:03d}.packet.log"
    remote_pid = f"/tmp/pavonis_cp3122_{state['campaign']}_{index:03d}.packet.pid"
    packet_local = attempt_dir / "packet_probe.log"

    netns_prepare_rc = ssh(
        "ue",
        attempt_dir / "netns_prepare.log",
        60,
        str(REMOTE_NETNS_PREPARE),
        "prepare",
    )
    arm_script = (
        "set -euo pipefail; "
        f"sha256sum {shlex.quote(str(REMOTE_PACKET_PROBE))} | grep -q '^{PINS['packet_probe']}  '; "
        f"test ! -e {shlex.quote(remote_pid)}; "
        f"nohup {shlex.quote(str(REMOTE_PACKET_PROBE))} >{shlex.quote(remote_log)} 2>&1 </dev/null & "
        f"echo $! >{shlex.quote(remote_pid)}; "
        "echo PAVONIS_CP3122_BOUNDED_PROBE_ARM=PASS"
    )
    arm_rc = 1
    if netns_prepare_rc == 0:
        arm_rc = ssh(
            "ue",
            attempt_dir / "probe_arm.log",
            60,
            "bash",
            "-lc",
            arm_script,
        )
    env = os.environ.copy()
    env.update(
        {
            "STAMP": stamp,
            "TAG": tag,
            "QCORE_SIM_FILE": str(REMOTE_OTA_SIM),
            "PAVONIS_OCUDU_GNB_BIN_OVERRIDE": str(REMOTE_BOUNDED_GNB),
            "PAVONIS_OCUDU_GNB_SHA_EXPECTED": PINS["remote_bounded_gnb"],
            "PAVONIS_PRACH_TIMING_COMPENSATION_SAMPLES": "237336",
            "PAVONIS_PRACH_TIMING_COMPENSATION_REPORT_SLOT_OFFSET": "0",
            "PAVONIS_GNB_RA_RESP_WINDOW": "40",
            "PAVONIS_PRACH_DEMOD_INPUT_STAGE1_STOP_ON_ACCEPT": "1",
            "PAVONIS_PRACH_STAGE1_TA_COMPENSATE_SELECTED_SHIFT": "1",
            "PAVONIS_PRACH_DEMOD_INPUT_STAGE1_MAX_CANDIDATES_OVERRIDE": "30",
            "PAVONIS_PRACH_DEMOD_INPUT_STAGE1_CANDIDATE_LIST_OVERRIDE":
                BOUNDED_STAGE1_CANDIDATE_LIST,
        }
    )
    runner_rc = 1
    if arm_rc == 0:
        runner_rc = run_logged(
            ["bash", str(BOUNDED_RUNNER)],
            attempt_dir / "runner.log",
            env=env,
            timeout=600,
        )

    packet_get_rc = scp_get(
        "ue",
        remote_log,
        packet_local,
        attempt_dir / "get_packet.log",
        timeout=90,
    )
    cleanup_script = (
        "set +e; "
        f"p={shlex.quote(remote_pid)}; "
        'if [[ -s "$p" ]]; then pid=$(cat "$p"); [[ "$pid" =~ ^[0-9]+$ ]] && kill "$pid" 2>/dev/null; fi; '
        f"rm -f {shlex.quote(remote_pid)} {shlex.quote(remote_log)}"
    )
    ssh(
        "ue",
        attempt_dir / "probe_cleanup.log",
        45,
        "bash",
        "-lc",
        cleanup_script,
    )
    netns_cleanup_rc = ssh(
        "ue",
        attempt_dir / "netns_cleanup.log",
        60,
        str(REMOTE_NETNS_PREPARE),
        "cleanup",
    )

    summary_path = ART / f"cp1733_stage6_realue_sib1_rachcfg_summary_{stamp}.json"
    nuc_tar = ART / f"{tag}_ran_{stamp}.tar.gz"
    sens_tar = ART / f"{tag}_ue_{stamp}.tar.gz"
    packet_pass = False
    if packet_local.is_file():
        packet_pass = "DATA_PROBE=PASS" in packet_local.read_text(
            encoding="utf-8", errors="replace"
        )
    summary_gate = False
    bounded_metrics: dict[str, Any] = {}
    mirror_log = (
        ART
        / f"cp1733_ran_artifacts_{stamp}"
        / "tmp"
        / "gnb_m2sdr_band3_ota.log"
    )
    gnb_log = mirror_log
    source: dict[str, Any] = {}
    if summary_path.is_file():
        source = json.loads(summary_path.read_text(encoding="utf-8"))
        packaged_gnb_log = source.get("gnb_log")
        if isinstance(packaged_gnb_log, str) and Path(packaged_gnb_log).is_file():
            gnb_log = Path(packaged_gnb_log)
    bounded_gnb_metrics = parse_bounded_gnb(gnb_log)
    if source:
        bounded_metrics = {
            "first_failed_stage": source.get("first_failed_stage"),
            "soapy_tx_timeout": source.get("gnb", {}).get("soapy_tx_timeout_lines"),
            "downlink_late": source.get("gnb", {}).get("downlink_late_lines"),
            "cell_select_ok": source.get("ue", {}).get("cell_select_ok"),
            "rar_accept": source.get("ue", {}).get("ra_response_accept_count"),
            "msg3_get": source.get("ue", {}).get("msg3_get_count"),
        }
        summary_gate = (
            bounded_metrics["soapy_tx_timeout"] == 0
            and bounded_metrics["downlink_late"] == 0
            and bounded_metrics["cell_select_ok"] is True
            and (bounded_metrics["rar_accept"] or 0) >= 1
            and (bounded_metrics["msg3_get"] or 0) >= 1
            and bounded_gnb_metrics["msg3_crc_ok_tbs11"] >= 1
        )
    postcheck = host_postcheck(attempt_dir)
    passed = (
        netns_prepare_rc == 0
        and arm_rc == 0
        and runner_rc == 0
        and packet_get_rc == 0
        and packet_pass
        and summary_gate
        and nuc_tar.is_file()
        and sens_tar.is_file()
        and netns_cleanup_rc == 0
        and postcheck
    )
    classification = "valid_success" if passed else "valid_failure"
    if netns_prepare_rc != 0 or arm_rc != 0:
        classification = "runner_launch_void"
    summary = {
        "checkpoint": 3122,
        "stage": stage,
        "index": index,
        "stamp": stamp,
        "classification": classification,
        "pass": passed,
        "netns_prepare_rc": netns_prepare_rc,
        "arm_rc": arm_rc,
        "runner_rc": runner_rc,
        "packet_get_rc": packet_get_rc,
        "packet_pass": packet_pass,
        "summary_gate": summary_gate,
        "netns_cleanup_rc": netns_cleanup_rc,
        "postcheck": postcheck,
        "metrics": bounded_metrics,
        "gnb_metrics": bounded_gnb_metrics,
        "artifacts": artifact_hashes(
            {
                "runner_log": attempt_dir / "runner.log",
                "packet_probe": packet_local,
                "stage_summary": summary_path,
                "gnb_log": gnb_log,
                "nuc_archive": nuc_tar,
                "sens_archive": sens_tar,
            }
        ),
    }
    record_attempt(root, state, summary, attempt_dir / "attempt_summary.json")

    for duplicate in [
        ART / f"cp1733_ran_artifacts_{stamp}",
        ART / f"cp1733_ue_artifacts_{stamp}",
    ]:
        if duplicate.is_dir() and nuc_tar.is_file() and sens_tar.is_file():
            shutil.rmtree(duplicate)

    print(
        f"CP3122_ATTEMPT stage=bounded raw={index} class={summary['classification']} "
        f"packet={int(packet_pass)}",
        flush=True,
    )
    return summary


def phone_reset(attempt_dir: Path, stamp: str) -> tuple[bool, dict[str, int]]:
    fresh_script = (
        "set -euo pipefail; "
        'serial=$(adb get-serialno | tr -d "\\r"); '
        'sub=$(adb shell settings get global multi_sim_data_call | tr -d "\\r"); '
        f"@REMOTE_HOME@/pavonis_cp2969_fresh_phone_epoch_remote.sh "
        f"{shlex.quote(stamp + '_FRESH')} \"$serial\" \"$sub\""
    )
    fresh_rc = ssh(
        "ue",
        attempt_dir / "phone_fresh_epoch.log",
        600,
        "bash",
        "-lc",
        fresh_script,
    )
    sim_rc = 1
    audit_rc = 1
    if fresh_rc == 0:
        sim_rc = ssh(
            "ue",
            attempt_dir / "phone_sim_cycle.log",
            180,
            "env",
            "PAVONIS_CP3086_SIM_POWER_CYCLE_APPROVED=1",
            "@REMOTE_HOME@/cp3086_phone_sim_power_cycle_remote.sh",
            "cycle",
            stamp + "_SIM",
        )
    if sim_rc == 0:
        time.sleep(15)
        audit_rc = ssh(
            "ue",
            attempt_dir / "phone_policy_audit.log",
            180,
            "@REMOTE_HOME@/cp3087_phone_policy_audit_remote.sh",
            stamp + "_POLICY",
        )
        if audit_rc != 0:
            time.sleep(15)
            audit_rc = ssh(
                "ue",
                attempt_dir / "phone_policy_audit_retry.log",
                180,
                "@REMOTE_HOME@/cp3087_phone_policy_audit_remote.sh",
                stamp + "_POLICY_RETRY",
            )
    return fresh_rc == 0 and sim_rc == 0 and audit_rc == 0, {
        "fresh_rc": fresh_rc,
        "sim_rc": sim_rc,
        "audit_rc": audit_rc,
    }


def extract_gnb_snapshot(archive: Path, output: Path) -> None:
    with tarfile.open(archive, "r:gz") as bundle:
        names = [
            member
            for member in bundle.getmembers()
            if member.isfile() and member.name.endswith("/gnb.snapshot.log")
        ]
        if len(names) != 1:
            raise RuntimeError("expected exactly one gNB snapshot")
        source = bundle.extractfile(names[0])
        if source is None:
            raise RuntimeError("cannot read gNB snapshot")
        with output.open("wb") as destination:
            shutil.copyfileobj(source, destination)


def run_phone_attempt(root: Path, state: dict[str, Any]) -> dict[str, Any]:
    stage = "phone"
    index = next_index(state, stage)
    stamp = f"{utc_stamp()}CP3122_PHONE_{index:02d}"
    attempt_dir = root / stage / f"raw_{index:03d}_{stamp}"
    attempt_dir.mkdir(mode=0o700, parents=True)

    reset_ok, reset_rcs = phone_reset(attempt_dir, stamp)
    runner_rc = 1
    analysis_rc = 1
    analysis_path = attempt_dir / "metric_summary.json"
    if reset_ok:
        env = os.environ.copy()
        env.update({"STAMP": stamp, "TAG": f"cp3122_{state['campaign']}_phone_{index:02d}"})
        runner_rc = run_logged(
            [str(PHONE_RUNNER)],
            attempt_dir / "runner.log",
            env=env,
            timeout=900,
        )

    nuc_tar = ART / f"cp2893_{stamp}_ran.tar.gz"
    sens_tar = ART / f"cp2893_{stamp}_ue.tar.gz"
    capture_tar = ART / f"cp3082_{stamp}_msg3_target_capture.tar.gz"
    gnb_log = attempt_dir / "gnb.snapshot.log"
    attach = ART / f"cp2893_{stamp}_attach_metrics.log"
    core_http = ART / f"cp2893_{stamp}_core_http_server.log"
    phone_http = ART / f"cp2893_{stamp}_phone_http_data.log"
    phone_echo = ART / f"cp2893_{stamp}_phone_ocudu_data.log"
    run_summary = ART / f"cp2893_{stamp}_summary.txt"
    required = [
        nuc_tar,
        sens_tar,
        capture_tar,
        attach,
        core_http,
        phone_http,
        phone_echo,
        run_summary,
    ]
    if all(path.is_file() for path in required):
        try:
            extract_gnb_snapshot(nuc_tar, gnb_log)
            analysis_rc = run_logged(
                [
                    sys.executable,
                    str(PHONE_ANALYZER),
                    "--stamp",
                    stamp,
                    "--gnb-log",
                    str(gnb_log),
                    "--attach",
                    str(attach),
                    "--core-http",
                    str(core_http),
                    "--phone-http",
                    str(phone_http),
                    "--phone-echo",
                    str(phone_echo),
                    "--run-summary",
                    str(run_summary),
                    "--nuc-archive",
                    str(nuc_tar),
                    "--sens-archive",
                    str(sens_tar),
                    "--capture-archive",
                    str(capture_tar),
                    "--output",
                    str(analysis_path),
                    "--require-pass",
                ],
                attempt_dir / "analyzer.log",
                timeout=120,
            )
        except (RuntimeError, tarfile.TarError):
            analysis_rc = 1

    postcheck = host_postcheck(attempt_dir)
    analysis: dict[str, Any] = {}
    if analysis_path.is_file():
        analysis = json.loads(analysis_path.read_text(encoding="utf-8"))
    passed = (
        reset_ok
        and runner_rc == 0
        and analysis_rc == 0
        and analysis.get("pass") is True
        and postcheck
    )
    service_log = ART / f"cp2893_{stamp}_srs_service_wait.log"
    ocudu_start = ART / f"cp2893_{stamp}_ocudu_start.log"
    if passed:
        classification = "valid_success"
    elif not reset_ok:
        classification = "phone_lifecycle_void"
    elif (
        service_log.is_file()
        and "CP2857_SRS_SERVICE=TIMEOUT"
        in service_log.read_text(encoding="utf-8", errors="replace")
        and not ocudu_start.is_file()
    ):
        classification = "primer_supply_void"
    elif analysis:
        classification = "valid_failure"
    else:
        classification = "runner_failure"

    compact_metrics: dict[str, Any] = {}
    if analysis:
        compact_metrics = {
            "pass_checks": analysis.get("pass_checks"),
            "data_gates": analysis.get("data_gates"),
            "attach": analysis.get("attach"),
            "echo": analysis.get("echo"),
            "http": analysis.get("http"),
            "runtime": analysis.get("gnb", {}).get("runtime"),
            "pusch": {
                key: analysis.get("gnb", {}).get("pusch", {}).get(key)
                for key in ["results", "crc_ok", "crc_ko", "crc_yield", "crc_ok_tbs11"]
            },
            "msg4": analysis.get("gnb", {}).get("msg4"),
            "health": analysis.get("gnb", {}).get("health"),
        }
    summary = {
        "checkpoint": 3122,
        "stage": stage,
        "index": index,
        "stamp": stamp,
        "classification": classification,
        "pass": passed,
        "reset": {"pass": reset_ok, **reset_rcs},
        "runner_rc": runner_rc,
        "analysis_rc": analysis_rc,
        "postcheck": postcheck,
        "metrics": compact_metrics,
        "artifacts": artifact_hashes(
            {
                "runner_log": attempt_dir / "runner.log",
                "analysis": analysis_path,
                "gnb_log": gnb_log,
                "run_summary": run_summary,
                "nuc_archive": nuc_tar,
                "sens_archive": sens_tar,
                "capture_archive": capture_tar,
            }
        ),
    }
    record_attempt(root, state, summary, attempt_dir / "attempt_summary.json")
    print(
        f"CP3122_ATTEMPT stage=phone raw={index} class={classification} "
        f"runner_rc={runner_rc}",
        flush=True,
    )
    return summary


def add_phone_seed(root: Path, state: dict[str, Any]) -> None:
    if any(item.get("seed") for item in state["attempts"] if item["stage"] == "phone"):
        return
    seed_dir = root / "phone" / "seed_cp3121d"
    seed_dir.mkdir(mode=0o700, parents=True, exist_ok=True)
    copied = seed_dir / "metric_summary.json"
    shutil.copy2(PHONE_SEED, copied)
    if sha256(copied) != PINS["phone_seed"]:
        raise RuntimeError("phone seed hash mismatch")
    source = json.loads(copied.read_text(encoding="utf-8"))
    summary = {
        "checkpoint": 3122,
        "stage": "phone",
        "index": 1,
        "stamp": source["stamp"],
        "classification": "valid_success",
        "pass": True,
        "seed": True,
        "seed_checkpoint": 3121,
        "staged_after_hardware_gate": False,
        "metrics": {
            "pass_checks": source["pass_checks"],
            "data_gates": source["data_gates"],
            "attach": source["attach"],
            "echo": source["echo"],
            "http": source["http"],
            "runtime": source["gnb"]["runtime"],
            "pusch": {
                key: source["gnb"]["pusch"][key]
                for key in ["results", "crc_ok", "crc_ko", "crc_yield", "crc_ok_tbs11"]
            },
            "msg4": source["gnb"]["msg4"],
            "health": source["gnb"]["health"],
        },
        "artifacts": artifact_hashes({"analysis": copied}),
    }
    record_attempt(root, state, summary, seed_dir / "attempt_summary.json")
    print("CP3122_PHONE_SEED=ADDED checkpoint=3121", flush=True)


def run_stage(
    root: Path,
    state: dict[str, Any],
    stage: str,
    runner: Any,
) -> None:
    target = int(state["targets"][stage])
    cap = int(state["raw_caps"][stage])
    while True:
        raw, valid = stage_counts(state, stage)
        if valid >= target:
            print(
                f"CP3122_STAGE=PASS stage={stage} raw={raw} valid={valid} target={target}",
                flush=True,
            )
            return
        if raw >= cap:
            raise RuntimeError(
                f"raw cap reached stage={stage} raw={raw} valid={valid} target={target}"
            )
        result = runner(root, state)
        attempts = [item for item in state["attempts"] if item["stage"] == stage]
        if (
            result["classification"].endswith("_void")
            and len(attempts) >= 2
            and attempts[-2]["classification"] == result["classification"]
        ):
            raise RuntimeError(
                f"two consecutive voids stage={stage} class={result['classification']}"
            )


def run_one_stage(
    root: Path,
    state: dict[str, Any],
    stage: str,
    runner: Any,
) -> None:
    target = int(state["targets"][stage])
    cap = int(state["raw_caps"][stage])
    raw, valid = stage_counts(state, stage)
    if valid >= target:
        print(
            f"CP3122_STAGE=PASS stage={stage} raw={raw} valid={valid} target={target}",
            flush=True,
        )
        return
    if raw >= cap:
        raise RuntimeError(
            f"raw cap reached stage={stage} raw={raw} valid={valid} target={target}"
        )
    result = runner(root, state)
    attempts = [item for item in state["attempts"] if item["stage"] == stage]
    if (
        result["classification"].endswith("_void")
        and len(attempts) >= 2
        and attempts[-2]["classification"] == result["classification"]
    ):
        raise RuntimeError(
            f"two consecutive voids stage={stage} class={result['classification']}"
        )


def print_status(state: dict[str, Any]) -> None:
    result = {
        "campaign": state["campaign"],
        "probe_pass": state["probe_pass"],
        "stages": {},
    }
    for stage in ["zmq", "bounded", "phone"]:
        raw, valid = stage_counts(state, stage)
        result["stages"][stage] = {
            "raw": raw,
            "valid": valid,
            "target": state["targets"][stage],
            "raw_cap": state["raw_caps"][stage],
        }
    print(json.dumps(result, sort_keys=True))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=["probe", "run", "run-one", "status"])
    parser.add_argument("--campaign", required=True)
    parser.add_argument("--root", type=Path)
    parser.add_argument("--zmq-valid", type=int, default=5)
    parser.add_argument("--bounded-valid", type=int, default=5)
    parser.add_argument("--phone-valid", type=int, default=20)
    parser.add_argument("--zmq-raw-cap", type=int, default=8)
    parser.add_argument("--bounded-raw-cap", type=int, default=8)
    parser.add_argument("--phone-raw-cap", type=int, default=40)
    args = parser.parse_args()

    if not re.fullmatch(r"[A-Za-z0-9_.-]+", args.campaign):
        raise SystemExit("invalid campaign identifier")
    targets = {
        "zmq": args.zmq_valid,
        "bounded": args.bounded_valid,
        "phone": args.phone_valid,
    }
    raw_caps = {
        "zmq": args.zmq_raw_cap,
        "bounded": args.bounded_raw_cap,
        "phone": args.phone_raw_cap,
    }
    if any(value < 2 for value in targets.values()):
        raise SystemExit("all valid targets must be at least 2")
    if any(raw_caps[key] < targets[key] for key in targets):
        raise SystemExit("raw caps must cover valid targets")

    root = args.root or (ART / f"cp3122_campaign_{args.campaign}")
    state_path = root / "campaign_state.json"
    if state_path.is_file():
        state = load_state(root)
        if (
            state["campaign"] != args.campaign
            or state["targets"] != targets
            or state["raw_caps"] != raw_caps
        ):
            raise SystemExit("campaign state/argument mismatch")
    else:
        if args.action == "status":
            raise SystemExit("campaign does not exist")
        root.mkdir(mode=0o700, parents=True, exist_ok=False)
        state = {
            "schema": 1,
            "checkpoint": 3122,
            "campaign": args.campaign,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "updated_utc": datetime.now(timezone.utc).isoformat(),
            "targets": targets,
            "raw_caps": raw_caps,
            "probe_pass": False,
            "probe_summary": None,
            "attempts": [],
        }
        save_state(root, state)

    recover_orphan_attempts(root, state)
    if args.action == "status":
        print_status(state)
        return 0
    if args.action == "probe":
        do_probe(root, state)
        print_status(state)
        return 0

    if os.environ.get("PAVONIS_CP3122_RUN_APPROVED") != "1":
        raise SystemExit("run gate closed: set PAVONIS_CP3122_RUN_APPROVED=1")
    if os.environ.get("PAVONIS_CP3122_RF_APPROVED") != "1":
        raise SystemExit("RF gate closed: set PAVONIS_CP3122_RF_APPROVED=1")
    ensure_local_pins()
    if not state["probe_pass"]:
        raise SystemExit("campaign probe has not passed")

    if args.action == "run-one":
        if stage_counts(state, "zmq")[1] < state["targets"]["zmq"]:
            run_one_stage(root, state, "zmq", run_zmq_attempt)
        elif stage_counts(state, "bounded")[1] < state["targets"]["bounded"]:
            run_one_stage(root, state, "bounded", run_bounded_attempt)
        else:
            add_phone_seed(root, state)
            if stage_counts(state, "phone")[1] < state["targets"]["phone"]:
                run_one_stage(root, state, "phone", run_phone_attempt)
            else:
                print("CP3122_ALL_STAGES_ALREADY_COMPLETE=1", flush=True)
        print_status(state)
        return 0

    run_stage(root, state, "zmq", run_zmq_attempt)
    run_stage(root, state, "bounded", run_bounded_attempt)
    add_phone_seed(root, state)
    run_stage(root, state, "phone", run_phone_attempt)
    state["complete"] = True
    save_state(root, state)
    print_status(state)
    print("CP3122_REPRODUCIBILITY_CAMPAIGN=PASS", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as error:
        print(f"CP3122_CAMPAIGN_STOP reason={error}", file=sys.stderr, flush=True)
        raise SystemExit(1)
