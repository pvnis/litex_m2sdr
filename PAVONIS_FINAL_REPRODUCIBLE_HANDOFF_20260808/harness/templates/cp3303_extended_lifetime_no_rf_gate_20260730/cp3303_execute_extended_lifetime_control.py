#!/usr/bin/env python3
"""Run one isolated parallel-TCP benchmark with sticky PRACH Stage 1."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import statistics
import subprocess
import sys
import tempfile


HERE = Path(__file__).resolve().parent
ART = HERE.parent
BASE_TOOLS = ART / "cp3159_conducted_benchmark_no_rf_gate_20260728"
DEADLINE_TOOLS = ART / "cp3165_conducted_downlink_deadline_no_rf_gate_20260728"
SSH = ART / "ssh_pexpect_run.py"
SCP_PUT = ART / "scp_pexpect_put.py"
SCP_GET = ART / "scp_pexpect_get.py"
RUNNER = ART / "cp2389_run_conres_mcs2_fallback_dl_al8_packet_proof.sh"
NETNS = ART / "cp2378_prepare_ue_netns.sh"
PARALLEL_TOOLS = ART / "cp3285_parallel_tcp_no_rf_gate_20260730"
SERVER = PARALLEL_TOOLS / "cp3285_parallel_tcp_server.py"
CLIENT = PARALLEL_TOOLS / "cp3285_parallel_tcp_client.py"
ANALYZER = PARALLEL_TOOLS / "cp3285_analyze_parallel.py"

REMOTE_SERVER = "@REMOTE_HOME@/pavonis_cp3285_parallel_tcp_server.py"
REMOTE_CLIENT = "@REMOTE_HOME@/pavonis_cp3285_parallel_tcp_client.py"
REMOTE_NETNS = "@REMOTE_HOME@/pavonis_cp2378_prepare_ue_netns.sh"
REMOTE_GNB = "@REMOTE_HOME@/CLionProjects/ocudu/build-clion/apps/gnb/gnb"
REMOTE_SIM = (
    "@REMOTE_HOME@/CLionProjects/m2sdr/litex_m2sdr/software/validation/"
    "ocudu/configs/sims-pavonis-ota.private.toml"
)
PINS = {
    "runner": "ed34033fd9bc011dcd7c77bc7e670c902544a81a37a89cba5c3439bc29dbf2f4",
    "netns": "49c1cb5d129c7a985dcc29534d1c28704278517c1ea124857b6d0f17c97931c4",
    "remote_gnb": "90a5db019d52b3f893572d96a8338eae67a81b1e4169e8eb12c4afab20375f8a",
}
CORE_HOST = os.environ.get("PAVONIS_CORE_HOST_ALIAS", "")
UE_HOST = os.environ.get("PAVONIS_UE_HOST_ALIAS", "")
STAGE1_CANDIDATES = (
    "-448:-0.5:30000,80:0.5:-6500,-64:0.5:-6500,-448:-0.5:31000,"
    "-448:-0.5:29000,16:-0.5:-25000,64:-0.5:-25000,120:0:0,"
    "-224:-0.5:60000,-96:0.5:24000,-448:0.5:48000,-224:0.5:49000,"
    "-192:-0.5:20000,-192:-0.5:35000,-192:-0.5:45000,"
    "64:-0.5:5000,64:-0.5:10000,64:-0.5:15000,64:-0.5:20000"
)
BASE_UE_TX_GUARD_SAMPLES = 40000
BASE_GNB_PRACH_TIMING_COMPENSATION_SAMPLES = 237336
DEFAULT_UE_TX_GUARD_SAMPLES = 56000
DEFAULT_GNB_PRACH_TIMING_COMPENSATION_SAMPLES = 253336
NON_PRACH_EDGE_SCHEDULER_ERROR_SAMPLES = 115200


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def transport(head: list[str], tail: list[str], timeout: int) -> int:
    with tempfile.NamedTemporaryFile(prefix="pavonis_transport_") as log:
        try:
            return subprocess.run(
                [
                    *head,
                    "--out",
                    log.name,
                    "--timeout",
                    str(timeout),
                    *tail,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=timeout + 10,
                check=False,
            ).returncode
        except subprocess.TimeoutExpired:
            return 124


def ssh(host: str, timeout: int, *remote: str) -> int:
    return transport(
        [sys.executable, str(SSH), "--host", host], ["--", *remote], timeout
    )


def put(host: str, local: Path, remote: str) -> int:
    return transport(
        [
            sys.executable,
            str(SCP_PUT),
            "--host",
            host,
            "--local",
            str(local),
            "--remote",
            remote,
        ],
        [],
        120,
    )


def get(host: str, remote: str, local: Path) -> int:
    return transport(
        [
            sys.executable,
            str(SCP_GET),
            "--host",
            host,
            "--remote",
            remote,
            "--local",
            str(local),
        ],
        [],
        180,
    )


def fields(line: str) -> dict[str, str]:
    return dict(re.findall(r"([A-Za-z0-9_]+)=([^\s]+)", line))


def validate_candidate_list(value: str, maximum: int) -> int:
    candidates = value.split(",") if value else []
    if not candidates or len(candidates) > 64:
        raise SystemExit("stage-1 candidate list must contain 1..64 entries")
    for candidate in candidates:
        parts = candidate.split(":")
        if len(parts) != 3:
            raise SystemExit(f"invalid stage-1 candidate: {candidate!r}")
        int(parts[0])
        float(parts[1])
        int(parts[2])
    if maximum < len(candidates) + 1 or maximum > 65:
        raise SystemExit("stage-1 max candidates must cover list plus baseline")
    return len(candidates)


def parse_radio(stage_summary: Path) -> dict[str, object]:
    summary = json.loads(stage_summary.read_text(encoding="utf-8"))
    ue_log_value = summary.get("ue_stdout")
    gnb_log_value = summary.get("gnb_log")
    ue_log = Path(ue_log_value) if isinstance(ue_log_value, str) else None
    gnb_log = Path(gnb_log_value) if isinstance(gnb_log_value, str) else None
    cell: dict[str, str] = {}
    if ue_log is not None and ue_log.is_file():
        for line in ue_log.open(encoding="utf-8", errors="replace"):
            if "event=cell_search_done" in line:
                cell = fields(line)
                break
    pusch: list[dict[str, str]] = []
    timeout_count = 0
    late_count = 0
    if gnb_log is not None and gnb_log.is_file():
        for line in gnb_log.open(encoding="utf-8", errors="replace"):
            if "PAVONIS_PUSCH_CSI_TRACE event=result" in line:
                pusch.append(fields(line))
            if "SoapySDR TX: writeStream timeout" in line:
                timeout_count += 1
            if "downlink processor is late" in line:
                late_count += 1
    crc_ok = [row for row in pusch if row.get("crc") == "OK"]
    sinr = [float(row["sinr_db"]) for row in crc_ok if "sinr_db" in row]
    tbs = [int(row["tbs_bytes"]) for row in pusch if "tbs_bytes" in row]
    return {
        "first_failed_stage": summary.get("first_failed_stage"),
        "ue_log_available": ue_log is not None and ue_log.is_file(),
        "gnb_log_available": gnb_log is not None and gnb_log.is_file(),
        "cell_select_ok": summary.get("ue", {}).get("cell_select_ok", False),
        "cell_snr_db": float(cell["snr"]) if "snr" in cell else None,
        "cell_cfo_hz": float(cell["cfo"]) if "cfo" in cell else None,
        "pusch_rows": len(pusch),
        "pusch_crc_ok": len(crc_ok),
        "pusch_crc_yield": len(crc_ok) / len(pusch) if pusch else None,
        "pusch_crc_ok_sinr_median_db": statistics.median(sinr) if sinr else None,
        "pusch_tbs_median_bytes": statistics.median(tbs) if tbs else None,
        "soapy_tx_timeout_count": timeout_count,
        "downlink_late_count": late_count,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stamp", required=True)
    parser.add_argument("--tag", required=True)
    parser.add_argument("--result-dir", type=Path, required=True)
    parser.add_argument("--duration", type=int, default=45)
    parser.add_argument("--streams", type=int, default=4)
    parser.add_argument("--port", type=int, default=31590)
    parser.add_argument("--tx-att", type=int, default=5)
    parser.add_argument("--gnb-rx-gain", type=int, default=40)
    parser.add_argument("--max-ue-mcs", type=int, default=9)
    parser.add_argument("--stage1-candidates", default=STAGE1_CANDIDATES)
    parser.add_argument("--stage1-max-candidates", type=int, default=30)
    parser.add_argument("--gnb-path", default=REMOTE_GNB)
    parser.add_argument("--gnb-sha256", default=PINS["remote_gnb"])
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()

    try:
        ue_tx_guard_samples = int(
            os.environ.get(
                "PAVONIS_CP3219_UE_TX_GUARD_SAMPLES",
                str(DEFAULT_UE_TX_GUARD_SAMPLES),
            )
        )
        gnb_prach_timing_compensation_samples = int(
            os.environ.get(
                "PAVONIS_CP3219_GNB_PRACH_TIMING_COMPENSATION_SAMPLES",
                str(DEFAULT_GNB_PRACH_TIMING_COMPENSATION_SAMPLES),
            )
        )
    except ValueError as error:
        raise SystemExit("CP3219 timing controls must be integers") from error
    independent_prach_control = os.environ.get(
        "PAVONIS_CP3264_ALLOW_INDEPENDENT_PRACH_COMPENSATION", "0"
    )
    if independent_prach_control not in {"0", "1"}:
        raise SystemExit("CP3264 independent PRACH compensation flag must be 0 or 1")
    allow_independent_prach_compensation = independent_prach_control == "1"
    guard_delta_samples = ue_tx_guard_samples - BASE_UE_TX_GUARD_SAMPLES
    if not 0 <= guard_delta_samples <= 32000:
        raise SystemExit("CP3219 UE TX guard delta must be 0..32000 samples")
    if (
        gnb_prach_timing_compensation_samples
        != BASE_GNB_PRACH_TIMING_COMPENSATION_SAMPLES + guard_delta_samples
        and not allow_independent_prach_compensation
    ):
        raise SystemExit(
            "CP3219 gNB PRACH compensation must move by the UE TX guard delta"
        )

    if not re.fullmatch(r"[A-Za-z0-9_]+", args.stamp):
        raise SystemExit("invalid stamp")
    if not 10 <= args.duration <= 60:
        raise SystemExit("duration must be 10..60 seconds")
    if not 2 <= args.streams <= 8:
        raise SystemExit("streams must be 2..8")
    if not 0 <= args.max_ue_mcs <= 28:
        raise SystemExit("max UE MCS must be 0..28")
    candidate_count = validate_candidate_list(
        args.stage1_candidates, args.stage1_max_candidates
    )
    if (
        not args.gnb_path.startswith("@REMOTE_HOME@/")
        or ".." in args.gnb_path
        or not re.fullmatch(r"/[A-Za-z0-9_./-]+", args.gnb_path)
    ):
        raise SystemExit("invalid gNB path")
    if not re.fullmatch(r"[0-9a-f]{64}", args.gnb_sha256):
        raise SystemExit("invalid gNB SHA-256")
    if sha256(RUNNER) != PINS["runner"] or sha256(NETNS) != PINS["netns"]:
        raise SystemExit("proven runner/netns hash mismatch")
    if args.validate_only:
        print(
            "CP3159_CONDUCTED_VALIDATE_ONLY=PASS "
            f"candidate_count={candidate_count} "
            f"candidate_limit={args.stage1_max_candidates} "
            f"streams={args.streams} "
            f"max_ue_mcs={args.max_ue_mcs} "
            f"ue_tx_guard_samples={ue_tx_guard_samples} "
            "gnb_prach_timing_compensation_samples="
            f"{gnb_prach_timing_compensation_samples} "
            f"guard_delta_samples={guard_delta_samples} "
            "independent_prach_compensation="
            f"{int(allow_independent_prach_compensation)} "
            "stage1_sticky_last_accept=1 "
            "non_prach_edge_scheduler_error_samples="
            f"{NON_PRACH_EDGE_SCHEDULER_ERROR_SAMPLES}"
        )
        return 0
    if (
        os.environ.get("PAVONIS_PHASE_B_RF_APPROVED") != "1"
        and os.environ.get("PAVONIS_CP3160_RF_APPROVED") != "1"
    ):
        print("CP3159_CONDUCTED_RF_GATE=CLOSED")
        return 2
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", CORE_HOST):
        raise SystemExit("missing or invalid core host alias")
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", UE_HOST):
        raise SystemExit("missing or invalid UE host alias")
    if args.result_dir.exists():
        raise SystemExit("result directory already exists")

    args.result_dir.mkdir(mode=0o700, parents=True)
    stage: dict[str, object] = {
        "schema": 1,
        "stamp": args.stamp,
        "rf_shape": {
            "tx_attenuation_db": args.tx_att,
            "gnb_rx_gain_db": args.gnb_rx_gain,
            "max_ue_mcs": args.max_ue_mcs,
            "channel_bandwidth_mhz": 10,
            "spatial_layers": 1,
            "duplex": "FDD",
            "ue_tx_guard_samples": ue_tx_guard_samples,
            "gnb_prach_timing_compensation_samples":
                gnb_prach_timing_compensation_samples,
            "independent_prach_compensation":
                allow_independent_prach_compensation,
            "coherent_timing_delta_samples": guard_delta_samples,
            "stage1_candidate_count_excluding_baseline": candidate_count,
            "stage1_candidate_limit": args.stage1_max_candidates,
            "stage1_sticky_last_accept": True,
            "non_prach_edge_scheduler_error_samples":
                NON_PRACH_EDGE_SCHEDULER_ERROR_SAMPLES,
            "tcp_streams": args.streams,
            "stage1_candidate_list_sha256": hashlib.sha256(
                args.stage1_candidates.encode("ascii")
            ).hexdigest(),
            "gnb_sha256": args.gnb_sha256,
        },
        "steps": {},
    }
    remote_prefix = f"/tmp/pavonis_cp3159_{args.stamp}"
    core_result = f"{remote_prefix}_core.json"
    ue_result = f"{remote_prefix}_ue.json"
    ready = f"{remote_prefix}.ready"
    core_pid = f"{remote_prefix}_core.pid"
    ue_pid = f"{remote_prefix}_ue.pid"
    core_log = f"{remote_prefix}_core.log"
    ue_log = f"{remote_prefix}_ue.log"

    def step(name: str, rc: int) -> int:
        stage["steps"][name] = rc
        return rc

    def save_stage() -> None:
        (args.result_dir / "execution_summary.json").write_text(
            json.dumps(stage, indent=2) + "\n", encoding="utf-8"
        )

    try:
        port_free = "test -z \"$(sudo -n ss -H -lun 'sport = :2152')\""
        if step(
            "private_user_plane_port_free",
            ssh(CORE_HOST, 30, "bash", "-lc", port_free),
        ):
            return 1
        if step("netns_prepare", ssh(UE_HOST, 60, REMOTE_NETNS, "prepare")):
            return 1
        if step("server_deploy", put(CORE_HOST, SERVER, REMOTE_SERVER)):
            return 1
        if step("client_deploy", put(UE_HOST, CLIENT, REMOTE_CLIENT)):
            return 1
        server_hash_command = (
            f"test \"$(sha256sum {shlex.quote(REMOTE_SERVER)} | "
            f"awk '{{print $1}}')\" = {sha256(SERVER)}"
        )
        if step(
            "server_hash",
            ssh(CORE_HOST, 30, "bash", "-lc", server_hash_command),
        ):
            return 1
        client_hash_command = (
            f"test \"$(sha256sum {shlex.quote(REMOTE_CLIENT)} | "
            f"awk '{{print $1}}')\" = {sha256(CLIENT)}"
        )
        if step(
            "client_hash",
            ssh(UE_HOST, 30, "bash", "-lc", client_hash_command),
        ):
            return 1
        server_start = (
            "set -euo pipefail; "
            f"rm -f {shlex.quote(core_result)} {shlex.quote(ready)} {shlex.quote(core_log)}; "
            f"nohup python3 {shlex.quote(REMOTE_SERVER)} --port {args.port} "
            f"--duration {args.duration} --result {shlex.quote(core_result)} "
            f"--streams {args.streams} "
            f"--ready {shlex.quote(ready)} --overall-timeout 840 "
            f">{shlex.quote(core_log)} 2>&1 </dev/null & echo $! >{shlex.quote(core_pid)}"
        )
        if step("server_launch", ssh(CORE_HOST, 45, "bash", "-lc", server_start)):
            return 1
        server_ready = (
            f"for i in $(seq 1 100); do test -s {shlex.quote(ready)} "
            f"&& exit 0; sleep 0.1; done; exit 1"
        )
        if step("server_ready", ssh(CORE_HOST, 45, "bash", "-lc", server_ready)):
            return 1
        client_start = (
            "set -euo pipefail; "
            f"rm -f {shlex.quote(ue_result)} {shlex.quote(ue_log)}; "
            "nohup bash -lc "
            + shlex.quote(
                "set -e; "
                "for i in $(seq 1 3000); do "
                "sudo -n ip netns exec ue1 ip -4 -o addr show dev tun_ue1 2>/dev/null "
                "| grep -q '10.255.0.2/' && "
                f"exec sudo -n ip netns exec ue1 python3 {shlex.quote(REMOTE_CLIENT)} "
                f"--host 10.255.0.1 --port {args.port} --duration {args.duration} "
                f"--streams {args.streams} "
                f"--result {shlex.quote(ue_result)}; "
                "sleep 0.2; done; exit 42"
            )
            + f" >{shlex.quote(ue_log)} 2>&1 </dev/null & echo $! >{shlex.quote(ue_pid)}"
        )
        if step("client_arm", ssh(UE_HOST, 45, "bash", "-lc", client_start)):
            return 1

        env = os.environ.copy()
        env.update(
            {
                "STAMP": args.stamp,
                "TAG": args.tag,
                "QCORE_SIM_FILE": REMOTE_SIM,
                "PAVONIS_OCUDU_GNB_BIN_OVERRIDE": args.gnb_path,
                "PAVONIS_OCUDU_GNB_SHA_EXPECTED": args.gnb_sha256,
                "GUARD_SAMPLES": str(ue_tx_guard_samples),
                "GNB_PRACH_TIMING_COMPENSATION_SAMPLES_OVERRIDE":
                    str(gnb_prach_timing_compensation_samples),
                "PAVONIS_PRACH_TIMING_COMPENSATION_REPORT_SLOT_OFFSET": "0",
                "PAVONIS_GNB_RA_RESP_WINDOW": "40",
                "PAVONIS_PRACH_DEMOD_INPUT_STAGE1_STOP_ON_ACCEPT": "1",
                "PAVONIS_PRACH_DEMOD_INPUT_STAGE1_STICKY_LAST_ACCEPT": "1",
                "PAVONIS_PRACH_STAGE1_TA_COMPENSATE_SELECTED_SHIFT": "1",
                "PAVONIS_NON_PRACH_UL_RX_OFFSET_EDGE_SCHEDULER_ERROR_SAMPLES":
                    str(NON_PRACH_EDGE_SCHEDULER_ERROR_SAMPLES),
                "PAVONIS_PRACH_DEMOD_INPUT_STAGE1_MAX_CANDIDATES_OVERRIDE":
                    str(args.stage1_max_candidates),
                "PAVONIS_PRACH_DEMOD_INPUT_STAGE1_CANDIDATE_LIST_OVERRIDE":
                    args.stage1_candidates,
                "TX_ATT": str(args.tx_att),
                "GNB_RX_GAIN": str(args.gnb_rx_gain),
                "GNB_MAX_UE_MCS": str(args.max_ue_mcs),
            }
        )
        with tempfile.NamedTemporaryFile(prefix="pavonis_runner_") as runner_log:
            with open(runner_log.name, "wb") as output:
                try:
                    runner_rc = subprocess.run(
                        ["bash", str(RUNNER)],
                        stdout=output,
                        stderr=subprocess.STDOUT,
                        env=env,
                        timeout=900,
                        check=False,
                    ).returncode
                except subprocess.TimeoutExpired:
                    runner_rc = 124
        step("radio_runner", runner_rc)

        wait_core = (
            f"for i in $(seq 1 120); do test -s {shlex.quote(core_result)} "
            f"&& python3 -c 'import json; assert json.load(open(\"{core_result}\"))[\"complete\"]' "
            "&& exit 0; sleep 1; done; exit 1"
        )
        wait_ue = (
            f"for i in $(seq 1 120); do test -s {shlex.quote(ue_result)} "
            f"&& python3 -c 'import json; assert json.load(open(\"{ue_result}\"))[\"complete\"]' "
            "&& exit 0; sleep 1; done; exit 1"
        )
        step("core_result_ready", ssh(CORE_HOST, 75, "bash", "-lc", wait_core))
        step("ue_result_ready", ssh(UE_HOST, 75, "bash", "-lc", wait_ue))
        local_core = args.result_dir / "core_sustained.json"
        local_ue = args.result_dir / "ue_sustained.json"
        step("core_result_fetch", get(CORE_HOST, core_result, local_core))
        step("ue_result_fetch", get(UE_HOST, ue_result, local_ue))
        analysis = args.result_dir / "sustained_analysis.json"
        if local_core.is_file() and local_ue.is_file():
            step(
                "analysis",
                subprocess.run(
                    [
                        sys.executable,
                        str(ANALYZER),
                        "--core",
                        str(local_core),
                        "--ue",
                        str(local_ue),
                        "--duration",
                        str(args.duration),
                        "--streams",
                        str(args.streams),
                        "--output",
                        str(analysis),
                    ],
                    check=False,
                ).returncode,
            )
        else:
            step("analysis", 1)

        stage_summary = ART / f"cp1733_stage6_realue_sib1_rachcfg_summary_{args.stamp}.json"
        if stage_summary.is_file():
            stage["radio_health"] = parse_radio(stage_summary)
            stage["radio_summary_sha256"] = sha256(stage_summary)
        stage["result_sha256"] = {
            path.name: sha256(path)
            for path in (local_core, local_ue, analysis)
            if path.is_file()
        }
        stage["verdict"] = (
            "pass"
            if runner_rc == 0
            and analysis.is_file()
            and json.loads(analysis.read_text())["verdict"] == "pass"
            and stage.get("radio_health", {}).get("cell_select_ok") is True
            else "fail"
        )
        save_stage()
        return 0 if stage["verdict"] == "pass" else 1
    finally:
        cleanup_core = (
            "set +e; "
            f"test -s {shlex.quote(core_pid)} && kill $(cat {shlex.quote(core_pid)}) 2>/dev/null; "
            f"rm -f {shlex.quote(core_pid)} {shlex.quote(ready)}"
        )
        cleanup_ue = (
            "set +e; "
            f"test -s {shlex.quote(ue_pid)} && kill $(cat {shlex.quote(ue_pid)}) 2>/dev/null; "
            f"rm -f {shlex.quote(ue_pid)}; {shlex.quote(REMOTE_NETNS)} cleanup"
        )
        ssh(CORE_HOST, 45, "bash", "-lc", cleanup_core)
        ssh(UE_HOST, 60, "bash", "-lc", cleanup_ue)
        post_port_rc = ssh(
            CORE_HOST,
            30,
            "bash",
            "-lc",
            "test -z \"$(sudo -n ss -H -lun 'sport = :2152')\"",
        )
        stage["steps"]["private_user_plane_port_free_after"] = post_port_rc
        if post_port_rc != 0:
            stage["verdict"] = "fail"
        if "verdict" not in stage:
            stage["verdict"] = "fail"
        save_stage()


if __name__ == "__main__":
    raise SystemExit(main())
