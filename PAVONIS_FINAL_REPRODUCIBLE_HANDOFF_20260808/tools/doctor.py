#!/usr/bin/env python3
"""No-RF package, transport, and host prerequisite check."""

from __future__ import annotations

import argparse
from pathlib import Path

from remote_common import load_site, role, run_ssh


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--site", required=True)
    parser.add_argument("--log-dir", required=True, type=Path)
    parser.add_argument("--local-only", action="store_true")
    args = parser.parse_args()
    site = load_site(args.site)
    args.log_dir.mkdir(parents=True, exist_ok=True)
    if args.local_only:
        print("PAVONIS_DOCTOR=PASS scope=local")
        return 0

    for role_name in ("ran", "ue"):
        target = role(site, role_name)
        required = "bash python3 sha256sum tar timeout"
        if role_name == "ue":
            required += " adb"
        script = (
            "set -euo pipefail; "
            "for x in " + required + "; do command -v \"$x\" >/dev/null; done; "
            f"test \"$HOME\" = {target.home}; "
            "test -w \"$HOME\"; "
            "printf 'PAVONIS_ROLE_DOCTOR=PASS role=%s\\n' " + role_name
        )
        rc = run_ssh(
            site, target, ["bash", "-lc", script],
            args.log_dir / f"{role_name}.log", 30
        )
        if rc:
            return rc
    ran = role(site, "ran")
    ran_runtime = (
        "set -euo pipefail; "
        f"h={ran.home}; "
        "for p in "
        "\"$h/pavonis_cp2989_qcore_unknown_update_fallback/qcore\" "
        "\"$h/pavonis_cp3025_stage1_energy_capture_gnb/gnb\" "
        "\"$h/pavonis_cp3109_scheduler_conres_rebase_gnb/gnb\" "
        "\"$h/pavonis_cp3597_msg4_geometry_trace_gnb/gnb\" "
        "\"$h/pavonis_cp2985_srsenb_pdu_session_dl_harq_retx/srsenb\" "
        "\"$h/pavonis_cp2656_srsenb/srsenb\" "
        "\"$h/CLionProjects/ocudu/build-clion/apps/gnb/gnb\"; "
        "do test -x \"$p\"; done; "
        "test \"$(stat -c %a \"$h/PAVONIS_RUNTIME_LAYOUT_ran_SHA256SUMS\")\" = 600; "
        "sha256sum -c \"$h/PAVONIS_RUNTIME_LAYOUT_ran_SHA256SUMS\" >/dev/null; "
        "test -f \"$h/pavonis_cp2939_rf_plugin/libsrsran_rf_soapy.so\"; "
        "test -f \"$h/CLionProjects/m2sdr/litex_m2sdr/software/soapysdr/LiteXM2SDRStreaming.cpp\"; "
        "sim=\"$h/CLionProjects/m2sdr/litex_m2sdr/software/validation/ocudu/configs/"
        "sims-pavonis-ota.private.toml\"; "
        "test -f \"$sim\"; test \"$(stat -c %a \"$sim\")\" = 600; "
        "test -r /dev/m2sdr0 -a -w /dev/m2sdr0; "
        "command -v SoapySDRUtil >/dev/null; SoapySDRUtil --info >/dev/null; "
        "test \"$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort -u)\" = performance; "
        "chrt --fifo 10 true; echo PAVONIS_RAN_RUNTIME_DOCTOR=PASS"
    )
    rc = run_ssh(
        site, ran, ["bash", "-lc", ran_runtime],
        args.log_dir / "ran-runtime.log", 60,
    )
    if rc:
        return rc
    ue = role(site, "ue")
    ue_aux = (
        "set -euo pipefail; "
        f"h={ue.home}; "
        "test \"$(stat -c %a \"$h/PAVONIS_RUNTIME_LAYOUT_ue_SHA256SUMS\")\" = 600; "
        "sha256sum -c \"$h/PAVONIS_RUNTIME_LAYOUT_ue_SHA256SUMS\" >/dev/null; "
        "test -x \"$h/CLionProjects/srsRAN_4G/build-clion/srsue/src/srsue\"; "
        "test -x \"$h/CLionProjects/srsRAN_4G/build-ue-bladerf-wno-gcc13-rrc/"
        "srsue/src/srsue\"; "
        "command -v bladeRF-cli >/dev/null; bladeRF-cli -p >/dev/null 2>&1; "
        "test \"$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort -u)\" = performance; "
        f"q={ue.home}/pavonis_qcsuper_cp2741; "
        "test -x \"$q/bin/python3\"; "
        "test \"$(\"$q/bin/python3\" -c 'import importlib.metadata; "
        "print(importlib.metadata.version(\"qcsuper\"))')\" = 2.1.0.post4; "
        f"test \"$(sha256sum {ue.home}/pavonis_fplmn_repair.cp2661.jar | cut -d ' ' -f1)\" = "
        "cc6c2c8ea415b504a2cdbfa9620eeb656b25fcbe9091663972ffcfbba77aa21e; "
        f"test \"$(sha256sum {ue.home}/pavonis_sim_power_cycle.jar | cut -d ' ' -f1)\" = "
        "6ee9fee93c89a183bd385802824404747280b473fcf1b72402fa7eacca599da2; "
        "test \"$(adb devices | awk '$2 == \"device\" {n++} END {print n+0}')\" = 1; "
        "serial=$(adb devices | awk '$2 == \"device\" {print $1}'); "
        "adb -s \"$serial\" shell su -c id | grep -q 'uid=0'; "
        "echo PAVONIS_UE_AUX_DOCTOR=PASS"
    )
    rc = run_ssh(
        site, ue, ["bash", "-lc", ue_aux], args.log_dir / "ue-aux.log", 45
    )
    if rc:
        return rc
    print("PAVONIS_DOCTOR=PASS scope=both_roles")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
