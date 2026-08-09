#!/usr/bin/env python3
"""Produce a privacy-safe, hash-traceable Pavonis phone-run summary."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import re
import statistics


FIELD_RE = re.compile(r"([A-Za-z0-9_]+)=([^\s]+)")


def fields(line: str) -> dict[str, str]:
    return dict(FIELD_RE.findall(line))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def marker(path: Path, prefix: str) -> dict[str, str]:
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if line.startswith(prefix):
                return fields(line)
    raise ValueError(f"missing marker {prefix} in {path.name}")


def marker_value(path: Path, name: str) -> int:
    prefix = f"{name}="
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if line.startswith(prefix):
                return int(line.removeprefix(prefix).strip())
    raise ValueError(f"missing marker {name} in {path.name}")


def finite(row: dict[str, str], value_name: str, valid_name: str) -> float | None:
    if row.get(valid_name) not in {"1", "true"}:
        return None
    try:
        value = float(row[value_name])
    except (KeyError, ValueError):
        return None
    return value if math.isfinite(value) else None


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def distribution(values: list[float]) -> dict[str, float | int | None]:
    if not values:
        return {
            "count": 0,
            "min": None,
            "p05": None,
            "median": None,
            "mean": None,
            "p95": None,
            "max": None,
        }
    return {
        "count": len(values),
        "min": min(values),
        "p05": percentile(values, 0.05),
        "median": statistics.median(values),
        "mean": statistics.fmean(values),
        "p95": percentile(values, 0.95),
        "max": max(values),
    }


def parse_gnb(path: Path) -> dict[str, object]:
    runtime = Counter()
    path_events: Counter[str] = Counter()
    pusch_rows: list[dict[str, str]] = []
    health = Counter()

    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if "PAVONIS_GNB_FALLBACK_DL_DCI_AGGR_LEV active value=4 level=n4" in line:
                runtime["fallback_al4"] += 1
            if "PAVONIS_GNB_FALLBACK_DL_DCI_AGGR_LEV active value=8 level=n8" in line:
                runtime["fallback_al8"] += 1
            if "PAVONIS_MSG3_CONRES_SLOT_REBASE" in line:
                runtime["scheduler_rebase"] += 1

            if "PAVONIS_MSG3_PATH_TRACE" in line:
                row = fields(line)
                event = row.get("event")
                if event:
                    path_events[f"{event}:{row.get('outcome', '')}"] += 1

            if "PAVONIS_PUSCH_CSI_TRACE event=result" in line:
                pusch_rows.append(fields(line))

            lowered = line.lower()
            if (
                "soapysdrdevice_writestream returned" in lowered
                and "timeout" in lowered
            ) or "soapysdr tx: writestream timeout" in lowered:
                health["soapy_tx_timeout"] += 1
            if "downlink late" in lowered or "downlink_late" in lowered:
                health["downlink_late"] += 1
            if "underflow" in lowered:
                health["underflow"] += 1
            if "time_error" in lowered or "time error" in lowered:
                health["time_error"] += 1

    crc_ok_rows = [row for row in pusch_rows if row.get("crc") == "OK"]
    metric_fields = {
        "sinr_db": "sinr_valid",
        "total_evm": "total_evm_valid",
        "symbol_evm": "symbol_evm_count",
        "cfo_hz": "cfo_valid",
        "time_align_us": "time_align_valid",
        "epre_db": "epre_valid",
        "rsrp_db": "rsrp_valid",
    }

    def collect(rows: list[dict[str, str]]) -> dict[str, dict[str, float | int | None]]:
        output: dict[str, dict[str, float | int | None]] = {}
        for value_name, valid_name in metric_fields.items():
            values: list[float] = []
            for row in rows:
                if value_name == "symbol_evm":
                    try:
                        expected = int(row.get(valid_name, "0"))
                    except (KeyError, ValueError):
                        continue
                    parsed: list[float] = []
                    for item in row.get(value_name, "").split(","):
                        _, separator, raw_value = item.partition(":")
                        if not separator:
                            continue
                        try:
                            value = float(raw_value)
                        except ValueError:
                            continue
                        if math.isfinite(value):
                            parsed.append(value)
                    if expected > 0 and len(parsed) == expected:
                        values.extend(parsed)
                    continue
                value = finite(row, value_name, valid_name)
                if value is not None:
                    values.append(value)
            output[value_name] = distribution(values)
        return output

    return {
        "runtime": {
            "fallback_al4_markers": runtime["fallback_al4"],
            "fallback_al8_markers": runtime["fallback_al8"],
            "scheduler_rebase_markers": runtime["scheduler_rebase"],
        },
        "pusch": {
            "results": len(pusch_rows),
            "crc_ok": len(crc_ok_rows),
            "crc_ko": len(pusch_rows) - len(crc_ok_rows),
            "crc_yield": len(crc_ok_rows) / len(pusch_rows) if pusch_rows else None,
            "crc_ok_tbs11": sum(
                row.get("tbs_bytes") == "11" for row in crc_ok_rows
            ),
            "all_results": collect(pusch_rows),
            "crc_ok_results": collect(crc_ok_rows),
        },
        "msg3_path_event_counts": dict(sorted(path_events.items())),
        "msg4": {
            "new_tx_allocated": path_events["fallback_pdsch:new_tx_allocated"],
            "conres_expired": path_events["fallback_pdsch:conres_expired"],
        },
        "health": {
            "soapy_tx_timeout": health["soapy_tx_timeout"],
            "downlink_late": health["downlink_late"],
            "underflow": health["underflow"],
            "time_error": health["time_error"],
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stamp", required=True)
    parser.add_argument("--gnb-log", required=True, type=Path)
    parser.add_argument("--attach", required=True, type=Path)
    parser.add_argument("--core-http", required=True, type=Path)
    parser.add_argument("--phone-http", required=True, type=Path)
    parser.add_argument("--phone-echo", required=True, type=Path)
    parser.add_argument("--run-summary", required=True, type=Path)
    parser.add_argument("--nuc-archive", required=True, type=Path)
    parser.add_argument("--sens-archive", required=True, type=Path)
    parser.add_argument("--capture-archive", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--require-pass", action="store_true")
    args = parser.parse_args()

    inputs = {
        "gnb_log": args.gnb_log,
        "attach": args.attach,
        "core_http": args.core_http,
        "phone_http": args.phone_http,
        "phone_echo": args.phone_echo,
        "run_summary": args.run_summary,
        "nuc_archive": args.nuc_archive,
        "sens_archive": args.sens_archive,
        "capture_archive": args.capture_archive,
    }
    for path in inputs.values():
        if not path.is_file():
            raise SystemExit(f"missing input {path}")

    gnb = parse_gnb(args.gnb_log)
    attach = marker(args.attach, "CP3121_ATTACH_METRICS ")
    core_http = marker(args.core_http, "CP3121_CORE_HTTP_RESULT ")
    phone_http = marker(args.phone_http, "CP3121_PHONE_HTTP ")
    phone_echo = marker(args.phone_echo, "CP3121_PHONE_ECHO ")

    data_gates = {
        "echo": marker_value(args.phone_echo, "CP3121_PHONE_ECHO_PASS"),
        "phone_http": marker_value(args.phone_http, "CP3121_PHONE_HTTP_PASS"),
        "core_http": marker_value(args.core_http, "CP3121_CORE_HTTP_PASS"),
        "final": marker_value(args.run_summary, "PAVONIS_FINAL_DATA_GATE_PASS"),
    }

    runtime = gnb["runtime"]
    pusch = gnb["pusch"]
    msg4 = gnb["msg4"]
    health = gnb["health"]
    pass_checks = {
        "clean_al8": runtime["fallback_al4_markers"] == 0
        and runtime["fallback_al8_markers"] == 1
        and runtime["scheduler_rebase_markers"] == 0,
        "msg3": pusch["crc_ok_tbs11"] >= 1,
        "msg4": msg4["new_tx_allocated"] >= 1 and msg4["conres_expired"] == 0,
        "data": all(value == 1 for value in data_gates.values()),
        "rf_health": all(value == 0 for value in health.values()),
    }
    passed = all(pass_checks.values())

    result = {
        "checkpoint": 3121,
        "stamp": args.stamp,
        "classification": "valid_success" if passed else "valid_failure",
        "pass": passed,
        "pass_checks": pass_checks,
        "data_gates": data_gates,
        "attach": {
            "time_to_attach_ms": int(attach["time_to_attach_ms"]),
        },
        "echo": {
            "received": int(phone_echo["received"]),
            "rtt_min_ms": float(phone_echo["rtt_min_ms"]),
            "rtt_avg_ms": float(phone_echo["rtt_avg_ms"]),
            "rtt_max_ms": float(phone_echo["rtt_max_ms"]),
            "rtt_mdev_ms": float(phone_echo["rtt_mdev_ms"]),
        },
        "http": {
            "echo_time_s": float(phone_http["echo_time_s"]),
            "download_bytes": int(phone_http["download_bytes"]),
            "download_time_s": float(phone_http["download_time_s"]),
            "download_Bps": float(phone_http["download_Bps"]),
            "upload_bytes": int(phone_http["upload_bytes"]),
            "upload_time_s": float(phone_http["upload_time_s"]),
            "upload_Bps": float(phone_http["upload_Bps"]),
            "core_download_send_ns": int(core_http["download_send_ns"]),
            "core_upload_receive_ns": int(core_http["upload_receive_ns"]),
        },
        "gnb": gnb,
        "provenance": {
            name: {
                "file": path.name,
                "sha256": sha256(path),
                "bytes": path.stat().st_size,
            }
            for name, path in inputs.items()
        },
    }

    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(
        f"CP3121_RUN_ANALYSIS={'PASS' if passed else 'FAIL'} "
        f"pusch={pusch['crc_ok']}/{pusch['results']} "
        f"attach_ms={result['attach']['time_to_attach_ms']} "
        f"down_Bps={result['http']['download_Bps']:.0f} "
        f"up_Bps={result['http']['upload_Bps']:.0f}"
    )
    return 0 if passed or not args.require_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
