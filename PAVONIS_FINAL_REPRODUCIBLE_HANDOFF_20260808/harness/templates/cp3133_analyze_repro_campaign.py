#!/usr/bin/env python3
"""Aggregate the hash-bound CP3122 campaign without exposing private identities."""

from __future__ import annotations

from collections import Counter
import argparse
import hashlib
import io
import json
from pathlib import Path
import tarfile
from typing import Any, Iterable

from cp3121_analyze_phone_metric_run import distribution, fields, finite


ART = Path(__file__).resolve().parent
PUSCH_MARKER = "PAVONIS_PUSCH_CSI_TRACE event=result"
METRIC_FIELDS = {
    "sinr_db": "sinr_valid",
    "total_evm": "total_evm_valid",
    "cfo_hz": "cfo_valid",
    "time_align_us": "time_align_valid",
    "epre_db": "epre_valid",
    "rsrp_db": "rsrp_valid",
}


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def pusch_rows(lines: Iterable[str]) -> list[dict[str, str]]:
    return [fields(line) for line in lines if PUSCH_MARKER in line]


def collect_distribution(
    rows: list[dict[str, str]], value_name: str, valid_name: str
) -> dict[str, float | int | None]:
    values = [
        value
        for row in rows
        if (value := finite(row, value_name, valid_name)) is not None
    ]
    return distribution(values)


def verify_attempts(
    root: Path, state: dict[str, Any]
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    attempts: list[dict[str, Any]] = []
    hashes: list[dict[str, Any]] = []
    for item in state["attempts"]:
        summary_path = root / item["summary"]
        actual_hash = sha256(summary_path)
        if actual_hash != item["summary_sha256"]:
            raise ValueError(f"attempt summary hash mismatch: {summary_path}")
        summary = load_json(summary_path)
        for key in ["stage", "index", "stamp", "classification"]:
            if summary[key] != item[key]:
                raise ValueError(f"attempt state mismatch for {key}: {summary_path}")
        attempts.append(summary)
        hashes.append(
            {
                "stage": summary["stage"],
                "index": summary["index"],
                "stamp": summary["stamp"],
                "classification": summary["classification"],
                "summary": str(summary_path.relative_to(ART)),
                "summary_sha256": actual_hash,
            }
        )
    return attempts, hashes


def stage_summary(
    attempts: list[dict[str, Any]], stage: str
) -> dict[str, Any]:
    selected = [item for item in attempts if item["stage"] == stage]
    classes = Counter(item["classification"] for item in selected)
    nonvoid = [
        item
        for item in selected
        if item["classification"] in {"valid_success", "valid_failure"}
    ]
    successes = classes["valid_success"]
    return {
        "raw": len(selected),
        "class_counts": dict(sorted(classes.items())),
        "valid_evaluable": len(nonvoid),
        "successes": successes,
        "success_rate_evaluable": successes / len(nonvoid) if nonvoid else None,
    }


def metric_distribution(
    phone: list[dict[str, Any]], *keys: str
) -> dict[str, float | int | None]:
    values: list[float] = []
    for item in phone:
        value: Any = item["metrics"]
        for key in keys:
            value = value[key]
        values.append(float(value))
    return distribution(values)


def seed_gnb_rows(
    attempt_dir: Path, metric: dict[str, Any]
) -> tuple[list[dict[str, str]], dict[str, Any]]:
    provenance = metric["provenance"]
    archive = ART / provenance["nuc_archive"]["file"]
    if sha256(archive) != provenance["nuc_archive"]["sha256"]:
        raise ValueError("seed ran archive hash mismatch")
    with tarfile.open(archive, "r:gz") as handle:
        members = [
            member for member in handle.getmembers()
            if member.isfile() and member.name.endswith("/gnb.snapshot.log")
        ]
        if len(members) != 1:
            raise ValueError("seed archive does not contain exactly one gNB snapshot")
        extracted = handle.extractfile(members[0])
        if extracted is None:
            raise ValueError("cannot read seed gNB snapshot")
        data = extracted.read()
    if sha256_bytes(data) != provenance["gnb_log"]["sha256"]:
        raise ValueError("seed gNB snapshot hash mismatch")
    rows = pusch_rows(io.TextIOWrapper(io.BytesIO(data), encoding="utf-8", errors="replace"))
    return rows, {
        "index": 1,
        "source": str(archive.relative_to(ART)),
        "archive_sha256": provenance["nuc_archive"]["sha256"],
        "gnb_snapshot_sha256": provenance["gnb_log"]["sha256"],
        "gnb_snapshot_bytes": provenance["gnb_log"]["bytes"],
        "pusch_rows": len(rows),
    }


def phone_gnb_rows(
    root: Path, summary: dict[str, Any]
) -> tuple[list[dict[str, str]], dict[str, Any]]:
    index = int(summary["index"])
    attempt_dir = next(
        path for path in (root / "phone").glob(f"raw_{index:03d}_*") if path.is_dir()
    )
    artifact = summary["artifacts"]["gnb_log"]
    snapshot = attempt_dir / artifact["file"]
    actual_hash = sha256(snapshot)
    if actual_hash != artifact["sha256"]:
        raise ValueError(f"phone raw{index} gNB snapshot hash mismatch")
    rows = pusch_rows(
        snapshot.open("r", encoding="utf-8", errors="replace")
    )
    return rows, {
        "index": index,
        "source": str(snapshot.relative_to(ART)),
        "gnb_snapshot_sha256": actual_hash,
        "gnb_snapshot_bytes": snapshot.stat().st_size,
        "pusch_rows": len(rows),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--campaign-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    root = args.campaign_root.resolve()
    state_path = root / "campaign_state.json"
    state = load_json(state_path)
    attempts, attempt_hashes = verify_attempts(root, state)
    phone = sorted(
        (
            item for item in attempts
            if item["stage"] == "phone"
            and item["classification"] == "valid_success"
        ),
        key=lambda item: int(item["index"]),
    )
    if len(phone) != 20:
        raise ValueError(f"expected 20 valid phone rows, found {len(phone)}")

    all_pusch: list[dict[str, str]] = []
    pusch_sources: list[dict[str, Any]] = []
    for summary in phone:
        index = int(summary["index"])
        attempt_dir = root / "phone" / "seed_cp3121d"
        metric_path = attempt_dir / "metric_summary.json"
        if index == 1:
            metric = load_json(metric_path)
            rows, source = seed_gnb_rows(attempt_dir, metric)
        else:
            rows, source = phone_gnb_rows(root, summary)
        expected = int(summary["metrics"]["pusch"]["results"])
        if len(rows) != expected:
            raise ValueError(
                f"phone raw{index} PUSCH count mismatch: {len(rows)} != {expected}"
            )
        all_pusch.extend(rows)
        pusch_sources.append(source)

    crc_ok = [row for row in all_pusch if row.get("crc") == "OK"]
    pooled = {
        "results": len(all_pusch),
        "crc_ok": len(crc_ok),
        "crc_ko": len(all_pusch) - len(crc_ok),
        "crc_yield": len(crc_ok) / len(all_pusch),
        "crc_ok_tbs11": sum(row.get("tbs_bytes") == "11" for row in crc_ok),
        "all_results": {
            name: collect_distribution(all_pusch, name, valid)
            for name, valid in METRIC_FIELDS.items()
        },
        "crc_ok_results": {
            name: collect_distribution(crc_ok, name, valid)
            for name, valid in METRIC_FIELDS.items()
        },
    }

    comparable = {
        "zmq_frozen_raw3_7": {
            "raw": 5,
            "successes": 5,
            "success_rate": 1.0,
        },
        "bounded_complete_fixture_raw8_14": {
            "raw": 7,
            "successes": 5,
            "failures": 2,
            "success_rate": 5 / 7,
            "failure_frontier": "Stage 6 PRACH Msg1 gNB detection",
        },
        "phone_seed_plus_raw2_21": {
            "raw": 21,
            "valid_evaluable": 20,
            "successes": 20,
            "runner_voids": 1,
            "attach_success_rate_valid": 1.0,
            "data_success_rate_valid": 1.0,
            "raw_to_success_rate": 20 / 21,
        },
    }

    result = {
        "checkpoint": 3133,
        "campaign": state["campaign"],
        "campaign_complete": state.get("complete") is True,
        "campaign_state": {
            "file": str(state_path.relative_to(ART)),
            "sha256": sha256(state_path),
            "updated_utc": state["updated_utc"],
        },
        "targets": state["targets"],
        "raw_caps": state["raw_caps"],
        "stages_all_attempts": {
            stage: stage_summary(attempts, stage)
            for stage in ["zmq", "bounded", "phone"]
        },
        "comparable_frozen_cohorts": comparable,
        "phone": {
            "valid_runs": len(phone),
            "attach_successes": sum(
                item["metrics"]["attach"]["time_to_attach_ms"] is not None
                for item in phone
            ),
            "data_successes": sum(
                item["metrics"]["data_gates"]["final"] == 1 for item in phone
            ),
            "time_to_attach_ms": metric_distribution(
                phone, "attach", "time_to_attach_ms"
            ),
            "echo_rtt_avg_ms": metric_distribution(phone, "echo", "rtt_avg_ms"),
            "http_round_trip_s": metric_distribution(phone, "http", "echo_time_s"),
            "download_Bps": metric_distribution(phone, "http", "download_Bps"),
            "upload_Bps": metric_distribution(phone, "http", "upload_Bps"),
            "download_time_s": metric_distribution(phone, "http", "download_time_s"),
            "upload_time_s": metric_distribution(phone, "http", "upload_time_s"),
            "echo_packets": {
                "received": sum(item["metrics"]["echo"]["received"] for item in phone),
                "expected": 4 * len(phone),
            },
            "runtime_markers": {
                "fallback_al4": sum(
                    item["metrics"]["runtime"]["fallback_al4_markers"] for item in phone
                ),
                "fallback_al8": sum(
                    item["metrics"]["runtime"]["fallback_al8_markers"] for item in phone
                ),
                "scheduler_rebase": sum(
                    item["metrics"]["runtime"]["scheduler_rebase_markers"]
                    for item in phone
                ),
            },
            "health_totals": {
                key: sum(item["metrics"]["health"][key] for item in phone)
                for key in [
                    "soapy_tx_timeout",
                    "downlink_late",
                    "underflow",
                    "time_error",
                ]
            },
            "pooled_pusch": pooled,
        },
        "provenance": {
            "attempt_summaries": attempt_hashes,
            "phone_pusch_sources": pusch_sources,
        },
        "privacy": {
            "contains_credentials": False,
            "contains_subscriber_identifiers": False,
            "contains_nas_payloads": False,
        },
    }
    args.output.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(
        "CP3133_CAMPAIGN_AGGREGATE=PASS "
        f"phone={len(phone)} pusch={len(crc_ok)}/{len(all_pusch)} "
        f"output={args.output}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
