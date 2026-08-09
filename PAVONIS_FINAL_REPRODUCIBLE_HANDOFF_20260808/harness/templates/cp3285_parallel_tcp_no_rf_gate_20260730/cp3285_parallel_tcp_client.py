#!/usr/bin/env python3
"""Concurrent-stream TCP throughput client for the isolated UE namespace."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import socket
import threading
import time
from typing import Callable


CHUNK = b"\xa5" * (256 * 1024)


def write_json(path: Path, value: object) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def request(connection: socket.socket, direction: str, duration: float) -> None:
    connection.sendall(
        json.dumps({"direction": direction, "duration": duration}).encode() + b"\n"
    )


def receive_downlink(
    host: str, port: int, duration: float, barrier: threading.Barrier
) -> dict[str, float | int]:
    with socket.create_connection((host, port), timeout=10) as connection:
        request(connection, "downlink", duration)
        barrier.wait()
        started = time.monotonic()

        def stop_receive() -> None:
            remaining = started + duration - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            try:
                connection.shutdown(socket.SHUT_RD)
            except OSError:
                pass

        stopper = threading.Thread(target=stop_receive, daemon=True)
        stopper.start()
        count = 0
        while True:
            try:
                block = connection.recv(256 * 1024)
            except OSError:
                break
            if not block:
                break
            count += len(block)
        elapsed = time.monotonic() - started
        stopper.join(timeout=1)
    return {"bytes": count, "seconds": elapsed, "bytes_per_second": count / elapsed}


def send_uplink(
    host: str, port: int, duration: float, barrier: threading.Barrier
) -> dict[str, object]:
    with socket.create_connection((host, port), timeout=10) as connection:
        request(connection, "uplink", duration)
        connection.settimeout(0.5)
        barrier.wait()
        started = time.monotonic()
        deadline = started + duration
        count = 0
        while time.monotonic() < deadline:
            try:
                count += connection.send(CHUNK)
            except socket.timeout:
                continue
        elapsed = time.monotonic() - started
        try:
            connection.shutdown(socket.SHUT_WR)
        except OSError:
            pass
        connection.settimeout(5)
        response = bytearray()
        while b"\n" not in response:
            block = connection.recv(4096)
            if not block:
                break
            response.extend(block)
        receiver = json.loads(response.split(b"\n", 1)[0]) if response else {}
    return {
        "bytes": count,
        "seconds": elapsed,
        "bytes_per_second": count / elapsed,
        "receiver_ack": receiver,
    }


def run_workers(
    target: Callable[..., dict[str, object]],
    arguments: list[tuple[object, ...]],
) -> list[dict[str, object]]:
    results: list[dict[str, object] | None] = [None] * len(arguments)
    errors: list[BaseException] = []

    def worker(index: int, args: tuple[object, ...]) -> None:
        try:
            results[index] = target(*args)
        except BaseException as error:
            errors.append(error)

    threads = [
        threading.Thread(target=worker, args=(index, args), daemon=True)
        for index, args in enumerate(arguments)
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    if errors:
        raise RuntimeError(f"parallel worker failed: {errors[0]}")
    return [result for result in results if result is not None]


def aggregate(rows: list[dict[str, object]], include_ack: bool = False) -> dict[str, object]:
    elapsed = max(float(row["seconds"]) for row in rows)
    count = sum(int(row["bytes"]) for row in rows)
    result: dict[str, object] = {
        "bytes": count,
        "seconds": elapsed,
        "bytes_per_second": count / elapsed,
        "per_stream": rows,
    }
    if include_ack:
        acknowledgements = [
            row.get("receiver_ack", {}) for row in rows if row.get("receiver_ack")
        ]
        ack_elapsed = max(float(ack["seconds"]) for ack in acknowledgements)
        ack_count = sum(int(ack["received_bytes"]) for ack in acknowledgements)
        result["receiver_ack"] = {
            "received_bytes": ack_count,
            "seconds": ack_elapsed,
            "per_stream": acknowledgements,
        }
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--streams", type=int, default=4)
    parser.add_argument("--result", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.streams <= 16:
        raise SystemExit("streams must be 1..16")

    result: dict[str, object] = {
        "schema": 2,
        "complete": False,
        "streams": args.streams,
        "requested_seconds_per_direction": args.duration,
    }
    barrier = threading.Barrier(args.streams)
    downlink_rows = run_workers(
        receive_downlink,
        [(args.host, args.port, args.duration, barrier)] * args.streams,
    )
    result["downlink"] = aggregate(downlink_rows)
    write_json(args.result, result)

    barrier = threading.Barrier(args.streams)
    uplink_rows = run_workers(
        send_uplink,
        [(args.host, args.port, args.duration, barrier)] * args.streams,
    )
    result["uplink"] = aggregate(uplink_rows, include_ack=True)
    result["complete"] = True
    write_json(args.result, result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

