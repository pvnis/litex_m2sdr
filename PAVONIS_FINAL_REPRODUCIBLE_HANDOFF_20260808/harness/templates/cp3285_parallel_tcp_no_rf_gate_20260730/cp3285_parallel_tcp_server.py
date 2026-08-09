#!/usr/bin/env python3
"""Concurrent-stream, two-direction, deadline-bounded TCP server."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import socket
import threading
import time
from typing import Callable


CHUNK = b"\x5a" * (256 * 1024)


def write_json(path: Path, value: object) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def read_header(connection: socket.socket) -> tuple[dict[str, object], bytes]:
    data = bytearray()
    while b"\n" not in data:
        block = connection.recv(4096)
        if not block:
            raise RuntimeError("connection closed before request header")
        data.extend(block)
        if len(data) > 4096:
            raise RuntimeError("request header exceeds 4096 bytes")
    line, remainder = bytes(data).split(b"\n", 1)
    return json.loads(line), remainder


def send_downlink(
    connection: socket.socket, duration: float, barrier: threading.Barrier
) -> dict[str, float | int]:
    with connection:
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
            except (BrokenPipeError, ConnectionResetError):
                break
        elapsed = time.monotonic() - started
    return {"bytes": count, "seconds": elapsed, "bytes_per_second": count / elapsed}


def receive_uplink(
    connection: socket.socket,
    duration: float,
    initial: bytes,
    barrier: threading.Barrier,
) -> dict[str, float | int]:
    with connection:
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
        count = len(initial)
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
        response = json.dumps({"received_bytes": count, "seconds": elapsed}).encode() + b"\n"
        try:
            connection.sendall(response)
        except OSError:
            pass
    return {"bytes": count, "seconds": elapsed, "bytes_per_second": count / elapsed}


def run_workers(
    target: Callable[..., dict[str, float | int]],
    arguments: list[tuple[object, ...]],
) -> list[dict[str, float | int]]:
    results: list[dict[str, float | int] | None] = [None] * len(arguments)
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


def aggregate(rows: list[dict[str, float | int]]) -> dict[str, object]:
    elapsed = max(float(row["seconds"]) for row in rows)
    count = sum(int(row["bytes"]) for row in rows)
    return {
        "bytes": count,
        "seconds": elapsed,
        "bytes_per_second": count / elapsed,
        "per_stream": rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--streams", type=int, default=4)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--ready", type=Path, required=True)
    parser.add_argument("--overall-timeout", type=float, default=540)
    args = parser.parse_args()
    if not 1 <= args.streams <= 16:
        raise SystemExit("streams must be 1..16")

    results: dict[str, object] = {
        "schema": 2,
        "complete": False,
        "streams": args.streams,
        "directions": {},
    }
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("0.0.0.0", args.port))
        listener.listen(args.streams * 2)
        listener.settimeout(args.overall_timeout)
        args.ready.write_text("ready\n", encoding="ascii")
        for expected in ("downlink", "uplink"):
            accepted: list[tuple[socket.socket, bytes]] = []
            for _ in range(args.streams):
                connection, _ = listener.accept()
                header, initial = read_header(connection)
                if header.get("direction") != expected:
                    connection.close()
                    raise RuntimeError(f"expected {expected}, got {header.get('direction')}")
                requested = float(header.get("duration", 0))
                if abs(requested - args.duration) > 0.001:
                    connection.close()
                    raise RuntimeError("client/server duration mismatch")
                accepted.append((connection, initial))

            barrier = threading.Barrier(args.streams)
            if expected == "downlink":
                if any(initial for _, initial in accepted):
                    raise RuntimeError("unexpected downlink request payload")
                rows = run_workers(
                    send_downlink,
                    [(connection, args.duration, barrier) for connection, _ in accepted],
                )
            else:
                rows = run_workers(
                    receive_uplink,
                    [
                        (connection, args.duration, initial, barrier)
                        for connection, initial in accepted
                    ],
                )
            results["directions"][expected] = aggregate(rows)
            write_json(args.result, results)
    results["complete"] = True
    write_json(args.result, results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

