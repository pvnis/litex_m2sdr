#!/usr/bin/env python3
"""Receive-shutdown-bounded source/sink for the CP3465 benchmark."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


BLOCK = b"\0" * 65536
DOWNLOAD_LIMIT = 1 << 31
MIN_DURATION = 30.0
MAX_DURATION = 60.0


class ResultStore:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.values: dict[str, dict[str, float | int]] = {}

    def record(self, direction: str, byte_count: int, elapsed: float) -> None:
        with self.lock:
            self.values[direction] = {
                "bytes": byte_count,
                "seconds": elapsed,
                "bytes_per_second": byte_count / elapsed if elapsed > 0 else 0.0,
            }

    def complete(self) -> bool:
        with self.lock:
            return "downlink" in self.values and "uplink" in self.values

    def snapshot(self) -> dict[str, dict[str, float | int]]:
        with self.lock:
            return dict(self.values)


class BenchmarkServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], store: ResultStore) -> None:
        super().__init__(address, BenchmarkHandler)
        self.store = store


class BenchmarkHandler(BaseHTTPRequestHandler):
    server: BenchmarkServer

    def log_message(self, _format: str, *_args: object) -> None:
        return

    def do_GET(self) -> None:
        if self.path == "/ready":
            self.send_response(200)
            self.send_header("Content-Length", "2")
            self.end_headers()
            self.wfile.write(b"OK")
            return
        if self.path != "/download":
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Content-Type", "application/octet-stream")
        self.send_header("Content-Length", str(DOWNLOAD_LIMIT))
        self.end_headers()
        start = time.monotonic()
        sent = 0
        try:
            while sent < DOWNLOAD_LIMIT:
                count = min(len(BLOCK), DOWNLOAD_LIMIT - sent)
                self.wfile.write(BLOCK[:count])
                sent += count
        except (BrokenPipeError, ConnectionResetError, socket.timeout):
            pass
        elapsed = max(time.monotonic() - start, 1e-9)
        self.server.store.record("downlink", sent, elapsed)

    def do_POST(self) -> None:
        if self.path != "/upload":
            self.send_error(404)
            return

        try:
            remaining = int(self.headers.get("Content-Length", "0"))
            duration = float(self.headers.get("X-Pavonis-Duration", "0"))
        except ValueError:
            self.send_error(400)
            return
        if not MIN_DURATION <= duration <= MAX_DURATION:
            self.send_error(400)
            return

        start = time.monotonic()
        read_done = threading.Event()
        deadline_fired = threading.Event()

        def stop_receive_at_deadline() -> None:
            if read_done.wait(duration):
                return
            deadline_fired.set()
            try:
                self.connection.shutdown(socket.SHUT_RD)
            except OSError:
                pass

        deadline_thread = threading.Thread(
            target=stop_receive_at_deadline, daemon=True
        )
        deadline_thread.start()
        received = 0
        try:
            while remaining > 0:
                chunk = self.rfile.read(min(65536, remaining))
                if not chunk:
                    break
                received += len(chunk)
                remaining -= len(chunk)
        except (ConnectionResetError, OSError):
            pass
        finally:
            read_done.set()
            deadline_thread.join(timeout=1)
        elapsed = max(time.monotonic() - start, 1e-9)
        self.server.store.record("uplink", received, elapsed)
        self.close_connection = True
        try:
            self.send_response(200)
            self.send_header("Content-Length", "2")
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(b"OK")
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--timeout", type=int, default=300)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 1024 <= args.port <= 65535:
        raise SystemExit("invalid port")
    if not 120 <= args.timeout <= 600:
        raise SystemExit("invalid timeout")

    store = ResultStore()
    server = BenchmarkServer(("0.0.0.0", args.port), store)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print("CP3153_SUSTAINED_SERVER_ARMED=1", flush=True)

    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and not store.complete():
        time.sleep(0.25)

    server.shutdown()
    server.server_close()
    thread.join(timeout=5)
    payload = {
        "schema": 1,
        "complete": store.complete(),
        "directions": store.snapshot(),
    }
    args.result.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(
        f"CP3153_SUSTAINED_SERVER_COMPLETE={int(payload['complete'])}",
        flush=True,
    )
    return 0 if payload["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
