#!/usr/bin/env python3
"""Serve bounded echo, download and upload transactions for one lab UE."""

from __future__ import annotations

import argparse
import socket
import sys
import time


def read_request(connection: socket.socket) -> tuple[str, dict[str, str], bytes]:
    request = bytearray()
    while b"\r\n\r\n" not in request and len(request) < 16384:
        chunk = connection.recv(4096)
        if not chunk:
            break
        request.extend(chunk)
    header, separator, body = bytes(request).partition(b"\r\n\r\n")
    if not separator:
        raise RuntimeError("incomplete HTTP header")
    lines = header.decode("ascii", errors="strict").split("\r\n")
    fields: dict[str, str] = {}
    for line in lines[1:]:
        name, marker, value = line.partition(":")
        if marker:
            fields[name.strip().lower()] = value.strip()
    content_length = int(fields.get("content-length", "0"))
    payload = bytearray(body)
    while len(payload) < content_length:
        chunk = connection.recv(min(65536, content_length - len(payload)))
        if not chunk:
            break
        payload.extend(chunk)
    if len(payload) != content_length:
        raise RuntimeError("incomplete HTTP body")
    return lines[0], fields, bytes(payload)


def send_response(connection: socket.socket, body: bytes) -> None:
    header = (
        b"HTTP/1.1 200 OK\r\n"
        + f"Content-Length: {len(body)}\r\n".encode("ascii")
        + b"Content-Type: application/octet-stream\r\n"
        + b"Connection: close\r\n\r\n"
    )
    connection.sendall(header)
    connection.sendall(body)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="10.255.0.1")
    parser.add_argument("--port", type=int, default=39087)
    parser.add_argument("--nonce", required=True)
    parser.add_argument("--expected-source", default="10.255.0.2")
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--transfer-bytes", type=int, default=1048576)
    args = parser.parse_args()

    if not args.nonce.replace("-", "").replace("_", "").isalnum():
        raise SystemExit("invalid nonce")
    if not 65536 <= args.transfer_bytes <= 8388608:
        raise SystemExit("invalid transfer size")

    expected = [
        ("GET", f"/pavonis/{args.nonce}"),
        ("GET", f"/pavonis/{args.nonce}/download"),
        ("POST", f"/pavonis/{args.nonce}/upload"),
    ]
    echo_body = f"PAVONIS_CP3121_ECHO={args.nonce}\n".encode("ascii")
    download_body = b"P" * args.transfer_bytes
    upload_bytes = 0
    source_ok = True
    request_ok = True
    download_send_ns = 0
    upload_receive_ns = 0

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.bind, args.port))
        server.listen(3)
        server.settimeout(args.timeout)
        print(
            f"CP3121_CORE_HTTP_SERVER=ARMED port={args.port} transfer_bytes={args.transfer_bytes}",
            flush=True,
        )

        for index, (method, path) in enumerate(expected):
            connection, peer = server.accept()
            with connection:
                connection.settimeout(15.0)
                receive_start_ns = time.monotonic_ns()
                request_line, _, payload = read_request(connection)
                receive_end_ns = time.monotonic_ns()
                source_ok = source_ok and peer[0] == args.expected_source
                request_ok = request_ok and request_line == f"{method} {path} HTTP/1.1"
                if index == 0:
                    send_response(connection, echo_body)
                elif index == 1:
                    send_start_ns = time.monotonic_ns()
                    send_response(connection, download_body)
                    download_send_ns = time.monotonic_ns() - send_start_ns
                else:
                    upload_bytes = len(payload)
                    upload_receive_ns = receive_end_ns - receive_start_ns
                    send_response(connection, b"PAVONIS_CP3121_UPLOAD=PASS\n")

    passed = (
        source_ok
        and request_ok
        and upload_bytes == args.transfer_bytes
        and download_send_ns > 0
        and upload_receive_ns > 0
    )
    print(
        "CP3121_CORE_HTTP_RESULT "
        f"source_ok={int(source_ok)} request_ok={int(request_ok)} "
        f"download_bytes={args.transfer_bytes} download_send_ns={download_send_ns} "
        f"upload_bytes={upload_bytes} upload_receive_ns={upload_receive_ns} "
        f"pass={int(passed)}",
        flush=True,
    )
    print(f"CP3121_CORE_HTTP_PASS={int(passed)}", flush=True)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())

