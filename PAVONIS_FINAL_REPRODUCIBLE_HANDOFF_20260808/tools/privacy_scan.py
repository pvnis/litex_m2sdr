#!/usr/bin/env python3
"""Fail closed on private identities, credentials, payloads, and run data."""

from __future__ import annotations

import os
from pathlib import Path
import re
import sys


PACKAGE = Path(__file__).resolve().parents[1]
BUNDLES = PACKAGE / "source" / "bundles"
FORBIDDEN_PATHS = (
    re.compile(r"(^|/)sims\.toml$", re.I),
    re.compile(r"(^|/)real-sims\.toml$", re.I),
    re.compile(r"\.local\.", re.I),
    re.compile(r"\.(pcap|dlf)$", re.I),
    re.compile(r"(^|/)(id_rsa|id_ed25519)$", re.I),
    re.compile(r"(^|/)(runs?|build(?:-clion)?)(/|$)", re.I),
)
CONTENT_RULES = (
    ("private-key", re.compile(rb"-----BEGIN [^-]*(PRIVATE KEY|OPENSSH PRIVATE KEY)-----")),
    ("known-phone-serial", re.compile(bytes.fromhex("3664623231316430"), re.I)),
    ("subscriber-number", re.compile(rb"(?<![0-9a-fA-F.])[0-9]{15}(?![0-9a-fA-F])")),
    ("labelled-subscriber", re.compile(rb"(?i)\\b(?:imsi|supi|guti|5g-s-tmsi|s-tmsi)(?:\\s*(?:=|:|is)\\s*|[-/])(?!REPLACE|<redacted>|\\[subscriber-redacted\\])[0-9][0-9:._-]{5,}")),
    ("credential-literal", re.compile(rb"(?i)\\b(?:ki|opc|subscriber[_ -]?key|password|passwd|secret)\\b\\s*(?:=|:)\\s*(?!REPLACE|<private-value>|\\[credential-redacted\\])(?:[\"'][0-9a-f]{8,}[\"']|[0-9a-f]{16,})")),
    ("nas-payload", re.compile(rb"(?i)\\b(?:pdu_hex|subpdu_hex|payload_hex|nas[_ -]?(?:payload|bytes|pdu))\\b[^\\n]{0,24}(?:0x)?[0-9a-f]{16,}")),
)
FORBIDDEN_LITERALS = {
    "legacy-account": tuple(
        bytes.fromhex(value)
        for value in (
            "2f686f6d652f73746566616e",
            "73746566616e40",
            "2f686f6d652f676e6f6d6574657374",
            "676e6f6d657465737440",
        )
    ),
    "legacy-host-address": tuple(
        bytes.fromhex(value)
        for value in (
            "3130302e3130362e382e3834",
            "3130302e3130372e35302e37",
            "3132382e3137382e3132322e3530",
        )
    ),
}


def main() -> int:
    failures: list[str] = []
    for root, directories, names in os.walk(PACKAGE):
        directories[:] = [name for name in directories if name not in {".git", "__pycache__", "work"}]
        for name in names:
            path = Path(root) / name
            if path.parent == BUNDLES:
                continue
            relative = path.relative_to(PACKAGE).as_posix()
            if any(rule.search(relative) for rule in FORBIDDEN_PATHS):
                failures.append(f"forbidden-path:{relative}")
                continue
            data = path.read_bytes()
            if b"\0" in data[:8192]:
                continue
            lowered = data.lower()
            for label, literals in FORBIDDEN_LITERALS.items():
                if any(literal in lowered for literal in literals):
                    failures.append(f"{label}:{relative}")
            for label, rule in CONTENT_RULES:
                if rule.search(data):
                    failures.append(f"{label}:{relative}")
    if failures:
        for failure in failures:
            print(f"PRIVACY_FAIL: {failure}", file=sys.stderr)
        return 1
    print("PAVONIS_PRIVACY_SCAN=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
