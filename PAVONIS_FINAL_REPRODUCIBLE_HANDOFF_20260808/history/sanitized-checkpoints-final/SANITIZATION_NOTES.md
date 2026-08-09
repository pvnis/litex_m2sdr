# Authoritative Sanitized Checkpoint History

## Scope

This directory is the final privacy-clean checkpoint corpus for the supervisor
handoff. It contains every available report through CP3637:

- `3,587` report files;
- `3,586` checkpoint numbers;
- CP0072 has two distinct reports, preserved as `CP0072_A.md` and
  `CP0072_B.md`;
- CP0001-CP0016 have no source report files;
- CP3633 has artifacts but no report or accepted conclusion and is not
  invented here.

## Method

The corpus was rebuilt atomically from the original report files in checkpoint
order. The deterministic campaign sanitizer is followed by the final package
privacy policy. Together they remove dates, wall clocks, host/user and endpoint
identities, IP/MAC addresses, network-interface names, attached device
identifiers, subscriber/network identities, credential references, and
NAS/payload-like byte strings while preserving technical conclusions and
artifact hashes.

Every report had to satisfy all of these gates before the output directory was
published:

1. no residual sensitive-category or package-policy match;
2. source and sanitized embedded SHA-256 multisets are identical;
3. no duplicate or unexpected checkpoint ID;
4. output membership exactly matches the source report corpus.

The hostname-assignment rule is line-local. This prevents an assignment label
at the end of one line from consuming a protected SHA placeholder on the next.

## Integrity

`HASH_MANIFEST.tsv` records the original source SHA-256 and final sanitized
SHA-256 for every report. Its SHA-256 is:

`1603f317ca0922e61b5d9c0946fb9d8d6f14ae2a1b5fcaef9e41867d078dc3dd`.

`tools/verify_package.py` validates every final sanitized hash, exact corpus
coverage, the privacy scan, and the package-wide manifest. The earlier
`history/sanitized-checkpoints/` path is retained only as a redirect; this
directory is the one to read or redistribute.
