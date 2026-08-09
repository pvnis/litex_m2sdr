# CP3636 - Final Supervisor Release

## Verdict

The Pavonis phone user-data campaign and final handoff are complete. The
package contains a final report, human TLDR, agent handoff, cold-lab runbook,
change-scope audit, privacy-clean source, account-neutral controller,
sanitized evidence, and a secondary checkpoint archive.

The primary result artifact is `docs/REPORT_FINAL.md`, SHA-256
`6225e5372410e7525f57fc8e18a7852038872f779d718ad1b26380a71d9c6404`.
The primary result data is `evidence/CP3634_FINAL_RUN_EVIDENCE.json`, SHA-256
`1a860a20f5b05faed1a35ab5319a1637069c4dd875f5537790dc605e085990ea`.

## Release gates

- package privacy scan: pass;
- four one-commit source bundles: verify and unpack pass;
- OCUDU overlay hashes: pass;
- all shell/Python syntax: pass;
- portable controller self-test: pass;
- slow and fast RF gates: closed before SSH, `rc=2`;
- generated work/run/build/cache directories in submission: zero;
- credentials, identity values, raw captures, run archives, and private role
  files in submission: zero.

The final package manifest is `manifests/SHA256SUMS`. Its detached SHA-256 is
recorded in the authoritative external CP3636 checkpoint because a manifest
cannot contain its own digest.

## Honest boundary

No new RF run was made for packaging. The valid proof remains the two August
5 runs recorded in the evidence JSON. Clean-account transport and
materialization are no-network tested; actual use on newly provisioned hosts
must follow ZMQ, bounded-hardware, and OTA gates in `docs/RUNBOOK.md`.

