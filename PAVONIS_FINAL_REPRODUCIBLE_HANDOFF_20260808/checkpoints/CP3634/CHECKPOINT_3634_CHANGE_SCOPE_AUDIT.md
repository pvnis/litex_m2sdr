# CP3634 - Final Change-Scope Audit

## Verdict

PASS, with a stated evidence boundary. No additional persistent change was
found in the recorded package, controller scripts, four frozen CP3121 files,
or the four relevant source repositories. The two documented RAN-role changes
are confirmed: one LiteX Soapy source restoration and one build-module
alignment, both with private pre-change backups.

This is not claimed as a whole-disk forensic proof because no complete
pre-session host snapshot existed.

## Evidence

- RAN-role repository-time audit since 2026-08-05: LiteX Soapy source `1`
  changed source file; OCUDU `0`; srsRAN `0`; qcore `0`.
- Current Soapy source SHA-256:
  `644e7858ee88d92791a495b7f4a24d762c9e819afb5337c0ef20d02e65ca4e76`.
- Built and deployed Soapy module SHA-256:
  `81b8cb2e079ed050a0056768cc349b0aaf31b25ae145eca359bd24934f4ff97c`.
- M2SDR module SHA-256:
  `3ded84bbc2ac7ff26a8192666c4083041ae54ebdfd808b48633ccedc91261ead`.
- Stack process count at audit: zero.
- Frozen CP3121 and eight hold-fork hashes are recorded in the sanitized JSON
  below.

## Artifacts

- `PAVONIS_FINAL_REPRODUCIBLE_HANDOFF_20260808/docs/CHANGE_SCOPE_AUDIT.md`
  SHA-256 `3239c2ac731faecb0f7200b0199639bc06078ac375354a28038f85a1b02e759a`.
- `PAVONIS_FINAL_REPRODUCIBLE_HANDOFF_20260808/evidence/CP3634_FINAL_RUN_EVIDENCE.json`
  SHA-256 `1a860a20f5b05faed1a35ab5319a1637069c4dd875f5537790dc605e085990ea`.
- `PAVONIS_FINAL_REPRODUCIBLE_HANDOFF_20260808/evidence/final-session/SESSION_RECORD_SANITIZED.md`
  SHA-256 `1e6767fc7aba4091f9ee2891c74b688a2d0d5c8459f664b31cf0535f42a69b74`.

## Frontier effect

The successful final phone runs are accepted as the campaign closure. CP3633
has raw attempt artifacts but no checkpoint report or accepted conclusion; it
is superseded by the final handoff and must not be used as a result.

