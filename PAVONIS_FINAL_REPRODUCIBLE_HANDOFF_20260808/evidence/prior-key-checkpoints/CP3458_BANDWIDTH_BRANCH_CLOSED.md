# CP3458 - Bandwidth branch closed

## Verdict

CP3458 is the second consecutive correctly configured PDSCH7 access void. The
bandwidth branch is closed without a demonstrated hardware throughput gain.

## Evidence

- All three packaged gNB configs contain PDSCH maximum MCS7.
- The UE completes cell search but does not select the cell, synchronize SFN,
  decode SIB1, or transmit PRACH. There is no endpoint traffic.
- The pre-UE TX gate passes. Final TX has one deadline break and `12294`
  stream-deficit samples, with zero hard errors.
- CP3457 immediately before it also had correct PDSCH7 propagation but stopped
  at RAR visibility. The two-run access-void rule is therefore exhausted.

## Bandwidth conclusion

The campaign established three separate facts:

1. CP3451 proves the corrected 15 MHz timing can attach and carry bidirectional
   user data, but only at `0.540068/0.397201 Mbit/s` in that roll.
2. CP3452 proves 15 MHz/PDSCH7 has `9.015192 Mbit/s` DL capacity over ZMQ.
3. CP3457/CP3458 cannot turn that no-RF potential into a measured OTA result
   because two consecutive correctly configured runs stop in access scatter.

The robust current hardware reference remains CP3424 at
`5.630004/13.106090 Mbit/s` on 10 MHz. The historical CP3191 15 MHz result
remains valid, but the current campaign has not produced a 15 MHz gain.
Twenty MHz remains retired because its 23.04 Msps producer path is not
real-time sustainable.

## Decision

Do not retry 15 MHz or reopen 20 MHz. Proceed to a no-RF MIMO feasibility
audit, then quantify producer-stall headroom.

## Artifacts

- `cp3458_bandwidth15_pdsch7_access_retry_20260731/cp3458_summary.json`
- `cp3458_bandwidth15_pdsch7_access_retry_20260731/results/attempt_20260731T062132Z/execution_summary.json`
  - SHA-256 `2f8f42527b4422860b2f6e6f3accec37136ced2252de10acd434ac59e90d032c`
- `cp1733_stage6_realue_sib1_rachcfg_summary_20260731T062132Z.json`
  - SHA-256 `21166c25adcc5d0da7615cd0a3732b450a5ac9fdc39ad3353d6d92db29c9ee82`
- `cp3458_bandwidth15_pdsch7_access_retry_[gNB host]_20260731T062132Z.tar.gz`
  - SHA-256 `b9029a6785f60456987069c8bf34a89300a89f7738ae148a123151bdb5b579fd`
- `cp3458_bandwidth15_pdsch7_access_retry_[UE host]_20260731T062132Z.tar.gz`
  - SHA-256 `f929ddbf712359ef3f96fe879a91f936a0cac6d9f676c60e5abfd43001ebeccd`
