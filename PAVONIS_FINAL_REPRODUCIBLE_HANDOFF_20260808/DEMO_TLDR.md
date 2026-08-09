# Pavonis Demonstration TLDR

## What is proven

A real Android handset used a private 5G NR standalone cell transmitted by a
LiteX M2SDR, registered with the project-owned qcore, established a private
PDU session, exchanged data in both directions, resolved public DNS, and
reached the public internet. The best final application result was
`25.1/1.61 Mbit/s` down/up. The same final fast run measured `19.4/0.436
Mbit/s` over the controlled 45-second in-network instrument.

## What is not claimed

This is a one-cell, one-handset proof of concept. The proven phone sequence is
primer-assisted; uplink remains much slower than downlink; and several
LiteX/Soapy timing and compatibility controls are explicit workarounds. The
25.1 Mbit/s value is a demonstrated roll, not a guaranteed minimum. The
unchanged controller also requires a broad sudo profile on dedicated lab
hosts; least-privilege conversion remains product hardening.

## Reproduce it

1. Read `docs/RUNBOOK.md` and create the two dedicated service-account roles.
2. Run `./pavonis-final.sh verify`.
3. Unpack/build the pinned source and populate only the external mode-600
   private files.
4. Run the top-level `run fast` command from `README.md` with a fresh stamp.

The command enforces ZMQ user-plane proof first, CP3424-class bounded
M2SDR/bladeRF traffic second, and the real handset third. The handset stage
automatically holds long enough to prove private gateway, public IP, DNS, and
HTTPS egress over NR before teardown. A failed or missing receipt stops the chain. The authoritative final receipt is
`work/runs/<STAMP>/STAGED_SUMMARY.json`.

## Evidence

Start with `evidence/CP3634_FINAL_RUN_EVIDENCE.json`, then
`docs/REPORT_FINAL.md`. The full sanitized checkpoint history is available
under `history/`, but it is intentionally not the primary handoff.
