# Pavonis Agent Handoff

## Current state

The user-data goal is complete and checkpointed. Do not reopen RF bring-up as
unfinished work. The final valid phone runs are the 10 MHz slow profile
`20260805T120946Z` and the 15 MHz MCS17 fast profile
`20260805T123403Z`. The latter proved registration, PDU activation,
bidirectional private traffic, DNS/public egress, 19.4 Mbit/s sustained
in-network downlink, 21.0 Mbit/s public HTTPS downlink, and a 25.1/1.61
Mbit/s speed test.

CP3638 is the final packaging/reproducibility closure. CP3637 established the
ordered public entry point; CP3638 then corrected the sanitized-history range
and the explicit srsRAN Soapy build option. The entry point enforces ZMQ ->
bounded M2SDR/bladeRF -> phone, with a hashed receipt at every stage. Primary
RF evidence is `evidence/CP3634_FINAL_RUN_EVIDENCE.json`. The authoritative
checkpoint corpus is `history/sanitized-checkpoints-final/`; the adjacent
incremental corpus is provenance only. `docs/CHANGE_SCOPE_AUDIT.md` records
the bounded “nothing else changed” conclusion, and `docs/COMPLETION_AUDIT.md`
maps every handoff requirement to current evidence.

## Proven controller

`pavonis-final.sh` is the only operator entry point. Its `run` command verifies the package,
materializes account/site paths, regenerates the dependency hash chain,
passes the local ZMQ user-plane gate, deploys/checks both roles, passes the
CP3424 bounded srsUE/bladeRF gate, and only then invokes either phone profile.
The phone adapter uses the proven hold to validate public-IP, DNS, and HTTPS
egress on the NR interface before release and records that proof in its
receipt:

- `slow`: corrected CP3121 hold fork;
- `fast`: corrected CP3612/CP3586 hold fork with the restored Soapy-source pin.

The package uses role names `ran` and `ue`; no historic personal account or
site address is embedded. Both roles must use the same service-account home.
The compatibility helper filenames remain because 80-plus proven scripts
call them, but their implementation is key-first and config driven.

The UE role also needs the packaged project helper JARs and pinned QCSuper
environment. `pavonis-final.sh install-ue-aux` installs them into the selected
service-account home. `doctor` verifies their hashes, QCSuper version, one
authorized handset, and working root access before any full staged run.

The unchanged controller's privilege model is intentionally documented rather
than disguised: its user-owned real-time helper is run through sudo, so the
dedicated-host example profile is broad. Do not call it least privilege or use
it on a shared/production system. A future hardening branch must move those
operations into root-owned fixed-function wrappers before narrowing sudo.

## Known workarounds

Do not present these as fixes:

- srsENB primer before OCUDU;
- primer duplicate reactive downlink and one post-ACK UL grant;
- CP2074B timed-remainder handling and per-shape TX lead;
- post-setup attenuation reassert;
- SI-PDSCH power-sign compatibility;
- empirical PRACH/TA/non-PRACH timing compensation;
- full phone reset/SIM cycle and enabled data roaming.

The retired 512-entry synthetic grant budget, forced AL4, broad stage-1
sweeps, per-sample hot-path traces, and full-rate append are not required.

## If work resumes

Start with `README.md`, then `docs/RUNBOOK.md`. Run `verify` and both `dry-run`
gates before RF. Never edit or reuse the historical package or a prior stamp.
A clean rebuild must produce explicit runtime pin overrides; the public `run`
command enforces ZMQ, bounded hardware, then phone OTA in order. Do not infer an RF regression from
one first-run-after-reboot deaf/timeout roll; rerun the same shape once.

The next technical research frontier, if separately approved, is not basic
attach. It is removal of the primer and transport/timing workarounds, followed
by strict-clean low-latency HARQ occupancy, 15 MHz rank/MIMO, and 20 MHz
producer headroom. None is required for the delivered POC claim.
