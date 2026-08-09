# Pavonis Final Reproducible Handoff

Status: **FINAL RELEASE**. This is the supervisor-facing entry point. It is
an operator package, not a draft report or a copy of the campaign workspace.

## Result in one minute

Pavonis is a private 5G NR SA lab chain built from qcore, OCUDU, a LiteX
M2SDR, a real handset, and project-owned test credentials. It has completed
OTA registration, private PDU-session activation, bidirectional user traffic,
DNS, and public-internet traffic.

The last two valid demonstrations were:

| Profile | In-network sustained DL/UL | Public-internet result |
|---|---:|---:|
| 10 MHz slow/reproducible | 6.43 / 0.092 Mbit/s | 7.9 / 0.16 Mbit/s HTTPS |
| 15 MHz MCS17 fast | 19.4 / 0.436 Mbit/s | 21.0 Mbit/s HTTPS; 25.1 / 1.61 Mbit/s speed test |

The fast result is the headline demonstration. It is an observed upper-band
roll, not a guaranteed minimum: the 20 ms timed-transmit lead is visible as
HARQ feedback latency and creates run-to-run downlink spread.

## Start here

1. Read `docs/REPORT_FINAL.md` for the result and limitations.
2. Read `docs/RUNBOOK.md` before touching either lab host.
3. Verify the package:

```bash
./pavonis-final.sh verify
```

4. Create a private site file outside this directory:

```bash
cp config/site.toml.example ../pavonis-site.toml
chmod 600 ../pavonis-site.toml
```

5. Replace every `REPLACE_...` value and configure key-based SSH for the
   dedicated `ran` and `ue` service-account roles. No historical personal
   account is required on either host.
6. Follow `docs/RUNBOOK.md` to unpack/build the pinned source, create the two
   private ZMQ role files, install the pinned UE observer prerequisites, and
   run the ordered gate:

```bash
./pavonis-final.sh run fast \
  --site ../pavonis-site.toml \
  --workspace "$HOME/pavonis-source" \
  --rf-approved \
  --stamp "$(date -u +%Y%m%dT%H%M%SZ)_FINAL"
```

That command cannot enter the handset stage until the no-RF ZMQ receipt and
the bounded M2SDR/bladeRF receipt both say `PASS`. The phone stage then proves
private sustained traffic plus gateway, public-IP, DNS, and HTTPS egress over
the handset's NR interface. It writes
`work/runs/<STAMP>/STAGED_SUMMARY.json` only after all three stages pass.

## What is included

- `docs/REPORT_FINAL.md`: final technical report, not a draft.
- `docs/RUNBOOK.md`: cold-lab build, configuration, bring-up, run, and teardown.
- `docs/RUNTIME_DEPENDENCIES.md`: complete packaged/private/site-owned boundary.
- `docs/CHANGE_SCOPE_AUDIT.md`: exact answer to “what else changed?”.
- `docs/COMPLETION_AUDIT.md`: requirement-by-requirement completion evidence.
- `docs/FILE_CHANGE_AND_SANITIZATION_INVENTORY.md`: clean-state file inventory.
- `AGENT_HANDOFF.md`: compact state for a future coding agent.
- `source/`: four privacy-clean, one-commit Git bundles plus patches/overlay.
- `harness/templates/`: slow/fast phone and CP3424 bounded-UE controllers,
  made account-neutral and re-pinned at materialization.
- `tools/`: generic SSH/SCP, source build, materialization, deployment, and verification.
- `runtime/ue-aux/`: two hash-pinned, project-owned handset helper JARs.
- `evidence/`: small current proof set and key prior checkpoint extracts.
- `history/sanitized-checkpoints-final/`: authoritative sanitized archive,
  intentionally secondary; the old path contains only a redirect.

## Honest boundary

The package contains no credentials, subscriber identity, handset serial,
private key, NAS payload, raw RF capture, run archive, or site address. Those
must remain private and external. A clean source build can differ bytewise
from the recorded binaries; `tools/make_pin_overrides.py` makes that
difference explicit, and the runbook requires the no-RF and staged hardware
gates before such a build can replace the recorded deployment.

The handset must be rooted for the retained-state and filtered-observer
helpers used by the proven baseline. QCSuper itself is installed from its
pinned public package by `install-ue-aux`; credentials and site values remain
external. The handset path still uses an srsENB primer before OCUDU. Standalone OCUDU
reaches RF access, RRC setup, and core registration, but has not completed the
private PDU session on this handset. That is a known POC workaround, not a
solved product behavior.

The unchanged harness also needs broad passwordless sudo on each dedicated
lab role because it invokes a service-account-owned real-time helper as root.
The exact private-lab profile is supplied and the runbook marks this as a POC
security limitation; it is not suitable for shared or production hosts.
