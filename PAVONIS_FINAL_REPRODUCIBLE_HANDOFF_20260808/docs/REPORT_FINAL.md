# Pavonis: A Reproducible Private 5G NR SA Link on LiteX M2SDR

Document status: **FINAL**  
Evidence cutoff: 2026-08-08  
Submission package: `PAVONIS_FINAL_REPRODUCIBLE_HANDOFF_20260808`

## Executive summary

Pavonis is a private, single-operator 5G NR standalone laboratory chain. It
combines a project-owned core, OCUDU gNB, LiteX M2SDR radio, project-owned test
subscriber, real Android handset, and a controlled over-the-air path. The
project goal was not merely to radiate an NR signal or register a handset. It
was to complete the protocol ladder and carry real bidirectional user data.

That goal is complete. The final 15 MHz MCS17 run attached the handset,
activated its private PDU session, passed private bidirectional traffic, used
public DNS, reached the public internet, sustained 19.4 Mbit/s downlink in the
in-lab 45-second instrument, downloaded public HTTPS data at 21.0 Mbit/s, and
completed an independent application speed test at 25.1 Mbit/s down and 1.61
Mbit/s up. The preceding 10 MHz control independently passed the same chain at
6.43/0.092 Mbit/s in-network and 7.9/0.16 Mbit/s over public HTTPS.

The result is a successful proof of concept, not a product-ready base station.
The phone path still uses an srsENB primer before OCUDU, several LiteX timing
and power compatibility controls remain explicit workarounds, uplink is much
slower than downlink, and the fast downlink varies with HARQ feedback latency.
Those limits are documented rather than hidden.

The primary sanitized proof artifact is
`evidence/CP3634_FINAL_RUN_EVIDENCE.json`, SHA-256
`1a860a20f5b05faed1a35ab5319a1637069c4dd875f5537790dc605e085990ea`.
The longer sanitized session record is
`evidence/final-session/SESSION_RECORD_SANITIZED.md`, SHA-256
`1e6767fc7aba4091f9ee2891c74b688a2d0d5c8459f664b31cf0535f42a69b74`.

## Goal and chain under test

The deployed chain is:

```text
real handset
    <--- private OTA NR Band 3 --->
LiteX M2SDR + srsENB primer, then LiteX M2SDR + OCUDU
    <--- N2/N3 --->
qcore + private UE tunnel + N6/NAT
    <--- controlled lab uplink --->
public DNS and internet
```

The controller reaches two role hosts by SSH. The `ran` role owns qcore,
srsENB, OCUDU, M2SDR, and data-plane routing. The `ue` role owns ADB control of
the handset and the optional bladeRF/srsUE regression path. Private
subscriber credentials are loaded from a mode-600 role file that is not part
of this package. No commercial network or third-party subscriber is involved.

The pass criterion is target-era session activation after OCUDU starts,
application data in both directions, endpoint byte growth, and clean teardown.
ADB is control-only and does not carry the measured payload.

## Final measurements

| Metric | 10 MHz slow profile | 15 MHz MCS17 fast profile |
|---|---:|---:|
| Valid run identifier | `20260805T120946Z` | `20260805T123403Z` |
| Attach | 20.637 s | pass |
| In-network sustained duration | established instrument | 45 s |
| In-network DL | 804,015 B/s = 6.43 Mbit/s | 2,425,101 B/s = 19.40 Mbit/s |
| In-network UL | 11,478 B/s = 0.092 Mbit/s | 54,468 B/s = 0.436 Mbit/s |
| Public HTTPS DL | 988,171 B/s = 7.91 Mbit/s | 2,622,826 B/s = 20.98 Mbit/s |
| Public HTTPS UL | 19,861 B/s = 0.159 Mbit/s | not separately repeated |
| Application speed test | control phase starved | 25.1 DL / 1.61 UL Mbit/s |
| Private gateway / public IP / DNS | 3/3 each | 3/3 each |
| Typical RTT in this run | 76-80 ms | 96-105 ms |

The public and in-network downlink values agree closely enough to exclude the
core's external route as the slow profile's bottleneck. The independent speed
test is higher than the 45-second instrument but lies in the previously
observed fast-profile upper band. It should be reported as a measured result,
not a guaranteed service rate.

The old `CP2880_DATA_PROBE_TRANSPORT core_rc=1` marker remains unresolved in
both shapes. It is not credited as a data failure because independent endpoint
counters, HTTP, ping, DNS, public HTTPS, and the application speed test all
pass. The marker is retained as a harness defect to fix.

## Campaign in brief

The campaign proceeded through more than three thousand numeric checkpoints.
The complete sanitized archive is separate under
`history/sanitized-checkpoints/`. The important experiment groups were:

- established known-frequency TX tones and bidirectional LiteX/bladeRF RF
  baselines before interpreting protocol failures;
- separated weak-link hypotheses from frame-scale coherence failures by
  replaying the exact gNB payload through continuous and timed helpers;
- proved that full-rate self-RX append, hot-path sample scans, and growing-log
  pollers perturb live PRACH and made those diagnostics invalid for ladder
  judgment;
- repaired CPU governor/EPP/real-time state after reboot and added mandatory
  host preflight;
- localized the original downlink lottery to descriptor churn caused by a
  timed/untimed partial-buffer boundary and deployed the CP2074B contiguous
  remainder handling;
- built bounded stage-1 PRACH search and stop-on-accept, then reached the first
  live Msg1, RAR, Msg3, security, registration, and user-data milestones;
- established a sustained 45-second phone benchmark beside the one-shot
  correctness harness;
- disproved RF power, MCS ceiling, and antennas as the original 10 MHz
  throughput limit;
- repaired the 15 MHz SSB/acquisition and uplink timing conversion;
- retired the 512-entry synthetic follow-up grant budget after proving native
  SR/BSR scheduling;
- showed that 20 MHz/23.04 Msps exceeds current producer headroom;
- mapped one-layer, 2T2R, bandwidth, modulation, and transmit-lead controls one
  at a time;
- traced the remaining downlink spread to HARQ feedback latency exposed by the
  timed-transmit lead;
- corrected the final hold fork, cleared a conflicting validation keeper,
  restored the recorded Soapy source provenance, and repeated both slow and
  fast phone paths through public internet traffic.

Negative and void experiments are part of the result. A run with no Msg1 does
not test RAR; a run with no session does not test throughput; a run perturbed
by full append does not test the detector. Preserving those distinctions
prevented plausible but false hardware and protocol conclusions.

## Finding 1: transport semantics can corrupt correct samples

The central engineering lesson was that correct host-visible samples do not
guarantee a correct over-the-air waveform. Early live gNB captures were weak
or incoherent at the UE even while pre-DMA and digital guards decoded. The
same exact payload became strongly decodable when replayed through a simple
continuous helper. Timed helper variants also worked, proving that timing
metadata itself was not inherently defective.

The discriminator was the descriptor sequence at the boundary where a timed
write returned partially and its unwritten remainder became an untimed
continuation. In a bad run, short non-EOB rows and repeated timestamps formed
a dense churn regime. A minimal LiteX Soapy change retained contiguous
untimed remainder buffers instead of repeatedly releasing and re-anchoring
them. The bounded descriptor gate then required zero short non-EOB rows, zero
late rows, zero deduplication, and no duplicate timed timestamps. Once the
mechanism was exercised, scheduled downlink supply returned and stayed good
through the later ladder.

The final package calls this CP2074B handling a workaround because the full
Soapy/LiteX hardware-time and ring-backpressure contract has not been derived
or upstreamed. Nevertheless, the evidence is causal: exact payload replay
closed the upper stack, descriptor tracing named the defective boundary, the
minimal behavior change removed the signature, and the protocol ladder
advanced immediately afterward.

## Finding 2: instrumentation is part of the real-time system

Several apparently read-only diagnostics changed the result. Full-rate
same-device RX append repeatedly converted accepted-PRACH shapes into
zero-detection runs. Buffering did not save a lower-PHY manifest when the
producer still scanned every sample. A process polling the entire growing gNB
log every 10 ms also reproduced zero acceptance. State-dump logging changed a
same-run post-DMA guard. These were not statistical suspicions: removing one
observer at a time restored the previous class.

The durable rule is therefore stronger than “log asynchronously.” Live PHY
instrumentation must be metadata-only or triggered on a bounded event/window;
no per-sample scan or formatting is permitted on the producer path. Arm
events occur in process, once per run. Raw windows are small, late, and flushed
at exit. Capped traces must be checked for coverage before a negative is
interpreted.

This rule matters beyond SDR work. A producer with millisecond deadlines can
be invalidated by allocation, string formatting, cache pressure, or repeated
file scans even when the diagnostic thread is “separate.” Instrument cost has
to be part of the experiment definition and health gate.

## Finding 3: performance limits came from different layers

The first sustained phone result, around 8.3 Mbit/s down at 10 MHz, was already
near the one-layer 52-PRB MCS9 allocation bound. Increasing only an MCS ceiling
did nothing because the deployed cap and channel width still constrained the
shape. A controlled 2T2R test later reached 16.36 Mbit/s and proved genuine
multi-layer capacity at 10 MHz. Converting the cell coherently to 15 MHz raised
one-layer throughput by about 51%, nearly linear with bandwidth. Raising the
active downlink shape to MCS17 then produced the 23-25 Mbit/s class.

The next apparent ceiling was not modulation or RF. The M2SDR timed-transmit
lead was visible to the MAC as HARQ feedback latency. A shorter lead increased
HARQ process turnover and produced a 31.16 Mbit/s mechanism result, but that
run had transmit timeouts, deadline breaks, and abandoned samples. It is not a
promotable operating point. The final fast profile retains the strict-clean
20 ms lead and accepts its run-to-run throughput spread.

Uplink is a separate limitation. Retiring the 512-entry synthetic grant budget
proved native SR/BSR works, but phone uplink remained small and variable. The
remaining primer one-shot grant, PUCCH reliability, TDD opportunity ratio,
and marginal uplink PHY quality are still entangled. The 1.61 Mbit/s
application result is the strongest current user-facing result; it does not
close the uplink root cause.

## Required controls: accommodation, workaround, dead scaffolding

### Legitimate private-lab accommodations

- private cell/core profile and role-hashed credentials;
- M2SDR channel routing and measured RF calibration;
- host-local N3 and controlled N6/NAT;
- reversible private routes and namespaces;
- host-state, SDR, USB, thermal, and real-time preflight;
- fresh run identifiers, mode-600 private files, and fail-closed cleanup.

These are properties of the authorized testbed, not defects to remove.

### Known workarounds that remain required

| Control | Defect or unknown it masks |
|---|---|
| srsENB primer before OCUDU | Standalone OCUDU does not complete this handset's retained private-session lifecycle. |
| Primer duplicate reactive downlink | ACK/HARQ attribution versus higher-layer delivery remains incomplete. |
| Primer post-setup-ACK one-shot UL grant | Native setup completion without this final grant has not been isolated. |
| CP2074B timed remainder, timed cadence, and per-shape lead | Full LiteX/Soapy hardware-time and producer-backpressure contract is not modeled. |
| Post-setup attenuation reassert | `setupStream()` resets active attenuation to 20 dB. |
| SI-PDSCH power-sign compatibility | FAPI/PDSCH sign handling otherwise creates a scheduled-SI handicap. |
| PRACH stage-1, TA correction, dynamic non-PRACH offset | Large sample-time presentation terms are empirical per sample rate. |
| Independent timed non-PRACH bursts | Removing them preserves access through Msg3 but collapses later PUSCH. |
| Fresh phone epoch and SIM cycle | Device-side retained 5GS state survives RAN/core teardown. |
| Data roaming enabled | The private cell is presented as roaming by this handset. |
| Dedicated-host broad sudo profile | The preserved harness runs a user-owned real-time helper as root; least privilege requires root-owned wrappers. |

The package is intentionally blunt: the proof works with duplicate reactive
primer downlink and the remaining one-shot uplink grant. Those are scaffolds
masking unresolved primer behavior, not product features.

### Retired scaffolding

The following are not part of a successful roll: forced AL4, the 512-entry
synthetic follow-up grant budget, the UE-side static PUSCH timestamp fixture,
broad stage-1 sweeps, full-rate append, per-sample hot-path traces, uncapped
SI/RAR/FAPI/EVM traces, growing-log polling, stale wrapper guards, and raw
QCSuper output as a success condition.

## Reproducibility and packaging

The final package removes the old personal-account and site-address coupling.
`config/site.toml.example` defines `ran` and `ue` roles. The three historical
transport helper filenames remain only as a compatibility API; their code is
key-first, host-key-checking, and site-config driven. One materializer
substitutes role home, controller path, N2/S1 addresses, and interfaces, then
recomputes every included dependency SHA to a fixed point. Both slow and fast
RF gates were tested closed with return code 2 before any SSH.

The first release audit found and corrected a meaningful packaging defect:
the documented staged order existed only as prose while the top-level `run`
command could enter the phone stage directly. The released command now
enforces three machine-validated receipts in order: a no-RF ZMQ attach and
user-plane ping; the CP3424-confirmed M2SDR/bladeRF 45-second bidirectional
gate; and the selected handset profile. It writes a final receipt only when
all three summaries pass. Distinct per-stage stamps prevent one stage from
overwriting another's artifacts.

The active fast chain's previously implicit handset dependencies are also
explicit. The package carries the two small project-owned helper JARs at their
proven hashes and installs QCSuper `2.1.0.post4` plus exact dependencies into
the selected UE service-account home. Preflight requires one authorized
rooted handset. No file or command depends on the historical operator account.

Fresh-account runtime aliases are created by one collision-safe installer,
not by an undocumented sequence of copies. Exact matching installs are
restartable, mismatches fail closed, and the remote doctor verifies the
resulting per-role SHA-256 manifest before the staged command can transmit.

Four one-commit, privacy-clean Git bundles carry source snapshots. The OCUDU
runtime overlay and review patch series are separate and hashed. A clean build
can differ bytewise from the deployed campaign binaries because of compiler,
ABI, and build-path provenance. That is handled explicitly: the operator
generates SHA override entries for the rebuilt files and must pass ZMQ,
bounded hardware, then phone OTA in order. No script silently accepts a hash
mismatch.

Package acceptance is stronger than syntax checking: a no-network self-test
materializes and re-pins the full graph, executes the recovered bounded
controller's validate-only path, proves its RF gate closes, proves both phone
RF gates close, checks deploy membership, creates private input skeletons,
executes a mocked streaming hold/public-egress/release path, and validates the
final receipt combiner. On the packaging workstation, qcore also
passed warnings-denied clippy and built in release mode. A complete native RAN
build was not claimed there: OCUDU correctly stopped when that workstation
lacked the SoapySDR development package and could not install it without
operator privilege. The clean RF hosts must satisfy the documented native
dependency gate before reproduction.

The CP3638 completion audit found and corrected two release-engineering gaps
after the first final label. First, the secondary archive stopped at CP3424
and its older incremental copies had undergone portability transformations
after some per-file manifests were written. The authoritative
`history/sanitized-checkpoints-final/` corpus is now rebuilt atomically from
all 3,587 available source reports through CP3637, preserves every embedded
SHA-256 multiset, has zero residual privacy-policy matches, and explicitly
records that CP3633 has no report. Second, the srsRAN build command now uses
the actual `ENABLE_SOAPYSDR` option and explicitly builds the
`srsran_rf_soapy` target instead of relying on its default-enabled dependency.
Both corrections are covered by the package verifier.

The exact cold-lab process, service-account layout, host packages, native
build, private file boundary, runtime aliases, RF commands, and teardown are
in `docs/RUNBOOK.md`. `docs/RUNTIME_DEPENDENCIES.md` states the exact
packaged/private/site-owned boundary. The change-scope result is in
`docs/CHANGE_SCOPE_AUDIT.md`. Their final hashes are recorded by the
package-wide manifest generated after this report.

## Limitations and roadmap

The solved boundary is one handset, one private cell, one lab path, primer
assisted, with bidirectional IP and public egress. The following remain open:

1. **Primer-free handset session.** Repair the OCUDU/core/handset retained
   session lifecycle without non-compliant reject causes or primer behavior.
2. **Primer crutch removal.** Explain and remove duplicate reactive downlink
   and the one post-ACK uplink grant after native ACK/SR association is fixed.
3. **Uplink throughput.** Separate PUCCH/SR/BSR quality, grant cadence, TDD
   opportunity, PHY SINR/EVM, and transport timing. Report a causal limiter,
   not only a small number.
4. **Strict-clean low-latency downlink.** Account for every HARQ process and
   producer opportunity, then reduce lead without timeout, deadline-break, or
   abandoned-sample debt.
5. **15 MHz rank/MIMO.** The prior 2T2R branch reached access but did not close
   connected rank delivery. Resume only behind a producer gate.
6. **20 MHz producer headroom.** The 23.04 Msps path is not currently
   real-time sustainable. This is a producer/DMA problem, not an antenna or
   power sweep.
7. **Product-grade scope.** Multi-UE, long-duration soak, mobility, recovery,
   coexistence, standards conformance, and security review are future work.
8. **Privilege hardening.** Replace the dedicated-host broad sudo profile with
   audited root-owned fixed-function helpers and a narrow command policy.

Two performance-envelope benchmarks should remain in every future report:
the real-world wireless handset result and a controllable high-SNR regression
result. The former measures service reality; the latter isolates capability.
Neither should be substituted for the other.

## Final conclusion

The LiteX M2SDR path did not fail the project goal. It now carries a real
handset from OTA access through a private 5G session to bidirectional user data
and public internet, with a measured 25.1/1.61 Mbit/s best demonstrated
application result. The evidence also explains why earlier runs looked
random: timed-write remainder semantics, observer effects, stale host state,
uplink sample-time presentation, active bandwidth/MCS limits, and HARQ
feedback latency each created distinct failure classes.

The POC is reproducible in the engineering sense required for handoff:
source, controllers, non-secret configs, build procedure, role-based remote
execution, actual ordered stage controllers, hash provenance, failure gates,
handset auxiliary setup, and sanitized evidence are in one verified package.
It is not represented as cleaner than it is. Primer
behavior, empirical timing controls, uplink performance, and producer
headroom remain explicit work rather than hidden assumptions.
