# File Change and Sanitization Inventory

This document is for future sanitization/release work. It describes what this
final package adds relative to clean upstream repositories and what was copied
or transformed from campaign state.

## Clean upstream repositories

The four Git bundles are history-free submission snapshots. Their branch heads
and hashes are documented in `RUNBOOK.md`; their bundle hashes are in
`manifests/SHA256SUMS`. They contain no private subscriber fixture or raw
capture. The OCUDU CP3406 overlay is deliberately separate and default-off.

`source/patches/` contains reviewable minimal hardening series reconstructed
from the campaign. It is not applied automatically by the final phone runner;
the one-commit snapshots already carry the intended review source. The patches
exist to make upstream review and future re-sanitization understandable.

## Added final-package files

| Path | Purpose |
|---|---|
| `README.md` | Human TLDR and entry point |
| `AGENT_HANDOFF.md` | Compact machine/agent state |
| `pavonis-final.sh` | Fail-closed verify/materialize/doctor/deploy/run CLI |
| `config/site.toml.example` | Account/site template; no live values |
| `config/runtime-pin-map.toml` | Recorded runtime artifact hashes |
| `config/sudoers.pavonis-lab.example` | Explicit dedicated-host POC privilege profile |
| `docs/REPORT_FINAL.md` | Final supervisor report |
| `docs/RUNBOOK.md` | Cold-lab reproduction procedure |
| `docs/RUNTIME_DEPENDENCIES.md` | Included/private/site-owned runtime boundary |
| `docs/CHANGE_SCOPE_AUDIT.md` | Bounded change-scope conclusion |
| `docs/COMPLETION_AUDIT.md` | Requirement-by-requirement final acceptance matrix |
| `tools/remote_common.py` | Key-first SSH/SCP transport by role |
| `tools/materialize_harness.py` | Site substitution and dependency re-pin |
| `tools/deploy_harness.py` | Hash-verified helper/config deployment |
| `tools/doctor.py` | No-RF controller/role preflight |
| `tools/privacy_scan.py` | Fail-closed content/path scan |
| `tools/verify_package.py` | Package, bundle, privacy, and syntax gate |
| `tools/unpack_source.sh` | Exact one-commit source extraction + overlay |
| `tools/build_from_source.sh` | Parallel native source build and tests |
| `tools/make_pin_overrides.py` | Explicit clean-build runtime hash mapping |
| `tools/install_runtime_layout.sh` | Collision-safe fresh-account runtime aliases |
| `tools/prepare_private.py` | Mode-600 ZMQ private-role skeleton creation |
| `tools/run_zmq_gate.sh` | No-RF attach and user-plane receipt |
| `tools/run_bounded_gate.py` | CP3424 M2SDR/bladeRF receipt adapter |
| `tools/run_phone_gate.py` | Slow/fast handset receipt adapter |
| `tools/write_staged_summary.py` | Ordered final receipt writer |
| `tools/install_ue_aux.py` | Pinned QCSuper/JAR setup on the UE role |
| `runtime/ue-aux/*.jar` | Exact project-owned handset state helpers |
| `harness/templates/pavonis_phone_public_egress_remote.sh` | NR-bound public-IP/DNS/HTTPS proof |
| `evidence/CP3634_FINAL_RUN_EVIDENCE.json` | Final two-run sanitized result |
| `history/sanitized-checkpoints-final/` | Atomic, source-bound sanitized checkpoint corpus through CP3637 |

The numeric reports under `checkpoints/` are additive closure records. The
large `history/sanitized-checkpoints-final/` tree is authoritative secondary
evidence and should not be used as an operator entry point. The old archive
path contains only a redirect; original incremental publication artifacts
remain outside this package.

## Harness provenance and transformations

`harness/templates/` contains 100 executable/controller helper files plus the
four exact non-secret primer configs. Sources are:

1. the prior 74-file reference controller;
2. the corrected four-file CP3121 hold chain;
3. the corrected four-file CP3612/CP3586 hold chain;
4. the 15 MHz primer/OCUDU/wrapper dependencies;
5. the three sustained benchmark helpers;
6. the CP3422/CP3424 robust bounded srsUE/bladeRF controller closure;
7. the exact primer `enb/rr/sib/rb` configs.

Original pre-sanitization hashes are in
`harness/provenance/ORIGINAL_TEMPLATE_SHA256SUMS`. The import then made only
portability/privacy changes:

- personal home paths became `@REMOTE_HOME@` or controller placeholders;
- physical host aliases became `ran` and `ue` roles;
- site addresses and interfaces became site placeholders;
- the embedded handset serial default became a required executor-discovered
  value;
- the three password/account-bound transport helpers were replaced by the
  generic transport while retaining compatibility filenames;
- the fast baseline check now points at a generated portable manifest;
- historic host-oriented helper filenames were role-normalized;
- all dependent SHA literals are regenerated after materialization.

The two UE helper JARs were copied byte-for-byte from the final proven role
host after a read-only hash audit. They contain project-owned helper classes,
not credentials or captures. QCSuper is not vendored; the installer pins its
public package and every direct dependency version used by CP2741.

No RF constants, scheduler settings, protocol payload, threshold, gain,
attenuation, MCS, bandwidth, timing compensation, or teardown semantics were
changed by sanitization. Runtime artifact hashes can change only through an
explicit private-site `runtime.pin_overrides` table.

## Files intentionally absent

- private subscriber/core role files and any real SIM credential;
- private keys, password files, SSH known-host contents, or account inventory;
- handset serial, subscriber identity, temporary identity, or NAS bytes;
- raw logs, RF IQ, diagnostics, packet captures, `.dlf`, run directories,
  archives, and build trees;
- binaries tied to one host ABI/kernel. Sources and recorded hashes are
  included instead.

## Re-sanitization checklist

1. Start from clean upstream plus the listed source snapshots/patches.
2. Re-import only the paths listed above; never bulk-copy an artifact root.
3. Reapply role/path/address placeholders before adding files to a manifest.
4. Reject every credential, identity, raw payload, raw capture, and run log.
5. Run `tools/privacy_scan.py` before generating `manifests/SHA256SUMS`.
6. Generate the final manifest last and verify membership as well as hashes.
7. Materialize with an example mode-600 site file and prove both RF gates
   close before publishing.
