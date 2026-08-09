# CP3637: Final Reproducible Release

Date: 2026-08-08  
RF activity: none  
Remote mutation: none

## Verdict

**PASS.** `PAVONIS_FINAL_REPRODUCIBLE_HANDOFF_20260808` is the final
supervisor-facing release. It supersedes CP3636's packaging claim because it
closes three implicit dependencies that CP3636 did not yet enforce: the
ordered ZMQ/bounded-hardware/phone ladder, the CP3424 bounded-controller
closure, and the active handset helper/runtime prerequisites.

The release is account-neutral. A clean operator creates dedicated `ran` and
`ue` service accounts and supplies private site/subscriber material outside
the package. No controller or document depends on a historical personal
account, host name, handset serial, credential, subscriber identity, or NAS
payload.

## Evidence

- `pavonis-final.sh run` verifies and materializes the package, requires a
  no-RF ZMQ `PING_OK` receipt, deploys and checks both roles, requires a
  CP3424-class M2SDR/bladeRF data receipt, and only then starts the selected
  handset profile.
- Each stage has a distinct fresh identifier and summary. The final
  `STAGED_SUMMARY.json` is written only when all three stage receipts pass.
- The phone adapter uses the proven bounded hold to test private gateway,
  public IP, DNS, and HTTPS over the handset NR interface before automatic
  release.
- The runtime-layout installer is collision-safe and restartable for exact
  matching bytes. The remote doctor verifies its mode-600 SHA-256 manifest,
  RF devices, performance governor, real-time permission, QCSuper version,
  helper JAR hashes, one authorized handset, and root access.
- The dedicated-host sudo requirement is explicit. Because the preserved
  harness runs a service-account-owned helper as root, the supplied lab profile
  is intentionally broad and is reported as a POC security limitation rather
  than mislabeled as least privilege.
- The no-network verifier passes privacy scanning, all bundle verification,
  shell/Python parsing, fixed-point materialization, closed RF gates, bounded
  validate-only execution, role-specific deploy membership, private-file mode
  checks, runtime-layout installation twice, public-proof hold/release, and
  final receipt creation.
- A fresh unpack reproduced all four recorded source revisions and both OCUDU
  overlay hashes. qcore passed clippy with warnings denied and a locked
  release build.
- The full native RAN build is an honest environment-limited partial on the
  packaging workstation: the host lacked the SoapySDR development package
  and noninteractive package-install privilege. It is not reported as a
  clean native RAN build pass; the runbook makes that dependency and native
  build gate mandatory.

## Change-Scope Result

The bounded CP3634 audit found no additional session-era source modification
in OCUDU, srsRAN, or qcore. The only RAN source modification was the documented
LiteX Soapy source restoration, SHA-256
`644e7858ee88d92791a495b7f4a24d762c9e819afb5337c0ef20d02e65ca4e76`.
The only other documented persistent change was alignment of the repository
build-copy Soapy module with the deployed module, SHA-256
`81b8cb2e079ed050a0056768cc349b0aaf31b25ae145eca359bd24934f4ff97c`.
No live source repository, historical report, RF configuration, remote file,
remote process, package, handset setting, or hardware state was changed by
the release-closure work.

## Artifacts

- `README.md` and `DEMO_TLDR.md`: human entry points.
- `docs/REPORT_FINAL.md`: final technical report.
- `docs/RUNBOOK.md`: cold-lab build, setup, staged run, and teardown.
- `AGENT_HANDOFF.md`: compact future-agent state.
- `docs/CHANGE_SCOPE_AUDIT.md`: bounded answer to what else changed.
- `docs/FILE_CHANGE_AND_SANITIZATION_INVENTORY.md`: clean-state inventory.
- `evidence/CP3634_FINAL_RUN_EVIDENCE.json`: sanitized final run evidence,
  SHA-256
  `1a860a20f5b05faed1a35ab5319a1637069c4dd875f5537790dc605e085990ea`.
- `evidence/final-session/SESSION_RECORD_SANITIZED.md`: sanitized session
  record, SHA-256
  `1e6767fc7aba4091f9ee2891c74b688a2d0d5c8459f664b31cf0535f42a69b74`.
- `manifests/SHA256SUMS`: complete package membership and byte manifest,
  excluding only itself. Its detached SHA-256 is recorded in the external
  CP3637 campaign checkpoint.

The release contains no RF run generated during packaging. The authoritative
performance evidence remains the two valid CP3631-era August 5 demonstrations
bound by `evidence/CP3634_FINAL_RUN_EVIDENCE.json`.
