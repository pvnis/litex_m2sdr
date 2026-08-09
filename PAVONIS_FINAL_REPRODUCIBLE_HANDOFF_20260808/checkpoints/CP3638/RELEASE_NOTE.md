# CP3638: Completion Audit and Final Reseal

Date: 2026-08-08  
RF activity: none  
Remote mutation: none

## Verdict

**PASS after correction.** A requirement-by-requirement audit found two gaps
behind CP3637's green package verifier. Both are corrected before this reseal:

1. the secondary checkpoint archive stopped at CP3424 and its older layered
   copies did not all match their pre-portability manifests;
2. the srsRAN build command relied on the default-enabled Soapy option while
   passing the wrong explicit option name.

## Corrections

`history/sanitized-checkpoints-final/` is now an atomic rebuild from every
available original report through CP3637: 3,587 reports spanning 3,586 IDs.
CP0072's two reports are retained; CP3633 is explicitly absent because it has
artifacts but no report. Generation requires zero residual sensitive matches,
the package privacy policy, exact output membership, and equality of each
source/output embedded SHA-256 multiset. Its manifest SHA-256 is
`1603f317ca0922e61b5d9c0946fb9d8d6f14ae2a1b5fcaef9e41867d078dc3dd`.

The old package-local incremental copies were removed to avoid two competing
archives; their original campaign artifacts and prior supervisor package were
not changed. The historical path now contains only a redirect.

`tools/build_from_source.sh` now passes `ENABLE_SOAPYSDR=ON` and
`ENABLE_RF_PLUGINS=ON` to srsRAN and explicitly builds
`srsran_rf_soapy`. The self-test asserts those exact controls. Source audit
confirms `srsran_rf_object` depends on enabled dynamic RF plugins, so the
explicit target and normal srsENB dependency agree.

## Completion Evidence

`docs/COMPLETION_AUDIT.md` maps every explicit handoff requirement to its
authoritative evidence and caveat. `tools/verify_package.py` now verifies the
authoritative history manifest and its fixed coverage invariant in addition
to package membership, privacy, source bundles, syntax, controller
materialization, closed RF gates, deploy membership, runtime layout, phone
auxiliary setup, public-proof release behavior, and staged receipts.

The bounded change-scope result remains unchanged: no unexplained persistent
change was found in the audited package/repositories/recorded host objects.
The known LiteX Soapy source restoration and module alignment remain named and
hash-bound. Because no whole-host pre-session image exists, this remains a
bounded engineering audit rather than a forensic claim about every host byte.

No RF, remote command, source-repository edit, package install, process change,
or handset setting was performed for CP3638. The native RAN build on the
packaging workstation remains an honest environment-limited partial due to
missing SoapySDR development headers; the cold-lab destination build is still
mandatory.

## Primary Artifacts

- `README.md` and `DEMO_TLDR.md`;
- `docs/REPORT_FINAL.md`;
- `docs/RUNBOOK.md`;
- `docs/COMPLETION_AUDIT.md`;
- `docs/CHANGE_SCOPE_AUDIT.md`;
- `docs/FILE_CHANGE_AND_SANITIZATION_INVENTORY.md`;
- `AGENT_HANDOFF.md`;
- `history/sanitized-checkpoints-final/HASH_MANIFEST.tsv`;
- `manifests/SHA256SUMS`.

The detached package-manifest hash, report hash, checkpoint report/summary
hashes, final file count, and size are recorded by the external CP3638 ledger
after the package-wide manifest is generated.
