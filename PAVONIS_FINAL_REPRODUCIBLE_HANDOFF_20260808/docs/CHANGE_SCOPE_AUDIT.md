# Change Scope Audit

Status: final, read-only audit completed 2026-08-08.

## Conclusion

No additional persistent change was found within the recorded and auditable
scope of the August 5 bring-up. The package bytes, the four frozen CP3121
controllers, and the eight hold-fork scripts are proven by SHA-256. On the RAN
host, the only session-era source-tree modification is the documented LiteX
Soapy streaming source restoration; the only other documented persistent
change is alignment of the repository build copy of the Soapy module with the
already deployed module. Both pre-change objects are preserved in the private
host backup directory.

This is a bounded statement, not a forensic claim about every byte on two
general-purpose hosts. There was no complete whole-host snapshot immediately
before the other agent's session. The audit therefore proves the package and
named repositories exactly, and finds no contrary evidence in process state,
source-tree timestamps, recorded backup inventory, or artifact hashes.

## Controller scope

The earlier session created two private records and eight additive hold-fork
scripts. Their exact hashes are in
`evidence/CP3634_FINAL_RUN_EVIDENCE.json`. The four CP3121 originals remain at
their recorded hashes. The historical supervisor package verifies with both
of its built-in verification entry points; its August 5 modifications are the
declared CP3630 demo-controller files and manifest update, not unexplained
drift.

The current work creates only this new final-handoff directory plus additive
CP3634-CP3637 reports in the campaign ledger. It does not edit the historical
supervisor package, the private session records, any live source repository,
or a prior checkpoint report. The clean-build exercise used an unpacked
temporary snapshot, not a live repository.

## RAN role scope

An exact repository-time audit beginning at 2026-08-05 00:00 found:

| Repository area | Session-era source files |
|---|---:|
| LiteX M2SDR Soapy source | 1 |
| OCUDU | 0 |
| srsRAN | 0 |
| qcore | 0 |

The one source file is the documented restoration. Its current SHA-256 is
`644e7858ee88d92791a495b7f4a24d762c9e819afb5337c0ef20d02e65ca4e76`.
The built and deployed Soapy modules both have SHA-256
`81b8cb2e079ed050a0056768cc349b0aaf31b25ae145eca359bd24934f4ff97c`.
The pre-restoration source and stale build module remain privately backed up
with hashes `888cfbda...4ce9` and `a15642a2...b0a2` respectively. They are not
in this sanitized package.

At audit time, no qcore, gNB, srsENB, or srsUE process was running. The two
detached validation keepers described in the sanitized session record had
been removed. No source/build/repository change was found on the UE role.

The release-closure audit accessed the UE role read-only to identify the
QCSuper version and hash two project-owned helper JARs. Exact copies of those
non-secret JARs were added to this release so a new service account does not
inherit hidden files from the historical one. No remote file, process,
package, handset setting, or RF state was changed by that provenance read.

## Hardware/software epoch

Both role hosts report Ubuntu 24.04, x86-64, kernel
`6.8.0-101-lowlatency`, and the performance governor. The M2SDR kernel module
has SHA-256
`3ded84bbc2ac7ff26a8192666c4083041ae54ebdfd808b48633ccedc91261ead`.
SoapySDR reports library `0.8.1`, API `0.8.0`, ABI `0.8`. The M2SDR gateware
identifies an M.2 build from 2026-06-15 12:00:51, PCIe Gen2 x1 with PTM
enabled, internal-XO clocking, and operational FPGA state.

The UE role reports bladeRF CLI `1.10.0`, libbladeRF `2.6.0`, firmware
`2.6.0`, FPGA `0.16.0` loaded from flash, and USB SuperSpeed. ADB `1.0.41`
reported exactly one authorized device. Serial values were removed before
packaging.

## Evidence

- `evidence/CP3634_FINAL_RUN_EVIDENCE.json`
- `evidence/final-session/SESSION_RECORD_SANITIZED.md`
- `harness/provenance/ORIGINAL_TEMPLATE_SHA256SUMS`
- `config/runtime-pin-map.toml`
- `docs/RUNTIME_DEPENDENCIES.md`
- package-wide `manifests/SHA256SUMS`
