# OCUDU CP3406 Reproduction Overlay

This directory contains the exact sanitized OCUDU Soapy TX source files used
to build the final Pavonis hardware-throughput binary. It is an additive
reproduction overlay for the history-free `ocudu.bundle`; it is not a new
upstream baseline or a claim that every diagnostic control belongs upstream.

The important final addition is the default-off
`OCUDU_SOAPY_TX_DEADLINE_WRITE_INSIDE_GUARD` policy. When enabled, a timed
write whose raw deadline is still positive but lies inside the reservation
guard gets one timeout bounded by that remaining raw lead. Already-expired
and ordinary writes retain their prior behavior. CP3412, CP3413, CP3415, and
CP3417 directly exercised the branch successfully; CP3410 and CP3424 prove
that it does not regress the accepted MCS19 path.

`tools/unpack_source.sh` applies this overlay automatically. For an audited
manual reconstruction, overlay these files after extracting the bundles:

```bash
rsync -a source/overlays/ocudu-cp3406/lib/ \
  <workspace>/ocudu/lib/
rsync -a source/overlays/ocudu-cp3406/tests/ \
  <workspace>/ocudu/tests/
```

Then configure and build OCUDU natively on the M2SDR host using the build
procedure in `docs/RUNBOOK.md`. Do not use a controller-host binary
on the RF host; the native ABI gate is mandatory.

Expected source hashes:

| Relative path | SHA-256 |
|---|---|
| `lib/radio/soapy/radio_soapy_tx_stream.cpp` | `50fee9094caedf2990faf0ac3771b9ca0ae9964d1d8904e72fbd336e2c16a0e7` |
| `lib/radio/soapy/radio_soapy_tx_stream.h` | `8d04d385ba3423c34dd652fcc80a904a04b32be501f6db5ddd1e9d1d47462142` |
| `lib/radio/soapy/radio_soapy_tx_deadline.h` | `79345305fc5bca4b3437e9453db32acf03da8b4a69478899fda89580e67feaf8` |
| `tests/unittests/radio/soapy/radio_soapy_tx_deadline_test.cpp` | `72629f6bf13d8753f461c0dee2870f8c075f98cc1cd0e72f3f4ee95e6532f9cd` |
| `tests/unittests/radio/CMakeLists.txt` | `a49a8f431161cfeb7b107560d49183983e3a5f453980304f71be0cce7070ece9` |
| `tests/unittests/radio/soapy/CMakeLists.txt` | `e18613459f5250a958d4086f2bc467b9182c53eb9ab55e44c6fde89b051dbc6f` |

The two test `CMakeLists.txt` files in this overlay register only the focused
deadline unit test against the cleaned submission snapshot. Standards-default
behavior is unchanged while the env flag is absent or zero.

The clean submission bundle plus this overlay was independently configured
and built natively on the M2SDR host with 16 jobs. The gNB target linked with
SHA-256
`7fc6e493babd9b6b1a25678698dbc61a808f384a4a27925d49904af2e68013a6`,
and `radio_soapy_tx_deadline_test` passed `1/1`. This build hash is not expected
to equal the deployed campaign binary because the submission bundle is the
cleaned review snapshot; RF deployment still follows the staged revalidation
gates.
