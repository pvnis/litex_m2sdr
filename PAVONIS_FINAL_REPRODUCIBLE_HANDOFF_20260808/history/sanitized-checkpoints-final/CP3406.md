# CP3406 - Deadline inside-guard no-RF gate

## Verdict

**PASS.** A minimal, default-off source policy now attempts future-deadline
writes that arrive inside the userspace reservation instead of abandoning them
before `writeStream`. The focused tests, native build, deployment, binary
proof, and frozen MCS19 config gate all pass. No RF was used.

## Implementation

`OCUDU_SOAPY_TX_DEADLINE_WRITE_INSIDE_GUARD=1` enables the new path. It is
false by default and read once during stream construction.

The deadline selector has three explicit outcomes:

1. Outside the guard: retain the existing guarded timeout.
2. Inside the guard with positive raw lead: attempt immediately, bounded by
   the remaining raw lead.
3. At or past the hardware deadline: retain the existing expired/break path.

The runtime hot path performs no environment lookup. Metadata-only summary
counters record attempts, successes, timeouts, and recovered samples.

## Verification

Five focused boundary tests pass on both the controller and native build host:

- disabled behavior preserves CP3405's guard expiry;
- the positive 91 us CP3405 case becomes an inside-guard attempt;
- zero/past lead remains expired;
- normal outside-guard selection is identical enabled or disabled;
- the maximum timeout still caps the write budget.

The native `gnb` rebuild is warning-clean and the focused ctest passes. The
first combined proof returned 141 only because `strings | grep -q` ran under
`pipefail`; compilation, linking, ctest, and binary hashing had already
passed. A pipeline-safe full-consumption check finds two feature strings and
returns zero.

The staged source was ported onto the exact native source state rather than
overwriting it from the different controller checkout. The previous source
files and qualified binary are backed up by hash.

The dedicated launcher:

- pins the new gNB and existing MCS19 config generator;
- enables only inside-guard handling in addition to the CP3405 shape;
- keeps guard 100 us, lead 8.5 ms, all timing offsets, PRACH candidates,
  MCS19, gains, and logging frozen;
- regenerates the exact prior MCS19 config hash.

## Next

Run the same 420-second transmit-only gate. Acceptance requires:

- live readback `inside_guard=1`;
- at least one successful inside-guard attempt if the boundary is exercised;
- zero deadline breaks/timeouts and sample deficits;
- zero partial/hard, driver-late/underflow/acquire, release-late/dedup, and
  downlink-late errors.

## Artifacts

- `cp3406_summary.json`
  - SHA-256
    `5808bb47bbb45d7c123b0ed9649e661328b2c0c21d6a798364c3ea7e680af2c5`
- `cp3406_deadline_inside_guard_mcs19_launcher.sh`
  - SHA-256
    `7a8983b386956191a6e6c6b773c553f299266a9c4665c9951e149fcfb28919e7`
- `native_patch/radio_soapy_tx_deadline.h`
  - SHA-256
    `79345305fc5bca4b3437e9453db32acf03da8b4a69478899fda89580e67feaf8`
- `native_patch/radio_soapy_tx_deadline_test.cpp`
  - SHA-256
    `72629f6bf13d8753f461c0dee2870f8c075f98cc1cd0e72f3f4ee95e6532f9cd`
- `native_patch/radio_soapy_tx_stream.cpp`
  - SHA-256
    `50fee9094caedf2990faf0ac3771b9ca0ae9964d1d8904e72fbd336e2c16a0e7`
- `native_patch/radio_soapy_tx_stream.h`
  - SHA-256
    `8d04d385ba3423c34dd652fcc80a904a04b32be501f6db5ddd1e9d1d47462142`
- Native gNB
  - SHA-256
    `cf33689670b6d9e8bace5102a930fbb00bdab46b9cfa1aad2dbba6c10a4c1d78`

No credentials, private identities, payload bytes, NAS bytes, or machine
identifiers are included.
