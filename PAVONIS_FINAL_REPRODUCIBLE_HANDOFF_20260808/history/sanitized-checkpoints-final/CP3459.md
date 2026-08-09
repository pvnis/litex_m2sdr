# CP3459 - MIMO feasibility, no RF

## Verdict

**PARTIAL / STRUCTURAL BLOCK.** The deployed SDRs and the OCUDU/Soapy path
expose a genuine two-channel 2T2R surface. The proven legacy srsUE does not
support a valid two-layer NR PDSCH path and therefore cannot be used as the
rank-2 capability benchmark.

No RF was transmitted and no baseline, config, binary, kernel, module, FPGA,
or gateware state was changed.

## Evidence

### SDR and radio backend

Read-only deployed Soapy probes report `2 Rx, 2 Tx` for both the M2SDR and
bladeRF plugins. The sanitized result is in `sdr_channel_inventory.txt`.

The local M2SDR plugin returns two channels, accepts only `{0,1}` for a
dual-channel stream, enforces matching TX/RX channel counts, sets AD9361
`rx2tx2`, enables 2T2R timing, and calls `ad9361_set_no_ch_mode(..., 2)`.
OCUDU's Soapy validator permits two channels in one stream and enforces the
same AD9361 TX/RX count constraint.

### OCUDU rank selection

OCUDU has a real spatial-layer control path rather than only duplicate RF
streams. Its CSI configuration requests `CRI/RI/PMI/CQI`, enables the two-port
codebook and rank restriction, and the scheduler updates the recommended
downlink layer count from the UE's reported RI.

### Legacy srsUE limit

The srsUE NR RRC path assigns `max_mimo_layers = 1` both at initial carrier
setup and on reconfiguration. In the NR PDSCH implementation, antenna-port
mapping and demapping are explicitly unimplemented; encoding maps only
`x[0]` to `sf_symbols[0]`, while decoding performs single-port predecoding.

The existing focused PDSCH test was rebuilt with all local cores. Its rank-1
control passes. The same test at rank 2 fails at the expected implementation
boundary:

```text
rank 1: rc=0
rank 2: rc=255
Unmatched number of RE (756 != 1512)
```

This is a structural software limitation, not an RF scatter result.

## Interpretation

The hardware is not proven incapable of 2x2. The M2SDR, bladeRF, LiteX Soapy
plugin, and OCUDU all expose the required two-channel surface. What is closed
is the plan to use this legacy srsUE as the rank-2 benchmark.

The real handset is the remaining credible rank-2 UE. Before using it, an
isolated 10 MHz OCUDU two-channel profile must pass config validation and a
bounded gNB-only producer-health gate. A 2x2 10 MHz stream doubles aggregate
sample payload and may enter the same producer-pressure class as the failed
single-channel 20 MHz / 23.04 Msps branch. That gate belongs before any phone
attach attempt.

## Artifacts

- `CHECKPOINT_3459_MIMO_FEASIBILITY_NO_RF.md`
- `cp3459_summary.json`
- `sdr_channel_inventory.txt`
- `pdsch_rank_unit_results.txt`
- `source_evidence_sha256.txt`

The artifact manifest and report SHA-256 are generated after this report is
closed.
