# OCUDU + LiteX-M2SDR 3.5 GHz SSB observability baseline

This is a reproducible external SSB observability baseline, not a full base-station baseline.

## Validated path

OCUDU `radio_ssb` transmits through the LiteX-M2SDR Soapy path, M2SDR/AD9361 RF, an external OTA/coupled path, bladeRF RX, and bounded offline PSS detection.

## Known-good debug profile

Copied OCUDU overlay files are stored under:

```text
litex_m2sdr/software/validation/ocudu/ocudu_overlay/apps/examples/phy/
```

The useful profile is:

```text
profile: m2sdr_35g_10MHz
DL center: 3500.00 MHz
SSB center: 3497.15 MHz
SSB offset: -2.85 MHz
sample rate: 11.52 Msps
SCS: 15 kHz
bandwidth: 52 RB
PCI: 1
baseband backoff: -b 6
```

Required M2SDR environment:

```text
M2SDR_LITEPCIE_ZERO_COPY=0
M2SDR_DIRECT_DEINTERLEAVE=1
M2SDR_SOAPY_TX_IDLE_FILL=0
M2SDR_SOAPY_TX_DEDUP_TIME=1
M2SDR_SOAPY_TX_CS16_TO_SC12=1
```

## Reproduced results

Successful `-b 6` captures:

```text
20260618T080534Z:
  TX clean
  SSB lines: 3999
  late notifications: 0
  best_expected_margin: 20.87
  best_expected_cfo: 0
  verdict: EXPECTED_PCI_VISIBLE

20260618T080905Z:
  TX clean
  SSB lines: 3978
  late notifications: 0
  best_expected_margin: 17.60
  best_expected_cfo: 0
  verdict: EXPECTED_PCI_WEAK

20260618T082657Z:
  TX clean
  SSB lines: 3976
  late notifications: 0
  best_expected_margin: 25.78
  best_expected_cfo: 0
  verdict: EXPECTED_PCI_VISIBLE
```

Conclusion:

```text
External SSB detection is reproduced.
Detection strength is weak-to-visible and variable.
TX timing at 3.5 GHz / 11.52 Msps / -b 6 is reproducibly clean.
```

## Known failures / avoid for now

```text
m2sdr_n78_20MHz at 61.44 Msps / 30 kHz SCS:
  fails timing with large TX late storms.

-b 3:
  produced late storms in the timing sweep.

Unbounded full Python PSS scans:
  too slow; always use bounded scans with timeout.
```

## Scripts

```text
scripts/run_radio_ssb_35g_b6_capture_safe.sh
scripts/run_radio_ssb_35g_backoff_timing_sweep.sh
scripts/capture_bladerf_raw_35g.sh
```

## TODO

What was the exact known-good RF baseline:

```text
M2SDR TX connector/channel
bladeRF RX connector/channel
antenna/cable/attenuator path
band and RF center
SSB center/offset
sample rate and SCS
M2SDR ATT/TX gain
bladeRF RX gain
external references/clocks, if any
whether the baseline was OTA or cabled
```
