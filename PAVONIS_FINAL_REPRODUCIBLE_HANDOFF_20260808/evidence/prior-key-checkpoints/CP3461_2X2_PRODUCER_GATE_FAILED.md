# CP3461 - 2x2 MIMO producer gate

## Verdict

**FAIL / STRICT PRODUCER HEALTH.** The isolated cell opened as genuine
`10 MHz, 2T2R`, and both M2SDR TX channels read back at 11.52 Msps. The
existing 11 ms lead did not deliver a lossless stream: two deadline breaks
abandoned 11,798 samples.

No UE or phone was started. This is a producer-capacity result, not an attach,
PHY, MIMO-rank, or throughput result.

## Runtime proof

The gNB startup records `10 MHz, 2T2R`. The Soapy readback reports
`nof_channels=2` and separate channel 0 and channel 1 entries, each at the
expected carrier, 11.52 Msps, and configured attenuation. The launcher
readback proves 11 ms TX lead and the 100 us inside-guard deadline policy.

The controller exits nonzero because the deliberately absent UE endpoints
never produce traffic results. That generic ladder verdict is ignored here;
the dedicated radio summary is the authority for this gNB-only gate.

## Producer result

Compact stop counters:

| Metric | Value |
| --- | ---: |
| Cumulative write timeouts | 2 |
| Deadline breaks | 2 |
| Abandoned / stream-deficit samples | 11,798 |
| Direct data-return deficit | 2,524 |
| Partial returns | 1 |
| Downlink-late lines | 0 |
| Release-late lines | 0 |
| Hard write errors | 0 |
| Inside-guard recoveries | 3 / 3, 3,066 samples |

The absence of per-timeout log lines does not make this clean; the binary-safe
stop summary records the two timeout events and exact deficit. This fails the
same zero-deficit criterion used by the single-channel campaign.

## Decision

Do not run the real phone on 2x2 yet. The configuration path is valid, but
11 ms does not provide deterministic producer headroom under the dual-channel
payload. MIMO is now blocked on the producer-headroom branch rather than on
RF power, SDR channel count, or OCUDU rank support.

Next, keep this exact gNB-only 2x2 stress shape and change one variable: TX
lead. A strict-clean lead can reopen one real-phone rank-2 attempt. If no
reasonable lead is strict-clean, 2x2 is outside the current host/driver
real-time envelope.

## Evidence

- `results/attempt_20260731T065340Z/execution_summary.json`
  - SHA-256 `59a2ae017abbac57771027669b3b59f2132293c90a34d7c5f4c32d06fb540314`
- `../cp1733_stage6_realue_sib1_rachcfg_summary_20260731T065340Z.json`
  - SHA-256 `d1ee8920930dd313ebc9ecc6f1e9ceaa17763e4ce22ba9031f3706afc4b17c4c`
- `../cp3461_mimo_2x2_gnb_only_[gNB host]_20260731T065340Z.tar.gz`
  - SHA-256 `30ae4ffcd7885dd83fa5527ffe06f95c2c20eaad493830754f293774077c38bf`
- M2SDR TX readback inside the extracted archive
  - SHA-256 `e31a2f1b303f9694b78679f71610b18eda24fd2c1b0d7725ed6c32302592f984`
- `producer_health.txt`
- `cp3461_summary.json`
