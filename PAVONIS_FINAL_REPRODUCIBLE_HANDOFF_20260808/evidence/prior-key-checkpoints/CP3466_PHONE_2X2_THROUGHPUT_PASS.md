# CP3466 - Real-phone 2x2 throughput pass

## Verdict

**PASS.** The isolated 10 MHz real-phone 2x2 profile attached, passed
bidirectional user data, and sustained `16.36088 Mbit/s` downlink and
`0.0903454 Mbit/s` uplink over 45 seconds per direction.

The downlink is `1.9638x` the CP3175 all-wireless 1x1 baseline. This closes the
unexplained 10 MHz downlink ceiling: the missing capability lever was the
second spatial layer, not wider bandwidth, higher MCS, RF power, or producer
stall tuning.

## Run

- Fresh stamp: `CP3466_20260731T093442Z`.
- Classification: `valid_success`.
- Runtime cell: 10 MHz, 15 kHz SCS, PDSCH maximum MCS9, genuine `2T2R`.
- TX readback: two channels, each at `11.52 Msps`.
- Phone timing family, primer path, data path, and CP3121 baseline were
  preserved.
- Attach-to-activation time: `22.289 s`.
- Bidirectional echo: pass.

## Sustained Throughput

| Direction | CP3175 1x1 | CP3466 2x2 | Change |
|---|---:|---:|---:|
| Downlink | 8.331264 Mbit/s | 16.360880 Mbit/s | +96.38% |
| Uplink | 0.124854 Mbit/s | 0.090345 Mbit/s | -27.64% |

The endpoint and duration cross-checks pass. The uplink result is valid but did
not improve; it remains a separate scheduling/PHY limitation rather than a
reason to discount the downlink result.

## Layer Proof

The planned scheduler DL-RI trace emitted zero rows. That telemetry axis is
therefore instrument-negative, and this checkpoint does **not** claim a direct
reported RI value.

The layer result is nevertheless proven by an independent transport bound:

1. The exact CP3466 runtime configuration is 10 MHz, SCS15, PDSCH MCS9.
2. CP3435 measured the one-layer, full-52-PRB MCS9 TBS as `1089 bytes`.
3. At one allocation per 1 ms slot, the absolute one-layer payload ceiling is
   `1089 * 8 * 1000 = 8.712 Mbit/s`.
4. CP3466 delivered `16.36088 Mbit/s` of application goodput, `1.878x` that
   transport ceiling.

Application goodput cannot exceed its radio transport payload. CP3466
therefore transported more than one layer even though the optional DL-RI
logger did not route rows.

## Producer Health

The two-channel run was strict-clean:

- Soapy TX timeouts: `0`;
- downlink-late events: `0`;
- final stream deficit: `0` samples;
- deadline write breaks/timeouts/abandonment: `0/0/0`;
- one 14-sample partial-return deficit was fully recovered.

This validates the producer-headroom work under the actual phone/data load,
not only the preceding gNB-only gates.

## Decision

- Promote the isolated real-phone 10 MHz 2x2 profile as the measured Pavonis
  downlink maximum.
- Close the old one-layer `~8.7 Mbit/s` downlink ceiling as explained.
- Do not reopen 15/20 MHz, MCS-above-9, RF-power, or antenna branches without
  new evidence; those branches were already closed independently.
- Keep uplink throughput and the missing optional DL-RI output as separate,
  non-blocking follow-up items.
- No repeat RF run is needed to establish this milestone.

## Evidence

- `cp3466_summary.json`
- `capacity_rank_bound.json`
- `producer_health.txt`
- `rank_summary.json`
- `../cp3465_phone_mimo_2x2_no_rf_gate_20260731/attempt_CP3466_20260731T093442Z/sustained_analysis.json`
  - SHA-256:
    `137f541f2462f2f9f6e2ac67e6be5fe4caa961e3ca85106b1a84ca6553e0555e`
- `../cp2893_CP3466_20260731T093442Z_summary.txt`
  - SHA-256:
    `0d24848ca1526f621994543a115c2d3b4a02d0ed2f8c0d3675c5158d4919baee`
- `../cp2893_CP3466_20260731T093442Z_[gNB host].tar.gz`
  - SHA-256:
    `9b594ff0409e82722e10293a5207a1bec4fa323449d400681afbc02b58f747c2`
- `../cp2893_CP3466_20260731T093442Z_[UE host].tar.gz`
  - SHA-256:
    `f7b599fb05fe0c6bea3bbd87c9332f7a323a613085bcfe9492e19db6dda71c19`

No credentials, subscriber identifiers, host identifiers, or NAS payload bytes
are included in this report.
