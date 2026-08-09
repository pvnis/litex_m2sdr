# CP3429 - PDSCH MCS16 M2SDR valid negative

## Verdict

**VALID PDSCH16 NEGATIVE; STRICT TX HEALTH PARTIAL.**

The sole-variable PDSCH MCS16 profile attaches and completes both sustained
45-second traffic directions, so this is not an RF or access void. Downlink
collapses to `0.608284 Mbit/s`, and the bounded early PDSCH trace decodes only
`75/218` rows (`34.40%`). MCS16 itself decodes only `59/191` rows (`30.89%`).
PDSCH MCS16 is too aggressive over the current M2SDR path.

The strict unique-stream TX gate is partial because one deadline break abandons
`9476` stream samples. This caveat is recorded, but one sub-millisecond deficit
cannot explain persistent PDSCH failure across `143/218` bounded rows and a
45-second receiver result at only `10.80%` of CP3424's downlink.

## Frozen shape

The run uses CP3428's gated profile:

- CP3424's final MCS19/lead11 operating point;
- PUSCH MCS19 unchanged;
- PDSCH `max_ue_mcs: 9 -> 16` as the only config delta;
- PRACH compensation `126564` samples;
- non-PRACH shifts `-125918/-122598`;
- edge calibration `-126560`;
- `100 us` deadline guard with inside-guard writes;
- 23 stage-1 candidates;
- four streams and sequential 45-second directions.

Run stamp: `20260731T023826Z`.

## Access and traffic

Cell selection passes at `13.378 dB` SNR and `-3923.021 Hz` CFO. Both endpoint
JSON files are complete.

| Direction | CP3429 | CP3424 reference | Retained |
|---|---:|---:|---:|
| Downlink | `0.608284 Mbit/s` | `5.630004 Mbit/s` | `10.80%` |
| Uplink | `5.699309 Mbit/s` | `13.106290 Mbit/s` | `43.49%` |

The outer wrapper returns rc1, but the endpoint and radio artifacts establish
that access and both traffic phases completed. The semantic result is valid.

## PDSCH evidence

The bounded UE trace covers 218 early post-access PDSCH decodes:

- CRC OK `75`, CRC KO `143`, yield `34.40%`;
- MCS16 rows `191`, CRC OK `59`, CRC KO `132`, yield `30.89%`;
- MCS16 median allocation `52 PRB`, median TBS `2112 bytes`;
- median MCS16 SNR is `5.329 dB` on CRC-OK rows and `3.5335 dB` on CRC-KO rows;
- retransmission RV values are present throughout the prefix.

This trace is bounded and is not presented as a full-transfer HARQ census. It
is sufficient to explain why a profile that nearly doubles ZMQ downlink at
CP3427 collapses over the hardware path.

## TX and uplink health

The log has zero Soapy timeout lines and zero downlink-late lines. The stop
summary records:

- one deadline break;
- `1502` write-deficit samples;
- `9476` stream-deficit/abandoned samples;
- two inside-guard successes recovering `2044` samples;
- zero hard errors.

Bounded PUSCH is `84/128` CRC OK (`65.625%`) with median accepted SINR
`5.266 dB` and median TBS `512 bytes`. Uplink is therefore also weaker than
CP3424, but it does not make the PDSCH16 conclusion ambiguous.

## Decision

Do not promote PDSCH16 and do not stack bandwidth, MIMO, or producer-stall
changes. The next discriminator is a sole-variable PDSCH ceiling of MCS13:
first prove it over ZMQ, then gate and measure the same CP3424 M2SDR shape.
MCS13 is the midpoint that can recover meaningful throughput without assuming
the MCS16 RF margin exists.

## Artifacts

- `cp3429_summary.json`
- `cp3429_bounded_pdsch_summary.json`
  - SHA-256
    `817adca3cc13510e5f6feef8756058ca2e7d97248a2670df8bc845206e704a0b`
- `analyze_bounded_pdsch.py`
  - SHA-256
    `f030fd797fbafcc37488874eb650f7982f1a46d1ddd6d85362fa69c9fe17e725`
- `results/attempt_20260731T023826Z/sustained_analysis.json`
  - SHA-256
    `4e6e528761bc30e76c31a023cb8a354940bcffd6a8e2dfcc08a9d271ae753c05`
- `results/attempt_20260731T023826Z/execution_summary.json`
  - SHA-256
    `15e83acc7e3353fe80d61f4ee8b4d55bdbdaae4d7d55b81ecaf4158ea355ad4a`
- `cp1733_stage6_realue_sib1_rachcfg_summary_20260731T023826Z.json`
  - SHA-256
    `3e191a3158367f4c9bd048ff29085bc1332f81088d954a1776a286735ce066da`
- `cp3429_lead11_mcs19_pdsch16_preconnected_RAN host_20260731T023826Z.tar.gz`
  - SHA-256
    `c37cd6c3c0be4d241ab77de91d0f024286eab3e7375393dc81dd623bfccdff61`
- `cp3429_lead11_mcs19_pdsch16_preconnected_UE host_20260731T023826Z.tar.gz`
  - SHA-256
    `f755fb5d428ef32ed486ba0bbd64767ee22f0ee4b91cf52b1e2f6a9543c36310`
