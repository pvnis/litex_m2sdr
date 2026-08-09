# CP3432 - PDSCH MCS13 M2SDR valid negative

## Verdict

**VALID PDSCH13 NEGATIVE; STRICT TX HEALTH PARTIAL.**

The sole-variable PDSCH MCS13 profile selects the cell, completes access, and
finishes both sustained 45-second traffic directions. It is not an RF void.
Downlink nevertheless collapses to `0.280585 Mbit/s`, and the bounded early
PDSCH trace decodes only `100/243` rows (`41.15%`). MCS13 itself decodes only
`79/205` rows (`38.54%`). PDSCH13 is too aggressive over the current M2SDR
path.

Three deadline breaks abandon `24340` stream samples, so strict TX health is
partial. Those short events cannot explain `143/243` persistent PDSCH
failures or receiver downlink at only `4.98%` of CP3424.

## Frozen shape

The run uses CP3431's gated profile:

- CP3424's final MCS19/lead11 operating point;
- PUSCH MCS19 unchanged;
- PDSCH `max_ue_mcs: 9 -> 13` as the only config delta;
- PRACH compensation `126564` samples;
- non-PRACH shifts `-125918/-122598`;
- edge calibration `-126560`;
- `100 us` deadline guard with inside-guard writes;
- 23 stage-1 candidates;
- four streams and sequential 45-second directions.

Run stamp: `20260731T030429Z`.

## Access and traffic

Cell selection passes at `12.591 dB` SNR and `-3932.392 Hz` CFO. Both endpoint
JSON files are complete.

| Direction | CP3432 | CP3424 reference | Retained |
|---|---:|---:|---:|
| Downlink | `0.280585 Mbit/s` | `5.630004 Mbit/s` | `4.98%` |
| Uplink | `5.105699 Mbit/s` | `13.106290 Mbit/s` | `38.96%` |

The outer wrapper returns rc1, but the endpoint and radio artifacts prove that
access and both traffic phases completed. The semantic result is valid.

## PDSCH evidence

The bounded UE trace covers 243 early post-access PDSCH decodes:

- CRC OK `100`, CRC KO `143`, yield `41.15%`;
- MCS13 rows `205`, CRC OK `79`, CRC KO `126`, yield `38.54%`;
- MCS13 median allocation `52 PRB`, median TBS `1569 bytes`;
- median MCS13 SNR is `4.581 dB` on CRC-OK rows and `4.1515 dB` on CRC-KO
  rows;
- MCS12 also decodes only `9/26` rows.

The bounded scope is not a full-transfer HARQ census, but it directly explains
the throughput collapse. CP3424's PDSCH9 prefix was `249/256` CRC OK, so the
hardware cliff lies above MCS9 and below MCS13.

## TX and uplink health

The log has zero Soapy timeout lines and zero downlink-late lines. The stop
summary records:

- three deadline breaks;
- `3546` write-deficit samples;
- `24340` stream-deficit/abandoned samples;
- one inside-guard success recovering `1022` samples;
- zero hard errors.

Bounded PUSCH is `71/128` CRC OK (`55.47%`) with median accepted SINR
`5.238 dB` and median TBS `239 bytes`. This is a weak run in both directions,
but the cell SNR and repeated PDSCH failures still make the MCS13 rejection
unambiguous.

## Decision

Do not promote PDSCH13. Test PDSCH11 as the sole next variable, ZMQ first and
then one gated M2SDR run. MCS11 brackets the known PDSCH9 pass and the
PDSCH12/13 failure evidence without stacking another subsystem change.

Bandwidth, MIMO, and producer-stall work remain locked until the downlink
operating ceiling is established.

## Artifacts

- `cp3432_summary.json`
- `cp3432_bounded_pdsch_summary.json`
  - SHA-256
    `3aca595ed6b8daa822b07a9adcf73203fa2e9e3250c35a1e1062f96af1acd1c4`
- `analyze_bounded_pdsch.py`
  - SHA-256
    `024c2c8a94ed83592bff57bae1fb04f44afccc40b2a22af716cc1eedd5b8d019`
- `results/attempt_20260731T030429Z/sustained_analysis.json`
  - SHA-256
    `a8d72ac5468784d91d3c497cdb60ed22b0f46b0becd621fcf57a1f83eec97e11`
- `results/attempt_20260731T030429Z/execution_summary.json`
  - SHA-256
    `ec92a8112b960c895dacf103b546bec9bebf3f775d56ad9acbd239cd1dd971d5`
- `cp1733_stage6_realue_sib1_rachcfg_summary_20260731T030429Z.json`
  - SHA-256
    `1aaa68d21c264b9c796a1e81f0d3cc64c0dd1796942567ec404a1bb4e00360f2`
- `cp3432_lead11_mcs19_pdsch13_preconnected_RAN host_20260731T030429Z.tar.gz`
  - SHA-256
    `edbe833cbbd31f93474327ad7f53dab13d2b4fe883593a8fef9fec124490863f`
- `cp3432_lead11_mcs19_pdsch13_preconnected_UE host_20260731T030429Z.tar.gz`
  - SHA-256
    `2b7a04367d22460cbec5eb80fe04bbdfe514d007b32c9607fa6fbb09da0d2d69`
