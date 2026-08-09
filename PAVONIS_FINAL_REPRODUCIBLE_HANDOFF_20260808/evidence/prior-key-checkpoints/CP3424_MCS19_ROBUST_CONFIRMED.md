# CP3424 - Lead-11 MCS19 robust operating point confirmed

## Verdict

**VALID PRIMARY PASS; SECONDARY PACKAGING FALSE NEGATIVE.** The frozen MCS19
shape at the accepted 11 ms timing margin completes attach and sustained
bidirectional traffic at `5.6300/13.1061 Mbit/s` DL/UL. Bounded PUSCH is
`128/128` CRC OK and unique-stream TX delivery is exact.

The outer rc1 is not a radio, payload, or TX-health failure. The UE host
packager returned nonzero because `tar` observed a detail log changing while
it was read. The archive was still retrieved, the UE log and all endpoint JSON
are present, sustained analysis passes, and the authoritative TX stop gate
returns zero.

## Evidence

Cell selection passes at `13.268 dB` SNR and `-3890.872 Hz` CFO. The complete
128-row PUSCH prefix decodes, including all `93` 64QAM rows at target code rate
`0.5049`; their median CRC-OK SINR is `24.74 dB`.

The endpoint-authoritative 45-second, four-stream transfer reports:

- downlink `5.6300 Mbit/s`;
- uplink `13.1061 Mbit/s`;
- valid duration and stream-count gates in both directions.

The TX stop summary records:

- stream samples equal returned samples at `4,818,193,770`;
- zero stream deficit, deadline break, timeout, late, abandoned sample, or
  hard error;
- one 480-sample call-level partial fully retried.

CP3424 reaches `87.93%` of CP3410's clean peak uplink and `83.48%` of its
clean peak downlink. That is ordinary run-to-run throughput scatter, while
the PHY and delivery gates remain stronger (`128/128` versus CP3410's
`126/128` PUSCH).

## Decision

Pin the final operating point to MCS19 with 11 ms transmit lead,
`PRACH=126564`, non-PRACH edge `-126560`, 100 us deadline guard, and the
inside-guard write policy enabled. Report CP3410 as the measured clean peak
(`6.7441/14.9059 Mbit/s`) and CP3424 as the independently confirmed robust
shape (`5.6300/13.1061 Mbit/s`).

The throughput ladder is closed. MCS20 and higher are retired because the
first clean MCS20 run lost CRC yield and throughput. Rare pre-UE
producer/deadline voids remain an operational caveat and use one unchanged
retry. The USB LiteX throughput recollection remains external context, not
campaign evidence.

## Artifacts

- `cp3424_summary.json`
- `bounded_pusch_summary.json`
- `../cp3423_lead11_mcs19_robust_ota_20260731/results/attempt_20260731T012747Z/execution_summary.json`
- `../cp3423_lead11_mcs19_robust_ota_20260731/results/attempt_20260731T012747Z/sustained_analysis.json`
- `../cp3423_lead11_mcs19_robust_ota_20260731/results/attempt_20260731T012747Z/core_sustained.json`
- `../cp3423_lead11_mcs19_robust_ota_20260731/results/attempt_20260731T012747Z/ue_sustained.json`
- `../cp1733_stage6_realue_sib1_rachcfg_summary_20260731T012747Z.json`
- `../cp1733_20260731T012747Z_sequence.log`
- `../cp1660_20260731T012747Z_sens_package_stage6_alltimed.log`

No credentials, private identities, payload bytes, or NAS bytes are included.
