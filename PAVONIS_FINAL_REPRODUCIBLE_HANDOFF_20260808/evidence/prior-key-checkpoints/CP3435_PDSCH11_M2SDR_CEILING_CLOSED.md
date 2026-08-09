# CP3435 - PDSCH MCS11 negative; MCS9 ceiling closed

## Verdict

**VALID PDSCH11 NEGATIVE; STRICT TX HEALTH PASS.**

CP3435 closes the unexplained 10 MHz downlink-ceiling branch. The cell selects
at `14.4 dB`, both 45-second directions complete, uplink reaches
`14.115683 Mbit/s`, and the unique TX stream has zero timeout, deadline-break,
late, abandoned, or stream-deficit events. Downlink nevertheless collapses to
`0.753979 Mbit/s`.

The measured M2SDR PDSCH ceiling is MCS9. The residual ceiling is the
QPSK-to-16QAM PHY-quality transition, not scheduler capacity, endpoint
accounting, or producer delivery.

## Frozen shape

CP3424's final hardware shape is unchanged except PDSCH
`max_ue_mcs: 9 -> 11`:

- PUSCH MCS19;
- TX lead `11,000,000 ns`;
- PRACH compensation `126564`;
- non-PRACH shifts `-125918/-122598`;
- edge calibration `-126560`;
- `100 us` guard with inside-guard writes;
- 23 stage-1 candidates;
- warning logging and sequential 45-second four-stream traffic.

Run stamp: `20260731T032155Z`.

## Traffic and PHY result

| Direction | CP3435 | CP3424 PDSCH9 | Retained |
|---|---:|---:|---:|
| Downlink | `0.753979 Mbit/s` | `5.630004 Mbit/s` | `13.39%` |
| Uplink | `14.115683 Mbit/s` | `13.106290 Mbit/s` | `107.70%` |

The bounded PDSCH prefix contains:

- `140/243` total CRC OK;
- MCS11 `119/211` CRC OK (`56.40%`), median `52 PRB/1217 bytes`;
- MCS10 `7/18` CRC OK (`38.89%`), median `52 PRB/1089 bytes`.

CP3424's PDSCH9 prefix was `249/256` CRC OK. MCS10 already fails despite the
same full-PRB `1089-byte` TBS observed at MCS9.

## Modulation boundary

OCUDU's TS 38.214 table maps:

- MCS9 to QPSK with target code rate `679/1024`;
- MCS10 to 16QAM with target code rate `340/1024`;
- MCS11 to 16QAM with target code rate `378/1024`.

Therefore the first nominal spectral-efficiency step changes modulation order
while leaving the observed full-PRB TBS at `1089 bytes`. Its collapse cannot
be blamed on grant size or slot cadence. CP3435 also removes producer delivery
as a confound:

- zero deadline breaks;
- zero stream deficit;
- zero abandoned samples;
- zero Soapy timeouts and downlink-late lines;
- one `480`-sample partial fully recovered in the returned stream.

The remaining issue is insufficient 16QAM EVM/coherence margin somewhere in
the M2SDR downlink OTA chain. The existing trace EVM value is modulation
dependent and does not itself localize the component; the class transition and
clean producer evidence establish the system boundary.

## Ceiling

For the current 10 MHz, one-layer profile:

- PDSCH MCS9 full-PRB TBS: `1089 bytes`;
- absolute one-allocation-per-1ms-slot payload ceiling: `8.712 Mbit/s`;
- best measured phone downlink: `8.331264 Mbit/s`;
- best measured srsUE/M2SDR downlink: `7.059214 Mbit/s`;
- CP3324 no-RF PDSCH9 downlink: `7.599795 Mbit/s`.

The phone result already reaches `95.63%` of the configured MCS9 slot ceiling.
No PDSCH MCS above 9 is promoted.

## Decision

Close the MCS-ceiling search. The next source-level investigation, if pursued,
is the 16QAM EVM/coherence boundary. Bandwidth and MIMO can now be discussed
honestly, but widening a QPSK-limited cell will trade sample-rate headroom for
more PRBs rather than fix the modulation defect. Producer-stall work is no
longer a prerequisite for this downlink ceiling because CP3435 is strict-clean.

## Artifacts

- `cp3435_summary.json`
- `cp3435_bounded_pdsch_summary.json`
  - SHA-256
    `e3035e3cdc8f16f7b8def98e48bbf88d7f07360703a140dd2b120a81a4818f19`
- `analyze_bounded_pdsch.py`
  - SHA-256
    `f796f9bcdfa18cc43dd7fba599229ec15e5e445c0f605820c5f6d102f3459200`
- `results/attempt_20260731T032155Z/sustained_analysis.json`
  - SHA-256
    `152ed825af08e290b233f7469cd850cabbf98c3f15d69540c4c20a9e14857a8d`
- `results/attempt_20260731T032155Z/execution_summary.json`
  - SHA-256
    `f1437996a22e5a987544358881cae02738246128d2f41f35b7e8c04bdb451178`
- `cp1733_stage6_realue_sib1_rachcfg_summary_20260731T032155Z.json`
  - SHA-256
    `572b4b801eaed25c673f906687d7f565608681cb187ecd6455e07cbb2b5c085d`
- `cp3435_lead11_mcs19_pdsch11_preconnected_RAN host_20260731T032155Z.tar.gz`
  - SHA-256
    `cb04ea895cc798a3a39667a4964375cc6a3317a6218dac94e302a87f82831e1f`
- `cp3435_lead11_mcs19_pdsch11_preconnected_UE host_20260731T032155Z.tar.gz`
  - SHA-256
    `952bbf4d19065f3762a45207e40af3adf127bdcbc66822363d4ba66cfb26d6c9`
- `[controller-workspace]/ocudu/lib/ran/pdsch/pdsch_mcs.cpp`
  - SHA-256
    `84a4dd349042967eef604e466e0bd18eaf2cfd6e4eb486f9bd0976a9fb1c0516`
