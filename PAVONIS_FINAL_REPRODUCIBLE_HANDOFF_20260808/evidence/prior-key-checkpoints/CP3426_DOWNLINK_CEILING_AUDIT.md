# CP3426 - Unexplained downlink ceiling, no-RF audit

## Verdict

**PASS, leading limiter localized.** The proven 10 MHz profiles cap PDSCH at
MCS9. The scheduler reaches that cap with full 52-PRB allocations, producing
an observed `1089-byte` transport block. At one allocation per 1 ms slot, the
absolute pre-overhead ceiling is therefore only:

`1089 * 8 / 0.001 = 8.712 Mbit/s`.

The next isolated variable is PDSCH `max_ue_mcs: 9 -> 16`, first over ZMQ with
no RF. Bandwidth, MIMO, and producer-stall headroom remain locked.

## Evidence

The parser joins six valid 45-second M2SDR runs:

| Run | PUSCH ceiling | DL Mbit/s | UL Mbit/s |
|---|---:|---:|---:|
| CP3390 | 16 | 7.059214 | 12.613862 |
| CP3393 | 16 | 6.472631 | 12.808777 |
| CP3396 | 28 | 7.045821 | 1.976477 |
| CP3410 | 19 | 6.744110 | 14.905918 |
| CP3421 | 20 | 6.238029 | 10.159133 |
| CP3424 | 19 | 5.630004 | 13.106090 |

Large PUSCH-MCS changes move uplink but leave downlink in a narrow
`5.630004-7.059214 Mbit/s` band because PDSCH remains capped at MCS9.

The no-RF CP3324 ZMQ control reaches `7.599795 Mbit/s`, or 87.23% of the
MCS9 one-slot ceiling. The sustained phone reference reaches
`8.331264 Mbit/s`, or 95.63%. These results are already close to the configured
ceiling; they do not support antennas, raw SNR, or PUSCH policy as the primary
downlink limiter.

## PDSCH trace proof

The bounded UE traces are early post-access prefixes, so they are not used as
sustained-transfer totals. They are sufficient to prove grant geometry:

- CP3410: `253/256` PDSCH CRC OK, 211 MCS9 rows, 167 full 52-PRB MCS9 rows;
- CP3424: `249/256` PDSCH CRC OK, 231 MCS9 rows, 171 full 52-PRB MCS9 rows;
- both runs report the same full-allocation TBS: `1089 bytes`.

The trace caps expire after about 1.2-1.4 seconds and therefore cannot answer
the sustained HARQ distribution. Recreating a live RF debug trace is not
justified while the cheaper PDSCH-cap discriminator remains untested.

## Endpoint accounting

Across all six sustained runs, the core sender has `8,590,960-10,012,000`
more downlink bytes accepted by its four sockets than the UE has received when
the 45-second timer stops. The approximately fixed four-socket backlog explains
the recurring sender/receiver crosscheck gap. It does not change the
receiver-authoritative throughput result and is not evidence of nine to ten
megabytes of radio loss.

## Decision

Create an isolated 10 MHz ZMQ config with PDSCH MCS16 as the only scheduler
change. Keep PUSCH at the established MCS16 ZMQ control policy. Require:

1. attach and 45-second four-stream bidirectional traffic;
2. downlink materially above the MCS9 ZMQ reference;
3. bounded debug proof of actual PDSCH MCS/TBS and DL HARQ behavior;
4. no RF until this gate passes.

If MCS16 raises ZMQ downlink, prepare the same sole-variable change on the
final MCS19/lead11 M2SDR profile. If it does not, revert it and inspect
downlink scheduler cadence before touching bandwidth or MIMO.

## Artifacts

- `cp3426_downlink_ceiling_analysis.json`
  - SHA-256
    `0de21ac1f754ffbc0b11b25170345b5b7342d286eb9a38893e6189561cf80115`
- `analyze_downlink_ceiling.py`
  - SHA-256
    `84375311d8fa43500d23d544d718d8b4382100b83fe00e1cda583cb1fe5c053a`
