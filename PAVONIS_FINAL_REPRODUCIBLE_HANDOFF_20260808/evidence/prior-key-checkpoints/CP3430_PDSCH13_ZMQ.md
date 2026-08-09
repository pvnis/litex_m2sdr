# CP3430 - PDSCH MCS13 ZMQ bracket

## Verdict

**PASS.** PDSCH MCS13 is a valid no-RF throughput gain and is suitable for one
isolated M2SDR discriminator.

The exact CP3427 ZMQ harness is reused with PDSCH `max_ue_mcs: 13`. Relative
to the original CP3324 control, the sole semantic change is PDSCH MCS9 to
MCS13. PUSCH MCS16, the binaries, private-role hashes, 10 MHz geometry,
traffic tools, and 45-second four-stream workload remain fixed.

## Throughput

Both attach and private user data pass.

| Profile | Receiver downlink | Receiver uplink |
|---|---:|---:|
| CP3324 PDSCH9 | `7.599795 Mbit/s` | `15.475241 Mbit/s` |
| CP3430 PDSCH13 | `10.989462 Mbit/s` | `15.549961 Mbit/s` |
| CP3427 PDSCH16 | `14.860932 Mbit/s` | `15.448868 Mbit/s` |

PDSCH13 raises downlink by `44.60%` over MCS9 and lands `26.05%` below
MCS16. The downlink sender reports `64,053,856` bytes and the receiver reports
`61,815,856` bytes; the `2,238,000` byte stop-time gap is consistent with the
already-classified four-socket queued-backlog effect.

## Scheduler proof

The configured OCUDU debug log is parsed on the RF host; only compact JSON is
retrieved. The best active 45-second window contains:

- `43,927` PDSCH rows, all new transmissions and zero retransmission rows;
- scheduled new data `11.514644 Mbit/s`;
- slot utilization `97.616%`;
- `37,337` MCS13 rows and `6,590` MCS12 rows;
- median `52 PRB` and `1569 byte` TBS;
- median one-slot scheduling gap;
- median eight-slot HARQ-process reuse gap.

This proves the measured gain is the expected grant/TBS effect rather than a
workload or scheduler-cadence change.

## Decision

Create a no-RF hardware gate from CP3428 with only PDSCH16 changed to PDSCH13,
then run one bounded M2SDR test. Preserve CP3424's MCS19/lead11, all timing,
RF, stage-1, capture, and traffic controls. Do not move to bandwidth, MIMO, or
producer-stall headroom until that result establishes the hardware downlink
operating point.

## Artifacts

- `gnb_zmq_fdd_band3_10mhz_pdsch13.yml`
  - SHA-256
    `485264aa40e684a889aedc800e23b1e81ced7d159b01e781a4527afad7163b4c`
- `cp3430_run_zmq_pdsch13_debug.sh`
  - SHA-256
    `0d8a163432530a0fe37a617fc1f5977db8c4cf833a1c6dabd2a1f892be363ca7`
- `parse_dl_scheduler.py`
  - SHA-256
    `db2b06c25cadf6f9ef41bb523d7d09418ea5caca88bc82160cef61dc1e7c7a3b`
- `core_sustained.json`
  - SHA-256
    `887cfed151e6ed2b72dd32fe43b227b1b94285ddf434270b7f7d9e06e93ee0df`
- `ue_sustained.json`
  - SHA-256
    `e034b4568415986947b9d5493726c650c8074c7301256bbd1c462917ec56dc56`
- `cp3430_dl_scheduler.json`
  - SHA-256
    `c148d52069d287dcbbee7567a18b8d1b7a74c743fb9eb75bd32668657111b581`
- `run_20260731T025550Z.log`
  - SHA-256
    `353b1b980c790c0dc90c512b9a3a7f3dc5422e227ef80142bd8abbb537a75744`
