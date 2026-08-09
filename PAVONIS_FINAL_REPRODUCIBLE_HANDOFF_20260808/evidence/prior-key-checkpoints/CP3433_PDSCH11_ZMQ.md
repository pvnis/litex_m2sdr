# CP3433 - PDSCH MCS11 ZMQ bracket

## Verdict

**PASS.** PDSCH MCS11 provides a clean, modest no-RF gain and is the final
hardware bracket between the proven PDSCH9 pass and the PDSCH12/13 failures.

The CP3430 ZMQ harness is reused with PDSCH `max_ue_mcs: 11`. Relative to the
original CP3324 control, the sole semantic change is PDSCH MCS9 to MCS11.
PUSCH MCS16, all binaries and private-role hashes, 10 MHz geometry, traffic
tools, and 45-second four-stream workload remain fixed.

## Throughput

Both attach and private user data pass.

| Profile | Receiver downlink | Receiver uplink |
|---|---:|---:|
| CP3324 PDSCH9 | `7.599795 Mbit/s` | `15.475241 Mbit/s` |
| CP3433 PDSCH11 | `8.523130 Mbit/s` | `15.179191 Mbit/s` |
| CP3430 PDSCH13 | `10.989462 Mbit/s` | `15.549961 Mbit/s` |

PDSCH11 raises downlink by `12.15%` over MCS9. The sender/receiver difference
is `2,132,128` bytes, consistent with the already-classified stop-time socket
backlog.

## Scheduler proof

The compact 45-second scheduler result contains:

- `43,816` PDSCH rows, all new transmissions and zero retransmission rows;
- scheduled new data `8.933799 Mbit/s`;
- slot utilization `97.369%`;
- `37,244` MCS11 rows and `6,572` MCS10 rows;
- median `52 PRB` and `1217 byte` TBS;
- median one-slot scheduling gap and eight-slot HARQ reuse.

The gain is the expected TBS effect, with no scheduler-cadence or workload
change.

## Decision

Gate PDSCH11 on CP3424's final MCS19/lead11 M2SDR profile and run it once.
If that hardware run has normal access and TX health but fails PDSCH/throughput,
close PDSCH9 as the measured hardware downlink ceiling. Do not add another MCS
bracket or stack bandwidth, MIMO, or producer changes.

## Artifacts

- `gnb_zmq_fdd_band3_10mhz_pdsch11.yml`
  - SHA-256
    `4b70c5aada6aba829d9036021f0972ffd92bd7c53e978807496902428dd485fe`
- `cp3433_run_zmq_pdsch11_debug.sh`
  - SHA-256
    `dabf78f63974a5c1c48d0bba1dedaf09aa48b3a0afa21187039a63abc0e7d44a`
- `parse_dl_scheduler.py`
  - SHA-256
    `db2b06c25cadf6f9ef41bb523d7d09418ea5caca88bc82160cef61dc1e7c7a3b`
- `core_sustained.json`
  - SHA-256
    `1e0458263a652352eb3e0278df6df8c8f476eabbb699a0993edaaa772311536e`
- `ue_sustained.json`
  - SHA-256
    `8e47736e79973f2f6cd1e0d63801f2f83679e9bae11422c7d3a430ab2bc4eddf`
- `cp3433_dl_scheduler.json`
  - SHA-256
    `37a09dd6bdb800aab9816672b23769690a444c76533e85a8dbf4a5e36f4bd443`
- `run_20260731T032001Z.log`
  - SHA-256
    `1c67eec31d18c8854291ffe3455a5a66abea5fad0b429c145f7d14cfddfd8ef0`
