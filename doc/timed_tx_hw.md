# Hardware timed TX (branch `hw-timed-tx`)

## Why

The `working-ue` plugin times TX in software: it anchors "sample N is emitted at board time T" once
per start (and again after every underflow) and lets a free-running 256 × 2048-sample DMA ring stream.
The anchor has one-buffer granularity plus the DMA start race and a whole-lap ambiguity (22.76 ms at
23.04 MSps), so the emission time of a stamped sample differs from its stamp by a random per-start
offset, and every re-anchor moves the timeline. A 5G UE synchronises to the *radiated* DL and its uplink
then lands that offset off the gNB's slot grid; PRACH B4 aliases the error modulo 33 µs, PUSCH does not.
OCUDU works around it by measuring the offset at start-up and shifting its RX timestamps
(`docs/m2sdr_soapy_bringup.md` in the OCUDU tree).

Hardware-timed radios (UHD, LimeSDR, bladeRF metadata mode) do not have the problem: the FPGA holds
each timestamped packet until its tick and drops it if it is late; the timeline never moves. This
branch gives the M2SDR the same semantics.

## Earlier attempts and what was wrong with them

* `timestamp_tx*` branches (Dec 2025 – Jan 2026): scheduler inside the header module, never fully
  functional ("streaming occurs in the wrong frame", "latched_ts 20 cycles delayed").
* `TimedTXArbiter` (Mar – May 2026, commits 822fe2a0 … 63739d35, only on `5gnr`/`stefan`): a
  sys-domain hold/pass/drop FSM upstream of the RFIC. It never reached `main`; the June commits made
  software timing the only mechanism. The structural problem: `ad9361/core.py` feeds the PHY through
  `tx_cdc` and a `tx_rfic_fifo` with **priming hysteresis** (output starts only when the FIFO is half
  full and stops when it empties), so anything gated upstream of it is released into a FIFO whose
  fill time depends on DMA burst timing — microseconds of release jitter, and every gap re-primes.

## Design

`litex_m2sdr/gateware/timed_tx_gate.py` — `TimedTXGate`, inserted between the TX header extractor
and the loopback mux (`header.tx.source → timed_tx → txrx_loopback.tx_sink`), sys domain, CSR slot 42.

Every DMA buffer (8192 bytes) already carries a 16-byte header when `header.tx.control.header_enable`
is set: a 64-bit sync word and a 64-bit **timestamp in ns of board time** (`time_gen`, the same clock
that stamps RX frames). The extractor strips it, latches `timestamp`, and frames the 1022-word payload
with `first`/`last`. On the payload's first word the gate decides:

| condition | action |
|---|---|
| `timestamp == 0` or gate disabled | pass (untimed, today's behaviour) |
| `time < timestamp` | **HOLD**: `sink.ready = 0` (DMA reader stalls, `hw_count` stops), nothing emitted; the PHY starves and outputs zeros; release when `time >= timestamp` |
| `time − timestamp ≤ late_margin` | **PASS** |
| `time − timestamp > late_margin` | **DROP** the whole frame, `late_count++` |

In a continuous stream only the first frame after a start or a gap is held; the following frames arrive
exactly on time and pass. After a host stall the frames that missed their time are dropped and the
stream re-aligns itself at the first on-time frame — no re-anchor, no timeline shift.

**Determinism.** While the gate is enabled the SoC forces `ad9361.tx_force_started` (CDC'd into the
rfic domain), which removes the RFIC TX FIFO priming hysteresis. After a hold the downstream FIFOs are
empty, so the release-to-air latency is a fixed pipeline depth (± one sys clock ± one rfic clock,
i.e. about one sample); during streaming the FIFOs act as jitter buffers only.

CSRs (`timed_tx_*`): `control.enable`, `control.reset_counts` (pulse), `late_margin` (ns, default
100 000), `late_count`, `held_count`, `passed_count`, `status.{state,active,holding}`, `armed_ts`.

Simulation: `test/test_timed_tx_gate.py` (pass-through, untimed, on-time, hold-until-timestamp,
late drop, in-margin late, back-to-back frames).

## Driver

* `libm2sdr`: `m2sdr_has_tx_timed_gate()`, `m2sdr_set_tx_timed_gate(dev, enable, late_margin_ns)`,
  `m2sdr_reset_tx_timed_gate_counts()`, `m2sdr_get_tx_timed_gate_stats()` — compiled out
  (`M2SDR_ERR_UNSUPPORTED`) when `csr.h` has no `CSR_TIMED_TX_BASE`.
* SoapySDR: device/stream argument `timed_tx=hardware` (`hw`, `fpga`) next to `software` (default) and
  `off`. In hardware mode the plugin enables the TX DMA header (MTU becomes 2044 samples), enables the
  gate at `activateStream` with the late margin (default one MTU duration), stamps every DMA buffer
  with the board time of its first sample — the caller's `timeNs` for timed writes, the running
  timeline for untimed ones — and reports frames the gate dropped as `SOAPY_SDR_TIME_ERROR` in
  `readStreamStatus`. The software placement (zero insertion, late rejection, anchoring, re-anchoring
  on underflow) is bypassed.

## Test procedure

1. `test/test_timed_tx_gate.py` — simulation.
2. Build: `source /opt/Xilinx/2026.1/Vivado/settings64.sh && ~/litex-venv/bin/python litex_m2sdr.py
   --variant m2 --with-pcie --pcie-gen 2 --pcie-lanes 4 --build`; check the routed WNS in
   `build/<name>/gateware/vivado.log` (the baseline closes with ~+0.04 ns, there is little slack).
3. Flash (`m2sdr_util flash-write <_operational.bin>`, `flash-reload`, PCIe rescan), rebuild and reload
   the kernel module (`litex_m2sdr/software/kernel`, headers are regenerated by the build), rebuild
   `user/` and the Soapy module and install it.
4. RF-free: `software/soapysdr/test_hw_timed_tx.py` — digital TX→RX loopback
   (`m2sdr_util reg-write 0x10800 1`), a counter burst stamped `now + 100 ms`; the hardware RX
   timestamp of its first sample minus the stamp must be a small constant across bursts and board
   resets; `--late-test` checks that a burst stamped in the past is dropped and counted.
5. RF: OCUDU with `timed_tx=hardware` in `device_args`; `align.py`'s RX-minus-TX offset must be the
   same on every restart, and becomes the static `ru_sdr.time_alignment_calibration`.
