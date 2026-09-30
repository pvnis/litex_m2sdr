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
and the loopback mux (`header.tx.source → timed_tx → txrx_loopback.tx_sink`), sys domain, CSR slot 43.

Every DMA buffer (8192 bytes) already carries a 16-byte header when `header.tx.control.header_enable`
is set: a 64-bit sync word and a 64-bit **timestamp in ns of board time** (`time_gen`, the same clock
that stamps RX frames). The extractor strips it, latches `timestamp`, and frames the 1022-word payload
with `first`/`last`. On the payload's first word the gate decides:

| condition | action |
|---|---|
| `timestamp == 0` or gate disabled | pass (untimed, today's behaviour) |
| `time < timestamp` | **HOLD**: `sink.ready = 0` (DMA reader stalls, `hw_count` stops), nothing emitted; the PHY starves and outputs zeros; release when `time >= timestamp` |
| `time − timestamp ≤ late_margin` | **PASS** |
| `time − timestamp > late_margin` | **DROP** the whole frame; `late_count++`, or `stale_count++` if older than `stale_margin` (a ring slot re-read a lap later) |

In a continuous stream only the first frame after a start or a gap is held; the following frames arrive
exactly on time and pass. After a host stall the frames that missed their time are dropped and the
stream re-aligns itself at the first on-time frame — no re-anchor, no timeline shift.

**Determinism.** While the gate is enabled the SoC forces `ad9361.tx_force_started` (CDC'd into the
rfic domain), which removes the RFIC TX FIFO priming hysteresis. After a hold the downstream FIFOs are
empty, so the release-to-air latency is a fixed pipeline depth (± one sys clock ± one rfic clock,
i.e. about one sample); during streaming the FIFOs act as jitter buffers only.

CSRs (`timed_tx_*`): `control.enable`, `control.reset_counts` (pulse), `late_margin` (ns, default
100 000), `stale_margin` (ns, default 10 000 000), `late_count`, `stale_count`, `held_count`,
`passed_count`, `status.{state,active,holding}`, `armed_ts`.

Simulation: `test/test_timed_tx_gate.py` (pass-through, untimed, on-time, hold-until-timestamp,
late drop, in-margin late, back-to-back frames).

## Driver

* `libm2sdr`: `m2sdr_has_tx_timed_gate()`, `m2sdr_set_tx_timed_gate(dev, enable, late_margin_ns,
  stale_margin_ns)`, `m2sdr_reset_tx_timed_gate_counts()`, `m2sdr_get_tx_timed_gate_stats()` — compiled
  out (`M2SDR_ERR_UNSUPPORTED`) when `csr.h` has no `CSR_TIMED_TX_BASE`. `m2sdr_set_tx_ring_lead()` /
  `m2sdr_get_tx_resync_events()` manage the write pointer against the free-running DMA reader (below).
* SoapySDR: device/stream argument `timed_tx=hardware` (`hw`, `fpga`) next to `software` (default) and
  `off`. In hardware mode the plugin enables the TX DMA header (MTU becomes 2044 samples), enables the
  gate at `activateStream`, stamps every DMA buffer with the board time of its first sample — the
  caller's `timeNs` for timed writes, the exact timeline for untimed ones — and reports frames the gate
  dropped as late as `SOAPY_SDR_TIME_ERROR` in `readStreamStatus`. The software placement (zero
  insertion, late rejection, anchoring, re-anchoring on underflow) is bypassed. Optional arguments:
  `tx_late_margin_ns` (default 1000), `tx_lead_buffers` (default 4).

## What the RF tests taught us (OCUDU, 23.04 MSps, band n78)

Each of these hid the working gate at first; all are in the plugin/libm2sdr, the gate itself did not
change except for the stale counter.

1. **The DMA synchronizer waits for a PPS edge.** The kernel arms `pcie_dma0.synchronizer` at reader
   start (mode 01: sync on PPS *and* TX data present); the TX stream then starts at the next PPS edge,
   up to 1 s later, and every frame stamped less than that ahead is already late. Hardware mode sets
   the synchronizer bypass — the frame stamps carry the timing.
2. **The reader prefetches and the ring is stale.** The LitePCIe reader free-runs over the 256-slot
   ring and prefetches ~2-3 buffers past its table index. A frame written at `hw_count` is never
   emitted, and slots not rewritten since the previous lap (22.7 ms) are re-read with their old
   stamps. The plugin zeroes the ring before the reader starts (zero header = untimed silence) and
   libm2sdr keeps the write pointer at least `lead` (4) slots past the reader's **live** table index
   (`READER_TABLE_LOOP_STATUS`; the kernel's `hw_count` is refreshed only every 8 buffers). The lead
   must stay well below one frame's sweep time: a skipped slot costs ~8 µs of DMA time, a frame lasts
   88.7 µs; with a lead of 24 the frame after every hold arrived late, was dropped, and the ring
   thrashed (30k stale drops/s, one third of the DL emitted).
3. **Rounding in the software timeline.** srsRAN/OCUDU stamp only the start of a burst; every later
   write is untimed and stamped from the plugin's timeline, which was advanced by
   `llround(n × 1e9 / rate)` per chunk. 0.3 ns per 2044-sample chunk is ~1 ppm: the emission offset
   ramped ~20 samples/s until the burst restarted. The timeline is now anchor + exact sample count.
4. **Late margin = pipeline fill.** A frame arriving after its stamp but within the margin is emitted
   at once and its lateness stays as a fill level in the (shallow) TX pipeline — every later frame is
   held until its stamp, so the level never drains. With a one-MTU margin the offset settled anywhere
   within a frame (1866 vs 168 samples on two starts). The default margin is 1 µs: every frame goes
   through the hold path, late frames are dropped (88.7 µs of silence, recovered by HARQ) instead of
   shifting the DL timing the UE synchronises to.
5. **Stale re-reads are not late frames.** OCUDU ends its TX burst on every `TIME_ERROR` and
   discards blocks until the end-of-burst is acknowledged; the resulting ring gap makes the reader
   sweep stale slots, whose drops were reported as `TIME_ERROR` — a self-sustaining loop after any
   host stall. The gate now counts drops older than `stale_margin` (plugin: half a lap) in
   `stale_count`; only `late_count` is reported to the application.
6. **The RFIC PHY loopback cannot measure the release offset.** In loopback the RX path only produces
   samples while TX data flows, so RX time freezes during a hold and the burst lands in a frame
   stamped at the write time. The loopback test (`software/soapysdr/test_hw_timed_tx.py`) proves hold,
   release, sample continuity and late/stale drops; the offset is measured over the air.

7. **RX timestamps must be sample-exact.** RX frames were labelled with each frame's FPGA nanosecond
   stamp. The stamps carry quantisation noise, so an application converting them to samples saw labels
   that were not contiguous — ±1 sample, 2000–3000 times per second on about half of the starts
   (it depends on the sub-sample phase at start). OCUDU's lower PHY treats any such mismatch as lost
   alignment and discards RX up to the next subframe: permanent "PUxCH request late" / "UL processor
   is busy" with the UE attached and the UL dead. This, not anything in the TX path, was the
   start-dependent UL storm (8 starts: 0 jumps ⇔ 0 lates, 42k–66k jumps ⇔ storm). RX labels are now
   produced by **counting samples** from an anchor; the FPGA stamp is only used to detect real gaps
   (whole frames lost, advanced by exactly that many frames). A map `ns0 ↔ tick0` converts ticks to
   board time for the FPGA gates.
8. **Timed RX start.** OCUDU seeds its timeline from "RX begins at `init_time`"; the plugin started RX
   immediately (~95 ms earlier). `gateware/timed_rx_start.py` holds the RX header inserter in reset
   until board time reaches the requested start, so the first frame delivered is stamped with it (a
   `lead` of 5 sys cycles pre-compensates the compare-to-stamp latency).
   `activateStream(RX, HAS_TIME, t)` arms it before the DMA writer starts.
9. **TX back-pressure.** A USRP's `send()` blocks once its FIFO is full of samples that are not due
   yet. The ring accepted 22.7 ms; OCUDU stamps its first frames 100 ms ahead, so its DL timeline ran
   177–194 slots ahead of RX at start. `tx_fifo_buffers` (default 96 = 8.5 ms) bounds the number of
   submitted-but-not-due frames; `writeStream` times out beyond it and the caller retries.

## Time base: `time_base=samples`

Internally every sample has an integer index ("tick") at the stream rate. By default the Soapy API
carries nanoseconds computed as `round(tick × 1e9 / rate)`, which an application rounding back gets
exactly. With the device argument `time_base=samples` every Soapy `timeNs` parameter carries the tick
itself: `readStream`/`acquireReadBuffer` time, `writeStream` time, `activateStream` time,
`readStreamStatus` time, `get/setHardwareTime`. No nanosecond value crosses the API, and nothing
depends on double precision (2^53 ns is 104 days of board time). `timed_rx=off` keeps the immediate RX
start; `tx_fifo_buffers=0` restores the whole ring.

## Sample counter in the gateware (`gateware/sample_time.py`)

The FPGA itself keeps time in samples. The AD9361 core has a 64-bit counter in the RFIC clock domain
that advances on every PHY RX word, consumed or not, by the number of sample periods in the word (1
in 2R2T, 2 in 1R1T where a word is two consecutive samples). `ad9361.tick_control.timebase` selects
what frame stamps and gates use: `time_gen` nanoseconds (default, what the other tools expect) or the
counter. The plugin selects the counter whenever `timed_tx=hardware` (`fpga_timebase=ns` and
`tx_fine_gate=off` fall back to the previous behaviour).

* **RX.** Each word crosses the clock-domain FIFO and the RX pipeline register together with its tick.
  The header inserter waits for the first payload word of a frame and stamps the frame with *that
  word's* tick, so the stamp is the index of the frame's first sample even if samples were dropped
  inside the FPGA (the drop shows as a jump between consecutive stamps). The plugin uses the stamp as
  the label; its running count only reports discontinuities.
* **Timed RX start.** The inserter is held (and drains) until the last word before the requested tick
  has gone by; the first frame delivered *is* the requested sample.
* **TX.** `TimedTXGate` (sys) still decides per frame — hold while far in the future, drop when late
  or stale — but on ticks, and releases `advance` (64) samples early. It pushes one stamp per frame
  through a small clock-domain FIFO to `TXFineGate` in the RFIC domain, right in front of the PHY,
  which gives every word of a timed frame its own tick (stamp + position) and emits it in the PHY
  slot carrying that tick. A word whose slot has passed is discarded; an early word waits. The stream
  can therefore lose samples but is never shifted, which is why the late margin can be generous
  again (half a frame): a frame that arrives a little late is trimmed at its head and the rest goes
  out on its exact ticks.
* **Word parity.** In 1R1T frames and PHY words start on every other tick. The plugin makes the
  counter even while nothing streams and starts a burst one sample early with a zero if needed.
* **Word phase.** The PHY's TX word counter follows the RX word counter (aligned to the chip's
  RX_FRAME) instead of free-running, so the RX-sample-to-TX-slot relation is the same after every
  initialisation.

CSRs: `ad9361.tick_control {timebase, load, read, fine_enable}`, `tick_write`, `tick_read`,
`tick_status {tx_waiting, tx_trimmed}`, `timed_tx.advance`. libm2sdr: `m2sdr_set_sample_timebase()`,
`m2sdr_get/set_sample_time()`, `m2sdr_get_sample_time_status()`, `m2sdr_set_tx_timed_gate_advance()`.
All `timed_tx` / `timed_rx` values (stamps, margins, start time) are samples in this mode.

Simulation: `test/test_sample_time.py` — the fine gate for all 16 RX/TX strobe phase pairs, late and
interrupted frames, odd stamps, the RX chain with dropped samples, timed start, and the coarse + fine
TX chain.

Result (gateware v7, OCUDU with the stock lower PHY, `timed_tx=hardware,time_base=samples`):

* counter rate 23.040 MHz; the first RX frame is exactly the requested tick (= OCUDU's `init_time`);
* 8 of 8 starts with 0 non-contiguous RX labels, 0 FPGA discontinuities, 0 late UL requests;
* RX-minus-TX SSB offset (`align.py`, phone silent): **42 samples on 9 of 9 cold starts**, each
  including a re-initialisation of the AD9361 — with the nanosecond gate it was 45–46, and 42–43 before
  the TX word phase was locked to the RX word phase;
* gate counters `passed == 11272 frames/s`, `late == stale == 0`, fine gate `trimmed == 0`;
* timing met: WNS +0.038 ns overall and in the 245.76 MHz RFIC domain.

## Test procedure

1. `test/test_timed_tx_gate.py` — simulation (pass-through, untimed, on-time, hold, late, stale,
   in-margin, back-to-back); `test/test_timed_rx_start.py` — timed RX start against the real RX
   header inserter (transparent, first frame stamped with the start time, late arm, re-arm).
2. Build: `source /opt/Xilinx/2026.1/Vivado/settings64.sh && ~/litex-venv/bin/python litex_m2sdr.py
   --variant m2 --with-pcie --pcie-gen 2 --pcie-lanes 4 --build`; check the routed WNS in
   `build/<name>/gateware/vivado.log` (the baseline closes with ~+0.04 ns, there is little slack).
3. Flash (`m2sdr_util flash-write <_operational.bin>`, `flash-reload`, PCIe rescan), rebuild and reload
   the kernel module (`litex_m2sdr/software/kernel`, headers are regenerated by the build), rebuild
   `user/` and the Soapy module and install it.
4. RF-free: `software/soapysdr/test_hw_timed_tx.py --lead-ms 300 --reps 4 --late-test` (RFIC PHY
   loopback): bursts stamped 300 ms ahead must be held (`status.holding`, `armed_ts`), all 8176
   samples must arrive contiguous, the late burst must be dropped and counted. Ignore the printed
   "release latency" (see 6 above).
5. RF: OCUDU with `timed_tx=hardware` in `device_args`; `align.py`'s RX-minus-TX offset must be the
   same on every dump generation and every restart, and becomes the static RX timestamp correction.
   Watch the gate counters (`m2sdr_util reg-read 0x15808..`): `late` and `stale` must stay flat while
   the host streams; `held ≈ passed ≈ frame rate`.
