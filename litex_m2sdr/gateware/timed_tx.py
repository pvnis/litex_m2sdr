#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
Timed TX Arbiter
================

Sample-accurate TX gating based on a sample-domain reference counter.
Sits between :class:`~litex_m2sdr.gateware.header.TXHeaderExtractor` and
the AD9361 TX pipeline, in the sys clock domain. Watches the per-frame
HAS_TIME and END_BURST flags carried in the 16-byte DMA header to build
burst descriptors, then holds the sample stream in a FIFO until the
sample counter reaches the burst's start timestamp.

Design notes
------------
* **Sample-count time base.** Comparison is `sample_count >= ts` with
  both sides being 64-bit unsigned sample indices. Sample-count math is
  exact, with no fractional ns conversion. The reference comes from
  :class:`SampleCounter` (CDC'd from rfic via continuous Gray code).
* **Per-burst descriptors.** A descriptor is one entry of `(ts,
  has_time, frame_count)` pushed into a small FIFO from the descriptor
  builder. Untimed continuous bursts are encoded as `has_time=0`,
  `frame_count=1` and flow through immediately.
* **Late drop.** When `has_time=1` and `sample_count > ts` at descriptor
  latch time, the burst is dropped (all its frames drained from
  `data_fifo`) and `late_count` is incremented.
* **Gap fill.** Between timed bursts, the arbiter optionally emits zero
  samples so the DAC stays fed and the host doesn't have to pad gaps
  with untimed frames.
* **FIFO sizing.** `data_fifo` defaults to depth 512 (matches the
  validated depth from dmd1 - 2048 caused timing closure failures on
  Artix-7). `ts_fifo` is 32 entries (one entry per burst).

Burst convention (host side)
----------------------------
Every DMA buffer carries a 16-byte header. The two MSBs of the first
word encode:

    bit 63: HAS_TIME    (this frame opens a timed burst; ts in word 1)
    bit 62: END_BURST   (this frame closes the current burst)

A burst is a sequence of one or more frames. The arbiter recognises:

    Single-frame timed:   HAS_TIME=1, END_BURST=1, ts=...
    Multi-frame timed:    first HAS_TIME=1, last END_BURST=1
    Multi-frame untimed:  HAS_TIME=0, ..., END_BURST=1
    Untimed standalone:   neither bit set, treated as single-frame
                          untimed burst (immediate pass-through).
"""

from migen import *

from litex.gen import *

from litepcie.common import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *


# Timed TX Arbiter ---------------------------------------------------------------------------------

class TimedTXArbiter(LiteXModule):
    """Sample-accurate TX gating using a sample-domain reference counter."""

    def __init__(self, header, sample_count,
                 data_width=64, data_fifo_depth=512, ts_fifo_depth=32):
        self.sink   = sink   = stream.Endpoint(dma_layout(data_width))
        self.source = source = stream.Endpoint(dma_layout(data_width))

        # Reset input: flushes data_fifo + ts_fifo + builder state so
        # stale data from a previous run can't block the arbiter.
        self.reset = Signal()

        # CSRs --------------------------------------------------------------------------------------
        self._enable = CSRStorage(1, reset=1,
            description="Timed TX enable. 0 = bypass (pure pass-through), 1 = active gating.")
        self._gap_fill = CSRStorage(1, reset=1,
            description="Emit zero samples while idle/waiting between timed bursts.")
        self._reset_counts = CSRStorage(1, reset=0,
            description="Write 1 to clear all status counters atomically.")
        self._late_count = CSRStatus(32,
            description="Bursts dropped because sample_count > ts at descriptor latch.")
        self._underrun_count = CSRStatus(32,
            description="Underruns inside a streaming burst (data_fifo went empty mid-frame).")
        self._drop_count = CSRStatus(32,
            description="Total bursts drained from data_fifo without transmission.")
        self._state = CSRStatus(2,
            description="FSM state: 0=IDLE, 1=WAIT, 2=STREAM, 3=DROP.")
        self._armed_ts = CSRStatus(64,
            description="Currently-armed burst start timestamp (sample count).")
        self._armed_has_time = CSRStatus(1,
            description="HAS_TIME bit of the currently-armed descriptor.")
        self._ts_pop_count = CSRStatus(32,
            description="Total number of burst descriptors latched (running total).")
        self._reset = CSRStorage(1, reset=0,
            description="Write 1 to flush timed-TX FIFOs/FSM state and clear status counters.")

        reset_req = Signal()
        self.comb += reset_req.eq(self.reset | (self._reset.re & self._reset.storage))

        # # #

        # Decode flags from the header word.
        # The header word is supplied as `header.header` (lower 64 bits of the
        # DMA header). Bit 63 = HAS_TIME, bit 62 = END_BURST.
        has_time_in  = header.header[63]

        # Data FIFO ---------------------------------------------------------------------------------
        # Buffers sample frames so the arbiter can pace release.
        self.data_fifo = data_fifo = ResetInserter()(
            stream.SyncFIFO(dma_layout(data_width), data_fifo_depth))
        self.comb += data_fifo.reset.eq(reset_req)
        self.comb += sink.connect(data_fifo.sink)

        # Burst descriptor FIFO ---------------------------------------------------------------------
        # One entry per DMA frame.
        self.ts_fifo = ts_fifo = ResetInserter()(stream.SyncFIFO([
            ("ts",       64),
            ("has_time",  1),
        ], ts_fifo_depth))
        self.comb += ts_fifo.reset.eq(reset_req)

        # Status counters / latched state -----------------------------------------------------------
        late_count          = Signal(32)
        underrun_count      = Signal(32)
        drop_count          = Signal(32)
        ts_pop_count        = Signal(32)
        ts_reg              = Signal(64)
        ts_has_time_r       = Signal()
        burst_seen          = Signal()
        gap_fill_active     = Signal()
        state_code          = Signal(2)
        data_fifo_was_valid = Signal()

        self.comb += [
            self._late_count.status.eq(late_count),
            self._underrun_count.status.eq(underrun_count),
            self._drop_count.status.eq(drop_count),
            self._state.status.eq(state_code),
            self._armed_ts.status.eq(ts_reg),
            self._armed_has_time.status.eq(ts_has_time_r),
            self._ts_pop_count.status.eq(ts_pop_count),
        ]

        # Counter-reset CSR clears status; full reset also flushes FIFOs/FSM.
        self.sync += If((self._reset_counts.re & self._reset_counts.storage) | reset_req,
            late_count.eq(0),
            underrun_count.eq(0),
            drop_count.eq(0),
            ts_pop_count.eq(0),
            ts_reg.eq(0),
            ts_has_time_r.eq(0),
            burst_seen.eq(0),
            gap_fill_active.eq(0),
        )

        # Descriptor builder ------------------------------------------------------------------------
        # Push one descriptor per DMA frame as soon as the header is
        # extracted. Waiting for END_BURST before producing a descriptor
        # would require buffering an entire burst before the FSM can
        # release any samples; long timed bursts can exceed data_fifo
        # depth and deadlock the TX path.
        self.sync += [
            ts_fifo.sink.valid.eq(0),
            If(header.update,
                ts_fifo.sink.valid.eq(1),
                ts_fifo.sink.ts.eq(header.timestamp),
                ts_fifo.sink.has_time.eq(has_time_in),
            ),
        ]

        # Underrun edge detection.
        self.sync += data_fifo_was_valid.eq(data_fifo.source.valid)

        # FSM ---------------------------------------------------------------------------------------
        self.fsm = fsm = FSM(reset_state="IDLE")
        self.comb += [
            If(fsm.ongoing("IDLE"),       state_code.eq(0)
            ).Elif(fsm.ongoing("WAIT"),   state_code.eq(1)
            ).Elif(fsm.ongoing("STREAM"), state_code.eq(2)
            ).Else(state_code.eq(3))
        ]

        def latch_desc():
            return [
                ts_fifo.source.ready.eq(1),
                NextValue(ts_reg, ts_fifo.source.ts),
                NextValue(ts_has_time_r, ts_fifo.source.has_time),
                NextValue(burst_seen, 0),
                NextValue(ts_pop_count, ts_pop_count + 1),
            ]

        fsm.act("IDLE",
            If(~self._enable.storage,
                # Bypass: arbiter disabled, pure pass-through.
                data_fifo.source.connect(source),
            ).Elif(ts_fifo.source.valid &
                   data_fifo.source.valid & data_fifo.source.first,
                # New burst ready: latch its descriptor and decide.
                *latch_desc(),
                If(ts_fifo.source.has_time & (sample_count > ts_fifo.source.ts),
                    # Already past the requested time -> drop the burst.
                    NextValue(late_count, late_count + 1),
                    NextState("DROP"),
                ).Elif(~ts_fifo.source.has_time,
                    NextValue(gap_fill_active, 0),
                    NextState("STREAM"),
                ).Else(
                    NextValue(gap_fill_active, 1),
                    NextState("WAIT"),
                ),
            ).Elif(self._gap_fill.storage & gap_fill_active,
                # Keep the DAC fed with zeros between timed bursts.
                source.valid.eq(1),
                source.first.eq(0),
                source.last.eq(0),
                source.data.eq(0),
            ),
        )

        fsm.act("WAIT",
            If(reset_req,
                NextState("IDLE"),
            ).Else(
                If(self._gap_fill.storage,
                    source.valid.eq(1),
                    source.first.eq(0),
                    source.last.eq(0),
                    source.data.eq(0),
                ),
                If(sample_count >= ts_reg,
                    NextState("STREAM"),
                ),
            ),
        )

        fsm.act("STREAM",
            If(reset_req,
                NextState("IDLE"),
            ).Else(
                data_fifo.source.connect(source),
                If(data_fifo.source.valid & source.ready,
                    NextValue(burst_seen, 1),
                ),
                # Underrun: streaming has started and data_fifo went empty
                # mid-burst.
                If(burst_seen & data_fifo_was_valid & ~data_fifo.source.valid,
                    NextValue(underrun_count, underrun_count + 1),
                ),
                If(source.valid & source.ready & source.last,
                    NextValue(gap_fill_active, ts_has_time_r),
                    NextState("IDLE"),
                ),
            ),
        )

        fsm.act("DROP",
            If(reset_req,
                NextState("IDLE"),
            ).Else(
                # Drain the dropped DMA frame from the data FIFO.
                data_fifo.source.ready.eq(1),
                If(data_fifo.source.valid & data_fifo.source.last,
                    NextValue(drop_count, drop_count + 1),
                    NextValue(gap_fill_active, 0),
                    NextState("IDLE"),
                ),
            ),
        )
