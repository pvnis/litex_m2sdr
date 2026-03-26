#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litepcie.common import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

# Timed TX Arbiter ---------------------------------------------------------------------------------

class TimedTXArbiter(LiteXModule):
    """
    Holds TX payload packets until TimeGenerator.time >= the packet's TX timestamp.
    After a timed burst has completed, the arbiter can synthesize zero-valued
    samples while idle/waiting so host software does not need to pad UL gaps
    with untimed DMA frames that would otherwise sit ahead of future timed
    burst starts in the serialized FIFO.

    Placement: after the TX header extractor. The extractor strips the DMA header
    (sync word + timestamp) from the stream and exposes the timestamp via its
    `timestamp` signal, pulsing `update` once per frame when a new timestamp is latched.

    A companion burst-descriptor FIFO captures one entry per logical burst:
        - burst start timestamp (from the first DMA frame with HAS_TIME=1)
        - whether the burst is timed
        - how many DMA frames belong to the burst

    Soapy timed bursts are encoded as:
        - first DMA frame:  HAS_TIME=1, timestamp=start time
        - middle frames:    HAS_TIME=0
        - last DMA frame:   END_BURST=1

    The arbiter therefore schedules an entire burst as the unit of release. Once
    a burst starts, all of its DMA frames stream straight through until the
    matching END_BURST. If the timed start is already late, the entire burst is
    dropped.

    FSM:
        IDLE      -- wait for a payload frame AND a matching timestamp in ts_fifo
        WAIT_TIME -- hold until time_gen.time >= ts_reg (optionally emit zeros)
        STREAM    -- forward samples to output until last=1
    """
    def __init__(self, header_extractor, time_gen, fifo_depth=2048, data_width=64, ts_fifo_depth=32):
        self.sink   = sink   = stream.Endpoint(dma_layout(data_width))
        self.source = source = stream.Endpoint(dma_layout(data_width))

        # Reset input: when asserted, flushes data_fifo and ts_fifo so that stale
        # data from a previously killed or failed TX run cannot block the arbiter.
        self.reset = reset = Signal()

        # CSRs.
        self._enable         = CSRStorage(1, reset=0,
            description="Timed TX enable: 1=hold until timestamp, 0=pass-through.")
        self._late_count     = CSRStatus(32,
            description="TX frames with HAS_TIME that arrived after their deadline.")
        self._underrun_count = CSRStatus(32,
            description="TX underrun events (FIFO transitioned from data-available to empty).")
        self._state          = CSRStatus(2,
            description="Timed TX FSM state: 0=IDLE, 1=WAIT_TIME, 2=STREAM, 3=DROP.")
        self._armed_ts       = CSRStatus(64,
            description="Timestamp currently armed for release by the Timed TX arbiter.")
        self._armed_has_time = CSRStatus(1,
            description="HAS_TIME bit associated with the currently armed timestamp.")
        self._ts_pop_count   = CSRStatus(32,
            description="Number of timestamp FIFO entries consumed by the Timed TX arbiter.")
        self._last_late_ts       = CSRStatus(64,
            description="Timestamp of the most recent frame flagged late.")
        self._last_late_time     = CSRStatus(64,
            description="time_gen.time captured when the most recent late frame was detected.")
        self._last_late_has_time = CSRStatus(1,
            description="HAS_TIME bit associated with the most recent late frame.")
        self._last_late_ts_pop   = CSRStatus(32,
            description="ts_pop_count captured when the most recent late frame was detected.")
        self._last_late_state    = CSRStatus(2,
            description="FSM state captured when the most recent late frame was detected.")
        self._reset_counts   = CSRStorage(1, reset=0,
            description="Write 1 to clear timed-TX counters/debug state atomically.")
        self._gap_fill       = CSRStorage(1, reset=1,
            description="When enabled, emit zero-valued samples between timed bursts instead of requiring host-padded untimed DMA frames.")

        # # #

        # Data FIFO: buffers incoming payload samples (first/last framed).
        # Wrapped with ResetInserter so stale data is cleared on reset.
        self.data_fifo = data_fifo = ResetInserter()(stream.SyncFIFO(dma_layout(data_width), fifo_depth))
        self.comb += [
            data_fifo.reset.eq(reset),
            sink.connect(data_fifo.sink),
        ]

        # Burst descriptor FIFO: one slot per logical burst.
        self.ts_fifo = ts_fifo = ResetInserter()(stream.SyncFIFO([
            ("ts",         64),
            ("has_time",    1),
            ("frame_count", 16),
        ], ts_fifo_depth))
        self.comb += ts_fifo.reset.eq(reset)

        # Burst-active status (for TDDSwitch).
        self.tx_burst_active = Signal()

        # Registers.
        ts_reg              = Signal(64)
        ts_has_time_reg     = Signal()
        burst_frames_reg    = Signal(16)
        burst_seen_data     = Signal()
        burst_timed_active  = Signal()
        late_count          = Signal(32)
        underrun_count      = Signal(32)
        ts_pop_count        = Signal(32)
        last_late_ts        = Signal(64)
        last_late_time      = Signal(64)
        last_late_has_time  = Signal()
        last_late_ts_pop    = Signal(32)
        last_late_state     = Signal(2)
        data_fifo_was_valid = Signal()  # previous-cycle valid, for underrun edge detection
        gap_fill_active     = Signal()
        state_code          = Signal(2)
        desc_build_open     = Signal()
        desc_build_ts       = Signal(64)
        desc_build_has_time = Signal()
        desc_build_count    = Signal(16)
        self.sync += data_fifo_was_valid.eq(data_fifo.source.valid)

        self.comb += [
            self._late_count.status.eq(late_count),
            self._underrun_count.status.eq(underrun_count),
            self._state.status.eq(state_code),
            self._armed_ts.status.eq(ts_reg),
            self._armed_has_time.status.eq(ts_has_time_reg),
            self._ts_pop_count.status.eq(ts_pop_count),
            self._last_late_ts.status.eq(last_late_ts),
            self._last_late_time.status.eq(last_late_time),
            self._last_late_has_time.status.eq(last_late_has_time),
            self._last_late_ts_pop.status.eq(last_late_ts_pop),
            self._last_late_state.status.eq(last_late_state),
        ]

        # Counter reset: writing 1 to _reset_counts clears both counters atomically.
        self.sync += If(self._reset_counts.re & self._reset_counts.storage,
            late_count.eq(0),
            underrun_count.eq(0),
            ts_pop_count.eq(0),
            ts_reg.eq(0),
            ts_has_time_reg.eq(0),
            burst_frames_reg.eq(0),
            last_late_ts.eq(0),
            last_late_time.eq(0),
            last_late_has_time.eq(0),
            last_late_ts_pop.eq(0),
            last_late_state.eq(0),
            gap_fill_active.eq(0),
            burst_timed_active.eq(0),
            desc_build_open.eq(0),
            desc_build_ts.eq(0),
            desc_build_has_time.eq(0),
            desc_build_count.eq(0),
        )

        # Build one descriptor per logical burst from the per-frame header flags.
        self.sync += If(reset,
            desc_build_open.eq(0),
            desc_build_ts.eq(0),
            desc_build_has_time.eq(0),
            desc_build_count.eq(0),
            ts_fifo.sink.valid.eq(0),
        ).Else(
            ts_fifo.sink.valid.eq(0),
            If(header_extractor.update,
                If(header_extractor.tx_flags[15],
                    # Timed start frame always opens a new burst descriptor.
                    desc_build_open.eq(~header_extractor.tx_flags[14]),
                    desc_build_ts.eq(header_extractor.timestamp),
                    desc_build_has_time.eq(1),
                    desc_build_count.eq(1),
                    If(header_extractor.tx_flags[14],
                        ts_fifo.sink.valid.eq(1),
                        ts_fifo.sink.ts.eq(header_extractor.timestamp),
                        ts_fifo.sink.has_time.eq(1),
                        ts_fifo.sink.frame_count.eq(1),
                    )
                ).Elif(desc_build_open,
                    # Continuation frame of an open burst.
                    If(header_extractor.tx_flags[14],
                        ts_fifo.sink.valid.eq(1),
                        ts_fifo.sink.ts.eq(desc_build_ts),
                        ts_fifo.sink.has_time.eq(desc_build_has_time),
                        ts_fifo.sink.frame_count.eq(desc_build_count + 1),
                        desc_build_open.eq(0),
                        desc_build_count.eq(0),
                    ).Else(
                        desc_build_count.eq(desc_build_count + 1),
                    )
                ).Else(
                    # Untimed frame with no open burst: treat as an immediate
                    # single-frame burst so untimed continuous TX still passes.
                    ts_fifo.sink.valid.eq(1),
                    ts_fifo.sink.ts.eq(0),
                    ts_fifo.sink.has_time.eq(0),
                    ts_fifo.sink.frame_count.eq(1),
                )
            )
        )

        # FSM.
        self.fsm = fsm = FSM(reset_state="IDLE")

        def latch_desc():
            return [
                ts_fifo.source.ready.eq(1),
                NextValue(ts_reg, ts_fifo.source.ts),
                NextValue(ts_has_time_reg, ts_fifo.source.has_time),
                NextValue(burst_frames_reg, ts_fifo.source.frame_count),
                NextValue(ts_pop_count, ts_pop_count + 1),
            ]

        self.comb += [
            If(fsm.ongoing("IDLE"),
                state_code.eq(0)
            ).Elif(fsm.ongoing("WAIT_TIME"),
                state_code.eq(1)
            ).Elif(fsm.ongoing("STREAM"),
                state_code.eq(2)
            ).Else(
                state_code.eq(3)
            )
        ]

        fsm.act("IDLE",
            If(~self._enable.storage,
                # Pass-through: bypass timestamp check, forward data directly.
                data_fifo.source.connect(source),
            ).Elif(data_fifo.source.valid & data_fifo.source.first & ts_fifo.source.valid,
                # New burst present with a matching descriptor: latch and decide.
                *latch_desc(),
                NextValue(burst_seen_data, 0),
                # Count timed frames (HAS_TIME=1) that were already past their deadline.
                If(ts_fifo.source.has_time & (time_gen.time > ts_fifo.source.ts),
                    NextValue(late_count, late_count + 1),
                    NextValue(last_late_ts, ts_fifo.source.ts),
                    NextValue(last_late_time, time_gen.time),
                    NextValue(last_late_has_time, ts_fifo.source.has_time),
                    NextValue(last_late_ts_pop, ts_pop_count + 1),
                    NextValue(last_late_state, state_code),
                    NextValue(burst_timed_active, ts_fifo.source.has_time),
                    NextState("DROP"),
                ).Elif(~ts_fifo.source.has_time,
                    NextValue(gap_fill_active, 0),
                    NextValue(burst_timed_active, 0),
                    NextState("STREAM"),
                ).Else(
                    NextValue(burst_timed_active, 1),
                    NextState("WAIT_TIME"),
                ),
            ).Elif(self._gap_fill.storage & gap_fill_active,
                # Keep the DAC path fed with zeros between timed bursts so the
                # host does not need to enqueue untimed padding frames.
                source.valid.eq(1),
                source.first.eq(0),
                source.last.eq(0),
                source.data.eq(0),
            )
        )

        fsm.act("WAIT_TIME",
            If(self._gap_fill.storage & gap_fill_active,
                source.valid.eq(1),
                source.first.eq(0),
                source.last.eq(0),
                source.data.eq(0),
            ),
            # Release when the hardware clock reaches (or passes) the TX timestamp.
            If(time_gen.time >= ts_reg,
                NextState("STREAM"),
            )
        )

        fsm.act("STREAM",
            self.tx_burst_active.eq(1),
            data_fifo.source.connect(source),
            If(data_fifo.source.valid & source.ready,
                NextValue(burst_seen_data, 1),
            ),
            # Count underruns only when a burst starves after streaming started.
            If(burst_seen_data & data_fifo_was_valid & ~data_fifo.source.valid,
                NextValue(underrun_count, underrun_count + 1),
            ),
            If(source.valid & source.ready & source.last,
                If(burst_frames_reg == 1,
                    NextValue(gap_fill_active, burst_timed_active | ts_has_time_reg),
                    NextValue(burst_timed_active, 0),
                    NextValue(burst_frames_reg, 0),
                    NextState("IDLE"),
                ).Else(
                    NextValue(burst_frames_reg, burst_frames_reg - 1),
                )
            )
        )

        fsm.act("DROP",
            data_fifo.source.ready.eq(1),
            If(data_fifo.source.valid & data_fifo.source.ready & data_fifo.source.last,
                If(burst_frames_reg == 1,
                    NextValue(burst_timed_active, 0),
                    NextValue(gap_fill_active, 0),
                    NextValue(burst_frames_reg, 0),
                    NextState("IDLE"),
                ).Else(
                    NextValue(burst_frames_reg, burst_frames_reg - 1),
                )
            )
        )
