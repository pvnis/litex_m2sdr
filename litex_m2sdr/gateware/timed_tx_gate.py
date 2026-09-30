#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2026 Pavonis / Enjoy-Digital contributors.
# SPDX-License-Identifier: BSD-2-Clause

"""Hardware timed-TX gate.

Sits between the TX header extractor and the loopback/RFIC path. Every DMA buffer (frame) carries a
16-byte header (64-bit sync word, 64-bit timestamp in ns of the frame's first sample) that the header
extractor strips, exposing ``header``/``timestamp`` and framing the payload with ``first``/``last``.

The gate gives the TX path USRP-like semantics:

* HOLD  while ``time < timestamp``: the frame is not consumed (back-pressure stalls the DMA reader),
         nothing is emitted, so the RFIC PHY starves and outputs zeros until the release instant.
* PASS  when ``timestamp <= time <= timestamp + late_margin``: the frame streams straight through.
         In a continuous stream every frame after the first release arrives exactly on time.
* DROP  when ``time - timestamp > late_margin``: the whole frame is consumed and discarded and
         ``late_count`` increments. After a host stall the stream therefore re-aligns itself at the
         first on-time frame instead of shifting the timeline.

A frame with ``timestamp == 0`` (libm2sdr writes 0 when the caller passes no time) is untimed and
passes immediately, so untimed tools keep working with the gate enabled. With ``enable == 0`` the
module is a pure pass-through (today's behaviour).

Determinism note: the release-to-air latency is a fixed pipeline depth only if the downstream FIFOs are
empty at release, which a hold guarantees, and if the RFIC TX FIFO does not add priming hysteresis;
the SoC forces the RFIC ``tx_rfic_fifo_started`` flag while the gate is enabled.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream

from litepcie.common import dma_layout

# Timed TX Gate ------------------------------------------------------------------------------------

class TimedTXGate(LiteXModule):
    def __init__(self, data_width=64, with_csr=True):
        self.sink   = sink   = stream.Endpoint(dma_layout(data_width)) # i (from TX header extractor)
        self.source = source = stream.Endpoint(dma_layout(data_width)) # o (to loopback / RFIC)

        self.time          = Signal(64) # i: board time in ns (sys domain copy).
        self.timestamp     = Signal(64) # i: extractor's latched timestamp of the current frame.
        self.frames_active = Signal()   # i: extractor strips headers (frames carry first/last).
        self.reset         = Signal()   # i: re-synchronise (DMA reader restart).

        self.enable        = Signal()   # i (CSR): 1 = timed gating, 0 = pass-through.
        self.late_margin   = Signal(32) # i (CSR): ns a frame may be late and still be emitted.
        self.active        = Signal()   # o: gating in effect (enable & frames_active).

        # Status.
        self.late_count   = Signal(32)  # o: frames dropped because they were late.
        self.held_count   = Signal(32)  # o: frames that waited for their timestamp.
        self.passed_count = Signal(32)  # o: frames emitted.
        self.state        = Signal(2)   # o: 0=IDLE, 1=HOLD, 2=PASS, 3=DROP.
        self.armed_ts     = Signal(64)  # o: timestamp of the frame being held/passed.
        self.holding      = Signal()    # o: level, 1 while a frame is held.

        # # #

        self.comb += self.active.eq(self.enable & self.frames_active)

        # Timing decisions are registered (64-bit compare/subtract every cycle, one register stage) so
        # they never sit on a single-cycle path; the FSM spends one DECIDE cycle per frame to use them.
        lateness  = Signal(64)   # combinational: time - timestamp
        is_late   = Signal()
        is_future = Signal()
        is_timed  = Signal()
        self.comb += lateness.eq(self.time - self.timestamp)
        # All three decisions are registered in the same stage from the same-cycle inputs, so a frame is
        # never judged with another frame's timestamp.
        self.sync += [
            is_future.eq(self.time < self.timestamp),
            is_late.eq((self.time >= self.timestamp) & (lateness > self.late_margin)),
            is_timed.eq(self.timestamp != 0),
        ]

        # FSM -----------------------------------------------------------------------------------------
        self.fsm = fsm = ResetInserter()(FSM(reset_state="IDLE"))
        self.comb += fsm.reset.eq(self.reset)

        fsm.act("IDLE",
            self.state.eq(0),
            If(~self.active,
                # Pass-through: identical to a wire.
                sink.connect(source),
            ).Elif(sink.valid & sink.first,
                # First payload word of a frame: the extractor latched this frame's timestamp before
                # emitting its payload. Pause one cycle so the registered compares reflect it.
                NextValue(self.armed_ts, self.timestamp),
                NextState("DECIDE")
            ).Elif(sink.valid,
                # Not at a frame boundary (e.g. enabled mid-frame): drain to the next boundary.
                sink.ready.eq(1),
                If(sink.last, NextState("IDLE"))
            )
        )
        fsm.act("DECIDE",
            self.state.eq(1),
            # sink.ready = 0: the first word waits while the registered compares settle.
            If(~is_timed,
                NextState("PASS")
            ).Elif(is_future,
                NextValue(self.held_count, self.held_count + 1),
                NextState("HOLD")
            ).Elif(is_late,
                NextValue(self.late_count, self.late_count + 1),
                NextState("DROP")
            ).Else(
                NextState("PASS")
            )
        )
        fsm.act("HOLD",
            self.state.eq(1),
            self.holding.eq(1),
            # sink.ready = 0, source.valid = 0: back-pressure upstream, starve downstream (zeros).
            # is_future is registered: release lands one cycle after time reaches the stamp.
            If(~self.active,
                NextState("PASS")
            ).Elif(~is_future,
                NextState("PASS")
            )
        )
        fsm.act("PASS",
            self.state.eq(2),
            sink.connect(source),
            If(sink.valid & sink.ready & sink.last,
                NextValue(self.passed_count, self.passed_count + 1),
                NextState("IDLE")
            )
        )
        fsm.act("DROP",
            self.state.eq(3),
            sink.ready.eq(1),
            If(sink.valid & sink.last,
                NextState("IDLE")
            )
        )

        if with_csr:
            self.add_csr()

    def add_csr(self, default_enable=0, default_late_margin_ns=100_000):
        self._control = CSRStorage(fields=[
            CSRField("enable", size=1, offset=0, values=[
                ("``0b0``", "Pass-through (software-timed TX)."),
                ("``0b1``", "Hardware timed TX: hold/pass/drop frames on their header timestamp."),
            ], reset=default_enable),
            CSRField("reset_counts", size=1, offset=1, pulse=True, description="Clear the counters."),
        ])
        self._late_margin  = CSRStorage(32, reset=default_late_margin_ns,
            description="Late margin in ns: a frame older than this at arrival is dropped.")
        self._late_count   = CSRStatus(32, description="Frames dropped as late.")
        self._held_count   = CSRStatus(32, description="Frames held until their timestamp.")
        self._passed_count = CSRStatus(32, description="Frames emitted.")
        self._status       = CSRStatus(fields=[
            CSRField("state",   size=2, offset=0, description="0=IDLE, 1=HOLD, 2=PASS, 3=DROP."),
            CSRField("active",  size=1, offset=2, description="Gating in effect."),
            CSRField("holding", size=1, offset=3, description="A frame is currently held."),
        ])
        self._armed_ts     = CSRStatus(64, description="Timestamp (ns) of the frame being held/emitted.")

        self.comb += [
            self.enable.eq(self._control.fields.enable),
            self.late_margin.eq(self._late_margin.storage),
            self._late_count.status.eq(self.late_count),
            self._held_count.status.eq(self.held_count),
            self._passed_count.status.eq(self.passed_count),
            self._status.fields.state.eq(self.state),
            self._status.fields.active.eq(self.active),
            self._status.fields.holding.eq(self.holding),
            self._armed_ts.status.eq(self.armed_ts),
        ]
        self.sync += If(self._control.fields.reset_counts,
            self.late_count.eq(0),
            self.held_count.eq(0),
            self.passed_count.eq(0),
        )
