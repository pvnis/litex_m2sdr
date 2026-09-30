#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2026 Pavonis / Enjoy-Digital contributors.
# SPDX-License-Identifier: BSD-2-Clause

"""Timed RX start.

Gives the RX stream the ``stream_cmd(time_spec, stream_now=False)`` semantics of a USRP: RX samples are
discarded until board time reaches ``start_time``, and the first DMA frame delivered to the host is the
one whose header timestamp is ``start_time``.

It does so by holding the RX header inserter in reset while closed: in reset the inserter drains its
sink (samples are dropped) and emits nothing, so the DMA writer sees no data. On opening, the inserter
restarts on a frame boundary and stamps the first frame. ``lead`` pre-compensates the fixed latency
between the compare and the cycle the inserter samples the time (5 sys cycles), so that the first
stamp equals ``start_time`` to within one time step.

Applications that derive their whole timeline from "RX begins at the time I asked for" (srsRAN/OCUDU
lower PHY: ``last_rx_timestamp = init_time``) need this; with an immediate start their DL timeline is
seeded from a time that is not where RX really begins.

With ``enable == 0`` the module is transparent (RX starts as soon as the DMA writer does).
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# Timed RX Start -----------------------------------------------------------------------------------

class TimedRXStart(LiteXModule):
    def __init__(self, with_csr=True, default_lead_ns=40):
        self.time       = Signal(64) # i: board time in ns (sys domain copy).
        self.enable     = Signal()   # i (CSR): 1 = hold RX until start_time; 0 = transparent.
        self.start_time = Signal(64) # i (CSR): board time (ns) of the first RX frame.
        self.lead       = Signal(32, reset=default_lead_ns) # i (CSR): compare-to-stamp latency (ns).

        # Sample-count time base (tick_mode = 1): start_time is a sample index. The inserter drains
        # its sink while held; ``sink_tick`` is the tick of the word it is draining and ``sink_fire``
        # pulses when it takes it. The gate opens after the last word before start_time, so the first
        # word kept -- the first payload word of the first frame -- is the sample start_time itself.
        self.tick_mode  = Signal()   # i
        self.sink_tick  = Signal(64) # i
        self.sink_fire  = Signal()   # i
        self.inc        = Signal(2)  # i: sample periods per word.

        self.hold       = Signal()   # o: keep the RX header inserter in reset (samples dropped).
        self.opened     = Signal()   # o: start time reached since the last arm.
        self.late       = Signal()   # o: start_time was already in the past when armed.
        self.open_time  = Signal(64) # o: board time at which the gate opened (tick mode: last word discarded).

        # # #

        # Registered compare (64-bit add + compare, never on a single-cycle path with the FSM).
        reached   = Signal()
        enable_d  = Signal()
        last_tick = Signal(64)   # tick of the last word to discard (registered: one compare per path).
        self.sync += [
            last_tick.eq(self.start_time - self.inc),
            reached.eq((self.time + self.lead) >= self.start_time),
            enable_d.eq(self.enable),
            If(~self.enable,
                self.opened.eq(0),
                self.late.eq(0),
            ).Elif(~self.opened,
                If(self.tick_mode,
                    # The word being drained is the last one before start_time (or we are past it).
                    If(self.sink_fire & (self.sink_tick >= last_tick),
                        self.opened.eq(1),
                        self.open_time.eq(self.sink_tick),
                        If(self.sink_tick >= self.start_time, self.late.eq(1)),
                    )
                ).Elif(reached,
                    self.opened.eq(1),
                    self.open_time.eq(self.time),
                    # Armed (enable rose one cycle ago) with the start time already behind us.
                    If(~enable_d, self.late.eq(1)),
                )
            ),
        ]
        self.comb += self.hold.eq(self.enable & ~self.opened)

        if with_csr:
            self.add_csr(default_lead_ns)

    def add_csr(self, default_lead_ns):
        self._control = CSRStorage(fields=[
            CSRField("enable", size=1, offset=0, values=[
                ("``0b0``", "Transparent: RX starts with the DMA writer."),
                ("``0b1``", "Armed: RX samples are dropped until board time reaches start_time."),
            ]),
        ])
        self._start_time = CSRStorage(64, description="Board time (ns) of the first RX frame.")
        self._lead       = CSRStorage(32, reset=default_lead_ns,
            description="Latency (ns) between the time compare and the header timestamp sample.")
        self._status     = CSRStatus(fields=[
            CSRField("opened", size=1, offset=0, description="Start time reached; RX is flowing."),
            CSRField("hold",   size=1, offset=1, description="Armed and waiting for start_time."),
            CSRField("late",   size=1, offset=2, description="start_time was in the past when armed."),
        ])
        self._open_time  = CSRStatus(64, description="Board time (ns) at which the gate opened.")

        self.comb += [
            self.enable.eq(self._control.fields.enable),
            self.start_time.eq(self._start_time.storage),
            self.lead.eq(self._lead.storage),
            self._status.fields.opened.eq(self.opened),
            self._status.fields.hold.eq(self.hold),
            self._status.fields.late.eq(self.late),
            self._open_time.status.eq(self.open_time),
        ]
