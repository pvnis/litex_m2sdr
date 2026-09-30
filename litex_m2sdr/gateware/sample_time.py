#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2026 Pavonis / Enjoy-Digital contributors.
# SPDX-License-Identifier: BSD-2-Clause

"""Sample-count time base.

Time is the index of a sample ("tick"), counted in the RFIC clock domain by the AD9361 core: the
counter advances by the number of sample periods in one PHY word (1 in 2R2T, 2 in 1R1T) on every RX
word strobe, whether or not the word is consumed. It is the only clock the ADC and DAC really share,
so stamping RX frames and releasing TX frames against it is exact by construction -- no nanosecond
conversion and no second oscillator involved (what a USRP calls ticks).

* ``RXTickTracker`` (sys): each RX word crosses the clock-domain FIFO and the RX pipeline register
  together with its tick. The tracker strips it and exposes, for the word currently offered to the
  consumer (the RX header inserter), the tick it was sampled at. A frame header stamped with that value is the index of the
  frame's first sample even if samples were dropped upstream.

* ``TXFineGate`` (rfic): emits every word of a timed frame in the PHY slot whose tick equals the
  word's own tick (frame stamp + position). Words that arrive after their slot are discarded, words
  that are early wait; the stream is therefore never shifted, only trimmed. The frame stamps reach it
  through a small clock-domain FIFO, one entry per frame, pushed by the sys-domain TimedTXGate, which
  still does the whole-frame hold / late / stale decisions and releases frames a little early.
"""

from migen import *

from litex.gen import *

from litex.soc.interconnect import stream

from litepcie.common import dma_layout

# Layouts ------------------------------------------------------------------------------------------

def rx_tick_layout():
    return [("data", 64), ("tick", 64)]

def tx_stamp_layout():
    return [("timed", 1), ("tick", 64)]

# RX Tick Tracker (sys) ----------------------------------------------------------------------------

class RXTickTracker(LiteXModule):
    """Strips the tick off the RX words and exposes the tick of the word currently offered downstream.

    It sits right after the RX pipeline register (which carries data and tick together) and in front
    of the bit-mode stage. In the 12-bit/SC16 format that stage is a wire, so the word offered to the
    final consumer (the RX header inserter) is the word offered here and ``tick`` is its sample index.
    In the repacking formats (8-bit, BFP8) words are not 1:1 and ``tick`` falls back to ``now``.
    """
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(rx_tick_layout())  # i (from the RX pipeline register)
        self.source = source = stream.Endpoint(dma_layout(64))    # o (to bit-mode / consumer)

        self.inc   = Signal(2)  # i: sample periods per word.
        self.exact = Signal()   # i: the consumer sees these words 1:1 (12-bit/SC16 mode).

        self.tick  = Signal(64) # o: tick of the word offered to the consumer.
        self.now   = Signal(64) # o: tick following the newest word taken (sys view of now).

        # # #

        self.comb += [
            sink.connect(source, omit={"tick"}),
            If(self.exact & sink.valid,
                self.tick.eq(sink.tick)
            ).Else(
                self.tick.eq(self.now)
            ),
        ]
        self.sync += If(sink.valid & sink.ready, self.now.eq(sink.tick + self.inc))

# TX Fine Gate (rfic) ------------------------------------------------------------------------------

class TXFineGate(LiteXModule):
    """Runs in the clock domain it is renamed to (rfic). ``source.ready`` is the PHY slot strobe."""
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(dma_layout(64))     # i (TX words, ``first`` on frame starts)
        self.source = source = stream.Endpoint(dma_layout(64))     # o (to the PHY)
        self.stamp  = stamp  = stream.Endpoint(tx_stamp_layout())  # i (one entry per frame)

        self.enable  = Signal()    # i
        self.tick    = Signal(64)  # i: sample counter.
        self.inc     = Signal(2)   # i: sample periods per word.

        self.trimmed = Signal(16)  # o: words discarded because their slot had passed (wraps).
        self.waiting = Signal()    # o: a timed word is waiting for its slot.

        # # #

        # Frame context. Ticks are compared on 32 bits: the sys-domain gate only lets through frames
        # that are within its margins of now, far inside +-2^31 samples.
        timed     = Signal()     # current frame is timed.
        loaded    = Signal()     # the stamp of the first word at the head has been loaded.
        head      = Signal(32)   # tick of the word at the head of the sink.
        slot      = Signal(32)   # tick the next PHY slot will carry.
        diff      = Signal(32)   # head - slot (combinational).
        on_slot   = Signal()     # registered: the head word belongs in the next slot.
        is_late   = Signal()     # registered: its slot has passed.
        diff_ok   = Signal()     # the registered flags reflect the current head and slot.

        is_first   = Signal()
        need_stamp = Signal()
        ctx_ready  = Signal()
        emit       = Signal()
        drop       = Signal()
        pop        = Signal()
        slot_upd   = Signal()
        load       = Signal()

        self.comb += [
            is_first.eq(sink.valid & sink.first),
            need_stamp.eq(is_first & ~loaded),
            load.eq(self.enable & need_stamp & stamp.valid),
            # A first word needs its stamp; the following words inherit the frame context.
            ctx_ready.eq(sink.valid & (~sink.first | loaded)),
            slot_upd.eq(source.ready),
        ]

        self.comb += [
            If(~self.enable,
                # Transparent; discard stamps so the two streams cannot get out of step.
                sink.connect(source),
                stamp.ready.eq(1),
            ).Else(
                stamp.ready.eq(load),
                If(ctx_ready & ~timed,
                    # Untimed frame: straight through.
                    sink.connect(source),
                ).Elif(ctx_ready & timed & diff_ok,
                    If(on_slot,
                        emit.eq(1),
                    ).Elif(is_late,
                        drop.eq(1),
                    ),
                ),
                If(emit,
                    sink.connect(source),
                ),
                If(drop,
                    sink.ready.eq(1),
                ),
            ),
            pop.eq(sink.valid & sink.ready),
            self.waiting.eq(self.enable & ctx_ready & timed & diff_ok & ~on_slot & ~is_late),
            diff.eq(head - slot),
        ]

        self.sync += [
            # The tick of the next slot: what the counter shows at this slot, plus one word. Predicted
            # rather than read so it is stable for the whole inter-slot interval whatever the phase
            # between the RX word strobe (which advances the counter) and the TX slot strobe.
            If(slot_upd,
                slot.eq(self.tick[:32] + self.inc)
            ),
            # Only single-bit flags leave this stage (the RFIC clock can be 245 MHz). A word whose tick
            # falls inside the slot's word also belongs to it: in 1R1T a word is two samples, so an odd
            # frame stamp is emitted one sample late instead of never matching.
            on_slot.eq((diff == 0) | ((self.inc == 2) & (diff == 0xffffffff))),
            is_late.eq(diff[31] & ~((self.inc == 2) & (diff == 0xffffffff))),
            diff_ok.eq(~(pop | slot_upd | load)),
            If(load,
                timed.eq(stamp.timed),
                head.eq(stamp.tick[:32]),
                loaded.eq(1),
            ),
            If(pop,
                If(sink.first, loaded.eq(0)),
                If(timed, head.eq(head + self.inc)),
            ),
            If(drop & sink.valid,
                self.trimmed.eq(self.trimmed + 1)
            ),
            If(~self.enable,
                timed.eq(0),
                loaded.eq(0),
            ),
        ]
