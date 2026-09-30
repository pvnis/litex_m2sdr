#!/usr/bin/env python3
#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2026 Pavonis / Enjoy-Digital contributors.
# SPDX-License-Identifier: BSD-2-Clause

"""TimedRXStart simulation with the real RX header inserter: the first frame's stamp is start_time."""

from migen import *
from migen.sim import passive

from litex.gen.sim import run_simulation

from litex_m2sdr.gateware.header         import RXHeaderInserter
from litex_m2sdr.gateware.timed_rx_start import TimedRXStart

NS_PER_CYCLE = 8        # 125 MHz sys clock
FRAME_WORDS  = 8
SYNC_WORD    = 0x5aa5_5aa5_5aa5_5aa5


class DUT(Module):
    def __init__(self):
        self.time = Signal(64)
        self.submodules.gate     = TimedRXStart(with_csr=False)
        self.submodules.inserter = RXHeaderInserter(data_width=64, with_csr=False)
        self.comb += [
            self.gate.time.eq(self.time),
            self.inserter.timestamp.eq(self.time),
            self.inserter.header.eq(SYNC_WORD),
            self.inserter.reset.eq(self.gate.hold),
        ]


def _run(scenario, cycles=1500):
    dut = DUT()
    out = []     # (time_ns, data, first, last)
    state = {"now": 0, "sample": 0}

    @passive
    def clock():
        while True:
            state["now"] += NS_PER_CYCLE
            yield dut.time.eq(state["now"])
            yield

    @passive
    def adc():
        # Free-running sample source: a new word every cycle, consumed or not.
        yield dut.inserter.sink.valid.eq(1)
        while True:
            yield dut.inserter.sink.data.eq(0x1000 + state["sample"])
            yield
            state["sample"] += 1

    @passive
    def monitor():
        while True:
            if (yield dut.inserter.source.valid) and (yield dut.inserter.source.ready):
                out.append((state["now"], (yield dut.inserter.source.data),
                            (yield dut.inserter.source.first), (yield dut.inserter.source.last)))
            yield

    def main():
        # inserter.enable models the DMA writer enable: the driver arms the gate before starting it.
        yield dut.inserter.header_enable.eq(1)
        yield dut.inserter.frame_cycles.eq(FRAME_WORDS)
        yield dut.inserter.source.ready.eq(1)
        yield
        yield from scenario(dut, state)
        for _ in range(cycles):
            yield
        state["late"]      = yield dut.gate.late
        state["opened"]    = yield dut.gate.opened
        state["open_time"] = yield dut.gate.open_time

    run_simulation(dut, [main(), clock(), adc(), monitor()])
    return out, state


def test_transparent_when_disabled():
    def scenario(dut, state):
        yield dut.gate.start_time.eq(10**12)     # far future: would hold if armed
        yield dut.inserter.enable.eq(1)
        yield
    out, state = _run(scenario, cycles=200)
    assert out and out[0][1] == SYNC_WORD        # frames flow straight away
    assert out[0][0] < 100


def test_first_frame_is_stamped_with_start_time():
    start = {}

    def scenario(dut, state):
        start["t"] = state["now"] + 4000         # 500 cycles ahead
        yield dut.gate.start_time.eq(start["t"])
        yield
        yield dut.gate.enable.eq(1)
        yield
        yield dut.inserter.enable.eq(1)
        yield
    out, state = _run(scenario)
    assert state["opened"] and not state["late"]
    # Nothing is emitted before the start time.
    assert all(t >= start["t"] for (t, _, _, _) in out), out[:3]
    # First word is the sync header, second is the timestamp == start_time (within one time step).
    assert out[0][1] == SYNC_WORD and out[0][2] == 1
    stamp = out[1][1]
    assert start["t"] <= stamp < start["t"] + NS_PER_CYCLE, (start["t"], stamp)
    # Then a full payload frame of consecutive samples, closed by 'last'.
    payload = out[2:2 + FRAME_WORDS]
    assert [d for (_, d, _, _) in payload] == list(range(payload[0][1], payload[0][1] + FRAME_WORDS))
    assert payload[-1][3] == 1
    # The samples produced while closed were dropped, not queued.
    assert payload[0][1] - 0x1000 > 400


def test_start_in_the_past_opens_immediately_and_flags_late():
    def scenario(dut, state):
        for _ in range(50):
            yield
        yield dut.gate.start_time.eq(state["now"] - 100)
        yield
        yield dut.gate.enable.eq(1)
        yield
        yield dut.inserter.enable.eq(1)
        yield
    out, state = _run(scenario, cycles=200)
    assert state["opened"] and state["late"]
    assert out and out[0][1] == SYNC_WORD


def test_rearm_holds_again():
    marks = {}

    def scenario(dut, state):
        yield dut.gate.start_time.eq(state["now"] + 800)
        yield
        yield dut.gate.enable.eq(1)
        yield
        yield dut.inserter.enable.eq(1)
        for _ in range(300):
            yield
        yield dut.inserter.enable.eq(0)           # stream stopped
        yield dut.gate.enable.eq(0)
        yield
        marks["rearm"] = state["now"]
        marks["t2"]    = state["now"] + 2400
        yield dut.gate.start_time.eq(marks["t2"])
        yield
        yield dut.gate.enable.eq(1)
        yield
        yield dut.inserter.enable.eq(1)
        yield
    out, state = _run(scenario)
    # (the word in flight on the cycle the stream is disabled may still be accepted)
    gap = [t for (t, _, _, _) in out if marks["rearm"] + 2 * NS_PER_CYCLE < t < marks["t2"]]
    assert gap == [], gap[:4]
    after = [(t, d, f) for (t, d, f, _) in out if t >= marks["t2"]]
    assert after[0][1] == SYNC_WORD and after[0][2] == 1
    assert marks["t2"] <= after[1][1] < marks["t2"] + NS_PER_CYCLE
