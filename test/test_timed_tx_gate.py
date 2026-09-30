#!/usr/bin/env python3
#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2026 Pavonis / Enjoy-Digital contributors.
# SPDX-License-Identifier: BSD-2-Clause

"""TimedTXGate simulation: hold / pass / drop / untimed / pass-through semantics."""

from migen import *
from migen.sim import passive

from litex.gen.sim import run_simulation

from litex_m2sdr.gateware.timed_tx_gate import TimedTXGate

NS_PER_CYCLE = 10
FRAME_WORDS  = 8


class Harness:
    """Drives frames into the gate like the header extractor does and records what comes out."""

    def __init__(self, dut):
        self.dut       = dut
        self.out       = []   # (word, time_at_accept)
        self.now       = 0

    @passive
    def clock(self):
        # Board time advances every cycle.
        while True:
            self.now += NS_PER_CYCLE
            yield self.dut.time.eq(self.now)
            yield

    @passive
    def sink_monitor(self):
        while True:
            if (yield self.dut.source.valid) and (yield self.dut.source.ready):
                self.out.append(((yield self.dut.source.data), self.now))
            yield

    def send_frame(self, base, ts, timeout=4000):
        """Present one frame (words base..base+N-1) whose extractor-latched timestamp is ts."""
        dut = self.dut
        yield dut.timestamp.eq(ts)      # latched by the extractor before the payload
        for i in range(FRAME_WORDS):
            yield dut.sink.data.eq(base + i)
            yield dut.sink.first.eq(1 if i == 0 else 0)
            yield dut.sink.last.eq(1 if i == FRAME_WORDS - 1 else 0)
            yield dut.sink.valid.eq(1)
            yield
            waited = 0
            while not (yield dut.sink.ready):
                yield
                waited += 1
                assert waited < timeout, "sink stalled for too long"
        yield dut.sink.valid.eq(0)
        yield dut.sink.first.eq(0)
        yield dut.sink.last.eq(0)


def _run(scenario, **cfg):
    dut = TimedTXGate(with_csr=False)
    h   = Harness(dut)

    def setup():
        yield dut.source.ready.eq(1)
        yield dut.frames_active.eq(cfg.get("frames_active", 1))
        yield dut.enable.eq(cfg.get("enable", 1))
        yield dut.late_margin.eq(cfg.get("late_margin", 100))
        yield
        yield from scenario(h)
        for _ in range(20):
            yield
        h.late   = yield dut.late_count
        h.held   = yield dut.held_count
        h.passed = yield dut.passed_count

    run_simulation(dut, [setup(), h.clock(), h.sink_monitor()])
    return dut, h


def words(h, base):
    return [w for (w, _) in h.out if base <= w < base + FRAME_WORDS]


def test_pass_through_when_disabled():
    def scenario(h):
        yield from h.send_frame(0x100, ts=10_000_000)   # far future: would hold if enabled
    dut, h = _run(scenario, enable=0)
    assert words(h, 0x100) == list(range(0x100, 0x100 + FRAME_WORDS))
    assert (h.held, h.late, h.passed) == (0, 0, 0)      # gate never engaged


def test_untimed_frame_passes_immediately():
    def scenario(h):
        yield from h.send_frame(0x200, ts=0)
    dut, h = _run(scenario)
    assert words(h, 0x200) == list(range(0x200, 0x200 + FRAME_WORDS))


def test_on_time_frame_passes():
    def scenario(h):
        yield from h.send_frame(0x300, ts=h.now + NS_PER_CYCLE)   # "now" at arrival
    dut, h = _run(scenario)
    assert words(h, 0x300) == list(range(0x300, 0x300 + FRAME_WORDS))


def test_future_frame_is_held_until_its_timestamp():
    target = {}

    def scenario(h):
        target["ts"] = h.now + 1_000                    # 100 cycles ahead
        yield from h.send_frame(0x400, ts=target["ts"])
    dut, h = _run(scenario)
    assert (h.held, h.late, h.passed) == (1, 0, 1)
    out = [(w, t) for (w, t) in h.out if 0x400 <= w < 0x400 + FRAME_WORDS]
    assert [w for w, _ in out] == list(range(0x400, 0x400 + FRAME_WORDS))
    first_time = out[0][1]
    # Released on the cycle time reaches the timestamp (allow one cycle of FSM latency).
    assert target["ts"] <= first_time <= target["ts"] + 4 * NS_PER_CYCLE, (target["ts"], first_time)


def test_late_frame_is_dropped_and_counted():
    def scenario(h):
        for _ in range(2000):                                       # let board time reach 20 us
            yield
        yield from h.send_frame(0x500, ts=h.now - 10_000)           # 10 us late, margin 100 ns
        yield from h.send_frame(0x600, ts=0)                        # a following untimed frame
    dut, h = _run(scenario)
    assert words(h, 0x500) == []                                   # dropped
    assert words(h, 0x600) == list(range(0x600, 0x600 + FRAME_WORDS))  # stream continues
    assert (h.late, h.passed) == (1, 1)


def test_slightly_late_frame_within_margin_passes():
    def scenario(h):
        for _ in range(200):
            yield
        yield from h.send_frame(0x700, ts=h.now - 50)               # 50 ns late, margin 100 ns
    dut, h = _run(scenario)
    assert words(h, 0x700) == list(range(0x700, 0x700 + FRAME_WORDS))


def test_back_to_back_frames_after_release_all_pass():
    def scenario(h):
        ts = h.now + 500
        for k in range(4):
            # Consecutive frames stamped contiguously; after the first release they are on time.
            yield from h.send_frame(0x800 + 0x10 * k, ts=ts + k * FRAME_WORDS * NS_PER_CYCLE)
    dut, h = _run(scenario)
    got = [w for (w, _) in h.out if 0x800 <= w < 0x900]
    assert len(got) == 4 * FRAME_WORDS
    assert got == sorted(got)
    assert (h.held, h.late, h.passed) == (1, 0, 4)     # only the first frame waited

