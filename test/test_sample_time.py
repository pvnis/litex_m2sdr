#!/usr/bin/env python3
#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2026 Pavonis / Enjoy-Digital contributors.
# SPDX-License-Identifier: BSD-2-Clause

"""Sample-count time base: TX fine gate (exact per-word release) and RX tick tracker."""

from migen import *
from migen.sim import passive

from litex.gen.sim import run_simulation

from litex_m2sdr.gateware.sample_time import RXTickTracker, TXFineGate

INC = 2          # 1R1T: two sample periods per PHY word
SLOT_PERIOD = 4  # one PHY slot every 4 clock cycles


# TX fine gate ------------------------------------------------------------------------------------

class FineHarness:
    """Models the RFIC side: a tick counter advancing every 4 cycles and a PHY slot strobe."""
    def __init__(self, dut, rx_phase=1, tx_phase=3):
        self.dut = dut
        self.cycle = 0
        self.tick = 1000
        self.rx_phase = rx_phase
        self.tx_phase = tx_phase
        self.emitted = []   # (data, tick shown by the counter in the slot it was emitted)

    @passive
    def clock(self):
        dut = self.dut
        yield dut.inc.eq(INC)
        while True:
            phase = self.cycle % SLOT_PERIOD
            yield dut.source.ready.eq(1 if phase == self.tx_phase else 0)
            yield dut.tick.eq(self.tick)
            yield
            if phase == self.rx_phase:
                self.tick += INC
            self.cycle += 1

    @passive
    def monitor(self):
        dut = self.dut
        while True:
            if (yield dut.source.valid) and (yield dut.source.ready):
                self.emitted.append(((yield dut.source.data), (yield dut.tick)))
            yield

    def push_stamp(self, timed, tick):
        dut = self.dut
        yield dut.stamp.timed.eq(timed)
        yield dut.stamp.tick.eq(tick)
        yield dut.stamp.valid.eq(1)
        yield
        while not (yield dut.stamp.ready):
            yield
        yield dut.stamp.valid.eq(0)

    def send_words(self, base, n, first=True, gap_after=None, gap=0):
        dut = self.dut
        for i in range(n):
            yield dut.sink.data.eq(base + i)
            yield dut.sink.first.eq(1 if (first and i == 0) else 0)
            yield dut.sink.valid.eq(1)
            yield
            while not (yield dut.sink.ready):
                yield
            if gap_after is not None and i == gap_after:
                yield dut.sink.valid.eq(0)
                for _ in range(gap):
                    yield
        yield dut.sink.valid.eq(0)
        yield dut.sink.first.eq(0)


def _fine(scenario, enable=1, **kw):
    dut = TXFineGate()
    h = FineHarness(dut, **kw)

    def main():
        yield dut.enable.eq(enable)
        for _ in range(8):
            yield
        yield from scenario(h)
        for _ in range(400):
            yield
        h.trimmed = yield dut.trimmed

    def stamps():
        # Stamps are queued by the scenario through h.stamp_queue, in frame order.
        for timed, tick in h.stamp_queue:
            yield from h.push_stamp(timed, tick)

    h.stamp_queue = []
    scenario_gen = main()
    run_simulation(dut, [scenario_gen, h.clock(), h.monitor(), stamps_runner(h)])
    return h


def stamps_runner(h):
    # Waits for the scenario to fill h.stamp_queue, then feeds each stamp once.
    done = 0
    for _ in range(4000):
        if done < len(h.stamp_queue):
            timed, tick = h.stamp_queue[done]
            yield from h.push_stamp(timed, tick)
            done += 1
        else:
            yield


def test_fine_words_leave_in_the_slot_of_their_tick():
    N = 12
    target = {}

    def scenario(h):
        target["t"] = h.tick + 60                      # 30 words ahead, even
        h.stamp_queue.append((1, target["t"]))
        yield from h.send_words(0x100, N)
    h = _fine(scenario)
    assert [d for d, _ in h.emitted] == list(range(0x100, 0x100 + N))
    assert [t for _, t in h.emitted] == [target["t"] + INC * i for i in range(N)], h.emitted[:4]
    assert h.trimmed == 0


def test_fine_is_exact_for_every_strobe_phase():
    for rx_phase in range(SLOT_PERIOD):
        for tx_phase in range(SLOT_PERIOD):
            target = {}

            def scenario(h):
                target["t"] = h.tick + 40
                h.stamp_queue.append((1, target["t"]))
                yield from h.send_words(0x200, 6)
            h = _fine(scenario, rx_phase=rx_phase, tx_phase=tx_phase)
            got = [t for _, t in h.emitted]
            assert len(got) == 6, (rx_phase, tx_phase, h.emitted)
            # Same constant offset for every word, and the same one for every phase pair.
            assert [g - got[0] for g in got] == [INC * i for i in range(6)], (rx_phase, tx_phase, got)
            assert got[0] - target["t"] in (0, -INC), (rx_phase, tx_phase, got[0], target["t"])


def test_fine_late_frame_is_trimmed_not_shifted():
    N = 40
    target = {}

    def scenario(h):
        target["t"] = h.tick - 20                      # 10 words in the past when it arrives
        h.stamp_queue.append((1, target["t"]))
        yield from h.send_words(0x300, N)
    h = _fine(scenario)
    assert h.trimmed > 0
    # Every word that does go out is in its own slot: tick = stamp + 2 * index.
    for d, t in h.emitted:
        assert t == target["t"] + INC * (d - 0x300), (hex(d), t)
    # and the tail of the frame made it.
    assert h.emitted and h.emitted[-1][0] == 0x300 + N - 1


def test_fine_supply_gap_loses_words_but_keeps_alignment():
    N = 30
    target = {}

    def scenario(h):
        target["t"] = h.tick + 40
        h.stamp_queue.append((1, target["t"]))
        yield from h.send_words(0x400, N, gap_after=9, gap=26)   # the source stalls mid-frame
    h = _fine(scenario)
    for d, t in h.emitted:
        assert t == target["t"] + INC * (d - 0x400), (hex(d), t)
    assert h.emitted[-1][0] == 0x400 + N - 1
    assert h.trimmed > 0


def test_fine_contiguous_frames_and_untimed_frame():
    target = {}

    def scenario(h):
        target["t"] = h.tick + 40
        h.stamp_queue.append((1, target["t"]))
        h.stamp_queue.append((1, target["t"] + INC * 8))
        h.stamp_queue.append((0, 0))
        yield from h.send_words(0x500, 8)
        yield from h.send_words(0x600, 8)
        yield from h.send_words(0x700, 5)               # untimed: straight through
    h = _fine(scenario)
    timed = [(d, t) for d, t in h.emitted if d < 0x700]
    assert [d for d, _ in timed] == list(range(0x500, 0x508)) + list(range(0x600, 0x608))
    assert [t for _, t in timed] == [target["t"] + INC * i for i in range(16)]
    assert [d for d, _ in h.emitted if d >= 0x700] == list(range(0x700, 0x705))


def test_fine_odd_stamp_is_one_sample_late_not_dropped():
    target = {}

    def scenario(h):
        target["t"] = h.tick + 41                      # odd
        h.stamp_queue.append((1, target["t"]))
        yield from h.send_words(0x800, 10)
    h = _fine(scenario)
    assert [d for d, _ in h.emitted] == list(range(0x800, 0x80a))
    assert h.emitted[0][1] == target["t"] + 1
    assert h.trimmed == 0


def test_fine_transparent_when_disabled():
    def scenario(h):
        yield from h.send_words(0x900, 6)
    h = _fine(scenario, enable=0)
    assert [d for d, _ in h.emitted] == list(range(0x900, 0x906))


# RX tick tracker -----------------------------------------------------------------------------------

def test_tracker_reports_the_tick_of_the_word_at_the_consumer():
    dut = RXTickTracker()
    seen = []   # (data, tick offered with it)

    def producer():
        yield dut.inc.eq(INC)
        yield dut.exact.eq(1)
        tick = 5000
        for i in range(40):
            if i == 17:
                tick += 3 * INC                         # three words dropped upstream
            yield dut.sink.data.eq(0x1000 + i)
            yield dut.sink.tick.eq(tick)
            yield dut.sink.valid.eq(1)
            yield
            while not (yield dut.sink.ready):
                yield
            yield dut.sink.valid.eq(0)
            yield
            tick += INC
        for _ in range(60):
            yield

    # A one-word pipeline register between the tracker and the consumer, and a slow consumer.
    class Wrap(Module):
        def __init__(self):
            self.submodules.dut = dut
            self.valid = Signal(); self.data = Signal(64); self.ready = Signal()
            self.comb += dut.source.ready.eq(~self.valid | self.ready)
            self.sync += [
                If(dut.source.valid & dut.source.ready, self.valid.eq(1), self.data.eq(dut.source.data)
                ).Elif(self.ready, self.valid.eq(0))
            ]
            self.comb += dut.pop.eq(self.valid & self.ready)
    w = Wrap()

    @passive
    def consumer():
        n = 0
        while True:
            yield w.ready.eq(1 if (n % 3) == 0 else 0)
            yield
            if (yield w.valid) and (yield w.ready):
                seen.append(((yield w.data), (yield dut.tick)))
            n += 1

    run_simulation(w, [producer(), consumer()])
    assert [d for d, _ in seen] == [0x1000 + i for i in range(40)]
    expect = [5000 + INC * i + (3 * INC if i >= 17 else 0) for i in range(40)]
    assert [t for _, t in seen] == expect, list(zip(seen, expect))[:20]


# RX chain in tick mode: tracker -> pipeline register -> header inserter (+ timed start) -------------

from litex.soc.interconnect import stream
from litepcie.common import dma_layout

from litex_m2sdr.gateware.header         import RXHeaderInserter
from litex_m2sdr.gateware.timed_rx_start import TimedRXStart
from litex_m2sdr.gateware.timed_tx_gate  import TimedTXGate

RX_FRAME_WORDS = 8
RX_SYNC        = 0x5aa5_5aa5_5aa5_5aa5


class RXChain(Module):
    def __init__(self):
        self.submodules.tracker  = tracker  = RXTickTracker()
        self.submodules.buffer   = buffer   = stream.Buffer(dma_layout(64))
        self.submodules.inserter = inserter = RXHeaderInserter(data_width=64, with_csr=False)
        self.submodules.gate     = gate     = TimedRXStart(with_csr=False)
        self.comb += [
            tracker.source.connect(buffer.sink),
            buffer.source.connect(inserter.sink),
            tracker.inc.eq(INC),
            tracker.exact.eq(1),
            tracker.pop.eq(inserter.sink.valid & inserter.sink.ready),
            inserter.timestamp.eq(tracker.tick),
            inserter.stamp_on_payload.eq(1),
            inserter.header.eq(RX_SYNC),
            inserter.reset.eq(gate.hold),
            gate.tick_mode.eq(1),
            gate.sink_tick.eq(tracker.tick),
            gate.sink_fire.eq(inserter.sink.valid & inserter.sink.ready),
            gate.inc.eq(INC),
        ]


def _rx_chain(scenario, drops=(), words=120, period=5):
    dut = RXChain()
    out = []   # (data, first)
    state = {"tick": 7000}

    def adc():
        # One word every `period` cycles; word i carries data = its own tick so frames are checkable.
        yield dut.inserter.header_enable.eq(1)
        yield dut.inserter.frame_cycles.eq(RX_FRAME_WORDS)
        yield dut.inserter.source.ready.eq(1)
        yield from scenario(dut, state)
        for i in range(words):
            if i in drops:
                state["tick"] += INC * drops[i]           # words lost before the clock-domain FIFO
            yield dut.tracker.sink.data.eq(state["tick"])
            yield dut.tracker.sink.tick.eq(state["tick"])
            yield dut.tracker.sink.valid.eq(1)
            yield
            while not (yield dut.tracker.sink.ready):
                yield
            yield dut.tracker.sink.valid.eq(0)
            state["tick"] += INC
            for _ in range(period - 1):
                yield
        for _ in range(40):
            yield
        state["late"] = yield dut.gate.late

    @passive
    def monitor():
        while True:
            if (yield dut.inserter.source.valid) and (yield dut.inserter.source.ready):
                out.append(((yield dut.inserter.source.data), (yield dut.inserter.source.first)))
            yield

    run_simulation(dut, [adc(), monitor()])
    # Split into frames: sync word, stamp, payload.
    frames = []
    i = 0
    while i + 2 + RX_FRAME_WORDS <= len(out):
        assert out[i][0] == RX_SYNC and out[i][1] == 1, (i, out[i])
        frames.append((out[i + 1][0], [d for d, _ in out[i + 2:i + 2 + RX_FRAME_WORDS]]))
        i += 2 + RX_FRAME_WORDS
    return frames, state


def test_rx_frame_stamp_is_the_tick_of_its_first_payload_word():
    def scenario(dut, state):
        yield dut.inserter.enable.eq(1)
        yield
    frames, _ = _rx_chain(scenario)
    assert len(frames) >= 10
    for stamp, payload in frames:
        assert stamp == payload[0], (stamp, payload[0])
        assert payload == [payload[0] + INC * k for k in range(RX_FRAME_WORDS)]


def test_rx_stamps_stay_exact_across_dropped_samples():
    def scenario(dut, state):
        yield dut.inserter.enable.eq(1)
        yield
    frames, _ = _rx_chain(scenario, drops={13: 5, 14: 1, 50: 200, 77: 3})
    assert len(frames) >= 10
    gaps = 0
    for stamp, payload in frames:
        assert stamp == payload[0], (stamp, payload)        # still the first payload word's tick
        gaps += sum(1 for a, b in zip(payload, payload[1:]) if b - a != INC)
    assert gaps >= 2                                          # the drops are visible inside frames


def test_rx_timed_start_first_frame_is_the_requested_tick():
    want = {}

    def scenario(dut, state):
        want["t"] = state["tick"] + INC * 37
        yield dut.gate.start_time.eq(want["t"])
        yield
        yield dut.gate.enable.eq(1)
        yield
        yield dut.inserter.enable.eq(1)
        yield
    frames, state = _rx_chain(scenario)
    assert not state["late"]
    assert frames[0][0] == want["t"] and frames[0][1][0] == want["t"], frames[0]
    for (s0, _), (s1, _) in zip(frames, frames[1:]):
        assert s1 - s0 == INC * RX_FRAME_WORDS


def test_rx_timed_start_in_the_past_starts_at_once_and_flags_late():
    def scenario(dut, state):
        yield dut.gate.start_time.eq(state["tick"] - 1000)
        yield
        yield dut.gate.enable.eq(1)
        yield
        yield dut.inserter.enable.eq(1)
        yield
    frames, state = _rx_chain(scenario)
    assert state["late"] and frames and frames[0][0] == frames[0][1][0]


# TX chain in tick mode: coarse gate (sys) -> small FIFOs -> fine gate ---------------------------------

TX_FRAME_WORDS = 8


class TXChain(Module):
    def __init__(self):
        self.submodules.coarse = coarse = TimedTXGate(with_csr=False)
        self.submodules.dfifo  = dfifo  = stream.SyncFIFO(dma_layout(64), 8, buffered=True)   # stands for tx_cdc
        self.submodules.sfifo  = sfifo  = stream.SyncFIFO([("timed", 1), ("tick", 64)], 8, buffered=True)
        self.submodules.fine   = fine   = TXFineGate()
        self.tick = Signal(64)
        self.comb += [
            coarse.source.connect(dfifo.sink),
            dfifo.source.connect(fine.sink),
            coarse.stamp.connect(sfifo.sink),
            sfifo.source.connect(fine.stamp),
            coarse.time.eq(self.tick),
            coarse.frames_active.eq(1),
            coarse.enable.eq(1),
            coarse.stamp_enable.eq(1),
            fine.enable.eq(1),
            fine.tick.eq(self.tick),
            fine.inc.eq(INC),
        ]


def _tx_chain(frames, advance=24, late_margin=16, stale_margin=4000):
    """frames: list of (offset_from_start_tick, base) ; returns (emitted[(data, tick)], stats)."""
    dut = TXChain()
    emitted = []
    state = {"cycle": 0, "tick": 20000}
    stats = {}

    @passive
    def clock():
        while True:
            phase = state["cycle"] % SLOT_PERIOD
            yield dut.fine.source.ready.eq(1 if phase == 3 else 0)
            yield dut.tick.eq(state["tick"])
            yield
            if phase == 1:
                state["tick"] += INC
            state["cycle"] += 1

    @passive
    def monitor():
        while True:
            if (yield dut.fine.source.valid) and (yield dut.fine.source.ready):
                emitted.append(((yield dut.fine.source.data), (yield dut.tick)))
            yield

    def host():
        yield dut.coarse.advance.eq(advance)
        yield dut.coarse.late_margin.eq(late_margin)
        yield dut.coarse.stale_margin.eq(stale_margin)
        for _ in range(8):
            yield
        t0 = state["tick"]
        stats["t0"] = t0
        for (off, base) in frames:
            yield dut.coarse.timestamp.eq(t0 + off if off is not None else 0)
            for i in range(TX_FRAME_WORDS):
                yield dut.coarse.sink.data.eq(base + i)
                yield dut.coarse.sink.first.eq(1 if i == 0 else 0)
                yield dut.coarse.sink.last.eq(1 if i == TX_FRAME_WORDS - 1 else 0)
                yield dut.coarse.sink.valid.eq(1)
                yield
                while not (yield dut.coarse.sink.ready):
                    yield
            yield dut.coarse.sink.valid.eq(0)
        for _ in range(600):
            yield
        stats["late"]    = yield dut.coarse.late_count
        stats["stale"]   = yield dut.coarse.stale_count
        stats["passed"]  = yield dut.coarse.passed_count
        stats["trimmed"] = yield dut.fine.trimmed

    run_simulation(dut, [host(), clock(), monitor()])
    return emitted, stats


def test_tx_chain_every_word_of_contiguous_frames_is_on_its_tick():
    W = INC * TX_FRAME_WORDS
    frames = [(400 + k * W, 0x1000 + 0x100 * k) for k in range(6)]
    emitted, stats = _tx_chain(frames)
    assert stats["passed"] == 6 and stats["late"] == 0 and stats["trimmed"] == 0, stats
    assert len(emitted) == 6 * TX_FRAME_WORDS
    for d, t in emitted:
        k, i = (d - 0x1000) // 0x100, (d - 0x1000) % 0x100
        assert t == stats["t0"] + 400 + k * W + INC * i, (hex(d), t)


def test_tx_chain_gap_between_bursts_and_far_future_frame():
    W = INC * TX_FRAME_WORDS
    frames = [(300, 0x2000), (300 + W, 0x2100), (2000, 0x2200), (2000 + W, 0x2300)]
    emitted, stats = _tx_chain(frames)
    assert len(emitted) == 4 * TX_FRAME_WORDS and stats["trimmed"] == 0
    for d, t in emitted:
        k, i = (d - 0x2000) // 0x100, (d - 0x2000) % 0x100
        assert t == stats["t0"] + frames[k][0] + INC * i, (hex(d), t)


def test_tx_chain_slightly_late_frame_is_trimmed_and_the_rest_is_exact():
    W = INC * TX_FRAME_WORDS
    # The first frame's tick is already 6 samples old when it reaches the coarse gate: within the
    # late margin, so it is forwarded; the fine gate discards its head and emits the rest on time.
    frames = [(-6, 0x3000), (-6 + W, 0x3100), (-6 + 2 * W, 0x3200)]
    emitted, stats = _tx_chain(frames)
    assert stats["late"] == 0 and stats["trimmed"] > 0, stats
    for d, t in emitted:
        k, i = (d - 0x3000) // 0x100, (d - 0x3000) % 0x100
        assert t == stats["t0"] - 6 + k * W + INC * i, (hex(d), t)
    assert [d for d, _ in emitted][-TX_FRAME_WORDS:] == [0x3200 + i for i in range(TX_FRAME_WORDS)]


def test_tx_chain_stale_and_late_frames_are_dropped_whole_and_stamps_stay_in_step():
    W = INC * TX_FRAME_WORDS
    frames = [(-6000, 0x4000), (-300, 0x4100), (500, 0x4200), (500 + W, 0x4300)]
    emitted, stats = _tx_chain(frames)
    assert stats["stale"] == 1 and stats["late"] == 1 and stats["passed"] == 2, stats
    want = [0x4200 + i for i in range(TX_FRAME_WORDS)] + [0x4300 + i for i in range(TX_FRAME_WORDS)]
    assert [d for d, _ in emitted] == want
    for d, t in emitted:
        k, i = (d - 0x4000) // 0x100, (d - 0x4000) % 0x100
        assert t == stats["t0"] + frames[k][0] + INC * i, (hex(d), t)


def test_tx_chain_untimed_frame_between_timed_ones():
    W = INC * TX_FRAME_WORDS
    frames = [(300, 0x5000), (None, 0x5100), (900, 0x5200)]
    emitted, stats = _tx_chain(frames)
    assert [d for d, _ in emitted if 0x5100 <= d < 0x5200] == [0x5100 + i for i in range(TX_FRAME_WORDS)]
    for d, t in emitted:
        if d < 0x5100:
            assert t == stats["t0"] + 300 + INC * (d - 0x5000)
        elif d >= 0x5200:
            assert t == stats["t0"] + 900 + INC * (d - 0x5200)
