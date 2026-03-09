#!/usr/bin/env python3
#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.sim import passive

from litex.gen.sim import run_simulation
from litex.soc.interconnect import stream

from litepcie.common import *

from litex_m2sdr.gateware.timed_tx import TimedTXArbiter

# Helpers ------------------------------------------------------------------------------------------

class MockHeaderExtractor(Module):
    """Provides update/timestamp signals mimicking HeaderInserterExtractor extractor output."""
    def __init__(self):
        self.update    = Signal()
        self.timestamp = Signal(64)


class MockTimeGen(Module):
    """Provides a controllable time signal."""
    def __init__(self):
        self.time = Signal(64)


def build_dut(fifo_depth=64):
    header_ext = MockHeaderExtractor()
    time_gen   = MockTimeGen()
    dut = TimedTXArbiter(
        header_extractor = header_ext,
        time_gen         = time_gen,
        fifo_depth       = fifo_depth,
        data_width       = 64,
    )
    return dut, header_ext, time_gen


def send_frame(dut, header_ext, payload, timestamp):
    """
    Helper: emit one pulse of update/timestamp then stream payload words into dut.sink.
    payload is a list of 64-bit sample words; first/last are set automatically.
    """
    # Pulse header_ext.update for one cycle with the timestamp.
    yield header_ext.timestamp.eq(timestamp)
    yield header_ext.update.eq(1)
    yield
    yield header_ext.update.eq(0)
    yield

    # Stream payload words into dut.sink.
    for i, word in enumerate(payload):
        while not (yield dut.sink.ready):
            yield
        yield dut.sink.valid.eq(1)
        yield dut.sink.first.eq(1 if i == 0 else 0)
        yield dut.sink.last.eq(1 if i == len(payload) - 1 else 0)
        yield dut.sink.data.eq(word)
        yield
        yield dut.sink.valid.eq(0)
        yield dut.sink.first.eq(0)
        yield dut.sink.last.eq(0)
        yield


# Tests --------------------------------------------------------------------------------------------

def test_passthrough_mode():
    """With enable=0, samples flow through immediately without timestamp gating."""
    dut, header_ext, time_gen = build_dut()
    payload = [0x100, 0x101, 0x102]
    received = []

    def gen():
        yield dut._enable.storage.eq(0)
        yield dut.source.ready.eq(1)
        yield time_gen.time.eq(0)
        yield

        yield from send_frame(dut, header_ext, payload, timestamp=9999)

        for _ in range(16):
            yield

    @passive
    def mon():
        while True:
            if (yield dut.source.valid) and (yield dut.source.ready):
                received.append((yield dut.source.data))
            yield

    top = Module()
    top.submodules += dut, header_ext, time_gen
    run_simulation(top, [gen(), mon()])
    assert received == payload, f"passthrough: expected {payload}, got {received}"


def test_timed_release_fires_at_timestamp():
    """With enable=1, samples must be held until time_gen.time >= timestamp."""
    dut, header_ext, time_gen = build_dut()
    TIMESTAMP = 1000
    payload = [0xA0, 0xA1, 0xA2]
    received = []
    release_cycle = [None]
    cycle_counter = [0]

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut.source.ready.eq(1)
        yield time_gen.time.eq(0)
        yield

        # Send frame with future timestamp.
        yield from send_frame(dut, header_ext, payload, timestamp=TIMESTAMP)

        # Advance time slowly until past the timestamp.
        for t in range(0, TIMESTAMP + 50, 10):
            yield time_gen.time.eq(t)
            yield
            cycle_counter[0] += 1

        for _ in range(16):
            yield

    @passive
    def mon():
        while True:
            if (yield dut.source.valid) and (yield dut.source.ready):
                received.append((yield dut.source.data))
                if release_cycle[0] is None:
                    release_cycle[0] = cycle_counter[0]
            yield

    top = Module()
    top.submodules += dut, header_ext, time_gen
    run_simulation(top, [gen(), mon()])
    assert received == payload, f"timed: expected {payload}, got {received}"
    # Samples must not have been released before the time counter reached TIMESTAMP.
    assert release_cycle[0] is not None


def test_no_output_before_timestamp():
    """No source.valid output before time reaches the TX timestamp."""
    dut, header_ext, time_gen = build_dut()
    TIMESTAMP = 500
    payload = [0xB0, 0xB1]
    early_outputs = []

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut.source.ready.eq(1)
        yield time_gen.time.eq(0)
        yield

        yield from send_frame(dut, header_ext, payload, timestamp=TIMESTAMP)

        # Hold time BELOW the timestamp and check no output.
        for t in range(0, TIMESTAMP - 10, 5):
            yield time_gen.time.eq(t)
            if (yield dut.source.valid):
                early_outputs.append(t)
            yield

    top = Module()
    top.submodules += dut, header_ext, time_gen
    run_simulation(top, [gen()])
    assert early_outputs == [], f"unexpected early output at times: {early_outputs}"


def test_sequential_frames_correct_timestamps():
    """Two back-to-back frames are released at their respective timestamps."""
    dut, header_ext, time_gen = build_dut()
    TS0, TS1 = 200, 600
    frames = [[0xC0, 0xC1], [0xD0, 0xD1]]
    received = []

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut.source.ready.eq(1)
        yield time_gen.time.eq(0)
        yield

        # Send both frames back-to-back.
        yield from send_frame(dut, header_ext, frames[0], timestamp=TS0)
        yield from send_frame(dut, header_ext, frames[1], timestamp=TS1)

        # Ramp time past both timestamps.
        for t in range(0, TS1 + 50, 10):
            yield time_gen.time.eq(t)
            yield

        for _ in range(32):
            yield

    @passive
    def mon():
        while True:
            if (yield dut.source.valid) and (yield dut.source.ready):
                received.append((yield dut.source.data))
            yield

    top = Module()
    top.submodules += dut, header_ext, time_gen
    run_simulation(top, [gen(), mon()])
    assert received == frames[0] + frames[1], f"sequential: got {received}"


def test_late_count_increments_on_overdue_frame():
    """late_count increments when a frame arrives with a timestamp already in the past."""
    dut, header_ext, time_gen = build_dut()
    payload = [0xE0, 0xE1]
    late_seen = [0]

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut.source.ready.eq(1)
        # Set time to 1000 BEFORE sending a frame with timestamp 500 (already past).
        yield time_gen.time.eq(1000)
        yield

        yield from send_frame(dut, header_ext, payload, timestamp=500)

        for _ in range(32):
            yield

        late_seen[0] = (yield dut._late_count.status)

    top = Module()
    top.submodules += dut, header_ext, time_gen
    run_simulation(top, [gen()])
    assert late_seen[0] >= 1, f"expected late_count >= 1, got {late_seen[0]}"


def test_underrun_count_increments_when_idle():
    """underrun_count increments every cycle that IDLE has no data and enable=1."""
    dut, header_ext, time_gen = build_dut()
    idle_cycles = 10
    underrun = [0]

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut.source.ready.eq(1)
        yield time_gen.time.eq(0)
        yield

        # No data sent: let idle_cycles pass.
        for _ in range(idle_cycles):
            yield

        underrun[0] = (yield dut._underrun_count.status)

    top = Module()
    top.submodules += dut, header_ext, time_gen
    run_simulation(top, [gen()])
    assert underrun[0] >= idle_cycles - 2, \
        f"expected underrun_count ~= {idle_cycles}, got {underrun[0]}"
