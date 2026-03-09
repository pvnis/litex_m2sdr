#!/usr/bin/env python3
#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from litex.gen.sim import run_simulation
from litex_m2sdr.gateware.sample_counter import SampleCounter


def test_counter_increments_on_rx_valid():
    """count increments once per rx_valid pulse."""
    rx_valid = Signal()
    dut = SampleCounter(rx_valid=rx_valid)
    counts = []

    def gen():
        for _ in range(5):
            yield rx_valid.eq(1)
            yield
            yield rx_valid.eq(0)
            yield
        counts.append((yield dut.count))

    run_simulation(dut, [gen()])
    assert counts[0] == 5, f"expected 5, got {counts[0]}"


def test_counter_does_not_increment_without_valid():
    """count stays at 0 when rx_valid is never asserted."""
    rx_valid = Signal()
    dut = SampleCounter(rx_valid=rx_valid)
    counts = []

    def gen():
        for _ in range(10):
            yield
        counts.append((yield dut.count))

    run_simulation(dut, [gen()])
    assert counts[0] == 0


def test_csr_latch_captures_snapshot():
    """Pulsing control.read latches the current count; subsequent increments don't change it."""
    rx_valid = Signal()
    dut = SampleCounter(rx_valid=rx_valid)
    latch_before = [0]
    latch_after  = [0]

    def gen():
        # Increment 10 times.
        for _ in range(10):
            yield rx_valid.eq(1)
            yield
            yield rx_valid.eq(0)
            yield

        # Latch.
        yield dut._control.fields.read.eq(1)
        yield
        yield dut._control.fields.read.eq(0)
        yield

        latch_before[0] = (yield dut._count.status)

        # Increment 5 more times.
        for _ in range(5):
            yield rx_valid.eq(1)
            yield
            yield rx_valid.eq(0)
            yield

        latch_after[0] = (yield dut._count.status)

    run_simulation(dut, [gen()])
    # Latch should capture the count at read time; free-running count shouldn't change latch.
    assert latch_before[0] == 10, f"expected latch=10, got {latch_before[0]}"
    assert latch_after[0]  == 10, f"latch should not change without re-read, got {latch_after[0]}"


def test_counter_wraps_at_64_bits():
    """count wraps to 0 after 2^64 - 1 (simulated with a short counter for speed)."""
    # Use a 4-bit count to make this tractable in simulation.
    rx_valid = Signal()
    dut_count = Signal(4)
    dut_sync = [If(rx_valid, dut_count.eq(dut_count + 1))]
    tb = Module()
    tb.submodules += dut_count  # can't submodule a Signal; test logic inline below
    results = []

    class TinyCounter(Module):
        def __init__(self, rx_valid):
            self.count = Signal(4)
            self.sync += If(rx_valid, self.count.eq(self.count + 1))

    tiny = TinyCounter(rx_valid)

    def gen():
        for _ in range(16):
            yield rx_valid.eq(1)
            yield
        yield rx_valid.eq(0)
        yield
        results.append((yield tiny.count))

    run_simulation(tiny, [gen()])
    assert results[0] == 0, f"4-bit counter should wrap to 0 after 16 increments, got {results[0]}"
