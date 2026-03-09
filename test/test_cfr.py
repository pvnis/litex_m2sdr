#!/usr/bin/env python3
#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from litex.gen.sim import run_simulation
from litex_m2sdr.gateware.cfr import CrestFactorReduction


# CFR Tests ----------------------------------------------------------------------------------------

def _pack(ia, qa, ib, qb):
    def to16(v):
        return v & 0xFFFF
    return to16(ia) | (to16(qa) << 16) | (to16(ib) << 32) | (to16(qb) << 48)


def _unpack(word):
    def s16(v):
        v &= 0xFFFF
        return v - 65536 if v >= 32768 else v
    return s16(word), s16(word >> 16), s16(word >> 32), s16(word >> 48)


def test_cfr_passthrough_when_disabled():
    """When enable=0, output must equal input exactly."""
    dut = CrestFactorReduction()
    results = []
    word = _pack(1000, -500, 200, -300)

    def gen():
        yield dut._enable.storage.eq(0)
        yield dut.sink.valid.eq(1)
        yield dut.source.ready.eq(1)
        yield dut.sink.data.eq(word)
        for _ in range(5):
            yield
            results.append((yield dut.source.data))

    run_simulation(dut, [gen()])
    assert all(r == word for r in results), f"passthrough failed: {[hex(r) for r in results]}"


def test_cfr_no_clip_below_threshold():
    """Samples within threshold must pass through (after pipeline fill)."""
    THRESHOLD = 20000
    IA_IN     = 5000
    dut = CrestFactorReduction()
    results = []

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut._threshold.storage.eq(THRESHOLD)
        yield dut.sink.valid.eq(1)
        yield dut.source.ready.eq(1)
        yield dut.sink.data.eq(_pack(IA_IN, 0, 0, 0))
        # Fill the pipeline (delay = n_taps//2 = 16 cycles).
        for _ in range(40):
            yield
            results.append((yield dut.source.data))

    run_simulation(dut, [gen()])
    # After the pipeline fills (≥16 cycles), steady-state output should equal input
    # (no clipping → FIR error is zero → output = delayed input).
    steady = results[20:]
    for r in steady:
        ia, qa, ib, qb = _unpack(r)
        assert ia == IA_IN, f"I_A should be {IA_IN}, got {ia}"
        assert qa == 0,     f"Q_A should be 0, got {qa}"


def test_cfr_clips_large_sample():
    """A sample well above threshold must be reduced in amplitude."""
    THRESHOLD = 10000
    IA_IN     = 30000   # 3× threshold
    dut = CrestFactorReduction()
    results = []

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut._threshold.storage.eq(THRESHOLD)
        yield dut.sink.valid.eq(1)
        yield dut.source.ready.eq(1)
        yield dut.sink.data.eq(_pack(IA_IN, 0, 0, 0))
        for _ in range(60):
            yield
            results.append((yield dut.source.data))

    run_simulation(dut, [gen()])
    # After pipeline fills, the output amplitude should be reduced relative to IA_IN.
    ia, _, _, _ = _unpack(results[-1])
    assert abs(ia) < IA_IN, f"CFR should reduce amplitude: got {ia}, input was {IA_IN}"
    assert abs(ia) <= 32767, f"output must not exceed 16-bit range"


def test_cfr_clip_count_increments():
    """_clip_count must increment when samples are clipped."""
    THRESHOLD = 5000
    dut = CrestFactorReduction()
    count_out = []

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut._threshold.storage.eq(THRESHOLD)
        yield dut.sink.valid.eq(1)
        yield dut.source.ready.eq(1)
        # Send samples above threshold.
        yield dut.sink.data.eq(_pack(20000, 0, 0, 0))
        for _ in range(10):
            yield
        count_out.append((yield dut._clip_count.status))

    run_simulation(dut, [gen()])
    assert count_out[0] > 0, f"clip_count should be > 0 after clipped samples, got {count_out[0]}"


def test_cfr_clip_count_zero_when_no_clip():
    """_clip_count must remain 0 when all samples are within threshold."""
    THRESHOLD = 32767
    dut = CrestFactorReduction()
    count_out = []

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut._threshold.storage.eq(THRESHOLD)
        yield dut.sink.valid.eq(1)
        yield dut.source.ready.eq(1)
        yield dut.sink.data.eq(_pack(1000, -500, 200, -300))
        for _ in range(10):
            yield
        count_out.append((yield dut._clip_count.status))

    run_simulation(dut, [gen()])
    assert count_out[0] == 0, f"clip_count should be 0 when no clipping occurs, got {count_out[0]}"


def test_cfr_output_clamped():
    """Output must not exceed ±32767 even with pathological inputs."""
    dut = CrestFactorReduction()
    results = []

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut._threshold.storage.eq(100)
        yield dut.sink.valid.eq(1)
        yield dut.source.ready.eq(1)
        yield dut.sink.data.eq(_pack(32767, 32767, -32768, -32768))
        for _ in range(60):
            yield
            results.append((yield dut.source.data))

    run_simulation(dut, [gen()])
    for r in results:
        ia, qa, ib, qb = _unpack(r)
        assert -32768 <= ia <= 32767, f"I_A out of range: {ia}"
        assert -32768 <= qa <= 32767, f"Q_A out of range: {qa}"
        assert -32768 <= ib <= 32767, f"I_B out of range: {ib}"
        assert -32768 <= qb <= 32767, f"Q_B out of range: {qb}"
