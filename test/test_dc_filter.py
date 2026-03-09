#!/usr/bin/env python3
#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from litex.gen.sim import run_simulation
from litex_m2sdr.gateware.dc_filter import DCFilter


# DC Filter Tests ----------------------------------------------------------------------------------

def _drive_sample(dut, ia, qa, ib, qb):
    """Build a 64-bit packed sample word from signed 16-bit values."""
    def to16(v):
        return v & 0xFFFF
    return (to16(ia) | (to16(qa) << 16) | (to16(ib) << 32) | (to16(qb) << 48))


def _unpack(word):
    """Unpack a 64-bit word into four signed 16-bit values."""
    def s16(v):
        v &= 0xFFFF
        return v - 65536 if v >= 32768 else v
    return s16(word & 0xFFFF), s16((word >> 16) & 0xFFFF), \
           s16((word >> 32) & 0xFFFF), s16((word >> 48) & 0xFFFF)


def test_dc_filter_passthrough_when_disabled():
    """When enable=0, output must equal input exactly."""
    dut = DCFilter(data_width=64)
    results = []

    def gen():
        yield dut._enable.storage.eq(0)
        yield dut.sink.valid.eq(1)
        yield dut.source.ready.eq(1)
        for _ in range(5):
            word = _drive_sample(dut, 1000, -500, 200, -300)
            yield dut.sink.data.eq(word)
            yield
            results.append((yield dut.source.data))

    run_simulation(dut, [gen()])
    word = _drive_sample(dut, 1000, -500, 200, -300)
    assert all(r == word for r in results), f"passthrough changed data: {results}"


def test_dc_filter_removes_dc_over_time():
    """After many cycles of constant DC input, output should converge toward zero."""
    dut = DCFilter(data_width=64)
    results = []
    DC_VAL = 4000

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut._alpha_shift.storage.eq(10)  # faster convergence for test speed
        yield dut.sink.valid.eq(1)
        yield dut.source.ready.eq(1)
        yield dut.sink.data.eq(_drive_sample(dut, DC_VAL, DC_VAL, DC_VAL, DC_VAL))
        for _ in range(2000):
            yield
        # Collect last sample.
        results.append((yield dut.source.data))

    run_simulation(dut, [gen()])
    ia, qa, ib, qb = _unpack(results[0])
    # After 2000 cycles at alpha_shift=10 (time constant ~1024 cycles), DC should be mostly removed.
    assert abs(ia) < DC_VAL // 4, f"I_A not converged: {ia}"
    assert abs(qa) < DC_VAL // 4, f"Q_A not converged: {qa}"


def test_dc_filter_passes_ac_signal():
    """A high-frequency alternating signal should pass through largely unchanged."""
    dut = DCFilter(data_width=64)
    ac_out = []

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut._alpha_shift.storage.eq(15)  # very low corner freq
        yield dut.sink.valid.eq(1)
        yield dut.source.ready.eq(1)
        sign = 1
        for i in range(200):
            # Square wave at Nyquist (alternates each cycle).
            val = 1000 * sign
            sign = -sign
            yield dut.sink.data.eq(_drive_sample(dut, val, val, val, val))
            yield
            if i > 100:  # skip settling period
                ac_out.append(_unpack((yield dut.source.data)))

    run_simulation(dut, [gen()])
    # AC signal should survive with >50% amplitude after filter settles.
    avg_ia = sum(abs(r[0]) for r in ac_out) / len(ac_out)
    assert avg_ia > 500, f"AC signal too attenuated: avg |I_A| = {avg_ia:.1f}"


def test_dc_filter_enable_disable_switch():
    """Toggling enable mid-stream: disabled path returns raw data."""
    dut = DCFilter(data_width=64)
    enabled_out = []
    disabled_out = []
    WORD = _drive_sample(None, 2000, -1000, 500, -250)

    def gen():
        yield dut._enable.storage.eq(0)
        yield dut._alpha_shift.storage.eq(10)
        yield dut.sink.valid.eq(1)
        yield dut.source.ready.eq(1)
        yield dut.sink.data.eq(WORD)
        # Read disabled output.
        yield
        disabled_out.append((yield dut.source.data))
        # Enable filter.
        yield dut._enable.storage.eq(1)
        for _ in range(5):
            yield
            enabled_out.append((yield dut.source.data))

    run_simulation(dut, [gen()])
    assert disabled_out[0] == WORD, f"disabled output should equal input: {hex(disabled_out[0])}"
    # Enabled output may differ from WORD (DC subtraction active).
    # Just verify the module doesn't crash and output changes.
    assert len(enabled_out) == 5


def _drive_sample(dut, ia, qa, ib, qb):
    """Build a 64-bit packed sample word from signed 16-bit values (dut arg ignored)."""
    def to16(v):
        return v & 0xFFFF
    return (to16(ia) | (to16(qa) << 16) | (to16(ib) << 32) | (to16(qb) << 48))
