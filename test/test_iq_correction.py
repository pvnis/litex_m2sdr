#!/usr/bin/env python3
#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from litex.gen.sim import run_simulation
from litex_m2sdr.gateware.iq_correction import IQCorrection


# IQ Correction Tests ------------------------------------------------------------------------------

def _pack(ia, qa, ib, qb):
    def to16(v):
        return v & 0xFFFF
    return to16(ia) | (to16(qa) << 16) | (to16(ib) << 32) | (to16(qb) << 48)


def _unpack(word):
    def s16(v):
        v &= 0xFFFF
        return v - 65536 if v >= 32768 else v
    return s16(word), s16(word >> 16), s16(word >> 32), s16(word >> 48)


def test_iq_correction_passthrough_when_disabled():
    """When enable=0 output must equal input."""
    dut = IQCorrection(data_width=64)
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


def test_iq_correction_identity_matrix():
    """Identity matrix (a=d=16384, b=c=0) must pass data unchanged."""
    dut = IQCorrection(data_width=64)
    results = []
    word = _pack(3000, -1500, 800, -400)

    def gen():
        yield dut._enable.storage.eq(1)
        # Identity: a=d=16384, b=c=0 (already reset values)
        yield dut.sink.valid.eq(1)
        yield dut.source.ready.eq(1)
        yield dut.sink.data.eq(word)
        for _ in range(5):
            yield
            results.append((yield dut.source.data))

    run_simulation(dut, [gen()])
    ia, qa, ib, qb = _unpack(word)
    for r in results:
        oia, oqa, oib, oqb = _unpack(r)
        assert oia == ia, f"I_A changed: {oia} != {ia}"
        assert oqa == qa, f"Q_A changed: {oqa} != {qa}"


def test_iq_correction_gain_scaling():
    """Setting a=8192 (=0.5 in Q2.14) should halve I_out."""
    dut = IQCorrection(data_width=64)
    results = []
    IA_IN = 4096

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut._ch_a_a.storage.eq(8192)   # 0.5x gain on I_A
        yield dut._ch_a_b.storage.eq(0)
        yield dut._ch_a_c.storage.eq(0)
        yield dut._ch_a_d.storage.eq(16384)  # 1.0x gain on Q_A
        yield dut.sink.valid.eq(1)
        yield dut.source.ready.eq(1)
        yield dut.sink.data.eq(_pack(IA_IN, 1000, 0, 0))
        for _ in range(5):
            yield
            results.append((yield dut.source.data))

    run_simulation(dut, [gen()])
    ia, qa, _, _ = _unpack(results[-1])
    assert ia == IA_IN // 2, f"expected I_A={IA_IN//2}, got {ia}"
    assert qa == 1000, f"Q_A should be unchanged: got {qa}"


def test_iq_correction_phase_mix():
    """Setting b=8192 mixes Q into I output: I_out = I + 0.5×Q."""
    dut = IQCorrection(data_width=64)
    results = []
    IA_IN, QA_IN = 2000, 1000

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut._ch_a_a.storage.eq(16384)  # a = 1.0
        yield dut._ch_a_b.storage.eq(8192)   # b = 0.5
        yield dut._ch_a_c.storage.eq(0)
        yield dut._ch_a_d.storage.eq(16384)
        yield dut.sink.valid.eq(1)
        yield dut.source.ready.eq(1)
        yield dut.sink.data.eq(_pack(IA_IN, QA_IN, 0, 0))
        for _ in range(5):
            yield
            results.append((yield dut.source.data))

    run_simulation(dut, [gen()])
    ia, qa, _, _ = _unpack(results[-1])
    expected_ia = IA_IN + QA_IN // 2   # 2000 + 500 = 2500
    assert abs(ia - expected_ia) <= 1, f"I_A phase mix: expected ~{expected_ia}, got {ia}"


def test_iq_correction_clamp():
    """Output must clamp to ±32767 on overflow."""
    dut = IQCorrection(data_width=64)
    results = []

    def gen():
        yield dut._enable.storage.eq(1)
        yield dut._ch_a_a.storage.eq(16384 * 2)  # 2.0x gain → overflow
        yield dut._ch_a_b.storage.eq(0)
        yield dut._ch_a_c.storage.eq(0)
        yield dut._ch_a_d.storage.eq(16384)
        yield dut.sink.valid.eq(1)
        yield dut.source.ready.eq(1)
        yield dut.sink.data.eq(_pack(30000, 0, 0, 0))  # 2×30000 = 60000 > 32767
        for _ in range(5):
            yield
            results.append((yield dut.source.data))

    run_simulation(dut, [gen()])
    ia, _, _, _ = _unpack(results[-1])
    assert ia == 32767, f"expected clamped 32767, got {ia}"
