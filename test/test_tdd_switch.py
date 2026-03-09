#!/usr/bin/env python3
#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from litex.gen.sim import run_simulation
from litex_m2sdr.gateware.tdd_switch import TDDSwitch


# Minimal stub so TDDSwitch can be instantiated without a full TimedTXArbiter.
class _TxActiveStub(Module):
    def __init__(self):
        self.tx_burst_active = Signal()


def _make_top():
    stub = _TxActiveStub()
    dut  = TDDSwitch(timed_tx_arbiter=stub)
    top  = Module()
    top.submodules += stub, dut
    return top, stub, dut


# Timing note: in Migen simulation, `yield signal.eq(v)` + `yield` applies the
# change AFTER the yield, so combinational detection (burst_rise/fall) is visible
# one yield later, and FSM NextState takes effect one further yield after that.
# Total pipeline: 2 yields from tx_burst_active edge to FSM state change visible.


# TDDSwitch Tests ----------------------------------------------------------------------------------

def test_tdd_pa_en_asserts_during_burst():
    """pa_en must be high while tx_burst_active is asserted (lead=trail=0)."""
    top, stub, dut = _make_top()
    during = []
    after  = []

    def gen():
        yield dut._lead_samples.storage.eq(0)
        yield dut._trail_samples.storage.eq(0)
        yield
        yield stub.tx_burst_active.eq(1)
        yield   # tx_burst_active becomes 1; burst_rise combinational
        yield   # FSM → LEAD, lead_counter=0 → NextState(TX_ON)
        yield   # FSM → TX_ON
        during.append((yield dut.pa_en))
        yield stub.tx_burst_active.eq(0)
        yield   # tx_burst_active becomes 0; burst_fall combinational; FSM TX_ON → NextState(TRAIL)
        yield   # FSM → TRAIL, trail_counter=0 → NextState(IDLE)
        yield   # FSM → IDLE
        after.append((yield dut.pa_en))

    run_simulation(top, [gen()])
    assert during[0] == 1, f"pa_en should be high during burst, got {during[0]}"
    assert after[0]  == 0, f"pa_en should de-assert after burst+trail, got {after[0]}"


def test_tdd_lead_delay():
    """pa_en asserts in LEAD state and stays high through TX_ON."""
    top, stub, dut = _make_top()
    lead_val = 3
    # We collect pa_en once FSM is definitely in LEAD (skip the first 0-cycle).
    in_lead = []

    def gen():
        yield dut._lead_samples.storage.eq(lead_val)
        yield dut._trail_samples.storage.eq(0)
        yield
        yield stub.tx_burst_active.eq(1)
        yield   # burst_rise fires
        yield   # FSM enters LEAD, lead_counter = lead_val; pa_en_int=1
        for _ in range(lead_val + 2):
            in_lead.append((yield dut.pa_en))
            yield
        yield stub.tx_burst_active.eq(0)
        yield

    run_simulation(top, [gen()])
    assert all(v == 1 for v in in_lead), f"pa_en dropped during lead/TX_ON: {in_lead}"


def test_tdd_trail_holds_pa_en():
    """pa_en stays high for trail_samples cycles after tx_burst_active falls."""
    top, stub, dut = _make_top()
    trail_val = 4
    post_burst = []

    def gen():
        yield dut._lead_samples.storage.eq(0)
        yield dut._trail_samples.storage.eq(trail_val)
        yield
        # Start burst.
        yield stub.tx_burst_active.eq(1)
        yield   # burst_rise
        yield   # FSM → LEAD → TX_ON (lead=0)
        yield   # FSM → TX_ON stable
        # End burst.
        yield stub.tx_burst_active.eq(0)
        yield   # burst_fall; FSM TX_ON → NextState(TRAIL)
        yield   # FSM → TRAIL, trail_counter = trail_val
        # Collect trail_val + 2 samples.
        for _ in range(trail_val + 2):
            post_burst.append((yield dut.pa_en))
            yield

    run_simulation(top, [gen()])
    # Trail counter counts from trail_val down to 0 (inclusive), so TRAIL stays
    # active for trail_val+1 cycles before NextState(IDLE) fires.  The final
    # sample (index trail_val+1) is in IDLE → pa_en=0.
    assert all(v == 1 for v in post_burst[:trail_val + 1]), \
        f"pa_en dropped early in trail: {post_burst}"
    assert post_burst[trail_val + 1] == 0, \
        f"pa_en should de-assert after trail, got {post_burst[trail_val + 1]}"


def test_tdd_disable_forces_pa_en_low():
    """When enable=0, pa_en must be 0 regardless of burst state."""
    top, stub, dut = _make_top()
    results = []

    def gen():
        yield dut._enable.storage.eq(0)
        yield dut._lead_samples.storage.eq(0)
        yield dut._trail_samples.storage.eq(0)
        yield
        yield stub.tx_burst_active.eq(1)
        for _ in range(5):
            yield
            results.append((yield dut.pa_en))

    run_simulation(top, [gen()])
    assert all(v == 0 for v in results), f"pa_en must stay 0 when disabled: {results}"


def test_tdd_invert_output():
    """When invert=1 output is active-low: 1 at idle, 0 during TX burst."""
    top, stub, dut = _make_top()
    at_idle      = []
    during_burst = []

    def gen():
        yield dut._invert.storage.eq(1)
        yield dut._lead_samples.storage.eq(0)
        yield dut._trail_samples.storage.eq(0)
        yield
        # Capture idle state (invert=1, pa_en_int=0 → output = ~0 = 1).
        at_idle.append((yield dut.pa_en))
        # Trigger burst.
        yield stub.tx_burst_active.eq(1)
        yield   # burst_rise
        yield   # FSM → LEAD/TX_ON; pa_en_int=1 → output = ~1 = 0
        yield
        during_burst.append((yield dut.pa_en))
        yield stub.tx_burst_active.eq(0)
        yield

    run_simulation(top, [gen()])
    assert at_idle[0]    == 1, f"inverted idle: expected 1, got {at_idle[0]}"
    assert during_burst[0] == 0, f"inverted burst: expected 0, got {during_burst[0]}"


def test_tdd_no_activity_without_burst():
    """pa_en stays 0 when tx_burst_active is never asserted."""
    top, stub, dut = _make_top()
    results = []

    def gen():
        yield dut._lead_samples.storage.eq(0)
        yield dut._trail_samples.storage.eq(0)
        yield
        for _ in range(20):
            results.append((yield dut.pa_en))
            yield

    run_simulation(top, [gen()])
    assert all(v == 0 for v in results), f"unexpected pa_en pulse at idle: {results}"
