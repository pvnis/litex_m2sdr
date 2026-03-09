#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litex.soc.interconnect.csr import *

# TDD Switch ---------------------------------------------------------------------------------------

class TDDSwitch(LiteXModule):
    """
    TDD TX/RX switch driver with programmable lead/lag offsets.

    Drives a PA_EN / TR_SW GPIO output aligned to TX burst boundaries.
    When a TX burst starts (tx_burst_active rises), the output is asserted
    _lead_samples cycles early (LEAD state).  When the burst ends
    (tx_burst_active falls), the output stays asserted for _trail_samples
    more cycles (TRAIL state), then de-asserts (IDLE).

    Counters operate in sys_clk cycles.  For sample-accurate timing the
    CDC latency from rfic→sys (~5 cycles) should be subtracted from
    _lead_samples in software, or absorbed into the default reset value.

    Ports:
        pa_en  -- combinational output; connect to GPIO pad and/or
                  ad9361.txnrx_override in litex_m2sdr.py.
    """
    def __init__(self, timed_tx_arbiter):
        self.pa_en = Signal()

        # CSRs.
        self._enable        = CSRStorage(1,  reset=1,
            description="TDD switch enable: 1=driven by burst state, 0=forced off.")
        self._invert        = CSRStorage(1,  reset=0,
            description="Invert output polarity (1=active-low PA_EN).")
        self._lead_samples  = CSRStorage(16, reset=20,
            description="Assert PA_EN this many sys_clk cycles before TX burst starts.")
        self._trail_samples = CSRStorage(16, reset=10,
            description="Hold PA_EN this many sys_clk cycles after TX burst ends.")

        # # #

        pa_en_int     = Signal()
        lead_counter  = Signal(16)
        trail_counter = Signal(16)

        # Detect rising / falling edge of tx_burst_active.
        burst_active_r = Signal()
        self.sync += burst_active_r.eq(timed_tx_arbiter.tx_burst_active)
        burst_rise = Signal()
        burst_fall = Signal()
        self.comb += [
            burst_rise.eq( timed_tx_arbiter.tx_burst_active & ~burst_active_r),
            burst_fall.eq(~timed_tx_arbiter.tx_burst_active &  burst_active_r),
        ]

        # FSM.
        self.fsm = fsm = FSM(reset_state="IDLE")

        fsm.act("IDLE",
            If(burst_rise,
                NextValue(lead_counter, self._lead_samples.storage),
                NextState("LEAD"),
            )
        )

        fsm.act("LEAD",
            pa_en_int.eq(1),
            If(lead_counter == 0,
                NextState("TX_ON"),
            ).Else(
                NextValue(lead_counter, lead_counter - 1),
            )
        )

        fsm.act("TX_ON",
            pa_en_int.eq(1),
            If(burst_fall,
                NextValue(trail_counter, self._trail_samples.storage),
                NextState("TRAIL"),
            )
        )

        fsm.act("TRAIL",
            pa_en_int.eq(1),
            If(trail_counter == 0,
                NextState("IDLE"),
            ).Else(
                NextValue(trail_counter, trail_counter - 1),
            )
        )

        # Output mux: enable + optional polarity invert.
        self.comb += self.pa_en.eq(
            Mux(self._enable.storage,
                Mux(self._invert.storage, ~pa_en_int, pa_en_int),
                0)
        )
