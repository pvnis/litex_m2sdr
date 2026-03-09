#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *

# Sample Counter -----------------------------------------------------------------------------------

class SampleCounter(LiteXModule):
    """
    64-bit free-running integer sample counter.

    Increments once per RX sample beat (when rx_valid is asserted), providing a
    sample-domain timestamp equivalent to LimeSDR_GW's SAMPLE_NR.  This is useful
    for srsRAN / OAI stacks that express TX timestamps as integer sample counts
    rather than nanoseconds.

    The counter runs in the `sys` clock domain; `rx_valid` must be a sys-domain
    pulse (one pulse per received sample pair from the RX CDC output).

    CSRs:
        _control.read  -- write 1 to latch the current count into _count (pulse).
        _count         -- latched 64-bit sample count (read after pulsing read).
    """
    def __init__(self, rx_valid, with_csr=True):
        self.count = Signal(64)

        # # #

        self.sync += If(rx_valid, self.count.eq(self.count + 1))

        if with_csr:
            self.add_csr()

    def add_csr(self):
        self._control = CSRStorage(fields=[
            CSRField("read", size=1, offset=0, pulse=True,
                description="Pulse to latch the current sample count into _count."),
        ])
        self._count = CSRStatus(64, description="Latched sample count (latch by pulsing control.read).")

        # # #

        latch = Signal(64)
        self.sync += If(self._control.fields.read, latch.eq(self.count))
        self.comb += self._count.status.eq(latch)
