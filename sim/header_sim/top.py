#!/usr/bin/env python3
from migen import *
from migen.genlib.cdc import ClockDomainsRenamer
from litex.soc.interconnect import stream

from litex_m2sdr.gateware.header import TXRXHeader

class Top(Module):
    def __init__(self, frames_per_packet=1024, data_width=64, max_packets=8):
        self.clk = Signal()
        self.header = TXRXHeader(data_width=64, with_csr=False)

        self.submodules.top = self.header.tx
        # Simple free-running timebase for 'now'
        self.now = Signal(64)

        self.enable = self.top.enable
        self.reset = self.top.reset
        self.header_enable = self.top.header_enable
        self.scheduling_enable = self.top.scheduling_enable
        # Shorthand handles
        self.sink   = self.top.sink
        self.source = self.top.source
        self.data_fifo = self.top.data_fifo
        # Always accept output (no backpressure) so it's easy to observe
        self.comb += [ 
            self.source.ready.eq(1),
            self.enable.eq(1),
            self.header_enable.eq(1),
            self.scheduling_enable.eq(1),
        ]
