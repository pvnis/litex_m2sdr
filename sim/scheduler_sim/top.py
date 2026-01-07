#!/usr/bin/env python3

from migen import *
from migen.genlib.cdc import ClockDomainsRenamer
from litex.soc.interconnect import stream

from litex.gen import *


from litex_m2sdr.gateware.ad9361.scheduler import Scheduler

class Top(Module):
    def __init__(self, frames_per_packet=1024, data_width=64, max_packets=8):
        # If your scheduler normally runs in "rfic", keep it in sys for this sim
        self.clk = Signal()

        self.submodules.top = ClockDomainsRenamer("rfic")(Scheduler(frames_per_packet, data_width, max_packets))
        # Simple free-running timebase for 'now'
        self.now = Signal(64)
        self.enable = self.top.enable
        self.reset = self.top.reset
        self._write_time = self.top._write_time
        self._current_ts = self.top._current_ts
        self.write_time_manually = self.top.write_time_manually
        self.control = self.top._control
        # Shorthand handles
        self.sink   = self.top.sink
        self.source = self.top.source
        self.data_fifo = self.top.data_fifo
        # Always accept output (no backpressure) so it's easy to observe
        self.comb += [ self.source.ready.eq(1)]