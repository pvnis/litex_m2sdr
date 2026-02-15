#
# This file is responsible for scheduling TX based on timestamps.
#
# author: Ismail Essaidi
# 


from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer
from litex.gen import *
from litepcie.common import *
from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *


class Scheduler(LiteXModule):
    def __init__(self, frames_per_packet = 1024, data_width=64, max_packets=8, meta_data_frames_per_packet = 2, with_csr = True): # just to start then maybe expand it to more packets
        assert frames_per_packet % (data_width // 8) == 0 # packet size must be multiple of data width in bytes (a frame is 8 bytes)
        assert meta_data_frames_per_packet == 2  # we have 2 frames for metadata: header + timestamp

        # Inputs/Outputs
        self.reset = Signal() # i
        self.enable = Signal() # i
        self.sink   = sink   = stream.Endpoint(dma_layout(data_width)) # i  (from TX_CDC)
        self.source = source = stream.Endpoint(dma_layout(data_width)) # o  (to gpio_tx_unpacker)
        self.timestamp = Signal(64) # i (timestamp of the packet at fifo output)
        self.header = Signal(64) # i (header of the packet at fifo output)
        self.time_input = Signal(64) # i (current time input from TimeGenerator via CDC)
        self.write_time_manually = Signal()
        self.manual_now = Signal(64)

        # Data Fifo: stores multiple packets
        self.data_fifo = data_fifo = stream.SyncFIFO(layout=dma_layout(data_width), depth=frames_per_packet * max_packets)

        # Internal time register - driven from external TimeGenerator or manual override
        # self.now is the actual time used by the scheduler logic
        self.now = Signal(64)
        self.sync += [
            If(self.write_time_manually,
                self.now.eq(self.manual_now),
            ).Else(
                self.now.eq(self.time_input),  # Track external time from TimeGenerator
            )
        ]

        # States
        self.streaming = streaming =  Signal(reset=0)
        self.frame_count = frame_count = Signal(10)   # enough for 0..1023 (1024 frames)
        self.latched_ts = latched_ts = Signal(64)
        latched_header = Signal(64)

        # ---------------------------
        # Input Assignement
        # ---------------------------
        # Default: pure bypass
        self.comb += [
            source.valid.eq(sink.valid),
            source.data.eq(sink.data),
            source.first.eq(sink.first),
            source.last.eq(sink.last),
            sink.ready.eq(source.ready),
        ]
        # Scheduled mode overrides
        self.comb += [
            If(self.enable,  # Always Read from upstream if I can
                sink.ready.eq(data_fifo.sink.ready), # backpressure
                data_fifo.sink.valid.eq(sink.valid),
                data_fifo.sink.data.eq(sink.data), # push data with timestamp and header into fifo
                data_fifo.sink.first.eq(sink.first),
                data_fifo.sink.last.eq(sink.last),
            ).Else(
                data_fifo.sink.valid.eq(0),
                data_fifo.sink.data.eq(0),
                data_fifo.sink.first.eq(0),
                data_fifo.sink.last.eq(0),
            )
        ]

        # ---------------------------
        # Capture header + timestamp
        # ---------------------------
        # This triggers only once per packet.
        is_header    = Signal()
        is_timestamp = Signal()
        self.comb += [ 
            is_header.eq(data_fifo.source.valid & ~streaming & (frame_count == 0)),
            is_timestamp.eq(data_fifo.source.valid & ~streaming & (frame_count == 1)),
        ]

        # ------------------------------------------------
        # Latching Metadata only when we consume from Fifo
        # ------------------------------------------------
        self.sync += [
            If(data_fifo.source.ready & data_fifo.source.valid & self.enable, # enable added as a condition
                If(is_header,   
                    latched_header.eq(data_fifo.source.data)
                ),
                If(is_timestamp,
                    latched_ts.eq(data_fifo.source.data),
                )
            )
        ]

        # ---------------------------
        # Start streaming when time has come
        # ---------------------------
        # Use signed comparison to detect if we've passed the timestamp
        # This handles wraparound correctly
        time_diff = Signal((65, True))  # Signed 65-bit for safe subtraction
        self.comb += time_diff.eq(self.now - latched_ts)

        can_start = Signal()
        self.comb += can_start.eq((time_diff >= 0) &  # Current time >= target time
                                (frame_count == 2) &   # Metadata consumed
                                data_fifo.source.valid &
                                self.enable)
        self.sync += [
            If(can_start & ~streaming,
                streaming.eq(1)
            )
        ]
        # ---------------------------
        # Assign output
        # ---------------------------
        self.comb += [
            If(self.enable,
                source.valid.eq(streaming & data_fifo.source.valid),
                source.data.eq(data_fifo.source.data),
                source.first.eq(data_fifo.source.first & streaming),
                source.last.eq(data_fifo.source.last & streaming)
            )
        ]
        pop_meta = is_header | is_timestamp
        self.comb += data_fifo.source.ready.eq(self.enable & 
                                               ((source.ready & streaming) | pop_meta))  # Pop FIFO when downstream is ready

        # ---------------------------
        # Count frames
        # ---------------------------
        self.sync += [
            If(data_fifo.source.ready & data_fifo.source.valid & self.enable,
                frame_count.eq(frame_count + 1)
            )
        ]
        # ---------------------------
        # Reset for next packet
        # ---------------------------
        self.sync += [
            If(~self.enable | ((frame_count == frames_per_packet - 1) & streaming & data_fifo.source.ready & data_fifo.source.valid),
                frame_count.eq(0),
                streaming.eq(0)
            )
        ]
        # ────────────────────────────────────────────────
        #      CSR Exposure
        # ────────────────────────────────────────────────
        if with_csr:
            self.add_csr()

    
    def add_csr(self):
        # Control
        self._control = CSRStorage(fields=[
            CSRField("read", size=1, offset=1, pulse=True),
            CSRField("write", size=1, offset=2, pulse=True),  # For manual override only (debug)
            CSRField("read_current_ts", size=1, offset=3, pulse=True),
        ])

        # Status
        self._fifo_level = CSRStatus(16, description="Current FIFO level in words")

        self._current_ts = CSRStatus(64, description="Current Timestamp at FIFO output (latched_ts)")
        self._read_time  = CSRStatus(64, description="Current time in nanoseconds (driven by TimeGenerator)")
        self._write_time = CSRStorage(64, description="Manual Time Override (ns) (DEBUG ONLY - normally driven by TimeGenerator)")
        # self._status     = CSRStatus(fields=[
        #     CSRField("sink_valid", size=1, offset=0),
        #     CSRField("sink_ready", size=1, offset=1),
        #     CSRField("source_valid", size=1, offset=2),
        #     CSRField("source_ready", size=1, offset=3),
        #     CSRField("fifo_sink_ready", size=1, offset=4),
        #     CSRField("fifo_source_ready", size=1, offset=5),
        # ])
        # self.comb += [
        #     self._status.fields.sink_valid.eq(self.sink.valid),
        #     self._status.fields.sink_ready.eq(self.sink.ready),
        #     self._status.fields.source_valid.eq(self.source.valid),
        #     self._status.fields.source_ready.eq(self.source.ready),
        #     # self._status.fields.fifo_sink_ready.eq(self.data_fifo.sink.ready),
        #     # self._status.fields.fifo_source_ready.eq(self.data_fifo.source.ready),
        # ]

        #Debug
        # self._streaming = CSRStatus(1,   description="Scheduler status: streaming flag")
        # self.comb+= [
        #     self._streaming.status.eq(self.streaming),
        # ]
        # ---------------------------
        # Important to Sync from scheduler domain to sys domain

        # Time Write (SW -> FPGA) - Manual override for debug only
        self.specials += MultiReg(self._write_time.storage, self.manual_now, "rfic")
        time_write_ps = PulseSynchronizer("sys", "rfic")
        self.submodules += time_write_ps
        self.comb += time_write_ps.i.eq(self._control.fields.write)
        self.comb += self.write_time_manually.eq(time_write_ps.o)

        # Time Read (RFIC -> SW) - Read current time for debug/verification
        time_read = Signal(64)
        time_read_ps = PulseSynchronizer("sys", "rfic")
        self.submodules += time_read_ps
        self.comb += time_read_ps.i.eq(self._control.fields.read)
        self.sync.rfic += If(time_read_ps.o, time_read.eq(self.now))
        self.specials += MultiReg(time_read, self._read_time.status)         # rfic -> sys

        # Latched Timestamp Read (RFIC -> SW) - For verification
        current_ts_read = Signal(64)
        current_ts_read_ps = PulseSynchronizer("sys", "rfic")
        self.submodules += current_ts_read_ps
        self.comb += current_ts_read_ps.i.eq(self._control.fields.read_current_ts)
        self.sync.rfic += If(current_ts_read_ps.o, current_ts_read.eq(self.latched_ts))
        self.specials += MultiReg(current_ts_read, self._current_ts.status)  # rfic -> sys