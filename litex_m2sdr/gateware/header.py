#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2025 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litepcie.common import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *


# Header Inserter/Extracter ------------------------------------------------------------------------

class HeaderInserterExtracter(LiteXModule):
    def __init__(self, mode="inserter", data_width=64, with_csr=True, frames_per_packet=1024,  max_packets=8):
        assert data_width == 64 # 8 bytes
        assert mode in ["inserter", "extracter"]
        self.sink   = sink   = stream.Endpoint(dma_layout(data_width)) # i   
        self.source = source = stream.Endpoint(dma_layout(data_width)) # o  

        self.reset         = Signal() # i

        self.update        = Signal()   # o
        self.header        = Signal(64) # i (Inserter) / o (Extracter)
        self.timestamp     = Signal(64) # i (Inserter) / o (Extracter)

        self.enable        = Signal()   # i (CSR).
        self.header_enable = Signal()   # i (CSR).
        self.frame_cycles  = Signal(32) # i (CSR).
        self.scheduling_enable = Signal()  # i (CSR)
        self.frame_count = frame_count = Signal(10)  # enough for 0..1023
        # Data Fifo: stores multiple packets
        self.data_fifo = data_fifo = stream.SyncFIFO(layout=dma_layout(data_width), depth=frames_per_packet * max_packets)

        # States and signals
        self.streaming = streaming = Signal(reset=0)
        self.latched_ts = latched_ts = Signal(64)
        self.latched_header = latched_header = Signal(64)
        self.packet_end = packet_end = Signal()
        self.now = Signal(64)  # i (current time)

        
        if with_csr:
            self.add_csr()

        # # #

        # Signals.
        # --------
        cycles = Signal(32)

        # FSM.
        # ----
        self.fsm = fsm = ResetInserter()(FSM(reset_state="RESET"))
        self.comb += self.fsm.reset.eq(self.reset | ~self.enable)

        # Reset.
        fsm.act("RESET",
            NextValue(cycles, 0),
            If(mode == "inserter",
                sink.ready.eq(self.reset)
            ),
            NextState("IDLE")
        )

        # Idle.
        fsm.act("IDLE", # In scheduling mode: IDLE is when we fill up the fifo but don't pop from it (condition to pop not satisfied)
            NextValue(cycles, 0),
            If(self.header_enable & ((~self.scheduling_enable) | (self.scheduling_enable & (frame_count == 0) & data_fifo.source.valid)), # in scheduling mode: equivalent to is_header
                NextState("HEADER")
            ).Elif(~self.header_enable, 
                NextState("FRAME")
            )
        )

        # Inserter specific. (RX)
        if mode == "inserter":
            # Header.
            fsm.act("HEADER",
                source.valid.eq(1),
                source.first.eq(1),
                source.data[0:64].eq(self.header),
                If(source.valid & source.ready,
                    NextState("TIMESTAMP"),
                )
            )
            # Timestamp.
            fsm.act("TIMESTAMP",
                source.valid.eq(1),
                source.data[0:64].eq(self.timestamp),
                If(source.valid & source.ready,
                    NextValue(self.update, 1), # only update for a new frame
                    NextState("FRAME"),
                )
            )

        # Extracter specific. (TX)
        if mode == "extracter":
            # Scheduling 
            self.write_time_manually = Signal()  # i (for manual time write)
            self.manual_now = Signal(64)  # i (manual time value)
            # Time Handling.
            self.sync += [
                If(self.write_time_manually,
                    self.now.eq(self.manual_now),
                ).Else(
                    self.now.eq(self.now + self.scheduling_enable),  # Increment only if scheduling enabled
                )
            ]

            # ---------------------------
            # Input Assignment
            # ---------------------------
            scheduled_mode = self.scheduling_enable
            self.comb += [
                If(scheduled_mode,
                    sink.ready.eq(data_fifo.sink.ready),  # backpressure
                    data_fifo.sink.valid.eq(sink.valid),
                    data_fifo.sink.data.eq(sink.data),  # push data with timestamp and header into fifo
                    data_fifo.sink.first.eq(sink.first),
                    data_fifo.sink.last.eq(sink.last),
                ).Else(
                    # Bypass: Direct connection
                    source.valid.eq(sink.valid),
                    source.data.eq(sink.data),
                    source.first.eq(sink.first),
                    source.last.eq(sink.last),
                    sink.ready.eq(source.ready),
                    # Disable FIFO input
                    data_fifo.sink.valid.eq(0),
                    data_fifo.sink.data.eq(0),
                    data_fifo.sink.first.eq(0),
                    data_fifo.sink.last.eq(0),
                )
            ]

            # ---------------------------
            # Capture header + timestamp 
            # ---------------------------
            is_header = Signal()
            is_timestamp = Signal()
            self.comb += [
                is_header.eq(data_fifo.source.valid & ~streaming & (frame_count == 0)),
                is_timestamp.eq(data_fifo.source.valid & ~streaming & (frame_count == 1)),
            ]

            # Latching Metadata 
            self.sync += [
                If(data_fifo.source.ready & data_fifo.source.valid & self.scheduling_enable,
                    If(is_header,
                        latched_header.eq(data_fifo.source.data)
                    ),
                    If(is_timestamp,
                        latched_ts.eq(data_fifo.source.data),
                    )
                )
            ]

            # Start streaming when time has come
            can_start = Signal()
            self.comb += can_start.eq((self.now >= latched_ts) &
                                      (frame_count == 2) &
                                      data_fifo.source.valid &
                                      self.scheduling_enable)
            self.sync += [
                If(can_start & ~streaming,
                    streaming.eq(1)
                )
            ]

            # Assign output (release from FIFO)
            self.comb += [
                If(self.scheduling_enable,
                    source.valid.eq(streaming & data_fifo.source.valid),
                    If(streaming,
                        source.data.eq(data_fifo.source.data),
                        source.first.eq(data_fifo.source.first),
                        source.last.eq(data_fifo.source.last)
                    )
                )
            ]

            # Pop FIFO logic (from Scheduler)
            pop_meta = is_header | is_timestamp
            self.comb += data_fifo.source.ready.eq(self.scheduling_enable &
                                                   ((source.ready & streaming) | pop_meta))

            # Count frames
            self.sync += [
                If(data_fifo.source.ready & data_fifo.source.valid & self.scheduling_enable,
                    frame_count.eq(frame_count + 1)
                )
            ]

            # Reset for next packet
            self.sync += [
                If(~self.scheduling_enable | ((frame_count == frames_per_packet - 1) & streaming & data_fifo.source.ready & data_fifo.source.valid),
                    frame_count.eq(0),
                    streaming.eq(0)
                )
            ]
            # Header.
            fsm.act("HEADER",
                If(self.scheduling_enable,
                    # In scheduling mode, header is latched when consumed
                    NextValue(self.header, latched_header),
                    NextState("TIMESTAMP")
                ).Else(
                    sink.ready.eq(1),
                    If(sink.valid & sink.ready & sink.first,
                        NextValue(self.header, sink.data[0:64]),
                        NextState("TIMESTAMP")
                    )
                )
            )

            # Timestamp.
            fsm.act("TIMESTAMP",
                If(self.scheduling_enable,
                    # Timestamp is latched
                    NextValue(self.timestamp, latched_ts),
                    NextValue(self.update, 1),
                    NextState("FRAME")
                ).Else(
                    sink.ready.eq(1),
                    If(sink.valid & sink.ready,
                        NextValue(self.timestamp, sink.data[0:64]),
                        NextValue(self.update, 1),  # only update for a new packet
                        NextState("FRAME")
                    )
                )
            )

        # Frame.
        connect_bypass = (mode == "inserter") | (~self.scheduling_enable)
        self.comb += packet_end.eq((frame_count == frames_per_packet - 1) & streaming & data_fifo.source.ready & data_fifo.source.valid & self.scheduling_enable)
        fsm.act("FRAME",
            NextValue(self.update, 0),
            If(packet_end,
                NextState("HEADER")
            ).Elif(self.header_enable,
                source.first.eq((cycles == 0) & (mode == "extracter")),
                source.last.eq(cycles == (self.frame_cycles - 1)),
                If(source.valid & source.ready,
                    NextValue(cycles, cycles + 1),
                    If(source.last,
                        NextValue(cycles, 0),
                        NextState("HEADER")
                    )
                )
            ),
            If(connect_bypass,
                sink.connect(source, omit={"first"})
            )
        )

    def add_csr(self, default_enable=1, default_header_enable=0, default_scheduler_enable=0, default_frame_cycles=8192/8 - 2):
        self._control = CSRStorage(fields=[
            CSRField("enable", size=1, offset=0, values=[
                ("``0b0``", "Module Disabled."),
                ("``0b1``", "Module Enabled."),
            ], reset=default_enable),
            CSRField("header_enable", size=1, offset=1, values=[
                ("``0b0``", "Header Inserter/Extracter Disabled."),
                ("``0b1``", "Header Inserter/Extracter Enabled."),
            ], reset=default_header_enable),
            CSRField("scheduling_enable", size=1, offset=2, values=[
                ("``0b0``", "TX scheduler Disabled"),
                ("``0b1``", "TX scheduler Enabled."),
            ], reset=default_scheduler_enable),
        ])
        self._frame_cycles = CSRStorage(32, description="Frame Cycles (64-bit word)", reset=int(default_frame_cycles))

        # # #

        self.comb += [
            self.enable.eq(self._control.fields.enable),
            self.header_enable.eq(self._control.fields.header_enable),
            self.scheduling_enable.eq(self._control.fields.scheduling_enable),
            self.frame_cycles.eq(self._frame_cycles.storage),
        ]

# TX Header Extracter ------------------------------------------------------------------------------

class TXHeaderExtracter(HeaderInserterExtracter):
    def __init__(self, data_width=128, with_csr=True):
        HeaderInserterExtracter.__init__(self,
            mode       = "extracter",
            data_width = data_width,
            with_csr   = with_csr,
        )

# RX Header Inserter -------------------------------------------------------------------------------

class RXHeaderInserter(HeaderInserterExtracter):
    def __init__(self, data_width=128, with_csr=True):
        HeaderInserterExtracter.__init__(self,
            mode       = "inserter",
            data_width = data_width,
            with_csr   = with_csr,
        )

# TX/RX Header -------------------------------------------------------------------------------------

class TXRXHeader(LiteXModule):
    def __init__(self, data_width, with_csr=True):
        # TX.
        self.tx = TXHeaderExtracter(data_width, with_csr)

        # RX.
        self.rx = RXHeaderInserter(data_width, with_csr)

        # CSR.
        if with_csr:
            self.last_tx_header    = CSRStatus(64, description="Last TX Header.")
            self.last_tx_timestamp = CSRStatus(64, description="Last TX Timestamp.")
            self.last_rx_header    = CSRStatus(64, description="Last RX Header.")
            self.last_rx_timestamp = CSRStatus(64, description="Last RX Timestamp.")
            self.sync += [
                # Reset.
                If(self.tx.reset,
                    self.last_tx_header.status.eq(0),
                    self.last_tx_timestamp.status.eq(0),
                ),
                If(self.rx.reset,
                    self.last_rx_header.status.eq(0),
                    self.last_rx_timestamp.status.eq(0),
                ),
                # TX Update.
                If(self.tx.scheduling_enable,
                    If(self.tx.packet_end, # only when a new frame is started
                        self.last_tx_header.status.eq(self.tx.latched_header),
                        self.last_tx_timestamp.status.eq(self.tx.latched_ts)
                    )
                ).Else(
                    If(self.tx.update, # only when a new frame is started
                        self.last_tx_header.status.eq(self.tx.header),
                        self.last_tx_timestamp.status.eq(self.tx.timestamp),
                    )
                ),
                # RX Update.
                If(self.rx.update, # only when a new frame is started
                    self.last_rx_header.status.eq(self.rx.header),
                    self.last_rx_timestamp.status.eq(self.rx.timestamp),
                )
            ]
