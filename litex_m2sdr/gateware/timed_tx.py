#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *

from litepcie.common import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

# Timed TX Arbiter ---------------------------------------------------------------------------------

class TimedTXArbiter(LiteXModule):
    """
    Holds TX payload packets until TimeGenerator.time >= the packet's TX timestamp.

    Placement: after the TX header extractor. The extractor strips the DMA header
    (sync word + timestamp) from the stream and exposes the timestamp via its
    `timestamp` signal, pulsing `update` once per frame when a new timestamp is latched.

    A companion timestamp FIFO (ts_fifo, depth=16) captures one timestamp per DMA
    frame, precisely ordered with the payload data flowing into the main data FIFO.

    FSM:
        IDLE      -- wait for a payload frame AND a matching timestamp in ts_fifo
        WAIT_TIME -- hold until time_gen.time >= ts_reg
        STREAM    -- forward samples to output until last=1
    """
    def __init__(self, header_extractor, time_gen, fifo_depth=2048, data_width=64):
        self.sink   = sink   = stream.Endpoint(dma_layout(data_width))
        self.source = source = stream.Endpoint(dma_layout(data_width))

        # CSRs.
        self._enable         = CSRStorage(1, reset=0,
            description="Timed TX enable: 1=hold until timestamp, 0=pass-through.")
        self._late_count     = CSRStatus(32,
            description="TX frames received after their deadline (late packets).")
        self._underrun_count = CSRStatus(32,
            description="TX underrun cycles (IDLE with no data ready).")

        # # #

        # Data FIFO: buffers incoming payload samples (first/last framed).
        self.data_fifo = data_fifo = stream.SyncFIFO(dma_layout(data_width), fifo_depth)
        self.comb += sink.connect(data_fifo.sink)

        # Timestamp FIFO: one slot per DMA frame, fed by header_extractor.update pulses.
        self.ts_fifo = ts_fifo = stream.SyncFIFO([("ts", 64)], 16)
        self.comb += [
            ts_fifo.sink.valid.eq(header_extractor.update),
            ts_fifo.sink.ts.eq(header_extractor.timestamp),
        ]

        # Burst-active status (for TDDSwitch).
        self.tx_burst_active = Signal()

        # Registers.
        ts_reg         = Signal(64)
        late_count     = Signal(32)
        underrun_count = Signal(32)

        self.comb += [
            self._late_count.status.eq(late_count),
            self._underrun_count.status.eq(underrun_count),
        ]

        # FSM.
        self.fsm = fsm = FSM(reset_state="IDLE")

        fsm.act("IDLE",
            If(~self._enable.storage,
                # Pass-through: bypass timestamp check, forward data directly.
                data_fifo.source.connect(source),
            ).Elif(data_fifo.source.valid & data_fifo.source.first & ts_fifo.source.valid,
                # New frame present with a matching timestamp: latch and decide.
                ts_fifo.source.ready.eq(1),
                NextValue(ts_reg, ts_fifo.source.ts),
                # Count frames that were already past their deadline on arrival.
                If(time_gen.time > ts_fifo.source.ts,
                    NextValue(late_count, late_count + 1),
                ),
                NextState("WAIT_TIME"),
            ).Elif(~data_fifo.source.valid,
                # No data: count underrun cycles.
                NextValue(underrun_count, underrun_count + 1),
            )
        )

        fsm.act("WAIT_TIME",
            # Release when the hardware clock reaches (or passes) the TX timestamp.
            If(time_gen.time >= ts_reg,
                NextState("STREAM"),
            )
        )

        fsm.act("STREAM",
            self.tx_burst_active.eq(1),
            data_fifo.source.connect(source),
            If(source.valid & source.ready & source.last,
                NextState("IDLE"),
            )
        )
