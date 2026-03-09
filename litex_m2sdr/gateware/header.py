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

# Header Inserter/Extractor ------------------------------------------------------------------------

class HeaderInserterExtractor(LiteXModule):
    def __init__(self, mode="inserter", data_width=64, with_csr=True):
        assert data_width == 64
        assert mode in ["inserter", "extractor"]
        self.sink   = sink   = stream.Endpoint(dma_layout(data_width)) # i
        self.source = source = stream.Endpoint(dma_layout(data_width)) # o

        self.reset         = Signal() # i

        self.update        = Signal()   # o
        self.header        = Signal(64) # i (Inserter) / o (Extractor)
        self.timestamp     = Signal(64) # i (Inserter) / o (Extractor)

        self.enable        = Signal()   # i (CSR).
        self.header_enable = Signal()   # i (CSR).
        self.frame_cycles  = Signal(32) # i (CSR).

        # Optional PPS+ticks timestamp mode (inserter only).
        self.ts_mode     = Signal()   # i: 0=ns from TimeGenerator, 1=PPS seconds+ticks
        self.pps_seconds = Signal(32) # i: whole-seconds counter from PPSGenerator
        self.pps_ticks   = Signal(32) # i: sample ticks since last PPS edge

        # TX burst flags (extractor only) — upper 16 bits of the sync word.
        self.tx_flags    = Signal(16) # o: TX_FLAG_HAS_TIME, TX_FLAG_END_BURST, etc.

        if with_csr:
            self.add_csr()

        # # #

        # Signals.
        # --------
        cycles = Signal(32)
        frame_cycles_eff = Signal(32)
        self.comb += frame_cycles_eff.eq(Mux(self.frame_cycles == 0, 1, self.frame_cycles))

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
        fsm.act("IDLE",
            NextValue(cycles, 0),
            If(self.header_enable,
                NextState("HEADER")
            ).Else(
                NextState("FRAME")
            )
        )

        # Inserter specific.
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
                If(self.ts_mode,
                    # PPS+ticks mode: upper 32 bits = whole seconds, lower 32 bits = sample ticks.
                    source.data[0:32].eq(self.pps_ticks),
                    source.data[32:64].eq(self.pps_seconds),
                ).Else(
                    # Default: pure 64-bit nanosecond timestamp.
                    source.data[0:64].eq(self.timestamp),
                ),
                If(source.valid & source.ready,
                    NextValue(self.update, 1),
                    NextState("FRAME"),
                )
            )

        # Extractor specific.
        if mode == "extractor":
            # Header.
            fsm.act("HEADER",
                sink.ready.eq(1),
                If(sink.valid & sink.ready & sink.first,
                    NextValue(self.header, sink.data[0:64]),
                    NextValue(self.tx_flags, sink.data[48:64]),
                    NextState("TIMESTAMP")
                )
            )
            # Timestamp.
            fsm.act("TIMESTAMP",
                sink.ready.eq(1),
                If(sink.valid & sink.ready,
                    NextValue(self.timestamp, sink.data[0:64]),
                    NextValue(self.update, 1),
                    NextState("FRAME")
                )
            )

        # Frame.
        fsm.act("FRAME",
            sink.connect(source, omit={"first"}),
            NextValue(self.update, 0),
            If(self.header_enable,
                source.first.eq((cycles == 0) & (mode == "extractor")),
                source.last.eq( cycles == (frame_cycles_eff - 1)),
                If(source.valid & source.ready,
                    NextValue(cycles, cycles + 1),
                    If(source.last,
                        NextValue(cycles, 0),
                        NextState("HEADER")
                    )
                )
            )
        )

    def add_csr(self, default_enable=1, default_header_enable=0, default_frame_cycles=8192/8 - 2):
        self._control = CSRStorage(fields=[
            CSRField("enable", size=1, offset=0, values=[
                ("``0b0``", "Module Disabled."),
                ("``0b1``", "Module Enabled."),
            ], reset=default_enable),
            CSRField("header_enable", size=1, offset=1, values=[
                ("``0b0``", "Header Inserter/Extractor Disabled."),
                ("``0b1``", "Header Inserter/Extractor Enabled."),
            ], reset=default_header_enable),
        ])
        self._frame_cycles = CSRStorage(32, description="Frame Cycles (64-bit words)", reset=int(default_frame_cycles))

        # # #

        self.comb += [
            self.enable.eq(self._control.fields.enable),
            self.header_enable.eq(self._control.fields.header_enable),
            self.frame_cycles.eq(self._frame_cycles.storage),
        ]

# TX Header Extractor ------------------------------------------------------------------------------

class TXHeaderExtractor(HeaderInserterExtractor):
    def __init__(self, data_width=128, with_csr=True):
        HeaderInserterExtractor.__init__(self,
            mode       = "extractor",
            data_width = data_width,
            with_csr   = with_csr,
        )

# RX Header Inserter -------------------------------------------------------------------------------

class RXHeaderInserter(HeaderInserterExtractor):
    def __init__(self, data_width=128, with_csr=True):
        HeaderInserterExtractor.__init__(self,
            mode       = "inserter",
            data_width = data_width,
            with_csr   = with_csr,
        )

# TX/RX Header -------------------------------------------------------------------------------------

class TXRXHeader(LiteXModule):
    def __init__(self, data_width, with_csr=True):
        # TX.
        self.tx = TXHeaderExtractor(data_width, with_csr)

        # RX.
        self.rx = RXHeaderInserter(data_width, with_csr)

        # PPS+ticks timestamp mode signals (wired from litex_m2sdr.py).
        self.pps_seconds = Signal(32)
        self.pps_ticks   = Signal(32)
        self.comb += [
            self.rx.pps_seconds.eq(self.pps_seconds),
            self.rx.pps_ticks.eq(self.pps_ticks),
        ]

        # CSR.
        if with_csr:
            self._ts_mode = CSRStorage(fields=[
                CSRField("mode", size=1, offset=0, values=[
                    ("``0b0``", "Pure 64-bit nanosecond timestamp (default)."),
                    ("``0b1``", "PPS+ticks: upper 32 bits = whole seconds, lower 32 = sample ticks."),
                ], description="RX timestamp format selection.")
            ])
            self.comb += self.rx.ts_mode.eq(self._ts_mode.fields.mode)

            self.last_tx_header    = CSRStatus(64, description="Last TX Header.")
            self.last_tx_timestamp = CSRStatus(64, description="Last TX Timestamp.")
            self.last_tx_flags     = CSRStatus(16, description="Burst flags from last TX header (upper 16 bits of sync word).")
            self.last_rx_header    = CSRStatus(64, description="Last RX Header.")
            self.last_rx_timestamp = CSRStatus(64, description="Last RX Timestamp.")
            self.sync += [
                # Reset.
                If(self.tx.reset,
                    self.last_tx_header.status.eq(0),
                    self.last_tx_timestamp.status.eq(0),
                    self.last_tx_flags.status.eq(0),
                ),
                If(self.rx.reset,
                    self.last_rx_header.status.eq(0),
                    self.last_rx_timestamp.status.eq(0),
                ),
                # TX Update.
                If(self.tx.update,
                    self.last_tx_header.status.eq(self.tx.header),
                    self.last_tx_timestamp.status.eq(self.tx.timestamp),
                    self.last_tx_flags.status.eq(self.tx.tx_flags),
                ),
                # RX Update.
                If(self.rx.update,
                    self.last_rx_header.status.eq(self.rx.header),
                    self.last_rx_timestamp.status.eq(self.rx.timestamp),
                )
            ]
