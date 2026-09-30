#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer, GrayDecoder

from litex.gen import *

from litex.soc.interconnect     import stream
from litex.soc.interconnect.csr import *

from litepcie.common import *

from litex_m2sdr.gateware.gpio import GPIORXPacker, GPIOTXUnpacker
from litex_m2sdr.gateware.sample_time import RXTickTracker, TXFineGate, rx_tick_layout, tx_stamp_layout

from litex_m2sdr.gateware.ad9361.phy     import AD9361PHY
from litex_m2sdr.gateware.ad9361.spi     import AD9361SPIMaster
from litex_m2sdr.gateware.ad9361.bitmode import AD9361TXBitMode, AD9361RXBitMode
from litex_m2sdr.gateware.ad9361.bitmode import _sign_extend
from litex_m2sdr.gateware.ad9361.prbs    import AD9361PRBSGenerator, AD9361PRBSChecker
from litex_m2sdr.gateware.ad9361.prbs    import AD9361PRBS1R1TGenerator, AD9361PRBS1R1TChecker
from litex_m2sdr.gateware.ad9361.agc     import (
    AGC_DEFAULT_HIGH_THRESHOLD,
    AGC_DEFAULT_LOW_THRESHOLD,
    AGCSaturationCount,
)

# Architecture -------------------------------------------------------------------------------------
#
# The AD9361 PHY has the following simplified architecture:
#                                                                 ┌───────────────────┐
#                                                                 │                   │
#                                                                 │     SPI Core      ├──► SPI
#                                                                 │                   │
#                                                                 └───────────────────┘
#                               ┌────────────┐  ┌───┐
#                               │            │  │   │      ┌──────────────────────────┐
#                               │  RX PRBS   ◄──┤   │      │                          │
#                               │            │  │ D │      │          ┌───────────┐   │
#                               └────────────┘  │ E │      │          │  RX Data  │   │
#                                               │ M ◄──────┼──────────┤    2:1    ◄───┼── RX Data
#                ┌──────┐  ┌──────┐   ┌──────┐  │ U │      │          │    DDR    │   │
#     Source     │      │  │12-bit│   │      │  │ X │      │          └───────────┘   │
#    (To DMA) ◄──┤ BUF  ◄──┤ 8-bit◄───┤ CDC  ◄──┤   ◄─┐    │                    X6    │
#                │      │  │ mode │   │      │  │   │ │    │                          │  From AD9361
#                └──────┘  └──────┘   └──────┘  └───┘ │    │                          │
#                                                     │    │          ┌───────────┐   │
#                                                     │    │          │  RX Clk   │   │
#                                                     │    │      ┌───┤    BUF    ◄───┼── RX Clk
#                                                    T│    │      │   │           │   │
#                                                    X│    │      │   └───────────┘   │
#                                                    -│    │      │                   │
#                                                    R│    │      │                   │
#                                                    X│    │      │RFIC Clk           │
#                                                    -│    │      │                   │
#                                                    L│    │      │                   │
#                                                    o│    │      │                   │
#                                                    o│    │      │   ┌───────────┐   │
#                                                    p│    │      │   │  TX Clk   │   │
#                                                    b│    │      └───►    2:1    ├───┼─► TX Clk
#                                                    a│    │          │    DDR    │   │
#                                                    c│    │          └───────────┘   │
#                                                    k│    │                          │
#                              ┌────────────┐  ┌───┐  │    │                          │
#                              │            │  │   │  │    │                          │  To AD9361
#                              │  TX PRBS   ├──►   │  │    │                          │
#                              │            │  │   │  │    │          ┌───────────┐   │
#                              └────────────┘  │ M │  │    │          │  TX Data  │   │
#                                              │ U ├──┴────┼──────────►    2:1    ├───┼─► TX Data
#                ┌──────┐  ┌──────┐  ┌──────┐  │ X │       │          │    DDR    │   │
#    Sink        │      │  │12-bit│  │      │  │   │       │          └───────────┘   │
#   (From DMA) ──►  BUF ├──► 8-bit├──► CDC  ├──►   │       │                    X6    │
#                │      │  │ mode │  │      │  │   │       │                          │
#                └──────┘  └──────┘  └──────┘  └───┘       │            PHY           │
#                                                          └──────────────────────────┘
# - The rfic_clk is recovered from the AD9361 RX Clk through a Clk buffer.
# - The rfic_clk is used for both TX/RX.
# - 2:1 Serialization/Deserialiation is used on TX/RX.
# - RX sampling (on the FPGA) is adjusted through AD9361 registers.
# - TX sampling (on the AD931) is adjusted through AD9361 registers.
# - An optional TX-RX loopback is implemented.
# - Sink/Source stream operate in sys_clk domain @ 64-bit and are converted to/from rfic_clk.

# AD9361 RFIC --------------------------------------------------------------------------------------

class AD9361RFICStreamBypass(LiteXModule):
    def __init__(self, layout=None):
        layout = dma_layout(64) if layout is None else layout
        self.sink   = stream.Endpoint(layout)
        self.source = stream.Endpoint(layout)

        # # #

        self.comb += self.sink.connect(self.source)


class AD9361RFIC(LiteXModule):
    def __init__(self, rfic_pads, spi_pads, sys_clk_freq,
        with_tx_fifo = False, tx_fifo_depth = 8192,
        with_rx_fifo = False, rx_fifo_depth = 8192):
        # Stream Endpoints -------------------------------------------------------------------------
        self.sink   = stream.Endpoint(dma_layout(64))
        self.source = stream.Endpoint(dma_layout(64))

        # Config/Control/Status registers ----------------------------------------------------------
        self._config = CSRStorage(fields=[
            CSRField("rst_n",  size=1, offset=0, values=[
                ("``0b0``", "Reset the AD9361."),
                ("``0b1``", "Enable the AD9361."),
            ]),
            CSRField("enable", size=1, offset=1, values=[
                ("``0b0``", "AD9361 disabled."),
                ("``0b1``", "AD9361 enabled."),
            ]),
            CSRField("txnrx",  size=1, offset=4, values=[
                ("``0b0``", "Set to TX mode."),
                ("``0b1``", "Set to RX mode."),
            ]),
            CSRField("en_agc", size=1, offset=5, values=[
                ("``0b0``", "Disable AGC."),
                ("``0b1``", "Enable AGC."),
            ]),
        ])
        self._ctrl = CSRStorage(fields=[
            CSRField("ctrl", size=4, offset=0, values=[
                ("``0b0000``", "All control pins low."),
                ("``0b1111``", "All control pins high."),
            ], description="AD9361's control pins.")
        ])
        self._stat = CSRStatus(fields=[
            CSRField("stat", size=8, offset=0, values=[
                ("``0b00000000``", "All status pins low."),
                ("``0b11111111``", "All status pins high."),
            ], description="AD9361's status pins.")
        ])
        self._bitmode = CSRStorage(fields=[
            CSRField("mode", size=2, offset=0, values=[
                ("``0b00``", "12-bit mode in SC16/Q11 transport containers."),
                ("``0b01``", " 8-bit mode in SC8/Q7 transport containers."),
                ("``0b10``", "BFP8 block-floating transport mode."),
            ], description="Sample format.")
        ])
        # Last stream settings programmed by host software.  These scratch
        # CSRs let an independent SATA control process describe and size the
        # stream already configured by GQRX without touching the RFIC.
        self._active_sample_rate = CSRStorage(32,
            description="Active host stream sample rate in samples/s.")
        self._active_rx_frequency_khz = CSRStorage(32,
            description="Active RX center frequency in kHz.")
        self._active_bandwidth = CSRStorage(32,
            description="Active RX bandwidth in Hz.")

        # # #

        # Clocking ---------------------------------------------------------------------------------
        self.cd_rfic = ClockDomain("rfic")

        # SPI --------------------------------------------------------------------------------------
        self.spi = AD9361SPIMaster(spi_pads, data_width=24, clk_divider=8)

        # Config / Status --------------------------------------------------------------------------
        self.sync += [
            # AD9361 Control.
            rfic_pads.rst_n.eq(self._config.fields.rst_n),
            rfic_pads.enable.eq(self._config.fields.enable),
            rfic_pads.txnrx.eq(self._config.fields.txnrx),
            rfic_pads.en_agc.eq(self._config.fields.en_agc),

            # AD9361 Control/Status IOs.
            rfic_pads.ctrl.eq(self._ctrl.storage),
            self._stat.fields.stat.eq(rfic_pads.stat),
        ]

        # PHY --------------------------------------------------------------------------------------
        self.phy = AD9361PHY(rfic_pads)

        # TX/RX UnPacker/Packer (GPIOs) ------------------------------------------------------------

        self.gpio_tx_unpacker = gpio_tx_unpacker = GPIOTXUnpacker()
        self.gpio_rx_packer   = gpio_rx_packer   = GPIORXPacker()

        # Cross domain crossing --------------------------------------------------------------------
        self.tx_cdc = tx_cdc = stream.ClockDomainCrossing(
            layout  = dma_layout(64),
            cd_from = "sys",
            cd_to   = "rfic",
            with_common_rst = True
        )
        self.rx_cdc = rx_cdc = stream.ClockDomainCrossing(
            layout  = rx_tick_layout(), # Each word crosses with the tick it was sampled at.
            cd_from = "rfic",
            cd_to   = "sys",
            with_common_rst = True
        )

        # Buffers (For Timings) --------------------------------------------------------------------
        self.tx_buffer = tx_buffer = stream.Buffer(dma_layout(64))
        self.rx_buffer = rx_buffer = stream.Buffer(rx_tick_layout())  # Data and its tick together.
        # Externally forced "started" (sys domain, quasi-static): the timed-TX gate sets it so the
        # FIFO never adds priming hysteresis between a release and the first emitted sample.
        self.tx_force_started = Signal()
        tx_force_started_rfic = Signal()
        self.specials += MultiReg(self.tx_force_started, tx_force_started_rfic, odomain="rfic")
        self.tx_rfic_fifo_started = tx_rfic_fifo_started = Signal()
        tx_rfic_fifo_started_hyst = Signal()
        if with_tx_fifo:
            self.tx_rfic_fifo = tx_rfic_fifo = ClockDomainsRenamer("rfic")(
                stream.SyncFIFO(dma_layout(64), depth=tx_fifo_depth, buffered=True)
            )
            tx_fifo_start_level = max(1, tx_fifo_depth//2)
            tx_rfic_fifo_primed = Signal()
            self.comb += tx_rfic_fifo_primed.eq(tx_rfic_fifo.level >= tx_fifo_start_level)
            self.sync.rfic += [
                If(tx_rfic_fifo_primed,
                    tx_rfic_fifo_started_hyst.eq(1)
                ).Elif(tx_rfic_fifo.level == 0,
                    tx_rfic_fifo_started_hyst.eq(0)
                )
            ]
            self.sync.rfic += tx_rfic_fifo_started.eq(tx_rfic_fifo_started_hyst | tx_force_started_rfic)
        else:
            self.tx_rfic_fifo = tx_rfic_fifo = AD9361RFICStreamBypass()
            self.comb += tx_rfic_fifo_started.eq(1)

        if with_rx_fifo:
            self.rx_rfic_fifo = rx_rfic_fifo = ClockDomainsRenamer("rfic")(
                stream.SyncFIFO(rx_tick_layout(), depth=rx_fifo_depth, buffered=True)
            )
        else:
            self.rx_rfic_fifo = rx_rfic_fifo = AD9361RFICStreamBypass(rx_tick_layout())

        # BitMode ----------------------------------------------------------------------------------
        self.tx_bitmode = tx_bitmode = AD9361TXBitMode()
        self.rx_bitmode = rx_bitmode = AD9361RXBitMode()
        self.comb += tx_bitmode.mode.eq(self._bitmode.fields.mode)
        self.comb += rx_bitmode.mode.eq(self._bitmode.fields.mode)

        # Sample Counter (time base) ---------------------------------------------------------------
        # See gateware/sample_time.py. ``tick`` is the index of the first sample of the PHY word being
        # received, in the rfic domain; it advances on every RX word strobe (consumed or not), by 2 in
        # 1R1T (a word is two consecutive samples) and by 1 in 2R2T.
        self._tick_control = CSRStorage(fields=[
            CSRField("timebase", size=1, offset=0, values=[
                ("``0b0``", "Frame timestamps and gates use time_gen (nanoseconds)."),
                ("``0b1``", "Frame timestamps and gates use the sample counter (ticks)."),
            ]),
            CSRField("load", size=1, offset=1, pulse=True, description="Load tick_write into the counter."),
            CSRField("read", size=1, offset=2, pulse=True, description="Latch the counter into tick_read."),
            CSRField("fine_enable", size=1, offset=3, values=[
                ("``0b0``", "TX words go to the PHY as they arrive."),
                ("``0b1``", "TX words are emitted in the PHY slot of their own tick (TXFineGate)."),
            ]),
        ])
        self._tick_write  = CSRStorage(64, description="Value loaded into the sample counter.")
        self._tick_read   = CSRStatus(64,  description="Sample counter (sys view), latched by control.read.")
        self._tick_status = CSRStatus(fields=[
            CSRField("rx_overflow", size=1,  offset=0,  description="RX tick queue overflowed (non 1:1 sample format)."),
            CSRField("tx_waiting",  size=1,  offset=1,  description="A TX word is waiting for its slot."),
            CSRField("tx_trimmed",  size=16, offset=16, description="TX words discarded by the fine gate (wraps)."),
        ])

        self.tick      = tick     = Signal(64)  # rfic.
        self.tick_inc  = Signal(2)              # sys:  sample periods per PHY word.
        tick_inc_rfic  = Signal(2)              # rfic.
        self.tick_mode = Signal()               # sys:  1 = the time base is the sample counter.
        self.comb += [
            self.tick_inc.eq(Mux(self.phy.control.fields.mode, 2, 1)),
            tick_inc_rfic.eq(Mux(self.phy.mode_rfic, 2, 1)),
            self.tick_mode.eq(self._tick_control.fields.timebase),
        ]
        self.tick_load_ps = tick_load_ps = PulseSynchronizer("sys", "rfic")
        self.comb += tick_load_ps.i.eq(self._tick_control.fields.load)
        self.sync.rfic += [
            If(tick_load_ps.o,
                tick.eq(self._tick_write.storage),  # Static by the time the pulse has crossed.
            ).Elif(self.phy.rx_strobe,
                tick.eq(tick + tick_inc_rfic),
            )
        ]

        # RX: tick tracker (sys), between the clock-domain FIFO and the bit-mode stage.
        self.rx_tick_tracker = rx_tick_tracker = RXTickTracker()
        self.rx_tick     = rx_tick_tracker.tick  # sys: tick of the word offered on ``source``.
        self.rx_tick_now = rx_tick_tracker.now   # sys: view of "now" (tick after the newest word).
        self.comb += [
            rx_tick_tracker.inc.eq(self.tick_inc),
            rx_tick_tracker.exact.eq(self._bitmode.fields.mode == 0b00),
            self._tick_status.fields.rx_overflow.eq(0),  # (kept for the register layout; no queue any more)
        ]
        self.sync += If(self._tick_control.fields.read, self._tick_read.status.eq(rx_tick_tracker.now))

        # TX: frame stamps (sys -> rfic) and the fine gate (rfic), right in front of the PHY.
        self.tx_stamp_cdc = tx_stamp_cdc = stream.ClockDomainCrossing(
            layout  = tx_stamp_layout(),
            cd_from = "sys",
            cd_to   = "rfic",
            depth   = 8,
            with_common_rst = True
        )
        self.tx_stamp    = tx_stamp_cdc.sink                   # sys: one entry per frame (TimedTXGate).
        self.fine_enable = self._tick_control.fields.fine_enable  # sys.
        self.tx_fine = tx_fine = ClockDomainsRenamer("rfic")(TXFineGate())
        tx_trim_gray = Signal(16)
        tx_trim_sys  = Signal(16)
        tx_waiting   = Signal()
        self.tx_trim_dec = tx_trim_dec = GrayDecoder(16)
        self.specials += [
            MultiReg(self._tick_control.fields.fine_enable, tx_fine.enable, odomain="rfic"),
            MultiReg(tx_trim_gray, tx_trim_sys),
            MultiReg(tx_fine.waiting, tx_waiting),
        ]
        self.sync.rfic += tx_trim_gray.eq(tx_fine.trimmed ^ tx_fine.trimmed[1:])
        self.comb += [
            tx_stamp_cdc.source.connect(tx_fine.stamp),
            tx_fine.tick.eq(tick),
            tx_fine.inc.eq(tick_inc_rfic),
            tx_trim_dec.i.eq(tx_trim_sys),
            self._tick_status.fields.tx_trimmed.eq(tx_trim_dec.o),
            self._tick_status.fields.tx_waiting.eq(tx_waiting),
        ]

        # Data Flow --------------------------------------------------------------------------------

        # TX.
        # ---
        # Sink -> TX Buffer -> TX BitMode -> TX CDC -> optional TX RFIC FIFO -> GPIOTXUnpacker -> PHY.
        self.tx_pipeline = stream.Pipeline(
            self.sink,
            tx_buffer,
            tx_bitmode,
            tx_cdc,
            tx_rfic_fifo,
            tx_fine,
            gpio_tx_unpacker,
        )
        self.comb += [
            self.phy.sink.valid.eq(gpio_tx_unpacker.source.valid & tx_rfic_fifo_started),
            gpio_tx_unpacker.source.ready.eq(self.phy.sink.ready & tx_rfic_fifo_started),
            self.phy.sink.ia.eq(gpio_tx_unpacker.source.data[0*16:1*16]),
            self.phy.sink.qa.eq(gpio_tx_unpacker.source.data[1*16:2*16]),
            self.phy.sink.ib.eq(gpio_tx_unpacker.source.data[2*16:3*16]),
            self.phy.sink.qb.eq(gpio_tx_unpacker.source.data[3*16:4*16]),
        ]

        # RX.
        # ---
        # PHY -> GPIORXPacker (+tick) -> optional RX RFIC FIFO -> RX CDC -> RX Buffer -> tick tracker -> RX BitMode -> Source.
        rx_tagged = stream.Endpoint(rx_tick_layout())  # rfic: packed word + its tick.
        self.comb += [
            gpio_rx_packer.source.connect(rx_tagged),
            rx_tagged.tick.eq(tick),                   # The strobe cycle: tick is this word's index.
        ]
        self.comb += [
            self.phy.source.connect(gpio_rx_packer.sink, keep={"valid", "ready"}),
            gpio_rx_packer.sink.data[0*16:1*16].eq(_sign_extend(self.phy.source.ia, 16)),
            gpio_rx_packer.sink.data[1*16:2*16].eq(_sign_extend(self.phy.source.qa, 16)),
            gpio_rx_packer.sink.data[2*16:3*16].eq(_sign_extend(self.phy.source.ib, 16)),
            gpio_rx_packer.sink.data[3*16:4*16].eq(_sign_extend(self.phy.source.qb, 16)),
        ]
        self.rx_pipeline = stream.Pipeline(
            rx_tagged,
            rx_rfic_fifo,
            rx_cdc,
            rx_buffer,
            rx_tick_tracker,
            rx_bitmode,
            self.source,
        )

    def add_prbs(self):
        self.prbs_tx = CSRStorage(fields=[
            CSRField("enable", size=1, offset= 0, values=[
                ("``0b0``", "Disable PRBS TX."),
                ("``0b1``", "Enable  PRBS TX."),
            ])])
        self.prbs_rx = CSRStatus(fields=[
            CSRField("synced", size=1, offset= 0, values=[
                ("``0b0``", "PRBS RX Out-of-Sync."),
                ("``0b1``", "PRBS_RX Synchronized."),
            ])])

        # # #

        phy = self.phy
        mode_rfic = Signal()
        prbs_tx_enable = Signal()
        self.specials += [
            MultiReg(phy.control.fields.mode, mode_rfic, odomain="rfic"),
            MultiReg(self.prbs_tx.fields.enable, prbs_tx_enable, odomain="rfic"),
        ]

        # PRBS TX.
        # --------
        prbs_generator_2r2t = AD9361PRBSGenerator()
        prbs_generator_2r2t = ResetInserter()(prbs_generator_2r2t)
        prbs_generator_2r2t = ClockDomainsRenamer("rfic")(prbs_generator_2r2t)
        prbs_generator_1r1t = AD9361PRBS1R1TGenerator()
        prbs_generator_1r1t = ResetInserter()(prbs_generator_1r1t)
        prbs_generator_1r1t = ClockDomainsRenamer("rfic")(prbs_generator_1r1t)
        self.submodules += prbs_generator_2r2t
        self.submodules += prbs_generator_1r1t
        self.comb += [
            prbs_generator_2r2t.reset.eq(~prbs_tx_enable | mode_rfic),
            prbs_generator_1r1t.reset.eq(~prbs_tx_enable | ~mode_rfic),
            prbs_generator_2r2t.ce.eq(phy.sink.ready),
            prbs_generator_1r1t.ce.eq(phy.sink.ready),
            If(prbs_tx_enable,
                phy.sink.valid.eq(1),
                If(mode_rfic,
                    phy.sink.ia.eq(prbs_generator_1r1t.o),
                    phy.sink.qa.eq(prbs_generator_1r1t.o),
                    phy.sink.ib.eq(prbs_generator_1r1t.o_next),
                    phy.sink.qb.eq(prbs_generator_1r1t.o_next),
                ).Else(
                    phy.sink.ia.eq(prbs_generator_2r2t.o),
                    phy.sink.qa.eq(prbs_generator_2r2t.o),
                    phy.sink.ib.eq(prbs_generator_2r2t.o),
                    phy.sink.qb.eq(prbs_generator_2r2t.o),
                )
            )
        ]

        # PRBS RX.
        # --------
        # 2R2T mode: each lane (channel) carries the full PRBS sequence; check
        # ia/ib independently. 1R1T mode: the a/b slots carry two consecutive
        # samples of ONE stream (each lane sees every other PRBS value), so a
        # dedicated interleaved checker is required. Select by PHY mode.

        synced_2r2t = Signal(reset=1)
        self.comb += synced_2r2t.eq(1)
        for data in [phy.source.ia, phy.source.ib]:
            prbs_checker = AD9361PRBSChecker()
            prbs_checker = ClockDomainsRenamer("rfic")(prbs_checker)
            self.submodules += prbs_checker
            self.comb += [
                prbs_checker.i.eq(data),
                prbs_checker.ce.eq(phy.source.valid),
                If(~prbs_checker.synced,
                    synced_2r2t.eq(0)
                ),
            ]

        prbs_checker_1r1t = AD9361PRBS1R1TChecker()
        prbs_checker_1r1t = ClockDomainsRenamer("rfic")(prbs_checker_1r1t)
        self.submodules += prbs_checker_1r1t
        self.comb += [
            prbs_checker_1r1t.ia.eq(phy.source.ia),
            prbs_checker_1r1t.ib.eq(phy.source.ib),
            prbs_checker_1r1t.ce.eq(phy.source.valid),
        ]

        synced = Signal()
        self.comb += synced.eq(Mux(mode_rfic, prbs_checker_1r1t.synced, synced_2r2t))
        self.specials += MultiReg(synced, self.prbs_rx.fields.synced)

    def add_agc(self):
        rx_cdc = self.rx_cdc
        self.agc_count_rx1_low = AGCSaturationCount(
            ce  = rx_cdc.source.valid & rx_cdc.source.ready,
            iqs = [rx_cdc.source.data[0*16:1*16], rx_cdc.source.data[1*16:2*16]],
            threshold_reset = AGC_DEFAULT_LOW_THRESHOLD,
        )
        self.agc_count_rx1_high = AGCSaturationCount(
            ce  = rx_cdc.source.valid & rx_cdc.source.ready,
            iqs = [rx_cdc.source.data[0*16:1*16], rx_cdc.source.data[1*16:2*16]],
            threshold_reset = AGC_DEFAULT_HIGH_THRESHOLD,
        )
        self.agc_count_rx2_low = AGCSaturationCount(
            ce  = rx_cdc.source.valid & rx_cdc.source.ready,
            iqs = [rx_cdc.source.data[2*16:3*16], rx_cdc.source.data[3*16:4*16]],
            threshold_reset = AGC_DEFAULT_LOW_THRESHOLD,
        )
        self.agc_count_rx2_high = AGCSaturationCount(
            ce  = rx_cdc.source.valid & rx_cdc.source.ready,
            iqs = [rx_cdc.source.data[2*16:3*16], rx_cdc.source.data[3*16:4*16]],
            threshold_reset = AGC_DEFAULT_HIGH_THRESHOLD,
        )
