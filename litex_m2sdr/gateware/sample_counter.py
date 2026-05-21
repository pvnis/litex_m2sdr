#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
Sample Counter
==============

Free-running 64-bit counter that increments once per AD9361 sample frame
(one IQ tuple). Lives in the RFIC clock domain so that downstream
timed-TX comparisons are sample-accurate without any CDC slop.

Use cases:
    1. Sample-domain time reference for :class:`TimedTXArbiter`. The TX
       gating FSM compares this counter to the per-burst start timestamp
       extracted from the DMA header and only releases samples once the
       counter has caught up.

    2. RX header timestamping. The :class:`RXHeaderInserter` embeds the
       counter value into the 16-byte header that precedes every RX DMA
       frame, so software gets a sample-domain capture time without
       having to extrapolate from nanoseconds.

    3. CSR-visible "what sample am I at?" snapshot. Software pulses
       ``control.read_latch`` and reads ``read_value`` to get an atomic
       64-bit snapshot. ``control.write`` + ``write_value`` lets software
       seed the counter (e.g. align to PPS).

CDC strategy
------------
``count_sys`` is exposed via Gray-coded continuous CDC: the rfic-domain
binary counter is encoded to Gray code (single-bit transitions per
increment), passed through MultiReg, then decoded back to binary in the
sys domain. This guarantees atomic 64-bit reads in the sys domain at the
cost of ~3 sys cycles of latency vs. the rfic counter. For sample-
accurate uses (RX header timestamping at a 23-30 MSPS sample rate) the
latency translates to under 1 sample of error.
"""

from functools import reduce

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *


def _binary_to_gray(bin_sig):
    return bin_sig ^ (bin_sig >> 1)


def _gray_to_binary(gray_sig, width):
    """Return a list of bit signals such that bin[i] = XOR(gray[i:width])."""
    out = []
    for i in range(width):
        out.append(reduce(lambda a, b: a ^ b, [gray_sig[j] for j in range(i, width)]))
    return out


class SampleCounter(LiteXModule):
    """Free-running 64-bit sample counter (rfic clock domain)."""

    def __init__(self, sample_strobe, init=0, with_csr=True):
        # IO --------------------------------------------------------------------------------------

        # `sample_strobe` is an rfic-domain pulse, asserted once per
        # AD9361 sample frame (one IQ tuple).
        self.sample_strobe = sample_strobe

        # Counter visible in the RFIC clock domain. For consumers in the
        # same domain (e.g. TimedTXArbiter) this is the no-CDC reference.
        self.count_rfic = Signal(64)

        # Counter visible in the sys clock domain via continuous
        # Gray-coded CDC. Lags `count_rfic` by ~3 sys cycles but is
        # atomic. Use this for sys-domain consumers like the RX header
        # inserter.
        self.count_sys = Signal(64)

        # # #

        # Counter (rfic domain) -------------------------------------------------------------------
        count_bin = Signal(64, reset=init)

        # Sys -> rfic: software write of the counter value, and software
        # read of a latched snapshot.
        if with_csr:
            self._control = CSRStorage(fields=[
                CSRField("write", offset=0, size=1, pulse=True,
                    description="Pulse to load `write_value` into the counter."),
                CSRField("read_latch", offset=1, size=1, pulse=True,
                    description="Pulse to snapshot the rfic count into `read_value`."),
            ])
            self._write_value = CSRStorage(64, reset=init,
                description="Counter value to load on `control.write`.")
            self._read_value = CSRStatus(64,
                description="Latched counter snapshot (refreshed by `control.read_latch`).")

            write_ps = PulseSynchronizer("sys", "rfic")
            read_ps  = PulseSynchronizer("sys", "rfic")
            self.submodules += write_ps, read_ps
            self.comb += [
                write_ps.i.eq(self._control.fields.write),
                read_ps.i.eq(self._control.fields.read_latch),
            ]
            write_value_rfic = Signal(64)
            self.specials += MultiReg(self._write_value.storage, write_value_rfic, "rfic")
        else:
            write_ps = None
            read_ps  = None
            write_value_rfic = Signal(64)

        # rfic increment / write logic.
        if write_ps is not None:
            self.sync.rfic += [
                If(write_ps.o,
                    count_bin.eq(write_value_rfic),
                ).Elif(self.sample_strobe,
                    count_bin.eq(count_bin + 1),
                )
            ]
        else:
            self.sync.rfic += If(self.sample_strobe, count_bin.eq(count_bin + 1))

        self.comb += self.count_rfic.eq(count_bin)

        # Continuous Gray-coded CDC rfic -> sys --------------------------------------------------
        count_gray = Signal(64)
        self.sync.rfic += count_gray.eq(_binary_to_gray(count_bin))

        gray_sys = Signal(64)
        self.specials += MultiReg(count_gray, gray_sys, "sys")

        # Decode Gray -> binary in sys with one pipeline stage to relieve
        # the 64-input XOR chain at bit 0 (~13 LUT levels combinational).
        gray_sys_r = Signal(64)
        self.sync += gray_sys_r.eq(gray_sys)
        bin_bits = _gray_to_binary(gray_sys_r, 64)
        count_sys_r = Signal(64)
        self.sync += [count_sys_r[i].eq(bin_bits[i]) for i in range(64)]
        self.comb += self.count_sys.eq(count_sys_r)

        # CSR read-latch snapshot ----------------------------------------------------------------
        if read_ps is not None:
            csr_latch_rfic = Signal(64)
            self.sync.rfic += If(read_ps.o, csr_latch_rfic.eq(count_bin))
            csr_latch_sys = Signal(64)
            self.specials += MultiReg(csr_latch_rfic, csr_latch_sys, "sys")
            self.comb += self._read_value.status.eq(csr_latch_sys)
