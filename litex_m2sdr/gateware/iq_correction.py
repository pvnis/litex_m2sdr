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

# IQ Correction ------------------------------------------------------------------------------------
#
# 2×2 real matrix correction per RX channel to compensate IQ amplitude and phase imbalance.
# Applied independently to channel A and channel B.
#
# Math (Q2.14 fixed-point coefficients, default = identity):
#   I_out = (a * I_in + b * Q_in) >> 14
#   Q_out = (c * I_in + d * Q_in) >> 14
#
# Data layout (64-bit word):
#   bits [ 0:16] = I_A   bits [16:32] = Q_A
#   bits [32:48] = I_B   bits [48:64] = Q_B
#
# When disabled (enable=0), the module is a transparent pass-through.
# Corrections are applied combinationally (1-cycle latency through the pipeline).

class IQCorrection(LiteXModule):
    def __init__(self, data_width=64):
        self.sink   = sink   = stream.Endpoint(dma_layout(data_width))
        self.source = source = stream.Endpoint(dma_layout(data_width))

        # CSRs.
        self._enable = CSRStorage(1, reset=0,
            description="IQ correction enable: 1=apply matrix, 0=pass-through.")

        # Channel A coefficients (Q2.14 fixed-point; 16384 = 1.0).
        self._ch_a_a = CSRStorage(18, reset=16384, description="Ch-A: I_out = a*I + b*Q; a (Q2.14).")
        self._ch_a_b = CSRStorage(18, reset=0,     description="Ch-A: b coefficient (Q2.14).")
        self._ch_a_c = CSRStorage(18, reset=0,     description="Ch-A: Q_out = c*I + d*Q; c (Q2.14).")
        self._ch_a_d = CSRStorage(18, reset=16384, description="Ch-A: d coefficient (Q2.14).")

        # Channel B coefficients.
        self._ch_b_a = CSRStorage(18, reset=16384, description="Ch-B: a coefficient (Q2.14).")
        self._ch_b_b = CSRStorage(18, reset=0,     description="Ch-B: b coefficient (Q2.14).")
        self._ch_b_c = CSRStorage(18, reset=0,     description="Ch-B: c coefficient (Q2.14).")
        self._ch_b_d = CSRStorage(18, reset=16384, description="Ch-B: d coefficient (Q2.14).")

        # # #

        self.comb += sink.connect(source, omit={"data"})

        def apply_matrix(i_in, q_in, a, b, c, d):
            """1-stage pipelined 2×2 Q2.14 matrix product, clamped to 16 bits.

            Stage 1 (sync): register the four individual DSP multiplications.
              Critical path: DSP multiply (~4 ns) → register.
            Stage 2 (comb): sum pairs, shift >> 14, clamp.
              Critical path: add (~2 ns) + shift (~1 ns) + clamp (~2 ns).
            """
            # Stage 1: register individual products (each maps to one DSP48).
            # reset_less=True: intermediate pipeline registers; startup garbage is
            # harmless and avoids growing the high-fanout system reset tree.
            ai_r = Signal((35, True), reset_less=True)
            bq_r = Signal((35, True), reset_less=True)
            ci_r = Signal((35, True), reset_less=True)
            dq_r = Signal((35, True), reset_less=True)
            self.sync += [
                ai_r.eq(a * i_in),
                bq_r.eq(b * q_in),
                ci_r.eq(c * i_in),
                dq_r.eq(d * q_in),
            ]
            # Stage 2 (comb): add, scale, clamp.
            # Clamp 36-bit signed result to 16-bit signed range.
            #
            # Bug fixed: the original `x < -32768` emits `x < -16'd32768` in Verilog.
            # -16'd32768 is an *unsigned* literal (0x8000 = 32768), so the comparison
            # becomes `unsigned(x) < 32768` — TRUE for every positive x in [0,32767],
            # clamping correct positive outputs to -32768 (same bug as DCFilter).
            #
            # Fix: use bit-slice conditions (all unsigned) instead of signed constants.
            # Negative overflow (x < -32768) iff sign bit (x[35]) is 1 AND bits[34:15]
            # are NOT all 1s (if they were all 1s the value would be in [-32768, -1]).
            # Upper bound: x > 32767 uses Migen's $signed({0,15'd32767}) which is correct.
            i_out_wide = Signal((36, True))
            q_out_wide = Signal((36, True))
            i_out      = Signal((16, True))
            q_out      = Signal((16, True))
            _NEG_UPPER = (1 << 20) - 1  # 0xFFFFF: all-ones for bits[34:15] (20 bits)
            self.comb += [
                i_out_wide.eq((ai_r + bq_r) >> 14),
                q_out_wide.eq((ci_r + dq_r) >> 14),
                If(i_out_wide > 32767,
                    i_out.eq(32767),
                ).Elif(i_out_wide[35] & (i_out_wide[15:35] != _NEG_UPPER),
                    i_out.eq(-32768),
                ).Else(
                    i_out.eq(i_out_wide[:16]),
                ),
                If(q_out_wide > 32767,
                    q_out.eq(32767),
                ).Elif(q_out_wide[35] & (q_out_wide[15:35] != _NEG_UPPER),
                    q_out.eq(-32768),
                ).Else(
                    q_out.eq(q_out_wide[:16]),
                ),
            ]
            return i_out, q_out

        # Extract signed 16-bit samples.
        ia_in = Signal((16, True))
        qa_in = Signal((16, True))
        ib_in = Signal((16, True))
        qb_in = Signal((16, True))
        self.comb += [
            ia_in.eq(sink.data[ 0:16]),
            qa_in.eq(sink.data[16:32]),
            ib_in.eq(sink.data[32:48]),
            qb_in.eq(sink.data[48:64]),
        ]

        # Coefficient signals (signed 18-bit → cast for multiply).
        a_a = Signal((18, True))
        b_a = Signal((18, True))
        c_a = Signal((18, True))
        d_a = Signal((18, True))
        a_b = Signal((18, True))
        b_b = Signal((18, True))
        c_b = Signal((18, True))
        d_b = Signal((18, True))
        self.comb += [
            a_a.eq(self._ch_a_a.storage),
            b_a.eq(self._ch_a_b.storage),
            c_a.eq(self._ch_a_c.storage),
            d_a.eq(self._ch_a_d.storage),
            a_b.eq(self._ch_b_a.storage),
            b_b.eq(self._ch_b_b.storage),
            c_b.eq(self._ch_b_c.storage),
            d_b.eq(self._ch_b_d.storage),
        ]

        # Apply correction matrices.
        ia_out, qa_out = apply_matrix(ia_in, qa_in, a_a, b_a, c_a, d_a)
        ib_out, qb_out = apply_matrix(ib_in, qb_in, a_b, b_b, c_b, d_b)

        # Mux corrected vs. passthrough.
        self.comb += If(self._enable.storage,
            source.data[ 0:16].eq(ia_out),
            source.data[16:32].eq(qa_out),
            source.data[32:48].eq(ib_out),
            source.data[48:64].eq(qb_out),
        ).Else(
            source.data.eq(sink.data),
        )
