#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import math

from migen import *

from litex.gen import *

from litepcie.common import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

# CFR FIR Coefficients -----------------------------------------------------------------------------
#
# 32-tap Hamming-windowed sinc low-pass FIR, cutoff at 0.45*Nyquist.
# Computed in Python at import time; constants are embedded in the Migen netlist.
# Q14 fixed-point (16384 = 1.0); sum ≈ 16384 (unity DC gain after normalization).

def _fir_taps(n=32, fc=0.45):
    """Hamming-windowed sinc FIR, normalized to Q14 unity DC gain."""
    center = (n - 1) / 2.0
    raw = []
    for i in range(n):
        x = i - center
        if abs(x) < 1e-9:
            h = 2.0 * fc
        else:
            h = math.sin(2.0 * math.pi * fc * x) / (math.pi * x)
        w = 0.54 - 0.46 * math.cos(2.0 * math.pi * i / (n - 1))
        raw.append(h * w)
    s = sum(raw)
    return [round(t / s * 16384) for t in raw]

# Crest Factor Reduction ---------------------------------------------------------------------------
#
# Clip-and-filter CFR to reduce TX PAPR for NR OFDM waveforms.
# Applied independently to channel A (I_A, Q_A) and channel B (I_B, Q_B).
#
# Pipeline:
#   x[n] ──►  Clip to ±threshold  ──►  error = x - clipped
#                                              │
#   x[n] ──► 16-cycle delay buffer             │
#        │                                     ▼
#        │                          32-tap FIR on error
#        │                                     │
#        └──────────► Subtractor ◄─────────────┘
#                         │
#                         ▼ clamped to ±32767
#
# CSRs:
#   _enable      — enable CFR (0 = pass-through)
#   _threshold   — clipping amplitude threshold (16-bit, Q0.15: 32767 = full scale)
#   _clip_count  — count of samples where a clip was applied (saturating)
#
# Data layout (64-bit word):
#   bits [ 0:16] = I_A  bits [16:32] = Q_A
#   bits [32:48] = I_B  bits [48:64] = Q_B
#
# When disabled (enable=0), the module is a transparent pass-through.

class CrestFactorReduction(LiteXModule):
    def __init__(self, data_width=64, fir_taps=None):
        n_taps = 32
        if fir_taps is None:
            fir_taps = _fir_taps(n_taps)
        n_taps  = len(fir_taps)
        # For a 32-tap symmetric FIR, group delay = (n_taps - 1) // 2 = 15 samples.
        # We delay the input by n_taps // 2 = 16 cycles to align with the FIR output.
        delay   = n_taps // 2

        self.sink   = sink   = stream.Endpoint(dma_layout(data_width))
        self.source = source = stream.Endpoint(dma_layout(data_width))

        # CSRs.
        self._enable    = CSRStorage(1,  reset=0,
            description="CFR enable: 1=apply clip-and-filter, 0=pass-through.")
        self._threshold = CSRStorage(16, reset=32767,
            description="Clipping threshold (16-bit amplitude; 32767 = full scale).")
        self._clip_count = CSRStatus(32,
            description="Number of samples where clipping was applied (saturating).")

        # # #

        # Pass valid/ready/first/last straight through; only data is modified.
        self.comb += sink.connect(source, omit={"data"})

        # Extract signed 16-bit input samples.
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

        # Threshold signal (unsigned).
        threshold = Signal(16)
        self.comb += threshold.eq(self._threshold.storage)

        # Helper: hard rectangular clip to ±threshold and return clip error.
        def clip_channel(i_in, q_in, name):
            """Combinationally clip I and Q to ±threshold; return (err_i, err_q, peak)."""
            thresh_s = Signal((17, True))  # signed headroom
            self.comb += thresh_s.eq(threshold)

            clipped_i = Signal((16, True), name=f"{name}_ci")
            clipped_q = Signal((16, True), name=f"{name}_cq")
            err_i     = Signal((16, True), name=f"{name}_ei")
            err_q     = Signal((16, True), name=f"{name}_eq")
            peak      = Signal(name=f"{name}_peak")

            self.comb += [
                # Clip I.
                If(i_in > thresh_s,
                    clipped_i.eq(threshold),
                ).Elif(i_in < -thresh_s,
                    clipped_i.eq(-threshold),
                ).Else(
                    clipped_i.eq(i_in),
                ),
                # Clip Q.
                If(q_in > thresh_s,
                    clipped_q.eq(threshold),
                ).Elif(q_in < -thresh_s,
                    clipped_q.eq(-threshold),
                ).Else(
                    clipped_q.eq(q_in),
                ),
                # Error = original - clipped.
                err_i.eq(i_in - clipped_i),
                err_q.eq(q_in - clipped_q),
                # Peak detected when either component was clipped.
                peak.eq((i_in > thresh_s) | (i_in < -thresh_s) |
                        (q_in > thresh_s) | (q_in < -thresh_s)),
            ]
            return err_i, err_q, peak

        err_ia, err_qa, peak_a = clip_channel(ia_in, qa_in, "a")
        err_ib, err_qb, peak_b = clip_channel(ib_in, qb_in, "b")

        peak_any = Signal()
        self.comb += peak_any.eq(peak_a | peak_b)

        # Clip counter (saturating).
        clip_cnt = Signal(32)
        self.sync += If(sink.valid & sink.ready & peak_any,
            If(clip_cnt != 0xFFFFFFFF, clip_cnt.eq(clip_cnt + 1))
        )
        self.comb += self._clip_count.status.eq(clip_cnt)

        # -- Input delay buffer --
        # Align original samples with FIR group delay (delay cycles).
        # Indexed 0 (one cycle ago) to delay-1 (delay cycles ago).
        def make_delay(sig_in, depth, name):
            sr = [Signal((16, True), name=f"{name}_{i}") for i in range(depth)]
            self.sync += If(sink.valid & sink.ready,
                sr[0].eq(sig_in),
                *[sr[i].eq(sr[i-1]) for i in range(1, depth)],
            )
            return sr

        ia_dly = make_delay(ia_in, delay, "ia_dly")
        qa_dly = make_delay(qa_in, delay, "qa_dly")
        ib_dly = make_delay(ib_in, delay, "ib_dly")
        qb_dly = make_delay(qb_in, delay, "qb_dly")

        # -- FIR shift registers (hold last n_taps error samples) --
        def make_fir_sr(err_in, depth, name):
            sr = [Signal((16, True), name=f"{name}_{i}") for i in range(depth)]
            self.sync += If(sink.valid & sink.ready,
                sr[0].eq(err_in),
                *[sr[i].eq(sr[i-1]) for i in range(1, depth)],
            )
            return sr

        fir_ia = make_fir_sr(err_ia, n_taps, "fir_ia")
        fir_qa = make_fir_sr(err_qa, n_taps, "fir_qa")
        fir_ib = make_fir_sr(err_ib, n_taps, "fir_ib")
        fir_qb = make_fir_sr(err_qb, n_taps, "fir_qb")

        # -- FIR multiply-accumulate (using coefficient symmetry) --
        # For symmetric FIR: taps[k] == taps[n_taps-1-k].
        # Pair taps from each end to halve the number of multiplications.
        n_half = n_taps // 2

        def fir_mac(sr, name):
            """Combinational FIR using symmetric coefficient pairs. Returns Q14-scaled output."""
            acc = Signal((40, True), name=f"{name}_acc")
            products = []
            for k in range(n_half):
                coeff = fir_taps[k]
                # Symmetric pair sum: sr[k] + sr[n_taps-1-k]
                pair_sum = Signal((17, True), name=f"{name}_ps_{k}")
                self.comb += pair_sum.eq(sr[k] + sr[n_taps - 1 - k])
                prod = Signal((35, True), name=f"{name}_p_{k}")
                self.comb += prod.eq(coeff * pair_sum)
                products.append(prod)
            self.comb += acc.eq(sum(products))
            out = Signal((17, True), name=f"{name}_out")
            self.comb += out.eq(acc >> 14)
            return out

        fir_out_ia = fir_mac(fir_ia, "ia")
        fir_out_qa = fir_mac(fir_qa, "qa")
        fir_out_ib = fir_mac(fir_ib, "ib")
        fir_out_qb = fir_mac(fir_qb, "qb")

        # -- Subtractor with saturation clamp --
        def sub_clamp(delayed, fir_out, name):
            diff = Signal((18, True), name=f"{name}_diff")
            out  = Signal((16, True), name=f"{name}_out")
            self.comb += [
                diff.eq(delayed - fir_out),
                If(diff > 32767,
                    out.eq(32767),
                ).Elif(diff < -32768,
                    out.eq(-32768),
                ).Else(
                    out.eq(diff),
                ),
            ]
            return out

        out_ia = sub_clamp(ia_dly[-1], fir_out_ia, "ia")
        out_qa = sub_clamp(qa_dly[-1], fir_out_qa, "qa")
        out_ib = sub_clamp(ib_dly[-1], fir_out_ib, "ib")
        out_qb = sub_clamp(qb_dly[-1], fir_out_qb, "qb")

        # -- Output mux: CFR or pass-through --
        self.comb += If(self._enable.storage,
            source.data[ 0:16].eq(out_ia),
            source.data[16:32].eq(out_qa),
            source.data[32:48].eq(out_ib),
            source.data[48:64].eq(out_qb),
        ).Else(
            source.data.eq(sink.data),
        )
