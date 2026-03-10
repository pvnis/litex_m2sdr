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

# DC Filter ----------------------------------------------------------------------------------------
#
# First-order IIR DC blocker for RX IQ data.  Applied independently to each of the four
# 16-bit components (I_A, Q_A, I_B, Q_B) packed in the 64-bit sample word.
#
# Algorithm (per component):
#   acc[n] = acc[n-1] + x[n] - (acc[n-1] >> alpha_shift)   # leaky integrator
#   y[n]   = x[n] - (acc[n] >> alpha_shift)                 # subtract DC estimate
#
# Corner frequency ≈ Fs / (2π × 2^alpha_shift).
# At Fs=30.72 MSPS, alpha_shift=15 → f_c ≈ 0.15 Hz (effectively DC-only removal).
#
# Data layout (64-bit word, same as AD9361 16-bit mode):
#   bits [ 0:16] = I_A  (channel A, I component, signed 16-bit)
#   bits [16:32] = Q_A  (channel A, Q component, signed 16-bit)
#   bits [32:48] = I_B  (channel B, I component, signed 16-bit)
#   bits [48:64] = Q_B  (channel B, Q component, signed 16-bit)
#
# When disabled (enable=0), the module is a transparent pass-through.

class DCFilter(LiteXModule):
    def __init__(self, data_width=64):
        self.sink   = sink   = stream.Endpoint(dma_layout(data_width))
        self.source = source = stream.Endpoint(dma_layout(data_width))

        # CSRs.
        self._enable      = CSRStorage(1, reset=0,
            description="DC filter enable: 1=apply filter, 0=pass-through.")
        self._alpha_shift = CSRStorage(5, reset=15,
            description="Filter aggressiveness: corner_freq ≈ Fs / (2π × 2^N). "
                        "Default 15 → ~0.15 Hz at 30.72 MSPS.")

        # # #

        # Pass valid/ready/first/last straight through; only data is modified.
        self.comb += [
            sink.connect(source, omit={"data"}),
        ]

        acc_bits = data_width // 4 + 16  # 32 bits: enough for 16-bit input + alpha_shift=15
        acc_ia = Signal((acc_bits, True))
        acc_qa = Signal((acc_bits, True))
        acc_ib = Signal((acc_bits, True))
        acc_qb = Signal((acc_bits, True))

        alpha = self._alpha_shift.storage

        # Registered DC estimates (acc >> alpha_shift), updated each sample.
        # Registering the barrel shift breaks the acc→shift→acc feedback timing loop
        # (variable 5-bit shift of a 32-bit value is ~5 ns; combined with the adder
        # chain in the accumulator update it would exceed the 8 ns clock period).
        # Using a 1-cycle-delayed DC estimate has no perceptible effect: the DC
        # corner frequency is <<1 Hz, so a single-sample lag is negligible.
        dc_ia = Signal((16, True))
        dc_qa = Signal((16, True))
        dc_ib = Signal((16, True))
        dc_qb = Signal((16, True))

        # Extract input samples (signed 16-bit).
        x_ia = Signal((16, True))
        x_qa = Signal((16, True))
        x_ib = Signal((16, True))
        x_qb = Signal((16, True))
        self.comb += [
            x_ia.eq(sink.data[ 0:16]),
            x_qa.eq(sink.data[16:32]),
            x_ib.eq(sink.data[32:48]),
            x_qb.eq(sink.data[48:64]),
        ]

        # Update accumulators and register DC estimates on each valid sample handshake.
        # dc_X at time n = acc_X[n-1] >> alpha  (1-cycle registered, breaks feedback loop).
        self.sync += If(sink.valid & sink.ready,
            dc_ia.eq(acc_ia >> alpha),
            dc_qa.eq(acc_qa >> alpha),
            dc_ib.eq(acc_ib >> alpha),
            dc_qb.eq(acc_qb >> alpha),
            acc_ia.eq(acc_ia + x_ia - dc_ia),
            acc_qa.eq(acc_qa + x_qa - dc_qa),
            acc_ib.eq(acc_ib + x_ib - dc_ib),
            acc_qb.eq(acc_qb + x_qb - dc_qb),
        )

        # Output: x[n] - dc[n], clamped to [-32768, 32767].
        # dc[n] is the registered estimate from the previous cycle (see above).
        y_ia = Signal((17, True))
        y_qa = Signal((17, True))
        y_ib = Signal((17, True))
        y_qb = Signal((17, True))
        self.comb += [
            y_ia.eq(x_ia - dc_ia),
            y_qa.eq(x_qa - dc_qa),
            y_ib.eq(x_ib - dc_ib),
            y_qb.eq(x_qb - dc_qb),
        ]

        def clamp16(y):
            """Clamp a 17-bit signed value to signed 16-bit range."""
            clamped = Signal((16, True))
            self.comb += If(y > 32767,
                clamped.eq(32767),
            ).Elif(y < -32768,
                clamped.eq(-32768),
            ).Else(
                clamped.eq(y),
            )
            return clamped

        c_ia = clamp16(y_ia)
        c_qa = clamp16(y_qa)
        c_ib = clamp16(y_ib)
        c_qb = clamp16(y_qb)

        # Route data through filter or bypass based on enable.
        self.comb += If(self._enable.storage,
            source.data[ 0:16].eq(c_ia),
            source.data[16:32].eq(c_qa),
            source.data[32:48].eq(c_ib),
            source.data[48:64].eq(c_qb),
        ).Else(
            source.data.eq(sink.data),
        )
