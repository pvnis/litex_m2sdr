#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
#
# BladeRF gNB Monitor
#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
#
# Uses an independent bladeRF 2.0 to monitor and characterize the TX output
# of the M2SDR-based gNB.  Measures received power, carrier frequency offset,
# and optionally records IQ data for offline analysis.
#
# Prerequisites:
#   pip install bladerf numpy scipy
#   sudo bladeRF-cli -p   # verify device is detected
#
# Usage:
#   python3 bladerf_gnb_monitor.py --freq 3600000000 --samplerate 30720000
#   python3 bladerf_gnb_monitor.py --freq 3600000000 --samplerate 61440000 --record-s 2 --output capture.npy
#   python3 bladerf_gnb_monitor.py --freq 3600000000 --samplerate 30720000 --duration 30

import sys
import time
import argparse
import numpy as np

try:
    import bladerf
    from bladerf._bladerf import ChannelLayout, Format, GainMode
except ImportError:
    print("ERROR: bladerf Python module not found.")
    print("       Install with: pip install bladerf")
    sys.exit(1)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def sc16q11_to_cf32(raw: np.ndarray) -> np.ndarray:
    """Convert interleaved SC16_Q11 int16 pairs to complex float32."""
    scale = 1.0 / 2048.0
    iq = raw.view(np.int16).astype(np.float32) * scale
    return iq[0::2] + 1j * iq[1::2]


def measure_power_dbfs(samples: np.ndarray) -> float:
    """RMS power in dBFS (0 dBFS = full-scale)."""
    rms = np.sqrt(np.mean(np.abs(samples) ** 2))
    if rms == 0:
        return -np.inf
    return 20.0 * np.log10(rms)


def measure_peak_dbfs(samples: np.ndarray) -> float:
    peak = np.max(np.abs(samples))
    if peak == 0:
        return -np.inf
    return 20.0 * np.log10(peak)


def estimate_carrier_offset(samples: np.ndarray, sample_rate: float) -> float:
    """
    Estimate dominant carrier offset from DC via FFT peak.
    Returns offset in Hz (positive = above centre freq).
    """
    n = len(samples)
    window = np.blackman(n)
    spectrum = np.abs(np.fft.fftshift(np.fft.fft(samples * window)))
    freqs = np.fft.fftshift(np.fft.fftfreq(n, d=1.0 / sample_rate))
    peak_idx = np.argmax(spectrum)
    return float(freqs[peak_idx])


def print_spectrum_bar(samples: np.ndarray, sample_rate: float, centre_hz: float,
                       bins: int = 64, height: int = 8):
    """Print a simple ASCII power spectrum with centre frequency and span labels."""
    n = len(samples)
    window = np.blackman(n)
    spectrum = np.abs(np.fft.fftshift(np.fft.fft(samples * window))) ** 2
    # Downsample to `bins` columns.
    chunk = len(spectrum) // bins
    bar_vals = np.array([np.mean(spectrum[i * chunk:(i + 1) * chunk]) for i in range(bins)])
    db_vals = 10.0 * np.log10(bar_vals + 1e-20)
    db_min, db_max = db_vals.min(), db_vals.max()
    db_range = max(db_max - db_min, 1.0)

    bw_mhz    = sample_rate / 1e6
    lo_mhz    = (centre_hz - sample_rate / 2) / 1e6
    hi_mhz    = (centre_hz + sample_rate / 2) / 1e6
    ctr_mhz   = centre_hz / 1e6
    lo_label  = f"{lo_mhz:.3f}M"
    ctr_label = f"{ctr_mhz:.3f}M"
    hi_label  = f"{hi_mhz:.3f}M"

    ylabel_w = 8  # width of the dB label column, e.g. "-123.4 "
    print(f"  Spectrum  centre={ctr_mhz:.3f} MHz  span={bw_mhz:.2f} MHz")
    for row in range(height, 0, -1):
        # threshold for this row, linearly mapped from db_max (top) to db_min (bottom)
        threshold = db_min + (row / height) * db_range
        db_label = f"{threshold:+.1f}"
        line = "".join("█" if v >= threshold else " " for v in db_vals)
        print(f"  {db_label:>{ylabel_w}}|{line}|")
    # bottom axis
    spacer = " " * (ylabel_w + 2)
    print(f"  {spacer}+" + "-" * bins + "+")
    # Three-point frequency axis: lo / centre / hi
    half = bins // 2
    axis = f"{lo_label:<{half}}{ctr_label:^{bins - 2*half}}{hi_label:>{half}}"
    print(f"  {spacer} {axis}")


# ---------------------------------------------------------------------------
# BladeRF capture
# ---------------------------------------------------------------------------

class BladeRFCapture:
    NUM_BUFFERS    = 16
    BUFFER_SAMPLES = 8192
    NUM_TRANSFERS  = 8
    TIMEOUT_MS     = 5000

    def __init__(self, freq_hz: float, sample_rate: float, bandwidth: float, gain: int,
                 serial: str = ""):
        ident = f"*:serial={serial}" if serial else ""
        self.dev = bladerf.BladeRF(ident)
        self.ch  = bladerf.CHANNEL_RX(0)
        self.sample_rate = sample_rate

        rx = self.dev.Channel(self.ch)
        rx.frequency  = int(freq_hz)
        rx.sample_rate = int(sample_rate)
        rx.bandwidth  = int(bandwidth)
        rx.gain_mode  = GainMode.Manual
        rx.gain       = gain

        self.dev.sync_config(
            layout        = ChannelLayout.RX_X1,
            fmt           = Format.SC16_Q11,
            num_buffers   = self.NUM_BUFFERS,
            buffer_size   = self.BUFFER_SAMPLES,
            num_transfers = self.NUM_TRANSFERS,
            stream_timeout= self.TIMEOUT_MS,
        )
        self.dev.enable_module(self.ch, True)
        print(f"[BladeRF] freq={rx.frequency/1e6:.3f} MHz  "
              f"rate={rx.sample_rate/1e6:.3f} MSPS  "
              f"bw={rx.bandwidth/1e6:.3f} MHz  gain={rx.gain} dB")

    def capture(self, n_samples: int) -> np.ndarray:
        """Capture n_samples, return as complex64 array."""
        raw = np.zeros(n_samples * 2, dtype=np.int16)
        self.dev.sync_rx(raw, n_samples, timeout_ms=self.TIMEOUT_MS)
        return sc16q11_to_cf32(raw)

    def close(self):
        self.dev.enable_module(self.ch, False)
        self.dev.close()


# ---------------------------------------------------------------------------
# Monitor loop
# ---------------------------------------------------------------------------

def run_monitor(args):
    bandwidth = args.bandwidth if args.bandwidth else args.samplerate * 0.8

    cap = BladeRFCapture(
        freq_hz     = args.freq,
        sample_rate = args.samplerate,
        bandwidth   = bandwidth,
        gain        = args.gain,
        serial      = args.serial,
    )

    snap_samples = int(args.samplerate * args.snap_ms / 1000.0)
    # Round up to multiple of BladeRFCapture.BUFFER_SAMPLES.
    snap_samples = max(snap_samples, BladeRFCapture.BUFFER_SAMPLES)

    record_samples = int(args.samplerate * args.record_s) if args.record_s > 0 else 0
    recorded       = []

    t_start   = time.monotonic()
    iteration = 0

    print(f"\n[Monitor] centre={args.freq/1e6:.3f} MHz  snap={snap_samples} samples"
          f"  interval={args.interval:.1f}s\n")

    try:
        while True:
            elapsed = time.monotonic() - t_start
            if args.duration > 0 and elapsed > args.duration:
                print("[Monitor] Duration reached, stopping.")
                break

            samples = cap.capture(snap_samples)
            power_dbfs = measure_power_dbfs(samples)
            peak_dbfs  = measure_peak_dbfs(samples)
            cfo_hz     = estimate_carrier_offset(samples, args.samplerate)

            carrier_mhz = (args.freq + cfo_hz) / 1e6
            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] iter={iteration:4d}  "
                  f"rms={power_dbfs:+6.1f} dBFS  "
                  f"peak={peak_dbfs:+6.1f} dBFS  "
                  f"CFO={cfo_hz/1e3:+8.2f} kHz  "
                  f"carrier={carrier_mhz:.3f} MHz")

            if args.spectrum and (iteration % args.spectrum_every == 0):
                print_spectrum_bar(samples, args.samplerate, args.freq)

            if record_samples > 0 and len(recorded) * snap_samples < record_samples:
                recorded.append(samples.copy())
                if len(recorded) * snap_samples >= record_samples:
                    all_samples = np.concatenate(recorded)[:record_samples]
                    outfile = args.output if args.output else "bladerf_capture.npy"
                    np.save(outfile, all_samples)
                    print(f"[Monitor] Saved {len(all_samples)} samples → {outfile}")
                    record_samples = 0  # don't save again

            iteration += 1
            if args.interval > 0:
                time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\n[Monitor] Interrupted.")
    finally:
        cap.close()

    if recorded and args.output:
        # Save whatever was collected if record target not yet reached.
        all_samples = np.concatenate(recorded)
        outfile = args.output
        np.save(outfile, all_samples)
        print(f"[Monitor] Saved {len(all_samples)} samples → {outfile}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Monitor gNB TX with an independent bladeRF 2.0")

    parser.add_argument("--freq",           type=float, default=3409e6,
                        help="Centre frequency in Hz (default: 3.409 GHz)")
    parser.add_argument("--samplerate",     type=float, default=30.72e6,
                        help="Sample rate in Hz (default: 30.72 MSPS)")
    parser.add_argument("--bandwidth",      type=float, default=0,
                        help="RF bandwidth in Hz (0 = 80%% of sample rate)")
    parser.add_argument("--gain",           type=int,   default=30,
                        help="RX gain in dB (default: 30)")
    parser.add_argument("--serial",         type=str,   default="",
                        help="bladeRF serial (leave empty for first device)")
    parser.add_argument("--snap-ms",        type=float, default=10.0,
                        help="Snapshot length in ms per measurement (default: 10)")
    parser.add_argument("--interval",       type=float, default=1.0,
                        help="Seconds between snapshots (default: 1.0; 0=continuous)")
    parser.add_argument("--duration",       type=float, default=0,
                        help="Total run time in seconds (0=infinite)")
    parser.add_argument("--record-s",       type=float, default=0,
                        help="Seconds of IQ data to record (0=none)")
    parser.add_argument("--output",         type=str,   default="",
                        help="Output .npy file for recorded IQ")
    parser.add_argument("--spectrum",       action="store_true",
                        help="Print ASCII spectrum each snapshot")
    parser.add_argument("--spectrum-every", type=int,   default=1,
                        help="Print spectrum every N iterations (default: 1)")

    args = parser.parse_args()
    run_monitor(args)


if __name__ == "__main__":
    main()
