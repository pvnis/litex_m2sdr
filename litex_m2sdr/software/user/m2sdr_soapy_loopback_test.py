#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
#
# M2SDR SoapySDR Basic Loopback Test
#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
#
# Validates basic SoapySDR TX→RX path using the Soapy PHY loopback mode.
# No timestamps, no timed TX — just a plain continuous tone.
#
# Prerequisites:
#   ./m2sdr_rf -samplerate 30.72e6
#   (loopback is enabled via the SoapySDR 'loopback_mode=phy' kwarg below)
#
# Usage:
#   python3 m2sdr_soapy_loopback_test.py [--samplerate 30720000]
#                                         [--freq-hz 500000] [--amplitude 0.3]

import sys
import time
import argparse
import threading
import numpy as np

try:
    import SoapySDR
    from SoapySDR import SOAPY_SDR_TX, SOAPY_SDR_RX, SOAPY_SDR_CF32
except ImportError:
    print("ERROR: SoapySDR Python bindings not found.")
    sys.exit(1)


def make_tone(n_samples, freq_hz, sample_rate, amplitude):
    t = np.arange(n_samples, dtype=np.float64)
    return (amplitude * np.exp(1j * 2 * np.pi * freq_hz / sample_rate * t)).astype(np.complex64)


def run_test(sample_rate, freq_hz, amplitude, record_s=0.5):
    """
    Transmit a continuous tone for record_s seconds and capture loopback RX.
    Returns True if the received power is above a reasonable floor.
    """
    # ------------------------------------------------------------------
    # Open device with loopback enabled
    # ------------------------------------------------------------------
    print("Opening SoapySDR device with loopback_mode=phy ...")
    dev = SoapySDR.Device({"driver": "LiteXM2SDR", "loopback_mode": "phy"})
    dev.setSampleRate(SOAPY_SDR_TX, 0, sample_rate)
    dev.setSampleRate(SOAPY_SDR_RX, 0, sample_rate)

    # ------------------------------------------------------------------
    # Set up streams
    # ------------------------------------------------------------------
    tx_stream = dev.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32)
    rx_stream = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    dev.activateStream(rx_stream)
    dev.activateStream(tx_stream)

    # ------------------------------------------------------------------
    # Record RX in a background thread while continuously transmitting
    # ------------------------------------------------------------------
    n_record   = int(record_s * sample_rate)
    rx_buf     = np.zeros(n_record, dtype=np.complex64)
    rx_done    = threading.Event()
    rx_received = [0]

    def rx_worker():
        chunk_size = 4096
        while rx_received[0] < n_record:
            chunk = np.zeros(min(chunk_size, n_record - rx_received[0]), dtype=np.complex64)
            sr = dev.readStream(rx_stream, [chunk], len(chunk), timeoutUs=1_000_000)
            if sr.ret > 0:
                rx_buf[rx_received[0]:rx_received[0] + sr.ret] = chunk[:sr.ret]
                rx_received[0] += sr.ret
            elif sr.ret < 0:
                print(f"WARNING: readStream error {sr.ret}")
        rx_done.set()

    rx_thread = threading.Thread(target=rx_worker, daemon=True)
    rx_thread.start()

    # ------------------------------------------------------------------
    # Transmit tone continuously while RX is recording
    # ------------------------------------------------------------------
    chunk_size = 1020
    tone_chunk = make_tone(chunk_size, freq_hz, sample_rate, amplitude)
    n_tx       = int(record_s * sample_rate)
    sent       = 0
    print(f"Transmitting {freq_hz/1e3:.1f} kHz tone for {record_s:.2f} s ...")
    t_start = time.monotonic()
    while sent < n_tx:
        sr = dev.writeStream(tx_stream, [tone_chunk], len(tone_chunk), 0, timeoutUs=1_000_000)
        if sr.ret < 0:
            print(f"WARNING: writeStream error {sr.ret}")
        else:
            sent += sr.ret

    # ------------------------------------------------------------------
    # Wait for recording to finish
    # ------------------------------------------------------------------
    rx_done.wait(timeout=record_s + 5.0)
    elapsed = time.monotonic() - t_start
    received = rx_received[0]
    print(f"Recorded {received} samples in {elapsed:.2f} s.")

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    dev.deactivateStream(tx_stream)
    dev.deactivateStream(rx_stream)
    dev.closeStream(tx_stream)
    dev.closeStream(rx_stream)

    # ------------------------------------------------------------------
    # Analysis: skip first 10 ms (startup transient), measure rest
    # ------------------------------------------------------------------
    skip = int(0.010 * sample_rate)
    if received <= skip:
        print("ERROR: not enough samples received.")
        return False

    signal = rx_buf[skip:received]
    power  = float(np.mean(np.abs(signal) ** 2))
    power_db = 10 * np.log10(power + 1e-12)

    print()
    print("=== Results ===")
    print(f"RX power : {power_db:.1f} dBFS  ({received} samples)")

    PASS_THRESHOLD_DB = -40.0   # Any signal clearly above thermal noise
    passed = power_db >= PASS_THRESHOLD_DB
    if passed:
        print(f"PASS: RX power {power_db:.1f} dBFS >= {PASS_THRESHOLD_DB} dBFS")
    else:
        print(f"FAIL: RX power {power_db:.1f} dBFS < {PASS_THRESHOLD_DB} dBFS  (no signal?)")

    return passed


def main():
    parser = argparse.ArgumentParser(description="M2SDR SoapySDR basic loopback test.")
    parser.add_argument("--samplerate", type=float, default=30_720_000, help="Sample rate Hz (default: 30720000)")
    parser.add_argument("--freq-hz",    type=float, default=500_000,    help="Tone frequency Hz (default: 500000)")
    parser.add_argument("--amplitude",  type=float, default=0.3,        help="Tone amplitude 0..1 (default: 0.3)")
    parser.add_argument("--record-s",   type=float, default=0.5,        help="Recording duration s (default: 0.5)")
    args = parser.parse_args()

    print("M2SDR SoapySDR Basic Loopback Test")
    print(f"  Rate      : {args.samplerate/1e6:.3f} MSPS")
    print(f"  Tone      : {args.freq_hz/1e3:.1f} kHz")
    print(f"  Amplitude : {args.amplitude}")
    print(f"  Record    : {args.record_s} s")
    print()

    ok = run_test(
        sample_rate = args.samplerate,
        freq_hz     = args.freq_hz,
        amplitude   = args.amplitude,
        record_s    = args.record_s,
    )
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
