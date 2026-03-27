#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
#
# M2SDR Timed TX Test
#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
#
# Tests the TimedTXArbiter by scheduling a tone burst at a future timestamp and
# verifying (via loopback or over the air) that it arrives at the expected time.
#
# Prerequisites:
#   ./m2sdr_rf -samplerate 30.72e6 -timed_tx_enable 1
#
# Usage (loopback):
#   python3 m2sdr_timed_tx_test.py --loopback
#
# Usage (over the air):
#   python3 m2sdr_timed_tx_test.py --rf-freq 2400000000 --tx-gain 0 --rx-gain 30

import sys
import time
import argparse
import threading
import numpy as np

try:
    import SoapySDR
    from SoapySDR import SOAPY_SDR_TX, SOAPY_SDR_RX, SOAPY_SDR_CF32
    from SoapySDR import SOAPY_SDR_HAS_TIME, SOAPY_SDR_END_BURST
except ImportError:
    print("ERROR: SoapySDR Python bindings not found.")
    print("       Install with: pip install SoapySDR  or  apt install python3-soapysdr")
    sys.exit(1)

# Stream status return codes (SoapySDR/Constants.h).  Use getattr so the
# script still works against older SoapySDR builds that don't export them.
_SDR_TIMEOUT    = getattr(SoapySDR, 'SOAPY_SDR_TIMEOUT',    -3)
_SDR_TIME_ERROR = getattr(SoapySDR, 'SOAPY_SDR_TIME_ERROR', -6)
_SDR_UNDERFLOW  = getattr(SoapySDR, 'SOAPY_SDR_UNDERFLOW',  -2)


def measure_loopback_latency(sample_rate, loopback=True, chunk=4096, n_chunks=200):
    """
    Open a fresh device, fire one immediate (no HAS_TIME) tone burst, and
    find when it appears in the RX stream.  Returns round-trip latency in
    seconds (resolution ±chunk/sample_rate), or 0.0 if undetected.

    This baseline is subtracted from timed-TX onset errors so the reported
    error reflects only the TimedTXArbiter accuracy, not the fixed pipeline.
    """
    print("=== Loopback Latency Baseline ===")
    args = {"driver": "LiteXM2SDR"}
    if loopback:
        args["loopback"] = "1"
    dev = SoapySDR.Device(args)
    dev.setSampleRate(SOAPY_SDR_TX, 0, sample_rate)
    dev.setSampleRate(SOAPY_SDR_RX, 0, sample_rate)

    tx_s = dev.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32)
    rx_s = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    dev.activateStream(rx_s)
    dev.activateStream(tx_s)

    skip = 4   # discard startup transients
    # Store raw IQ for all chunks so we can do sub-chunk onset detection.
    raw = [np.zeros(chunk, dtype=np.complex64) for _ in range(n_chunks)]
    pwr = np.zeros(n_chunks, dtype=np.float64)

    ts = [0] * n_chunks  # HW timestamp (ns) of first sample in each chunk

    # Collect skip startup chunks.
    for i in range(skip):
        sr = dev.readStream(rx_s, [raw[i]], chunk, timeoutUs=1_000_000)
        if sr.ret > 0:
            if sr.flags & SOAPY_SDR_HAS_TIME:
                ts[i] = sr.timeNs
            pwr[i] = float(np.mean(np.abs(raw[i][:sr.ret])**2))

    # Note RX position, fire one tone burst immediately (no HAS_TIME), then
    # record HW time right after so we can anchor latency to the clock.
    tx_ref     = skip
    tone       = make_tone(chunk, 500_000, sample_rate, 0.3)
    dev.writeStream(tx_s, [tone], len(tone), SOAPY_SDR_END_BURST, timeoutUs=1_000_000)
    hw_write_ns = int(dev.getHardwareTime())

    # Collect the rest of the RX window.
    for i in range(skip, n_chunks):
        sr = dev.readStream(rx_s, [raw[i]], chunk, timeoutUs=1_000_000)
        if sr.ret > 0:
            if sr.flags & SOAPY_SDR_HAS_TIME:
                ts[i] = sr.timeNs
            pwr[i] = float(np.mean(np.abs(raw[i][:sr.ret])**2))

    dev.deactivateStream(tx_s)
    dev.deactivateStream(rx_s)
    dev.closeStream(tx_s)
    dev.closeStream(rx_s)
    SoapySDR.Device.unmake(dev)

    # Noise floor from pre-burst chunks; coarse onset = first chunk above 10× floor.
    floor     = float(np.mean(pwr[:skip])) if skip else 1e-12
    threshold = 10.0 * floor + 1e-12
    onset_c   = next((i for i in range(tx_ref, n_chunks) if pwr[i] > threshold), None)

    chunk_ms = chunk / sample_rate * 1000.0
    print(f"  Chunk size   : {chunk} samples  ({chunk_ms:.3f} ms/chunk)")
    if onset_c is None:
        print("  WARNING: tone not detected — latency unmeasurable, using 0")
        return 0.0

    # Sub-chunk fine detection: slide a 32-sample window within the onset chunk.
    fine_win  = 32
    fine_thr  = threshold * fine_win   # threshold is mean power per sample
    sq        = np.abs(raw[onset_c])**2
    cs        = np.concatenate([[0.0], np.cumsum(sq)])
    win_pwr   = cs[fine_win:] - cs[:-fine_win]  # sum over fine_win samples
    hits      = np.where(win_pwr > fine_thr)[0]
    fine_j    = int(hits[0]) if len(hits) else 0

    coarse_samp = (onset_c - tx_ref) * chunk + fine_j   # chunk-count estimate

    # Prefer HW-anchored latency if the driver provided a timestamp for the
    # onset chunk.  This eliminates the writeStream-duration bias.
    if ts[onset_c]:
        onset_hw_ns = ts[onset_c] + int(fine_j / sample_rate * 1e9)
        latency_s   = (onset_hw_ns - hw_write_ns) / 1e9
        anchor_str  = "HW-anchored"
    else:
        latency_s   = coarse_samp / sample_rate
        anchor_str  = "chunk-count estimate"

    print(f"  TX at chunk  : {tx_ref}  onset at chunk: {onset_c}  (+{onset_c - tx_ref} chunk(s))")
    print(f"  Fine offset  : +{fine_j} samples within onset chunk")
    print(f"  Loopback latency: {int(latency_s * sample_rate)} samples  "
          f"({latency_s * 1000:.6f} ms)  [{anchor_str}]")
    return latency_s


def make_tone(n_samples, freq_hz, sample_rate, amplitude):
    """Return complex64 tone samples."""
    t = np.arange(n_samples, dtype=np.float64)
    phase = 2 * np.pi * freq_hz / sample_rate * t
    return (amplitude * np.exp(1j * phase)).astype(np.complex64)


def run_test(delay_s, duration_s, freq_hz, amplitude, sample_rate,
             rf_freq=None, tx_gain=None, rx_gain=None, loopback=False,
             record_extra_s=1.0):
    """
    Schedule a tone burst at (now + delay_s), record for (delay_s + duration_s + record_extra_s),
    then verify the tone is absent before the burst and present after.

    The RX recording runs in a background thread that starts BEFORE the TX burst
    is queued.  writeStream blocks for ~delay_s while the FPGA holds the data in
    its TimedTX FIFO; the RX thread captures the pre-burst silence during that
    time so the analysis windows land in the right place.

    Returns True if the test passes.
    """
    # ------------------------------------------------------------------
    # Measure round-trip loopback latency first, on a fresh device open,
    # so we can subtract it from the timed-TX onset error.
    # ------------------------------------------------------------------
    loopback_latency_s = measure_loopback_latency(sample_rate, loopback=loopback)
    print()

    # ------------------------------------------------------------------
    # Open device
    # ------------------------------------------------------------------
    args = {"driver": "LiteXM2SDR"}
    if loopback:
        args["loopback"] = "1"
    dev = SoapySDR.Device(args)
    dev.setSampleRate(SOAPY_SDR_TX, 0, sample_rate)
    dev.setSampleRate(SOAPY_SDR_RX, 0, sample_rate)
    if rf_freq is not None:
        dev.setFrequency(SOAPY_SDR_TX, 0, rf_freq)
        dev.setFrequency(SOAPY_SDR_RX, 0, rf_freq)
    if tx_gain is not None:
        dev.setGain(SOAPY_SDR_TX, 0, tx_gain)
    if rx_gain is not None:
        dev.setGain(SOAPY_SDR_RX, 0, rx_gain)

    # ------------------------------------------------------------------
    # Set up streams
    # ------------------------------------------------------------------
    tx_stream = dev.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32)
    rx_stream = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)

    dev.activateStream(rx_stream)
    dev.activateStream(tx_stream)

    # ------------------------------------------------------------------
    # Build TX burst first — make_tone() for a 500 ms burst at 30.72 MSPS
    # takes ~450 ms of CPU time, which would push the timestamp into the
    # past if we called getHardwareTime() before it.
    # ------------------------------------------------------------------
    n_burst = int(duration_s * sample_rate)
    burst   = make_tone(n_burst, freq_hz, sample_rate, amplitude)
    print(f"Burst         : {n_burst} samples  ({duration_s*1000:.1f} ms)  @ {freq_hz/1e3:.1f} kHz")

    # ------------------------------------------------------------------
    # Start RX recording in a background thread BEFORE writing TX burst.
    #
    # writeStream will block for ~delay_s (the FPGA holds the data in its
    # TimedTX FIFO until the hardware clock reaches tx_time_ns, which
    # back-pressures the DMA ring and causes poll() to wait).  The RX
    # thread runs concurrently during that wait, capturing the pre-burst
    # silence so that analysis windows land in the correct place.
    # ------------------------------------------------------------------
    total_record_s = delay_s + duration_s + record_extra_s
    n_record       = int(total_record_s * sample_rate)
    rx_buf         = np.zeros(n_record, dtype=np.complex64)
    rx_received    = [0]
    rx_time0       = [None]   # HW timestamp (ns) of first received sample
    rx_done        = threading.Event()

    def rx_worker():
        chunk_size = 4096
        while rx_received[0] < n_record:
            chunk = np.zeros(min(chunk_size, n_record - rx_received[0]), dtype=np.complex64)
            sr    = dev.readStream(rx_stream, [chunk], len(chunk), timeoutUs=1_000_000)
            if sr.ret > 0:
                if rx_time0[0] is None and (sr.flags & SOAPY_SDR_HAS_TIME):
                    rx_time0[0] = sr.timeNs
                rx_buf[rx_received[0]:rx_received[0] + sr.ret] = chunk[:sr.ret]
                rx_received[0] += sr.ret
            elif sr.ret < 0:
                print(f"WARNING: readStream error {sr.ret}, continuing ...")
        rx_done.set()

    print(f"Recording     : {n_record} samples  ({total_record_s:.2f} s) ...")
    t_start   = time.monotonic()
    rx_thread = threading.Thread(target=rx_worker, daemon=True)
    rx_thread.start()

    # ------------------------------------------------------------------
    # Read hardware time NOW — burst generation is complete, RX is running,
    # so this timestamp is taken as close as possible to writeStream start.
    # ------------------------------------------------------------------
    hw_time_ns = dev.getHardwareTime()
    tx_time_ns = hw_time_ns + int(delay_s * 1e9)
    print(f"Hardware time : {hw_time_ns / 1e9:.6f} s")
    print(f"TX scheduled  : {tx_time_ns / 1e9:.6f} s  (+{delay_s:.3f} s)")

    # ------------------------------------------------------------------
    # Write burst with timed TX flags (chunked to MTU-sized writes).
    #
    # The timeout must cover the full scheduled delay plus a safety margin
    # because the first writeStream call that finds the DMA ring full will
    # block in poll() until the FPGA starts consuming (timestamp fires).
    # ------------------------------------------------------------------
    MTU = 1020  # samples per writeStream call (fits in one DMA buffer with 2-word header)
    write_timeout_us = int((delay_s + 5.0) * 1e6)
    offset = 0
    first  = True
    while offset < n_burst:
        chunk_end  = min(offset + MTU, n_burst)
        chunk      = burst[offset:chunk_end]
        flags      = SOAPY_SDR_HAS_TIME if first else 0
        at_time    = tx_time_ns         if first else 0
        last_chunk = (chunk_end == n_burst)
        if last_chunk:
            flags |= SOAPY_SDR_END_BURST
        sr = dev.writeStream(tx_stream, [chunk], len(chunk), flags, timeNs=at_time,
                             timeoutUs=write_timeout_us)
        if sr.ret < 0:
            print(f"ERROR: writeStream returned {sr.ret}")
            return False
        offset = chunk_end
        first  = False

    print("Burst written to TX FIFO.")

    # ------------------------------------------------------------------
    # Wait for RX recording to finish
    # ------------------------------------------------------------------
    rx_done.wait(timeout=total_record_s + 10.0)
    elapsed  = time.monotonic() - t_start
    received = rx_received[0]
    print(f"Recorded {received} samples in {elapsed:.2f} s.")

    # ------------------------------------------------------------------
    # Check TX stream status for late / underrun events from the
    # TimedTXArbiter.  Poll a few times with a short timeout; stop as
    # soon as we get TIMEOUT (no more events pending).
    # ------------------------------------------------------------------
    late_count    = 0
    underrun_count = 0
    for _ in range(20):
        sr = dev.readStreamStatus(tx_stream, timeoutUs=50_000)
        if sr.ret == _SDR_TIME_ERROR:
            late_count += 1
        elif sr.ret == _SDR_UNDERFLOW:
            underrun_count += 1
        else:
            break  # TIMEOUT or other — no more events
    if late_count or underrun_count:
        print(f"WARNING: TX status: {late_count} late event(s), {underrun_count} underrun(s)")
    else:
        print("TX status: no late or underrun events.")

    # ------------------------------------------------------------------
    # Cleanup streams
    # ------------------------------------------------------------------
    dev.deactivateStream(tx_stream)
    dev.deactivateStream(rx_stream)
    dev.closeStream(tx_stream)
    dev.closeStream(rx_stream)

    # ------------------------------------------------------------------
    # Analysis: compare power before and after expected TX time
    # ------------------------------------------------------------------
    # "Before" window: middle of the pre-burst silence
    pre_start  = int(delay_s * 0.3 * sample_rate)
    pre_end    = int(delay_s * 0.7 * sample_rate)
    # "During" window: centre of the burst
    burst_start = int((delay_s + duration_s * 0.1) * sample_rate)
    burst_end   = int((delay_s + duration_s * 0.9) * sample_rate)

    pre_power   = float(np.mean(np.abs(rx_buf[pre_start:pre_end])**2))
    burst_power = float(np.mean(np.abs(rx_buf[burst_start:burst_end])**2))

    pre_db    = 10 * np.log10(pre_power   + 1e-12)
    burst_db  = 10 * np.log10(burst_power + 1e-12)
    delta_db  = burst_db - pre_db

    print()
    print("=== Results ===")
    print(f"Pre-burst power  : {pre_db:.1f} dBFS  (samples {pre_start}–{pre_end})")
    print(f"Burst power      : {burst_db:.1f} dBFS  (samples {burst_start}–{burst_end})")
    print(f"Delta            : {delta_db:.1f} dB")

    # Coarse onset: 5 ms sliding RMS to find the approximate burst region.
    window     = int(sample_rate * 0.005)  # 5 ms coarse window
    stride     = window // 2
    rms        = np.array([
        np.sqrt(np.mean(np.abs(rx_buf[i:i+window])**2))
        for i in range(0, received - window, stride)
    ])
    coarse_thr  = 0.5 * float(np.max(rms))
    coarse_idx  = int(np.argmax(rms > coarse_thr))
    coarse_samp = coarse_idx * stride

    # Fine onset: 32-sample sliding window within ±1 coarse window of coarse onset.
    fine_win    = 32
    fine_thr    = pre_power * 1000.0   # 1000× pre-burst noise floor (~30 dB above floor)
    s           = max(0, coarse_samp - window)
    e           = min(received - fine_win, coarse_samp + window)
    sq          = np.abs(rx_buf[s : e + fine_win])**2
    cs          = np.concatenate([[0.0], np.cumsum(sq)])
    win_pwr     = (cs[fine_win:] - cs[:-fine_win]) / fine_win
    hits        = np.where(win_pwr > fine_thr)[0]
    fine_rel    = int(hits[0]) if len(hits) else 0
    onset_s     = (s + fine_rel) / sample_rate
    expected_s  = delay_s
    error_ms  = abs(onset_s - expected_s) * 1000

    # HW-anchored corrected error: onset_hw_ns vs scheduled tx_time_ns.
    # If the driver provided RX timestamps, use them to eliminate the
    # T_rx_start reference ambiguity entirely.
    if rx_time0[0] is not None:
        onset_sample    = s + fine_rel
        onset_hw_ns     = rx_time0[0] + int(onset_sample / sample_rate * 1e9)
        corrected_ms    = (onset_hw_ns - tx_time_ns - loopback_latency_s * 1e9) / 1e6
        ref_str         = "HW-anchored"
    else:
        corrected_ms    = error_ms - loopback_latency_s * 1000.0
        ref_str         = "approx"
    print(f"Burst onset      : ~{onset_s:.6f} s  (expected {expected_s:.3f} s,  "
          f"raw error {error_ms:.3f} ms,  corrected {corrected_ms:.3f} ms [{ref_str}])")

    # Pass/fail criteria
    POWER_DELTA_MIN_DB = 20   # burst must be at least 20 dB above pre-burst noise
    TIMING_ERROR_MAX_MS = 50  # onset must be within 50 ms of scheduled time

    passed = True
    if delta_db < POWER_DELTA_MIN_DB:
        print(f"FAIL: power delta {delta_db:.1f} dB < {POWER_DELTA_MIN_DB} dB threshold")
        passed = False
    else:
        print(f"PASS: power delta {delta_db:.1f} dB >= {POWER_DELTA_MIN_DB} dB")

    if abs(corrected_ms) > TIMING_ERROR_MAX_MS:
        print(f"FAIL: corrected timing error {corrected_ms:.3f} ms > {TIMING_ERROR_MAX_MS} ms threshold")
        passed = False
    else:
        print(f"PASS: corrected timing error {corrected_ms:.3f} ms <= {TIMING_ERROR_MAX_MS} ms")

    if late_count:
        print(f"FAIL: TimedTXArbiter reported {late_count} late TX event(s)")
        passed = False
    else:
        print("PASS: no late TX events from TimedTXArbiter")

    return passed


def main():
    parser = argparse.ArgumentParser(description="M2SDR Timed TX test (loopback or over the air).")
    parser.add_argument("--delay-s",      type=float, default=0.1,        help="Seconds from now to schedule the burst (default: 0.1)")
    parser.add_argument("--duration-s",   type=float, default=0.5,        help="Burst duration in seconds (default: 0.5)")
    parser.add_argument("--freq-hz",      type=float, default=500_000,    help="Baseband tone frequency in Hz (default: 500000)")
    parser.add_argument("--amplitude",    type=float, default=0.3,        help="Tone amplitude 0..1 (default: 0.3)")
    parser.add_argument("--samplerate",   type=float, default=30_720_000, help="Sample rate in Hz (default: 30720000)")
    parser.add_argument("--rf-freq",      type=float, default=None,       help="RF center frequency in Hz, e.g. 2400000000")
    parser.add_argument("--tx-gain",      type=float, default=None,       help="TX gain in dB")
    parser.add_argument("--rx-gain",      type=float, default=None,       help="RX gain in dB")
    parser.add_argument("--loopback",     action="store_true",            help="Enable AD9361 BIST loopback (default: off)")
    args = parser.parse_args()

    print("M2SDR Timed TX Test")
    print(f"  Delay     : {args.delay_s} s")
    print(f"  Duration  : {args.duration_s} s")
    print(f"  Tone      : {args.freq_hz/1e3:.1f} kHz")
    print(f"  Amplitude : {args.amplitude}")
    print(f"  Rate      : {args.samplerate/1e6:.3f} MSPS")
    if args.rf_freq is not None:
        print(f"  RF freq   : {args.rf_freq/1e6:.3f} MHz")
    if args.tx_gain is not None:
        print(f"  TX gain   : {args.tx_gain} dB")
    if args.rx_gain is not None:
        print(f"  RX gain   : {args.rx_gain} dB")
    print(f"  Loopback  : {'yes' if args.loopback else 'no'}")
    print()

    ok = run_test(
        delay_s     = args.delay_s,
        duration_s  = args.duration_s,
        freq_hz     = args.freq_hz,
        amplitude   = args.amplitude,
        sample_rate = args.samplerate,
        rf_freq     = args.rf_freq,
        tx_gain     = args.tx_gain,
        rx_gain     = args.rx_gain,
        loopback    = args.loopback,
    )

    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
