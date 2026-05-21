#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
#
# M2SDR Loopback Latency Test
#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
#
# Sends a tone burst over TX, captures RX samples, locates the burst onset,
# and reports the round-trip pipeline latency.  The reported latency in samples
# is the recommended `time_alignment_calibration` magnitude for OCUDU configs.
#
# Prerequisites:
#   m2sdr_rf --samplerate <rate>   (initialise AD9361 at the target rate first)
#
# Usage:
#   python3 m2sdr_loopback_latency.py [--samplerate 23040000] [--loopback]
#                                      [--loopback-mode phy|ad9361]
#                                      [--freq-hz 500000] [--n-runs 3]

import sys
import argparse
import threading
import time
import numpy as np

try:
    import SoapySDR
    from SoapySDR import SOAPY_SDR_TX, SOAPY_SDR_RX, SOAPY_SDR_CF32
    from SoapySDR import SOAPY_SDR_HAS_TIME, SOAPY_SDR_END_BURST
except ImportError:
    print("ERROR: SoapySDR Python bindings not found.")
    sys.exit(1)


def make_tone(n_samples, freq_hz, sample_rate, amplitude=0.5, start_index=0):
    t = start_index + np.arange(n_samples, dtype=np.float64)
    return (amplitude * np.exp(1j * 2 * np.pi * freq_hz / sample_rate * t)).astype(np.complex64)


def make_marker(n_samples, amplitude=0.5, seed=0):
    rng = np.random.default_rng(seed)
    symbols = rng.integers(0, 4, size=n_samples, dtype=np.int8)
    phases = (np.pi / 2.0) * symbols.astype(np.float64)
    return (amplitude * np.exp(1j * phases)).astype(np.complex64)


def sample_index_to_time_ns(index, chunk_offsets, ts, nrx, sample_rate):
    chunk_i = int(np.searchsorted(chunk_offsets[1:], index, side="right"))
    intra   = int(index - chunk_offsets[chunk_i])
    intra   = min(intra, max(0, int(nrx[chunk_i]) - 1))
    return ts[chunk_i] + int(round(intra / sample_rate * 1e9))


def time_ns_to_sample_index(time_ns, chunk_offsets, ts, nrx, sample_rate):
    for i in range(len(nrx)):
        if nrx[i] <= 0 or not ts[i]:
            continue
        chunk_end_ns = ts[i] + int(round(nrx[i] / sample_rate * 1e9))
        if time_ns < chunk_end_ns:
            intra = max(0, int(np.floor((time_ns - ts[i]) * sample_rate / 1e9)))
            intra = min(intra, max(0, int(nrx[i]) - 1))
            return int(chunk_offsets[i] + intra)
    return None


def measure_once(dev, sample_rate, freq_hz, chunk=4096, n_chunks=512, n_skip=4,
                 lead_time_s=0.050, warmup_s=2.0, rx_gated_by_tx=False,
                 amplitude=0.95):
    """
    Schedule a single timed TX transition (silence -> marker) and locate the
    onset in the RX capture by correlating against the known marker.

    Flow:
      1. Set up + activate TX + RX streams.
      2. RX *warm-up*: drain reads until RX actually produces samples. The
         FPGA's internal PPS startup defers RX for ~0.5-1 s after activate;
         we don't want that latency contaminating the loopback measurement.
      3. Once RX is live, schedule TX `lead_time_s` in the future.
      4. Continue reading until enough chunks span the expected marker.
      5. Correlate the entire RX window against the marker.
      6. Report two latencies:
         - pipeline_latency_ns: peak_hw - tx_time_ns (FPGA loopback path,
           the real metric)
         - to_marker_latency_ns: peak_hw - expected_marker_ns (matches the
           script's `time_alignment_calibration` magnitude)

    Returns dict with metrics, or None if the burst wasn't detected.
    """
    tx_s = dev.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32)
    rx_s = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    dev.activateStream(tx_s)
    dev.activateStream(rx_s)

    tx_mtu = int(dev.getStreamMTU(tx_s))
    rx_mtu = int(dev.getStreamMTU(rx_s))
    chunk  = min(chunk, tx_mtu, rx_mtu)

    # --- Step 1: warm up RX so the FPGA's PPS-startup delay isn't counted.
    # PHY loopback gates RX on TX, so there's nothing to warm up there;
    # just wait a fixed interval to let the FPGA's internal PPS startup
    # complete before TX fires.
    rx_live_ts = None
    if rx_gated_by_tx:
        time.sleep(min(warmup_s, 1.2))
        print(f"    [phy/gated] sleep {warmup_s:.2f}s for FPGA PPS startup")
    else:
        warmup_chunks = 0
        warmup_buf = np.empty(chunk, dtype=np.complex64)
        warmup_deadline = time.monotonic() + warmup_s
        while time.monotonic() < warmup_deadline:
            sr = dev.readStream(rx_s, [warmup_buf], chunk, timeoutUs=200_000)
            if sr.ret > 0 and (sr.flags & SOAPY_SDR_HAS_TIME):
                rx_live_ts = sr.timeNs
                warmup_chunks += 1
                if warmup_chunks >= 8:
                    break
        if rx_live_ts is None:
            print(f"    RX did not become live within {warmup_s}s warmup "
                  f"(try --rx-gated-by-tx if using PHY loopback)")
            dev.deactivateStream(tx_s); dev.deactivateStream(rx_s)
            dev.closeStream(tx_s); dev.closeStream(rx_s)
            return None
        print(f"    [warmup] RX live after {warmup_chunks} chunks, ts={rx_live_ts}")

    # --- Step 2: capture buffers spanning the upcoming TX burst.
    capture_total_s = lead_time_s + 0.030  # 30 ms tail after marker
    capture_chunks = int(np.ceil(capture_total_s * sample_rate / chunk)) + 16
    capture_chunks = max(capture_chunks, n_chunks)

    raw = [np.zeros(chunk, dtype=np.complex64) for _ in range(capture_chunks)]
    pwr = np.zeros(capture_chunks, dtype=np.float64)
    nrx = np.zeros(capture_chunks, dtype=np.int64)
    ts  = np.zeros(capture_chunks, dtype=np.int64)

    marker_offset = max(1, chunk // 4)
    marker_len    = chunk - marker_offset
    marker = make_marker(marker_len, amplitude=amplitude,
                         seed=int(dev.getHardwareTime()) & 0xffffffff)
    transition_chunk = np.zeros(chunk, dtype=np.complex64)
    transition_chunk[marker_offset:] = marker
    silence_chunk = np.zeros(chunk, dtype=np.complex64)

    tx_errors = []
    rx_errors = []
    rx_total_samples = 0

    tx_time_ns = int(dev.getHardwareTime() + lead_time_s * 1e9)
    expected_marker_ns = tx_time_ns + int(round(marker_offset / sample_rate * 1e9))

    # Keep TX active across the marker so AD9361 BIST and antenna paths
    # see a continuous transmission to route to RX. Surround the marker
    # with N buffers of silence on each side; only the first buffer
    # carries HAS_TIME, only the last buffer carries END_BURST.
    pad_before_buffers = 4
    pad_after_buffers  = int(np.ceil(0.030 * sample_rate / chunk)) + 16  # ~30 ms tail

    def tx_worker():
        per_buf_timeout_us = int(0.5e6)
        # Silence before the marker. The whole TX burst is timed from
        # the first pre-padding buffer; the marker then lands exactly
        # pad_before_buffers buffers later.
        chunk_ns = int(round(chunk / sample_rate * 1e9))
        burst_start_ns = tx_time_ns - pad_before_buffers * chunk_ns
        for k in range(pad_before_buffers):
            flags = SOAPY_SDR_HAS_TIME if k == 0 else 0
            time_ns = burst_start_ns if k == 0 else 0
            sr = dev.writeStream(
                tx_s, [silence_chunk], chunk,
                flags, time_ns, timeoutUs=per_buf_timeout_us)
            if sr.ret != chunk:
                tx_errors.append(("pre", k, sr.ret))
                return
        # The marker buffer is an untimed continuation of the same burst.
        sr = dev.writeStream(
            tx_s, [transition_chunk], chunk,
            0, 0,
            timeoutUs=int((lead_time_s + 1.0) * 1e6))
        if sr.ret != chunk:
            tx_errors.append(("marker", sr.ret))
            return
        # Tail of silence to keep TX DMA fed while RX captures.
        for k in range(pad_after_buffers):
            flags = SOAPY_SDR_END_BURST if k == pad_after_buffers - 1 else 0
            sr = dev.writeStream(
                tx_s, [silence_chunk], chunk,
                flags, 0, timeoutUs=per_buf_timeout_us)
            if sr.ret != chunk:
                tx_errors.append(("post", k, sr.ret))
                return

    tx_thread = threading.Thread(target=tx_worker, daemon=True)
    tx_thread.start()

    capture_deadline = time.monotonic() + max(5.0, capture_total_s + 2.0)
    last_progress = time.monotonic()
    chunks_read = 0
    for i in range(capture_chunks):
        if time.monotonic() >= capture_deadline and rx_total_samples == 0:
            print("    RX produced no samples before capture deadline; aborting this run")
            break
        sr = dev.readStream(rx_s, [raw[i]], chunk, timeoutUs=100_000)
        if sr.ret > 0:
            nrx[i] = sr.ret
            chunks_read = i + 1
            if sr.flags & SOAPY_SDR_HAS_TIME:
                ts[i] = sr.timeNs
            pwr[i] = float(np.mean(np.abs(raw[i][:sr.ret]) ** 2))
            rx_total_samples += sr.ret
        elif sr.ret == -1:
            pass
        else:
            rx_errors.append(sr.ret)

        now = time.monotonic()
        if rx_total_samples == 0 and now - last_progress >= 1.0:
            print("    waiting for RX samples...")
            last_progress = now

    n_chunks = max(chunks_read, 1)

    tx_thread.join(timeout=5.0)

    dev.deactivateStream(tx_s)
    dev.deactivateStream(rx_s)
    dev.closeStream(tx_s)
    dev.closeStream(rx_s)

    if tx_errors:
        print(f"    TX errors: {tx_errors[:8]}")
    if rx_errors:
        print(f"    RX errors: {rx_errors[:8]}")

    valid_chunks = [i for i in range(n_chunks) if nrx[i] > 0 and ts[i]]
    if not valid_chunks:
        print(f"    no timestamped RX captured (rx_total={rx_total_samples})")
        return None

    print(
        f"    chunk={chunk}  n_chunks={n_chunks}  "
        f"valid={len(valid_chunks)}  rx_samples={rx_total_samples}  "
        f"tx_time={tx_time_ns}"
    )

    rx_cat = np.concatenate([raw[i][:nrx[i]] for i in valid_chunks]).astype(np.complex64, copy=False)
    chunk_offsets = np.zeros(len(valid_chunks) + 1, dtype=np.int64)
    for j, i in enumerate(valid_chunks):
        chunk_offsets[j + 1] = chunk_offsets[j] + nrx[i]
    ts_valid = [int(ts[i]) for i in valid_chunks]
    nrx_valid = [int(nrx[i]) for i in valid_chunks]

    rx_start_ns = ts_valid[0]
    rx_end_ns = ts_valid[-1] + int(round(nrx_valid[-1] / sample_rate * 1e9))
    rx_window_ms = (rx_end_ns - rx_start_ns) / 1e6

    pwr_db = 10 * np.log10(pwr + 1e-30)
    floor_ids = valid_chunks[:max(1, n_skip)]
    floor_db = float(np.median(pwr_db[floor_ids]))
    peak_chunk_idx = int(np.argmax(pwr_db[valid_chunks]))
    peak_chunk_db = float(pwr_db[valid_chunks[peak_chunk_idx]])
    print(
        f"    rx_window=[{rx_start_ns}, {rx_end_ns})  span={rx_window_ms:.3f} ms"
    )
    print(
        f"    floor={floor_db:.1f} dBFS  peak_chunk={peak_chunk_idx}  "
        f"peak_chunk_db={peak_chunk_db:.1f}  contrast={peak_chunk_db-floor_db:+.1f} dB"
    )

    # Full-window correlation: do not gamble on a targeted search_start
    # since worker timestamps can lead/lag wall time during transients.
    corr = np.abs(np.correlate(rx_cat, marker, mode="valid"))
    if len(corr) == 0:
        print(f"    empty correlation: rx_len={len(rx_cat)} marker_len={len(marker)}")
        return None

    peak_idx = int(np.argmax(corr))
    peak_val = float(corr[peak_idx])
    baseline = float(np.median(corr)) + 1e-12
    snr = peak_val / baseline
    peak_hw_ns = sample_index_to_time_ns(
        peak_idx, chunk_offsets, ts_valid, nrx_valid, sample_rate)
    print(
        f"    corr_peak={peak_val:.3g}  median={baseline:.3g}  "
        f"snr={snr:.1f}  peak_idx={peak_idx}  peak_hw={peak_hw_ns}"
    )

    if snr < 4.0:
        print(f"    burst not detected (snr {snr:.1f} below threshold)")
        return None

    pipeline_latency_ns = peak_hw_ns - tx_time_ns
    to_marker_latency_ns = peak_hw_ns - expected_marker_ns
    pipeline_samp = int(round(pipeline_latency_ns / 1e9 * sample_rate))
    to_marker_samp = int(round(to_marker_latency_ns / 1e9 * sample_rate))

    return {
        "tx_time_ns":          tx_time_ns,
        "expected_marker_ns":  expected_marker_ns,
        "peak_hw_ns":          peak_hw_ns,
        "peak_idx":            peak_idx,
        "pipeline_latency_ns": pipeline_latency_ns,
        "pipeline_samples":    pipeline_samp,
        "to_marker_latency_ns": to_marker_latency_ns,
        "to_marker_samples":   to_marker_samp,
        "snr":                 snr,
    }


def run(sample_rate, freq_hz, loopback_mode, n_runs,
        tx_att_db=0.0, rx_gain_db=20.0, amplitude=0.95):
    args = {"driver": "LiteXM2SDR"}
    if loopback_mode != "off":
        args["loopback_mode"] = loopback_mode

    print(f"Opening SoapySDR device (loopback={loopback_mode}, "
          f"tx_att={tx_att_db} dB, rx_gain={rx_gain_db} dB, amp={amplitude}) ...")
    dev = SoapySDR.Device(args)
    dev.setSampleRate(SOAPY_SDR_TX, 0, sample_rate)
    dev.setSampleRate(SOAPY_SDR_RX, 0, sample_rate)
    # Gain knobs: SoapySDR's TX gain is "negative attenuation" (89 dB scale on
    # AD9361). Use the named TX1 channel; AD9361 1T1R uses ch 0.
    try:
        # AD9361 TX gain range is roughly [-89.75, 0] dB. Convert
        # attenuation -> gain by negation, clipped.
        tx_gain_db = max(-89.75, min(0.0, -tx_att_db))
        dev.setGain(SOAPY_SDR_TX, 0, tx_gain_db)
    except Exception as e:
        print(f"  warn: setGain(TX, 0, {tx_gain_db}) failed: {e}")
    try:
        dev.setGain(SOAPY_SDR_RX, 0, rx_gain_db)
    except Exception as e:
        print(f"  warn: setGain(RX, 0, {rx_gain_db}) failed: {e}")

    # PHY loopback gates RX on TX activity, so we can't warm RX up.
    rx_gated_by_tx = (loopback_mode == "phy")

    results = []
    for run_i in range(n_runs):
        r = measure_once(dev, sample_rate, freq_hz,
                         rx_gated_by_tx=rx_gated_by_tx,
                         amplitude=amplitude)
        if r is None:
            print(f"  Run {run_i + 1}/{n_runs}: burst not detected")
            continue
        pipeline_ms = r["pipeline_latency_ns"] / 1e6
        to_marker_ms = r["to_marker_latency_ns"] / 1e6
        print(
            f"  Run {run_i + 1}/{n_runs}: pipeline {r['pipeline_samples']} samples "
            f"({pipeline_ms:.3f} ms)  to_marker {r['to_marker_samples']} samples "
            f"({to_marker_ms:.3f} ms)  snr={r['snr']:.0f}"
        )
        results.append(r)

    del dev

    if not results:
        print("\nERROR: burst not detected in any run — check loopback cable or --loopback flag.")
        return False

    pipeline_samps = [r["pipeline_samples"] for r in results]
    to_marker_samps = [r["to_marker_samples"] for r in results]
    pipeline_med = int(np.median(pipeline_samps))
    to_marker_med = int(np.median(to_marker_samps))
    pipeline_ms  = pipeline_med / sample_rate * 1000.0
    to_marker_ms = to_marker_med / sample_rate * 1000.0

    print()
    print("=== Result ===")
    print(f"Pipeline latency (peak_hw - tx_time)         : "
          f"{pipeline_med} samples  ({pipeline_ms:.3f} ms)  "
          f"[median of {len(results)} run(s)]")
    print(f"To-marker latency (peak_hw - expected_marker): "
          f"{to_marker_med} samples  ({to_marker_ms:.3f} ms)")
    print()
    print("Recommended OCUDU config:")
    print(f"  time_alignment_calibration: {-to_marker_med}")
    print()
    print("Update in: ocudu-pavo/configs/m2sdr-nocore.yml")
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Measure TX→RX pipeline latency for OCUDU time_alignment_calibration.")
    parser.add_argument("--samplerate", type=float, default=23_040_000,
                        help="Sample rate Hz (default: 23040000, matches OCUDU test config)")
    parser.add_argument("--freq-hz",    type=float, default=500_000,
                        help="Tone frequency Hz (default: 500000)")
    parser.add_argument("--loopback",   action="store_true",
                        help="Enable loopback using the legacy default mode (equivalent to --loopback-mode=phy)")
    parser.add_argument("--loopback-mode", choices=["off", "phy", "ad9361"], default="off",
                        help="Loopback mode to use: off, FPGA PHY loopback, or AD9361 internal loopback")
    parser.add_argument("--n-runs",     type=int,   default=3,
                        help="Number of burst measurements to average (default: 3)")
    parser.add_argument("--tx-att",     type=float, default=0.0,
                        help="TX attenuation in dB (default: 0 — max output for loopback)")
    parser.add_argument("--rx-gain",    type=float, default=20.0,
                        help="RX gain in dB (default: 20)")
    parser.add_argument("--amplitude",  type=float, default=0.95,
                        help="Marker amplitude in [0, 1] (default: 0.95)")
    args = parser.parse_args()

    if args.loopback and args.loopback_mode == "off":
        args.loopback_mode = "phy"

    print("M2SDR Loopback Latency Test")
    print(f"  Sample rate : {args.samplerate / 1e6:.3f} MSPS")
    print(f"  Tone        : {args.freq_hz / 1e3:.1f} kHz")
    loopback_desc = {
        "off": "off (needs RF cable)",
        "phy": "phy (FPGA internal)",
        "ad9361": "ad9361 (RFIC internal)",
    }[args.loopback_mode]
    print(f"  Loopback    : {loopback_desc}")
    print(f"  Runs        : {args.n_runs}")
    print()

    ok = run(args.samplerate, args.freq_hz, args.loopback_mode, args.n_runs,
             tx_att_db=args.tx_att, rx_gain_db=args.rx_gain,
             amplitude=args.amplitude)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
