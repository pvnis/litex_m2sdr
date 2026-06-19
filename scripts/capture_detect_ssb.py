#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
#
# Capture a 5G NR downlink on a bladeRF and detect the gNB's SSB (PSS).
#
# Self-contained: captures IQ from the bladeRF at the cell's DL center, then
# runs a self-validated NR PSS correlator (SCS-aware) that sweeps carrier
# frequency offset and reports, per N_id2, a discriminative detection margin
# (best sequence vs. 2nd-best at the same instant — broadband bursts score
# ~1, a real PSS scores tens-to-hundreds). A synthetic self-test runs first
# to prove the detector is configured correctly for the chosen SCS.
#
# Defaults are for the OCUDU band-78 cell (dl_arfcn 626000 = 3390 MHz,
# ssb_arfcn 625632, SCS 30 kHz). Read those two ARFCNs off the gNB banner
# ("Cell pci=... dl_arfcn=... dl_ssb_arfcn=...") and pass them if they differ.
#
# Usage (needs USB access -> run with sudo):
#   sudo python3 scripts/capture_detect_ssb.py
#   sudo python3 scripts/capture_detect_ssb.py --dl-arfcn 626000 --ssb-arfcn 625632 \
#        --scs-khz 30 --gain 55 --pci 1
#
# Exit code 0 if the SSB is detected (margin >= --margin-pass), else 1.

import argparse
import logging
import sys
import time
from datetime import datetime

import numpy as np

try:
    import bladerf
    from bladerf import _bladerf
except ImportError:
    print("ERROR: bladeRF Python bindings not found (pip install bladerf / pybladerf).")
    sys.exit(2)

LOG = logging.getLogger("ssb_detect")


# ---------------------------------------------------------------------------
# NR helpers
# ---------------------------------------------------------------------------
def nr_arfcn_to_hz(nref: int) -> float:
    """3GPP TS 38.104 global frequency raster."""
    if 0 <= nref < 600000:
        return nref * 5_000.0
    if 600000 <= nref < 2016667:
        return 3_000_000_000.0 + (nref - 600000) * 15_000.0
    if 2016667 <= nref <= 3279165:
        return 24_250_080_000.0 + (nref - 2016667) * 60_000.0
    raise ValueError(f"NR-ARFCN {nref} outside the global raster")


def pss_sequence(nid2: int) -> np.ndarray:
    """NR PSS d_PSS(n), length 127 (TS 38.211 7.4.2.2)."""
    x = [0, 1, 1, 0, 1, 1, 1]
    for i in range(127 - 7):
        x.append((x[i + 4] + x[i]) % 2)
    return np.array([1 - 2 * x[(n + 43 * nid2) % 127] for n in range(127)], dtype=float)


def pss_time_domain(nid2: int, nfft: int) -> np.ndarray:
    """Time-domain PSS OFDM symbol (127 subcarriers centred at DC), unit RMS."""
    grid = np.zeros(nfft, dtype=complex)
    grid[np.arange(-63, 64) % nfft] = pss_sequence(nid2)
    td = np.fft.ifft(grid)
    return td / np.sqrt(np.mean(np.abs(td) ** 2))


def search_pss(x: np.ndarray, srate: float, nfft: int, ssb_offset_hz: float,
               cfo_lo: int, cfo_hi: int, cfo_step: int):
    """Sweep CFO; return per-N_id2 (margin, cfo_hz, sample_idx, peak/rms).

    margin = peak(best N_id2) / peak(2nd-best N_id2) at the strongest instant.
    A genuine PSS makes one N_id2 dominate; broadband bursts/noise do not.
    """
    t = np.arange(x.size) / srate
    # Centre the SSB at DC.
    x = x * np.exp(-1j * 2 * np.pi * ssb_offset_hz * t)
    L, M = x.size, nfft
    fftlen = 1 << int(np.ceil(np.log2(L + M - 1)))
    templates = [np.fft.fft(pss_time_domain(n, nfft)[::-1].conj(), fftlen) for n in range(3)]

    per = [(-1.0, 0, 0, 0.0)] * 3   # per N_id2: (margin, cfo, idx, snr)
    for cfo in range(cfo_lo, cfo_hi + 1, cfo_step):
        xr = x * np.exp(-1j * 2 * np.pi * cfo * t)
        X = np.fft.fft(xr, fftlen)
        corr = np.stack([np.abs(np.fft.ifft(X * templates[n])[M - 1:L]) for n in range(3)])
        idx = int(corr.max(0).argmax())              # strongest instant
        vals = corr[:, idx]
        order = np.argsort(vals)[::-1]
        win = int(order[0])
        margin = float(vals[order[0]] / (vals[order[1]] + 1e-12))
        snr = float(vals[win] / (np.sqrt(np.mean(corr[win] ** 2)) + 1e-12))
        if margin > per[win][0]:
            per[win] = (margin, cfo, idx, snr)
    return per


# ---------------------------------------------------------------------------
# bladeRF capture
# ---------------------------------------------------------------------------
def open_rx(center_hz: float, srate: float, bw: float, gain_db: float):
    """Open + configure the bladeRF RX; return (dev, ch)."""
    LOG.info("Opening bladeRF ...")
    dev = bladerf.BladeRF()
    ch = bladerf.CHANNEL_RX(0)
    dev.set_frequency(ch, int(center_hz))
    dev.set_sample_rate(ch, int(srate))
    dev.set_bandwidth(ch, int(bw))
    dev.set_gain(ch, int(gain_db))
    actual_f = dev.get_frequency(ch)
    LOG.info("RX tuned: %.6f MHz (req %.6f), %.3f MSPS, BW %.1f MHz, gain %d dB",
             actual_f / 1e6, center_hz / 1e6, srate / 1e6, bw / 1e6, gain_db)
    dev.sync_config(layout=_bladerf.ChannelLayout.RX_X1,
                    fmt=_bladerf.Format.SC16_Q11,
                    num_buffers=32, buffer_size=16 * 1024,
                    num_transfers=16, stream_timeout=5000)
    dev.enable_module(ch, True)
    return dev, ch


def read_iq(dev, ch, n_samples: int, settle: bool) -> np.ndarray:
    chunk = 256 * 1024
    buf = bytearray(chunk * 4)
    out = np.empty(n_samples, dtype=np.complex64)
    got = 0
    if settle:  # discard a couple of chunks so AGC/transients settle (first read only)
        for _ in range(2):
            dev.sync_rx(buf, chunk, timeout_ms=5000)
    while got < n_samples:
        m = min(chunk, n_samples - got)
        dev.sync_rx(buf, m, timeout_ms=5000)
        raw = np.frombuffer(bytes(buf[:m * 4]), dtype="<i2")
        out[got:got + m] = (raw[0::2].astype(np.float32) + 1j * raw[1::2].astype(np.float32)) / 2048.0
        got += m
    return out


# ---------------------------------------------------------------------------
def main() -> int:
    p = argparse.ArgumentParser(description="Capture an NR DL on bladeRF and detect the gNB SSB.")
    p.add_argument("--dl-arfcn", type=int, default=626000, help="DL NR-ARFCN (cell centre). Default band-78 626000=3390 MHz")
    p.add_argument("--ssb-arfcn", type=int, default=625632, help="SSB NR-ARFCN (gNB banner 'dl_ssb_arfcn'). Default 625632")
    p.add_argument("--scs-khz", type=float, default=30.0, help="SSB subcarrier spacing kHz (band 78 = 30)")
    p.add_argument("--pci", type=int, default=1, help="Expected PCI (to check N_id2 = PCI mod 3)")
    p.add_argument("--srate", type=float, default=23.04e6, help="Sample rate (Hz)")
    p.add_argument("--bw", type=float, default=20e6, help="Analog RX bandwidth (Hz)")
    p.add_argument("--gain", type=float, default=55.0, help="bladeRF RX gain (dB)")
    p.add_argument("--duration", type=float, default=0.40, help="Capture duration (s)")
    p.add_argument("--detect-ms", type=float, default=45.0, help="Window used for PSS detection (ms)")
    p.add_argument("--cfo-range", type=int, default=60000, help="CFO search +/- Hz")
    p.add_argument("--cfo-step", type=int, default=2000, help="CFO search step Hz (use 1000 for finer margin/slower)")
    p.add_argument("--margin-pass", type=float, default=20.0, help="Detection margin threshold for PASS")
    p.add_argument("--monitor", type=int, default=1, metavar="N",
                   help="Keep ONE bladeRF session and capture/detect N times (shows true "
                        "within-session CFO stability vs per-open jitter). Default 1.")
    p.add_argument("--monitor-gap", type=float, default=1.0, help="Seconds between monitor captures")
    p.add_argument("--save-iq", type=str, default=None, help="Optional path to save the raw SC16 capture (single-shot only)")
    p.add_argument("--save-cf32", type=str, default=None, help="Optional path to save capture as complex-float32 (srsRAN COMPLEX_FLOAT_BIN, single-shot)")
    p.add_argument("--log-file", type=str, default=None, help="Log file path (default: auto /tmp/ssb_detect_<ts>.log)")
    args = p.parse_args()

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = args.log_file or f"/tmp/ssb_detect_{ts}.log"
    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", "%H:%M:%S")
    LOG.setLevel(logging.INFO)
    sh = logging.StreamHandler(sys.stdout); sh.setFormatter(fmt); LOG.addHandler(sh)
    fh = logging.FileHandler(log_path); fh.setFormatter(fmt); LOG.addHandler(fh)

    center_hz = nr_arfcn_to_hz(args.dl_arfcn)
    ssb_hz = nr_arfcn_to_hz(args.ssb_arfcn)
    ssb_offset = ssb_hz - center_hz
    nfft = int(round(args.srate / (args.scs_khz * 1e3)))
    n_samples = int(args.srate * args.duration)
    exp_nid2 = args.pci % 3

    LOG.info("================ NR SSB capture+detect ================")
    LOG.info("log file        : %s", log_path)
    LOG.info("DL centre       : %.3f MHz (arfcn %d)", center_hz / 1e6, args.dl_arfcn)
    LOG.info("SSB centre      : %.3f MHz (arfcn %d)  => %+0.3f MHz baseband offset",
             ssb_hz / 1e6, args.ssb_arfcn, ssb_offset / 1e6)
    LOG.info("SCS / SSB FFT   : %.0f kHz / N=%d   srate %.3f MSPS", args.scs_khz, nfft, args.srate / 1e6)
    LOG.info("expected N_id2  : %d (PCI %d mod 3)", exp_nid2, args.pci)
    LOG.info("CFO search      : +/-%d Hz step %d Hz", args.cfo_range, args.cfo_step)

    # ---- self-test: detector must find a synthetic SSB strongly ----
    LOG.info("--- detector self-test (synthetic N_id2=%d SSB) ---", exp_nid2)
    rng = np.random.default_rng(0)
    sig = (rng.standard_normal(int(args.srate * 0.05)) +
           1j * rng.standard_normal(int(args.srate * 0.05))).astype(np.complex64) * 0.4
    pss = pss_time_domain(exp_nid2, nfft).astype(np.complex64)
    for k in range(2):
        s = 5000 + k * int(args.srate * 0.02)
        sig[s:s + nfft] += 3.0 * pss
    tt = np.arange(sig.size) / args.srate
    sig = sig * np.exp(1j * 2 * np.pi * (ssb_offset + 5000) * tt).astype(np.complex64)
    st = search_pss(sig, args.srate, nfft, ssb_offset, -12000, 12000, 2000)
    st_margin = st[exp_nid2][0]
    LOG.info("self-test: N_id2=%d margin=%.1f  (should be >>10) -> %s",
             exp_nid2, st_margin, "OK" if st_margin > 30 else "WARNING: detector config suspect")

    # ---- open bladeRF once (so monitor mode shows true within-session stability) ----
    try:
        dev, ch = open_rx(center_hz, args.srate, args.bw, args.gain)
    except Exception as e:
        LOG.error("Could not open bladeRF: %s", e)
        return 2

    margins, cfos, dets = [], [], 0
    try:
        for it in range(max(1, args.monitor)):
            iq = read_iq(dev, ch, n_samples, settle=(it == 0))
            rms = 20 * np.log10(np.sqrt(np.mean(np.abs(iq) ** 2)) + 1e-12)
            peak = 20 * np.log10(np.max(np.abs(iq)) + 1e-12)
            clip = float(np.mean((np.abs(iq.real) > 0.98) | (np.abs(iq.imag) > 0.98))) * 100
            if args.save_iq and args.monitor == 1:
                b = np.empty(2 * iq.size, dtype="<i2")
                b[0::2] = np.clip(np.round(iq.real * 2048), -32768, 32767)
                b[1::2] = np.clip(np.round(iq.imag * 2048), -32768, 32767)
                b.tofile(args.save_iq)
                LOG.info("saved raw IQ -> %s", args.save_iq)
            if args.save_cf32 and args.monitor == 1:
                iq.astype(np.complex64).tofile(args.save_cf32)  # cf_t = interleaved float32
                LOG.info("saved cf32 (%d samples) -> %s", iq.size, args.save_cf32)

            win = iq[: int(args.srate * args.detect_ms / 1e3)]
            per = search_pss(win, args.srate, nfft, ssb_offset,
                             -args.cfo_range, args.cfo_range, args.cfo_step)
            best = max(range(3), key=lambda n: per[n][0])
            bm, bcfo, bidx, bsnr = per[best]
            ok = bm >= args.margin_pass and best == exp_nid2
            dets += int(ok)
            margins.append(bm); cfos.append(bcfo)

            tag = "DETECTED " if ok else ("WEAK     " if bm >= 12 and best == exp_nid2 else "NO-SSB   ")
            if args.monitor > 1:
                LOG.info("[%02d/%02d] %s N_id2=%d margin=%5.1f CFO=%+7d Hz (%+.1f ppm) | "
                         "RMS %.1f peak %.1f dBFS%s", it + 1, args.monitor, tag, best, bm, bcfo,
                         bcfo / center_hz * 1e6, rms, peak,
                         "  CLIP!" if peak > -0.2 else "")
            else:
                LOG.info("captured %d samples  RMS %.1f dBFS  peak %.1f dBFS  clipping %.4f%%",
                         iq.size, rms, peak, clip)
                if peak > -0.2:
                    LOG.warning("RX near full-scale (peak %.1f dBFS) -> may be clipping; lower --gain", peak)
                for n in range(3):
                    m, cfo, idx, snr = per[n]
                    LOG.info("  N_id2=%d : margin=%6.1f  CFO=%+7d Hz  t=%7.3f ms  peak/rms=%.1f%s",
                             n, m, cfo, idx / args.srate * 1e3, snr,
                             "   <== expected (PCI)" if n == exp_nid2 else "")
            if args.monitor > 1 and it + 1 < args.monitor:
                time.sleep(args.monitor_gap)
    finally:
        dev.enable_module(ch, False); dev.close()

    # ---- verdict / summary ----
    LOG.info("======================================================")
    if args.monitor > 1:
        mar = np.array(margins, float); cf = np.array(cfos, float)
        LOG.info("MONITOR SUMMARY over %d captures (one bladeRF session):", args.monitor)
        LOG.info("  detected (margin>=%.0f & PCI-match): %d/%d", args.margin_pass, dets, args.monitor)
        LOG.info("  margin   : min %.1f  median %.1f  max %.1f", mar.min(), np.median(mar), mar.max())
        LOG.info("  CFO (Hz) : min %+.0f  median %+.0f  max %+.0f  | std %.0f Hz (%.1f ppm)",
                 cf.min(), np.median(cf), cf.max(), cf.std(), cf.std() / center_hz * 1e6)
        LOG.info("  => CFO std %.0f Hz within ONE session: %s", cf.std(),
                 "STABLE (compensatable with SRSUE_BLADE_PPM)" if cf.std() < 5000 else
                 "UNSTABLE (gNB reference is drifting fast -> hard for any UE)")
        ret = 0 if dets > 0 else 1
    else:
        bm, bcfo = margins[0], cfos[0]
        best = max(range(3), key=lambda n: per[n][0]); detected = bm >= args.margin_pass and best == exp_nid2
        LOG.info("BEST: N_id2=%d  margin=%.1f  CFO=%+d Hz  ppm=%+.1f", best, bm, bcfo, bcfo / center_hz * 1e6)
        if detected:
            LOG.info("VERDICT: SSB DETECTED (margin %.1f >= %.1f, N_id2=%d matches PCI %d). "
                     "gNB downlink is on the air and decodable.", bm, args.margin_pass, best, args.pci)
        elif bm >= args.margin_pass and best != exp_nid2:
            LOG.warning("VERDICT: a PSS was found (margin %.1f) but N_id2=%d != expected %d "
                        "(wrong PCI, or spectral inversion / waveform distortion).", bm, best, exp_nid2)
        elif bm >= 12:
            LOG.warning("VERDICT: WEAK / MARGINAL (margin %.1f). SSB likely present but too weak "
                        "for reliable acquisition. Raise SSB SNR (power/backoff) or improve coupling.", bm)
        else:
            LOG.error("VERDICT: NO SSB DETECTED (margin %.1f ~ noise floor). Check the gNB is "
                      "transmitting on this band, the ARFCNs/SCS, and the RX gain/frequency.", bm)
        ret = 0 if detected else 1
    LOG.info("======================================================")
    return ret


if __name__ == "__main__":
    sys.exit(main())
