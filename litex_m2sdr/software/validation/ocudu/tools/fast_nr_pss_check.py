#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np
from scipy.signal import oaconvolve

def nr_pss(nid2: int) -> np.ndarray:
    # NR PSS m-sequence, N_ID_2 in {0,1,2}.
    x = np.zeros(127 + 7, dtype=np.int8)
    x[:7] = [0, 1, 1, 0, 1, 1, 1]
    for i in range(127):
        x[i + 7] = (x[i + 4] + x[i]) & 1
    idx = (np.arange(127) + 43 * nid2) % 127
    return (1 - 2 * x[idx]).astype(np.complex64)

def pss_time_template(nid2: int, nfft: int) -> np.ndarray:
    # Put PSS on subcarriers -63..+63 and IFFT to one OFDM symbol.
    X = np.zeros(nfft, dtype=np.complex64)
    X[np.arange(-63, 64) % nfft] = nr_pss(nid2)
    s = np.fft.ifft(X).astype(np.complex64) * np.sqrt(nfft)
    s = s / (np.sqrt(np.sum(np.abs(s) ** 2)) + 1e-12)
    return s.astype(np.complex64)

def load_sc16(path: Path) -> np.ndarray:
    raw = np.fromfile(path, dtype=np.int16)
    if raw.size < 4:
        raise RuntimeError(f"empty or too-short capture: {path}")
    if raw.size % 2:
        raw = raw[:-1]
    raw = raw.reshape(-1, 2)
    x = raw[:, 0].astype(np.float32) + 1j * raw[:, 1].astype(np.float32)
    x = (x / 2048.0).astype(np.complex64)
    x = x - np.mean(x)
    return x.astype(np.complex64)

def summarize_energy(x: np.ndarray, fs: float) -> None:
    rms = np.sqrt(np.mean(np.abs(x) ** 2))
    peak = np.max(np.abs(x))
    print(f"samples={x.size}")
    print(f"duration_s={x.size / fs:.6f}")
    print(f"rms_dBFS={20*np.log10(rms + 1e-15):.2f}")
    print(f"peak_dBFS={20*np.log10(peak + 1e-15):.2f}")

    block = 8192
    nb = x.size // block
    if nb:
        y = x[:nb * block].reshape(nb, block)
        brms = 20 * np.log10(np.sqrt(np.mean(np.abs(y) ** 2, axis=1)) + 1e-15)
        print(f"block_rms_median_dBFS={np.median(brms):.2f}")
        print(f"block_rms_max_dBFS={np.max(brms):.2f}")
        print(f"block_rms_max_over_median_dB={np.max(brms) - np.median(brms):.2f}")

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("raw", type=Path)
    ap.add_argument("--fs", type=float, required=True)
    ap.add_argument("--fc", type=float, required=True)
    ap.add_argument("--ssb", type=float, required=True)
    ap.add_argument("--expected-nid2", type=int, default=1)
    ap.add_argument("--cfo-min", type=float, default=-30000.0)
    ap.add_argument("--cfo-max", type=float, default=30000.0)
    ap.add_argument("--cfo-step", type=float, default=1000.0)
    ap.add_argument("--max-samples", type=int, default=0, help="0 means full file")
    args = ap.parse_args()

    x = load_sc16(args.raw)
    if args.max_samples and x.size > args.max_samples:
        # Keep the middle of the file; avoids startup/shutdown edge effects.
        start = (x.size - args.max_samples) // 2
        x = x[start:start + args.max_samples].copy()
        print(f"cropped_to_samples={x.size}")

    fs = args.fs
    fc = args.fc
    ssb = args.ssb
    ssb_offset = ssb - fc
    nfft = int(round(fs / 15000.0))

    print("===== input =====")
    print(f"raw={args.raw}")
    print(f"fs={fs:.3f}")
    print(f"fc={fc:.3f}")
    print(f"ssb_abs={ssb:.3f}")
    print(f"ssb_offset_Hz={ssb_offset:.1f}")
    print(f"ofdm_nfft={nfft}")
    print(f"scs_Hz={fs / nfft:.6f}")
    print(f"expected_NID2={args.expected_nid2}")
    summarize_energy(x, fs)

    locals_by_nid2 = {nid2: pss_time_template(nid2, nfft) for nid2 in (0, 1, 2)}
    n = np.arange(x.size, dtype=np.float32)

    cfo_grid = np.arange(args.cfo_min, args.cfo_max + 0.1, args.cfo_step, dtype=np.float32)
    results = []

    print()
    print("===== search =====")
    print(f"cfo_grid_Hz={args.cfo_min:+.0f}..{args.cfo_max:+.0f} step {args.cfo_step:.0f}")
    print("method=scipy.signal.oaconvolve FFT overlap-add, full capture scan")

    for cfo in cfo_grid:
        mix_freq = np.float32(ssb_offset + cfo)
        rot = np.exp((-2j * np.pi * mix_freq / fs) * n).astype(np.complex64)
        y = (x * rot).astype(np.complex64)

        pwr = (np.abs(y) ** 2).astype(np.float64)
        cs = np.concatenate(([0.0], np.cumsum(pwr)))

        for nid2, templ in locals_by_nid2.items():
            # Correlation y with templ: sum y[i+k] * conj(templ[k]).
            h = np.conj(templ[::-1]).astype(np.complex64)
            corr = oaconvolve(y, h, mode="valid")

            L = templ.size
            e = cs[L:] - cs[:-L]
            score = np.abs(corr) / (np.sqrt(e) + 1e-12)

            idx = int(np.argmax(score))
            results.append({
                "score": float(score[idx]),
                "nid2": nid2,
                "cfo": float(cfo),
                "mixed_offset": float(mix_freq),
                "sample": idx,
                "time": idx / fs,
            })

    results.sort(key=lambda r: r["score"], reverse=True)

    print()
    print("===== top correlations =====")
    for r in results[:30]:
        print(
            f"score={r['score']:.6f} "
            f"NID2={r['nid2']} "
            f"CFO_Hz={r['cfo']:+.0f} "
            f"mixed_offset_Hz={r['mixed_offset']:+.0f} "
            f"sample={r['sample']} "
            f"time_s={r['time']:.6f}"
        )

    print()
    print("===== best per NID2 =====")
    best_by_nid2 = {}
    for nid2 in (0, 1, 2):
        rows = [r for r in results if r["nid2"] == nid2]
        best_by_nid2[nid2] = rows[0]
        r = rows[0]
        print(
            f"NID2={nid2}: "
            f"score={r['score']:.6f}, "
            f"CFO_Hz={r['cfo']:+.0f}, "
            f"time_s={r['time']:.6f}, "
            f"sample={r['sample']}"
        )

    expected = best_by_nid2[args.expected_nid2]
    others = [best_by_nid2[i]["score"] for i in (0, 1, 2) if i != args.expected_nid2]
    ratio = expected["score"] / (max(others) + 1e-12)

    print()
    print("===== verdict helper =====")
    print(f"expected_NID2={args.expected_nid2}")
    print(f"expected_score={expected['score']:.6f}")
    print(f"expected_vs_best_other_ratio={ratio:.3f}")

    if ratio > 1.20:
        print("VERDICT=PSS_EXPECTED_NID2_DOMINATES")
    elif ratio > 1.05:
        print("VERDICT=PSS_EXPECTED_NID2_SLIGHTLY_HIGHER")
    else:
        print("VERDICT=PSS_AMBIGUOUS_OR_DETECTOR_NEEDS_REFINEMENT")

if __name__ == "__main__":
    main()
