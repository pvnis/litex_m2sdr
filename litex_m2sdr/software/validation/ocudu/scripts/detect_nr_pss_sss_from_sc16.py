#!/usr/bin/env python3
import argparse
from pathlib import Path
import numpy as np

def nr_pss(nid2: int) -> np.ndarray:
    x = np.zeros(127 + 7, dtype=np.int8)
    x[:7] = [0, 1, 1, 0, 1, 1, 1]
    for i in range(127):
        x[i + 7] = (x[i + 4] + x[i]) & 1
    idx = (np.arange(127) + 43 * nid2) % 127
    return (1 - 2 * x[idx]).astype(np.complex64)

def pss_ofdm_symbol(nid2: int, nfft: int) -> np.ndarray:
    X = np.zeros(nfft, dtype=np.complex64)
    k = np.arange(-63, 64)
    X[k % nfft] = nr_pss(nid2)
    s = np.fft.ifft(X) * np.sqrt(nfft)
    s = s.astype(np.complex64)
    s /= np.sqrt(np.sum(np.abs(s) ** 2)) + 1e-12
    return s

def nr_sss(nid1: int, nid2: int) -> np.ndarray:
    # 3GPP NR SSS, length 127.
    # N_ID_1 in [0,335], N_ID_2 in [0,2].
    x0 = np.zeros(127 + 7, dtype=np.int8)
    x1 = np.zeros(127 + 7, dtype=np.int8)
    x0[0] = 1
    x1[0] = 1
    for i in range(127):
        x0[i + 7] = (x0[i + 4] + x0[i]) & 1
        x1[i + 7] = (x1[i + 1] + x1[i]) & 1

    m0 = 15 * (nid1 // 112) + 5 * nid2
    m1 = nid1 % 112
    n = np.arange(127)

    d = (1 - 2 * x0[(n + m0) % 127]) * (1 - 2 * x1[(n + m1) % 127])
    return d.astype(np.complex64)

def load_sc16(path: Path) -> np.ndarray:
    raw = np.fromfile(path, dtype=np.int16)
    if raw.size < 4:
        raise SystemExit(f"ERROR: empty capture: {path}")
    if raw.size % 2:
        raw = raw[:-1]
    raw = raw.reshape(-1, 2)
    x = raw[:, 0].astype(np.float32) + 1j * raw[:, 1].astype(np.float32)
    x = x / 2048.0
    x = x - np.mean(x)
    return x.astype(np.complex64)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("raw")
    ap.add_argument("--fs", type=float, required=True)
    ap.add_argument("--fc", type=float, required=True)
    ap.add_argument("--ssb", type=float, required=True)
    ap.add_argument("--expected-pci", type=int, default=1)
    ap.add_argument("--max-pss-candidates", type=int, default=12)
    ap.add_argument("--sss-delta-min", type=int, default=760)
    ap.add_argument("--sss-delta-max", type=int, default=900)
    ap.add_argument("--cfo-min", type=float, default=-30000.0)
    ap.add_argument("--cfo-max", type=float, default=30000.0)
    ap.add_argument("--cfo-step", type=float, default=2500.0)
    args = ap.parse_args()

    raw = Path(args.raw)
    x = load_sc16(raw)

    fs = args.fs
    fc = args.fc
    ssb_abs = args.ssb
    ssb_offset = ssb_abs - fc
    nfft = int(round(fs / 15000.0))

    expected_nid2 = args.expected_pci % 3
    expected_nid1 = args.expected_pci // 3

    print("===== input =====")
    print(f"raw={raw}")
    print(f"samples={x.size}")
    print(f"duration_s={x.size / fs:.6f}")
    print(f"fs={fs:.3f}")
    print(f"fc={fc:.3f}")
    print(f"ssb_abs={ssb_abs:.3f}")
    print(f"expected_ssb_offset_Hz={ssb_offset:.1f}")
    print(f"nfft={nfft}")
    print(f"expected_pci={args.expected_pci}")
    print(f"expected_nid1={expected_nid1}")
    print(f"expected_nid2={expected_nid2}")

    # Energy-gated regions, same idea as the previous PSS checker.
    block = 8192
    nb = x.size // block
    xb = x[:nb * block].reshape(nb, block)
    brms = 20 * np.log10(np.sqrt(np.mean(np.abs(xb) ** 2, axis=1)) + 1e-15)
    med = float(np.median(brms))
    thr = med + 6.0
    cand = np.where(brms > thr)[0]

    if cand.size == 0:
        cand = np.argsort(brms)[-20:]
    else:
        cand = cand[np.argsort(brms[cand])[-60:]]

    segments = []
    pad = 65536
    for b in sorted(set(int(v) for v in cand)):
        a = max(0, b * block - pad)
        z = min(x.size, (b + 1) * block + pad)
        if not segments or a > segments[-1][1]:
            segments.append([a, z])
        else:
            segments[-1][1] = max(segments[-1][1], z)
    segments = segments[:16]

    print()
    print("===== energy regions searched =====")
    print(f"block_rms_median_dBFS={med:.2f}")
    print(f"threshold_dBFS={thr:.2f}")
    for a, z in segments:
        print(f"segment samples {a}:{z} time {a/fs:.6f}-{z/fs:.6f}s")

    locals_by_nid2 = {nid2: pss_ofdm_symbol(nid2, nfft) for nid2 in (0, 1, 2)}
    cfo_grid = np.arange(args.cfo_min, args.cfo_max + 0.1, args.cfo_step)

    pss_hits = []
    for a, z in segments:
        seg0 = x[a:z]
        sample_idx = np.arange(a, z, dtype=np.float64)

        for cfo in cfo_grid:
            fmix = ssb_offset + cfo
            rot = np.exp(-2j * np.pi * fmix * sample_idx / fs).astype(np.complex64)
            y = seg0 * rot

            pwr = np.abs(y) ** 2
            cs = np.concatenate(([0.0], np.cumsum(pwr, dtype=np.float64)))

            for nid2, local in locals_by_nid2.items():
                if y.size < local.size:
                    continue
                corr = np.correlate(y, local, mode="valid")
                win_energy = cs[local.size:] - cs[:-local.size]
                score = np.abs(corr) / (np.sqrt(win_energy) + 1e-12)

                i = int(np.argmax(score))
                pss_hits.append({
                    "score": float(score[i]),
                    "nid2": nid2,
                    "cfo": float(cfo),
                    "fmix": float(fmix),
                    "sample": int(a + i),
                    "time": float((a + i) / fs),
                })

    pss_hits.sort(key=lambda d: d["score"], reverse=True)

    print()
    print("===== best PSS hits =====")
    for r in pss_hits[:20]:
        print(
            f"score={r['score']:.4f} nid2={r['nid2']} "
            f"cfo_Hz={r['cfo']:+.1f} sample={r['sample']} time_s={r['time']:.6f}"
        )

    pss_expected = [r for r in pss_hits if r["nid2"] == expected_nid2]
    if not pss_expected:
        print("ERROR: no expected-NID2 PSS candidates")
        raise SystemExit(1)

    pss_candidates = pss_expected[:args.max_pss_candidates]

    # Precompute SSS table for the already-known NID2.
    sss_table = np.stack([nr_sss(nid1, expected_nid2) for nid1 in range(336)], axis=0)
    sss_table = sss_table / (np.sqrt(np.sum(np.abs(sss_table) ** 2, axis=1, keepdims=True)) + 1e-12)

    sss_hits = []
    k = np.arange(-63, 64)

    for pr in pss_candidates:
        pss_sample = pr["sample"]
        cfo = pr["cfo"]
        fmix = ssb_offset + cfo

        # Window around expected SSS OFDM-symbol body.
        # PSS and SSS are two OFDM symbols apart in the SS/PBCH block.
        for delta in range(args.sss_delta_min, args.sss_delta_max + 1):
            s0 = pss_sample + delta
            s1 = s0 + nfft
            if s0 < 0 or s1 > x.size:
                continue

            idx = np.arange(s0, s1, dtype=np.float64)
            y = x[s0:s1] * np.exp(-2j * np.pi * fmix * idx / fs).astype(np.complex64)

            Y = np.fft.fft(y) / np.sqrt(nfft)
            bins = Y[k % nfft].astype(np.complex64)
            bins = bins - np.mean(bins)
            bins_norm = bins / (np.sqrt(np.sum(np.abs(bins) ** 2)) + 1e-12)

            corr = np.abs(sss_table @ np.conj(bins_norm))
            nid1 = int(np.argmax(corr))
            score = float(corr[nid1])
            sss_hits.append({
                "score": score,
                "nid1": nid1,
                "nid2": expected_nid2,
                "pci": 3 * nid1 + expected_nid2,
                "delta": delta,
                "sample": s0,
                "time": s0 / fs,
                "pss_score": pr["score"],
                "pss_sample": pss_sample,
                "cfo": cfo,
            })

    sss_hits.sort(key=lambda d: d["score"], reverse=True)

    print()
    print("===== best SSS / PCI hits =====")
    for r in sss_hits[:30]:
        mark = " EXPECTED" if r["pci"] == args.expected_pci else ""
        print(
            f"score={r['score']:.4f} pci={r['pci']} nid1={r['nid1']} nid2={r['nid2']} "
            f"delta={r['delta']} cfo_Hz={r['cfo']:+.1f} "
            f"time_s={r['time']:.6f} pss_score={r['pss_score']:.4f}{mark}"
        )

    print()
    print("===== best per PCI expectation =====")
    expected_rows = [r for r in sss_hits if r["pci"] == args.expected_pci]
    wrong_rows = [r for r in sss_hits if r["pci"] != args.expected_pci]

    if expected_rows:
        er = expected_rows[0]
        print(
            f"EXPECTED_PCI={args.expected_pci}: score={er['score']:.4f}, "
            f"nid1={er['nid1']}, nid2={er['nid2']}, delta={er['delta']}, "
            f"cfo_Hz={er['cfo']:+.1f}, time_s={er['time']:.6f}"
        )
    else:
        print(f"EXPECTED_PCI={args.expected_pci}: NOT_FOUND")

    if wrong_rows:
        wr = wrong_rows[0]
        print(
            f"BEST_WRONG_PCI={wr['pci']}: score={wr['score']:.4f}, "
            f"nid1={wr['nid1']}, nid2={wr['nid2']}, delta={wr['delta']}"
        )

    if expected_rows and wrong_rows:
        ratio = expected_rows[0]["score"] / (wrong_rows[0]["score"] + 1e-12)
        print(f"expected_vs_best_wrong_ratio={ratio:.3f}")
        if ratio >= 1.20:
            print("VERDICT=SSS_EXPECTED_PCI_DOMINATES")
        else:
            print("VERDICT=SSS_AMBIGUOUS_OR_WRONG")
    elif expected_rows:
        print("VERDICT=SSS_EXPECTED_PCI_FOUND_NO_WRONG_ROWS")
    else:
        print("VERDICT=SSS_EXPECTED_PCI_NOT_FOUND")

if __name__ == "__main__":
    main()
