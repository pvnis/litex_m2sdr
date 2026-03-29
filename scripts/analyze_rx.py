#!/usr/bin/env python3
"""Analyze a 2T2R IQ capture from m2sdr_record.

Usage: python3 analyze_rx.py <file.bin> [sample_rate_hz]

File format: interleaved int16 [I1, Q1, I2, Q2] repeating.
"""
import sys
import numpy as np

filename   = sys.argv[1] if len(sys.argv) > 1 else "/tmp/rx_loopback.bin"
fs         = float(sys.argv[2]) if len(sys.argv) > 2 else 30.72e6
N          = 8192

data = np.fromfile(filename, dtype=np.int16)
total_frames = len(data) // 4
print(f"File: {filename}")
print(f"  {len(data)} int16 words → {total_frames} IQ frames  ({total_frames/fs*1000:.1f} ms at {fs/1e6:.3f} MSPS)")
print(f"  Non-zero: {np.count_nonzero(data)}/{len(data)} ({100*np.count_nonzero(data)/len(data):.1f}%)")
print()

I1 = data[0::4].astype(np.float32)
Q1 = data[1::4].astype(np.float32)
I2 = data[2::4].astype(np.float32)
Q2 = data[3::4].astype(np.float32)

print("Component stats (full file):")
for name, s in [("I1", I1), ("Q1", Q1), ("I2", I2), ("Q2", Q2)]:
    print(f"  {name}: mean={s.mean():+7.1f}  std={s.std():7.1f}  min={s.min():6.0f}  max={s.max():6.0f}")
print()

freq = np.fft.fftshift(np.fft.fftfreq(N, d=1/fs))

for ch_name, I, Q in [("CH1", I1, Q1), ("CH2", I2, Q2)]:
    iq      = I[:N] + 1j * Q[:N]
    spectrum = np.abs(np.fft.fftshift(np.fft.fft(iq)))
    peak_idx = np.argmax(spectrum)
    peak_f   = freq[peak_idx]
    sig_pwr  = spectrum[peak_idx]**2
    noise_pwr = np.median(spectrum**2)
    snr_db   = 10 * np.log10(sig_pwr / noise_pwr) if noise_pwr > 0 else float("nan")
    print(f"{ch_name}: peak={peak_f/1e6:+8.4f} MHz   SNR={snr_db:6.1f} dB")
    top5 = np.argsort(spectrum)[::-1][:5]
    for i in top5:
        print(f"       {freq[i]/1e6:+8.4f} MHz   mag={spectrum[i]:.0f}")
    print()
