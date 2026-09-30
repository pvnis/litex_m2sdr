#!/usr/bin/env python3
#
# This file is part of LiteX-M2SDR.
#
# Copyright (c) 2026 Pavonis / Enjoy-Digital contributors.
# SPDX-License-Identifier: BSD-2-Clause

"""Hardware timed-TX gate test through the FPGA digital TX->RX loopback (no RF involved).

Stamps a burst of counter samples for board time T = now + lead and checks, from the hardware RX
timestamps, when the gate released it:

    release latency = rx_ts(first burst sample) - T

which must be a small constant across runs (it is the gate->header pipeline depth; the RF path's
additional constant is measured over the air with align.py). Also checks that a burst stamped
further in the past than the late margin is dropped and counted, and prints the gate counters.

Usage (root, gNB stopped):
    sudo ./test_hw_timed_tx.py --rate 23.04e6 --lead-ms 100 --reps 3
    sudo ./test_hw_timed_tx.py --mode software ...   # same test through the software timeline
"""

import argparse
import os
import re
import subprocess
import sys
import threading
import time

import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_TX, SOAPY_SDR_RX, SOAPY_SDR_CS16, SOAPY_SDR_HAS_TIME, SOAPY_SDR_END_BURST

HERE = os.path.dirname(os.path.abspath(__file__))

# The Python binding does not forward the driver's log messages by default; print them ourselves.
def _log_handler(level, message):
    print(f"[soapy {level}] {message}", flush=True)
SoapySDR.registerLogHandler(_log_handler)          # keep a module-level reference
SoapySDR.setLogLevel(SoapySDR.SOAPY_SDR_INFO)      # the env var does not take effect in this binding
M2SDR_UTIL = os.path.join(HERE, "..", "user", "m2sdr_util")
CSR_H = os.path.join(HERE, "..", "kernel", "csr.h")
LOOPBACK_CTRL = 0x10800  # CSR_TXRX_LOOPBACK_CONTROL_ADDR
PHY_CTRL = 0xc02c        # CSR_AD9361_PHY_CONTROL_ADDR: bit0 mode (1R1T/2R2T), bit1 loopback


def csr(name):
    with open(CSR_H) as f:
        m = re.search(rf"#define {name} (0x[0-9a-fA-F]+|\d+)", f.read())
    return int(m.group(1), 0) if m else None


def reg_write(addr, value):
    subprocess.run([M2SDR_UTIL, "reg-write", hex(addr), hex(value)], check=True, capture_output=True)


def reg_read(addr):
    out = subprocess.run([M2SDR_UTIL, "reg-read", hex(addr)], check=True, capture_output=True, text=True).stdout
    m = re.search(r":\s*(0x[0-9a-fA-F]+)", out)      # "Reg 0x...: 0xVALUE"
    return int(m.group(1), 16) if m else None


def reg_read64(addr):
    hi, lo = reg_read(addr), reg_read(addr + 4)
    return None if hi is None or lo is None else (hi << 32) | lo


def gate_stats():
    st = {}
    for n in ["CONTROL", "LATE_COUNT", "HELD_COUNT", "PASSED_COUNT", "STATUS"]:
        a = csr(f"CSR_TIMED_TX_{n}_ADDR")
        st[n.lower()] = reg_read(a) if a is not None else None
    a = csr("CSR_TIMED_TX_ARMED_TS_ADDR")
    st["armed_ts"] = reg_read64(a) if a is not None else None
    # Header extractor view: control (bit1 = header_enable) and the last header/timestamp it latched.
    st["hdr_tx_ctrl"] = reg_read(csr("CSR_HEADER_TX_CONTROL_ADDR"))
    st["last_tx_hdr"] = hex(reg_read64(csr("CSR_HEADER_LAST_TX_HEADER_ADDR")) or 0)
    st["last_tx_ts"] = reg_read64(csr("CSR_HEADER_LAST_TX_TIMESTAMP_ADDR"))
    return st


NONCE = 0x400 | (int(time.time()) & 0xff)   # per-run marker (< 0x800 so it survives 12-bit bit-mode conversion)


def find_burst(rx_i16, nsamp):
    """Return the index of the first burst sample (counter pattern I=i&0x7ff, Q=NONCE) in interleaved CS16 data."""
    i = rx_i16[0::2].astype(np.int32)
    q = rx_i16[1::2].astype(np.int32)
    q12 = q & 0xfff
    i12 = i & 0xfff
    cand = np.where((q12 == NONCE) & (i12 == 0))[0]
    for c in cand:
        if c + 8 < len(i12) and np.array_equal(i12[c:c + 8], np.arange(8)):
            return int(c)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=23.04e6)
    ap.add_argument("--lead-ms", type=float, default=100.0)
    ap.add_argument("--burst", type=int, default=2044 * 4, help="burst length in samples")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--mode", default="hardware", choices=["hardware", "software"])
    ap.add_argument("--late-test", action="store_true", help="also send a burst 5 ms in the past (must be dropped)")
    ap.add_argument("--tx-lead", type=int, default=8, help="plugin tx_lead_buffers (ring slots written ahead of the reader)")
    ap.add_argument("--loopback", default="rfic", choices=["rfic", "digital", "none"],
                    help="rfic: AD9361 PHY data loopback (paced by the sample clock); digital: FPGA stream loopback (unpaced)")
    ap.add_argument("--raw-peek", action="store_true",
                    help="after activation, stop the FPGA from stripping TX headers so RX shows raw TX buffers; print frame heads")
    args = ap.parse_args()

    if args.loopback == "digital":
        reg_write(LOOPBACK_CTRL, 1)
    try:
        run(args)
    finally:
        if args.loopback == "digital":
            reg_write(LOOPBACK_CTRL, 0)
        if args.loopback == "rfic":
            c = reg_read(PHY_CTRL)
            if c is not None:
                reg_write(PHY_CTRL, c & ~2)


def run(args):
    # Same call order as OCUDU: streams first, then rates. (setSampleRate before setupStream with a
    # FIR profile trips a divide-by-zero in the ADI driver's ad9361_gc_update.)
    # This SoapySDR Python binding rejects dict arguments in make() ("no match"); use the string form.
    dev = SoapySDR.Device(f"driver=LiteXM2SDR,timed_tx={args.mode},rx_worker_packets=2048,tx_lead_buffers={args.tx_lead}")
    rx = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0])
    tx = dev.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, [0], SoapySDR.KwargsFromString(f"timed_tx={args.mode}"))
    for d in (SOAPY_SDR_RX, SOAPY_SDR_TX):
        dev.setSampleRate(d, 0, args.rate)
    mtu_tx = dev.getStreamMTU(tx)
    mtu_rx = dev.getStreamMTU(rx)
    print(f"mode={args.mode} rate={args.rate/1e6:.2f} MSps mtu tx={mtu_tx} rx={mtu_rx} nonce={NONCE:#x}")

    dev.activateStream(rx)
    dev.activateStream(tx)
    time.sleep(0.2)
    if args.loopback == "rfic":
        # AD9361 PHY data loopback: TX samples re-enter the RX path inside the RFIC interface, paced by
        # the sample clock. Set after activation so the plugin's channel-mode write is preserved.
        c = reg_read(PHY_CTRL)
        reg_write(PHY_CTRL, c | 2)
        print(f"rfic phy loopback on (phy_control {c:#x} -> {(c | 2):#x})")
    if args.raw_peek:
        # header.tx.control: enable=1, header_enable=0 -> the extractor passes frames untouched, so the
        # RX loopback shows the raw TX DMA buffers including the 16-byte header written by libm2sdr.
        reg_write(csr("CSR_HEADER_TX_CONTROL_ADDR"), 1)
        nb = 2044 * 2
        burst = np.zeros(nb * 2, np.int16); burst[0::2] = np.arange(nb) & 0x7ff; burst[1::2] = NONCE
        T = dev.getHardwareTime() + 30_000_000
        sent = 0
        while sent < nb:
            r = dev.writeStream(tx, [burst[sent * 2:]], min(mtu_tx, nb - sent), SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST, T + int(sent / args.rate * 1e9), timeoutUs=1_000_000)
            if r.ret < 0: print("  writeStream", r.ret); break
            sent += r.ret
        deadline = time.time() + 1.5; shown = 0
        buf = np.zeros(mtu_rx * 2, np.int16)
        while time.time() < deadline and shown < 3:
            r = dev.readStream(rx, [buf], mtu_rx, timeoutUs=200000)
            if r.ret <= 0: continue
            idx = find_burst(buf[: r.ret * 2], r.ret)
            if idx is None: continue
            u16 = buf.view(np.uint16)
            lo = max(0, idx * 2 - 16)
            print(f"  raw: burst at sample idx {idx} of RX frame (ts={r.timeNs}); 16 u16 before it: {[hex(x) for x in u16[lo: idx * 2]]}")
            print(f"       frame head (first 12 u16): {[hex(x) for x in u16[:12]]}   expected header u16 LE: 5aa5 x4 then stamp {T:#x}")
            shown += 1
        reg_write(csr("CSR_HEADER_TX_CONTROL_ADDR"), 3)
        dev.deactivateStream(tx); dev.deactivateStream(rx); dev.closeStream(tx); dev.closeStream(rx)
        return
    # RX capture thread: keeps up with the 11k buffers/s so the RX ring is never lapped (a lapped
    # ring hands back stale headers with fresh data and the timestamps become meaningless).
    frames = []          # (timeNs, flags, samples i16 interleaved)
    stop = threading.Event()
    jumps = []
    def rx_loop():
        buf = np.zeros(mtu_rx * 2, np.int16)
        expect = None
        while not stop.is_set():
            r = dev.readStream(rx, [buf], mtu_rx, timeoutUs=100000)
            if r.ret <= 0:
                continue
            if expect is not None and abs(r.timeNs - expect) > 200:
                jumps.append((expect, r.timeNs))
            expect = r.timeNs + int(round(r.ret / args.rate * 1e9))
            frames.append((r.timeNs, r.flags, buf[: r.ret * 2].copy()))
    th = threading.Thread(target=rx_loop, daemon=True); th.start()
    time.sleep(0.3)

    n = args.burst
    stamps = []   # (rep, T, nonce, late)
    for rep in range(args.reps + (1 if args.late_test else 0)):
        late = args.late_test and rep == args.reps
        nonce = 0x400 | ((NONCE + rep) & 0xff)
        burst = np.zeros(n * 2, np.int16)
        burst[0::2] = np.arange(n) & 0x7ff      # counter in I
        burst[1::2] = nonce                      # per-burst marker in Q
        now = dev.getHardwareTime()
        T = now - 5_000_000 if late else now + int(args.lead_ms * 1e6)
        flags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST
        sent = 0
        while sent < n:
            r = dev.writeStream(tx, [burst[sent * 2:]], min(mtu_tx, n - sent), flags, T + int(sent / args.rate * 1e9), timeoutUs=1_000_000)
            if r.ret < 0:
                print(f"  writeStream error {r.ret} ({SoapySDR.errToStr(r.ret)})")
                break
            sent += r.ret
        stamps.append((rep, T, nonce, late))
        if not late:
            wait = (T - dev.getHardwareTime()) / 1e9 - 0.05
            if wait > 0:
                time.sleep(wait)
            mid = gate_stats()
            print(f"rep {rep}: T={T} nonce={nonce:#x}  mid-hold(T-50ms): holding={bool(mid['status'] & 8)} held={mid['held_count']} late={mid['late_count']} armed_ts={mid['armed_ts']}")
            time.sleep(0.05 + 0.2)
        else:
            print(f"rep {rep}: late burst T={T} (5 ms in the past) nonce={nonce:#x}")
            time.sleep(0.3)

    time.sleep(0.3)
    stop.set(); th.join(1.0)
    st = gate_stats()
    dev.deactivateStream(tx)
    dev.deactivateStream(rx)
    dev.closeStream(tx)
    dev.closeStream(rx)

    print(f"captured {len(frames)} RX frames, {len(jumps)} timestamp jumps; gate={st}")
    for j in jumps[:5]:
        print(f"  jump: expected {j[0]} got {j[1]} ({(j[1]-j[0])/1e3:+.1f} us)")
    if not frames:
        return
    ts0 = np.array([f[0] for f in frames], np.int64)
    data = np.concatenate([f[2] for f in frames])
    lens = np.array([len(f[2]) // 2 for f in frames]); starts = np.concatenate([[0], np.cumsum(lens)[:-1]])
    i12 = data[0::2].astype(np.int32) & 0xfff
    q12 = data[1::2].astype(np.int32) & 0xfff
    results = []
    for rep, T, nonce, late in stamps:
        cand = np.where((q12 == nonce) & (i12 == 0))[0]
        hits = [int(c) for c in cand if c + 8 < len(i12) and np.array_equal(i12[c:c + 8], np.arange(8))]
        if late:
            print(f"late burst: {'DROPPED (ok)' if not hits else f'EMITTED at {len(hits)} place(s) (unexpected)'}")
            continue
        if not hits:
            print(f"rep {rep}: burst not found in capture")
            continue
        c = hits[0]
        k = int(np.searchsorted(starts, c, side='right') - 1)
        rx_ts = int(ts0[k]) + int(round((c - starts[k]) / args.rate * 1e9))
        results.append(rx_ts - T)
        # Count how many of the burst's samples survived contiguous (counter continuity).
        m = 0
        while c + m < len(i12) and i12[c + m] == (m & 0x7ff) and q12[c + m] == nonce:
            m += 1
        print(f"rep {rep}: rx_ts={rx_ts}  release latency = {rx_ts - T:+d} ns ({(rx_ts - T) * args.rate / 1e9:+.1f} samples)  contiguous {m}/{n} samples, {len(hits)} occurrence(s)")
    if results:
        a = np.array(results)
        print(f"release latency over {len(a)} bursts: mean {a.mean():+.0f} ns, spread {a.max()-a.min()} ns ({(a.max()-a.min())*args.rate/1e9:.1f} samples)")


if __name__ == "__main__":
    sys.exit(main())
