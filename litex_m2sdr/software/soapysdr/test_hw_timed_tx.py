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
import time

import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_TX, SOAPY_SDR_RX, SOAPY_SDR_CS16, SOAPY_SDR_HAS_TIME, SOAPY_SDR_END_BURST

HERE = os.path.dirname(os.path.abspath(__file__))
M2SDR_UTIL = os.path.join(HERE, "..", "user", "m2sdr_util")
CSR_H = os.path.join(HERE, "..", "kernel", "csr.h")
LOOPBACK_CTRL = 0x10800  # CSR_TXRX_LOOPBACK_CONTROL_ADDR


def csr(name):
    with open(CSR_H) as f:
        m = re.search(rf"#define {name} (0x[0-9a-fA-F]+|\d+)", f.read())
    return int(m.group(1), 0) if m else None


def reg_write(addr, value):
    subprocess.run([M2SDR_UTIL, "reg-write", hex(addr), hex(value)], check=True, capture_output=True)


def reg_read(addr):
    out = subprocess.run([M2SDR_UTIL, "reg-read", hex(addr)], check=True, capture_output=True, text=True).stdout
    m = re.search(r"(0x[0-9a-fA-F]+)", out)
    return int(m.group(1), 16) if m else None


def gate_stats():
    names = ["LATE_COUNT", "HELD_COUNT", "PASSED_COUNT", "STATUS"]
    st = {}
    for n in names:
        a = csr(f"CSR_TIMED_TX_{n}_ADDR")
        st[n.lower()] = reg_read(a) if a is not None else None
    return st


def find_burst(rx_i16, nsamp):
    """Return the index of the first burst sample (counter pattern I=i&0x7ff) in interleaved CS16 data."""
    i = rx_i16[0::2].astype(np.int32)
    q = rx_i16[1::2].astype(np.int32)
    # Burst samples carry I = (i & 0x7ff) - 1024 + 1024 = i & 0x7ff and Q = 0x400 as a marker.
    cand = np.where((q == 0x400) & (i == 0))[0]
    for c in cand:
        if c + 8 < len(i) and np.array_equal(i[c:c + 8], np.arange(8)):
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
    ap.add_argument("--no-loopback", action="store_true", help="leave the FPGA loopback untouched")
    args = ap.parse_args()

    if not args.no_loopback:
        reg_write(LOOPBACK_CTRL, 1)
    try:
        run(args)
    finally:
        if not args.no_loopback:
            reg_write(LOOPBACK_CTRL, 0)


def run(args):
    dev = SoapySDR.Device(dict(driver="LiteXM2SDR", timed_tx=args.mode, ad9361_fir_profile="bypass"))
    for d in (SOAPY_SDR_RX, SOAPY_SDR_TX):
        dev.setSampleRate(d, 0, args.rate)
    rx = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0])
    tx = dev.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, [0], dict(timed_tx=args.mode))
    mtu_tx = dev.getStreamMTU(tx)
    mtu_rx = dev.getStreamMTU(rx)
    print(f"mode={args.mode} rate={args.rate/1e6:.2f} MSps mtu tx={mtu_tx} rx={mtu_rx}")

    dev.activateStream(rx)
    dev.activateStream(tx)
    time.sleep(0.2)
    # Drain whatever the RX side already has.
    scratch = np.zeros(mtu_rx * 2, np.int16)
    for _ in range(50):
        if dev.readStream(rx, [scratch], mtu_rx, timeoutUs=20000).ret <= 0:
            break

    n = args.burst
    burst = np.zeros(n * 2, np.int16)
    burst[0::2] = np.arange(n) & 0x7ff      # counter in I
    burst[1::2] = 0x400                      # marker in Q

    results = []
    for rep in range(args.reps + (1 if args.late_test else 0)):
        late = args.late_test and rep == args.reps
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
            flags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST
        # Read RX until the burst shows up or we are well past T.
        deadline = time.time() + args.lead_ms / 1000 + 1.0
        buf = np.zeros(mtu_rx * 2, np.int16)
        found = None
        while time.time() < deadline and found is None:
            r = dev.readStream(rx, [buf], mtu_rx, timeoutUs=200000)
            if r.ret <= 0:
                continue
            idx = find_burst(buf[: r.ret * 2], r.ret)
            if idx is not None:
                has_time = bool(r.flags & SOAPY_SDR_HAS_TIME)
                rx_ts = r.timeNs + int(idx / args.rate * 1e9)
                found = (rx_ts, has_time, idx)
        st = gate_stats()
        if late:
            print(f"late burst: T={T} (5 ms in the past) -> {'DROPPED (ok)' if found is None else 'EMITTED (unexpected)'}  gate={st}")
        elif found is None:
            print(f"rep {rep}: burst not seen within deadline  gate={st}")
        else:
            rx_ts, has_time, idx = found
            results.append(rx_ts - T)
            print(f"rep {rep}: T={T} rx_ts={rx_ts} (hw_ts={has_time}, idx {idx})  release latency = {rx_ts - T:+d} ns  gate={st}")
        time.sleep(0.1)

    dev.deactivateStream(tx)
    dev.deactivateStream(rx)
    dev.closeStream(tx)
    dev.closeStream(rx)
    if results:
        a = np.array(results)
        print(f"release latency over {len(a)} bursts: mean {a.mean():+.0f} ns, spread {a.max()-a.min()} ns ({(a.max()-a.min())*args.rate/1e9:.1f} samples)")


if __name__ == "__main__":
    sys.exit(main())
