#!/usr/bin/env python3
"""
monitor_dma.py — Read m2sdr DMA flow CSRs while gNB (or any SoapySDR app) is running.

Uses LITEPCIE_IOCTL_REG ioctl — does NOT acquire the DMA lock, safe to run
alongside any process that holds the DMA reader/writer lock.

Usage:
    python3 monitor_dma.py [--device /dev/m2sdr0] [--interval 1.0]
"""

import argparse
import fcntl
import os
import struct
import time

# ---------------------------------------------------------------------------
# IOCTL definition
# _IOWR('S', 0, struct litepcie_ioctl_reg)
# struct litepcie_ioctl_reg { uint32_t addr; uint32_t val; uint8_t is_write; }
# sizeof = 12 bytes (4+4+1 + 3 padding)
# _IOWR(type, nr, size): (3<<30) | (type<<8) | nr | (size<<16)
# = 0xC0000000 | (0x53 << 8) | 0 | (12 << 16) = 0xC00C5300
# ---------------------------------------------------------------------------
LITEPCIE_IOCTL_REG  = 0xC00C5300
IOCTL_REG_FMT       = "=IIBxxx"   # addr(4) val(4) is_write(1) pad(3) = 12 bytes

# CSR addresses (from litex_m2sdr/software/kernel/csr.h)
CSR_DMA_WRITER_LOOP_STATUS   = 0x6014   # RX path: [31:16]=loop count, [15:0]=buf index
CSR_DMA_WRITER_TABLE_LEVEL   = 0x6018   # RX path: queued writer descriptors / level
CSR_DMA_READER_LOOP_STATUS   = 0x6034   # TX path: same layout
CSR_DMA_READER_TABLE_LEVEL   = 0x6038   # TX path: queued reader descriptors / level
CSR_DMA_WRITER_ENABLE        = 0x6000
CSR_DMA_READER_ENABLE        = 0x6020
CSR_AD9361_PHY_CONTROL       = 0xC020   # bit0 = 0:2T2R, 1:1T1R

CSR_TIME_GEN_CONTROL         = 0x8800   # bit1 = READ latch
CSR_TIME_GEN_READ_TIME_HI    = 0x8804   # upper 32 bits of latched HW time
CSR_TIME_GEN_READ_TIME_LO    = 0x8808   # lower 32 bits

CSR_TIMED_TX_LATE_COUNT      = 0x12804
CSR_TIMED_TX_UNDERRUN_COUNT  = 0x12808

CSR_SAMPLE_COUNTER_CONTROL   = 0x13000  # write bit0=1 to latch
CSR_SAMPLE_COUNTER_COUNT_HI  = 0x13004  # upper 32 bits of latched 64-bit sample count
CSR_SAMPLE_COUNTER_COUNT_LO  = 0x13008  # lower 32 bits

EXPECTED_SRATE = 23.04e6  # Hz — change if using a different sample rate
DMA_BUFFER_PER_IRQ = 1


def csr_read(fd: int, addr: int) -> int:
    buf = bytearray(struct.pack(IOCTL_REG_FMT, addr, 0, 0))
    fcntl.ioctl(fd, LITEPCIE_IOCTL_REG, buf)
    return struct.unpack(IOCTL_REG_FMT, bytes(buf))[1]


def csr_write(fd: int, addr: int, val: int) -> None:
    buf = bytearray(struct.pack(IOCTL_REG_FMT, addr, val, 1))
    fcntl.ioctl(fd, LITEPCIE_IOCTL_REG, buf)


def read_sample_counter(fd: int) -> int:
    """Latch and read the 64-bit RX sample counter."""
    csr_write(fd, CSR_SAMPLE_COUNTER_CONTROL, 1)
    hi = csr_read(fd, CSR_SAMPLE_COUNTER_COUNT_HI)
    lo = csr_read(fd, CSR_SAMPLE_COUNTER_COUNT_LO)
    return (hi << 32) | lo


def read_hw_time_ns(fd: int) -> int:
    """Latch and read the 64-bit hardware time (nanoseconds)."""
    ctrl = csr_read(fd, CSR_TIME_GEN_CONTROL)
    csr_write(fd, CSR_TIME_GEN_CONTROL, ctrl | (1 << 1))   # set READ latch
    csr_write(fd, CSR_TIME_GEN_CONTROL, ctrl & ~(1 << 1))  # clear latch
    hi = csr_read(fd, CSR_TIME_GEN_READ_TIME_HI)
    lo = csr_read(fd, CSR_TIME_GEN_READ_TIME_LO)
    return (hi << 32) | lo


def read_phy_mode(fd: int) -> str:
    """Read AD9361 PHY mode from CSR: 1T1R or 2T2R."""
    ctrl = csr_read(fd, CSR_AD9361_PHY_CONTROL)
    return "1T1R" if (ctrl & 0x1) else "2T2R"


def rx_samples_per_beat(phy_mode: str) -> int:
    """Convert RX beat count to complex samples per active channel."""
    return 2 if phy_mode == "1T1R" else 1


def read_interrupt_count(pattern: str) -> int:
    """Sum CPU interrupt counters for /proc/interrupts lines containing pattern."""
    total = 0
    with open("/proc/interrupts", "r", encoding="utf-8") as f:
        for line in f:
            if pattern not in line:
                continue
            fields = line.split()
            for token in fields[1:]:
                if token.isdigit():
                    total += int(token)
                else:
                    break
    return total


def main() -> None:
    ap = argparse.ArgumentParser(description="Monitor m2sdr DMA/CSR flow metrics.")
    ap.add_argument("--device",   default="/dev/m2sdr0", help="Device node (default: /dev/m2sdr0)")
    ap.add_argument("--interval", type=float, default=1.0, help="Poll interval in seconds (default: 1.0)")
    ap.add_argument("--srate",    type=float, default=EXPECTED_SRATE,
                    help=f"Expected sample rate Hz (default: {EXPECTED_SRATE:.0f})")
    ap.add_argument("--irq-pattern", default="m2sdr",
                    help="Substring to match in /proc/interrupts (default: m2sdr)")
    args = ap.parse_args()

    fd = os.open(args.device, os.O_RDWR)

    # Print header
    hdr = (f"{'Elapsed':>8}  {'RX beat/s':>12}  {'RX samp/s':>12}  {'Mode':>6}  {'TX buf/s':>10}  {'IRQ/s':>10}  "
           f"{'IRQ:buf':>8}  {'RX en':>5}  {'RX lvl':>6}  {'RX loops':>9}  {'RX idx':>7}  "
           f"{'TX en':>5}  {'TX lvl':>6}  {'TX loops':>9}  {'TX idx':>7}  "
           f"{'Late':>6}  {'Underrun':>8}  {'HW time s':>10}  {'Status':>6}")
    sep = "-" * len(hdr)
    print(hdr)
    print(sep)

    prev_sc = read_sample_counter(fd)
    prev_irq = read_interrupt_count(args.irq_pattern)
    prev_tx_stat = csr_read(fd, CSR_DMA_READER_LOOP_STATUS)
    prev_t  = time.monotonic()
    start_t = prev_t
    line    = 0

    try:
        while True:
            time.sleep(args.interval)
            now = time.monotonic()

            sc      = read_sample_counter(fd)
            rx_stat = csr_read(fd, CSR_DMA_WRITER_LOOP_STATUS)
            tx_stat = csr_read(fd, CSR_DMA_READER_LOOP_STATUS)
            rx_lvl  = csr_read(fd, CSR_DMA_WRITER_TABLE_LEVEL)
            tx_lvl  = csr_read(fd, CSR_DMA_READER_TABLE_LEVEL)
            rx_en   = csr_read(fd, CSR_DMA_WRITER_ENABLE) & 0x1
            tx_en   = csr_read(fd, CSR_DMA_READER_ENABLE) & 0x1
            phy_mode = read_phy_mode(fd)
            irq_cnt = read_interrupt_count(args.irq_pattern)
            late    = csr_read(fd, CSR_TIMED_TX_LATE_COUNT)
            underrun= csr_read(fd, CSR_TIMED_TX_UNDERRUN_COUNT)
            hw_ns   = read_hw_time_ns(fd)

            elapsed  = now - start_t
            delta_sc = sc - prev_sc
            dt       = now - prev_t
            beat_rate = delta_sc / dt if dt > 0 else 0.0
            rate      = beat_rate * rx_samples_per_beat(phy_mode)
            delta_irq = irq_cnt - prev_irq

            prev_tx_loops = (prev_tx_stat >> 16) & 0xFFFF
            prev_tx_idx   = prev_tx_stat & 0xFFFF
            cur_tx_loops  = (tx_stat >> 16) & 0xFFFF
            cur_tx_idx    = tx_stat & 0xFFFF
            delta_tx_bufs = ((cur_tx_loops - prev_tx_loops) & 0xFFFF) * 512 + (cur_tx_idx - prev_tx_idx)
            tx_buf_rate   = delta_tx_bufs / dt if dt > 0 else 0.0
            irq_rate      = delta_irq / dt if dt > 0 else 0.0
            irq_per_buf   = (delta_irq / delta_tx_bufs) if delta_tx_bufs > 0 else 0.0

            rx_loops = (rx_stat >> 16) & 0xFFFF
            rx_idx   = rx_stat & 0xFFFF
            tx_loops = cur_tx_loops
            tx_idx   = cur_tx_idx

            rx_active = delta_sc > 0
            tx_active = delta_tx_bufs > 0

            # Only validate a direction when it is active in this sample window.
            rate_ok  = (not rx_active) or (abs(rate - args.srate) < args.srate * 0.05)
            late_ok  = (not tx_active) or (late == 0)
            urun_ok  = (not tx_active) or (underrun == 0)
            irq_ok   = (not tx_active) or (abs(irq_per_buf - (1.0 / DMA_BUFFER_PER_IRQ)) < 0.2)
            status   = "OK" if (rate_ok and late_ok and urun_ok and irq_ok) else "WARN"

            print(f"{elapsed:8.1f}  {beat_rate:12.0f}  {rate:12.0f}  {phy_mode:>6}  {tx_buf_rate:10.0f}  {irq_rate:10.0f}  "
                  f"{irq_per_buf:8.3f}  {rx_en:5d}  {rx_lvl:6d}  {rx_loops:9}  {rx_idx:7}  "
                  f"{tx_en:5d}  {tx_lvl:6d}  {tx_loops:9}  {tx_idx:7}  "
                  f"{late:6}  {underrun:8}  "
                  f"{hw_ns / 1e9:10.3f}  {status:>6}")

            prev_sc, prev_t = sc, now
            prev_irq = irq_cnt
            prev_tx_stat = tx_stat

            # Reprint header every 40 lines for readability
            line += 1
            if line % 40 == 0:
                print(sep)
                print(hdr)
                print(sep)

    except KeyboardInterrupt:
        pass
    finally:
        os.close(fd)


if __name__ == "__main__":
    main()
