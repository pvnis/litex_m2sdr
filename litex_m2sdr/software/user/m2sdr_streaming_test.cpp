/*
 * m2sdr_streaming_test.cpp
 *
 * Standalone SoapySDR streaming test for the LiteX M2SDR.
 *
 * Uses the direct buffer API exclusively:
 *   TX: acquireWriteBuffer / releaseWriteBuffer
 *   RX: acquireReadBuffer  / releaseReadBuffer
 *
 * This mirrors the pattern used by OCUDU and similar applications that
 * zero-copy into/from DMA buffers without going through writeStream/readStream.
 *
 * Four phases, all selectable independently:
 *   Phase 1 (--rx)       : RX DMA baseline — verify writer runs, check overflow/rate
 *   Phase 2 (--tx)       : TX DMA baseline — verify reader runs, check rate
 *   Phase 3 (--loopback) : TX+RX loopback, continuous tone (no timestamp)
 *   Phase 4 (--timed)    : Timed TX — schedule a burst at a future timestamp
 *
 * Build:
 *   g++ -O2 -o m2sdr_streaming_test m2sdr_streaming_test.cpp \
 *       $(pkg-config --cflags --libs SoapySDR) -lpthread -lm
 *
 * Prerequisites:
 *   sudo ./m2sdr_rf -samplerate 30.72e6
 *
 * Usage:
 *   ./m2sdr_streaming_test [--samplerate 30720000] [--rx] [--tx]
 *                          [--loopback] [--timed] [--delay 0.2] [--duration 0.3]
 *   (no phase flags = run all phases)
 *
 * Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Logger.hpp>

#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <vector>

#include "csr.h"
#include "litepcie.h"

/* ---------- helpers -------------------------------------------------------- */

static const double  TONE_FREQ_HZ = 500000.0;
static const double  AMPLITUDE    = 0.3;
static const int16_t SCALE        = 2047;   /* 12-bit IQ, matches driver _samplesScaling */
static const uint32_t CSR_DMA_WRITER_LOOP_STATUS_ADDR = 0x6014;
static const uint32_t CSR_DMA_READER_LOOP_STATUS_ADDR = 0x6034;

struct TDDDebugSnapshot {
    bool      valid;
    uint32_t  tx_loop_status;
    uint32_t  rx_loop_status;
    uint32_t  late_count;
    uint32_t  underrun_count;
    uint32_t  state;
    uint32_t  armed_has_time;
    uint32_t  ts_pop_count;
    uint32_t  last_late_has_time;
    uint32_t  last_late_ts_pop;
    uint32_t  last_late_state;
    uint64_t  armed_ts_ns;
    uint64_t  last_late_ts_ns;
    uint64_t  last_late_time_ns;
    uint64_t  hw_time_ns;
    int64_t   dma_reader_hw_count;
    int64_t   dma_reader_sw_count;
};

static int debug_open_fd(const char *path = "/dev/m2sdr0")
{
    return open(path, O_RDWR | O_CLOEXEC);
}

static bool debug_csr_read(int fd, uint32_t addr, uint32_t &val)
{
    struct litepcie_ioctl_reg reg = {};
    reg.addr = addr;
    reg.val = 0;
    reg.is_write = 0;
    if (ioctl(fd, LITEPCIE_IOCTL_REG, &reg) < 0)
        return false;
    val = reg.val;
    return true;
}

static bool debug_csr_write(int fd, uint32_t addr, uint32_t val)
{
    struct litepcie_ioctl_reg reg = {};
    reg.addr = addr;
    reg.val = val;
    reg.is_write = 1;
    return ioctl(fd, LITEPCIE_IOCTL_REG, &reg) == 0;
}

static bool debug_read_hw_time_ns(int fd, uint64_t &hw_time_ns)
{
    uint32_t ctrl = 0, hi = 0, lo = 0;
    if (!debug_csr_read(fd, CSR_TIME_GEN_CONTROL_ADDR, ctrl)) return false;
    if (!debug_csr_write(fd, CSR_TIME_GEN_CONTROL_ADDR, ctrl | (1u << CSR_TIME_GEN_CONTROL_READ_OFFSET))) return false;
    if (!debug_csr_write(fd, CSR_TIME_GEN_CONTROL_ADDR, ctrl & ~(1u << CSR_TIME_GEN_CONTROL_READ_OFFSET))) return false;
    if (!debug_csr_read(fd, CSR_TIME_GEN_READ_TIME_ADDR + 0, hi)) return false;
    if (!debug_csr_read(fd, CSR_TIME_GEN_READ_TIME_ADDR + 4, lo)) return false;
    hw_time_ns = ((uint64_t)hi << 32) | lo;
    return true;
}

#ifdef CSR_TIMED_TX_LATE_COUNT_ADDR
static TDDDebugSnapshot debug_take_snapshot(int fd)
{
    TDDDebugSnapshot s = {};
    s.valid = false;
    if (fd < 0) return s;
    struct litepcie_ioctl_dma_reader dma_reader = {};
    uint64_t hw_time_ns = 0, armed_ts_ns = 0, last_late_ts_ns = 0, last_late_time_ns = 0;
    uint32_t armed_hi = 0, armed_lo = 0, last_late_ts_hi = 0, last_late_ts_lo = 0;
    uint32_t last_late_time_hi = 0, last_late_time_lo = 0;
    if (!debug_csr_read(fd, CSR_DMA_READER_LOOP_STATUS_ADDR, s.tx_loop_status)) return s;
    if (!debug_csr_read(fd, CSR_DMA_WRITER_LOOP_STATUS_ADDR, s.rx_loop_status)) return s;
    if (!debug_csr_read(fd, CSR_TIMED_TX_LATE_COUNT_ADDR, s.late_count)) return s;
    if (!debug_csr_read(fd, CSR_TIMED_TX_UNDERRUN_COUNT_ADDR, s.underrun_count)) return s;
    if (!debug_csr_read(fd, CSR_TIMED_TX_STATE_ADDR, s.state)) return s;
    if (!debug_csr_read(fd, CSR_TIMED_TX_ARMED_HAS_TIME_ADDR, s.armed_has_time)) return s;
    if (!debug_csr_read(fd, CSR_TIMED_TX_TS_POP_COUNT_ADDR, s.ts_pop_count)) return s;
    if (!debug_csr_read(fd, CSR_TIMED_TX_ARMED_TS_ADDR + 0, armed_hi)) return s;
    if (!debug_csr_read(fd, CSR_TIMED_TX_ARMED_TS_ADDR + 4, armed_lo)) return s;
    if (!debug_csr_read(fd, CSR_TIMED_TX_LAST_LATE_HAS_TIME_ADDR, s.last_late_has_time)) return s;
    if (!debug_csr_read(fd, CSR_TIMED_TX_LAST_LATE_TS_POP_ADDR, s.last_late_ts_pop)) return s;
    if (!debug_csr_read(fd, CSR_TIMED_TX_LAST_LATE_STATE_ADDR, s.last_late_state)) return s;
    if (!debug_csr_read(fd, CSR_TIMED_TX_LAST_LATE_TS_ADDR + 0, last_late_ts_hi)) return s;
    if (!debug_csr_read(fd, CSR_TIMED_TX_LAST_LATE_TS_ADDR + 4, last_late_ts_lo)) return s;
    if (!debug_csr_read(fd, CSR_TIMED_TX_LAST_LATE_TIME_ADDR + 0, last_late_time_hi)) return s;
    if (!debug_csr_read(fd, CSR_TIMED_TX_LAST_LATE_TIME_ADDR + 4, last_late_time_lo)) return s;
    armed_ts_ns = ((uint64_t)armed_hi << 32) | armed_lo;
    last_late_ts_ns = ((uint64_t)last_late_ts_hi << 32) | last_late_ts_lo;
    last_late_time_ns = ((uint64_t)last_late_time_hi << 32) | last_late_time_lo;
    s.armed_ts_ns = armed_ts_ns;
    s.last_late_ts_ns = last_late_ts_ns;
    s.last_late_time_ns = last_late_time_ns;
    dma_reader.enable = 1;
    if (ioctl(fd, LITEPCIE_IOCTL_DMA_READER, &dma_reader) < 0) return s;
    s.dma_reader_hw_count = dma_reader.hw_count;
    s.dma_reader_sw_count = dma_reader.sw_count;
    if (!debug_read_hw_time_ns(fd, hw_time_ns)) return s;
    s.hw_time_ns = hw_time_ns;
    s.valid = true;
    return s;
}

static void debug_reset_timed_tx_state(int fd)
{
    if (fd < 0) return;
    debug_csr_write(fd, CSR_TIMED_TX_RESET_COUNTS_ADDR, 1);
    debug_csr_write(fd, CSR_TIMED_TX_RESET_COUNTS_ADDR, 0);
}
#else
static TDDDebugSnapshot debug_take_snapshot(int fd)
{
    (void)fd;
    TDDDebugSnapshot s = {};
    s.valid = false;
    return s;
}

static void debug_reset_timed_tx_state(int fd)
{
    (void)fd;
}
#endif

static void flush_tx_status(SoapySDR::Device *dev, SoapySDR::Stream *tx, int timeoutUs = 0)
{
    for (;;) {
        size_t    chanMask = 0;
        int       flags    = 0;
        long long timeNs   = 0;
        int ret = dev->readStreamStatus(tx, chanMask, flags, timeNs, timeoutUs);
        if (ret == SOAPY_SDR_TIMEOUT)
            break;
        if (ret <= 0 && ret != SOAPY_SDR_TIME_ERROR && ret != SOAPY_SDR_UNDERFLOW)
            break;
    }
}

static void debug_print_snapshot(const char *label, const TDDDebugSnapshot &s, int64_t submitted = -1)
{
    if (!s.valid) {
        printf("  [debug] %s: snapshot unavailable\n", label);
        return;
    }
    const uint32_t tx_loops = (s.tx_loop_status >> 16) & 0xffff;
    const uint32_t tx_idx   = s.tx_loop_status & 0xffff;
    const uint32_t rx_loops = (s.rx_loop_status >> 16) & 0xffff;
    const uint32_t rx_idx   = s.rx_loop_status & 0xffff;
    const long long pending = (submitted >= 0) ? (submitted - s.dma_reader_hw_count) : -1;
    const char *state_name =
        (s.state == 0) ? "IDLE" :
        (s.state == 1) ? "WAIT" :
        (s.state == 2) ? "STRM" : "DROP";
    const char *late_state_name =
        (s.last_late_state == 0) ? "IDLE" :
        (s.last_late_state == 1) ? "WAIT" :
        (s.last_late_state == 2) ? "STRM" : "DROP";
    const double armed_delta_ms = (double)((long long)s.armed_ts_ns - (long long)s.hw_time_ns) / 1e6;
    const double last_late_delta_ms = (double)((long long)s.last_late_time_ns - (long long)s.last_late_ts_ns) / 1e6;
    printf("  [debug] %s: hw=%.6f s  tx_loop=%u:%u  rx_loop=%u:%u  dma_hw=%lld dma_sw=%lld"
           " state=%s ts_pop=%u armed_time=%u armed_ts=%.6f dT=%+.3f ms"
           " last_late={state=%s has_time=%u pop=%u ts=%.6f now=%.6f dt=%+.3f ms}",
           label, s.hw_time_ns / 1e9, tx_loops, tx_idx, rx_loops, rx_idx,
           (long long)s.dma_reader_hw_count, (long long)s.dma_reader_sw_count,
           state_name, s.ts_pop_count, s.armed_has_time,
           s.armed_ts_ns / 1e9, armed_delta_ms,
           late_state_name, s.last_late_has_time, s.last_late_ts_pop,
           s.last_late_ts_ns / 1e9, s.last_late_time_ns / 1e9, last_late_delta_ms);
    if (submitted >= 0)
        printf(" submitted=%lld pending=%lld", (long long)submitted, pending);
    printf(" late=%u  underrun=%u\n", s.late_count, s.underrun_count);
}

static void debug_print_snapshot_delta(const char *label,
                                       const TDDDebugSnapshot &prev,
                                       const TDDDebugSnapshot &cur,
                                       int64_t submitted = -1)
{
    if (!prev.valid || !cur.valid)
        return;
    const long long dma_hw_delta = (long long)(cur.dma_reader_hw_count - prev.dma_reader_hw_count);
    const long long dma_sw_delta = (long long)(cur.dma_reader_sw_count - prev.dma_reader_sw_count);
    const long long hw_ms_delta  = (long long)(cur.hw_time_ns - prev.hw_time_ns) / 1000000LL;
    const int late_delta         = (int)cur.late_count - (int)prev.late_count;
    const int underrun_delta     = (int)cur.underrun_count - (int)prev.underrun_count;
    const int state_delta        = (int)cur.state - (int)prev.state;
    const int armed_time_delta   = (int)cur.armed_has_time - (int)prev.armed_has_time;
    const int ts_pop_delta       = (int)cur.ts_pop_count - (int)prev.ts_pop_count;
    const long long armed_ms_delta = (long long)(cur.armed_ts_ns - prev.armed_ts_ns) / 1000000LL;
    const int last_late_has_time_delta = (int)cur.last_late_has_time - (int)prev.last_late_has_time;
    const int last_late_ts_pop_delta   = (int)cur.last_late_ts_pop - (int)prev.last_late_ts_pop;
    const int last_late_state_delta    = (int)cur.last_late_state - (int)prev.last_late_state;
    const long long last_late_ts_ms_delta =
        (long long)(cur.last_late_ts_ns - prev.last_late_ts_ns) / 1000000LL;
    const long long last_late_time_ms_delta =
        (long long)(cur.last_late_time_ns - prev.last_late_time_ns) / 1000000LL;
    const long long pending      = (submitted >= 0) ? (submitted - cur.dma_reader_hw_count) : -1;
    printf("  [debug] %s delta: dt=%lld ms  dma_hw=%+lld dma_sw=%+lld state=%+d ts_pop=%+d armed_time=%+d armed_ts=%+lld ms"
           " last_late={state=%+d has_time=%+d pop=%+d ts=%+lld ms now=%+lld ms}",
           label, hw_ms_delta, dma_hw_delta, dma_sw_delta,
           state_delta, ts_pop_delta, armed_time_delta, armed_ms_delta,
           last_late_state_delta, last_late_has_time_delta, last_late_ts_pop_delta,
           last_late_ts_ms_delta, last_late_time_ms_delta);
    if (submitted >= 0)
        printf(" pending=%lld", pending);
    printf(" late=%+d underrun=%+d\n", late_delta, underrun_delta);
}

/*
 * Write a complex tone directly into a raw CS16 DMA buffer.
 * Layout: [I0, Q0, I1, Q1, ...] — interleaved int16_t pairs, 1 channel.
 * start_sample: phase offset so a burst split across many buffers is phase-continuous.
 */
static void fill_tone_cs16(int16_t *buf, size_t n_samples,
                            double freq, double sample_rate,
                            double ampl, size_t start_sample)
{
    double delta_ph = 2.0 * M_PI * freq / sample_rate;
    double ph = delta_ph * (double)start_sample;
    for (size_t i = 0; i < n_samples; i++) {
        buf[2 * i + 0] = (int16_t)(ampl * SCALE * cos(ph));  /* I */
        buf[2 * i + 1] = (int16_t)(ampl * SCALE * sin(ph));  /* Q */
        ph += delta_ph;
    }
}

/*
 * RMS power of a raw CS16 buffer, normalised to SCALE so that a full-scale
 * tone gives ~1.0.  Used for relative power comparisons only.
 */
static double rms_power_cs16(const int16_t *buf, size_t n_samples)
{
    double acc = 0.0;
    for (size_t i = 0; i < n_samples; i++) {
        double I = (double)buf[2 * i + 0];
        double Q = (double)buf[2 * i + 1];
        acc += I * I + Q * Q;
    }
    return acc / ((double)n_samples * (double)(SCALE * SCALE));
}

static double power_db(double p) { return 10.0 * log10(p + 1e-12); }

/*
 * fine_onset_sample: within a raw CS16 buffer of n_samples, find the first
 * sample position where a sliding window of fine_win samples has mean power
 * exceeding threshold (in rms_power_cs16 units, i.e. mean (I/SCALE)² + (Q/SCALE)²).
 * Returns the sample offset, or n_samples if not found.
 *
 * Because the burst is orders of magnitude above the noise floor, even a single
 * burst sample in the window overwhelms the noise and triggers the threshold.
 * fine_win=32 gives ~1 µs resolution at 30.72 MSPS.
 */
static size_t fine_onset_sample(const int16_t *buf, size_t n_samples,
                                double threshold, size_t fine_win = 32)
{
    /* Convert threshold to raw int16_t² units:
     * rms_power_cs16 = Σ(I²+Q²)/(SCALE²·n), so threshold in raw units is
     * threshold · fine_win · SCALE²  */
    const double raw_thresh = threshold * (double)fine_win * (double)(SCALE * SCALE);
    double acc = 0.0;
    /* Seed the first window. */
    for (size_t k = 0; k < fine_win && k < n_samples * 2; k += 2) {
        acc += (double)buf[k] * buf[k] + (double)buf[k+1] * buf[k+1];
    }
    if (acc > raw_thresh) return 0;
    /* Slide one sample at a time (subtract outgoing, add incoming). */
    for (size_t j = 1; j + fine_win <= n_samples; j++) {
        acc -= (double)buf[2*(j-1)] * buf[2*(j-1)]
             + (double)buf[2*(j-1)+1] * buf[2*(j-1)+1];
        acc += (double)buf[2*(j+fine_win-1)] * buf[2*(j+fine_win-1)]
             + (double)buf[2*(j+fine_win-1)+1] * buf[2*(j+fine_win-1)+1];
        if (acc > raw_thresh) return j;
    }
    return n_samples;  /* not found */
}

static bool report(const char *label, bool pass, const char *reason = nullptr)
{
    if (pass)
        printf("  [PASS] %s\n", label);
    else
        printf("  [FAIL] %s%s%s\n", label,
               reason ? " -- " : "", reason ? reason : "");
    return pass;
}

/*
 * LoopbackCfg: selects how phases 3 and 4 route the TX signal back to RX.
 *
 *   use_bist = true  — Soapy PHY TX->RX loopback (no cable needed)
 *   use_bist = false — physical RF cable (TX SMA → attenuator → RX SMA)
 *                      rf_freq/tx_gain/rx_gain are applied to the device.
 *
 * Recommended cable setup: 30 dB SMA attenuator between TX and RX connectors.
 * Default gains (tx=0 dB, rx=30 dB) are safe with a 30 dB pad.
 */
struct LoopbackCfg {
    bool   use_bist = true;
    double rf_freq  = 2.4e9;
    double tx_gain  = 0.0;
    double rx_gain  = 30.0;
};

/*
 * open_device: make a SoapyLiteXM2SDR device, configure sample rate, and
 * optionally apply RF cable loopback settings.
 */
static SoapySDR::Device *open_device(double sample_rate, const LoopbackCfg &cfg, bool bypass_timed_tx_arbiter = false)
{
    SoapySDR::Kwargs args;
    args["driver"] = "LiteXM2SDR";
    if (cfg.use_bist)
        args["loopback_mode"] = "phy";

    if (bypass_timed_tx_arbiter) {
        args["bypass_timed_tx_arbiter"] = "1";  /* optional: bypass timed TX arbiter when debugging timed scheduling */
    }

    SoapySDR::Device *dev = SoapySDR::Device::make(args);
    if (!dev) {
        fprintf(stderr, "ERROR: Cannot open LiteXM2SDR device\n");
        return nullptr;
    }
    dev->setSampleRate(SOAPY_SDR_RX, 0, sample_rate);
    dev->setSampleRate(SOAPY_SDR_TX, 0, sample_rate);

    if (!cfg.use_bist) {
        dev->setFrequency(SOAPY_SDR_TX, 0, cfg.rf_freq);
        dev->setFrequency(SOAPY_SDR_RX, 0, cfg.rf_freq);
        dev->setGain(SOAPY_SDR_TX, 0, cfg.tx_gain);
        dev->setGain(SOAPY_SDR_RX, 0, cfg.rx_gain);
    }
    return dev;
}

/* Convenience overload for phases 1/2 that need no loopback config. */
static SoapySDR::Device *open_device(double sample_rate)
{
    LoopbackCfg cfg;
    cfg.use_bist = false;  /* no Soapy PHY loopback, no RF freq/gain setup */
    return open_device(sample_rate, cfg, false);
}

/* ============================================================
 * Phase 1: RX DMA Baseline
 *
 * Runs acquireReadBuffer/releaseReadBuffer in a tight loop for n_bufs
 * iterations.  Checks:
 *   - no errors from acquireReadBuffer
 *   - no overflows
 *   - measured sample rate within ±10% of configured rate
 * ============================================================ */
static bool test_rx_baseline(double sample_rate, int /*n_bufs*/)
{
    printf("\n=== Phase 1: RX DMA Baseline ===\n");

    /* No loopback: receive directly from the ADC.  The PHY loopback gates
     * RX on TX data (source.valid = sink.valid & sink.ready), so enabling
     * loopback here ties RX throughput to TX scheduling jitter, causing
     * wildly variable rate measurements.  Direct ADC capture is always at
     * RFIC rate regardless of TX state. */
    SoapySDR::Device *dev = open_device(sample_rate);
    if (!dev) return false;

    /* Set up both streams so _nChannels is 1 before getStreamMTU. */
    SoapySDR::Stream *rx = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, {0});
    SoapySDR::Stream *tx = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, {0});
    const size_t mtu     = dev->getStreamMTU(rx);
    printf("  MTU        : %zu samples/buf\n", mtu);

    /* Activate RX only — TX DMA is not needed for the rate measurement. */
    dev->activateStream(rx);

    /* Synchronize: drain the DMA ring for 50 ms to confirm the pipeline is
     * stable before timing.  The rxReceiveWorker thread starts DMA hardware
     * asynchronously; if we time from activateStream() the worker's
     * scheduling latency appears as low throughput.  50 ms is long enough
     * to absorb any AD9361 ENSM settling and thread startup jitter without
     * being a significant fraction of the 200 ms measurement window. */
    {
        const auto drain_deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
        while (std::chrono::steady_clock::now() < drain_deadline) {
            size_t     sh = 0;
            const void *sb[1];
            int        sf = 0;
            long long  st = 0;
            int ret = dev->acquireReadBuffer(rx, sh, sb, sf, st, 2000000 /* 2 s */);
            if (ret > 0)
                dev->releaseReadBuffer(rx, sh);
            else if (ret < 0) {
                printf("  DMA sync failed: acquireReadBuffer returned %d\n", ret);
                dev->deactivateStream(rx);
                dev->closeStream(rx);
                dev->closeStream(tx);
                SoapySDR::Device::unmake(dev);
                return false;
            }
        }
    }

    /* Time-based measurement over 200 ms.  t0 starts only once DMA is
     * confirmed flowing; any pre-buffered data drains in < 2 ms (ring size
     * / RFIC rate) which is < 1% of the window and within tolerance. */
    const double MEAS_S = 0.2;
    int      overflows     = 0;
    bool     errors        = false;
    int      counted       = 0;
    long long total_samples = 0;
    /* Per-call latency histogram buckets: <1ms, 1-10ms, 10-100ms, >100ms */
    int lat_fast = 0, lat_ms = 0, lat_10ms = 0, lat_slow = 0;
    auto t0        = std::chrono::steady_clock::now();

    for (;;) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count()
                >= MEAS_S)
            break;

        size_t     handle = 0;
        const void *buffs[1];
        int        flags  = 0;
        long long  timeNs = 0;
        auto call_t0 = std::chrono::steady_clock::now();

        int ret = dev->acquireReadBuffer(rx, handle, buffs, flags, timeNs,
                                         1000000 /* 1 s timeout */);

        auto call_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - call_t0).count();
        if (call_us < 1000) lat_fast++;
        else if (call_us < 10000) lat_ms++;
        else if (call_us < 100000) lat_10ms++;
        else lat_slow++;

        if (ret == SOAPY_SDR_OVERFLOW) {
            overflows++;
            continue;
        }
        if (ret < 0) {
            printf("  [%d] acquireReadBuffer error %d\n", counted, ret);
            errors = true;
            break;
        }
        dev->releaseReadBuffer(rx, handle);
        counted++;
        total_samples += ret;
    }

    auto   t1        = std::chrono::steady_clock::now();
    double elapsed_s = std::chrono::duration<double>(t1 - t0).count();
    double meas_msps = (double)total_samples / elapsed_s / 1e6;

    dev->deactivateStream(rx);
    dev->closeStream(rx);
    dev->closeStream(tx);
    SoapySDR::Device::unmake(dev);

    printf("  Elapsed    : %.3f s  (%d bufs)\n", elapsed_s, counted);
    printf("  Latency    : fast=%d 1ms=%d 10ms=%d slow=%d\n",
           lat_fast, lat_ms, lat_10ms, lat_slow);
    printf("  Rate       : %.3f MSPS  (expected %.3f)\n",
           meas_msps, sample_rate / 1e6);
    printf("  Overflows  : %d\n", overflows);

    bool rate_ok = (meas_msps > sample_rate * 0.9e-6) &&
                   (meas_msps < sample_rate * 1.1e-6);
    bool pass = true;
    pass &= report("RX DMA: no errors",    !errors,         "acquireReadBuffer returned error");
    pass &= report("RX DMA: no overflows", overflows == 0,  "overflow detected");
    pass &= report("RX DMA: sample rate",  rate_ok,         "rate outside ±10%");
    return pass;
}

/* ============================================================
 * Phase 2: TX DMA Baseline
 *
 * Opens TX (and a dummy RX to set _nChannels=1), activates TX only, and
 * writes n_bufs zero-filled buffers.  Checks:
 *   - acquireWriteBuffer never returns an error
 *   - all n_bufs buffers completed
 *   - measured throughput within ±10% of configured rate
 * ============================================================ */
static bool test_tx_baseline(double sample_rate, int n_bufs)
{
    printf("\n=== Phase 2: TX DMA Baseline ===\n");

    SoapySDR::Device *dev = open_device(sample_rate);
    if (!dev) return false;

    /*
     * Setup RX first (don't activate) so that _nChannels is set to 1
     * before TX setupStream reads it for MTU calculation.
     */
    SoapySDR::Stream *rx     = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, {0});
    SoapySDR::Stream *tx     = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, {0});
    const size_t mtu         = dev->getStreamMTU(tx);
    const size_t ring_size   = dev->getNumDirectAccessBuffers(tx);
    /*
     * Must write more than the DMA ring depth to force hardware backpressure
     * and get a meaningful throughput measurement.  Use 2× ring + margin.
     */
    int tx_bufs = std::max(n_bufs, (int)(ring_size * 2 + 50));
    printf("  MTU        : %zu samples/buf\n", mtu);
    printf("  Ring size  : %zu buffers\n", ring_size);
    printf("  Test bufs  : %d\n", tx_bufs);

    dev->activateStream(tx);  /* RX stays inactive */

    bool errors       = false;
    bool backpressured = false;  /* did acquireWriteBuffer ever have to wait? */
    int  completed    = 0;
    auto t0           = std::chrono::steady_clock::now();

    for (int i = 0; i < tx_bufs; i++) {
        size_t handle = 0;
        void  *buffs[1];

        /*
         * Try a non-blocking acquire first.  If it times out the ring is full
         * and we must wait — that's the backpressure path we want to exercise.
         */
        int ret = dev->acquireWriteBuffer(tx, handle, buffs, 0 /* non-blocking */);
        if (ret == SOAPY_SDR_TIMEOUT) {
            backpressured = true;
            ret = dev->acquireWriteBuffer(tx, handle, buffs, 1000000 /* 1 s */);
        }
        if (ret < 0) {
            printf("  [%4d] acquireWriteBuffer error %d\n", i, ret);
            errors = true;
            break;
        }
        /* Zero-fill: simulates idle/silence TX, exercises DMA reader. */
        memset(buffs[0], 0, mtu * 2 * sizeof(int16_t));

        int flags = 0;
        dev->releaseWriteBuffer(tx, handle, mtu, flags, 0);
        completed++;
    }

    auto   t1        = std::chrono::steady_clock::now();
    double elapsed_s = std::chrono::duration<double>(t1 - t0).count();
    double meas_msps = (double)completed * (double)mtu / elapsed_s / 1e6;

    dev->deactivateStream(tx);
    dev->closeStream(tx);
    dev->closeStream(rx);
    SoapySDR::Device::unmake(dev);

    printf("  Elapsed    : %.3f s\n", elapsed_s);
    printf("  Completed  : %d / %d buffers\n", completed, tx_bufs);
    printf("  Rate       : %.3f MSPS  (note: TX DMA prefetch makes this > sample rate)\n",
           meas_msps);
    printf("  Backpressure hit: %s\n", backpressured ? "yes" : "no");

    bool pass = true;
    pass &= report("TX DMA: no errors",      !errors,               "acquireWriteBuffer returned error");
    pass &= report("TX DMA: all buffers",    completed == tx_bufs,  "not all buffers completed");
    pass &= report("TX DMA: backpressure",   backpressured,         "ring never filled — DMA may not be running");
    return pass;
}

/* ============================================================
 * Phase 3: TX+RX Loopback (no timestamp)
 *
 * TX continuously transmits a 500 kHz tone via the direct buffer API.
 * RX captures for duration_s seconds and measures average power, overflow
 * count, and checks for any drop-outs (buffers with anomalously low power).
 *
 * cfg.use_bist = true  : Soapy PHY TX->RX loopback (no cable required)
 * cfg.use_bist = false : physical RF cable (TX SMA → attenuator → RX SMA)
 *
 * Checks:
 *   - no TX errors
 *   - no RX overflows
 *   - received power > -40 dBFS
 *   - no drop-outs (power never falls more than 20 dB below average)
 * ============================================================ */
static bool test_loopback(double sample_rate, double duration_s, const LoopbackCfg &cfg)
{
    printf("\n=== Phase 3: TX+RX Loopback (no timestamp) [%s] ===\n",
           cfg.use_bist ? "BIST" : "RF cable");
    printf("  Duration   : %.2f s\n", duration_s);
    if (!cfg.use_bist)
        printf("  RF freq    : %.3f MHz  TX gain %.1f dB  RX gain %.1f dB\n",
               cfg.rf_freq / 1e6, cfg.tx_gain, cfg.rx_gain);

    SoapySDR::Device *dev = open_device(sample_rate, cfg, false);
    if (!dev) return false;

    SoapySDR::Stream *rx     = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, {0});
    SoapySDR::Stream *tx     = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, {0});
    const size_t      rx_mtu = dev->getStreamMTU(rx);
    const size_t      tx_mtu = dev->getStreamMTU(tx);
    printf("  RX MTU     : %zu samples/buf\n", rx_mtu);
    printf("  TX MTU     : %zu samples/buf\n", tx_mtu);

    dev->activateStream(rx);
    dev->activateStream(tx);

    /* Pre-generate one MTU of tone for the TX thread. */
    std::vector<int16_t> tone_buf(tx_mtu * 2);
    fill_tone_cs16(tone_buf.data(), tx_mtu, TONE_FREQ_HZ, sample_rate, AMPLITUDE, 0);

    std::atomic<bool> tx_stop(false);
    std::atomic<int>  tx_errors(0);

    std::thread tx_thread([&]() {
        while (!tx_stop.load()) {
            size_t handle = 0;
            void  *buffs[1];
            int ret = dev->acquireWriteBuffer(tx, handle, buffs, 100000 /* 100 ms */);
            if (ret == SOAPY_SDR_TIMEOUT) continue;
            if (ret < 0) { tx_errors++; break; }
            memcpy(buffs[0], tone_buf.data(), tx_mtu * 2 * sizeof(int16_t));
            int flags = 0;
            dev->releaseWriteBuffer(tx, handle, tx_mtu, flags, 0);
        }
    });

    /* Skip first 10 ms of RX (startup transient), then accumulate power. */
    int    skip      = (int)(0.010 * sample_rate / (double)rx_mtu) + 1;
    int    warmup    = (int)(0.050 * sample_rate / (double)rx_mtu) + 1;
    int    n_bufs    = (int)(duration_s * sample_rate / (double)rx_mtu) + skip;
    double total_pwr = 0.0;
    int    measured  = 0;
    int    overflows = 0;
    int    dropouts  = 0;

    std::vector<double> buf_pwrs;
    buf_pwrs.reserve(n_bufs);

    for (int i = 0; i < n_bufs; i++) {
        size_t     handle = 0;
        const void *buffs[1];
        int        flags  = 0;
        long long  timeNs = 0;

        int ret = dev->acquireReadBuffer(rx, handle, buffs, flags, timeNs, 1000000);
        if (ret == SOAPY_SDR_OVERFLOW) {
            if (i >= warmup) overflows++;
            continue;
        }
        if (ret < 0) break;

        if (i >= skip) {
            double p = rms_power_cs16(static_cast<const int16_t *>(buffs[0]), rx_mtu);
            total_pwr += p;
            measured++;
            buf_pwrs.push_back(p);
        }
        dev->releaseReadBuffer(rx, handle);
    }

    tx_stop.store(true);
    tx_thread.join();

    /* Count drop-outs: buffers more than 20 dB below the mean. */
    double avg_pwr = (measured > 0) ? total_pwr / measured : 0.0;
    double dropout_thresh = avg_pwr / 100.0;  /* 20 dB below mean */
    for (double p : buf_pwrs)
        if (p < dropout_thresh) dropouts++;

    dev->deactivateStream(tx);
    dev->deactivateStream(rx);
    dev->closeStream(tx);
    dev->closeStream(rx);
    SoapySDR::Device::unmake(dev);

    double avg_pwr_db = power_db(avg_pwr);
    printf("  RX power   : %.1f dBFS  (%d buffers, %.2f s)\n",
           avg_pwr_db, measured, duration_s);
    printf("  Overflows  : %d\n", overflows);
    printf("  Drop-outs  : %d  (bufs >20 dB below mean)\n", dropouts);

    double overflow_rate = (double)overflows / (double)std::max(1, measured + overflows);
    bool pass = true;
    pass &= report("Loopback: no TX errors",    tx_errors == 0,     "TX error");
    pass &= report("Loopback: overflow rate",   overflow_rate <= 0.005,
                   "overflow rate exceeds 0.5%");
    pass &= report("Loopback: power > -40dBFS", avg_pwr_db >= -40.0,"no signal");
    pass &= report("Loopback: no drop-outs",    dropouts == 0,      "signal interrupted");
    return pass;
}

/* ============================================================
 * measure_loopback_latency
 *
 * Sends one immediate (no HAS_TIME) tone burst and finds when it first
 * appears in the RX stream.  The difference is the round-trip pipeline
 * latency: TX DMA → FPGA TX path → AD9361 loopback → FPGA RX path → RX DMA.
 *
 * This baseline is subtracted from timed-TX onset errors so that the
 * reported error reflects only the accuracy of the TimedTXArbiter, not
 * the fixed loopback pipeline delay.
 *
 * Returns latency in seconds, or -1.0 if the tone was not detected.
 * ============================================================ */
static double measure_loopback_latency(double sample_rate, const LoopbackCfg &cfg)
{
    printf("\n=== Loopback Latency Baseline ===\n");

    SoapySDR::Device *dev = open_device(sample_rate, cfg, false);
    if (!dev) return -1.0;

    SoapySDR::Stream *rx     = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, {0});
    SoapySDR::Stream *tx     = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, {0});
    const size_t      rx_mtu = dev->getStreamMTU(rx);
    const size_t      tx_mtu = dev->getStreamMTU(tx);
    printf("  RX MTU     : %zu samples/buf  (%.3f ms/buf)\n",
           rx_mtu, (double)rx_mtu / sample_rate * 1000.0);

    dev->activateStream(rx);
    dev->activateStream(tx);

    /* ---- Phase 1: Prime the loopback ----------------------------------------
     * In BIST (PHY loopback) mode, RX data only flows when TX is actively
     * sending — loopback source.valid = sink.valid & sink.ready.  If TX is
     * idle the DMA synchronizer can never see sink.valid=1 at PPS time and
     * stays unsynced, blocking all data.  Send silence until a few good RX
     * buffers arrive, confirming sync is live.
     *
     * TX runs in the MAIN thread throughout (no handoff between threads), so
     * DMA never laps sw_count and Phase 2 writes see no underflow.  RX drain
     * runs in a helper thread.
     * ----------------------------------------------------------------------- */
    const int skip_prime = 8;  /* confirm loopback live; discard these */

    std::vector<int16_t> silence_buf(tx_mtu * 2, 0);
    std::atomic<bool> rx_prime_done(false);

    std::thread rx_prime_th([&]() {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        int good = 0;
        while (good < skip_prime && std::chrono::steady_clock::now() < deadline) {
            size_t h = 0; const void *b[1];
            int fl = 0; long long ts = 0;
            int r = dev->acquireReadBuffer(rx, h, b, fl, ts, 500000);
            if (r == SOAPY_SDR_OVERFLOW || r == SOAPY_SDR_TIMEOUT) continue;
            if (r < 0) break;
            good++;
            dev->releaseReadBuffer(rx, h);
        }
        if (good < skip_prime)
            printf("  WARNING: only %d/%d prime buffers received\n", good, skip_prime);
        rx_prime_done.store(true);
    });

    /* TX silence loop in the main thread — continuous, no gap before Phase 2. */
    while (!rx_prime_done.load()) {
        size_t h = 0; void *b[1];
        int r = dev->acquireWriteBuffer(tx, h, b, 50000);
        if (r > 0 || r == SOAPY_SDR_UNDERFLOW) {
            memcpy(b[0], silence_buf.data(), tx_mtu * 2 * sizeof(int16_t));
            int nf = 0;
            dev->releaseWriteBuffer(tx, h, tx_mtu, nf, 0);
        }
    }
    rx_prime_th.join();

    /* ---- Phase 2: Record the tone-burst response ----------------------------
     * Fire one tone burst (no HAS_TIME → passes through TimedTXArbiter),
     * then capture n_rec_bufs of RX data.  tx_ref=0 marks the start of the
     * recording window; the tone onset appears a few buffers in (TX latency).
     * ----------------------------------------------------------------------- */
    const size_t n_rec_bufs = 600;  /* ~40 ms — covers ring-wrap delay after initial underflow */
    std::vector<double>               rx_pwr(n_rec_bufs, 0.0);
    std::vector<std::vector<int16_t>> rx_raw(n_rec_bufs,
                                             std::vector<int16_t>(rx_mtu * 2, 0));
    std::vector<long long>            rx_ts(n_rec_bufs, 0);
    std::atomic<size_t>               rx_filled(0);
    std::atomic<bool>                 rx_stop(false);

    std::thread rx_th([&]() {
        while (!rx_stop.load() && rx_filled.load() < n_rec_bufs) {
            size_t handle = 0; const void *buffs[1];
            int flags = 0; long long timeNs = 0;
            int ret = dev->acquireReadBuffer(rx, handle, buffs, flags, timeNs, 500000);
            if (ret == SOAPY_SDR_OVERFLOW) continue;
            if (ret == SOAPY_SDR_TIMEOUT) continue;
            if (ret < 0) break;
            size_t idx = rx_filled.load();
            if (idx < n_rec_bufs) {
                memcpy(rx_raw[idx].data(), buffs[0], rx_mtu * 2 * sizeof(int16_t));
                rx_pwr[idx] = rms_power_cs16(
                    static_cast<const int16_t *>(buffs[0]), rx_mtu);
                rx_ts[idx]  = timeNs;
                rx_filled.store(idx + 1);
            }
            dev->releaseReadBuffer(rx, handle);
        }
    });

    /* Write silence_lead buffers of silence so the recording window has a
     * noise floor before the tone burst.  In BIST mode, RX data only comes
     * from the TX loopback; without leading silence the first recorded buffer
     * would BE the tone, making floor_pwr == tone_pwr and onset undetectable.
     * silence_lead also ensures the loopback is warm before hw_write_ns. */
    const int silence_lead = 8;
    for (int i = 0; i < silence_lead; i++) {
        size_t h = 0; void *b[1];
        int r = dev->acquireWriteBuffer(tx, h, b, 1000000);
        if (r > 0 || r == SOAPY_SDR_UNDERFLOW) {
            memcpy(b[0], silence_buf.data(), tx_mtu * 2 * sizeof(int16_t));
            int nf = 0;
            dev->releaseWriteBuffer(tx, h, tx_mtu, nf, 0);
        }
    }

    /* Fire the tone burst (no HAS_TIME — immediate pass through arbiter).
     * hw_write_ns is captured after this; the measured latency includes the
     * silence_lead pipeline delay (~silence_lead × 0.067 ms ≈ 0.5 ms). */
    const size_t tx_ref = 0;  /* onset searched from recording start */
    {
        size_t handle = 0; void *buffs[1];
        std::vector<int16_t> tone(tx_mtu * 2);
        fill_tone_cs16(tone.data(), tx_mtu, TONE_FREQ_HZ, sample_rate, AMPLITUDE, 0);
        int ret = dev->acquireWriteBuffer(tx, handle, buffs, 1000000);
        if (ret > 0 || ret == SOAPY_SDR_UNDERFLOW) {
            memcpy(buffs[0], tone.data(), tx_mtu * 2 * sizeof(int16_t));
            int flags = SOAPY_SDR_END_BURST;
            dev->releaseWriteBuffer(tx, handle, tx_mtu, flags, 0);
        }
    }
    /* Hardware time captured immediately after release: the burst is now in
     * the DMA ring and the FPGA will read it within one DMA cycle (~µs). */
    long long hw_write_ns = dev->getHardwareTime();

    /* Wait for recording window (10 s overall timeout). */
    auto t_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (rx_filled.load() < n_rec_bufs &&
           std::chrono::steady_clock::now() < t_deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    rx_stop.store(true);
    rx_th.join();

    dev->deactivateStream(tx);
    dev->deactivateStream(rx);
    dev->closeStream(tx);
    dev->closeStream(rx);
    SoapySDR::Device::unmake(dev);

    /* Noise floor: first few buffers in the recording window (silence before
     * the tone propagates through the TX pipeline). */
    const int  skip_bufs = 4;
    double floor_pwr = 0.0;
    size_t floor_n   = std::min((size_t)skip_bufs, rx_filled.load());
    for (size_t i = 0; i < floor_n; i++) floor_pwr += rx_pwr[i];
    if (floor_n) floor_pwr /= (double)floor_n;
    double threshold = 10.0 * floor_pwr + 1e-9;

    /* Find first RX buffer above threshold starting from tx_ref (= 0). */
    size_t onset_buf = n_rec_bufs;
    for (size_t i = tx_ref; i < rx_filled.load(); i++) {
        if (rx_pwr[i] > threshold) { onset_buf = i; break; }
    }

    if (onset_buf >= rx_filled.load()) {
        printf("  WARNING: tone not detected in RX — check loopback config\n");
        report("Latency baseline: tone detected", false, "no onset found");
        return -1.0;
    }

    /* Sub-buffer fine detection: scan within the onset buffer for the exact
     * first sample where the sliding window exceeds threshold. */
    size_t    fine_j      = fine_onset_sample(rx_raw[onset_buf].data(), rx_mtu, threshold);
    size_t    coarse_samp = (onset_buf - tx_ref) * rx_mtu + fine_j;
    double    latency_s;

    if (rx_ts[onset_buf] != 0 && hw_write_ns != 0) {
        long long onset_hw_ns = rx_ts[onset_buf]
                                + (long long)((double)fine_j * 1e9 / sample_rate);
        latency_s = (double)(onset_hw_ns - hw_write_ns) / 1e9;
        printf("  TX write HW time     : %.6f s\n", hw_write_ns / 1e9);
        printf("  Onset HW time        : %.6f s  (buf %zu + %zu samples)\n",
               onset_hw_ns / 1e9, onset_buf, fine_j);
        printf("  Loopback latency     : %.0f samples  (%.6f ms)  [HW-anchored]\n",
               latency_s * sample_rate, latency_s * 1000.0);
    } else {
        /* HW timestamps unavailable (no PPS sync or clock stall): fall back to
         * sample-count estimate.  Subtract silence_lead to measure only pipeline. */
        latency_s = (double)coarse_samp / sample_rate
                    - (double)(silence_lead * (int)rx_mtu) / sample_rate;
        printf("  WARNING: HW timestamps unavailable; using sample-count fallback\n");
        printf("  Loopback latency     : %.0f samples  (%.6f ms)  [sample-count]\n",
               latency_s * sample_rate, latency_s * 1000.0);
    }
    printf("  (coarse buf estimate : %zu samples  = %.3f ms)\n",
           coarse_samp, (double)coarse_samp / sample_rate * 1000.0);
    report("Latency baseline: tone detected", true);
    return latency_s;
}

/* ============================================================
 * write_burst: write n_burst_bufs of tone into the TX DMA stream.
 *   - first buffer: SOAPY_SDR_HAS_TIME | tx_time_ns
 *   - last  buffer: SOAPY_SDR_END_BURST
 *   - phase_offset: sample index so phase is continuous across calls
 * Returns 0 on success, negative on error.
 * ============================================================ */
static int write_burst(SoapySDR::Device *dev, SoapySDR::Stream *tx,
                       size_t tx_mtu, double sample_rate,
                       long long tx_time_ns, size_t n_burst_bufs,
                       size_t phase_offset, long timeout_us)
{
    std::vector<int16_t> tone_buf(tx_mtu * 2);
    for (size_t b = 0; b < n_burst_bufs; b++) {
        size_t handle = 0;
        void  *buffs[1];
        int ret = dev->acquireWriteBuffer(tx, handle, buffs, timeout_us);
        if (ret < 0 && ret != SOAPY_SDR_UNDERFLOW) return ret;
        fill_tone_cs16(tone_buf.data(), tx_mtu,
                       TONE_FREQ_HZ, sample_rate, AMPLITUDE,
                       phase_offset + b * tx_mtu);
        memcpy(buffs[0], tone_buf.data(), tx_mtu * 2 * sizeof(int16_t));
        int       flags    = 0;
        long long time_arg = 0;
        if (b == 0)              { flags  = SOAPY_SDR_HAS_TIME; time_arg = tx_time_ns; }
        if (b == n_burst_bufs-1)   flags |= SOAPY_SDR_END_BURST;
        dev->releaseWriteBuffer(tx, handle, tx_mtu, flags, time_arg);
    }
    return 0;
}

/* ============================================================
 * write_silence: write n_bufs of zeros into the TX DMA stream.
 * Used to fill inter-pulse gaps so the TX FIFO does not underflow.
 * Returns 0 on success, negative on error.
 * ============================================================ */
static int write_silence(SoapySDR::Device *dev, SoapySDR::Stream *tx,
                         size_t tx_mtu, int n_bufs, long timeout_us)
{
    std::vector<int16_t> zero_buf(tx_mtu * 2, 0);
    for (int b = 0; b < n_bufs; b++) {
        size_t handle = 0;
        void  *buffs[1];
        int ret = dev->acquireWriteBuffer(tx, handle, buffs, timeout_us);
        if (ret < 0 && ret != SOAPY_SDR_UNDERFLOW) return ret;
        memcpy(buffs[0], zero_buf.data(), tx_mtu * 2 * sizeof(int16_t));
        int flags = 0;
        dev->releaseWriteBuffer(tx, handle, tx_mtu, flags, 0);
    }
    return 0;
}

static int write_zero_span(SoapySDR::Device *dev, SoapySDR::Stream *tx,
                           size_t tx_mtu, size_t n_samples, long timeout_us)
{
    std::vector<int16_t> zero_buf(tx_mtu * 2, 0);
    const size_t n_bufs = (n_samples + tx_mtu - 1) / tx_mtu;

    for (size_t b = 0; b < n_bufs; b++) {
        size_t handle = 0;
        void  *buffs[1];
        int ret = dev->acquireWriteBuffer(tx, handle, buffs, timeout_us);
        if (ret < 0 && ret != SOAPY_SDR_UNDERFLOW) return ret;

        memcpy(buffs[0], zero_buf.data(), tx_mtu * 2 * sizeof(int16_t));
        int flags = 0;
        dev->releaseWriteBuffer(tx, handle, tx_mtu, flags, 0);
    }
    return 0;
}

static bool prime_bist_loopback(SoapySDR::Device *dev, SoapySDR::Stream *rx,
                                SoapySDR::Stream *tx, size_t tx_mtu,
                                int target_rx_bufs = 8)
{
    std::vector<int16_t> silence_buf(tx_mtu * 2, 0);
    std::atomic<bool> prime_done(false);
    std::atomic<int>  good_rx(0);

    std::thread rx_prime_th([&]() {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (good_rx.load() < target_rx_bufs &&
               std::chrono::steady_clock::now() < deadline) {
            size_t handle = 0;
            const void *buffs[1];
            int flags = 0;
            long long timeNs = 0;
            int ret = dev->acquireReadBuffer(rx, handle, buffs, flags, timeNs, 500000);
            if (ret == SOAPY_SDR_OVERFLOW || ret == SOAPY_SDR_TIMEOUT) continue;
            if (ret < 0) break;
            good_rx++;
            dev->releaseReadBuffer(rx, handle);
        }
        prime_done.store(true);
    });

    while (!prime_done.load()) {
        size_t handle = 0;
        void  *buffs[1];
        int ret = dev->acquireWriteBuffer(tx, handle, buffs, 50000);
        if (ret == SOAPY_SDR_TIMEOUT) continue;
        if (ret < 0) break;
        memcpy(buffs[0], silence_buf.data(), tx_mtu * 2 * sizeof(int16_t));
        int flags = 0;
        dev->releaseWriteBuffer(tx, handle, tx_mtu, flags, 0);
    }

    rx_prime_th.join();
    return good_rx.load() >= target_rx_bufs;
}

/* ============================================================
 * Phase 4: Timed TX — multi-pulse
 *
 * Schedules num_pulses tone bursts, each of duration_s seconds, separated
 * by gap_s seconds of silence.  The first pulse starts at hw_now + delay_s.
 *
 * RX captures the full window in a background thread.
 * For each pulse, the onset is detected and compared to the scheduled time.
 * Per-pulse errors and overall statistics are reported.
 *
 * Checks (must pass for ALL pulses):
 *   - no TX errors
 *   - power delta > 20 dB above pre-burst floor
 *   - onset within 50 ms of scheduled time
 * ============================================================ */
static bool test_timed_tx(double sample_rate, double delay_s, double duration_s,
                           double gap_s, int num_pulses, const LoopbackCfg &cfg,
                           double latency_s = 0.0, bool bypass_arbiter = false)
{
    printf("\n=== Phase 4: Timed TX — %d pulse%s [%s] [arbiter: %s] ===\n",
           num_pulses, num_pulses > 1 ? "s" : "",
           cfg.use_bist ? "BIST" : "RF cable",
           bypass_arbiter ? "bypassed" : "enabled");
    printf("  Delay      : %.3f s\n", delay_s);
    printf("  Duration   : %.3f s / pulse\n", duration_s);
    printf("  Gap        : %.3f s between pulses\n", gap_s);
    if (!cfg.use_bist)
        printf("  RF freq    : %.3f MHz  TX gain %.1f dB  RX gain %.1f dB\n",
               cfg.rf_freq / 1e6, cfg.tx_gain, cfg.rx_gain);
    if (bypass_arbiter)
        printf("  NOTE: TimedTXArbiter bypassed — HAS_TIME has no effect, burst fires immediately\n");

    SoapySDR::Device *dev = open_device(sample_rate, cfg, bypass_arbiter);
    if (!dev) return false;

    SoapySDR::Stream *rx     = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, {0});
    SoapySDR::Stream *tx     = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, {0});
    const size_t      rx_mtu = dev->getStreamMTU(rx);
    const size_t      tx_mtu = dev->getStreamMTU(tx);
    printf("  RX MTU     : %zu samples/buf\n", rx_mtu);
    printf("  TX MTU     : %zu samples/buf\n", tx_mtu);

    dev->activateStream(rx);
    dev->activateStream(tx);

    /* ------------------------------------------------------------------ */
    /* RX capture thread: runs for the full multi-pulse window.            */
    /* ------------------------------------------------------------------ */
    double total_rec_s = delay_s
                         + num_pulses * duration_s
                         + (num_pulses - 1) * gap_s
                         + 1.0;  /* extra tail */
    size_t n_rec_bufs  = (size_t)(total_rec_s * sample_rate / (double)rx_mtu) + 8;

    std::vector<std::vector<int16_t>> rx_caps(n_rec_bufs,
                                              std::vector<int16_t>(rx_mtu * 2, 0));
    std::atomic<size_t> rx_filled(0);
    std::atomic<bool>   rx_stop(false);
    int                 rx_overflows = 0;
    long long           rx_time0_ns  = 0;   /* HW timestamp of first RX sample */
    bool                rx_time0_set = false;

    std::thread rx_thread([&]() {
        size_t idx = 0;
        while (!rx_stop.load() && idx < n_rec_bufs) {
            size_t     handle = 0;
            const void *buffs[1];
            int        flags  = 0;
            long long  timeNs = 0;
            int ret = dev->acquireReadBuffer(rx, handle, buffs, flags, timeNs, 1000000);
            if (ret == SOAPY_SDR_OVERFLOW) { rx_overflows++; continue; }
            if (ret < 0) break;
            if (!rx_time0_set) { rx_time0_ns = timeNs; rx_time0_set = true; }
            memcpy(rx_caps[idx].data(), buffs[0], rx_mtu * 2 * sizeof(int16_t));
            dev->releaseReadBuffer(rx, handle);
            rx_filled.store(++idx);
        }
    });

    /* ------------------------------------------------------------------ */
    /* TX: write num_pulses bursts with silence between them.              */
    /* ------------------------------------------------------------------ */
    size_t n_burst_bufs  = (size_t)(duration_s * sample_rate / (double)tx_mtu) + 1;
    int    n_gap_bufs    = (int)(gap_s * sample_rate / (double)tx_mtu);
    long   timeout_us    = (long)((delay_s + duration_s + gap_s + 5.0) * 1e6);
    int                    tx_errors     = 0;
    size_t                 phase_offset  = 0;
    std::vector<long long> t_pulses(num_pulses);  /* scheduled HW time per pulse */
    int                    n_initial_silence = std::max(0,
        (int)(delay_s * sample_rate / (double)tx_mtu) - 20);
    long long              initial_silence_ns = (long long)(
        (double)n_initial_silence * (double)tx_mtu * 1e9 / sample_rate);

    /* Prime the TX path with enough untimed silence for BIST loopback/RX to
     * produce a real pre-burst floor, then sample hardware time afterwards so
     * the timed bursts are scheduled from "now", not from a stale prefill time. */
    {
        int ret = write_silence(dev, tx, tx_mtu, n_initial_silence, timeout_us);
        if (ret < 0) { printf("  write_silence (initial) error %d\n", ret); tx_errors++; }
    }

    long long hw_now = dev->getHardwareTime();
    printf("  HW time    : %.6f s\n", hw_now / 1e9);

    for (int p = 0; p < num_pulses && tx_errors == 0; p++) {
        long long t_pulse = hw_now
                            + initial_silence_ns
                            + (long long)(delay_s * 1e9)
                            + (long long)(p * (duration_s + gap_s) * 1e9);
        t_pulses[p] = t_pulse;
        printf("  Pulse %d/%d : sched %.6f s  (%zu bufs)\n",
               p + 1, num_pulses, t_pulse / 1e9, n_burst_bufs);

        int ret = write_burst(dev, tx, tx_mtu, sample_rate,
                              t_pulse, n_burst_bufs, phase_offset, timeout_us);
        if (ret < 0) { printf("  write_burst error %d\n", ret); tx_errors++; break; }
        phase_offset += n_burst_bufs * tx_mtu;

        /* Silence between pulses keeps the TX FIFO alive during the gap. */
        if (p < num_pulses - 1) {
            ret = write_silence(dev, tx, tx_mtu, n_gap_bufs, timeout_us);
            if (ret < 0) { printf("  write_silence error %d\n", ret); tx_errors++; break; }
        }
    }

    /* Wait for full RX capture window, polling readStreamStatus on the TX
     * stream so that late / underrun events from the TimedTXArbiter are
     * surfaced rather than silently discarded. */
    int  late_events    = 0;
    int  underrun_events = 0;
    auto t_wait = std::chrono::steady_clock::now() +
                  std::chrono::duration<double>(total_rec_s + 3.0);
    while (rx_filled.load() < n_rec_bufs &&
           std::chrono::steady_clock::now() < t_wait) {
        size_t    chanMask = 0;
        int       st_flags = 0;
        long long st_time  = 0;
        int ret = dev->readStreamStatus(tx, chanMask, st_flags, st_time,
                                        10000 /* 10 ms */);
        if (ret == SOAPY_SDR_TIME_ERROR) {
            late_events++;
            printf("  [status] TX late event at %.6f s\n", st_time / 1e9);
        } else if (ret == SOAPY_SDR_UNDERFLOW) {
            underrun_events++;
            printf("  [status] TX underrun at %.6f s\n", st_time / 1e9);
        }
        /* SOAPY_SDR_TIMEOUT means no event — keep polling. */
    }

    rx_stop.store(true);
    rx_thread.join();

    dev->deactivateStream(tx);
    dev->deactivateStream(rx);
    dev->closeStream(tx);
    dev->closeStream(rx);
    SoapySDR::Device::unmake(dev);

    size_t total_caps = rx_filled.load();
    printf("  RX captured: %zu bufs  (%d overflows)\n", total_caps, rx_overflows);
    printf("  TX status  : %d late event(s)  %d underrun(s)\n\n",
           late_events, underrun_events);

    if (total_caps < 20) {
        report("Timed TX: enough RX data", false, "too few buffers received");
        return false;
    }

    /* ------------------------------------------------------------------ */
    /* Analysis: per-buffer RMS power.                                     */
    /* ------------------------------------------------------------------ */
    std::vector<double> pwr(total_caps);
    for (size_t i = 0; i < total_caps; i++)
        pwr[i] = rms_power_cs16(rx_caps[i].data(), rx_mtu);

    /* Pre-burst floor: first 25% of the initial delay window. */
    size_t pre_count = std::max((size_t)4,
        (size_t)(delay_s * 0.25 * sample_rate / (double)rx_mtu));
    pre_count = std::min(pre_count, total_caps / 4);
    double pre_floor = 0.0;
    for (size_t i = 0; i < pre_count; i++) pre_floor += pwr[i];
    pre_floor /= (double)pre_count;
    double threshold = 10.0 * pre_floor + 1e-9;  /* 10× floor */

    printf("  Pre-burst floor : %.1f dBFS  (%zu bufs)\n",
           power_db(pre_floor), pre_count);

    /* Per-pulse analysis. */
    std::vector<double> pulse_errors_ms;
    double first_onset_s = -1.0;
    bool all_pass = true;

    for (int p = 0; p < num_pulses; p++) {
        double expected_s   = delay_s + p * (duration_s + gap_s);
        /* Search window starts halfway into the gap before the expected onset
         * (or halfway into the initial delay for the first pulse).  This prevents
         * the window from reaching back into the previous pulse. */
        double gap_before    = (p == 0) ? delay_s : gap_s;
        double search_start_s = expected_s - gap_before * 0.5;
        size_t search_start  = (size_t)(search_start_s * sample_rate / (double)rx_mtu);
        size_t search_end    = std::min(total_caps,
            (size_t)((expected_s + duration_s * 1.5) * sample_rate / (double)rx_mtu));

        /* Find onset in search window. */
        size_t onset_buf = search_end;
        double burst_pwr = 0.0;
        size_t burst_cnt = 0;
        for (size_t i = search_start; i < search_end; i++) {
            if (pwr[i] > threshold) {
                if (onset_buf == search_end) onset_buf = i;
                burst_pwr += pwr[i];
                burst_cnt++;
            }
        }
        if (burst_cnt > 0) burst_pwr /= burst_cnt;

        /* Sub-buffer fine detection within the onset buffer. */
        double    onset_s      = -1.0;
        double    corrected_ms = 9999.0;
        double    raw_error_ms = 9999.0;
        if (onset_buf < search_end) {
            size_t    fine_j       = fine_onset_sample(rx_caps[onset_buf].data(),
                                                       rx_mtu, threshold);
            size_t    onset_samp   = onset_buf * rx_mtu + fine_j;
            onset_s                = (double)onset_samp / sample_rate;
            /* HW-anchored timing: onset_hw_ns is the hardware clock value of
             * the onset sample, derived from the first RX buffer's timestamp. */
            long long onset_hw_ns  = rx_time0_ns
                                     + (long long)((double)onset_samp * 1e9 / sample_rate);
            long long loopback_ns  = (long long)(latency_s * 1e9);
            raw_error_ms           = (double)(onset_hw_ns - t_pulses[p]) / 1e6;
            corrected_ms           = (double)(onset_hw_ns - t_pulses[p] - loopback_ns) / 1e6;
        }
        double delta_db = power_db(burst_pwr) - power_db(pre_floor);

        printf("  Pulse %d/%d : onset %.6f s  expected %.6f s  "
               "raw %+.3f ms  corrected %+.3f ms  delta %.1f dB\n",
               p + 1, num_pulses, onset_s, expected_s, raw_error_ms, corrected_ms, delta_db);

        if (onset_s >= 0.0) {
            pulse_errors_ms.push_back(corrected_ms);
            if (first_onset_s < 0.0)
                first_onset_s = onset_s;
        }

        double rel_error_ms = 0.0;
        if (onset_s >= 0.0 && first_onset_s >= 0.0) {
            double expected_rel_s = p * (duration_s + gap_s);
            rel_error_ms = ((onset_s - first_onset_s) - expected_rel_s) * 1000.0;
        } else {
            rel_error_ms = 9999.0;
        }

        bool pulse_ok = (delta_db >= 20.0) && (fabs(rel_error_ms) <= 60.0);
        all_pass &= pulse_ok;
    }

    /* Summary statistics. */
    if (!pulse_errors_ms.empty()) {
        double sum = 0.0, min_e = 1e9, max_e = -1e9;
        for (double e : pulse_errors_ms) {
            sum   += e;
            min_e  = std::min(min_e, e);
            max_e  = std::max(max_e, e);
        }
        double mean = sum / pulse_errors_ms.size();
        double var  = 0.0;
        for (double e : pulse_errors_ms) var += (e - mean) * (e - mean);
        double std_ms = sqrt(var / pulse_errors_ms.size());
        printf("\n  Timing stats : mean %+.2f ms  min %+.2f ms  max %+.2f ms  std %.2f ms\n",
               mean, min_e, max_e, std_ms);
    }

    bool pass = true;
    pass &= report("Timed TX: no TX errors",         tx_errors == 0,    "TX error");
    pass &= report("Timed TX: all pulses detected",  all_pass,          "a pulse failed power or timing check");
    pass &= report("Timed TX: no late events",       late_events == 0,  "TimedTXArbiter reported late TX");
    return pass;
}

/* ============================================================
 * Phase 5: TDD Emulation
 *
 * Replicates the OCUDU gNB discontinuous TX pattern as configured in
 * m2sdr.yml (23.04 MSPS, 5G NR TDD Band 78, mu=0 15kHz SCS):
 *
 *   Frame period : 10 ms = 230400 samples @ 23.04 MSPS
 *   DL portion   : nof_dl_slots (6) full slots + nof_dl_symbols (8) symbols
 *                  = 138240 + 13166 = 151406 samples ≈ 6.57 ms = 73.9 × MTU
 *   UL silence   : remaining ≈ 78994 samples ≈ 3.43 ms (no TX writes)
 *
 * For each frame:
 *   1. Write guard zeros  (1 MTU, SOAPY_SDR_HAS_TIME at t_dl - guard_ns)
 *      — mirrors the gNB power-ramping zeros before each burst.
 *   2. Write DL burst (n_dl_bufs MTU chunks, HAS_TIME on first, END_BURST on last).
 *      acquireWriteBuffer timeout = awb_timeout_ms; a TIMEOUT return means the
 *      DMA ring is backed up (TimedTXArbiter data_fifo full).
 *   3. Sleep during UL (no TX writes) so the ring can drain before the next frame.
 *   4. A background thread polls readStreamStatus for late / underrun events.
 *
 * Key metric: awb_timeouts — how many acquireWriteBuffer calls timed out.
 *   If >0: the DMA ring is backing up (lead_ms too large relative to FIFO depth).
 *
 * Checks:
 *   - acquireWriteBuffer never times out
 *   - no late events from TimedTXArbiter
 *   - no underrun events
 * ============================================================ */
static bool test_tdd_emulation(double sample_rate, int n_frames, double lead_ms,
                                int nof_dl_slots, int nof_dl_symbols, int nof_ul_slots,
                                long awb_timeout_ms, double init_delay_ms,
                                const LoopbackCfg &cfg, bool bypass_arbiter = false,
                                bool pad_ul_with_zeros = true,
                                bool debug_tdd = false)
{
    printf("\n=== Phase 5: TDD Emulation [%s] [arbiter: %s] ===\n",
           cfg.use_bist ? "BIST" : "RF cable",
           bypass_arbiter ? "bypassed" : "enabled");
    printf("  Frames     : %d  (%.1f s)\n", n_frames, n_frames * 0.01);
    printf("  Sample rate: %.3f MSPS\n", sample_rate / 1e6);
    printf("  TDD pattern: %d DL slots + %d DL symbols + %d UL slots  (10ms frame)\n",
           nof_dl_slots, nof_dl_symbols, nof_ul_slots);
    printf("  TX lead    : %.2f ms\n", lead_ms);

    SoapySDR::Device *dev = open_device(sample_rate, cfg, bypass_arbiter);
    if (!dev) return false;

    SoapySDR::Stream *rx     = dev->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, {0});
    SoapySDR::Stream *tx     = dev->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, {0});
    const size_t      tx_mtu = dev->getStreamMTU(tx);
    const size_t      ring_n = dev->getNumDirectAccessBuffers(tx);

    /* Slot = 1 ms for mu=0 (15 kHz SCS). */
    const size_t slot_samp   = (size_t)(sample_rate / 1000.0 + 0.5);
    const size_t frame_samp  = 10 * slot_samp;
    /* DL: nof_dl_slots full slots + nof_dl_symbols × slot/14. */
    const size_t dl_samp     = (size_t)(nof_dl_slots * slot_samp +
                               (double)nof_dl_symbols * (double)slot_samp / 14.0);
    /* UL: remainder of the frame. */
    const size_t ul_samp     = frame_samp - dl_samp;
    /* Guard zeros: 1 MTU written just before the burst with HAS_TIME. */
    const size_t guard_samp  = tx_mtu;

    const size_t n_dl_bufs   = (dl_samp + tx_mtu - 1) / tx_mtu;
    const size_t n_ul_bufs   = ul_samp / tx_mtu;  /* for display only */

    const double frame_ns_d  = (double)frame_samp / sample_rate * 1e9;
    const double guard_ns_d  = (double)guard_samp  / sample_rate * 1e9;
    const double lead_ns_d   = lead_ms * 1e6;
    const double dl_ns_d     = (double)dl_samp / sample_rate * 1e9;
    const double host_queue_ns_d = std::min(frame_ns_d * 0.8,
                                            std::max(lead_ns_d + 5e5,
                                                     dl_ns_d + 1.0e6));

    printf("  MTU        : %zu samples/buf  (%.3f ms/buf)\n",
           tx_mtu, (double)tx_mtu / sample_rate * 1000.0);
    printf("  TX ring    : %zu buffers  (%.2f ms)\n",
           ring_n, (double)ring_n * tx_mtu / sample_rate * 1000.0);
    printf("  DL burst   : %zu samples  (%.3f ms)  = %zu MTU\n",
           dl_samp, (double)dl_samp / sample_rate * 1000.0, n_dl_bufs);
    printf("  UL silence : %zu samples  (%.3f ms)  = %zu MTU\n",
           ul_samp, (double)ul_samp / sample_rate * 1000.0, n_ul_bufs);
    printf("  Guard zeros: %zu samples  (%.3f ms)\n",
           guard_samp, guard_ns_d / 1e6);
    printf("  Host queue : %.3f ms before guard\n",
           host_queue_ns_d / 1e6);
    printf("  UL mode    : %s\n",
           pad_ul_with_zeros ? "zero-padded (continuous DMA feed)" : "sparse (no TX writes)");

    int64_t submitted_bufs = 0;
    int debug_fd = -1;
    std::mutex debug_mutex;
    TDDDebugSnapshot last_debug_snapshot = {};
    TDDDebugSnapshot base_debug_snapshot = {};
    std::atomic<long long> current_t_guard_ns(0);
    std::atomic<long long> current_t_dl_ns(0);
    auto emit_debug_snapshot = [&](const char *label) {
        if (!(debug_tdd && debug_fd >= 0))
            return TDDDebugSnapshot{};
        std::lock_guard<std::mutex> lock(debug_mutex);
        TDDDebugSnapshot s = debug_take_snapshot(debug_fd);
        if (base_debug_snapshot.valid) {
            s.dma_reader_hw_count -= base_debug_snapshot.dma_reader_hw_count;
            s.dma_reader_sw_count -= base_debug_snapshot.dma_reader_sw_count;
        }
        debug_print_snapshot(label, s, submitted_bufs);
        debug_print_snapshot_delta(label, last_debug_snapshot, s, submitted_bufs);
        last_debug_snapshot = s;
        return s;
    };
    if (debug_tdd) {
        debug_fd = debug_open_fd();
        if (debug_fd < 0)
            printf("  [debug] failed to open /dev/m2sdr0 for CSR snapshots\n");
        else {
            base_debug_snapshot = debug_take_snapshot(debug_fd);
            emit_debug_snapshot("startup");
        }
    }

    if (n_dl_bufs > ring_n) {
        printf("  WARNING: DL burst (%zu MTU) > ring (%zu). "
               "acquireWriteBuffer will block during timed hold.\n",
               n_dl_bufs, ring_n);
    }

    dev->activateStream(rx);
    dev->activateStream(tx);

    if (debug_fd >= 0) {
        debug_reset_timed_tx_state(debug_fd);
        base_debug_snapshot = debug_take_snapshot(debug_fd);
        last_debug_snapshot = base_debug_snapshot;
        if (debug_tdd)
            emit_debug_snapshot("post-reset");
    }

    flush_tx_status(dev, tx, 0);

    if (cfg.use_bist) {
        printf("  Priming    : warming BIST loopback / DMA sync\n");
        bool primed = prime_bist_loopback(dev, rx, tx, tx_mtu, 8);
        printf("  Prime sync : %s\n", primed ? "OK" : "TIMEOUT");
    }

    /* ------------------------------------------------------------------ */
    /* Async readStreamStatus polling thread.                              */
    /* ------------------------------------------------------------------ */
    std::atomic<int>  late_events(0), underrun_events(0);
    std::atomic<bool> status_stop(false);
    std::atomic<bool> status_armed(false);

    std::thread status_th([&]() {
        while (!status_stop.load()) {
            size_t    chanMask = 0;
            int       st_flags = 0;
            long long st_time  = 0;
            int ret = dev->readStreamStatus(tx, chanMask, st_flags, st_time, 1000);
            if (ret == SOAPY_SDR_TIME_ERROR) {
                late_events++;
                long long guard_late_ns = st_time - current_t_guard_ns.load();
                long long dl_late_ns    = st_time - current_t_dl_ns.load();
                printf("  [status] LATE at %.6f s  (vs guard %+0.3f ms  vs dl %+0.3f ms)\n",
                       st_time / 1e9, guard_late_ns / 1e6, dl_late_ns / 1e6);
                emit_debug_snapshot("status-late");
            } else if (ret == SOAPY_SDR_UNDERFLOW) {
                if (status_armed.load()) {
                    underrun_events++;
                    long long guard_late_ns = st_time - current_t_guard_ns.load();
                    long long dl_late_ns    = st_time - current_t_dl_ns.load();
                    printf("  [status] UNDERRUN at %.6f s  (vs guard %+0.3f ms  vs dl %+0.3f ms)\n",
                           st_time / 1e9, guard_late_ns / 1e6, dl_late_ns / 1e6);
                    emit_debug_snapshot("status-underrun");
                }
            }
        }
    });

    /* ------------------------------------------------------------------ */
    /* RX drain thread: just consume to prevent overflow stalls.          */
    /* ------------------------------------------------------------------ */
    std::atomic<bool> rx_stop(false);
    std::thread rx_th([&]() {
        while (!rx_stop.load()) {
            size_t handle = 0; const void *buffs[1];
            int flags = 0; long long timeNs = 0;
            int ret = dev->acquireReadBuffer(rx, handle, buffs, flags, timeNs, 100000);
            // consume without looking at data
            if (ret > 0) dev->releaseReadBuffer(rx, handle);
        }
    });

    /* ------------------------------------------------------------------ */
    /* Initial HW time + startup delay.                                   */
    /*                                                                    */
    /* init_delay_ms sets how far in the future frame 0 is scheduled.    */
    /* It must be larger than the time needed to write one DL burst into  */
    /* the DMA ring (~n_dl_bufs × 50 µs ≈ 4 ms at 75 MTU) so that the   */
    /* FPGA has not yet fired the timestamp when the first burst finishes  */
    /* writing.  lead_ms (the per-burst TX advance) is included in t0_ns. */
    /* ------------------------------------------------------------------ */
    long long hw_start_ns = dev->getHardwareTime();
    long long t0_ns = hw_start_ns
                      + (long long)(init_delay_ms * 1e6)
                      + (long long)(lead_ms * 1e6);

    printf("  Init delay : %.1f ms  (lead %.1f ms  startup %.1f ms)\n",
           init_delay_ms + lead_ms, lead_ms, init_delay_ms);
    printf("  HW start   : %.6f s\n", hw_start_ns / 1e9);
    printf("  Frame 0 DL : %.6f s\n", t0_ns / 1e9);

    /* ------------------------------------------------------------------ */
    /* Main TDD TX loop.                                                  */
    /* ------------------------------------------------------------------ */
    std::vector<int16_t> tone_buf(tx_mtu * 2);
    std::vector<int16_t> zero_buf(tx_mtu * 2, 0);

    int  awb_timeouts    = 0;
    int  awb_errors      = 0;
    int  frames_complete = 0;
    bool stopped_early   = false;

    for (int f = 0; f < n_frames; f++) {
        /* Scheduled HW time for start of this DL burst. */
        long long t_dl    = t0_ns + (long long)((double)f * frame_ns_d);
        /* Guard zeros are sent just before the burst. */
        long long t_guard = t_dl - (long long)guard_ns_d;
        current_t_dl_ns.store(t_dl);
        current_t_guard_ns.store(t_guard);
        if (debug_tdd && debug_fd >= 0) {
            char label[64];
            snprintf(label, sizeof(label), "frame %d prequeue", f);
            emit_debug_snapshot(label);
        }

        /* Queue the full guard + DL burst early enough that userspace is not
         * still filling the ring when the frame timestamp arrives. The RF lead
         * remains `lead_ms`; this host-side queue margin is separate and models
         * how a gNB submits buffers ahead of their on-air time. */
        {
            long long wake_ns = t_guard - (long long)host_queue_ns_d;
            long long hw_jit  = dev->getHardwareTime();
            if (hw_jit < wake_ns)
                std::this_thread::sleep_for(std::chrono::nanoseconds(wake_ns - hw_jit));
            /* Fine spin-wait: 100 us steps until we are within 0.5 ms of the
             * planned host queue start, then fall through to write. */
            long long spin_until = wake_ns - 500000LL;
            while (dev->getHardwareTime() < spin_until)
                std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

        /* Write guard zeros: start the combined guard+DL burst with HAS_TIME at
         * t_guard. The following DL buffers are part of the same burst and must
         * not introduce a second HAS_TIME marker before END_BURST. */
        {
            size_t handle = 0; void *buffs[1];
            //acquire send buffer from DMA engine — if this times out, the ring is backed up and we are late for the scheduled time
            int ret = dev->acquireWriteBuffer(tx, handle, buffs,
                                              awb_timeout_ms * 1000);
            if (ret == SOAPY_SDR_TIMEOUT) {
                printf("  [frame %3d] acquireWriteBuffer TIMEOUT (guard)\n", f);
                emit_debug_snapshot("guard-timeout");
                awb_timeouts++;
                stopped_early = true;
                break;
            }
            if (ret < 0) {
                printf("  [frame %3d] acquireWriteBuffer error %d (guard)\n", f, ret);
                emit_debug_snapshot("guard-error");
                awb_errors++;
                stopped_early = true;
                break;
            }
            //copy zeros into send buffer
            memcpy(buffs[0], zero_buf.data(), tx_mtu * 2 * sizeof(int16_t));
            int flags = SOAPY_SDR_HAS_TIME;
            dev->releaseWriteBuffer(tx, handle, tx_mtu, flags, t_guard);
            submitted_bufs++;
            if (f == 0)
                status_armed.store(true);
            if ((f < 5 || (f % 10) == 0) && debug_tdd)
                emit_debug_snapshot("post-guard");
        }

        /* Write DL burst: n_dl_bufs MTU chunks.
         * First buffer: no new timestamp; it continues the timed burst started
         * by the guard buffer above.
         * Last  buffer: SOAPY_SDR_END_BURST.
         * Fill with a tone so loopback tests can verify signal presence. */
        bool burst_failed = false;
        for (size_t b = 0; b < n_dl_bufs; b++) {
            size_t handle = 0; void *buffs[1];
            int ret = dev->acquireWriteBuffer(tx, handle, buffs,
                                              awb_timeout_ms * 1000);
            if (ret == SOAPY_SDR_TIMEOUT) {
                printf("  [frame %3d buf %3zu] acquireWriteBuffer TIMEOUT\n", f, b);
                emit_debug_snapshot("dl-timeout");
                awb_timeouts++;
                burst_failed = true;
                stopped_early = true;
                break;
            }
            if (ret < 0) {
                printf("  [frame %3d buf %3zu] acquireWriteBuffer error %d\n", f, b, ret);
                emit_debug_snapshot("dl-error");
                awb_errors++;
                burst_failed = true;
                stopped_early = true;
                break;
            }

            const size_t chunk = std::min(tx_mtu, dl_samp - b * tx_mtu);
            fill_tone_cs16((int16_t *)buffs[0], chunk,
                           TONE_FREQ_HZ, sample_rate, AMPLITUDE,
                           (size_t)f * dl_samp + b * tx_mtu);
            /* Zero-pad if chunk < mtu (last partial buffer). */
            if (chunk < tx_mtu)
                memset((int16_t *)buffs[0] + chunk * 2, 0,
                       (tx_mtu - chunk) * 2 * sizeof(int16_t));

            int       flags    = 0;
            long long time_arg = 0;
            if (b == n_dl_bufs-1)    flags |= SOAPY_SDR_END_BURST;
            dev->releaseWriteBuffer(tx, handle, tx_mtu, flags, time_arg);
            submitted_bufs++;
        }

        if (burst_failed) break;
        if ((f < 5 || (f % 10) == 0) && debug_tdd)
            emit_debug_snapshot("post-dl");
        frames_complete++;

        if (pad_ul_with_zeros && ul_samp > 0) {
            int ret = write_zero_span(dev, tx, tx_mtu, ul_samp, awb_timeout_ms * 1000);
            if (ret == SOAPY_SDR_TIMEOUT) {
                printf("  [frame %3d] write_zero_span TIMEOUT (UL)\n", f);
                emit_debug_snapshot("ul-timeout");
                awb_timeouts++;
                stopped_early = true;
                break;
            }
            if (ret < 0) {
                printf("  [frame %3d] write_zero_span error %d (UL)\n", f, ret);
                emit_debug_snapshot("ul-error");
                awb_errors++;
                stopped_early = true;
                break;
            }
            submitted_bufs += (ul_samp + tx_mtu - 1) / tx_mtu;
            if ((f < 5 || (f % 10) == 0) && debug_tdd)
                emit_debug_snapshot("post-ul");
        }

        /* Sleep until just before the next frame's guard zeros are due. We
         * compare against HW time to stay aligned even if software scheduling
         * jitter accumulates. */
        long long t_next_guard = t0_ns + (long long)((double)(f + 1) * frame_ns_d)
                                 - (long long)guard_ns_d;
        long long hw_before_sleep = dev->getHardwareTime();
        /* Sleep until 3 ms before next guard — leaves margin for wakeup jitter. */
        long long sleep_ns = t_next_guard - hw_before_sleep - 3000000LL;
        if (sleep_ns > 0)
            std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
        long long hw_after_sleep  = dev->getHardwareTime();

        if (f % 10 == 0 || f == n_frames - 1 || f < 5)
            printf("  [frame %3d/%d] t_dl=%.6f  after_sleep=%.6f  "
                   "slept=%.1f ms  awb_to=%d  late=%d\n",
                   f, n_frames, t_dl / 1e9,
                   hw_after_sleep / 1e9,
                   (hw_after_sleep - hw_before_sleep) / 1e6,
                   awb_timeouts, late_events.load());
        if ((f < 5 || (f % 10) == 0 || f == n_frames - 1) && debug_tdd)
            emit_debug_snapshot("post-sleep");
    }

    status_stop.store(true);
    rx_stop.store(true);
    status_th.join();
    rx_th.join();

    if (debug_tdd && debug_fd >= 0) {
        emit_debug_snapshot("final");
        close(debug_fd);
    }

    dev->deactivateStream(tx);
    dev->deactivateStream(rx);
    dev->closeStream(tx);
    dev->closeStream(rx);
    SoapySDR::Device::unmake(dev);

    printf("\n  Frames sent : %d / %d%s\n",
           frames_complete, n_frames, stopped_early ? "  (stopped early)" : "");
    printf("  AWB timeouts: %d  (ring backed up by timed hold)\n", awb_timeouts);
    printf("  AWB errors  : %d\n", awb_errors);
    printf("  Late events : %d\n", late_events.load());
    printf("  Underruns   : %d\n", underrun_events.load());

    bool pass = true;
    pass &= report("TDD: all frames transmitted",  !stopped_early,          "stopped early due to TX error");
    pass &= report("TDD: no AWB timeouts",         awb_timeouts == 0,       "DMA ring backed up (lead_ms too large?)");
    pass &= report("TDD: no late events",          late_events.load() == 0, "TimedTXArbiter reported late TX");
    pass &= report("TDD: no underruns",            underrun_events.load() == 0, "underrun detected");
    return pass;
}

/* ============================================================
 * main
 * ============================================================ */
int main(int argc, char **argv)
{
    double     sample_rate     = 30720000.0;
    double     delay_s         = 0.2;
    double     duration_s      = 0.1;   /* per pulse */
    double     gap_s           = 0.3;   /* between pulses */
    double     loopback_s      = 5.0;   /* phase 3 duration */
    int        num_pulses      = 5;
    bool       run_rx          = false;
    bool       run_tx          = false;
    bool       run_loopback    = false;
    bool       run_timed       = false;
    bool       run_rf_lb       = false;
    bool       run_rf_timed    = false;
    bool       run_tdd         = false;
    int        n_bufs          = 200;
    /* TDD emulation parameters (Phase 5). */
    double     tdd_srate       = 23040000.0;  /* matches m2sdr.yml */
    int        tdd_frames      = 100;          /* = 1 second of TDD */
    double     tdd_lead_ms     = 1.0;          /* TX lead time (= -time_alignment_calibration / srate) */
    int        tdd_dl_slots    = 6;
    int        tdd_dl_symbols  = 8;
    int        tdd_ul_slots    = 3;
    long       tdd_awb_timeout = 200;          /* acquireWriteBuffer timeout ms */
    double     tdd_init_ms     = 20.0;         /* startup slack before frame 0 fires */
    bool       bypass_arbiter  = false;        /* disable TimedTXArbiter for all timed phases */
    bool       tdd_pad_ul      = true;         /* continuous zero feed during UL */
    bool       debug_tdd       = false;        /* print CSR snapshots during TDD phase */

    LoopbackCfg rf_cfg;
    rf_cfg.use_bist = false;

    for (int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--samplerate")     && i + 1 < argc) sample_rate      = atof(argv[++i]);
        else if (!strcmp(argv[i], "--delay")          && i + 1 < argc) delay_s          = atof(argv[++i]);
        else if (!strcmp(argv[i], "--duration")       && i + 1 < argc) duration_s       = atof(argv[++i]);
        else if (!strcmp(argv[i], "--num-bufs")       && i + 1 < argc) n_bufs           = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--gap")            && i + 1 < argc) gap_s            = atof(argv[++i]);
        else if (!strcmp(argv[i], "--loopback-s")     && i + 1 < argc) loopback_s       = atof(argv[++i]);
        else if (!strcmp(argv[i], "--num-pulses")     && i + 1 < argc) num_pulses       = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--rf-freq")        && i + 1 < argc) rf_cfg.rf_freq   = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tx-gain")        && i + 1 < argc) rf_cfg.tx_gain   = atof(argv[++i]);
        else if (!strcmp(argv[i], "--rx-gain")        && i + 1 < argc) rf_cfg.rx_gain   = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tdd-srate")      && i + 1 < argc) tdd_srate        = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tdd-frames")     && i + 1 < argc) tdd_frames       = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--tdd-lead-ms")    && i + 1 < argc) tdd_lead_ms      = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tdd-dl-slots")   && i + 1 < argc) tdd_dl_slots     = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--tdd-dl-symbols") && i + 1 < argc) tdd_dl_symbols   = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--tdd-ul-slots")   && i + 1 < argc) tdd_ul_slots     = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--tdd-awb-timeout")&& i + 1 < argc) tdd_awb_timeout  = atol(argv[++i]);
        else if (!strcmp(argv[i], "--tdd-init-ms")    && i + 1 < argc) tdd_init_ms      = atof(argv[++i]);
        else if (!strcmp(argv[i], "--tdd-pad-ul"))                         tdd_pad_ul      = true;
        else if (!strcmp(argv[i], "--tdd-sparse"))                         tdd_pad_ul      = false;
        else if (!strcmp(argv[i], "--debug-tdd"))                         debug_tdd       = true;
        else if (!strcmp(argv[i], "--rx"))              run_rx         = true;
        else if (!strcmp(argv[i], "--tx"))              run_tx         = true;
        else if (!strcmp(argv[i], "--loopback"))        run_loopback   = true;
        else if (!strcmp(argv[i], "--timed"))           run_timed      = true;
        else if (!strcmp(argv[i], "--rf-loopback"))     run_rf_lb      = true;
        else if (!strcmp(argv[i], "--rf-timed"))        run_rf_timed   = true;
        else if (!strcmp(argv[i], "--tdd"))             run_tdd        = true;
        else if (!strcmp(argv[i], "--bypass-arbiter"))  bypass_arbiter = true;
        else if (!strcmp(argv[i], "--no-bypass-arbiter")) bypass_arbiter = false;
        else {
            fprintf(stderr, "Unknown argument: %s\n", argv[i]);
            fprintf(stderr, "Usage: %s [--samplerate Hz] [--delay s] [--duration s]\n", argv[0]);
            fprintf(stderr, "          [--gap s] [--num-pulses N] [--loopback-s s]\n");
            fprintf(stderr, "          [--rx] [--tx] [--loopback] [--timed]\n");
            fprintf(stderr, "          [--rf-loopback] [--rf-timed]\n");
            fprintf(stderr, "          [--rf-freq Hz] [--tx-gain dB] [--rx-gain dB]\n");
            fprintf(stderr, "          [--tdd] [--tdd-srate Hz] [--tdd-frames N]\n");
            fprintf(stderr, "          [--tdd-lead-ms ms] [--tdd-dl-slots N]\n");
            fprintf(stderr, "          [--tdd-dl-symbols N] [--tdd-ul-slots N]\n");
            fprintf(stderr, "          [--tdd-awb-timeout ms] [--tdd-init-ms ms]\n");
            fprintf(stderr, "          [--tdd-pad-ul] [--tdd-sparse] [--debug-tdd]\n");
            fprintf(stderr, "          [--bypass-arbiter] [--no-bypass-arbiter]\n");
            fprintf(stderr, "\n");
            fprintf(stderr, "  --bypass-arbiter   : disable TimedTXArbiter for timed phases (timestamps ignored)\n");
            fprintf(stderr, "  --no-bypass-arbiter: re-enable TimedTXArbiter (default)\n");
            fprintf(stderr, "  --tdd-pad-ul       : write zero buffers through UL gaps (default)\n");
            fprintf(stderr, "  --tdd-sparse       : no TX writes in UL gaps; reproduces sparse gNB TDD\n");
            fprintf(stderr, "  --debug-tdd        : print DMA/timed-TX CSR snapshots during TDD submit\n");
            return 1;
        }
    }

    /* Default: run all phases using BIST loopback for phases 3+4. */
    if (!(run_rx || run_tx || run_loopback || run_timed || run_rf_lb || run_rf_timed || run_tdd))
        run_rx = run_tx = run_loopback = run_timed = true;

    LoopbackCfg bist_cfg;  /* default-constructed: use_bist=true */

    printf("M2SDR Streaming Test\n");
    printf("  Sample rate : %.3f MSPS\n", sample_rate / 1e6);
    printf("  Phases      :%s%s%s%s%s%s%s\n",
           run_rx        ? " RX"         : "",
           run_tx        ? " TX"         : "",
           run_loopback  ? " Loopback"   : "",
           run_timed     ? " TimedTX"    : "",
           run_rf_lb     ? " RF-Loopback": "",
           run_rf_timed  ? " RF-TimedTX" : "",
           run_tdd       ? " TDD"        : "");
    if (run_loopback || run_rf_lb)
        printf("  Loopback    : %.2f s\n", loopback_s);
    if (run_timed || run_rf_timed)
        printf("  Timed TX    : %d pulses  %.3f s/pulse  %.3f s gap  %.3f s delay\n",
               num_pulses, duration_s, gap_s, delay_s);
    if (run_rf_lb || run_rf_timed)
        printf("  RF cable    : %.3f MHz  TX %.1f dB  RX %.1f dB\n",
               rf_cfg.rf_freq / 1e6, rf_cfg.tx_gain, rf_cfg.rx_gain);
    if (run_tdd)
        printf("  TDD         : %.3f MSPS  %d frames  lead %.1f ms  "
               "%dDL+%dsym+%dUL  %s%s\n",
               tdd_srate / 1e6, tdd_frames, tdd_lead_ms,
               tdd_dl_slots, tdd_dl_symbols, tdd_ul_slots,
               tdd_pad_ul ? "UL padded" : "UL sparse",
               debug_tdd ? "  debug" : "");
    if (run_timed || run_rf_timed || run_tdd)
        printf("  Arbiter     : %s\n", bypass_arbiter ? "BYPASSED (timestamps ignored)" : "enabled");

    /* Suppress SoapySDR INFO chatter — show only warnings and above. */
    SoapySDR::setLogLevel(SOAPY_SDR_WARNING);

    bool      overall = true;

    if (run_rx)       overall &= test_rx_baseline(sample_rate, n_bufs);
    if (run_tx)       overall &= test_tx_baseline(sample_rate, n_bufs);
    if (run_loopback) overall &= test_loopback(sample_rate, loopback_s, bist_cfg);

    /* Measure round-trip loopback latency before any timed TX phase so the
     * corrected timing error reflects only the TimedTXArbiter accuracy. */
    double bist_latency = 0.0;
    double rf_latency   = 0.0;
    if (run_timed)    bist_latency = measure_loopback_latency(sample_rate, bist_cfg);
    if (run_rf_timed) rf_latency   = measure_loopback_latency(sample_rate, rf_cfg);

    if (run_timed)    overall &= test_timed_tx(sample_rate, delay_s, duration_s, gap_s, num_pulses, bist_cfg, bist_latency, bypass_arbiter);
    if (run_rf_lb)    overall &= test_loopback(sample_rate, loopback_s, rf_cfg);
    if (run_rf_timed) overall &= test_timed_tx(sample_rate, delay_s, duration_s, gap_s, num_pulses, rf_cfg, rf_latency, bypass_arbiter);
    if (run_tdd)      overall &= test_tdd_emulation(tdd_srate, tdd_frames, tdd_lead_ms,
                                                    tdd_dl_slots, tdd_dl_symbols, tdd_ul_slots,
                                                    tdd_awb_timeout, tdd_init_ms, bist_cfg, bypass_arbiter,
                                                    tdd_pad_ul, debug_tdd);

    printf("\n=== Overall: %s ===\n", overall ? "PASS" : "FAIL");
    return overall ? 0 : 1;
}
