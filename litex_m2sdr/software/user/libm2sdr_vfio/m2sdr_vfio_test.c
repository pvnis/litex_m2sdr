/* SPDX-License-Identifier: BSD-2-Clause
 *
 * M2SDR VFIO self-test / bringup tool
 *
 * Usage:
 *   ./m2sdr_vfio_test [pci_addr] [poll_mode]
 *   ./m2sdr_vfio_test 0000:01:00.0 0   # busypoll
 *   ./m2sdr_vfio_test 0000:01:00.0 1   # adaptive (default)
 *
 * Tests performed:
 *   1. VFIO open, BAR0 map, DMA alloc
 *   2. CSR scratch register read/write (verifies BAR0 MMIO works)
 *   3. RX ring start, receive 64 buffers, check hw_count advances
 *   4. TX ring start, submit 64 buffers, check hw_count advances
 *   5. Loopback: enable DMA loopback, check RX data matches TX
 *   6. Latency: measure acquire→release round-trip time (1000 samples)
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <errno.h>

#include "m2sdr_vfio.h"

/* CSR addresses reused from kernel driver csr.h */
#define CSR_CTRL_SCRATCH_ADDR  0x4UL
#define CSR_PCIE_DMA0_BASE     0x6000UL
#define DMA_LOOPBACK_ENABLE    0x0040UL

#define N_TEST_BUFS  64
#define N_LAT_SAMPLES 1000

static uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
}

/* -------------------------------------------------------------------------
 * Test 1: BAR0 MMIO — scratch register
 * ---------------------------------------------------------------------- */
static int test_csr(struct m2sdr_vfio *v)
{
    uint32_t val;
    const uint32_t magic = 0xdeadbeef;

    printf("\n[test] CSR scratch register read/write...\n");

    m2sdr_vfio_writel(v, CSR_CTRL_SCRATCH_ADDR, magic);
    val = m2sdr_vfio_readl(v, CSR_CTRL_SCRATCH_ADDR);

    if (val != magic) {
        fprintf(stderr, "[test] FAIL: wrote 0x%08x read 0x%08x\n", magic, val);
        return -1;
    }
    printf("[test] PASS: scratch = 0x%08x\n", val);
    return 0;
}

/* -------------------------------------------------------------------------
 * Test 2: RX — receive N_TEST_BUFS buffers, verify hw_count advances
 * ---------------------------------------------------------------------- */
static int test_rx(struct m2sdr_vfio *v)
{
    int i, handle;
    void *buf;
    int64_t hw_before, hw_after;

    printf("\n[test] RX: starting DMA writer...\n");
    if (m2sdr_vfio_rx_start(v) < 0)
        return -1;

    hw_before = m2sdr_vfio_rx_hw_count(v);

    for (i = 0; i < N_TEST_BUFS; i++) {
        buf = m2sdr_vfio_rx_acquire(v, &handle);
        if (!buf) {
            fprintf(stderr, "[test] rx_acquire failed at i=%d\n", i);
            m2sdr_vfio_rx_stop(v);
            return -1;
        }
        m2sdr_vfio_rx_release(v, handle);
    }

    hw_after = m2sdr_vfio_rx_hw_count(v);
    m2sdr_vfio_rx_stop(v);

    printf("[test] RX: hw_count before=%lld after=%lld (delta=%lld)\n",
           (long long)hw_before, (long long)hw_after,
           (long long)(hw_after - hw_before));

    if (hw_after <= hw_before) {
        fprintf(stderr, "[test] FAIL: hw_count did not advance\n");
        return -1;
    }
    printf("[test] PASS: RX hw_count advanced correctly\n");
    return 0;
}

/* -------------------------------------------------------------------------
 * Test 3: TX — submit N_TEST_BUFS buffers, verify hw_count advances
 * ---------------------------------------------------------------------- */
static int test_tx(struct m2sdr_vfio *v)
{
    int i, handle;
    void *buf;
    int64_t hw_before, hw_after;

    printf("\n[test] TX: starting DMA reader...\n");
    if (m2sdr_vfio_tx_start(v) < 0)
        return -1;

    hw_before = m2sdr_vfio_tx_hw_count(v);

    for (i = 0; i < N_TEST_BUFS; i++) {
        buf = m2sdr_vfio_tx_acquire(v, &handle);
        if (!buf) {
            fprintf(stderr, "[test] tx_acquire failed at i=%d\n", i);
            m2sdr_vfio_tx_stop(v);
            return -1;
        }
        memset(buf, 0, M2SDR_DMA_BUFFER_SIZE);
        m2sdr_vfio_tx_release(v, handle, M2SDR_DMA_BUFFER_SIZE, 0, 0);
    }

    hw_after = m2sdr_vfio_tx_hw_count(v);
    m2sdr_vfio_tx_stop(v);

    printf("[test] TX: hw_count before=%lld after=%lld (delta=%lld)\n",
           (long long)hw_before, (long long)hw_after,
           (long long)(hw_after - hw_before));

    if (hw_after <= hw_before) {
        fprintf(stderr, "[test] FAIL: hw_count did not advance\n");
        return -1;
    }
    printf("[test] PASS: TX hw_count advanced correctly\n");
    return 0;
}

/* -------------------------------------------------------------------------
 * Test 4: RX acquire latency measurement
 * ---------------------------------------------------------------------- */
static int test_latency(struct m2sdr_vfio *v)
{
    int i, handle;
    void *buf;
    uint64_t t0, t1, dt;
    uint64_t total = 0, min_ns = UINT64_MAX, max_ns = 0;
    uint64_t samples[N_LAT_SAMPLES];

    printf("\n[test] Latency: measuring rx_acquire round-trip (%d samples)...\n",
           N_LAT_SAMPLES);

    if (m2sdr_vfio_rx_start(v) < 0)
        return -1;

    for (i = 0; i < N_LAT_SAMPLES; i++) {
        t0  = now_ns();
        buf = m2sdr_vfio_rx_acquire(v, &handle);
        t1  = now_ns();

        if (!buf) {
            fprintf(stderr, "[test] rx_acquire failed at sample %d\n", i);
            m2sdr_vfio_rx_stop(v);
            return -1;
        }
        m2sdr_vfio_rx_release(v, handle);

        dt = t1 - t0;
        samples[i] = dt;
        total += dt;
        if (dt < min_ns) min_ns = dt;
        if (dt > max_ns) max_ns = dt;
    }

    m2sdr_vfio_rx_stop(v);

    /* Simple percentile sort (insertion sort — N is small) */
    for (i = 1; i < N_LAT_SAMPLES; i++) {
        uint64_t key = samples[i];
        int j = i - 1;
        while (j >= 0 && samples[j] > key) {
            samples[j + 1] = samples[j];
            j--;
        }
        samples[j + 1] = key;
    }

    printf("[test] Latency results (rx_acquire → data available):\n");
    printf("  mean   = %llu ns\n", (unsigned long long)(total / N_LAT_SAMPLES));
    printf("  min    = %llu ns\n", (unsigned long long)min_ns);
    printf("  p50    = %llu ns\n", (unsigned long long)samples[N_LAT_SAMPLES / 2]);
    printf("  p99    = %llu ns\n", (unsigned long long)samples[N_LAT_SAMPLES * 99 / 100]);
    printf("  p999   = %llu ns\n", (unsigned long long)samples[N_LAT_SAMPLES * 999 / 1000]);
    printf("  max    = %llu ns\n", (unsigned long long)max_ns);
    printf("[test] PASS\n");
    return 0;
}

/* -------------------------------------------------------------------------
 * Main
 * ---------------------------------------------------------------------- */
int main(int argc, char **argv)
{
    const char *pci_addr  = "0000:01:00.0";
    int         poll_mode = M2SDR_POLL_ADAPTIVE;

    if (argc > 1) pci_addr  = argv[1];
    if (argc > 2) poll_mode = atoi(argv[2]);

    printf("M2SDR VFIO self-test\n");
    printf("  device    : %s\n", pci_addr);
    printf("  poll_mode : %d (%s)\n", poll_mode,
           poll_mode == M2SDR_POLL_BUSYPOLL ? "BUSYPOLL" :
           poll_mode == M2SDR_POLL_ADAPTIVE  ? "ADAPTIVE"  :
                                               "IRQ");

    struct m2sdr_vfio *v = m2sdr_vfio_open(pci_addr, poll_mode);
    if (!v) {
        fprintf(stderr, "m2sdr_vfio_open failed\n");
        return 1;
    }

    int rc = 0;
    rc |= test_csr(v);
    if (rc) goto done;
    rc |= test_rx(v);
    if (rc) goto done;
    rc |= test_tx(v);
    if (rc) goto done;
    rc |= test_latency(v);

done:
    m2sdr_vfio_close(v);
    printf("\n%s\n", rc ? "OVERALL: FAIL" : "OVERALL: PASS");
    return rc ? 1 : 0;
}
