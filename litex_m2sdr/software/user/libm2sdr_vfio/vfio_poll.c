/* SPDX-License-Identifier: BSD-2-Clause
 *
 * M2SDR VFIO User-Mode Driver — polling engine and buffer acquire/release
 *
 * Three poll modes:
 *
 *  BUSYPOLL  — tight spin on LOOP_STATUS.  Lowest latency (~100 ns round-
 *              trip on modern x86), but burns one full CPU core.  Best for
 *              dedicated real-time threads.
 *
 *  ADAPTIVE  — spin for up to SPIN_BUDGET_NS nanoseconds, then block on
 *              the MSI eventfd.  Good default for 5G NR: spins during
 *              active bursts (< 1 µs), sleeps during inter-subframe gaps.
 *
 *  IRQ       — always block on the MSI eventfd.  Lowest CPU usage; latency
 *              is dominated by kernel→user eventfd delivery (~2–5 µs).
 *
 * LOOP_STATUS register layout (32-bit, read-only):
 *   [31:16]  loop_count   number of complete passes through the ring
 *   [15:0]   index        current descriptor being processed (0–255)
 *
 * hw_count = loop_count × 256 + index
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>
#include <sys/select.h>
#include <sys/eventfd.h>

#include "m2sdr_vfio.h"
#include "vfio_priv.h"

/* Spin budget before sleeping on eventfd (nanoseconds) */
#define SPIN_BUDGET_NS  5000ULL  /* 5 µs */

/* Half-ring threshold: TX considers > 128 pending buffers as full */
#define TX_MAX_PENDING  (M2SDR_DMA_BUFFER_COUNT / 2)

/* -------------------------------------------------------------------------
 * Timing helper
 * ---------------------------------------------------------------------- */

static inline uint64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/* -------------------------------------------------------------------------
 * Internal: read hw_count from a LOOP_STATUS register
 * ---------------------------------------------------------------------- */

static int64_t read_hw_count(struct m2sdr_vfio *v, uint32_t status_reg_off,
                              int64_t prev_hw_count)
{
    uint32_t status = mmio_read32(v->bar0,
                                  M2SDR_CSR_DMA0_BASE + status_reg_off);
    return loop_status_to_hw_count(status, prev_hw_count);
}

/* -------------------------------------------------------------------------
 * Internal: block on MSI eventfd until it fires or timeout_ns elapses.
 * Drains the eventfd counter so the next wait is clean.
 * Returns 0 if event received, -ETIMEDOUT otherwise.
 * ---------------------------------------------------------------------- */

static int wait_msi(struct m2sdr_vfio *v, uint64_t timeout_ns)
{
    struct timespec ts;
    uint64_t val;
    fd_set   rfds;
    int      ret;

    if (v->irq_fd < 0)
        return -ENOENT;

    FD_ZERO(&rfds);
    FD_SET(v->irq_fd, &rfds);

    ts.tv_sec  = timeout_ns / 1000000000ULL;
    ts.tv_nsec = timeout_ns % 1000000000ULL;

    ret = pselect(v->irq_fd + 1, &rfds, NULL, NULL, &ts, NULL);
    if (ret > 0) {
        /* Drain eventfd counter */
        ssize_t __attribute__((unused)) _r = read(v->irq_fd, &val, sizeof(val));
        /* Clear MSI in gateware */
        m2sdr_vfio_writel(v, M2SDR_CSR_PCIE_MSI_CLEAR,
                          (1u << M2SDR_DMA0_WRITER_IRQ_BIT) |
                          (1u << M2SDR_DMA0_READER_IRQ_BIT));
        return 0;
    }
    return -ETIMEDOUT;
}

/* -------------------------------------------------------------------------
 * Internal: wait until at least one new buffer is available.
 * Returns number of newly available buffers (> 0), or < 0 on error.
 * ---------------------------------------------------------------------- */

static int wait_rx_buffer(struct m2sdr_vfio *v)
{
    uint64_t deadline;
    int64_t  hw;
    int      avail;

    if (v->poll_mode == M2SDR_POLL_BUSYPOLL) {
        /* Tight spin */
        while (1) {
            hw    = read_hw_count(v, M2SDR_DMA_WRITER_LOOP_STATUS, v->rx_hw_count);
            avail = (int)(hw - v->rx_sw_count);
            if (avail > 0) {
                v->rx_hw_count = hw;
                return avail;
            }
            /* Hint to CPU that we're in a spin loop */
            __asm__ volatile("pause" ::: "memory");
        }
    }

    if (v->poll_mode == M2SDR_POLL_IRQ) {
        /* Pure eventfd wait, 500 ms timeout then retry */
        while (1) {
            wait_msi(v, 500000000ULL);
            hw    = read_hw_count(v, M2SDR_DMA_WRITER_LOOP_STATUS, v->rx_hw_count);
            avail = (int)(hw - v->rx_sw_count);
            if (avail > 0) {
                v->rx_hw_count = hw;
                return avail;
            }
        }
    }

    /* ADAPTIVE: spin for SPIN_BUDGET_NS, then sleep */
    deadline = now_ns() + SPIN_BUDGET_NS;
    while (now_ns() < deadline) {
        hw    = read_hw_count(v, M2SDR_DMA_WRITER_LOOP_STATUS, v->rx_hw_count);
        avail = (int)(hw - v->rx_sw_count);
        if (avail > 0) {
            v->rx_hw_count = hw;
            return avail;
        }
        __asm__ volatile("pause" ::: "memory");
    }

    /* Sleep on eventfd for up to 500 ms */
    wait_msi(v, 500000000ULL);
    hw    = read_hw_count(v, M2SDR_DMA_WRITER_LOOP_STATUS, v->rx_hw_count);
    avail = (int)(hw - v->rx_sw_count);
    if (avail > 0)
        v->rx_hw_count = hw;
    return (avail > 0) ? avail : 0;
}

static int wait_tx_buffer(struct m2sdr_vfio *v)
{
    uint64_t deadline;
    int64_t  hw;
    int      pending, free_slots;

    if (v->poll_mode == M2SDR_POLL_BUSYPOLL) {
        while (1) {
            hw        = read_hw_count(v, M2SDR_DMA_READER_LOOP_STATUS, v->tx_hw_count);
            pending   = (int)(v->tx_sw_count - hw);
            free_slots = TX_MAX_PENDING - pending;
            if (free_slots > 0) {
                v->tx_hw_count = hw;
                return free_slots;
            }
            __asm__ volatile("pause" ::: "memory");
        }
    }

    if (v->poll_mode == M2SDR_POLL_IRQ) {
        while (1) {
            wait_msi(v, 500000000ULL);
            hw        = read_hw_count(v, M2SDR_DMA_READER_LOOP_STATUS, v->tx_hw_count);
            pending   = (int)(v->tx_sw_count - hw);
            free_slots = TX_MAX_PENDING - pending;
            if (free_slots > 0) {
                v->tx_hw_count = hw;
                return free_slots;
            }
        }
    }

    /* ADAPTIVE */
    deadline = now_ns() + SPIN_BUDGET_NS;
    while (now_ns() < deadline) {
        hw        = read_hw_count(v, M2SDR_DMA_READER_LOOP_STATUS, v->tx_hw_count);
        pending   = (int)(v->tx_sw_count - hw);
        free_slots = TX_MAX_PENDING - pending;
        if (free_slots > 0) {
            v->tx_hw_count = hw;
            return free_slots;
        }
        __asm__ volatile("pause" ::: "memory");
    }

    wait_msi(v, 500000000ULL);
    hw        = read_hw_count(v, M2SDR_DMA_READER_LOOP_STATUS, v->tx_hw_count);
    pending   = (int)(v->tx_sw_count - hw);
    free_slots = TX_MAX_PENDING - pending;
    if (free_slots > 0)
        v->tx_hw_count = hw;
    return (free_slots > 0) ? free_slots : 0;
}

/* -------------------------------------------------------------------------
 * Public: RX buffer acquire / release
 * ---------------------------------------------------------------------- */

void *m2sdr_vfio_rx_acquire(struct m2sdr_vfio *v, int *handle)
{
    /* Check if a buffer is already available without polling */
    int64_t hw    = read_hw_count(v, M2SDR_DMA_WRITER_LOOP_STATUS, v->rx_hw_count);
    int     avail = (int)(hw - v->rx_sw_count);

    if (avail <= 0) {
        /* Need to wait */
        avail = wait_rx_buffer(v);
        if (avail <= 0)
            return NULL;
    } else {
        v->rx_hw_count = hw;
    }

    *handle = (int)(v->rx_sw_count % M2SDR_DMA_BUFFER_COUNT);
    return (char *)v->rx_virt + (size_t)(*handle) * M2SDR_DMA_BUFFER_SIZE;
}

void m2sdr_vfio_rx_release(struct m2sdr_vfio *v, int handle)
{
    (void)handle;  /* index is implicit from sw_count */
    v->rx_sw_count++;
}

int64_t m2sdr_vfio_rx_hw_count(struct m2sdr_vfio *v)
{
    v->rx_hw_count = read_hw_count(v, M2SDR_DMA_WRITER_LOOP_STATUS,
                                   v->rx_hw_count);
    return v->rx_hw_count;
}

int m2sdr_vfio_rx_wait(struct m2sdr_vfio *v, int64_t *hw_count_out,
                        int64_t sw_count_hint, long timeout_us)
{
    uint64_t deadline = now_ns() + (uint64_t)timeout_us * 1000ULL;
    int64_t  hw;
    int      avail;

    /* Spin phase */
    while (1) {
        hw    = read_hw_count(v, M2SDR_DMA_WRITER_LOOP_STATUS, v->rx_hw_count);
        avail = (int)(hw - sw_count_hint);
        if (avail > 0) {
            v->rx_hw_count = hw;
            *hw_count_out  = hw;
            return avail;
        }
        if (now_ns() >= deadline)
            break;
        if (v->poll_mode == M2SDR_POLL_BUSYPOLL) {
            __asm__ volatile("pause" ::: "memory");
            continue;
        }
        /* ADAPTIVE/IRQ: sleep on eventfd for remaining time or SPIN_BUDGET_NS */
        if (v->irq_fd >= 0) {
            uint64_t remain = deadline - now_ns();
            if (remain == 0) break;
            wait_msi(v, remain < SPIN_BUDGET_NS ? remain : SPIN_BUDGET_NS);
        } else {
            __asm__ volatile("pause" ::: "memory");
        }
    }

    hw    = read_hw_count(v, M2SDR_DMA_WRITER_LOOP_STATUS, v->rx_hw_count);
    avail = (int)(hw - sw_count_hint);
    v->rx_hw_count = hw;
    *hw_count_out  = hw;
    return avail > 0 ? avail : 0;
}

void m2sdr_vfio_rx_sw_set(struct m2sdr_vfio *v, int64_t count)
{
    v->rx_sw_count = count;
}

/* -------------------------------------------------------------------------
 * Public: TX buffer acquire / release
 * ---------------------------------------------------------------------- */

void *m2sdr_vfio_tx_acquire(struct m2sdr_vfio *v, int *handle)
{
    /* Check without polling */
    int64_t hw        = read_hw_count(v, M2SDR_DMA_READER_LOOP_STATUS, v->tx_hw_count);
    int     pending   = (int)(v->tx_sw_count - hw);
    int     free_slots = TX_MAX_PENDING - pending;

    if (free_slots <= 0) {
        free_slots = wait_tx_buffer(v);
        if (free_slots <= 0)
            return NULL;
    } else {
        v->tx_hw_count = hw;
    }

    *handle = (int)(v->tx_sw_count % M2SDR_DMA_BUFFER_COUNT);
    return (char *)v->tx_virt + (size_t)(*handle) * M2SDR_DMA_BUFFER_SIZE;
}

void m2sdr_vfio_tx_release(struct m2sdr_vfio *v, int handle,
                            int n_bytes, uint64_t time_ns, int flags)
{
    (void)handle;
    (void)n_bytes;
    (void)time_ns;
    (void)flags;
    /* In loop mode the DMA engine reads each buffer as the ring advances.
     * All we need to do is advance sw_count so the next acquire points to
     * the correct buffer slot.  Timed TX timestamp insertion is handled
     * by the caller writing the sync word + timestamp header into the
     * buffer before calling tx_release (same as SoapySDR streaming today). */
    v->tx_sw_count++;
}

int64_t m2sdr_vfio_tx_hw_count(struct m2sdr_vfio *v)
{
    v->tx_hw_count = read_hw_count(v, M2SDR_DMA_READER_LOOP_STATUS,
                                   v->tx_hw_count);
    return v->tx_hw_count;
}

int m2sdr_vfio_tx_wait(struct m2sdr_vfio *v, int64_t *hw_count_out,
                        int64_t sw_count_hint, long timeout_us)
{
    uint64_t deadline = now_ns() + (uint64_t)timeout_us * 1000ULL;
    int64_t  hw;
    int      pending, free_slots;

    while (1) {
        hw         = read_hw_count(v, M2SDR_DMA_READER_LOOP_STATUS, v->tx_hw_count);
        pending    = (int)(sw_count_hint - hw);
        free_slots = TX_MAX_PENDING - pending;
        if (free_slots > 0) {
            v->tx_hw_count = hw;
            *hw_count_out  = hw;
            return free_slots;
        }
        if (now_ns() >= deadline)
            break;
        if (v->poll_mode == M2SDR_POLL_BUSYPOLL) {
            __asm__ volatile("pause" ::: "memory");
            continue;
        }
        if (v->irq_fd >= 0) {
            uint64_t remain = deadline - now_ns();
            if (remain == 0) break;
            wait_msi(v, remain < SPIN_BUDGET_NS ? remain : SPIN_BUDGET_NS);
        } else {
            __asm__ volatile("pause" ::: "memory");
        }
    }

    hw         = read_hw_count(v, M2SDR_DMA_READER_LOOP_STATUS, v->tx_hw_count);
    pending    = (int)(sw_count_hint - hw);
    free_slots = TX_MAX_PENDING - pending;
    v->tx_hw_count = hw;
    *hw_count_out  = hw;
    return free_slots > 0 ? free_slots : 0;
}
