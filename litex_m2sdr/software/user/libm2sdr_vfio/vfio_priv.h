/* SPDX-License-Identifier: BSD-2-Clause
 *
 * M2SDR VFIO User-Mode Driver — internal state (not part of public API)
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 */

#ifndef __VFIO_PRIV_H
#define __VFIO_PRIV_H

#include <stdint.h>
#include <stdbool.h>
#include "m2sdr_vfio.h"

/* Internal representation of the device */
struct m2sdr_vfio {
    /* VFIO file descriptors */
    int  container_fd;
    int  group_fd;
    int  device_fd;
    int  irq_fd;          /* MSI eventfd, -1 in BUSYPOLL mode */
    int  poll_mode;       /* M2SDR_POLL_* */

    /* BAR0 MMIO mapping */
    void  *bar0;
    size_t bar0_size;

    /* RX DMA ring (DMA Writer: FPGA → host) */
    void    *rx_virt;         /* virtual address of ring buffer */
    uint64_t rx_iova;         /* IOVA programmed into descriptor table */
    size_t   rx_size;
    bool     rx_iova_mapped;
    bool     rx_active;
    int64_t  rx_hw_count;     /* last hw_count read from LOOP_STATUS */
    int64_t  rx_sw_count;     /* buffers consumed by software */
    uint32_t rx_hw_count_last_raw; /* for loop_count wrap detection */

    /* TX DMA ring (DMA Reader: host → FPGA) */
    void    *tx_virt;
    uint64_t tx_iova;
    size_t   tx_size;
    bool     tx_iova_mapped;
    bool     tx_active;
    int64_t  tx_hw_count;
    int64_t  tx_sw_count;     /* buffers submitted by software */
    uint32_t tx_hw_count_last_raw;
};

/* -------------------------------------------------------------------------
 * Inline MMIO helpers
 *
 * Use __ATOMIC_ACQUIRE/RELEASE for correct MMIO ordering on x86 without
 * a full mfence.  On x86, loads/stores to WC or UC memory are already
 * strongly ordered; the compiler barrier is sufficient.
 * ---------------------------------------------------------------------- */

static inline uint32_t mmio_read32(void *bar, uint32_t off)
{
    volatile uint32_t *p = (volatile uint32_t *)((char *)bar + off);
    return *p;
}

static inline void mmio_write32(void *bar, uint32_t off, uint32_t val)
{
    volatile uint32_t *p = (volatile uint32_t *)((char *)bar + off);
    *p = val;
    /* Compiler barrier — prevents reordering of writes around this point */
    __asm__ volatile("" ::: "memory");
}

/* Compute hw_count from a LOOP_STATUS register value, handling the
 * 16-bit loop_count wrap-around the same way the kernel driver does. */
static inline int64_t loop_status_to_hw_count(uint32_t status,
                                               int64_t  prev_hw_count)
{
    uint32_t loop_count = status >> 16;
    uint32_t index      = status & 0xffff;
    int64_t  candidate  = (int64_t)loop_count * M2SDR_DMA_BUFFER_COUNT + index;

    /* Detect when loop_count (16-bit) has wrapped: if candidate appears
     * to have gone backwards, add one full wrap period. */
    if (candidate < (prev_hw_count & (int64_t)0xffffffffffff0000LL)) {
        candidate += (int64_t)(1 << (16 + 8)); /* 16-bit loop_count × 256 */
    }
    return candidate;
}

/* Forward declarations (implementations in other .c files) */
void m2sdr_vfio_rx_stop(struct m2sdr_vfio *v);
void m2sdr_vfio_tx_stop(struct m2sdr_vfio *v);
uint32_t m2sdr_vfio_readl(struct m2sdr_vfio *v, uint32_t csr_addr);
void     m2sdr_vfio_writel(struct m2sdr_vfio *v, uint32_t csr_addr, uint32_t val);

#endif /* __VFIO_PRIV_H */
