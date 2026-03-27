/* SPDX-License-Identifier: BSD-2-Clause
 *
 * M2SDR VFIO User-Mode Driver — DMA descriptor ring initialisation
 *
 * Programs the 256-entry TX (Reader) and RX (Writer) descriptor rings
 * directly via BAR0 MMIO, replicating what the kernel driver does in
 * litepcie_dma_writer_start() / litepcie_dma_reader_start().
 *
 * Descriptor format (3 × 32-bit MMIO writes, in order):
 *   TABLE_VALUE[0]  = DMA_LAST_DISABLE | IRQ_DISABLE_if_not_IRQ_boundary | size
 *   TABLE_VALUE[1]  = addr[31:0]
 *   TABLE_WE        = addr[63:32]   ← this write commits the entry
 *
 * The IRQ fires every DMA_BUFFER_PER_IRQ (8) buffers: specifically at
 * i % 8 == 0 (entries 0, 8, 16, … 248).
 *
 * Loop mode: LOOP_PROG_N is cleared (0) while programming, then set (1)
 * after all descriptors are written to activate the loop.
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 */

#include <stdio.h>
#include <unistd.h>   /* usleep */
#include "m2sdr_vfio.h"
#include "vfio_priv.h"

/* -------------------------------------------------------------------------
 * Internal: write one descriptor into a ring
 * ---------------------------------------------------------------------- */

static void write_descriptor(struct m2sdr_vfio *v,
                              uint32_t dma_base,
                              uint32_t value_off,
                              uint32_t we_off,
                              uint64_t buf_iova,
                              int      index)
{
    uint32_t flags = M2SDR_DMA_LAST_DISABLE;

    /* Generate MSI every M2SDR_DMA_BUFFER_PER_IRQ buffers.
     * Same logic as kernel: IRQ fires when (i % PER_IRQ == 0). */
    if (index % M2SDR_DMA_BUFFER_PER_IRQ != 0)
        flags |= M2SDR_DMA_IRQ_DISABLE;

    /* Word 0: flags | transfer size */
    mmio_write32(v->bar0, dma_base + value_off,
                 flags | M2SDR_DMA_BUFFER_SIZE);

    /* Word 1: address bits [31:0] */
    mmio_write32(v->bar0, dma_base + value_off + 4,
                 (uint32_t)(buf_iova & 0xffffffffu));

    /* Word 2: address bits [63:32] — commits the descriptor */
    mmio_write32(v->bar0, dma_base + we_off,
                 (uint32_t)(buf_iova >> 32));
}

/* -------------------------------------------------------------------------
 * Public: RX ring start / stop  (DMA Writer: FPGA → host)
 * ---------------------------------------------------------------------- */

int m2sdr_vfio_rx_start(struct m2sdr_vfio *v)
{
    uint32_t base = M2SDR_CSR_DMA0_BASE;
    int i;

    /* Disable engine and flush any stale descriptors */
    mmio_write32(v->bar0, base + M2SDR_DMA_WRITER_ENABLE,      0);
    mmio_write32(v->bar0, base + M2SDR_DMA_WRITER_FLUSH,       1);
    mmio_write32(v->bar0, base + M2SDR_DMA_WRITER_LOOP_PROG_N, 0); /* programming mode */

    /* Write all 256 descriptors */
    for (i = 0; i < M2SDR_DMA_BUFFER_COUNT; i++) {
        uint64_t iova = v->rx_iova + (uint64_t)i * M2SDR_DMA_BUFFER_SIZE;
        write_descriptor(v, base,
                         M2SDR_DMA_WRITER_VALUE,
                         M2SDR_DMA_WRITER_WE,
                         iova, i);
    }

    /* Activate loop mode */
    mmio_write32(v->bar0, base + M2SDR_DMA_WRITER_LOOP_PROG_N, 1);

    /* Reset software counters */
    v->rx_hw_count         = 0;
    v->rx_sw_count         = 0;
    v->rx_hw_count_last_raw = 0;
    v->rx_active           = true;

    /* Enable engine */
    mmio_write32(v->bar0, base + M2SDR_DMA_WRITER_ENABLE, 1);

    /* Enable DMA synchronizer (RX only, bit 1) — mirrors kernel behaviour */
    mmio_write32(v->bar0, base + M2SDR_DMA_SYNC_ENABLE, 0x0);
    mmio_write32(v->bar0, base + M2SDR_DMA_SYNC_ENABLE, 0x2);

    printf("[vfio] RX DMA started (%d descriptors)\n", M2SDR_DMA_BUFFER_COUNT);
    return 0;
}

void m2sdr_vfio_rx_stop(struct m2sdr_vfio *v)
{
    uint32_t base = M2SDR_CSR_DMA0_BASE;

    mmio_write32(v->bar0, base + M2SDR_DMA_WRITER_LOOP_PROG_N, 0);
    mmio_write32(v->bar0, base + M2SDR_DMA_WRITER_FLUSH,       1);
    usleep(1000);
    mmio_write32(v->bar0, base + M2SDR_DMA_WRITER_ENABLE,      0);
    mmio_write32(v->bar0, base + M2SDR_DMA_WRITER_FLUSH,       1);

    /* Disable synchronizer only if TX is also stopped */
    if (!v->tx_active) {
        mmio_write32(v->bar0, base + M2SDR_DMA_SYNC_BYPASS, 0x0);
        mmio_write32(v->bar0, base + M2SDR_DMA_SYNC_ENABLE, 0x0);
    }

    v->rx_active   = false;
    v->rx_hw_count = 0;
    v->rx_sw_count = 0;
    printf("[vfio] RX DMA stopped\n");
}

/* -------------------------------------------------------------------------
 * Public: TX ring start / stop  (DMA Reader: host → FPGA)
 * ---------------------------------------------------------------------- */

int m2sdr_vfio_tx_start(struct m2sdr_vfio *v)
{
    uint32_t base = M2SDR_CSR_DMA0_BASE;
    int i;

    /* Disable engine and flush */
    mmio_write32(v->bar0, base + M2SDR_DMA_READER_ENABLE,      0);
    mmio_write32(v->bar0, base + M2SDR_DMA_READER_FLUSH,       1);
    mmio_write32(v->bar0, base + M2SDR_DMA_READER_LOOP_PROG_N, 0);

    /* Write all 256 descriptors */
    for (i = 0; i < M2SDR_DMA_BUFFER_COUNT; i++) {
        uint64_t iova = v->tx_iova + (uint64_t)i * M2SDR_DMA_BUFFER_SIZE;
        write_descriptor(v, base,
                         M2SDR_DMA_READER_VALUE,
                         M2SDR_DMA_READER_WE,
                         iova, i);
    }

    /* Activate loop mode */
    mmio_write32(v->bar0, base + M2SDR_DMA_READER_LOOP_PROG_N, 1);

    /* Reset software counters */
    v->tx_hw_count          = 0;
    v->tx_sw_count          = 0;
    v->tx_hw_count_last_raw = 0;
    v->tx_active            = true;

    /* Enable engine */
    mmio_write32(v->bar0, base + M2SDR_DMA_READER_ENABLE, 1);

    /* Enable DMA synchronizer (TX+RX, bit 0) */
    mmio_write32(v->bar0, base + M2SDR_DMA_SYNC_ENABLE, 0x0);
    mmio_write32(v->bar0, base + M2SDR_DMA_SYNC_ENABLE, 0x1);

    printf("[vfio] TX DMA started (%d descriptors)\n", M2SDR_DMA_BUFFER_COUNT);
    return 0;
}

void m2sdr_vfio_tx_stop(struct m2sdr_vfio *v)
{
    uint32_t base = M2SDR_CSR_DMA0_BASE;

    mmio_write32(v->bar0, base + M2SDR_DMA_READER_LOOP_PROG_N, 0);
    mmio_write32(v->bar0, base + M2SDR_DMA_READER_FLUSH,       1);
    usleep(1000);
    mmio_write32(v->bar0, base + M2SDR_DMA_READER_ENABLE,      0);
    mmio_write32(v->bar0, base + M2SDR_DMA_READER_FLUSH,       1);

    if (!v->rx_active) {
        mmio_write32(v->bar0, base + M2SDR_DMA_SYNC_BYPASS, 0x0);
        mmio_write32(v->bar0, base + M2SDR_DMA_SYNC_ENABLE, 0x0);
    }

    v->tx_active   = false;
    v->tx_hw_count = 0;
    v->tx_sw_count = 0;
    printf("[vfio] TX DMA stopped\n");
}
