/* SPDX-License-Identifier: BSD-2-Clause
 *
 * M2SDR VFIO User-Mode Driver
 * Public API header
 *
 * Replaces the litepcie kernel driver with a zero-syscall hot path via VFIO:
 *  - BAR0 mapped directly into user space (no ioctl for CSR access)
 *  - DMA buffers allocated as hugepages and registered with VFIO IOMMU
 *  - LOOP_STATUS register polled directly (no ioctl for hw_count)
 *  - MSI delivered to eventfd for adaptive sleep (no kernel interrupt handler)
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 */

#ifndef __M2SDR_VFIO_H
#define __M2SDR_VFIO_H

#include <stdint.h>
#include <stddef.h>

/* -------------------------------------------------------------------------
 * Constants (must stay in sync with kernel driver config.h / csr.h)
 * ---------------------------------------------------------------------- */

#define M2SDR_DMA_BUFFER_COUNT       256
#define M2SDR_DMA_BUFFER_SIZE        8192
#define M2SDR_DMA_BUFFER_TOTAL_SIZE  (M2SDR_DMA_BUFFER_COUNT * M2SDR_DMA_BUFFER_SIZE)
#define M2SDR_DMA_BUFFER_PER_IRQ     8

/* PCIe BAR0 register offsets (from csr.h / config.h) */
#define M2SDR_CSR_BASE               0x00000000UL
#define M2SDR_CSR_PCIE_MSI_ENABLE    0x5800UL
#define M2SDR_CSR_PCIE_MSI_CLEAR     0x5804UL
#define M2SDR_CSR_PCIE_MSI_VECTOR    0x5808UL

/* DMA0 base and offsets */
#define M2SDR_CSR_DMA0_BASE          0x6000UL
#define M2SDR_DMA_WRITER_ENABLE      0x0000UL
#define M2SDR_DMA_WRITER_VALUE       0x0004UL   /* flags|size word */
#define M2SDR_DMA_WRITER_VALUE_ADDR  0x0008UL   /* addr[31:0] */
#define M2SDR_DMA_WRITER_WE          0x000cUL   /* addr[63:32], commits entry */
#define M2SDR_DMA_WRITER_LOOP_PROG_N 0x0010UL
#define M2SDR_DMA_WRITER_LOOP_STATUS 0x0014UL
#define M2SDR_DMA_WRITER_LEVEL       0x0018UL
#define M2SDR_DMA_WRITER_FLUSH       0x001cUL
#define M2SDR_DMA_READER_ENABLE      0x0020UL
#define M2SDR_DMA_READER_VALUE       0x0024UL
#define M2SDR_DMA_READER_VALUE_ADDR  0x0028UL
#define M2SDR_DMA_READER_WE          0x002cUL
#define M2SDR_DMA_READER_LOOP_PROG_N 0x0030UL
#define M2SDR_DMA_READER_LOOP_STATUS 0x0034UL
#define M2SDR_DMA_READER_LEVEL       0x0038UL
#define M2SDR_DMA_READER_FLUSH       0x003cUL
#define M2SDR_DMA_LOOPBACK_ENABLE    0x0040UL
#define M2SDR_DMA_SYNC_BYPASS        0x0044UL
#define M2SDR_DMA_SYNC_ENABLE        0x0048UL

/* Descriptor flags */
#define M2SDR_DMA_IRQ_DISABLE        (1u << 24)
#define M2SDR_DMA_LAST_DISABLE       (1u << 25)

/* MSI interrupt bit assignments */
#define M2SDR_DMA0_READER_IRQ_BIT    0   /* TX */
#define M2SDR_DMA0_WRITER_IRQ_BIT    1   /* RX */

/* IOVA base addresses for DMA rings (arbitrary, non-overlapping) */
#define M2SDR_RX_IOVA_BASE           0x00000000ULL
#define M2SDR_TX_IOVA_BASE           0x00400000ULL  /* 4 MB offset */

/* -------------------------------------------------------------------------
 * Poll modes
 * ---------------------------------------------------------------------- */

#define M2SDR_POLL_BUSYPOLL  0   /* tight spin — lowest latency, 1 full core */
#define M2SDR_POLL_ADAPTIVE  1   /* spin then sleep on MSI eventfd (recommended) */
#define M2SDR_POLL_IRQ       2   /* block on MSI eventfd only */

/* -------------------------------------------------------------------------
 * Opaque handle
 * ---------------------------------------------------------------------- */

struct m2sdr_vfio;

/* -------------------------------------------------------------------------
 * Lifecycle
 * ---------------------------------------------------------------------- */

/*
 * Open the VFIO device, map BAR0, allocate DMA buffers, program descriptor
 * rings, set up MSI eventfd (for ADAPTIVE/IRQ modes).
 *
 * pci_addr: "0000:01:00.0" format
 * poll_mode: M2SDR_POLL_*
 *
 * Returns handle on success, NULL on error (errno set).
 */
struct m2sdr_vfio *m2sdr_vfio_open(const char *pci_addr, int poll_mode);

/* Stop DMA, unmap BAR0, free DMA buffers, close VFIO fds. */
void m2sdr_vfio_close(struct m2sdr_vfio *v);

/* -------------------------------------------------------------------------
 * CSR access  (replaces litepcie_readl / litepcie_writel)
 * ---------------------------------------------------------------------- */

uint32_t m2sdr_vfio_readl(struct m2sdr_vfio *v, uint32_t csr_addr);
void     m2sdr_vfio_writel(struct m2sdr_vfio *v, uint32_t csr_addr, uint32_t val);

/* -------------------------------------------------------------------------
 * RX streaming  (DMA Writer engine: FPGA → host)
 * ---------------------------------------------------------------------- */

int   m2sdr_vfio_rx_start(struct m2sdr_vfio *v);
void  m2sdr_vfio_rx_stop(struct m2sdr_vfio *v);

/*
 * Acquire the next filled RX buffer.
 * Blocks (or spins) according to poll_mode until a buffer is ready.
 * Returns pointer to buffer data; sets *handle to the buffer index.
 * Returns NULL on error.
 */
void *m2sdr_vfio_rx_acquire(struct m2sdr_vfio *v, int *handle);

/* Mark buffer as consumed, allowing the DMA engine to refill it. */
void  m2sdr_vfio_rx_release(struct m2sdr_vfio *v, int handle);

/* Raw hw_count (total buffers filled by hardware since rx_start). */
int64_t m2sdr_vfio_rx_hw_count(struct m2sdr_vfio *v);

/*
 * Wait up to timeout_us for at least one new filled RX buffer.
 * Writes the new hw_count into *hw_count_out.
 * Returns number of available buffers (hw_count - sw_count_hint), 0 on
 * timeout, < 0 on error.
 */
int m2sdr_vfio_rx_wait(struct m2sdr_vfio *v, int64_t *hw_count_out,
                        int64_t sw_count_hint, long timeout_us);

/*
 * Advance the internal rx_sw_count to `count` (used after overflow drain
 * to keep the VFIO internal state consistent with the application's view).
 */
void m2sdr_vfio_rx_sw_set(struct m2sdr_vfio *v, int64_t count);

/* -------------------------------------------------------------------------
 * TX streaming  (DMA Reader engine: host → FPGA)
 * ---------------------------------------------------------------------- */

int   m2sdr_vfio_tx_start(struct m2sdr_vfio *v);
void  m2sdr_vfio_tx_stop(struct m2sdr_vfio *v);

/*
 * Acquire the next free TX buffer.
 * Blocks (or spins) until a buffer slot is available.
 * Returns pointer to buffer; sets *handle.
 */
void *m2sdr_vfio_tx_acquire(struct m2sdr_vfio *v, int *handle);

/*
 * Submit a filled TX buffer.
 * n_bytes: number of bytes written (padded to DMA_BUFFER_SIZE internally).
 * time_ns: scheduled TX time in nanoseconds (0 = immediate).
 * flags:   reserved, pass 0.
 */
void  m2sdr_vfio_tx_release(struct m2sdr_vfio *v, int handle,
                             int n_bytes, uint64_t time_ns, int flags);

/* Raw hw_count (total buffers consumed by hardware since tx_start). */
int64_t m2sdr_vfio_tx_hw_count(struct m2sdr_vfio *v);

/*
 * Wait up to timeout_us for at least one free TX buffer slot.
 * pending = sw_count - hw_count.  Returns free slots available, 0 on timeout.
 */
int m2sdr_vfio_tx_wait(struct m2sdr_vfio *v, int64_t *hw_count_out,
                        int64_t sw_count_hint, long timeout_us);

#endif /* __M2SDR_VFIO_H */
