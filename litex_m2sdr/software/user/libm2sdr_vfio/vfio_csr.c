/* SPDX-License-Identifier: BSD-2-Clause
 *
 * M2SDR VFIO User-Mode Driver — CSR read/write helpers
 *
 * Replaces litepcie_readl() / litepcie_writel() (which go via ioctl) with
 * direct MMIO loads/stores into the BAR0 mapping.
 *
 * The same csr.h address constants used by the kernel driver are reused
 * unchanged.  CSR_BASE is 0x00000000, so csr_addr == BAR0 byte offset.
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 */

#include "m2sdr_vfio.h"
#include "vfio_priv.h"

uint32_t m2sdr_vfio_readl(struct m2sdr_vfio *v, uint32_t csr_addr)
{
    return mmio_read32(v->bar0, csr_addr - M2SDR_CSR_BASE);
}

void m2sdr_vfio_writel(struct m2sdr_vfio *v, uint32_t csr_addr, uint32_t val)
{
    mmio_write32(v->bar0, csr_addr - M2SDR_CSR_BASE, val);
}
