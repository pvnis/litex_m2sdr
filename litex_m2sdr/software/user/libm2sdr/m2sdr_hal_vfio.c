/* SPDX-License-Identifier: BSD-2-Clause
 *
 * LiteX-M2SDR library — VFIO HAL
 *
 * Routes m2sdr_hal_readl / m2sdr_hal_writel through direct BAR0 MMIO
 * via libm2sdr_vfio, replacing the ioctl-based litepcie path.
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 */

#include "m2sdr_internal.h"

#ifdef USE_VFIO

int m2sdr_hal_readl(struct m2sdr_dev *dev, uint32_t addr, uint32_t *val)
{
    if (!dev || !val || !dev->vfio)
        return M2SDR_ERR_INVAL;
    *val = m2sdr_vfio_readl(dev->vfio, addr);
    return M2SDR_ERR_OK;
}

int m2sdr_hal_writel(struct m2sdr_dev *dev, uint32_t addr, uint32_t val)
{
    if (!dev || !dev->vfio)
        return M2SDR_ERR_INVAL;
    m2sdr_vfio_writel(dev->vfio, addr, val);
    return M2SDR_ERR_OK;
}

#endif /* USE_VFIO */
