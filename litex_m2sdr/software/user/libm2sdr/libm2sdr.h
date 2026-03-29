/*
 * LiteX-M2SDR library
 *
 * This file is part of LiteX-M2SDR.
 *
 * Copyright (c) 2024-2026 Enjoy-Digital <enjoy-digital.fr>
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef M2SDR_LIB_H
#define M2SDR_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Constants */

/* ICAP */
#define ICAP_CMD_REG   0b00100
#define ICAP_CMD_IPROG 0b01111

#define ICAP_IDCODE_REG   0b01100

#define ICAP_BOOTSTS_REG  0b10110
#define ICAP_BOOTSTS_VALID    (1 << 0)
#define ICAP_BOOTSTS_FALLBACK (1 << 1)

/* Macros */

#ifdef USE_LITEPCIE

#include "liblitepcie.h"

#define m2sdr_conn_type int
#define m2sdr_conn_cast(conn) ((m2sdr_conn_type)(intptr_t)(conn))
#define m2sdr_writel(conn, addr, val) litepcie_writel(m2sdr_conn_cast(conn), addr, val)
#define m2sdr_readl(conn, addr)       litepcie_readl(m2sdr_conn_cast(conn), addr)

#elif USE_LITEETH

#include "etherbone.h"

#define m2sdr_conn_type struct eb_connection *
#define m2sdr_conn_cast(conn) ((m2sdr_conn_type)(conn))
#define m2sdr_writel(conn, addr, val) eb_write32(m2sdr_conn_cast(conn), val, addr)
#define m2sdr_readl(conn, addr)       eb_read32(m2sdr_conn_cast(conn), addr)

#elif USE_VFIO

struct m2sdr_dev;
int m2sdr_reg_read(struct m2sdr_dev *dev, uint32_t addr, uint32_t *val);
int m2sdr_reg_write(struct m2sdr_dev *dev, uint32_t addr, uint32_t val);

#define m2sdr_conn_type struct m2sdr_dev *
#define m2sdr_conn_cast(conn) ((m2sdr_conn_type)(conn))
#define m2sdr_writel(conn, addr, val) do { (void)m2sdr_reg_write(m2sdr_conn_cast(conn), addr, val); } while (0)
#define m2sdr_readl(conn, addr) ({ uint32_t _v = 0; (void)m2sdr_reg_read(m2sdr_conn_cast(conn), addr, &_v); _v; })

#else

#error "Define USE_LITEPCIE, USE_LITEETH, or USE_VFIO for build configuration"

#endif

/* Libs */

#include "m2sdr_si5351_i2c.h"
#include "m2sdr_ad9361_spi.h"
#include "m2sdr_flash.h"

#ifdef __cplusplus
}
#endif

#endif /* M2SDR_LIB_H */
