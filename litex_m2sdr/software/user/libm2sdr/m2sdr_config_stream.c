/* SPDX-License-Identifier: BSD-2-Clause
 *
 * ICAP stream helper
 *
 * This file is part of LiteX-M2SDR.
 *
 * Copyright (C) 2024-2026 Enjoy-Digital
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include "csr.h"
#include "libm2sdr.h"
#include "m2sdr_flash.h"

#ifdef CSR_ICAP_BASE

static int wait_icap_done(void *conn)
{
    const int timeout_ms = 5000;
    const int poll_delay_us = 1000;
    int elapsed_ms = 0;

    while (!(m2sdr_readl(conn, CSR_ICAP_DONE_ADDR) & 1)) {
        if (elapsed_ms >= timeout_ms)
            return 1;
        usleep(poll_delay_us);
        elapsed_ms += poll_delay_us / 1000;
    }
    return 0;
}

int m2sdr_config_stream(void *conn,
                        uint8_t *buf, uint32_t size,
                        void (*progress_cb)(void *opaque, const char *fmt, ...),
                        void *opaque)
{
    uint32_t i = 0;
    const uint32_t step = 1024 * 1024;

    if (!buf || size == 0)
        return 1;

    /* Stream the buffer in 32-bit little-endian words. Pad trailing bytes
     * with 0x00 if not aligned to 4 bytes. */
    while (i < size) {
        uint32_t word = 0;
        for (int b = 0; b < 4; b++) {
            uint32_t idx = i + b;
            uint8_t v = (idx < size) ? buf[idx] : 0x00;
            word |= ((uint32_t)v) << (8 * b);
        }

        m2sdr_writel(conn, CSR_ICAP_ADDR_ADDR, ICAP_FDRI_REG);
        m2sdr_writel(conn, CSR_ICAP_DATA_ADDR, word);
        m2sdr_writel(conn, CSR_ICAP_WRITE_ADDR, 1);

        if (wait_icap_done(conn))
            return 1;

        m2sdr_writel(conn, CSR_ICAP_WRITE_ADDR, 0);

        i += 4;

        if (progress_cb && (i % step) == 0)
            progress_cb(opaque, "Streamed %u bytes\r", i);
    }

    if (progress_cb)
        progress_cb(opaque, "\n");

    return 0;
}

#else

int m2sdr_config_stream(void *conn,
                        uint8_t *buf, uint32_t size,
                        void (*progress_cb)(void *opaque, const char *fmt, ...),
                        void *opaque)
{
    (void)conn; (void)buf; (void)size; (void)progress_cb; (void)opaque;
    fprintf(stderr, "ICAP support not present in this gateware.\n");
    return 1;
}

#endif
