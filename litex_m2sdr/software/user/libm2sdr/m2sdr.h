/* SPDX-License-Identifier: BSD-2-Clause
 *
 * LiteX-M2SDR public device API
 *
 * This file provides the public libm2sdr surface used by the transport-aware
 * device layer and the Soapy integration.
 */

#ifndef M2SDR_H
#define M2SDR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "../libm2sdr_vfio/m2sdr_vfio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define M2SDR_API_VERSION    1
#define M2SDR_ABI_VERSION    1
#define M2SDR_VERSION_STRING "0.1"

#define M2SDR_DEVICE_STR_MAX 256
#define M2SDR_IDENT_MAX      256

enum m2sdr_error {
    M2SDR_ERR_OK = 0,
    M2SDR_ERR_UNEXPECTED = -1,
    M2SDR_ERR_INVAL = -2,
    M2SDR_ERR_IO = -3,
    M2SDR_ERR_TIMEOUT = -4,
    M2SDR_ERR_NO_MEM = -5,
    M2SDR_ERR_UNSUPPORTED = -6,
    M2SDR_ERR_PARSE = -7,
    M2SDR_ERR_RANGE = -8,
    M2SDR_ERR_STATE = -9,
};

enum m2sdr_format {
    M2SDR_FORMAT_CS16 = 0,
    M2SDR_FORMAT_CS8 = 1,
};

enum m2sdr_transport_kind {
    M2SDR_TRANSPORT_KIND_UNKNOWN = 0,
    M2SDR_TRANSPORT_KIND_LITEPCIE = 1,
    M2SDR_TRANSPORT_KIND_LITEETH = 2,
    M2SDR_TRANSPORT_KIND_VFIO = 3,
};

struct m2sdr_dev;

struct m2sdr_version {
    uint32_t api;
    uint32_t abi;
    const char *version_str;
};

struct m2sdr_devinfo {
    char transport[32];
    char path[M2SDR_DEVICE_STR_MAX];
    char serial[32];
    char identification[M2SDR_IDENT_MAX];
};

struct m2sdr_capabilities {
    uint32_t api_version;
    uint32_t features;
    uint32_t board_info;
    uint32_t pcie_config;
    uint32_t eth_config;
    uint32_t sata_config;
};

struct m2sdr_clock_info {
    uint32_t reserved;
};

struct m2sdr_fpga_sensors {
    double temperature_c;
    double vccint_v;
    double vccaux_v;
    double vccbram_v;
};

const char *m2sdr_strerror(int err);
void m2sdr_get_version(struct m2sdr_version *ver);
int m2sdr_open(struct m2sdr_dev **dev_out, const char *device_identifier);
void m2sdr_close(struct m2sdr_dev *dev);

int m2sdr_reg_read(struct m2sdr_dev *dev, uint32_t addr, uint32_t *val);
int m2sdr_reg_write(struct m2sdr_dev *dev, uint32_t addr, uint32_t val);
struct m2sdr_vfio *m2sdr_get_vfio(struct m2sdr_dev *dev);
int m2sdr_get_fd(struct m2sdr_dev *dev);
void *m2sdr_get_eb_handle(struct m2sdr_dev *dev);
int m2sdr_get_transport(struct m2sdr_dev *dev, enum m2sdr_transport_kind *transport);
void *m2sdr_get_handle(struct m2sdr_dev *dev);

int m2sdr_get_device_info(struct m2sdr_dev *dev, struct m2sdr_devinfo *info);
int m2sdr_get_device_list(struct m2sdr_devinfo *list, size_t max, size_t *count);
int m2sdr_get_capabilities(struct m2sdr_dev *dev, struct m2sdr_capabilities *caps);
int m2sdr_get_identifier(struct m2sdr_dev *dev, char *buf, size_t len);
int m2sdr_get_fpga_git_hash(struct m2sdr_dev *dev, uint32_t *hash);
int m2sdr_get_clock_info(struct m2sdr_dev *dev, struct m2sdr_clock_info *info);
int m2sdr_get_time(struct m2sdr_dev *dev, uint64_t *time_ns);
int m2sdr_set_time(struct m2sdr_dev *dev, uint64_t time_ns);
int m2sdr_get_fpga_dna(struct m2sdr_dev *dev, uint64_t *dna);
int m2sdr_get_fpga_sensors(struct m2sdr_dev *dev, struct m2sdr_fpga_sensors *sensors);

int m2sdr_set_bitmode(struct m2sdr_dev *dev, bool enable_8bit);
int m2sdr_set_dma_loopback(struct m2sdr_dev *dev, bool enable);
int m2sdr_set_rx_header(struct m2sdr_dev *dev, bool enable, bool strip_header);
int m2sdr_set_tx_header(struct m2sdr_dev *dev, bool enable);
int m2sdr_gpio_config(struct m2sdr_dev *dev, bool enable, bool loopback, bool source_csr);
int m2sdr_gpio_write(struct m2sdr_dev *dev, uint8_t value, uint8_t oe);
int m2sdr_gpio_read(struct m2sdr_dev *dev, uint8_t *value);

#ifdef __cplusplus
}
#endif

#endif /* M2SDR_H */
