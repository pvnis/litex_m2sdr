/* SPDX-License-Identifier: BSD-2-Clause
 *
 * M2SDR Utility.
 *
 * This file is part of LiteX-M2SDR project.
 *
 * Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <inttypes.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>

#include "ad9361/util.h"
#include "ad9361/ad9361.h"

#include "liblitepcie.h"
#include "libm2sdr.h"

#include "m2sdr_config.h"

/* Parameters */
/*------------*/

#define DMA_CHECK_DATA   /* Un-comment to disable data check */
#define DMA_RANDOM_DATA  /* Un-comment to disable data random */

/* Variables */
/*-----------*/

static char litepcie_device[1024];
static int litepcie_device_num;

sig_atomic_t keep_running = 1;

void intHandler(int dummy) {
    keep_running = 0;
}

#ifdef CSR_SI5351_I2C_BASE

/* SI5351 */
/*--------*/

static void test_si5351_scan(void)
{
    int fd;

    fd = open(litepcie_device, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    printf("\e[1m[> SI53512 I2C Bus Scan:\e[0m\n");
    printf("-----------------------------\n");
    m2sdr_si5351_i2c_scan(fd);
    printf("\n");

    close(fd);
}

static void test_si5351_init(void)
{
    int fd;

    fd = open(litepcie_device, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    printf("\e[1m[> SI53512 Init...\e[0m\n");
    m2sdr_si5351_i2c_config(fd, SI5351_I2C_ADDR, si5351_config, sizeof(si5351_config)/sizeof(si5351_config[0]));
    printf("Done.\n");

    close(fd);
}

#endif

#ifdef CSR_CDCM6208_BASE

/* CDCM6208 Dump */
/*---------------*/

static void test_cdcm6208_dump(void)
{
    int i;
    int fd;

    fd = open(litepcie_device, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    /* CDCM6208 SPI Init */
    m2sdr_cdcm6208_spi_init(fd);

    /* CDCM6208 SPI Dump */
    for (i=0; i<128; i++)
        printf("Reg 0x%02x: 0x%04x\n", i, m2sdr_cdcm6208_spi_read(fd, i));

    printf("\n");

    close(fd);
}

/* CDCM6208 Init */
/*---------------*/

static void test_cdcm6208_init(void)
{
    int i;
    int fd;

    fd = open(litepcie_device, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    /* CDCM6208 SPI Init */
    m2sdr_cdcm6208_spi_init(fd);

    /* CDCM6208 SPI Init */
    for (i = 0; i < sizeof(cdcm6208_regs_vcxo_38p4) / sizeof(cdcm6208_regs_vcxo_38p4[0]); i++) {
        m2sdr_cdcm6208_spi_write(fd, cdcm6208_regs_vcxo_38p4[i][0], cdcm6208_regs_vcxo_38p4[i][1]);
    }

    printf("CDCM6208 SPI initialization completed.\n");

    close(fd);
}

#endif

/* AD9361 Dump */
/*-------------*/

static void test_ad9361_dump(void)
{
    int i;
    int fd;

    fd = open(litepcie_device, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    /* AD9361 SPI Init */
    m2sdr_ad9361_spi_init(fd);

    /* AD9361 SPI Dump */
    for (i=0; i<128; i++)
        printf("Reg 0x%02x: 0x%04x\n", i, m2sdr_ad9361_spi_read(fd, i));

    printf("\n");

    close(fd);
}

/* Info */
/*------*/

static void info(void)
{
    int fd;
    int i;
    unsigned char fpga_identifier[256];

    fd = open(litepcie_device, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }


    printf("\e[1m[> FPGA/SoC Info:\e[0m\n");
    printf("-----------------\n");

    for (i = 0; i < 256; i ++)
        fpga_identifier[i] = litepcie_readl(fd, CSR_IDENTIFIER_MEM_BASE + 4 * i);
    printf("SoC Identifier   : %s.\n", fpga_identifier);
#ifdef CSR_DNA_BASE
    printf("FPGA DNA         : 0x%08x%08x\n",
        litepcie_readl(fd, CSR_DNA_ID_ADDR + 4 * 0),
        litepcie_readl(fd, CSR_DNA_ID_ADDR + 4 * 1)
    );
#endif
#ifdef CSR_XADC_BASE
    printf("FPGA Temperature : %0.1f °C\n",
           (double)litepcie_readl(fd, CSR_XADC_TEMPERATURE_ADDR) * 503.975/4096 - 273.15);
    printf("FPGA VCC-INT     : %0.2f V\n",
           (double)litepcie_readl(fd, CSR_XADC_VCCINT_ADDR) / 4096 * 3);
    printf("FPGA VCC-AUX     : %0.2f V\n",
           (double)litepcie_readl(fd, CSR_XADC_VCCAUX_ADDR) / 4096 * 3);
    printf("FPGA VCC-BRAM    : %0.2f V\n",
           (double)litepcie_readl(fd, CSR_XADC_VCCBRAM_ADDR) / 4096 * 3);
#endif
    printf("\n");

    printf("\e[1m[> AD9361 Info:\e[0m\n");
    printf("---------------\n");

    printf("AD9361 Product ID  : %04x \n", m2sdr_ad9361_spi_read(fd, REG_PRODUCT_ID));
    printf("AD9361 Temperature : %0.1f °C\n",
        (double)DIV_ROUND_CLOSEST(m2sdr_ad9361_spi_read(fd, REG_TEMPERATURE) * 1000000, 1140)/1000);

    close(fd);
}

/* Scratch */
/*---------*/

void scratch_test(void)
{
    int fd;

    printf("\e[1m[> Scratch register test:\e[0m\n");
    printf("-------------------------\n");

    /* Open LitePCIe device. */
    fd = open(litepcie_device, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    /* Write to scratch register. */
    printf("Write 0x12345678 to Scratch register:\n");
    litepcie_writel(fd, CSR_CTRL_SCRATCH_ADDR, 0x12345678);
    printf("Read: 0x%08x\n", litepcie_readl(fd, CSR_CTRL_SCRATCH_ADDR));

    /* Read from scratch register. */
    printf("Write 0xdeadbeef to Scratch register:\n");
    litepcie_writel(fd, CSR_CTRL_SCRATCH_ADDR, 0xdeadbeef);
    printf("Read: 0x%08x\n", litepcie_readl(fd, CSR_CTRL_SCRATCH_ADDR));

    /* Close LitePCIe device. */
    close(fd);
}

/* SPI Flash */
/*-----------*/

#ifdef CSR_FLASH_BASE

static void flash_progress(void *opaque, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    fflush(stdout);
    va_end(ap);
}

static void flash_program(uint32_t base, const uint8_t *buf1, int size1)
{
    int fd;

    uint32_t size;
    uint8_t *buf;
    int sector_size;
    int errors;

    /* Open LitePCIe device. */
    fd = open(litepcie_device, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    /* Get flash sector size and pad size to it. */
    sector_size = litepcie_flash_get_erase_block_size(fd);
    size = ((size1 + sector_size - 1) / sector_size) * sector_size;

    /* Alloc buffer and copy data to it. */
    buf = calloc(1, size);
    if (!buf) {
        fprintf(stderr, "%d: alloc failed\n", __LINE__);
        exit(1);
    }
    memcpy(buf, buf1, size1);

    /* Program flash. */
    printf("Programming (%d bytes at 0x%08x)...\n", size, base);
    errors = litepcie_flash_write(fd, buf, base, size, flash_progress, NULL);
    if (errors) {
        printf("Failed %d errors.\n", errors);
        exit(1);
    } else {
        printf("Success.\n");
    }

    /* Free buffer and close LitePCIe device. */
    free(buf);
    close(fd);
}

static void flash_write(const char *filename, uint32_t offset)
{
    uint8_t *data;
    int size;
    FILE * f;

    /* Open data source file. */
    f = fopen(filename, "rb");
    if (!f) {
        perror(filename);
        exit(1);
    }

    /* Get size, alloc buffer and copy data to it. */
    fseek(f, 0L, SEEK_END);
    size = ftell(f);
    fseek(f, 0L, SEEK_SET);
    data = malloc(size);
    if (!data) {
        fprintf(stderr, "%d: malloc failed\n", __LINE__);
        exit(1);
    }
    ssize_t ret = fread(data, size, 1, f);
    fclose(f);

    /* Program file to flash */
    if (ret != 1)
        perror(filename);
    else
        flash_program(offset, data, size);

    /* Free buffer */
    free(data);
}

static void flash_read(const char *filename, uint32_t size, uint32_t offset)
{
    int fd;
    FILE * f;
    uint32_t base;
    uint32_t sector_size;
    uint8_t byte;
    int i;

    /* Open data destination file. */
    f = fopen(filename, "wb");
    if (!f) {
        perror(filename);
        exit(1);
    }

    /* Open LitePCIe device. */
    fd = open(litepcie_device, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    /* Get flash sector size. */
    sector_size = litepcie_flash_get_erase_block_size(fd);

    /* Read flash and write to destination file. */
    base = offset;
    for (i = 0; i < size; i++) {
        if ((i % sector_size) == 0) {
            printf("Reading 0x%08x\r", base + i);
            fflush(stdout);
        }
        byte = litepcie_flash_read(fd, base + i);
        fwrite(&byte, 1, 1, f);
    }

    /* Close destination file and LitePCIe device. */
    fclose(f);
    close(fd);
}

static void flash_reload(void)
{
    int fd;

    /* Open LitePCIe device. */
    fd = open(litepcie_device, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    /* Reload FPGA through ICAP.*/
    litepcie_reload(fd);

    /* Notice user to reboot the hardware.*/
    printf("================================================================\n");
    printf("= PLEASE REBOOT YOUR HARDWARE TO START WITH NEW FPGA GATEWARE  =\n");
    printf("================================================================\n");

    /* Close LitePCIe device. */
    close(fd);
}
#endif

/* DMA */
/*-----*/

static inline int64_t add_mod_int(int64_t a, int64_t b, int64_t m)
{
    a += b;
    if (a >= m)
        a -= m;
    return a;
}

static int get_next_pow2(int data_width)
{
    int x = 1;
    while (x < data_width)
        x <<= 1;
    return x;
}

#ifdef DMA_CHECK_DATA

static inline uint32_t seed_to_data(uint32_t seed)
{
#ifdef DMA_RANDOM_DATA
    /* Return pseudo random data from seed. */
    return seed * 69069 + 1;
#else
    /* Return seed. */
    return seed;
#endif
}

static uint32_t get_data_mask(int data_width)
{
    int i;
    uint32_t mask;
    mask = 0;
    for (i = 0; i < 32/get_next_pow2(data_width); i++) {
        mask <<= get_next_pow2(data_width);
        mask |= (1 << data_width) - 1;
    }
    return mask;
}

static void write_pn_data(uint32_t *buf, int count, uint32_t *pseed, int data_width)
{
    int i;
    uint32_t seed;
    uint32_t mask = get_data_mask(data_width);

    seed = *pseed;
    for(i = 0; i < count; i++) {
        buf[i] = (seed_to_data(seed) & mask);
        seed = add_mod_int(seed, 1, DMA_BUFFER_SIZE / sizeof(uint32_t));
    }
    *pseed = seed;
}

static int check_pn_data(const uint32_t *buf, int count, uint32_t *pseed, int data_width)
{
    int i, errors;
    uint32_t seed;
    uint32_t mask = get_data_mask(data_width);

    errors = 0;
    seed = *pseed;
    for (i = 0; i < count; i++) {
        if (buf[i] != (seed_to_data(seed) & mask)) {
            errors ++;
        }
        seed = add_mod_int(seed, 1, DMA_BUFFER_SIZE / sizeof(uint32_t));
    }
    *pseed = seed;
    return errors;
}
#endif

static void dma_test(uint8_t zero_copy, uint8_t external_loopback, int data_width, int auto_rx_delay)
{
    static struct litepcie_dma_ctrl dma = {.use_reader = 1, .use_writer = 1};
    dma.loopback = external_loopback ? 0 : 1;

    if (data_width > 32 || data_width < 1) {
        fprintf(stderr, "Invalid data width %d\n", data_width);
        exit(1);
    }

    /* Statistics */
    int i = 0;
    int64_t reader_sw_count_last = 0;
    int64_t last_time;
    uint32_t errors = 0;

#ifdef DMA_CHECK_DATA
    uint32_t seed_wr = 0;
    uint32_t seed_rd = 0;
    uint8_t  run = (auto_rx_delay == 0);
#else
    uint8_t run = 1;
#endif

    signal(SIGINT, intHandler);

    printf("\e[1m[> DMA loopback test:\e[0m\n");
    printf("---------------------\n");

    if (litepcie_dma_init(&dma, litepcie_device, zero_copy))
        exit(1);

    dma.reader_enable = 1;
    dma.writer_enable = 1;

    /* Test loop. */
    last_time = get_time_ms();
    for (;;) {
        /* Exit loop on CTRL+C. */
        if (!keep_running)
            break;

        /* Update DMA status. */
        litepcie_dma_process(&dma);

#ifdef DMA_CHECK_DATA
        char *buf_wr;
        char *buf_rd;

        /* DMA-TX Write. */
        while (1) {
            /* Get Write buffer. */
            buf_wr = litepcie_dma_next_write_buffer(&dma);
            /* Break when no buffer available for Write. */
            if (!buf_wr)
                break;
            /* Write data to buffer. */
            write_pn_data((uint32_t *) buf_wr, DMA_BUFFER_SIZE / sizeof(uint32_t), &seed_wr, data_width);
        }

        /* DMA-RX Read/Check */
        while (1) {
            /* Get Read buffer. */
            buf_rd = litepcie_dma_next_read_buffer(&dma);
            /* Break when no buffer available for Read. */
            if (!buf_rd)
                break;
            /* Skip the first 128 DMA loops. */
            if (dma.writer_hw_count < 128*DMA_BUFFER_COUNT)
                break;
            /* When running... */
            if (run) {
                /* Check data in Read buffer. */
                errors += check_pn_data((uint32_t *) buf_rd, DMA_BUFFER_SIZE / sizeof(uint32_t), &seed_rd, data_width);
                /* Clear Read buffer */
                memset(buf_rd, 0, DMA_BUFFER_SIZE);
            } else {
                /* Find initial Delay/Seed (Useful when loopback is introducing delay). */
                uint32_t errors_min = 0xffffffff;
                for (int delay = 0; delay < DMA_BUFFER_SIZE / sizeof(uint32_t); delay++) {
                    seed_rd = delay;
                    errors = check_pn_data((uint32_t *) buf_rd, DMA_BUFFER_SIZE / sizeof(uint32_t), &seed_rd, data_width);
                    //printf("delay: %d / errors: %d\n", delay, errors);
                    if (errors < errors_min)
                        errors_min = errors;
                    if (errors < (DMA_BUFFER_SIZE / sizeof(uint32_t)) / 2) {
                        printf("RX_DELAY: %d (errors: %d)\n", delay, errors);
                        run = 1;
                        break;
                    }
                }
                if (!run) {
                    printf("Unable to find DMA RX_DELAY (min errors: %d/%ld), exiting.\n",
                        errors_min,
                        DMA_BUFFER_SIZE / sizeof(uint32_t));
                    goto end;
                }
            }

        }
#endif

        /* Statistics every 200ms. */
        int64_t duration = get_time_ms() - last_time;
        if (run & (duration > 200)) {
            /* Print banner every 10 lines. */
            if (i % 10 == 0)
                printf("\e[1mDMA_SPEED(Gbps)\tTX_BUFFERS\tRX_BUFFERS\tDIFF\tERRORS\e[0m\n");
            i++;
            /* Print statistics. */
            printf("%14.2f\t%10" PRIu64 "\t%10" PRIu64 "\t%4" PRIu64 "\t%6u\n",
                   (double)(dma.reader_sw_count - reader_sw_count_last) * DMA_BUFFER_SIZE * 8 * data_width / (get_next_pow2(data_width) * (double)duration * 1e6),
                   dma.reader_sw_count,
                   dma.writer_sw_count,
                   (uint64_t) abs(dma.reader_sw_count - dma.writer_sw_count),
                   errors);
            /* Update errors/time/count. */
            errors = 0;
            last_time = get_time_ms();
            reader_sw_count_last = dma.reader_sw_count;
        }
    }


    /* Cleanup DMA. */
#ifdef DMA_CHECK_DATA
end:
#endif
    litepcie_dma_cleanup(&dma);
}

/* Clk Measurement */
/*-----------------*/

static void clk_measurement_test(int num_measurements, int delay_between_tests)
{
    int fd;
    int i;
    int64_t start_time, current_time, elapsed_time;
    uint32_t previous_values[4];
    uint32_t current_values[4];

    /* Open LitePCIe device. */
    fd = open(litepcie_device, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Could not init driver\n");
        exit(1);
    }

    printf("\e[1m[> Clk Measurement Test:\e[0m\n");
    printf("-------------------------\n");

    /* Latch and read initial values for each clock */
    litepcie_writel(fd, CSR_CLK_MEASUREMENT_CLK0_LATCH_ADDR, 1);
    litepcie_writel(fd, CSR_CLK_MEASUREMENT_CLK1_LATCH_ADDR, 1);
    litepcie_writel(fd, CSR_CLK_MEASUREMENT_CLK2_LATCH_ADDR, 1);
    litepcie_writel(fd, CSR_CLK_MEASUREMENT_CLK3_LATCH_ADDR, 1);
    previous_values[0] = litepcie_readl(fd, CSR_CLK_MEASUREMENT_CLK0_VALUE_ADDR);
    previous_values[1] = litepcie_readl(fd, CSR_CLK_MEASUREMENT_CLK1_VALUE_ADDR);
    previous_values[2] = litepcie_readl(fd, CSR_CLK_MEASUREMENT_CLK2_VALUE_ADDR);
    previous_values[3] = litepcie_readl(fd, CSR_CLK_MEASUREMENT_CLK3_VALUE_ADDR);
    start_time = time(NULL);

    printf("%d\n", previous_values[3]);

    for (i = 0; i < num_measurements; i++) {
        sleep(delay_between_tests);

        /* Latch and read current values for each clock */
        litepcie_writel(fd, CSR_CLK_MEASUREMENT_CLK0_LATCH_ADDR, 1);
        litepcie_writel(fd, CSR_CLK_MEASUREMENT_CLK1_LATCH_ADDR, 1);
        litepcie_writel(fd, CSR_CLK_MEASUREMENT_CLK2_LATCH_ADDR, 1);
        litepcie_writel(fd, CSR_CLK_MEASUREMENT_CLK3_LATCH_ADDR, 1);
        current_values[0] = litepcie_readl(fd, CSR_CLK_MEASUREMENT_CLK0_VALUE_ADDR);
        current_values[1] = litepcie_readl(fd, CSR_CLK_MEASUREMENT_CLK1_VALUE_ADDR);
        current_values[2] = litepcie_readl(fd, CSR_CLK_MEASUREMENT_CLK2_VALUE_ADDR);
        current_values[3] = litepcie_readl(fd, CSR_CLK_MEASUREMENT_CLK3_VALUE_ADDR);
        current_time = time(NULL);

        printf("%d\n", current_values[3]);

        /* Calculate the actual elapsed time */
        elapsed_time = current_time - start_time;
        start_time = current_time;  // Update the start_time for the next iteration

        for (int clk_index = 0; clk_index < 4; clk_index++) {
            /* Compute the difference between the current and previous values */
            uint32_t delta_value = current_values[clk_index] - previous_values[clk_index];
            double frequency_mhz = delta_value / (elapsed_time * 1e6);
            printf("Measurement %d, Clock %d: Frequency: %.2f MHz\n", i + 1, clk_index, frequency_mhz);

            /* Update the previous value for the next iteration */
            previous_values[clk_index] = current_values[clk_index];
        }
    }

    close(fd);
}

/* Help */
/*------*/

static void help(void)
{
    printf("M2SDR utilities\n"
           "usage: m2sdr_util [options] cmd [args...]\n"
           "\n"
           "options:\n"
           "-h                                Help.\n"
           "-c device_num                     Select the device (default = 0).\n"
           "-z                                Enable zero-copy DMA mode.\n"
           "-e                                Use external loopback (default = internal).\n"
           "-w data_width                     Width of data bus (default = 32).\n"
           "-a                                Automatic DMA RX-Delay calibration.\n"
           "\n"
           "available commands:\n"
           "info                              Get Board information.\n"
           "\n"
           "dma_test                          Test DMA.\n"
           "scratch_test                      Test Scratch register.\n"
           "clks                              Test Clks frequencies.\n"
           "\n"
#ifdef  CSR_SI5351_I2C_BASE
           "si5351_scan                       Scan SI5351 I2C Bus.\n"
           "si5351_init                       Init SI5351.\n"
           "\n"
#endif
#ifdef CSR_CDCM6208_BASE
           "cdcm6208_dump                     Dump CDCM6208 Registers.\n"
           "cdcm6208_init                     Init CDCM6208.\n"
           "\n"
#endif
           "ad9361_dump                       Dump AD9361 Registers.\n"
           "\n"
#ifdef CSR_FLASH_BASE
           "flash_write filename [offset]     Write file contents to SPI Flash.\n"
           "flash_read filename size [offset] Read from SPI Flash and write contents to file.\n"
           "flash_reload                      Reload FPGA Image.\n"
#endif
           );
    exit(1);
}

/* Main */
/*------*/

int main(int argc, char **argv)
{
    const char *cmd;
    int c;
    static uint8_t litepcie_device_zero_copy;
    static uint8_t litepcie_device_external_loopback;
    static int litepcie_data_width;
    static int litepcie_auto_rx_delay;

    litepcie_device_num = 0;
    litepcie_data_width = 32;
    litepcie_auto_rx_delay = 0;
    litepcie_device_zero_copy = 0;
    litepcie_device_external_loopback = 0;

    /* Parameters. */
    for (;;) {
        c = getopt(argc, argv, "hc:w:zea");
        if (c == -1)
            break;
        switch(c) {
        case 'h':
            help();
            break;
        case 'c':
            litepcie_device_num = atoi(optarg);
            break;
        case 'w':
            litepcie_data_width = atoi(optarg);
            break;
        case 'z':
            litepcie_device_zero_copy = 1;
            break;
        case 'e':
            litepcie_device_external_loopback = 1;
            break;
        case 'a':
            litepcie_auto_rx_delay = 1;
            break;
        default:
            exit(1);
        }
    }

    /* Show help when too much args. */
    if (optind >= argc)
        help();

    /* Select device. */
    snprintf(litepcie_device, sizeof(litepcie_device), "/dev/m2sdr%d", litepcie_device_num);

    cmd = argv[optind++];

    /* Info cmds. */
    if (!strcmp(cmd, "info"))
        info();

    /* Scratch cmds. */
    else if (!strcmp(cmd, "scratch_test"))
        scratch_test();

    /* Clks measurement cmds. */
    else if (!strcmp(cmd, "clks")) {
        int num_measurements = 10;
        int delay_between_tests = 1;

        if (optind < argc)
            num_measurements = atoi(argv[optind++]);
        if (optind < argc)
            delay_between_tests = atoi(argv[optind++]);

        clk_measurement_test(num_measurements, delay_between_tests);
    }

    /* SI5351 cmds. */
#ifdef CSR_SI5351_I2C_BASE
    else if (!strcmp(cmd, "si5351_scan"))
        test_si5351_scan();
    else if (!strcmp(cmd, "si5351_init"))
        test_si5351_init();
#endif

    /* CDCM6208 cmds. */
#ifdef CSR_CDCM6208_BASE
    else if (!strcmp(cmd, "cdcm6208_dump"))
        test_cdcm6208_dump();
    else if (!strcmp(cmd, "cdcm6208_init"))
        test_cdcm6208_init();
#endif

    /* AD9361 cmds. */
    else if (!strcmp(cmd, "ad9361_dump"))
        test_ad9361_dump();

    /* SPI Flash cmds. */
#if CSR_FLASH_BASE
    else if (!strcmp(cmd, "flash_write")) {
        const char *filename;
        uint32_t offset = CONFIG_FLASH_IMAGE_SIZE;  /* Operational */
        if (optind + 1 > argc)
            goto show_help;
        filename = argv[optind++];
        if (optind < argc)
            offset = strtoul(argv[optind++], NULL, 0);
        flash_write(filename, offset);
    }
    else if (!strcmp(cmd, "flash_read")) {
        const char *filename;
        uint32_t size = 0;
        uint32_t offset = CONFIG_FLASH_IMAGE_SIZE; /* Operational */
        if (optind + 2 > argc)
            goto show_help;
        filename = argv[optind++];
        size = strtoul(argv[optind++], NULL, 0);
        if (optind < argc)
            offset = strtoul(argv[optind++], NULL, 0);
        flash_read(filename, size, offset);
    }
    else if (!strcmp(cmd, "flash_reload"))
        flash_reload();
#endif

    /* DMA cmds. */
    else if (!strcmp(cmd, "dma_test"))
        dma_test(
            litepcie_device_zero_copy,
            litepcie_device_external_loopback,
            litepcie_data_width,
            litepcie_auto_rx_delay);

    /* Show help otherwise. */
    else
        goto show_help;

    return 0;

show_help:
        help();

    return 0;
}
