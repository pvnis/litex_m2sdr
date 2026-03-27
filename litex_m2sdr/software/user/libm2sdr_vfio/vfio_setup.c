/* SPDX-License-Identifier: BSD-2-Clause
 *
 * M2SDR VFIO User-Mode Driver — device open, BAR0 mmap, DMA allocation
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/eventfd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <linux/vfio.h>

#include "m2sdr_vfio.h"
#include "vfio_priv.h"

/* -------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------- */

/* Read a sysfs symlink target and return the final path component as a
 * string.  Used to find the IOMMU group number from the PCI device path. */
static int sysfs_read_link_basename(const char *link_path, char *out, size_t out_sz)
{
    char target[256];
    ssize_t n = readlink(link_path, target, sizeof(target) - 1);
    if (n < 0)
        return -1;
    target[n] = '\0';

    /* basename (last component after '/') */
    char *base = strrchr(target, '/');
    base = base ? base + 1 : target;
    /* stpncpy pads with NULs and avoids the -Wstringop-truncation false
     * positive that strncpy triggers on older gcc versions. */
    size_t blen = strlen(base);
    size_t copy = blen < out_sz - 1 ? blen : out_sz - 1;
    memcpy(out, base, copy);
    out[copy] = '\0';
    return 0;
}

/* Resolve the IOMMU group number for a PCI address like "0000:01:00.0". */
static int vfio_get_group(const char *pci_addr)
{
    char link[256], group_str[64];
    snprintf(link, sizeof(link),
             "/sys/bus/pci/devices/%s/iommu_group", pci_addr);
    if (sysfs_read_link_basename(link, group_str, sizeof(group_str)) < 0) {
        fprintf(stderr, "[vfio] cannot read iommu_group for %s: %s\n",
                pci_addr, strerror(errno));
        return -1;
    }
    return atoi(group_str);
}

/* Allocate a DMA buffer.  Try 2 MiB hugepages first; fall back to 4K pages.
 * Both work with VFIO IOMMU — hugepages just reduce TLB pressure. */
static void *alloc_dma_buf(size_t size, bool *used_hugepages)
{
    void *ptr;

    /* Try 2 MiB hugepages */
    ptr = mmap(NULL, size,
               PROT_READ | PROT_WRITE,
               MAP_PRIVATE | MAP_ANONYMOUS | MAP_HUGETLB | MAP_POPULATE,
               -1, 0);
    if (ptr != MAP_FAILED) {
        *used_hugepages = true;
        return ptr;
    }

    /* Fall back to 4K pages (mlock to prevent swapping) */
    ptr = mmap(NULL, size,
               PROT_READ | PROT_WRITE,
               MAP_PRIVATE | MAP_ANONYMOUS | MAP_POPULATE,
               -1, 0);
    if (ptr == MAP_FAILED)
        return NULL;

    if (mlock(ptr, size) < 0) {
        fprintf(stderr, "[vfio] mlock failed (%s) — DMA may be unreliable\n",
                strerror(errno));
        /* Non-fatal: continue anyway */
    }

    *used_hugepages = false;
    return ptr;
}

/* Register a virtual address range with the VFIO IOMMU container.
 * The kernel IOMMU driver pins the pages and creates IOVA→PA mappings.
 * Returns the IOVA on success, UINT64_MAX on failure. */
static int vfio_iommu_map(int container_fd, void *vaddr, size_t size, uint64_t iova)
{
    struct vfio_iommu_type1_dma_map dma_map = {
        .argsz = sizeof(dma_map),
        .flags = VFIO_DMA_MAP_FLAG_READ | VFIO_DMA_MAP_FLAG_WRITE,
        .vaddr = (uintptr_t)vaddr,
        .iova  = iova,
        .size  = size,
    };

    if (ioctl(container_fd, VFIO_IOMMU_MAP_DMA, &dma_map) < 0) {
        fprintf(stderr, "[vfio] VFIO_IOMMU_MAP_DMA failed: %s\n",
                strerror(errno));
        return -1;
    }
    return 0;
}

static void vfio_iommu_unmap(int container_fd, uint64_t iova, size_t size)
{
    struct vfio_iommu_type1_dma_unmap dma_unmap = {
        .argsz = sizeof(dma_unmap),
        .flags = 0,
        .iova  = iova,
        .size  = size,
    };
    ioctl(container_fd, VFIO_IOMMU_UNMAP_DMA, &dma_unmap);
}

/* -------------------------------------------------------------------------
 * Public: m2sdr_vfio_open
 * ---------------------------------------------------------------------- */

struct m2sdr_vfio *m2sdr_vfio_open(const char *pci_addr, int poll_mode)
{
    struct m2sdr_vfio *v;
    struct vfio_group_status group_status = { .argsz = sizeof(group_status) };
    struct vfio_device_info  device_info  = { .argsz = sizeof(device_info)  };
    struct vfio_region_info  bar0_info    = { .argsz = sizeof(bar0_info),
                                              .index = VFIO_PCI_BAR0_REGION_INDEX };
    char   path[256];
    int    group_num;
    int    ret;

    v = calloc(1, sizeof(*v));
    if (!v)
        return NULL;

    v->container_fd = -1;
    v->group_fd     = -1;
    v->device_fd    = -1;
    v->irq_fd       = -1;
    v->poll_mode    = poll_mode;
    v->rx_active    = false;
    v->tx_active    = false;

    /* ------------------------------------------------------------------
     * Step 1: Open VFIO container
     * ---------------------------------------------------------------- */
    v->container_fd = open("/dev/vfio/vfio", O_RDWR);
    if (v->container_fd < 0) {
        fprintf(stderr, "[vfio] open /dev/vfio/vfio: %s\n", strerror(errno));
        goto err;
    }

    if (ioctl(v->container_fd, VFIO_GET_API_VERSION) != VFIO_API_VERSION) {
        fprintf(stderr, "[vfio] VFIO API version mismatch\n");
        goto err;
    }

    if (!ioctl(v->container_fd, VFIO_CHECK_EXTENSION, VFIO_TYPE1_IOMMU)) {
        fprintf(stderr, "[vfio] VFIO_TYPE1_IOMMU not supported\n");
        goto err;
    }

    /* ------------------------------------------------------------------
     * Step 2: Open IOMMU group
     * ---------------------------------------------------------------- */
    group_num = vfio_get_group(pci_addr);
    if (group_num < 0)
        goto err;

    snprintf(path, sizeof(path), "/dev/vfio/%d", group_num);
    v->group_fd = open(path, O_RDWR);
    if (v->group_fd < 0) {
        fprintf(stderr, "[vfio] open %s: %s\n", path, strerror(errno));
        fprintf(stderr, "[vfio] Is vfio-pci bound? Try:\n");
        fprintf(stderr, "  sudo modprobe vfio-pci\n");
        fprintf(stderr, "  sudo sh -c 'echo 10ee 7021 > /sys/bus/pci/drivers/vfio-pci/new_id'\n");
        fprintf(stderr, "  sudo sh -c 'echo %s > /sys/bus/pci/devices/%s/driver/unbind'\n",
                pci_addr, pci_addr);
        fprintf(stderr, "  sudo sh -c 'echo %s > /sys/bus/pci/drivers/vfio-pci/bind'\n",
                pci_addr);
        goto err;
    }

    ioctl(v->group_fd, VFIO_GROUP_GET_STATUS, &group_status);
    if (!(group_status.flags & VFIO_GROUP_FLAGS_VIABLE)) {
        fprintf(stderr, "[vfio] group %d not viable (not all devices bound to vfio)\n",
                group_num);
        goto err;
    }

    /* ------------------------------------------------------------------
     * Step 3: Attach group to container and set IOMMU type
     * ---------------------------------------------------------------- */
    if (ioctl(v->group_fd, VFIO_GROUP_SET_CONTAINER, &v->container_fd) < 0) {
        fprintf(stderr, "[vfio] VFIO_GROUP_SET_CONTAINER: %s\n", strerror(errno));
        goto err;
    }

    /* Try TYPE1v2 first (supports dirty page tracking), fall back to TYPE1 */
    ret = ioctl(v->container_fd, VFIO_SET_IOMMU, VFIO_TYPE1v2_IOMMU);
    if (ret < 0)
        ret = ioctl(v->container_fd, VFIO_SET_IOMMU, VFIO_TYPE1_IOMMU);
    if (ret < 0) {
        fprintf(stderr, "[vfio] VFIO_SET_IOMMU: %s\n", strerror(errno));
        goto err;
    }

    /* ------------------------------------------------------------------
     * Step 4: Get device fd
     * ---------------------------------------------------------------- */
    v->device_fd = ioctl(v->group_fd, VFIO_GROUP_GET_DEVICE_FD, pci_addr);
    if (v->device_fd < 0) {
        fprintf(stderr, "[vfio] VFIO_GROUP_GET_DEVICE_FD(%s): %s\n",
                pci_addr, strerror(errno));
        goto err;
    }

    ioctl(v->device_fd, VFIO_DEVICE_GET_INFO, &device_info);
    printf("[vfio] device %s: %u regions, %u irqs\n",
           pci_addr, device_info.num_regions, device_info.num_irqs);

    /* ------------------------------------------------------------------
     * Step 5: Map BAR0
     * ---------------------------------------------------------------- */
    if (ioctl(v->device_fd, VFIO_DEVICE_GET_REGION_INFO, &bar0_info) < 0) {
        fprintf(stderr, "[vfio] VFIO_DEVICE_GET_REGION_INFO BAR0: %s\n",
                strerror(errno));
        goto err;
    }

    if (!(bar0_info.flags & VFIO_REGION_INFO_FLAG_MMAP)) {
        fprintf(stderr, "[vfio] BAR0 not mmappable\n");
        goto err;
    }

    v->bar0 = mmap(NULL, bar0_info.size,
                   PROT_READ | PROT_WRITE, MAP_SHARED,
                   v->device_fd, (off_t)bar0_info.offset);
    if (v->bar0 == MAP_FAILED) {
        fprintf(stderr, "[vfio] mmap BAR0: %s\n", strerror(errno));
        v->bar0 = NULL;
        goto err;
    }
    v->bar0_size = bar0_info.size;
    printf("[vfio] BAR0 mapped: %zu bytes at %p\n", v->bar0_size, v->bar0);

    /* ------------------------------------------------------------------
     * Step 6: Allocate DMA buffers and register with IOMMU
     * ---------------------------------------------------------------- */
    bool rx_huge, tx_huge;

    v->rx_virt = alloc_dma_buf(M2SDR_DMA_BUFFER_TOTAL_SIZE, &rx_huge);
    if (!v->rx_virt) {
        fprintf(stderr, "[vfio] RX DMA alloc failed: %s\n", strerror(errno));
        goto err;
    }
    v->rx_size = M2SDR_DMA_BUFFER_TOTAL_SIZE;
    v->rx_iova = M2SDR_RX_IOVA_BASE;

    if (vfio_iommu_map(v->container_fd, v->rx_virt, v->rx_size, v->rx_iova) < 0)
        goto err;
    v->rx_iova_mapped = true;

    v->tx_virt = alloc_dma_buf(M2SDR_DMA_BUFFER_TOTAL_SIZE, &tx_huge);
    if (!v->tx_virt) {
        fprintf(stderr, "[vfio] TX DMA alloc failed: %s\n", strerror(errno));
        goto err;
    }
    v->tx_size = M2SDR_DMA_BUFFER_TOTAL_SIZE;
    v->tx_iova = M2SDR_TX_IOVA_BASE;

    if (vfio_iommu_map(v->container_fd, v->tx_virt, v->tx_size, v->tx_iova) < 0)
        goto err;
    v->tx_iova_mapped = true;

    printf("[vfio] RX DMA: virt=%p iova=0x%llx size=%zu (%s)\n",
           v->rx_virt, (unsigned long long)v->rx_iova, v->rx_size,
           rx_huge ? "hugepages" : "4K pages");
    printf("[vfio] TX DMA: virt=%p iova=0x%llx size=%zu (%s)\n",
           v->tx_virt, (unsigned long long)v->tx_iova, v->tx_size,
           tx_huge ? "hugepages" : "4K pages");

    /* ------------------------------------------------------------------
     * Step 7: Set up MSI eventfd (for ADAPTIVE / IRQ poll modes)
     * ---------------------------------------------------------------- */
    if (poll_mode != M2SDR_POLL_BUSYPOLL) {
        v->irq_fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
        if (v->irq_fd < 0) {
            fprintf(stderr, "[vfio] eventfd: %s\n", strerror(errno));
            goto err;
        }

        /* Allocate irq_set with one eventfd payload */
        struct {
            struct vfio_irq_set hdr;
            int                 fds[1];
        } irq_set;

        irq_set.hdr.argsz  = sizeof(irq_set);
        irq_set.hdr.flags  = VFIO_IRQ_SET_DATA_EVENTFD |
                             VFIO_IRQ_SET_ACTION_TRIGGER;
        irq_set.hdr.index  = VFIO_PCI_MSI_IRQ_INDEX;
        irq_set.hdr.start  = 0;   /* MSI vector 0 */
        irq_set.hdr.count  = 1;
        irq_set.fds[0]     = v->irq_fd;

        if (ioctl(v->device_fd, VFIO_DEVICE_SET_IRQS, &irq_set) < 0) {
            fprintf(stderr, "[vfio] VFIO_DEVICE_SET_IRQS: %s\n", strerror(errno));
            /* Non-fatal for ADAPTIVE mode — fall back to pure spin */
            close(v->irq_fd);
            v->irq_fd = -1;
            fprintf(stderr, "[vfio] MSI setup failed, falling back to BUSYPOLL\n");
            v->poll_mode = M2SDR_POLL_BUSYPOLL;
        } else {
            /* Enable DMA writer (RX) and reader (TX) MSI bits in gateware */
            uint32_t msi_en = (1u << M2SDR_DMA0_WRITER_IRQ_BIT) |
                              (1u << M2SDR_DMA0_READER_IRQ_BIT);
            m2sdr_vfio_writel(v, M2SDR_CSR_PCIE_MSI_ENABLE, msi_en);
            printf("[vfio] MSI eventfd set up (irq_fd=%d)\n", v->irq_fd);
        }
    }

    printf("[vfio] open complete, poll_mode=%d\n", v->poll_mode);
    return v;

err:
    m2sdr_vfio_close(v);
    return NULL;
}

/* -------------------------------------------------------------------------
 * Public: m2sdr_vfio_close
 * ---------------------------------------------------------------------- */

void m2sdr_vfio_close(struct m2sdr_vfio *v)
{
    if (!v)
        return;

    /* Stop DMA engines */
    if (v->bar0) {
        if (v->rx_active)
            m2sdr_vfio_rx_stop(v);
        if (v->tx_active)
            m2sdr_vfio_tx_stop(v);
    }

    /* Disable MSI */
    if (v->irq_fd >= 0) {
        if (v->bar0)
            m2sdr_vfio_writel(v, M2SDR_CSR_PCIE_MSI_ENABLE, 0);
        close(v->irq_fd);
        v->irq_fd = -1;
    }

    /* Unmap BAR0 */
    if (v->bar0) {
        munmap(v->bar0, v->bar0_size);
        v->bar0 = NULL;
    }

    /* Unmap and free DMA buffers */
    if (v->rx_iova_mapped)
        vfio_iommu_unmap(v->container_fd, v->rx_iova, v->rx_size);
    if (v->tx_iova_mapped)
        vfio_iommu_unmap(v->container_fd, v->tx_iova, v->tx_size);

    if (v->rx_virt) { munmap(v->rx_virt, v->rx_size); v->rx_virt = NULL; }
    if (v->tx_virt) { munmap(v->tx_virt, v->tx_size); v->tx_virt = NULL; }

    /* Close VFIO fds */
    if (v->device_fd  >= 0) { close(v->device_fd);  v->device_fd  = -1; }
    if (v->group_fd   >= 0) { close(v->group_fd);   v->group_fd   = -1; }
    if (v->container_fd >= 0) { close(v->container_fd); v->container_fd = -1; }

    free(v);
}
