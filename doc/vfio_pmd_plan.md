# M2SDR VFIO User-Mode Driver Plan

## 1. Executive Summary

The goal is to eliminate kernel-crossing overhead from the M2SDR TX/RX data path by
implementing a VFIO-based user-mode driver.  The current latency budget is dominated
by syscalls: `ioctl()` to read DMA progress, `ioctl()` to advance the software ring
pointer, and `poll()` to wait for buffer availability.  With VFIO all three disappear
from the hot path — replaced by direct MMIO loads/stores and a busy-poll loop in
user space.

**No gateware changes are required.**  The LitePCIeDMA engine already exposes
everything needed via BAR0 registers, and the descriptor ring already runs in
autonomous loop mode.  The kernel driver is simply bypassed.


## 2. Current Architecture and Latency Sources

### 2.1 Data Path Today

```
5G NR stack / SoapySDR
        │
   acquireWriteBuffer()
        │
   poll(fd, POLLOUT)          ← syscall #1 — wait for HW to advance
        │
   ioctl(MMAP_DMA_READER)     ← syscall #2 — read hw_count from kernel
        │  (kernel reads LOOP_STATUS register, copies to user)
        │
   [write samples to buffer]
        │
   ioctl(MMAP_DMA_READER_UPDATE) ← syscall #3 — tell kernel new sw_count
        │  (kernel writes READER_ENABLE CSR)
        │
   releaseWriteBuffer()
```

The same pattern applies symmetrically for RX (DMA Writer path).

### 2.2 Latency Budget (approximate, 5950X host)

| Source                         | Typical cost |
|-------------------------------|-------------|
| `poll()` syscall overhead      | 1–3 µs      |
| `ioctl()` round-trip (×2–3)    | 1–2 µs each |
| Kernel interrupt handler       | 2–5 µs      |
| Context-switch (if sleeping)   | 5–20 µs     |
| **Total per buffer (TX or RX)**| **~10–30 µs** |

At 30.72 MHz sample rate, one 8192-sample DMA buffer spans ~267 µs, so these
overheads are a sizeable fraction of one buffer time and become dominant when
sub-buffer latency matters (e.g. HARQ retransmissions).

### 2.3 What the Kernel Driver Actually Does

The kernel driver (`software/kernel/main.c`) performs three functions:

1. **DMA memory allocation** — calls `dma_alloc_coherent()` for 256×8192 B TX and
   RX buffers, stores physical addresses.
2. **Ring programming** — writes physical addresses + sizes into the DMA descriptor
   table registers once at startup.
3. **Hot-path service** — on every buffer boundary (8 buffers/IRQ), reads the
   `LOOP_STATUS` register, updates `hw_count`, wakes poll waiters, and processes
   `sw_count` updates via ioctl.

Steps 1 and 2 happen once.  Step 3 is the latency culprit — it forces every
progress check through the kernel.


## 3. Gateware Analysis

### 3.1 What We Need from Hardware

| Capability | Register | Already present? |
|------------|----------|-----------------|
| DMA descriptor ring (loop mode) | `WRITER/READER_TABLE_*` | Yes |
| HW progress counter | `WRITER/READER_TABLE_LOOP_STATUS` | Yes |
| Enable/disable DMA | `WRITER/READER_ENABLE` | Yes |
| MSI interrupt delivery | PCIe MSI engine | Yes |
| MMIO CSR access | BAR0 | Yes |

### 3.2 Loop Status Register

```
LOOP_STATUS[31:16]  loop_count  — number of complete passes through the 256-entry ring
LOOP_STATUS[15:0]   index       — current descriptor index (0–255)
```

`hw_count = loop_count × 256 + index`

This is a 32-bit register read — one load instruction with VFIO MMIO mapping.

### 3.3 Conclusion

**Zero gateware changes are needed.**  Every required register is already mapped
into BAR0 and is readable/writable from user space once VFIO grants access.

The one optional gateware enhancement (discussed in §8) would be a "DMA
completion doorbell" FIFO so that a sleeping thread can be woken without a
timer, but this is an optimisation, not a prerequisite.


## 4. VFIO Architecture

### 4.1 How VFIO Works

VFIO (Virtual Function I/O) is a Linux kernel framework that:

- Exposes a PCIe device's BARs directly to user space via `mmap()`.
- Provides an IOMMU abstraction so user space can register arbitrary memory
  regions and obtain IOVA (I/O Virtual Addresses) to program into device DMA
  engines — without needing physically contiguous memory.
- Delivers MSI/MSI-X interrupts to user space via `eventfd`, eliminating the
  kernel interrupt handler from the hot path.

No privileged operations are required once the device is bound to `vfio-pci`.

### 4.2 Proposed Architecture

```
┌───────────────────────────────────────────────────────────┐
│                  Application / SoapySDR                    │
├───────────────────────────────────────────────────────────┤
│            libm2sdr_vfio  (new — this plan)               │
│                                                           │
│  ┌──────────────┐  ┌────────────────┐  ┌──────────────┐  │
│  │  vfio_setup  │  │  dma_ring_mgr  │  │  poll_engine │  │
│  │  bar0_map()  │  │  tx_submit()   │  │  busy_poll() │  │
│  │  dma_alloc() │  │  rx_consume()  │  │  irq_wait()  │  │
│  └──────┬───────┘  └───────┬────────┘  └──────┬───────┘  │
│         │                  │                   │          │
└─────────┼──────────────────┼───────────────────┼──────────┘
          │ mmap(BAR0)        │ direct MMIO       │ eventfd
          ▼                  ▼                   ▼
     /dev/vfio/<N>      BAR0 registers      MSI interrupt
          │
          ▼
     VFIO IOMMU
     (maps hugepages → IOVA for DMA engine)
          │
          ▼
     M2SDR FPGA  ←─── PCIe Gen2 x4 ───→  DMA buffers
```

### 4.3 Memory Model

Instead of `dma_alloc_coherent()` (kernel), we:

1. Allocate 2 MB hugepages (`MAP_HUGETLB | MAP_ANONYMOUS`) for TX and RX.
2. Call `VFIO_IOMMU_MAP_DMA` with the virtual address + size — the kernel IOMMU
   driver pins the pages and returns an IOVA.
3. Program the IOVA into the DMA descriptor table via BAR0 MMIO.

Hugepages give us physically-contiguous 2 MiB regions, which keeps IOMMU
mapping simple (one entry per ring) and avoids TLB pressure on the CPU side.

### 4.4 Hot-Path Comparison

| Operation | Current (kernel) | VFIO |
|-----------|-----------------|------|
| Read DMA hw_count | `ioctl()` → kernel reads LOOP_STATUS | `*bar0_ptr` load (1 MMIO read) |
| Advance sw_count  | `ioctl()` (RX) or `ioctl()` (TX) | `*bar0_ptr = val` store (1 MMIO write) |
| Wait for buffer   | `poll()` → sleep/wake cycle | busy-poll register, or `read(eventfd)` |
| Write sample data | already zero-copy | zero-copy (same hugepage) |

In the steady state (constant data flow, no underruns) **zero syscalls** are
needed per buffer.


## 5. Implementation Plan

### Phase 0 — Prerequisites (no code changes)

1. **Bind device to vfio-pci**:
   ```
   echo "10ee 7024" > /sys/bus/pci/drivers/vfio-pci/new_id
   echo "0000:XX:00.0" > /sys/bus/pci/devices/0000:XX:00.0/driver/unbind
   echo "0000:XX:00.0" > /sys/bus/pci/drivers/vfio-pci/bind
   ```
2. **IOMMU enabled in kernel**: `intel_iommu=on` or `amd_iommu=on` in
   kernel cmdline.  Check: `dmesg | grep -i iommu`.
3. **Hugepages available**: `echo 4 > /proc/sys/vm/nr_hugepages` (2 MiB ×4 =
   8 MiB, sufficient for TX+RX rings plus headroom).
4. **User permissions**: add user to `vfio` group, or run with CAP_SYS_ADMIN
   during bring-up.

No changes to the existing kernel driver or gateware are required for this
phase.  The kernel driver simply will not be loaded for the VFIO-bound device.

---

### Phase 1 — VFIO Setup Library (`software/user/libm2sdr_vfio/vfio_setup.c`)

Implement device open, BAR mapping, and DMA IOMMU setup.

**Key structs:**

```c
struct m2sdr_vfio {
    int  container_fd;    /* /dev/vfio/vfio           */
    int  group_fd;        /* /dev/vfio/<group>        */
    int  device_fd;       /* VFIO device fd           */
    void *bar0;           /* mmap'd BAR0              */
    size_t bar0_size;     /* from REGION_INFO         */

    /* TX (READER) DMA ring */
    void    *tx_virt;     /* hugepage virtual address */
    uint64_t tx_iova;     /* IOMMU address for device */
    size_t   tx_size;     /* DMA_BUFFER_TOTAL_SIZE    */

    /* RX (WRITER) DMA ring */
    void    *rx_virt;
    uint64_t rx_iova;
    size_t   rx_size;

    /* MSI eventfd (optional) */
    int  irq_fd;
};
```

**Functions:**

```c
int  m2sdr_vfio_open(struct m2sdr_vfio *v, const char *pci_addr);
void m2sdr_vfio_close(struct m2sdr_vfio *v);
int  m2sdr_vfio_map_bar0(struct m2sdr_vfio *v);
int  m2sdr_vfio_alloc_dma(struct m2sdr_vfio *v);
int  m2sdr_vfio_setup_msi(struct m2sdr_vfio *v);  /* optional */
```

**Implementation notes:**

- `vfio_open`: open `/dev/vfio/vfio` (container), find group from sysfs,
  open `/dev/vfio/<group>`, call `VFIO_GROUP_GET_DEVICE_FD`.
- `map_bar0`: call `VFIO_DEVICE_GET_REGION_INFO` for region index 0
  (BAR0), then `mmap()` that region.
- `alloc_dma`: `mmap(MAP_HUGETLB|MAP_ANONYMOUS)` × 2, then
  `ioctl(container_fd, VFIO_IOMMU_MAP_DMA, ...)` for each, choosing
  IOVA base addresses (e.g. `0x0000_0000` for RX, `0x0020_0000` for TX).
- `setup_msi`: `eventfd(0, EFD_NONBLOCK)`, then
  `VFIO_DEVICE_SET_IRQS` with `VFIO_IRQ_SET_ACTION_TRIGGER|VFIO_IRQ_SET_DATA_EVENTFD`.

---

### Phase 2 — DMA Ring Initialisation (`vfio_dma.c`)

Program the 256-entry descriptor rings directly via BAR0 MMIO, replicating
what the kernel driver does in `litepcie_dma_reader_start()` /
`litepcie_dma_writer_start()`.

The DMA base address in BAR0 is `CSR_PCIE_DMA0_BASE` (0x6000).

**Descriptor programming sequence (matching kernel driver):**

```c
/* For each of 256 buffers, write one descriptor */
for (int i = 0; i < 256; i++) {
    uint64_t addr = ring_iova + (uint64_t)i * DMA_BUFFER_SIZE;
    uint32_t len  = DMA_BUFFER_SIZE;
    uint32_t flags = (i % DMA_BUFFER_PER_IRQ == DMA_BUFFER_PER_IRQ-1)
                     ? 0 : DMA_IRQ_DISABLE;
    flags |= DMA_LAST_DISABLE;   /* let HW manage bursting */

    /* Write low address + length + flags */
    mmio_write32(bar0, DMA_BASE + TABLE_VALUE_OFFSET,
                 (uint32_t)addr | (len << 0) | flags);
    /* Write high address (WE = commit) */
    mmio_write32(bar0, DMA_BASE + TABLE_WE_OFFSET,
                 (uint32_t)(addr >> 32));
}

/* Set loop mode */
mmio_write32(bar0, DMA_BASE + TABLE_LOOP_PROG_N_OFFSET, 0);

/* Enable DMA engine */
mmio_write32(bar0, DMA_BASE + ENABLE_OFFSET, 1);
```

Same for READER (TX) ring with `tx_iova`.

**Helper macros:**

```c
static inline uint32_t mmio_read32(void *bar, uint32_t off) {
    return __atomic_load_n((uint32_t *)((char *)bar + off),
                           __ATOMIC_ACQUIRE);
}
static inline void mmio_write32(void *bar, uint32_t off, uint32_t val) {
    __atomic_store_n((uint32_t *)((char *)bar + off), val,
                     __ATOMIC_RELEASE);
}
```

`__ATOMIC_ACQUIRE`/`RELEASE` provides the correct memory ordering for MMIO
without a full `mfence` on x86.

---

### Phase 3 — Polling Engine (`vfio_poll.c`)

Replace `poll()` + `ioctl()` with direct register reads.

```c
struct m2sdr_dma_state {
    int64_t hw_count;   /* last read from LOOP_STATUS */
    int64_t sw_count;   /* our consumption pointer    */
    void   *bar0;
    uint32_t status_reg_off;   /* WRITER or READER STATUS */
};

/* Non-blocking: returns number of new buffers available */
static inline int m2sdr_dma_poll(struct m2sdr_dma_state *s)
{
    uint32_t status = mmio_read32(s->bar0, s->status_reg_off);
    uint32_t loop   = status >> 16;
    uint32_t idx    = status & 0xffff;
    int64_t  hw     = (int64_t)loop * 256 + idx;
    int      avail  = (int)(hw - s->hw_count);
    s->hw_count = hw;
    return avail;
}
```

**Polling strategy — three modes, selectable at runtime:**

| Mode | Mechanism | Latency | CPU cost |
|------|-----------|---------|----------|
| `BUSYPOLL` | Tight spin on LOOP_STATUS | Sub-microsecond | 1 full core |
| `ADAPTIVE` | Spin N cycles, then sleep on MSI eventfd | ~1–5 µs | Low when idle |
| `IRQ` | Block on MSI eventfd (like current driver) | ~5–15 µs | Minimal |

`ADAPTIVE` is the recommended default for 5G NR: spin during active bursts,
sleep during inter-subframe gaps.

---

### Phase 4 — CSR Access Helpers (`vfio_csr.c`)

Replace `litepcie_readl()` / `litepcie_writel()` (which call `ioctl()`) with
direct MMIO equivalents:

```c
static inline uint32_t m2sdr_readl(struct m2sdr_vfio *v, uint32_t addr) {
    return mmio_read32(v->bar0, addr);
}
static inline void m2sdr_writel(struct m2sdr_vfio *v, uint32_t addr, uint32_t val) {
    mmio_write32(v->bar0, addr, val);
}
```

This covers all AD9361 SPI, TDD switch, TimedTX, SampleCounter, and
capability register accesses.  The existing `csr.h` address constants are
reused unchanged.

---

### Phase 5 — SoapySDR Integration

Two integration approaches, from least to most invasive:

#### Option A — Drop-in replacement (recommended for initial integration)

Add a compile-time or runtime flag `LITEPCIE_USE_VFIO` to
`LiteXM2SDRStreaming.cpp`.  When set:

- Replace `litepcie_dma_init()` with `m2sdr_vfio_open()` + ring init.
- Replace `litepcie_dma_poll()` with `m2sdr_dma_poll()`.
- Replace `litepcie_readl()` / `litepcie_writel()` with `m2sdr_readl()` /
  `m2sdr_writel()`.
- Buffer pointers (`_rx_buf`, `_tx_buf`) point directly into hugepage VAs.

No changes to the SoapySDR API surface.  The `acquireReadBuffer()` /
`releaseReadBuffer()` / `acquireWriteBuffer()` / `releaseWriteBuffer()` call
signatures are identical.

#### Option B — New SoapySDR module

Create `LiteXM2SDRDevice_VFIO` as a separate SoapySDR driver
(`driver=LiteXM2SDR_VFIO`).  More future-proof but requires duplicating device
enumeration and settings logic.

---

### Phase 6 — Regression and Latency Testing

1. **Correctness**: run existing `m2sdr_util` loopback test with VFIO driver.
   Compare sample integrity against kernel driver output.

2. **Latency measurement**:
   - Instrument `acquireWriteBuffer()` → `releaseWriteBuffer()` with
     `clock_gettime(CLOCK_MONOTONIC_RAW)`.
   - Compare distributions (kernel driver vs VFIO) at 30.72 MHz, 61.44 MHz.
   - Measure tail latency (p99, p999) — this matters most for HARQ.

3. **CPU utilisation**: `perf stat` / `htop` for busy-poll mode vs adaptive
   mode.

4. **Timed TX accuracy**: send a timed burst; compare scheduled vs actual
   TX time using RX loopback + SampleCounter CSR.


## 6. File Layout

```
software/user/
├── libm2sdr_vfio/
│   ├── Makefile
│   ├── m2sdr_vfio.h          # public header
│   ├── vfio_setup.c          # Phase 1: device open, BAR map, DMA alloc
│   ├── vfio_dma.c            # Phase 2: descriptor ring init
│   ├── vfio_poll.c           # Phase 3: poll engine
│   └── vfio_csr.c            # Phase 4: CSR read/write helpers
│
└── liblitepcie/              # existing — unchanged in kernel-driver mode
    └── ...

software/soapysdr/
├── LiteXM2SDRStreaming.cpp   # Phase 5: add VFIO path behind flag
└── ...
```

A single new file `software/user/libm2sdr_vfio/m2sdr_vfio.h` provides the
complete public API.  The library links only against libc (no kernel headers,
no LitePCIe kernel driver headers).


## 7. Public API Sketch

```c
/* m2sdr_vfio.h */

#define M2SDR_POLL_BUSYPOLL  0
#define M2SDR_POLL_ADAPTIVE  1
#define M2SDR_POLL_IRQ       2

struct m2sdr_vfio;  /* opaque */

/* Setup */
struct m2sdr_vfio *m2sdr_vfio_open(const char *pci_addr, int poll_mode);
void               m2sdr_vfio_close(struct m2sdr_vfio *v);

/* CSR access (replaces litepcie_readl/writel) */
uint32_t m2sdr_vfio_readl(struct m2sdr_vfio *v, uint32_t csr_addr);
void     m2sdr_vfio_writel(struct m2sdr_vfio *v, uint32_t csr_addr, uint32_t val);

/* DMA streaming — RX (WRITER engine) */
int    m2sdr_vfio_rx_start(struct m2sdr_vfio *v);
void   m2sdr_vfio_rx_stop(struct m2sdr_vfio *v);
void  *m2sdr_vfio_rx_acquire(struct m2sdr_vfio *v, int *handle);  /* zero-copy ptr */
void   m2sdr_vfio_rx_release(struct m2sdr_vfio *v, int handle);

/* DMA streaming — TX (READER engine) */
int    m2sdr_vfio_tx_start(struct m2sdr_vfio *v);
void   m2sdr_vfio_tx_stop(struct m2sdr_vfio *v);
void  *m2sdr_vfio_tx_acquire(struct m2sdr_vfio *v, int *handle);
void   m2sdr_vfio_tx_release(struct m2sdr_vfio *v, int handle,
                              int n_bytes, uint64_t time_ns, int flags);

/* Direct LOOP_STATUS read (for advanced callers) */
int64_t m2sdr_vfio_rx_hw_count(struct m2sdr_vfio *v);
int64_t m2sdr_vfio_tx_hw_count(struct m2sdr_vfio *v);
```


## 8. Optional Gateware Enhancement (Not Required)

If busy-polling proves too expensive during idle gaps, a small gateware change
would allow precise interrupt-driven wakeup without the current per-8-buffer
MSI:

**"Completion doorbell" CSR**: a single CSR write from user space programs a
threshold `N`.  When `LOOP_STATUS.index` advances past `N`, the DMA engine
fires an MSI.  This lets user space arm the interrupt only when needed (before
sleeping), avoiding spurious wakeups.

This is analogous to DPDK's `rx_free_thresh` mechanism.  It is not critical for
initial bring-up — the adaptive polling mode in Phase 3 achieves similar
behaviour with pure software logic.


## 9. Risks and Mitigations

| Risk | Likelihood | Mitigation |
|------|------------|------------|
| IOMMU not enabled or not present | Medium | Document cmdline; detect at startup and error clearly |
| Hugepage allocation fails | Low | Fall back to 4K pages with multiple IOMMU mappings |
| VFIO group permissions | Medium | Document `vfio` group membership; provide udev rule |
| Cache coherency issues with MMIO | Low | Use `__ATOMIC_ACQUIRE/RELEASE`; add `lfence` if needed on ARM |
| DMA ring desync after crash | Medium | Add ring reset sequence in `m2sdr_vfio_open()` |
| Coexistence with kernel driver | N/A | Device must be bound to `vfio-pci`, not `litepcie`; document |
| Timed TX timestamp accuracy | Low | Unchanged — TimedTXArbiter in gateware handles this |


## 10. Suggested Implementation Order

1. Phase 0 — bind device, verify IOMMU, check hugepages work (30 min)
2. Phase 1 — VFIO setup + BAR0 mmap; verify CSR reads match kernel driver (1–2 days)
3. Phase 4 — CSR helpers; port `m2sdr_rf` to use them (1 day)
4. Phase 2 — DMA ring init; verify gateware runs ring (1–2 days)
5. Phase 3 — polling engine; verify RX samples are correct (1–2 days)
6. Phase 5A — SoapySDR integration behind flag; run loopback (1–2 days)
7. Phase 6 — latency measurement and tuning (1–2 days)

Total estimated effort: ~1 week of focused development.

The existing kernel driver remains fully functional and unmodified throughout.


---

## 11. Work Log

### 2026-03-10 — Session 1

#### Phase 0: Prerequisites verified

- **Device**: `0000:01:00.0` Xilinx 10ee:7021 (PCIe Gen2 x1 link), IOMMU group 15.
- **IOMMU**: Active — group type `DMA-FQ` (fault-queue variant of TYPE1). IOMMU groups
  present without explicit kernel cmdline flag (platform auto-enables it). ✅
- **vfio-pci module**: Available at
  `/lib/modules/6.17.0-14-generic/kernel/drivers/vfio/pci/vfio-pci.ko.zst`, not yet
  loaded. ✅
- **`/dev/vfio/vfio`**: Present, world-readable/writable. ✅
- **Hugepages**: 0 pre-allocated (2 MiB page size available). The library falls back to
  `mlock()`'d 4K pages automatically.
- **Current driver**: `m2sdr` (custom name for the LitePCIe kernel driver).
- **User permissions**: `dmd` in `sudo` group.

**To bind to vfio-pci before running the test** (one-time, needs sudo):
```bash
sudo modprobe vfio-pci
sudo sh -c 'echo 10ee 7021 > /sys/bus/pci/drivers/vfio-pci/new_id'
sudo sh -c 'echo 0000:01:00.0 > /sys/bus/pci/devices/0000:01:00.0/driver/unbind'
sudo sh -c 'echo 0000:01:00.0 > /sys/bus/pci/drivers/vfio-pci/bind'
# To return to kernel driver:
sudo sh -c 'echo 0000:01:00.0 > /sys/bus/pci/drivers/vfio-pci/unbind'
sudo sh -c 'echo 0000:01:00.0 > /sys/bus/pci/drivers/m2sdr/bind'
```

#### Phases 1–4: Library implemented and built cleanly

All source files written and compiled with `-Wall -Wextra`, zero warnings.

**Files created** (`software/user/libm2sdr_vfio/`):

| File | Purpose |
|------|---------|
| `m2sdr_vfio.h` | Public API header |
| `vfio_priv.h` | Internal struct, MMIO helpers, loop_status decoder |
| `vfio_setup.c` | Phase 1: VFIO open, BAR0 mmap, DMA hugepage alloc + IOMMU map, MSI eventfd |
| `vfio_dma.c` | Phase 2: descriptor ring programming (RX writer + TX reader) |
| `vfio_poll.c` | Phase 3: BUSYPOLL / ADAPTIVE / IRQ poll modes, acquire/release API |
| `vfio_csr.c` | Phase 4: direct MMIO CSR read/write (replaces litepcie ioctl) |
| `m2sdr_vfio_test.c` | Self-test: CSR scratch, RX/TX hw_count, latency measurement |
| `Makefile` | Builds `libm2sdr_vfio.a` and `m2sdr_vfio_test` |

**Key implementation notes**:
- Descriptor format confirmed from kernel driver source: 3 MMIO writes per entry
  (flags|size → addr_lo → addr_hi/WE-commit), matching `litepcie_dma_writer_start()`.
- IRQ pattern: MSI fires every 8 buffers at `i % 8 == 0` (entries 0, 8, 16 … 248).
- Synchronizer control preserved: RX-only start sets bit 1, TX start sets bit 0 (same
  as kernel driver).
- Hugepage fallback: if `MAP_HUGETLB` fails, falls back to `mlock()`'d 4K pages.
  VFIO IOMMU handles scatter-gather transparently.
- MSI setup: if `VFIO_DEVICE_SET_IRQS` fails (e.g. device still in passthrough),
  library silently degrades to BUSYPOLL.
- `loop_status_to_hw_count()`: handles 16-bit `loop_count` wrap-around identically
  to the kernel driver interrupt handler.

#### Phase 5: SoapySDR integration

**Completed.** All `USE_VFIO` branches added to `LiteXM2SDRDevice.cpp`,
`LiteXM2SDRDevice.hpp`, and `LiteXM2SDRStreaming.cpp`. Scope of changes:

- **Constructor**: opens device via `"vfio:" + pci_addr`, accepts `vfio_poll`
  kwarg (0=busypoll, 1=adaptive, 2=irq); populates `_dma_mmap_info` from VFIO
  constants; stores `_vfio = _dev->vfio`.
- **Destructor**: stops RX/TX VFIO DMA on close.
- **setupStream**: starts RX/TX ring; sets `_rx_stream.buf` / `_tx_stream.buf`
  to the VFIO virtual memory regions.
- **closeStream**: stops ring via `m2sdr_vfio_rx_stop` / `m2sdr_vfio_tx_stop`.
- **activateStream**: resets counters; writes crossbar CSR.
- **deactivateStream**: no-op (ring keeps running; stop on closeStream).
- **acquireReadBuffer**: timeout-aware poll via `m2sdr_vfio_rx_wait`; drain on
  overflow via `m2sdr_vfio_rx_sw_set`.
- **releaseReadBuffer**: advances `_rx_stream.sw_count`; calls
  `m2sdr_vfio_rx_sw_set` (no ioctl).
- **acquireWriteBuffer**: timeout-aware poll via `m2sdr_vfio_tx_wait`; computes
  buffer slot from `_tx_stream.user_count`; handles underflow detection and TX
  DMA header test.
- **releaseWriteBuffer**: advances `_tx_stream.sw_count` locally (loop-mode
  ring is pre-programmed; no ioctl or descriptor submission needed).
- **getNumDirectAccessBuffers / getDirectAccessBufferAddrs**: VFIO uses same
  direct-access path as LITEPCIE (combined `#if USE_LITEPCIE || USE_VFIO`).
- **CMakeLists.txt**: added `USE_VFIO` cmake option; discovers `libm2sdr_vfio`;
  sets `-DUSE_VFIO=1 -DUSE_LITEPCIE=0 -DUSE_LITEETH=0` and links
  `libm2sdr_vfio.a`.

Build with: `cmake -DUSE_VFIO=ON ..`

#### Phase 6: Latency testing

**Pending** — requires device rebound to vfio-pci. The `m2sdr_vfio_test` binary
includes the latency benchmark (1000 samples, reports mean/p50/p99/p999/max).

#### Next steps

1. Rebind device to vfio-pci (commands above).
2. Run `./m2sdr_vfio_test 0000:01:00.0 1` to validate BAR0 MMIO and DMA ring.
3. Build SoapySDR module with `cmake -DUSE_VFIO=ON ..` and run loopback test.
4. Run latency benchmark and compare against kernel driver baseline.
