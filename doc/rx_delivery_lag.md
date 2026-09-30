# RX delivery lag: interrupts, the completed-buffer count, and what a poll-mode driver would change

Branch `hw-timed-tx`, 2026-09-30. Measured with OCUDU (gNB, TDD n78, 23.04 MSps, 1R1T, SC16) on an
i7-9750H laptop (6 cores, stock Ubuntu kernel 7.0, governor `performance`). A DMA buffer is 8 KiB =
2044 samples = 88.7 µs.

"Delivery lag" here is the age of the **newest** sample of a DMA buffer at the moment the receiving
thread gets the buffer: FPGA pipeline + DMA + wake-up. It is measured against the gateware sample
counter, so it needs `timed_tx=hardware` (`M2SDR_RX_LAG_STATS=<seconds>` prints percentiles from the
plugin; `M2SDR_RX_LAG_EVERY=<n>` thins the measurement, which costs one counter read per buffer).

## Why it matters

A real-time consumer generates its TX timeline from the RX timeline: OCUDU's lower PHY releases a DL
slot when the newest RX timestamp reaches `slot start − rx_to_tx_delay` (stock: 1 ms). Every
microsecond of RX lag is taken from the time the TX samples have to reach the FPGA before they are due.

## What was wrong

Two things delayed every buffer, and only one of them was the interrupt cadence.

1. **One interrupt per 8 buffers** (`DMA_BUFFER_PER_IRQ`). The kernel's completed-buffer count is
   refreshed by the DMA interrupt, and `poll()` sleeps until then: buffers arrive in batches of 8, the
   first of each batch 7 frames (621 µs) old.
2. **The completed-buffer count was one short.** The LitePCIe table's `loop_status` is the index of the
   descriptor that completed *last* (the table entry is popped when its last write request is issued),
   not the number of completed descriptors. The driver used the bare index, so the newest complete
   buffer was withheld until the next interrupt: one full frame (88.7 µs) at best, and with batching
   the 8th buffer of a batch waited for the following batch.
3. `poll()` only reported `POLLIN` with more than 2 buffers pending, which would have re-batched the
   delivery by 3 once the interrupt cadence was 1.

## What changed

| Where | Change |
|---|---|
| `kernel/main.c` | module parameters `rx_irq_period` (default **1**) and `tx_irq_period` (default 8): buffers per interrupt, read at stream start, changeable in `/sys/module/m2sdr/parameters/` |
| `kernel/main.c` | writer count = `loop_status` index **+ 1** in the writer interrupt (a writer interrupt is raised by a completion); a stale pending writer interrupt is cleared at writer start |
| `kernel/main.c` | `POLLIN` as soon as one buffer is complete |
| `libm2sdr` | `m2sdr_set_rx_busy_poll()`: the RX acquire spins on the writer's live table index instead of sleeping in `poll()` |
| Soapy plugin | device argument `rx_poll=irq` (default) or `rx_poll=busy`; lag statistics and a completeness check (`M2SDR_RX_LAG_STATS`) in both the worker and the direct RX path |

The completeness check compares the last 256 bytes of each buffer as seen at delivery with the DMA
memory one buffer later: 0 differences in every run (several million buffers), i.e. with the corrected
count no buffer is handed out before its last bytes have landed.

## Measurements

Delivery lag in µs, 10 s windows of 112 721 buffers each, typical window shown (ranges over windows
where they differ).

| Mode | min | p50 | p90 | p99 | p99.9 | max | IRQ/s | cost |
|---|---|---|---|---|---|---|---|---|
| before: 8 buffers/IRQ, count one short | 139 | 472 | 718 | 724 | 764 | 1360 | 2.8 k | |
| 8 buffers/IRQ, count fixed | 12 | 386 | 630 | 664 | 676–716 | 1100–1700 | 2.8 k | |
| 1 buffer/IRQ, count one short | 95 | 98 | 98 | 126 | 352 | 1103 | 12.7 k | |
| **1 buffer/IRQ, count fixed (new default)** | 6 | 8 | 10 | 38–46 | 204–320 | 660–1040 | 12.7 k | worker thread ≈ 9 % of a core |
| same, no worker thread (OCUDU's FIFO-97 RX thread reads the ring) | 6 | 8 | 10 | 38–58 | 232–272 | 580–1050 | 12.7 k | |
| same, deep idle states blocked (`/dev/cpu_dma_latency` = 10 µs) | 6 | 8 | 10 | 14 | 156–164 | 460–1240 | 12.7 k | idle power |
| busy-poll (`rx_poll=busy`, 256 buffers/IRQ) | 3 | 4 | 4–6 | 8–22 | 130–208 | 550–1090 | 1.5 k | one core at 100 % |
| busy-poll, deep idle states blocked | 3 | 4 | 4 | 8 | 130–176 | 545–650 | 1.5 k | one core at 100 % |

As seen by OCUDU's lower PHY (one more thread hop): 355 µs average (156–545) before, 20 µs (18–26) now.

Host facts that set the tail, independent of the driver:

- `hwlat` tracer (interrupts disabled, so only firmware/hardware can steal the CPU), 40 s with the gNB
  stopped: gaps in every one-second window, 21 of 40 windows above 100 µs, maximum 224 µs. This laptop
  loses up to ~0.2 ms to SMI-class events at least once per second: that is the p99.9 of every row.
- The Wi-Fi interface re-associates every ~30 min 15 s (kernel log); one of these stalled both the RX
  and the TX path for 11.6 ms (RX discontinuity of 267 764 samples).
- A spinning thread at normal priority is preempted like any other: the busy-poll runs also showed
  stalls of 6, 12 and 19 ms that the interrupt-driven runs of the same length did not.

## What a DPDK-style poll-mode driver would and would not change

A PMD replaces "interrupt → handler → wake a sleeping thread" by a thread that spins on the device's
ring state from user space, with the BAR and the DMA memory (hugepages) mapped into the process and
the device bound to `vfio-pci`/UIO instead of a kernel driver. `rx_poll=busy` is that wake-up model on
the existing driver (the index is still read through an ioctl, ~1–2 µs per iteration; a real PMD reads
the mapped register or, better, polls the next frame header in DMA memory with no MMIO at all).

What the measurements say:

- **Median: 8 µs → 4 µs.** The interrupt path costs about 4 µs (MSI, three CSR reads and a write in the
  handler, scheduler wake-up). Against an 88.7 µs frame and a 1 ms budget this is noise.
- **p99: 40 µs → 8–20 µs.** Most of that is idle-state exit latency; blocking deep idle states gives
  14 µs with interrupts.
- **p99.9 and max: no change.** They are set by firmware stalls and host events, not by the wake-up
  mechanism. A PMD only delivers a better tail together with what it is normally deployed with:
  isolated cores (`isolcpus`, `nohz_full`, `rcu_nocbs`), no SMI-heavy firmware, IRQs steered away. On
  a shared core a spinning thread is *worse* in the tail than a sleeping one that wakes with priority.
- **Cost:** one core of six at 100 % instead of ~9 % of a core plus 12.7 k interrupts/s.
- **Throughput is not the issue.** DPDK exists for millions of packets per second. This stream is
  11 272 frames/s and 92 MB/s (245 MB/s at 61.44 MSps 2R2T).

Where a PMD (or just polling) would start to pay:

- **Smaller frames.** The frame fill time (88.7 µs here) is the floor for the *oldest* sample of a
  frame in any model. With 1 KiB frames (11 µs) the interrupt rate would be 90 k/s and polling becomes
  the natural design. At 61.44 MSps 2R2T a frame is already 16.6 µs, i.e. 60 k interrupts/s at
  `rx_irq_period=1`: use a larger period or `rx_poll=busy` there.
- **A host prepared for it** (isolated cores, RT kernel): then the p99.9 would follow the median.
- **TX.** A PMD brings its own TX queue model, a descriptor ring with a tail pointer the host advances.
  That is a true FIFO, and it is what the TX path is missing today (next section). But that benefit
  comes from the descriptor model, which LitePCIe already offers (table "prog" mode) and the kernel
  driver could use; it does not need DPDK.

Costs specific to this board: the kernel driver also provides CSR access for every tool, the PTP
clock, LiteUART and SATA; a `vfio-pci` binding loses those or requires them to be re-implemented in
user space. OCUDU's DPDK support is for the Ethernet fronthaul (split 7.2), not for a baseband radio,
so a PMD would also need a new OCUDU radio driver instead of the SoapySDR one.

Conclusion: for RX delivery the interrupt-driven driver with one interrupt per buffer is within 4 µs of
a poll-mode driver at the median and identical in the tail on this host. The remaining latency budget
problem is on the TX side and is about ring semantics, not about interrupts.

## What still limits the RX-to-TX budget: the TX ring

With the RX lag gone, OCUDU hands each DL slot to the driver `rx_to_tx_delay − ~60 µs` ahead of its time
(stock 1 ms: 942 µs average, 673–899 µs minimum per 5 s window). The TX path as it is built cannot live
on that, and the reason is not latency but what happens when the host is late once:

- The TX DMA runs in LitePCIe *loop* mode: the reader free-runs through a 256-slot ring and is only
  paced by the timed gate holding the frame at its head. The host must write each frame ahead of the
  reader (`tx_lead_buffers` = 4 slots = 355 µs) plus what the FPGA has already prefetched (an 8 KiB
  buffering FIFO and the reader's request FIFO, ~1.5–2 frames), about 0.55 ms in total.
- When a host stall lets the reader catch up with the write pointer, nothing holds it any more: every
  slot ahead holds a frame from the previous lap, which the gate drops as stale at PCIe speed. The
  reader then laps the ring every ~2 ms and the host, which writes 11 frames per ms, cannot get 4 slots
  ahead of a reader that moves 125 slots per ms. Frames written behind it are found one lap later,
  already late.

Trace of one episode at a 1.5 ms delay (`episode.c`, state sampled every ~170 µs; reader advance,
gate counters and "armed − now" per sample):

```
  t(ms)   reader   late  stale passed
  -3.2 … 0.9   host stall: samples 0.8 ms apart, reader +23, stale +18
   1.1    +2      0      0     2      normal again for 4 ms
   5.0    +5      0      4     2      reader reaches the write pointer
   5.2   +20      0     21     0      racing: ~125 slots/ms, everything stale ...
   6.2   +21     20      0     0      ... or late: frames written behind the reader, one lap old
   ...
  11.5   +15      0     16     0      one frame caught in time (armed 1.39 ms ahead): held
  15.0   +14      0     15     1      passed, and racing again
   ...                                 (continues for tens of ms)
```

Over a 45 s run at 1.5 ms the reader index advanced at up to 137 840 frames/s (nominal 11 272) in 253
bursts. An episode lasts 70–80 ms and costs about 500 late and 4000 stale frames; the frames that were
actually late because of the stall are a handful.

| `OCUDU_LPHY_RX_TO_TX_DELAY_US` | TX lead at hand-off (min / avg) | what happens |
|---|---|---|
| 5000 (before this work) | 4188 / 4507 µs with the old RX lag; 4909 / 4949 µs now | clean |
| 4000 (new default) | 3650–3900 / 3950 µs | one episode in a first 5-minute soak with phone traffic (+452 late, +4500 stale), none in the following 11-minute soak (lowest lead 2349 µs); 6 of 6 starts attach and ping 150/150 |
| 2000 | 1812 / 1943 µs | clean for minutes, then an episode when the phone attached (lead dipped to 1155 µs): +1000 late, the guardian restarted the gNB |
| 1500 | 1368 / 1446 µs | 8–12 episodes per 45 s |
| 1000 (stock) | 673–899 / 942 µs | continuous episodes, not usable |

So the delay is no longer sized by the RX lag (that needed ~1 ms of it) but by the largest host stall the
TX ring must never see, because one underrun costs 80 ms instead of the stall itself.

### What would fix it

TX needs FIFO semantics: the reader must stop when there is nothing new, the way a USRP's flow-controlled
TX FIFO (or a poll-mode driver's descriptor ring with a tail pointer) does. Two ways, both without gateware:

1. **LitePCIe table *prog* mode** (kernel driver + libm2sdr): the driver queues one descriptor per
   submitted frame instead of pre-loading a looping table; the reader stops when the table is empty. A
   frame then needs only the PCIe fetch time of lead (tens of µs), an underrun costs exactly the late
   frames, and the ring-lead / resync / stale logic goes away. Cost: three CSR writes per frame in the
   submit ioctl.
2. **A silence timeline in the ring** (libm2sdr + plugin only): every slot ahead of the write pointer is
   pre-stamped with the time it will be due and a zero payload, so the reader always advances in real
   time, and a frame that arrives too late is simply not seen. Needs ~0.25 ms of lead (the FPGA
   prefetch) and a fixed frame grid.

With either, the stock 1 ms budget leaves 0.7–0.9 ms for host jitter, and the last OCUDU core change
(`OCUDU_LPHY_RX_TO_TX_DELAY_US`) can go. On this host a late slot would then still happen whenever a
stall exceeds that margin (the per-10 s maximum of the RX lag is 0.5–1.1 ms, occasionally 2–3 ms), but
it would cost that slot and nothing else, as with any radio.
