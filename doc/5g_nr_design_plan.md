# M2SDR 5G NR Capability: Comprehensive Gap Analysis and Implementation Plan

**Sources:** Direct analysis of local checkouts at `/home/dmd/LimeSDR_GW`, `/home/dmd/bladeRF`,
`/home/dmd/LimeSuiteNG`, and `/home/dmd/m2sdr`. All module names, signal names, file paths, and
code references below are verified from the actual source trees.

---

## 1. Reference Architecture Summary

### 1.1 LimeSDR_GW — Buffer-Based Packet Arbiter

The LimeSDR_GW timed TX is implemented in
`LimeDFB_LiteX/tx_path_top/src/pct2data_buf_rd.vhd` as the `PCT2DATA_BUF_RD` VHDL entity.
It holds up to `G_BUFF_COUNT=4` incoming TX packets in 4 KB AXI-Stream FIFOs and releases
each packet's samples downstream only when the current RX sample counter reaches the packet's
embedded timestamp:

```vhdl
-- pct2data_buf_rd.vhd lines 216-238
if (state = WAIT_HEADER and S_AXIS_TVALID(curbuf) = '1') then
    if (unsigned(S_AXIS_TDATA(127 downto 64)) < unsigned(SAMPLE_NR)) then
        compare_less  <= '1';   -- late packet → drop + set PCT_LOSS_FLG
    elsif (unsigned(S_AXIS_TDATA(127 downto 64)) = unsigned(SAMPLE_NR)) then
        compare_equal <= '1';   -- on time → release
    end if;
end if;
```

Key signals:
- `SAMPLE_NR` (64-bit): free-running RX sample counter, CDC'd from `lms_rx` domain
- `PCT_LOSS_FLG`: persistent flag set on every late/dropped packet, cleared by software
- Packet header bits `[127:64]`: TX timestamp (samples since epoch)
- Packet header bits `[63:0]`: sync/flag word

RX timestamping (`rx_path_top.py` line 269): `PCT_HDR_1 = bp_sample_nr_counter` in classic
mode (TS_SEL=0), or upper 32 bits = PPS seconds count + lower 32 bits = clock ticks since
last PPS in mixed mode (TS_SEL=1).

TDD control (`tdd_control.vhd`): simple combinational gate — `TDD_OUT = AUTO_IN` (when
`AUTO_ENABLE=1`). The `AUTO_IN` input is the current TX-active status from the TX datapath.
RF switch outputs (`RX_RF_SW_OUT`, `TX_RF_SW_OUT`) are driven from the same signal. No
sample-count-based lead/lag offset is implemented; switching tracks the instantaneous
TX/RX state.

### 1.2 bladeRF — Dedicated `time_tamer` IP

bladeRF uses a standalone Nuand IP block at
`hdl/fpga/ip/nuand/time_tamer/vhdl/time_tamer.vhd`. It maintains a 64-bit counter clocked
in the AD9361 `ts_clock` domain and implements a compare-and-trigger FSM:

```vhdl
-- time_tamer.vhd lines 286-325
when WAIT_FOR_COMPARE =>
    if (compare_time = timestamp) then
        ts_time_trigger <= '1';   -- pulse when target time reached
        fsm := WAIT_FOR_CLEAR;
    end if;

-- Late packet detection (lines 357-366)
if (hold_time < current_time) then
    fsm      := PAST_TIME;
    status_past <= '1';           -- software-readable flag
end if;
```

Software writes an 8-byte target timestamp to memory-mapped registers (addr 0–7), then
writes `0x00` to addr 8 to arm the timer. When `ts_time_trigger` fires, the TX pipeline
releases samples. `status_past` provides explicit late-packet feedback to the host.

TX/RX metadata uses `packet_control_t` records (`fifo_readwrite_p.vhd`):
- `pkt_flags[7:0]`: TX_START, TX_END, burst markers
- `pkt_sop` / `pkt_eop`: AXI-Stream packet boundaries
- Metadata FIFOs are async (separate `tx_clock` / `rx_clock` domains), 2048 words deep

AD9361 interface: 6 LVDS pairs + FRAME + CLK, DDR 2:1, identical to m2sdr's `AD9361PHY`.
NIOS II soft processor handles AD9361 SPI configuration.

### 1.3 LimeSuiteNG — Userspace TX Scheduler + Full DSP Stack

`src/streaming/TRXLooper.cpp` implements a dual-threaded streamer (separate RX/TX work threads).
TX scheduling logic (lines 1371–1482):

```cpp
// Underrun detection
txAdvance = pkt->counter - rxNow;  // negative = late
if (txAdvance < 0) {
    stats.underrun++;
    // drop packet, report via statusCallback
}
```

Packet format (`src/streaming/DataPacket.h`):
```c
struct FPGA_TxDataPacket {
    uint8_t header0;         // bit 4 = ignore_timestamp
    uint8_t payloadSizeLSB;
    uint8_t payloadSizeMSB;
    uint8_t reserved[5];
    int64_t counter;         // 64-bit TX timestamp
    uint8_t data[4080];      // sample payload
};
```

SoapySDR binding (`plugins/soapysdr/Streaming.cpp`):
- `SOAPY_SDR_HAS_TIME` → sets `waitForTimestamp` in `StreamMeta`
- `SOAPY_SDR_END_BURST` → sets `flushPartialPacket`
- Time unit conversion: `SoapySDR::timeNsToTicks(timeNs, sampleRate)`

CFR: `src/DSP/CFR/CrestFactorReduction.h` — threshold-based peak clipper with 32-tap FIR
post-filter, configurable oversample factor.

Calibration: `src/chips/LMS7002M/mcu_dc_iq_calibration.cpp` — MCU binary running on the
LMS7002M's embedded ARM M4. Handles DC offset and IQ imbalance correction automatically
on stream activation.

---

## 2. M2SDR Current State

### 2.1 What Is Present and Adequate

| Component | File | Status |
|---|---|---|
| 64-bit ns hardware clock (Q8.24, ~60 ps resolution) | `gateware/time.py` | ✅ Solid |
| PPS generator + PPS sync to time counter | `gateware/pps.py` | ✅ Solid |
| RX header with timestamp injection | `gateware/header.py` | ✅ Solid |
| TX header timestamp extraction (CSR-readable) | `gateware/header.py` | ✅ Extracted but unused |
| PCIe PTM → FPGA time sync | `litex_m2sdr.py` | ✅ Solid |
| White Rabbit sub-ns sync (baseboard) | `wr_helper.py` | ✅ Solid |
| AD9361 2T2R LVDS PHY, 12-bit DDR | `gateware/ad9361/phy.py` | ✅ Solid |
| 61.44 / 122.88 MSPS modes | `gateware/ad9361/core.py` | ✅ Solid |
| 38.4 MHz SI5351 reference (3GPP-aligned) | `libm2sdr/m2sdr_config.h` | ✅ Solid |
| Zero-copy PCIe DMA ring buffer | `software/kernel/main.c` | ✅ Solid |
| PTP 1588 clock in kernel driver | `software/kernel/main.c` | ✅ Present |
| SoapySDR driver with RX timestamp extraction | `soapysdr/LiteXM2SDRStreaming.cpp` | ✅ Partial |
| TX overflow/underflow detection (reactive) | `soapysdr/LiteXM2SDRStreaming.cpp` | ⚠️ Reactive only |
| VRT timestamps on Ethernet path | `gateware/vrt.py` | ✅ RX only |

### 2.2 The Critical Missing Piece: TX Timestamp Gating

The TX header extractor in `gateware/header.py` reads the 64-bit timestamp from DMA word 1
and makes it available via CSR — but the timestamp is never compared against `TimeGenerator.time`
to hold or release the TX stream. The `_TX_DMA_HEADER_TEST` guard in
`soapysdr/LiteXM2SDRStreaming.cpp` (lines 897–901) inserts a monotonically-incrementing fake
counter as the TX timestamp, not the actual requested transmission time.

The SoapySDR `activateStream()` busy-polls `getHardwareTime()` in a 1 ms sleep loop (lines
932–942) and submits the DMA buffer only after `now >= timeNs` — this is a **host-side**
software delay, not sample-accurate hardware gating. Jitter from OS scheduling and PCIe
interrupt latency is typically 50–500 µs, incompatible with 5G NR's 500 µs slot duration.

---

## 3. Full Gap Analysis

### 3.1 Gateware Gaps

| # | Feature | LimeSDR_GW | bladeRF | m2sdr | Priority |
|---|---|---|---|---|---|
| G1 | **Timed TX arbiter** (hold until timestamp fires) | `pct2data_buf_rd.vhd` | `time_tamer.vhd` | ❌ | **Critical** |
| G2 | **TX burst flags** (START/END in DMA header) | bits [63:0] flag word | `pkt_flags[7:0]` | ❌ | **Critical** |
| G3 | **Late packet detection + flag** | `PCT_LOSS_FLG` | `status_past` | ❌ | **Critical** |
| G4 | **TX underrun fill** (zeros when FIFO empty) | Software | Software | ❌ | **Critical** |
| G5 | **TDD TX/RX switch** driven by sample counter | `tdd_control.vhd` | Packet flags | ❌ | **Critical** |
| G6 | PA_EN with programmable lead/lag offset | Partial | GPIO | ❌ | High |
| G7 | Sample-aligned DMA frame size (NR slot) | Yes | Yes | ❌ | High |
| G8 | Separate 64-bit sample counter (integer samples) | `SAMPLE_NR` | `ts_time` | ❌ (ns only) | High |
| G9 | RX timestamp mode: PPS seconds + sub-second ticks | TS_SEL=1 | ✅ | ❌ | Medium |
| G10 | CFR (clip + FIR filter) in TX pipeline | Roadmap | ❌ | ❌ | Medium |
| G11 | IQ imbalance correction in RX pipeline | LMS7002M HW | `iq_correction.vhd` | ❌ | Medium |
| G12 | DC offset filter in RX pipeline | LMS7002M HW | `ad_dcfilter.v` | ❌ | Medium |
| G13 | Capability register: advertise timed TX | N/A | N/A | ❌ | Low |

### 3.2 Software / Userspace Gaps

| # | Feature | LimeSuiteNG | m2sdr | Priority |
|---|---|---|---|---|
| S1 | **TX timestamp encoding into DMA header** | `FPGA_TxDataPacket.counter` | ❌ (fake counter) | **Critical** |
| S2 | **SoapySDR HAS_TIME → hardware gate** (not polling) | `waitForTimestamp` → FPGA | ❌ (busy-wait) | **Critical** |
| S3 | **END_BURST → gateware burst end flag** | `flushPartialPacket` | ❌ | **Critical** |
| S4 | Late packet / underrun status readback | `stats.underrun`, `PCT_LOSS_FLG` CSR | ❌ | High |
| S5 | TX advance monitoring (txAdvance metric) | `AvgRmsCounter` | ❌ | High |
| S6 | Automatic IQ/DC calibration on freq change | MCU `mcu_dc_iq_calibration` | Manual | Medium |
| S7 | Multi-format support (I12, I16, F32) | `samplesConversion.h` | Fixed (I16) | Medium |
| S8 | NR-aligned sample rate configuration | Implicit | Manual | Medium |
| S9 | CFR configuration API | `CrestFactorReduction` | ❌ | Low |
| S10 | VRT TX path (Ethernet mode) | N/A | ❌ (RX only) | Low |

### 3.3 System / Reference Clock Gaps

| # | Feature | Requirement | m2sdr current | Gap |
|---|---|---|---|---|
| C1 | Frequency accuracy | ≤ ±50 ppb (3GPP TS 38.104) | ±10 ppm (SI5351 free-run) | **Critical** — needs ext. 10 MHz GPSDO or WR |
| C2 | Phase noise | EVM < –40 dB for 256QAM | Marginal at 2.4 GHz | Medium — OCXO reference recommended |
| C3 | Sample rate = NR-aligned clock chain | 30.72/61.44/122.88 MSPS | ✅ (38.4 MHz ref → AD9361) | None |
| C4 | DMA buffer depth ≥ 4 NR slots | 4 × 120 KB @ 30.72 MSPS | ~256 × 8 KB = 2 MB | None |

---

## 4. Implementation Design

### 4.1 Gateware: TX Timestamp Arbiter (`timed_tx.py`)

New LiteX module to be inserted between `HeaderInserterExtractor` (TX extractor) and the
existing `TXRXLoopback` / AD9361 sink:

```
DMA TX → Header Extractor → [NEW: TimedTXArbiter] → Loopback Mux → AD9361 Sink
```

**Architecture:**

```
                ┌────────────────────────────────────┐
                │         TimedTXArbiter              │
  sink ─────────►  Hold FIFO (N packets deep)         │
  (from header  │  ┌──────────────────────────────┐   │
   extractor)   │  │ [hdr][ts][sample0]...[sampleN]│  │
                │  └──────────┬───────────────────┘   │
                │             │ ts_extracted           │
                │  ┌──────────▼──────────────────┐    │
                │  │  Comparator                  │    │
                │  │  time_gen.time >= ts?         │    │
                │  └──────────┬──────────────────┘    │
                │             │ fire                   │
                │  ┌──────────▼──────────────────┐    │
  source ◄───────  │  Gate + Underrun Zero Fill   │    │
  (to loopback)  │  └─────────────────────────────┘   │
                └────────────────────────────────────┘
```

**LiteX Python design:**

```python
class TimedTXArbiter(LiteXModule):
    """
    Holds TX stream packets until TimeGenerator.time >= extracted TX timestamp.
    Packet format: word0=sync, word1=timestamp_ns, word2..N=samples.
    Drops late packets and increments late_count CSR.
    Outputs zeros (silence) when no valid burst is ready for the current time.
    """
    def __init__(self, time_gen, fifo_depth=2048, data_width=64):
        self.sink   = sink   = stream.Endpoint(dma_layout(data_width))
        self.source = source = stream.Endpoint(dma_layout(data_width))

        # CSRs
        self._late_count    = CSRStatus(32, description="Late packets dropped")
        self._underrun_count= CSRStatus(32, description="Underrun cycles")
        self._burst_end_seen= CSRStatus(1,  description="Last burst_end flag seen")
        self._enable        = CSRStorage(1, reset=1, description="Enable timed TX (1) or pass-through (0)")

        # Internal FIFO to buffer incoming packets
        self.fifo = fifo = stream.SyncFIFO(dma_layout(data_width), fifo_depth)
        self.comb += sink.connect(fifo.sink)

        # State machine
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")

        ts_reg    = Signal(64)   # extracted timestamp from packet word 1
        hdr_count = Signal(2)    # 0=sync word, 1=timestamp word, 2+=samples
        late_count    = Signal(32)
        underrun_count= Signal(32)

        self.comb += [
            self._late_count.status.eq(late_count),
            self._underrun_count.status.eq(underrun_count),
        ]

        fsm.act("IDLE",
            # Wait for packet start in FIFO
            If(fifo.source.valid & fifo.source.first,
                NextState("READ_HEADER")
            ).Else(
                # Output silence (zeros) while waiting
                source.valid.eq(1),
                source.data.eq(0),
                source.first.eq(0),
                source.last.eq(0),
                NextValue(underrun_count, underrun_count + 1),
            )
        )

        fsm.act("READ_HEADER",
            # Word 0: sync word — consume silently
            fifo.source.ready.eq(1),
            If(fifo.source.valid,
                NextState("READ_TIMESTAMP")
            )
        )

        fsm.act("READ_TIMESTAMP",
            # Word 1: 64-bit timestamp
            fifo.source.ready.eq(1),
            If(fifo.source.valid,
                NextValue(ts_reg, fifo.source.data),
                NextState("WAIT_TIME")
            )
        )

        fsm.act("WAIT_TIME",
            # Hold until time_gen.time >= ts_reg
            If(time_gen.time >= ts_reg,
                NextState("STREAM")
            ).Elif(time_gen.time[63:32] > ts_reg[63:32],
                # Late: time has passed the target
                NextValue(late_count, late_count + 1),
                NextState("DROP_PACKET")
            )
        )

        fsm.act("STREAM",
            # Pass samples to output
            fifo.source.connect(source),
            If(source.valid & source.ready & source.last,
                NextState("IDLE")
            )
        )

        fsm.act("DROP_PACKET",
            # Drain remaining samples of late packet
            fifo.source.ready.eq(1),
            If(fifo.source.valid & fifo.source.last,
                NextState("IDLE")
            )
        )
```

**Integration in `litex_m2sdr.py`:**

```python
# After HeaderInserterExtractor (TX extractor):
self.timed_tx = TimedTXArbiter(time_gen=self.time_gen)
self.comb += [
    header.tx.source.connect(self.timed_tx.sink),
    self.timed_tx.source.connect(loopback.tx_sink),
]
```

### 4.2 Gateware: TX Burst Flags in DMA Header

Extend the 64-bit sync word (header word 0) to carry burst control flags. The current sync
word is a fixed magic value `0x5aa55aa55aa55aa5`. Repurpose the upper 16 bits as flags:

```
Word 0 layout (64-bit):
  [63:48] flags:
     bit 63: TX_HAS_TIME  — timestamp in word 1 is valid
     bit 62: TX_END_BURST — last packet of this burst; insert silence after
     bit 61: TX_START     — first packet of a new burst
     bit 60: TX_MIMO      — 2T2R mode; ch-A and ch-B interleaved
  [47:0]  sync magic: 0x5aa55aa5ffff (lower 48 bits)
Word 1: 64-bit timestamp (nanoseconds from TimeGenerator)
Word 2+: sample data
```

Modify `HeaderInserterExtractor` extractor path to latch flags into a new CSR:

```python
self._tx_flags = CSRStatus(16, description="TX burst flags from last packet header")
```

### 4.3 Gateware: TDD Symbol-Boundary Switch (`tdd_switch.py`)

New module that drives a GPIO output (PA_EN / TDD_SW) synchronously with the TX burst
state, using sample-counter-based lead/lag offsets:

```python
class TDDSwitch(LiteXModule):
    """
    Drives a TDD GPIO output (PA_EN / TR_SW) aligned to TX burst boundaries,
    with programmable lead (assert PA_EN before TX starts) and
    trail (hold PA_EN after TX ends) offsets in sample counts.
    """
    def __init__(self, time_gen, timed_tx_arbiter):
        self.pa_en = Signal()   # connect to GPIO pad

        self._lead_samples  = CSRStorage(16, reset=20, description="PA_EN lead time in samples")
        self._trail_samples = CSRStorage(16, reset=10, description="PA_EN trail time in samples")
        self._enable        = CSRStorage(1,  reset=1)
        self._invert        = CSRStorage(1,  reset=0)

        # Monitor timed_tx_arbiter state
        # Assert pa_en = _lead_samples before STREAM state fires
        # De-assert pa_en = _trail_samples after last sample of burst
        # ... (full FSM implementation)
```

This requires adding a `tx_active` status signal to `TimedTXArbiter`.

### 4.4 Gateware: Sample-Aligned DMA Frame Sizes

The current default `frame_cycles = 1022` yields 1022 × 8 bytes = 8,176 bytes ≈ 2044 samples
per DMA frame. For NR μ=1 at 30.72 MSPS, one slot = 15,360 samples (2 ch × 4 bytes = 6 × slot
samples). The `frame_cycles` CSR already allows software to change this; the driver must set it
to slot-aligned values at stream activation:

| NR config | Sample rate | Slot samples | frame_cycles (2ch, 4B/sample) |
|---|---|---|---|
| μ=0, 20 MHz | 30.72 MSPS | 15,360 | 7,680 |
| μ=1, 40 MHz | 61.44 MSPS | 15,360 | 7,680 |
| μ=1, 100 MHz | 122.88 MSPS | 15,360 | 7,680 |

Software change in `LiteXM2SDRDevice.cpp`:
```cpp
// In setSampleRate():
uint32_t slot_samples = (uint32_t)(sampleRate * 0.5e-3);  // 0.5ms per slot
uint32_t frame_cycles = slot_samples - 2;  // subtract 2 header words
litepcie_writel(fd, CSR_HEADER_RX_FRAME_CYCLES_ADDR, frame_cycles);
litepcie_writel(fd, CSR_HEADER_TX_FRAME_CYCLES_ADDR, frame_cycles);
```

### 4.5 Gateware: Sample Counter (Integer Samples)

The current `TimeGenerator` counts in fractional nanoseconds. For the timed TX comparator,
a pure integer sample counter avoids floating-point and clock-frequency-dependent conversions.
Add a `SampleCounter` alongside `TimeGenerator`:

```python
class SampleCounter(LiteXModule):
    """
    64-bit sample counter, incremented by the valid pulse from the RX CDC output.
    Provides an integer sample-domain timestamp (matches LimeSDR_GW SAMPLE_NR).
    """
    def __init__(self, rx_valid):
        self.count = Signal(64)
        self._read = CSRStorage(1)
        self._count = CSRStatus(64)

        latch = Signal(64)
        self.sync += [
            If(rx_valid, self.count.eq(self.count + 1)),
            If(self._read.storage, latch.eq(self.count)),
        ]
        self.comb += self._count.status.eq(latch)
```

The `TimedTXArbiter` can use either the ns-domain `TimeGenerator.time` (more flexible,
works with WR and PTM) or the sample-domain `SampleCounter.count` (simpler for
srsRAN-style integer sample timestamps).

---

## 5. Software Implementation Design

### 5.1 SoapySDR: Encode Real TX Timestamp into DMA Header

Replace the fake timestamp in `LiteXM2SDRStreaming.cpp` `acquireWriteBuffer()`:

```cpp
// Current (WRONG):
uint64_t fakeTimestamp = ...;
memcpy(buf + 8, &fakeTimestamp, 8);

// Required:
// 1. Build flags word
uint16_t flags = 0;
if (hasTime)     flags |= TX_FLAG_HAS_TIME;
if (endBurst)    flags |= TX_FLAG_END_BURST;
uint64_t syncWord = (0x5aa55aa5ULL) | ((uint64_t)flags << 48);
memcpy(buf + 0, &syncWord, 8);

// 2. Write requested TX timestamp (ns) into header word 1
uint64_t timeNs_le = htole64((uint64_t)timeNs);
memcpy(buf + 8, &timeNs_le, 8);
```

Remove the busy-wait polling loop from `activateStream()` — the gateware arbiter now handles
the timing. The host submits buffers ahead of time (target: 2–4 ms lead), and the FPGA
fires at the exact nanosecond.

### 5.2 SoapySDR: END_BURST Flag Propagation

```cpp
// In releaseWriteBuffer():
if (flags & SOAPY_SDR_END_BURST) {
    // Set TX_FLAG_END_BURST in the DMA header of the submitted buffer
    uint64_t syncWord;
    memcpy(&syncWord, buf + 0, 8);
    syncWord |= ((uint64_t)TX_FLAG_END_BURST << 48);
    memcpy(buf + 0, &syncWord, 8);
}
```

### 5.3 SoapySDR: Late Packet and Underrun Readback

```cpp
// In readStreamStatus():
uint32_t late    = litepcie_readl(fd, CSR_TIMED_TX_LATE_COUNT_ADDR);
uint32_t underrun= litepcie_readl(fd, CSR_TIMED_TX_UNDERRUN_COUNT_ADDR);
if (late > _last_late) {
    chanMask = 1;
    flags    = SOAPY_SDR_TIME_ERROR;
    timeNs   = getHardwareTime("");
    return SOAPY_SDR_TIME_ERROR;
}
```

### 5.4 SoapySDR: NR Sample Rate Setup

```cpp
void LiteXM2SDRDevice::setSampleRate(const int direction, const size_t channel,
                                      const double rate) {
    // ... existing AD9361 config ...

    // Set slot-aligned DMA frame size
    uint32_t slot_samples = (uint32_t)(rate * 500e-6);  // 0.5 ms per NR slot
    uint32_t frame_cycles  = slot_samples - 2;           // minus 2 header words
    litepcie_writel(_fd, CSR_HEADER_RX_FRAME_CYCLES_ADDR, frame_cycles);
    litepcie_writel(_fd, CSR_HEADER_TX_FRAME_CYCLES_ADDR, frame_cycles);
}
```

### 5.5 Reference Clock: External 10 MHz

For production NR operation the SI5351C `sync_clk_in` (uFL connector) must be driven from
a GPSDO or OCXO. The existing SI5351 config `si5351_clkin_10m_38p4m_config` handles this.
Software change in `m2sdr_rf` and SoapySDR device setup:

```cpp
// Enable external sync (already supported by m2sdr_rf via -sync=external):
// This generates AD9361 refclk at refclk_freq from the external 10 MHz
m2sdr_rf -sync=external -refclk_freq=38400000
```

Document required hardware: GPS-disciplined 10 MHz source (e.g., Leo Bodnar GPS Reference,
Trimble Resolution T, or Z3801A).

---

## 6. Optional DSP Blocks

### 6.1 CFR (Crest Factor Reduction)

For deployments with an external PA, add a pipeline stage after the DMA sink:

```
DMA TX → Header Extractor → TimedTXArbiter → [CFR] → Loopback Mux → AD9361
```

LimeSuiteNG's CFR (`CrestFactorReduction.h`) uses:
1. Threshold detection: `|I|² + |Q|² > threshold²`
2. Delay buffer to align peak with cancellation pulse
3. 32-tap FIR shaping filter on the cancellation signal
4. Iterate 2–4 times for better PAPR reduction (4–6 dB typical)

Xilinx DSP48E1 slices are available in abundance on the XC7A200T; a 32-tap complex FIR
costs ~8 DSP48 slices per stage, well within budget.

Configurable CSRs: `cfr_threshold`, `cfr_enable`, `cfr_iterations`.

### 6.2 IQ Imbalance Correction (RX)

The AD9361 on-chip IQ calibration covers initial calibration but drifts with temperature.
A digital correction in the FPGA RX path maintains EVM over time:

```
AD9361 PHY → GPIO Packer → RX CDC → [IQ Corrector] → BitMode → Header → DMA
```

Correction matrix (2×2 real):
```
I_out = a*I_in + b*Q_in
Q_out = c*I_in + d*Q_in
```
Coefficients written via CSR, computed by host software from pilot symbols or loopback
measurement. bladeRF implements this in `hdl/fpga/ip/nuand/synthesis/iq_correction.vhd`.

### 6.3 DC Offset Filter (RX)

A simple first-order IIR DC blocker per channel:
```
y[n] = x[n] - x[n-1] + α·y[n-1]
```
α ≈ 1 - 2⁻¹⁵ for a corner frequency well below 1 kHz at 30.72 MSPS.
bladeRF implements this in `hdl/fpga/ip/analogdevicesinc/hdl/library/common/ad_dcfilter.v`.

---

## 7. Implementation Phases

### Phase 1 — Minimum Viable 5G NR (Gateware + Driver)

**Gateware changes:**
1. `gateware/timed_tx.py`: New `TimedTXArbiter` module
2. `gateware/header.py`: Add burst flags to sync word (bits [63:48])
3. `litex_m2sdr.py`: Insert `TimedTXArbiter` in TX pipeline; add `SampleCounter`

**Software changes:**
4. `soapysdr/LiteXM2SDRStreaming.cpp`: Encode real TX timestamp + flags into DMA header
5. `soapysdr/LiteXM2SDRDevice.cpp`: Set slot-aligned `frame_cycles` in `setSampleRate()`
6. `soapysdr/LiteXM2SDRStreaming.cpp`: Implement `readStreamStatus()` reading late/underrun CSRs

**External requirement:**
- 10 MHz GPSDO or OCXO into SI5351C uFL (for frequency accuracy)

**Enables:** srsRAN/OAI gNB in small-cell mode, μ=0 (20 MHz BW), FR1.

### Phase 2 — Full TDD + Improved EVM

**Gateware changes:**
7. `gateware/tdd_switch.py`: New `TDDSwitch` with sample-counter lead/lag
8. `litex_m2sdr.py`: Wire `TDDSwitch` to GPIO output (configurable pin)
9. `gateware/ad9361/core.py`: Drive `txnrx` from `TDDSwitch` (not static CSR)
10. `gateware/header.py`: RX timestamp mixed mode (PPS seconds + sub-second ticks)

**Software changes:**
11. `soapysdr/LiteXM2SDRDevice.cpp`: TDD switch API (setAntenna or custom kwargs)
12. `software/user/m2sdr_rf.c`: Add `-tdd_lead`, `-tdd_trail` options

**Enables:** Clean TX/RX switching for TDD NR, PA_EN control for external amplifier, better
adjacent channel leakage compliance.

### Phase 3 — DSP and Calibration

**Gateware changes:**
13. `gateware/cfr.py`: CFR (clip + FIR), configurable threshold, enable CSR
14. `gateware/iq_correction.py`: 2×2 IQ correction matrix (4 × 18-bit CSR coefficients)
15. `gateware/dc_filter.py`: IIR DC blocker per RX channel
16. `litex_m2sdr.py`: Wire CFR into TX path, IQ/DC into RX path

**Software changes:**
17. `soapysdr/LiteXM2SDRDevice.cpp`: Expose CFR threshold as SoapySDR custom arg
18. Offline calibration utility: Measure IQ imbalance via loopback, write correction coefficients

**Enables:** 256QAM EVM compliance, better PA efficiency, production-grade RF performance.

### Phase 4 — ORAN / Fronthaul (Optional)

For split-RU deployments using the baseboard Ethernet:
- eCPRI framing layer in gateware (significant effort)
- IEEE 1588v2 PTP class C timing (White Rabbit already provides this)
- O-RAN 7.2x interface to an O-DU

This phase is only required for O-RAN-compliant deployments; a standalone srsRAN/OAI setup
on the host does not need it.

---

## 8. Features Not Required for 5G NR

| Feature | Reason not needed |
|---|---|
| DPD (Digital Pre-Distortion) | AD9361 output ≤ 0 dBm; no external PA distortion to correct at this power level |
| PRACH gateware accelerator | srsRAN/OAI PRACH detector runs in software on x86; 100 µs compute, within budget |
| SSB/PSS/SSS hardware | gNB pre-computes IQ sequences; TX as normal DMA burst |
| HARQ buffer in FPGA | All HARQ logic in srsRAN/OAI host software |
| Carrier aggregation (inter-band) | AD9361 single LO; intra-band CA within 56 MHz analog BW is sufficient |
| NR μ=3 (120 kHz FR2 / mmWave) | AD9361 max ~122.88 MSPS; FR2 requires >245 MSPS — needs a different RFIC |
| MAC/scheduling offload | srsRAN/OAI handle all L2/L3 on the host CPU |

---

## 9. Key Reference Files

| File | What to study |
|---|---|
| `/home/dmd/LimeSDR_GW/gateware/LimeDFB_LiteX/tx_path_top/src/pct2data_buf_rd.vhd` | Reference timed TX arbiter (adapt to LiteX/Migen) |
| `/home/dmd/bladeRF/hdl/fpga/ip/nuand/time_tamer/vhdl/time_tamer.vhd` | Alternative: dedicated timer IP with compare-and-trigger |
| `/home/dmd/bladeRF/hdl/fpga/ip/nuand/synthesis/iq_correction.vhd` | IQ imbalance correction reference |
| `/home/dmd/bladeRF/hdl/fpga/ip/analogdevicesinc/hdl/library/common/ad_dcfilter.v` | DC offset filter reference |
| `/home/dmd/LimeSuiteNG/src/streaming/TRXLooper.cpp` | TX scheduling advance, underrun metrics |
| `/home/dmd/LimeSuiteNG/src/streaming/DataPacket.h` | Packet header format reference |
| `/home/dmd/LimeSuiteNG/src/DSP/CFR/CrestFactorReduction.h` | CFR design reference |
| `/home/dmd/m2sdr/litex_m2sdr/gateware/time.py` | Existing TimeGenerator to connect to arbiter |
| `/home/dmd/m2sdr/litex_m2sdr/gateware/header.py` | Extend for burst flags |
| `/home/dmd/m2sdr/litex_m2sdr/software/soapysdr/LiteXM2SDRStreaming.cpp` | Main file to update for real timestamp encoding |

---

## 10. Work Items (Persistent TODO Tracker)

Status legend: `[ ]` TODO · `[~]` IN PROGRESS · `[x]` DONE

Exact wiring in `litex_m2sdr.py` (verified):
```
crossbar.mux.source → header.tx.sink
header.tx.source    → txrx_loopback.tx_sink   (line 622)  ← TimedTXArbiter inserts here
txrx_loopback.tx_source → ad9361.sink          (line 623)
ad9361.source → txrx_loopback.rx_sink          (line 628)
txrx_loopback.rx_source → header.rx.sink       (line 629)
```

---

### Phase 1 — Minimum Viable 5G NR

#### GW-1 · Create `litex_m2sdr/gateware/timed_tx.py`

**Goal:** Hold TX stream packets in a FIFO and release them only when
`time_gen.time >= extracted_timestamp`. Output silence (zeros) when no burst is ready.

**New file:** `litex_m2sdr/gateware/timed_tx.py`

**Class:** `TimedTXArbiter(LiteXModule)`

**Constructor args:**
- `time_gen` — reference to the `TimeGenerator` instance (provides `.time` Signal(64))
- `fifo_depth=2048` — depth of internal `SyncFIFO` in 64-bit words
- `data_width=64`

**Ports:**
- `self.sink`   — `stream.Endpoint(dma_layout(data_width))` — input from HeaderExtractor
- `self.source` — `stream.Endpoint(dma_layout(data_width))` — output to TXRXLoopback
- `self.tx_burst_active` — `Signal()` — high while in STREAM state (used by TDDSwitch in Phase 2)

**Internal FIFO:** `stream.SyncFIFO(dma_layout(data_width), fifo_depth)` — buffers the incoming
packet stream. `sink` connects directly to `fifo.sink`.

**FSM states:**

| State | Action |
|---|---|
| `IDLE` | If `fifo.source.valid & fifo.source.first`: go to `READ_HEADER`. Else: drive `source.valid=1, source.data=0, source.first=0, source.last=0` (silence) and increment `underrun_count`. |
| `READ_HEADER` | Consume word 0 (sync+flags) from FIFO: `fifo.source.ready=1`. Latch `fifo.source.data[48:64]` into `flags_reg`. When `fifo.source.valid`: go to `READ_TIMESTAMP`. |
| `READ_TIMESTAMP` | Consume word 1 (timestamp_ns): `fifo.source.ready=1`. Latch `fifo.source.data` into `ts_reg`. When `fifo.source.valid`: go to `WAIT_TIME` if `TX_FLAG_HAS_TIME` set in `flags_reg`, else go directly to `STREAM`. |
| `WAIT_TIME` | Hold. If `time_gen.time >= ts_reg`: go to `STREAM`. If `time_gen.time[63:32] > ts_reg[63:32]` (late by >1 s, guards against 64-bit wrap false positives): increment `late_count`, go to `DROP_PACKET`. |
| `STREAM` | Pass FIFO samples to `source` (`fifo.source.connect(source)`). Assert `tx_burst_active=1`. On `source.valid & source.ready & source.last`: go to `IDLE`. |
| `DROP_PACKET` | Drain remaining FIFO words of this packet: `fifo.source.ready=1`. On `fifo.source.valid & fifo.source.last`: go to `IDLE`. |

**Late detection precision:** compare using the full 64-bit ns value. Only flag as late when
`time_gen.time > ts_reg` (strictly greater). Equality fires STREAM.

**CSRs (via `add_csr()` called in `__init__`):**
- `_enable`          — CSRStorage(1, reset=1) — pass-through mode when 0 (skip WAIT_TIME)
- `_late_count`      — CSRStatus(32) — incremented on every dropped late packet
- `_underrun_count`  — CSRStatus(32) — incremented every cycle silence is output in IDLE

**Flag constants (define at top of file):**
```python
TX_FLAG_HAS_TIME    = (1 << 15)
TX_FLAG_END_BURST   = (1 << 14)
TX_FLAG_START_BURST = (1 << 13)
```

**Status:** `[ ]`

---

#### GW-2 · Extend `litex_m2sdr/gateware/header.py` — burst flags in TX sync word

**Goal:** Use the upper 16 bits of the 64-bit TX header word (the sync/magic word) as burst
control flags. The lower 48 bits remain `0x5aa55aa5ffff` (any stable magic is fine).

**Changes to `header.py`:**

1. Add module-level constants:
   ```python
   TX_FLAG_HAS_TIME    = (1 << 15)
   TX_FLAG_END_BURST   = (1 << 14)
   TX_FLAG_START_BURST = (1 << 13)
   TX_SYNC_MAGIC       = 0x5aa5_5aa5_5aa5  # lower 48 bits
   ```

2. In `HeaderInserterExtractor.__init__` (extractor path, `mode="extractor"`):
   - Add `self.tx_flags = Signal(16)` as an output
   - In the `HEADER` FSM state, after latching `self.header`, also:
     ```python
     NextValue(self.tx_flags, sink.data[48:64])
     ```

3. In `TXRXHeader.__init__`: add a new CSR:
   ```python
   self.last_tx_flags = CSRStatus(16, description="Burst flags from last TX header word.")
   ```
   Wire it:
   ```python
   If(self.tx.update,
       self.last_tx_flags.status.eq(self.tx.tx_flags),
   )
   ```

**No changes needed to RX header inserter.** The RX sync word stays as-is.

**Status:** `[ ]`

---

#### GW-3 · Wire `TimedTXArbiter` in `litex_m2sdr.py`

**Goal:** Insert `TimedTXArbiter` between `header.tx.source` and `txrx_loopback.tx_sink`.

**Changes to `litex_m2sdr.py`:**

1. Add import at top:
   ```python
   from litex_m2sdr.gateware.timed_tx import TimedTXArbiter
   ```

2. After `self.header = TXRXHeader(data_width=64)` (around line 607), add:
   ```python
   self.timed_tx = TimedTXArbiter(time_gen=self.time_gen, data_width=64)
   ```

3. Replace lines 621–623:
   ```python
   # Before:
   self.comb += [
       self.header.tx.source.connect(self.txrx_loopback.tx_sink),
       self.txrx_loopback.tx_source.connect(self.ad9361.sink),
   ]
   # After:
   self.comb += [
       self.header.tx.source.connect(self.timed_tx.sink),
       self.timed_tx.source.connect(self.txrx_loopback.tx_sink),
       self.txrx_loopback.tx_source.connect(self.ad9361.sink),
   ]
   ```

**Status:** `[ ]`

---

#### SW-1 · Real TX timestamp in `soapysdr/LiteXM2SDRStreaming.cpp`

**Goal:** Replace the fake incrementing counter with the actual requested TX timestamp and
burst flags. Remove the busy-wait polling loop from TX. Always enable the 16-byte TX header.

**Changes to `LiteXM2SDRStreaming.cpp`:**

1. Remove the `#if USE_LITEPCIE && defined(_TX_DMA_HEADER_TEST)` compile guard (lines 97–100).
   Make `TX_DMA_HEADER_SIZE = 16` unconditional.

2. Add flag constants near the top (after includes):
   ```cpp
   static constexpr uint16_t TX_FLAG_HAS_TIME    = (1 << 15);
   static constexpr uint16_t TX_FLAG_END_BURST   = (1 << 14);
   static constexpr uint16_t TX_FLAG_START_BURST = (1 << 13);
   static constexpr uint64_t TX_SYNC_MAGIC       = 0x5aa55aa55aa5ULL; // lower 48 bits
   ```

3. Add a helper `writeTxHeader(void* buf, long long timeNs, int soapyFlags)`:
   ```cpp
   static void writeTxHeader(void* buf, long long timeNs, int soapyFlags) {
       uint16_t flags = 0;
       if (soapyFlags & SOAPY_SDR_HAS_TIME)  flags |= TX_FLAG_HAS_TIME;
       if (soapyFlags & SOAPY_SDR_END_BURST) flags |= TX_FLAG_END_BURST;
       uint64_t word0 = TX_SYNC_MAGIC | ((uint64_t)flags << 48);
       uint64_t word1 = (uint64_t)timeNs;
       memcpy((uint8_t*)buf + 0, &word0, 8);
       memcpy((uint8_t*)buf + 8, &word1, 8);
   }
   ```

4. In `acquireWriteBuffer()` (around line 832): remove the entire fake-timestamp block
   (lines 881–901). Store `timeNs` and `flags` into the stream struct for use in
   `releaseWriteBuffer()`.

5. In `releaseWriteBuffer()` (line 926): call `writeTxHeader(buf, timeNs, flags)` using the
   `timeNs` and `flags` passed by the caller.

6. In `activateStream()` for TX (line 465–469): remove the busy-wait polling loop (lines
   932–942). The FPGA arbiter now handles timing; the host must only ensure buffers are
   submitted far enough in advance (recommended: 2–4 ms lead time).

7. Update `_tx_buf_size` computation (line 295) — no change needed since `TX_DMA_HEADER_SIZE`
   is already being subtracted.

**Status:** `[ ]`

---

#### SW-2 · Slot-aligned frame size in `soapysdr/LiteXM2SDRDevice.cpp`

**Goal:** Set DMA frame size to one NR slot (0.5 ms) on both TX and RX header inserters
whenever `setSampleRate()` is called.

**Changes to `LiteXM2SDRDevice.cpp`:**

In `setSampleRate(direction, channel, rate)`, after the existing AD9361 sample rate config,
add (use the existing `litepcie_writel` / CSR write mechanism already present for other CSRs):

```cpp
// Set DMA frame size = 1 NR slot = 0.5 ms worth of samples
// frame_cycles counts 64-bit words; each word = 4 bytes × 2 ch = 8 bytes = 4 complex samples
uint32_t slot_samples  = (uint32_t)(rate * 0.5e-3);      // samples per 0.5 ms slot
uint32_t frame_cycles  = (slot_samples / 4) - 2;          // 64-bit words minus 2 header words
litepcie_writel(_fd, CSR_HEADER_RX_FRAME_CYCLES_ADDR, frame_cycles);
litepcie_writel(_fd, CSR_HEADER_TX_FRAME_CYCLES_ADDR, frame_cycles);
```

This affects both RX (where frame_cycles controls RX packet size) and TX (where it controls
how many sample words the extractor expects per header). Ensure CSR names match what gets
generated after GW-3 is built (check `csr.h` after build).

**Status:** `[ ]`

---

#### SW-3 · Late/underrun status in `soapysdr/LiteXM2SDRStreaming.cpp`

**Goal:** Implement `readStreamStatus()` to surface late-packet and underrun counters from
the `TimedTXArbiter` CSRs to the SoapySDR caller.

**Changes to `LiteXM2SDRStreaming.cpp`:**

Implement (or extend if it already exists) `readStreamStatus()`:

```cpp
int SoapyLiteXM2SDR::readStreamStatus(SoapySDR::Stream* stream,
    size_t& chanMask, int& flags, long long& timeNs, const long long timeoutUs) {

    // Only meaningful for TX stream
    if (stream != _tx_stream.stream) return SOAPY_SDR_NOT_SUPPORTED;

    uint32_t late    = litepcie_readl(_fd, CSR_TIMED_TX_LATE_COUNT_ADDR);
    uint32_t underrun= litepcie_readl(_fd, CSR_TIMED_TX_UNDERRUN_COUNT_ADDR);

    if (late > _tx_stream.last_late_count) {
        _tx_stream.last_late_count = late;
        chanMask = 1;
        flags    = SOAPY_SDR_TIME_ERROR;
        timeNs   = (long long)getHardwareTime("");
        return SOAPY_SDR_TIME_ERROR;
    }
    if (underrun > _tx_stream.last_underrun_count) {
        _tx_stream.last_underrun_count = underrun;
        chanMask = 1;
        flags    = SOAPY_SDR_UNDERFLOW;
        timeNs   = (long long)getHardwareTime("");
        return SOAPY_SDR_UNDERFLOW;
    }
    return SOAPY_SDR_TIMEOUT;
}
```

Add `last_late_count` and `last_underrun_count` fields to the TX stream struct in
`LiteXM2SDRDevice.hpp`.

CSR names (`CSR_TIMED_TX_LATE_COUNT_ADDR`, `CSR_TIMED_TX_UNDERRUN_COUNT_ADDR`) will be
auto-generated by LiteX after GW-1 and GW-3 are built. Check `csr.h` to confirm names.

**Status:** `[ ]`

---

#### Ph1-commit · Commit Phase 1

After all Phase 1 items are done and tested (at minimum: `python3 -m pytest -v test` passes,
gateware builds without errors):

```bash
git add litex_m2sdr/gateware/timed_tx.py \
        litex_m2sdr/gateware/header.py \
        litex_m2sdr.py \
        litex_m2sdr/software/soapysdr/LiteXM2SDRStreaming.cpp \
        litex_m2sdr/software/soapysdr/LiteXM2SDRDevice.cpp \
        litex_m2sdr/software/soapysdr/LiteXM2SDRDevice.hpp
git commit -m "nr: phase 1 — timed TX arbiter, burst flags, slot-aligned DMA frames"
```

**Status:** `[ ]`

---

### Phase 2 — TDD Clean Switching

#### GW-4 · Create `litex_m2sdr/gateware/tdd_switch.py`

**Goal:** Drive a GPIO-connected PA_EN / TDD_SW signal with programmable lead/lag offsets
relative to TX burst start/end, synchronized to the sample clock.

**New file:** `litex_m2sdr/gateware/tdd_switch.py`

**Class:** `TDDSwitch(LiteXModule)`

**Constructor args:**
- `timed_tx_arbiter` — reference to `TimedTXArbiter` instance (provides `.tx_burst_active`)
- `sys_clk_freq` — used to document timing (offsets in samples at rfic rate, not sys cycles)

**Ports:**
- `self.pa_en` — `Signal()` — connect to GPIO pad in `litex_m2sdr.py`

**CSRs:**
- `_lead_samples`  — CSRStorage(16, reset=20) — assert PA_EN this many samples before TX burst
- `_trail_samples` — CSRStorage(16, reset=10) — hold PA_EN this many samples after TX burst ends
- `_enable`        — CSRStorage(1, reset=1)
- `_invert`        — CSRStorage(1, reset=0)  — invert output polarity

**FSM states:**

| State | Trigger | Action |
|---|---|---|
| `IDLE` | `tx_burst_active` rises | Load `lead_counter = _lead_samples.storage`, go to `LEAD` |
| `LEAD` | Count down `lead_counter` | Assert `pa_en=1`. When `lead_counter==0`, go to `TX_ON` |
| `TX_ON` | `tx_burst_active` falls | Assert `pa_en=1`. On fall: load `trail_counter`, go to `TRAIL` |
| `TRAIL` | Count down `trail_counter` | Assert `pa_en=1`. When `trail_counter==0`, go to `IDLE`, de-assert |

Note: Lead/lag counters count in `sys_clk` cycles. For sample-accurate timing, the CDC
latency from rfic→sys domain (~5 cycles) should be subtracted from `_lead_samples` in
software or accounted for in default reset values.

**Output:**
```python
self.comb += self.pa_en.eq(
    Mux(self._enable.storage,
        Mux(self._invert.storage, ~pa_en_int, pa_en_int),
        0)
)
```

**Status:** `[ ]`

---

#### GW-5 · Drive `txnrx` from `TDDSwitch` in `litex_m2sdr/gateware/ad9361/core.py`

**Goal:** When TDD mode is enabled, drive the AD9361 `txnrx` pin from `TDDSwitch.pa_en`
instead of the static CSR bit.

**Current state in `core.py`:**
- `txnrx` is driven by `_config.fields.txnrx` (a static CSR bit)

**Changes to `core.py`:**

1. Add an input signal to `AD9361RFIC`:
   ```python
   self.txnrx_override = Signal()       # i: external TDD signal
   self.txnrx_use_override = Signal()   # i: 1 = use override, 0 = use CSR
   ```

2. Change the `txnrx` output assignment from:
   ```python
   self.comb += pads.txnrx.eq(self._config.fields.txnrx)
   ```
   to:
   ```python
   self.comb += pads.txnrx.eq(
       Mux(self.txnrx_use_override, self.txnrx_override, self._config.fields.txnrx)
   )
   ```

**Status:** `[ ]`

---

#### GW-6 · Wire `TDDSwitch` in `litex_m2sdr.py`

**Goal:** Instantiate `TDDSwitch`, wire its output to both a GPIO pad (PA_EN) and
`ad9361.txnrx_override`.

**Changes to `litex_m2sdr.py`:**

1. Add `with_tdd=False` to `BaseSoC.__init__` argument list.

2. Add import:
   ```python
   from litex_m2sdr.gateware.tdd_switch import TDDSwitch
   ```

3. After `self.timed_tx = ...`, add (conditionally):
   ```python
   if with_tdd:
       self.tdd_switch = TDDSwitch(
           timed_tx_arbiter=self.timed_tx,
           sys_clk_freq=sys_clk_freq,
       )
       # Wire to AD9361 txnrx override
       self.comb += [
           self.ad9361.txnrx_override.eq(self.tdd_switch.pa_en),
           self.ad9361.txnrx_use_override.eq(1),
       ]
       # Wire to GPIO pad (use existing GPIO infrastructure or a dedicated pad)
       # platform.request("gpio", N) — pin assignment TBD by hardware
   ```

4. Add `--with-tdd` CLI argument in `main()`.

**Status:** `[ ]`

---

#### GW-7 · RX timestamp mixed mode in `litex_m2sdr/gateware/header.py`

**Goal:** Support a PPS-synchronized timestamp format in RX headers: upper 32 bits = whole
seconds (PPS count), lower 32 bits = sample ticks since last PPS.

**Changes to `header.py`:**

1. Add `ts_mode` input signal (or CSR) to `RXHeaderInserter`:
   - `ts_mode=0`: current behavior — pure 64-bit ns from `TimeGenerator`
   - `ts_mode=1`: mixed PPS+ticks — requires `pps_seconds` (32-bit) and `pps_ticks` (32-bit)
     inputs connected from PPSGenerator

2. In `RXHeaderInserter.__init__`, add optional inputs:
   ```python
   self.pps_seconds = Signal(32)  # i: seconds counter from PPSGenerator
   self.pps_ticks   = Signal(32)  # i: sample ticks since last PPS edge
   self.ts_mode     = Signal()    # i: 0=ns, 1=pps+ticks
   ```

3. In the `TIMESTAMP` FSM state, change:
   ```python
   source.data[0:64].eq(self.timestamp)
   ```
   to:
   ```python
   source.data[0:64].eq(Mux(self.ts_mode,
       Cat(self.pps_ticks, self.pps_seconds),
       self.timestamp
   ))
   ```

4. Add a sample tick counter (in rfic clock domain, CDC'd to sys) driven by `ad9361.source.valid`.
   Reset on PPS pulse. Connect to `pps_ticks`.

5. In `litex_m2sdr.py`, connect the new signals and add a `_ts_mode` CSR.

**Status:** `[ ]`

---

#### SW-4 · TDD kwargs in `soapysdr/LiteXM2SDRDevice.cpp`

**Goal:** Expose TDD switch lead/trail settings and enable as SoapySDR device kwargs.

**Changes to `LiteXM2SDRDevice.cpp`:**

In `getSettingInfo()` / `writeSetting()` (or the constructor kwargs handling):

```cpp
// In constructor, after parsing other kwargs:
if (args.count("tdd_enable"))
    litepcie_writel(_fd, CSR_TDD_SWITCH_ENABLE_ADDR, std::stoi(args.at("tdd_enable")));
if (args.count("tdd_lead"))
    litepcie_writel(_fd, CSR_TDD_SWITCH_LEAD_SAMPLES_ADDR, std::stoi(args.at("tdd_lead")));
if (args.count("tdd_trail"))
    litepcie_writel(_fd, CSR_TDD_SWITCH_TRAIL_SAMPLES_ADDR, std::stoi(args.at("tdd_trail")));
```

CSR names auto-generated after GW-6 build. Check `csr.h`.

**Status:** `[ ]`

---

#### SW-5 · TDD options in `software/user/m2sdr_rf.c`

**Goal:** Add TDD mode options to `m2sdr_rf`.

**Changes to `m2sdr_rf.c`:**

1. Add CLI options: `-tdd=on/off`, `-tdd_lead=N`, `-tdd_trail=N`
2. Parse and write to CSRs via `litepcie_writel` (same mechanism as other CSR writes in this file)

**Status:** `[ ]`

---

#### Ph2-commit · Commit Phase 2

```bash
git add litex_m2sdr/gateware/tdd_switch.py \
        litex_m2sdr/gateware/ad9361/core.py \
        litex_m2sdr/gateware/header.py \
        litex_m2sdr.py \
        litex_m2sdr/software/soapysdr/LiteXM2SDRDevice.cpp \
        litex_m2sdr/software/user/m2sdr_rf.c
git commit -m "nr: phase 2 — TDD switch with lead/lag, txnrx override, RX mixed timestamp"
```

**Status:** `[ ]`

---

### Phase 3 — DSP Blocks

#### GW-8 · Create `litex_m2sdr/gateware/cfr.py`

**Goal:** Clip-and-filter CFR to reduce TX PAPR by 4–6 dB for NR OFDM waveforms.

**Reference:** `/home/dmd/LimeSuiteNG/src/DSP/CFR/CrestFactorReduction.h`

**New file:** `litex_m2sdr/gateware/cfr.py`

**Class:** `CrestFactorReduction(LiteXModule)`

**Constructor args:** `data_width=64`, `fir_taps=32`

**Pipeline (all in sys clock domain, after CDC from rfic):**

```
sink (64-bit: IA/QA/IB/QB 16-bit each)
  │
  ▼
Peak Detector: compute |I|²+|Q|² for ch-A and ch-B per sample group
  │  threshold exceeded?
  ▼
Delay Buffer: holds input aligned to peak detection latency (~8 cycles)
  │
  ▼
Pulse Generator: when peak detected, emit shaped cancellation pulse
  │
  ▼
FIR Filter: 32-tap symmetric half-band filter on cancellation pulse
  │
  ▼
Subtractor: delayed_input - filtered_pulse → source (clamped to ±32767)
```

**CSRs:**
- `_threshold`  — CSRStorage(16, reset=32767) — |IQ|² threshold (sqrt: ~full scale)
- `_enable`     — CSRStorage(1, reset=0)
- `_clip_count` — CSRStatus(32) — counts clipped samples

**FIR coefficients:** hardcoded symmetric half-band in Python list; implemented using
DSP48E1 primitives via Migen. Use `litex.soc.cores.utils` or direct Migen DSP primitives.

**Integration point:** between `self.timed_tx.source` and `self.txrx_loopback.tx_sink`.

**Status:** `[ ]`

---

#### GW-9 · Create `litex_m2sdr/gateware/iq_correction.py`

**Goal:** Apply a 2×2 real correction matrix to compensate IQ amplitude/phase imbalance on
the RX path.

**Reference:** `/home/dmd/bladeRF/hdl/fpga/ip/nuand/synthesis/iq_correction.vhd`

**New file:** `litex_m2sdr/gateware/iq_correction.py`

**Class:** `IQCorrection(LiteXModule)`

**Constructor args:** `data_width=64`

**Math (per channel A and B independently):**
```
I_out = (a * I_in + b * Q_in) >> 14   # a, b are Q2.14 fixed-point
Q_out = (c * I_in + d * Q_in) >> 14
```
Default: `a=16384, b=0, c=0, d=16384` (identity, no correction)

**CSRs (per channel, 4 coefficients × 2 channels = 8 CSRs):**
- `_ch_a_coeff_a`, `_ch_a_coeff_b`, `_ch_a_coeff_c`, `_ch_a_coeff_d` — CSRStorage(18, signed)
- `_ch_b_coeff_a`, `_ch_b_coeff_b`, `_ch_b_coeff_c`, `_ch_b_coeff_d`
- `_enable` — CSRStorage(1, reset=0)

**Integration point:** in RX path between `rxrx_loopback.rx_source` and `header.rx.sink`
(i.e., after the AD9361 RX data exits loopback, before header insertion).

**Status:** `[ ]`

---

#### GW-10 · Create `litex_m2sdr/gateware/dc_filter.py`

**Goal:** First-order IIR DC blocker per RX channel to suppress LO leakthrough.

**Reference:** `/home/dmd/bladeRF/hdl/fpga/ip/analogdevicesinc/hdl/library/common/ad_dcfilter.v`

**New file:** `litex_m2sdr/gateware/dc_filter.py`

**Class:** `DCFilter(LiteXModule)`

**Constructor args:** `data_width=64`, `alpha_shift=15` (α = 1 - 2^-15)

**Per-channel IIR (I and Q independently, applied to ch-A and ch-B in same module):**
```
acc[n] = acc[n-1] + x[n] - (acc[n-1] >> alpha_shift)
y[n]   = x[n] - (acc[n] >> alpha_shift)
```
Corner frequency ≈ Fs / (2π × 2^alpha_shift) ≈ 0.15 Hz at 30.72 MSPS — effectively DC-only.

**CSRs:**
- `_enable`      — CSRStorage(1, reset=0)
- `_alpha_shift` — CSRStorage(5, reset=15) — configurable filter corner

**Integration point:** same as IQ correction — between loopback RX source and header RX sink,
applied after IQ correction.

**Status:** `[ ]`

---

#### GW-11 · Wire CFR/IQ/DC into `litex_m2sdr.py`

**Goal:** Add optional DSP pipeline stages with feature flags.

**Changes to `litex_m2sdr.py`:**

1. Add imports:
   ```python
   from litex_m2sdr.gateware.cfr          import CrestFactorReduction
   from litex_m2sdr.gateware.iq_correction import IQCorrection
   from litex_m2sdr.gateware.dc_filter     import DCFilter
   ```

2. Add args to `BaseSoC.__init__`:
   `with_cfr=False, with_iq_correction=False, with_dc_filter=False`

3. TX path — after `self.timed_tx`, before loopback:
   ```python
   tx_source = self.timed_tx.source
   if with_cfr:
       self.cfr = CrestFactorReduction(data_width=64)
       self.comb += tx_source.connect(self.cfr.sink)
       tx_source = self.cfr.source
   self.comb += tx_source.connect(self.txrx_loopback.tx_sink)
   ```

4. RX path — after loopback, before header:
   ```python
   rx_source = self.txrx_loopback.rx_source
   if with_iq_correction:
       self.iq_correction = IQCorrection(data_width=64)
       self.comb += rx_source.connect(self.iq_correction.sink)
       rx_source = self.iq_correction.source
   if with_dc_filter:
       self.dc_filter = DCFilter(data_width=64)
       self.comb += rx_source.connect(self.dc_filter.sink)
       rx_source = self.dc_filter.source
   self.comb += rx_source.connect(self.header.rx.sink)
   ```
   *(Replace the existing direct `self.txrx_loopback.rx_source.connect(self.header.rx.sink)` at line 629.)*

5. Add CLI flags: `--with-cfr`, `--with-iq-correction`, `--with-dc-filter`

**Status:** `[ ]`

---

#### SW-6 · CFR and IQ correction kwargs in `soapysdr/LiteXM2SDRDevice.cpp`

**Goal:** Expose CFR threshold and IQ correction coefficients as SoapySDR settings.

**Changes to `LiteXM2SDRDevice.cpp`:**

```cpp
// In writeSetting():
if (key == "cfr_threshold")
    litepcie_writel(_fd, CSR_CFR_THRESHOLD_ADDR, (uint32_t)(std::stof(value) * 32767));
if (key == "cfr_enable")
    litepcie_writel(_fd, CSR_CFR_ENABLE_ADDR, std::stoi(value));
if (key == "iq_correction_enable")
    litepcie_writel(_fd, CSR_IQ_CORRECTION_ENABLE_ADDR, std::stoi(value));
// IQ coefficients exposed as "iq_a_coeff_a" etc.
```

**Status:** `[ ]`

---

#### SW-7 · Offline IQ calibration utility

**Goal:** Measure IQ imbalance via FPGA loopback and compute correction matrix coefficients.

**New file:** `software/user/m2sdr_iq_cal.py` (Python script)

**Procedure:**
1. Enable FPGA loopback (`TXRXLoopback` CSR)
2. Transmit a known single-tone (use `tone_gen.py`)
3. Capture RX samples (`m2sdr_record`)
4. Compute IQ amplitude ratio and phase offset from captured data
5. Derive 2×2 correction matrix coefficients
6. Write coefficients to `CSR_IQ_CORRECTION_*` via `litepcie_writel`

**Status:** `[ ]`

---

#### Ph3-commit · Commit Phase 3

```bash
git add litex_m2sdr/gateware/cfr.py \
        litex_m2sdr/gateware/iq_correction.py \
        litex_m2sdr/gateware/dc_filter.py \
        litex_m2sdr.py \
        litex_m2sdr/software/soapysdr/LiteXM2SDRDevice.cpp \
        litex_m2sdr/software/user/m2sdr_iq_cal.py
git commit -m "nr: phase 3 — CFR, IQ correction, DC filter DSP blocks"
```

**Status:** `[ ]`

---

### Summary Checklist

```
Phase 1 — Minimum Viable 5G NR
  [x] GW-1  gateware/timed_tx.py — TimedTXArbiter
  [x] GW-2  gateware/header.py   — burst flags in TX sync word
  [x] GW-3  litex_m2sdr.py       — wire TimedTXArbiter
  [x] SW-1  LiteXM2SDRStreaming.cpp — real TX timestamp + flags
  [ ] SW-2  LiteXM2SDRDevice.cpp   — slot-aligned frame_cycles
  [x] SW-3  LiteXM2SDRStreaming.cpp — readStreamStatus() late/underrun
  [x] Ph1-commit

Phase 2 — TDD Clean Switching
  [x] GW-4  gateware/tdd_switch.py      — TDDSwitch module
  [x] GW-5  gateware/ad9361/core.py     — txnrx override input
  [x] GW-6  litex_m2sdr.py              — wire TDDSwitch + with_tdd flag
  [x] GW-7  gateware/header.py          — RX mixed timestamp mode
  [x] SW-4  LiteXM2SDRDevice.cpp        — tdd_enable/lead/trail kwargs
  [x] SW-5  software/user/m2sdr_rf.c    — -tdd CLI options
  [x] Ph2-commit

Phase 3 — DSP Blocks
  [x] GW-8  gateware/cfr.py             — CrestFactorReduction
  [x] GW-9  gateware/iq_correction.py   — IQCorrection 2×2 matrix
  [x] GW-10 gateware/dc_filter.py       — DCFilter IIR
  [x] GW-11 litex_m2sdr.py              — wire DSP blocks + CLI flags
  [x] SW-6  LiteXM2SDRDevice.cpp        — CFR/IQ SoapySDR settings
  [ ] SW-7  software/user/m2sdr_iq_cal.py — offline IQ calibration
  [ ] Ph3-commit
```
