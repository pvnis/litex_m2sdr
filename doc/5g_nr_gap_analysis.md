# M2SDR vs LimeSDR_GW: 5G NR Gap Analysis

## What m2sdr Has (foundations are solid)

| Feature | m2sdr Status |
|---|---|
| 64-bit hardware nanosecond time counter | ✅ `TimeGenerator` in `time.py` |
| RX timestamping (every frame) | ✅ `HeaderInserterExtractor` injects hw time |
| TX timestamp extraction (CSR-readable) | ✅ Extracted from DMA header word 1 |
| PPS generation / PPS sync | ✅ `PPSGenerator` in `pps.py` |
| White Rabbit sub-µs sync (optional) | ✅ baseboard variant |
| Loopback mode | ✅ `TXRXLoopback` |
| 8/16-bit mode, 1R1T/2R2T | ✅ |
| PCIe DMA with frame boundaries | ✅ |
| PRBS TX/RX for calibration | ✅ |

---

## The Core Gap: Timed TX Is Missing

**LimeSDR_GW architecture:**
```
Host submits: [ts=307200, samples...]  [ts=322560, samples...]
                      ↓
             TX timestamp FIFO
                      ↓
          Comparator: sample_counter >= burst_ts?
                      ↓ YES
             Samples → DAC → RF
```

**m2sdr architecture today:**
```
Host submits: [ts=307200, samples...]
                      ↓
          Header extractor (reads ts into CSR, discards it)
                      ↓
             Samples → DAC → RF  ← immediately, ignoring timestamp
```

The m2sdr extracts the TX timestamp and makes it readable via CSR — but **never acts on it**. There is no comparator, no scheduling FIFO, no hold-until-right-time mechanism. TX is purely flow-through.

---

## Specific Missing Features for 5G NR

### 1. Timed TX Arbiter (most critical)

The FPGA needs to hold TX samples in a buffer and release them only when `time_counter >= burst_timestamp`. Without this, srsRAN/OAI can't schedule slot-aligned transmissions — they'd have to account for all pipeline latency manually and even then have no guarantee of sample-accurate delivery.

**What needs building:** A TX scheduling module:
```
TX header timestamp → comparator vs TimeGenerator.time
                           ↓ time reached
              Release samples to AD9361 PHY
```

### 2. Late Packet Detection & Underrun Flags

When the host misses a deadline (USB jitter, kernel scheduling), the FPGA needs to:
- Detect `time_counter > burst_timestamp` (packet arrived late)
- Drop the late burst and set a flag/counter readable by software
- Continue outputting silence rather than transmitting at the wrong slot

Without this, a late packet gets transmitted at the wrong timeslot, corrupting the NR frame structure for all UEs.

### 3. Automatic Silence/Zero Insertion Between Bursts

5G NR TDD alternates TX and RX every slot. Between TX bursts, the FPGA must output `I=0, Q=0` at full rate — not stall or leave the bus undefined. m2sdr's PHY does default to 0 when `valid=0`, but there's no burst-aware sequencer ensuring this happens cleanly at burst boundaries. The current design relies entirely on the host software sending silence samples to fill gaps.

### 4. TX Burst Flags (TX_START / TX_END)

LimeSDR packet headers carry `TX_START` and `TX_END` flags to mark burst boundaries explicitly. m2sdr has a sync word and timestamp in the header but no burst boundary flags. Without these, the FPGA can't distinguish "this is the last packet of slot N's burst" from "keep transmitting" — which is needed to cleanly gate silence insertion.

SoapySDR has `SOAPY_SDR_END_BURST` but the m2sdr gateware currently ignores it.

### 5. TDD Switch / PA_EN Gating

In TDD NR, the RF switch and PA must be gated: PA_EN goes high some microseconds *before* the TX burst starts (to let the PA settle), and goes low at TX_END before the RX window opens. LimeSDR_GW generates `PA_EN` as a sample-counter-gated signal with a programmable lead time.

m2sdr has GPIO outputs but no hardware mechanism to automatically assert a `TDD_SW`/`PA_EN` signal tied to burst start/end timestamps. This means the host would have to bit-bang GPIO timing, which is jittery.

### 6. TX Underrun Handling

When the TX FIFO runs dry mid-burst (host didn't supply samples in time), the correct behavior is:
- Output zeros (silence) for the remainder of the burst
- Flag an underrun counter in a CSR

Currently m2sdr's PHY outputs 0 when `valid=0`, so the silence is there, but there's no underrun detection or status feedback to the host.

---

## Summary Table

| Feature | LimeSDR_GW | m2sdr today | Gap |
|---|---|---|---|
| Free-running sample counter | ✅ | ✅ (ns-based TimeGenerator) | None |
| RX timestamp injection | ✅ | ✅ | None |
| TX timestamp parsing | ✅ | ✅ (CSR readable) | None |
| **Timed TX arbiter** | ✅ | ❌ | **Critical** |
| **Late packet detection** | ✅ | ❌ | **Critical** |
| **TX burst flags (START/END)** | ✅ | ❌ | **Critical** |
| Silence insertion between bursts | ✅ hardware | ⚠️ software/implicit | Needed |
| Underrun detection & flagging | ✅ | ❌ | Important |
| TDD PA_EN/TDD_SW gating | ✅ | ❌ | Important |
| PPS sync | ✅ (optional) | ✅ | None |
| Multi-device MIMO sync | ✅ SYNC pin | ⚠️ WR (baseboard) | Partial |

---

## Implementation Roadmap

The minimum viable additions for 5G NR operation, in priority order:

1. **TX scheduling FIFO + timestamp comparator** — a new gateware module between the header extractor and the AD9361 sink. The extracted timestamp triggers sample release when `TimeGenerator.time >= tx_timestamp`. This is the single most impactful change.

2. **TX burst state machine** — tracks START/END flags, generates silence between bursts, detects late packets, exposes underrun/late counters via CSR.

3. **TDD switch output** — a CSR-configured lead/lag offset applied to burst boundaries, driving a GPIO pin as `PA_EN`.

4. **SoapySDR streaming updates** — pass `HAS_TIME` flag down to the gateware header, communicate `END_BURST`, read late/underrun counters and surface them as async events.

The TimeGenerator, PPS infrastructure, and header mechanism are exactly the right foundations for building all of this. The m2sdr already has everything needed except the actual "hold TX until timestamp fires" comparator logic.

---

## Background: LimeSDR_GW Reference Architecture

### TX Path
- Host sends 4096-byte packets with a 16-byte header: 64-bit timestamp (sample count), TX_START/TX_END flags, payload length.
- Packets queue in a deep BRAM FIFO inside the FPGA.
- A comparator checks `sample_counter >= burst_timestamp` every clock cycle; when true, samples are released to the DAC.
- Between bursts: `I=0, Q=0` is driven at full rate (silence, not bus-idle).
- Late packets (`sample_counter > burst_ts`): dropped, `tx_late_counter` CSR incremented.
- Underrun (FIFO empty at fire time): silence output, underrun flag set.

### RX Path
- Continuous capture at full ADC rate.
- Sample counter value latched at FIFO entry time, attached to packet header.
- Fixed number of IQ samples per packet (e.g., 1020 complex), then bulk transfer to host.

### 5G NR Timing Model
With 30.72 MHz sample clock (NR numerology μ=1, 20 MHz BW):

| Boundary | Sample interval |
|---|---|
| OFDM symbol | ~2048 + CP samples |
| Slot (0.5 ms) | 15 360 samples |
| Subframe (1 ms) | 30 720 samples |
| Radio frame (10 ms) | 307 200 samples |

The host stack (srsRAN, OAI) computes the absolute sample number for each TX slot start and places it in the packet timestamp field. The FPGA fires at that exact sample count. The FPGA itself has no NR-specific framing logic — it executes timed bursts at host-specified sample boundaries.
