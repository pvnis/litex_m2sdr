# M2SDR Testing Guide

## AD9361 BIST / Loopback Tests

All tests are run via `m2sdr_rf` after building the software stack (`cd litex_m2sdr/software && ./build.py`).

### 1. Internal AD9361 TX→RX Loopback (mode 1)

Routes the TX digital datapath internally back to RX inside the AD9361 — no RF path involved. Good for verifying the FPGA↔AD9361 data interface.

```bash
./m2sdr_rf -samplerate=30720000 -bandwidth=56000000 \
           -tx_freq=2400000000 -rx_freq=2400000000 \
           -loopback=1
```

Then transmit a tone and receive it:

```bash
# Terminal 1: generate a tone
./m2sdr_gen -s 30720000 -t tone -f 1e6 -a 0.5

# Terminal 2: capture RX
./m2sdr_record -s 30720000 -c 2 out.bin
```

### 2. FPGA RX→TX Loopback (mode 2)

Routes the RX data from the AD9361 back out as TX (HDL loopback). Useful for relay/repeater testing.

```bash
./m2sdr_rf -samplerate=30720000 -bandwidth=56000000 \
           -tx_freq=2400000000 -rx_freq=2400000000 \
           -loopback=2
```

### 3. BIST TX Tone Injection

Injects an internal tone on the TX side, bypassing the DMA. Lets you verify the RF TX chain without any host streaming.

```bash
./m2sdr_rf -samplerate=30720000 -bandwidth=56000000 \
           -tx_freq=2400000000 -rx_freq=2400000000 \
           -bist_tx_tone -bist_tone_freq=1000000
```

### 4. BIST RX Tone Injection

Injects a tone on the RX digital path. Lets you test RX software processing without any over-the-air signal.

```bash
./m2sdr_rf -samplerate=30720000 -bandwidth=56000000 \
           -tx_freq=2400000000 -rx_freq=2400000000 \
           -bist_rx_tone -bist_tone_freq=1000000
```

### 5. BIST PRBS Test (Interface Calibration)

The most thorough interface test. Scans all FPGA↔AD9361 clk/data delay combinations to find valid eye openings. Useful after board bring-up, rework, or signal integrity debugging. Auto-applies the optimal RX and TX clk+data delays.

```bash
./m2sdr_rf -samplerate=30720000 -bandwidth=56000000 \
           -tx_freq=2400000000 -rx_freq=2400000000 \
           -bist_prbs
```

Prints a 16×16 delay matrix for both RX and TX and programs the optimal delays into the AD9361.

---

### Mode Summary

| Flag | What it does |
|------|-------------|
| `-loopback=0` | Normal operation (default) |
| `-loopback=1` | AD9361 internal TX→RX digital loopback |
| `-loopback=2` | FPGA HDL RX→TX loopback |
| `-bist_tx_tone` | Inject tone at TX digital input |
| `-bist_rx_tone` | Inject tone at RX digital output |
| `-bist_prbs` | PRBS delay scan — full FPGA↔AD9361 interface calibration |

The `-bist_tone_freq` argument (in Hz) sets the injected tone frequency for both `-bist_tx_tone` and `-bist_rx_tone`.

---

## DMA Data Integrity Tests

These tests verify that the data received matches what was transmitted, using a PN (pseudo-noise) sequence with hardware error counting. There are two levels depending on how much of the path you want to exercise.

### Level 1: FPGA DMA Loopback (no AD9361)

The FPGA `TXRXLoopback` block routes the TX DMA stream directly into the RX DMA stream, bypassing the AD9361 entirely. This verifies PCIe DMA integrity end-to-end.

```bash
./m2sdr_util dma_test
```

With zero-copy DMA and automatic RX delay calibration:

```bash
./m2sdr_util -z -a dma_test
```

### Level 2: Full Path Through AD9361

Combines the AD9361 internal digital loopback (`-loopback=1`) with the DMA test in external mode (`-e`). Data travels: host → PCIe DMA → FPGA → AD9361 TX datapath → AD9361 RX datapath → FPGA → PCIe DMA → host. The PN sequence is verified on the RX side with error counting.

```bash
# Step 1: configure RF with AD9361 internal loopback
./m2sdr_rf -samplerate=30720000 -bandwidth=56000000 \
           -tx_freq=2400000000 -rx_freq=2400000000 \
           -loopback=1

# Step 2: run DMA test in external mode (FPGA loopback disabled, data goes through AD9361)
./m2sdr_util dma_test -e
```

With zero-copy DMA and automatic RX delay calibration:

```bash
./m2sdr_util -z -a dma_test -e
```

The `-a` flag scans for the correct RX delay offset automatically — needed because the AD9361 introduces a fixed pipeline latency between TX and RX. Without it, the PN checker will start with the wrong seed and report false errors.

### DMA Test Output

The test prints throughput and error counts periodically:

```
[> DMA loopback test:
---------------------
RX_DELAY: 42 (errors: 0, confirmations: 3)
Speed: 14523.4 MB/s  Errors: 0
Speed: 14521.8 MB/s  Errors: 0
...
```

A non-zero error count after the RX delay is locked indicates a data integrity problem in the path.

### DMA Test Options

| Flag | What it does |
|------|-------------|
| (none) | FPGA internal loopback (bypasses AD9361) |
| `-e` | External mode — data flows through AD9361 (use with `m2sdr_rf -loopback=1`) |
| `-z` | Zero-copy DMA mode (higher throughput) |
| `-a` | Automatic RX delay calibration |
| `-w N` | Data bus width for PN mask (default: 32) |
| `-W N` | Warmup buffers before checking starts (default: 2048×buffer count) |
| `-d N` | Test duration in seconds (0 = run until Ctrl+C) |
