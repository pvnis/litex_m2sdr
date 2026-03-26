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

These tests verify that the data received matches what was transmitted, using a PN (pseudo-noise) sequence with hardware error counting.

> **Note:** `dma_test` only works on bit-exact paths. The AD9361 is **not** a transparent byte pipe — even in internal loopback mode, TX data passes through the AD9361 digital baseband filters (interpolation, decimation), which transform the raw bit pattern. Use `dma_test` for PCIe DMA integrity only; use the tone/PRBS methods below for the AD9361 signal path.

### FPGA DMA Loopback (no AD9361)

The FPGA `TXRXLoopback` block routes the TX DMA stream directly into the RX DMA stream, bypassing the AD9361 entirely. This verifies PCIe DMA integrity end-to-end.

```bash
./m2sdr_util dma_test
```

With zero-copy DMA and automatic RX delay calibration:

```bash
./m2sdr_util -z -a dma_test
```

### DMA Test Output

```
[> DMA loopback test:
---------------------
RX_DELAY: 42 (errors: 0, confirmations: 3)
Speed: 14523.4 MB/s  Errors: 0
Speed: 14521.8 MB/s  Errors: 0
...
```

A non-zero error count after the RX delay is locked indicates a PCIe DMA integrity problem.

### DMA Test Options

| Flag | What it does |
|------|-------------|
| `-z` | Zero-copy DMA mode (higher throughput) |
| `-a` | Automatic RX delay calibration |
| `-w N` | Data bus width for PN mask (default: 32) |
| `-W N` | Warmup buffers before checking starts (default: 2048×buffer count) |
| `-d N` | Test duration in seconds (0 = run until Ctrl+C) |

---

## AD9361 Signal Path Verification

The AD9361 processes IQ data (filtering, gain, decimation/interpolation), so raw PN patterns do not survive the loopback. Use these methods instead.

### Tone Loopback Test

Enable AD9361 internal loopback, inject a tone on TX, capture on RX, and verify the tone frequency in the captured file.

```bash
# Terminal 1: configure RF with AD9361 internal loopback
./m2sdr_rf -samplerate=30720000 -bandwidth=56000000 \
           -tx_freq=2400000000 -rx_freq=2400000000 \
           -loopback=1

# Terminal 2: transmit a 1 MHz tone
./m2sdr_gen -s 30720000 -t tone -f 1e6 -a 0.5

# Terminal 3: capture a few seconds of RX
./m2sdr_record -s 30720000 -c 2 -n 30720000 loopback_rx.bin
```

Then verify the tone in Python:

```python
import numpy as np
data = np.fromfile("loopback_rx.bin", dtype=np.int16)
iq = data[0::2].astype(np.float32) + 1j * data[1::2].astype(np.float32)
spectrum = np.abs(np.fft.fftshift(np.fft.fft(iq[:4096])))
freq = np.fft.fftshift(np.fft.fftfreq(4096, d=1/30.72e6))
peak = freq[np.argmax(spectrum)]
print(f"Peak at {peak/1e6:.3f} MHz (expected 1.000 MHz)")
```

### BIST PRBS Interface Calibration

For verifying the FPGA ↔ AD9361 LVDS data interface (not the RF path), use the built-in PRBS scan (see BIST section above).

### Test Layer Summary

| Layer | Tool | What it verifies |
|-------|------|-----------------|
| PCIe DMA | `dma_test` | Host ↔ FPGA DMA bit-exact integrity |
| FPGA ↔ AD9361 LVDS | `m2sdr_rf -bist_prbs` | Clock/data delay calibration |
| AD9361 TX→RX signal | tone loopback + Python | Signal fidelity through AD9361 baseband |
| Full RF chain | cable loopback + tone | End-to-end TX→antenna→RX path |

---

## Timed TX / TDD Validation Sequence

This sequence is intended for validating the timed-TX path after gateware changes, with the specific goal of supporting `ocudu-pavo` in TDD mode through the Soapy driver.

The key failure mode we are trying to catch is:

- host TX ring backs up around the scheduled TDD bursts
- `acquireWriteBuffer()` starts timing out
- Timed TX reports `LATE` or `UNDERRUN`
- PCIe stays up, but future-timestamped TX does not retire buffers correctly

### Pre-checks

Confirm the board is alive after reload:

```bash
m2sdr_util info
```

Record at least:

- `FPGA Status`
- `PCIe Speed`
- `PCIe Lanes`
- `PCIe PTM`
- `DSP Features / Timed TX`
- `Board Time`

Expected:

- `FPGA Status      : Operational`
- `PCIe Speed       : Gen2`
- `PCIe Lanes       : x1`
- `PCIe PTM         : Enabled`
- `Timed TX         : Yes`

### 1. Baseline untimed TX sanity check

This verifies that plain continuous TX still works before any timed scheduling is involved.

```bash
cd /home/dmd/m2sdr/litex_m2sdr/software/user
./m2sdr_streaming_test --tx --samplerate 23040000
```

Watch for:

- final `PASS`
- no repeated TX errors
- no signs of stuck DMA or PCIe stalls

If this fails, stop. The problem is below timed-TX/TDD.

### 2. Timed TX pulse sanity check

This verifies future-timestamped bursts without the full TDD duty-cycle pattern.

```bash
./m2sdr_streaming_test --timed --samplerate 23040000 --delay 0.25 --duration 0.02 --gap 0.10 --num-pulses 5
```

Record these lines from the output:

- `=== Phase 4: Timed TX`
- `RX captured: ...`
- `TX status  : X late event(s)  Y underrun(s)`
- `Pulse N/N : onset ... raw ... corrected ... delta ...`
- `Timing stats : mean ... min ... max ... std ...`
- final report lines:
  - `Timed TX: no TX errors`
  - `Timed TX: all pulses detected`
  - `Timed TX: no late events`

Pass criteria:

- `TX status  : 0 late event(s)  0 underrun(s)`
- all per-pulse detections present
- final `PASS`

### 3. TDD emulation, padded UL

This is the main hardware test for the current gateware. It keeps the DMA reader fed across UL with zeros and is the first test that should pass before trying sparse UL.

```bash
./m2sdr_streaming_test --tdd \
  --tdd-srate 23040000 \
  --tdd-frames 100 \
  --tdd-lead-ms 1.0 \
  --tdd-dl-slots 6 \
  --tdd-dl-symbols 8 \
  --tdd-ul-slots 3 \
  --tdd-awb-timeout 200 \
  --tdd-init-ms 20 \
  --tdd-pad-ul
```

This matches the intended `m2sdr.yml` timing shape:

- `23.04 MSPS`
- `6 DL slots + 8 DL symbols + 3 UL slots`
- `10 ms` frame

Record these lines:

- `=== Phase 5: TDD Emulation`
- `TX ring    : ...`
- `DL burst   : ...`
- `UL silence : ...`
- `Guard zeros: ...`
- `Host queue : ...`
- `UL mode    : zero-padded (continuous DMA feed)`
- frame progress lines:
  - `[frame NNN] hw=... slept=... awb_to=... late=...`
- timeout/error lines if any:
  - `[frame NNN] acquireWriteBuffer TIMEOUT (guard)`
  - `[frame NNN] acquireWriteBuffer error ...`
  - `[frame NNN] acquireWriteBuffer TIMEOUT (burst B/K)`
  - `[frame NNN] acquireWriteBuffer error ... (burst ...)`
- async status lines:
  - `[status] LATE at ...`
  - `[status] UNDERRUN at ...`
- final summary:
  - `Frames OK    : ...`
  - `AWB timeouts : ...`
  - `AWB errors   : ...`
  - `Late events  : ...`
  - `Underruns    : ...`
  - `TDD: no AWB timeouts`
  - `TDD: no AWB errors`
  - `TDD: no late events`
  - `TDD: no underruns`

Pass criteria:

- `AWB timeouts : 0`
- `AWB errors   : 0`
- `Late events  : 0`
- `Underruns    : 0`
- final `PASS`

Most important counters:

- `AWB timeouts`
  - primary indicator that the host TX ring is backing up
- `Late events`
  - direct indicator that Timed TX missed a scheduled timestamp
- `Underruns`
  - indicator that the TX data path starved after the burst stream was armed

### 4. TDD emulation, sparse UL

This is the closest approximation to the discontinuous gNB behavior and is the next test after padded UL passes.

```bash
./m2sdr_streaming_test --tdd \
  --tdd-srate 23040000 \
  --tdd-frames 100 \
  --tdd-lead-ms 1.0 \
  --tdd-dl-slots 6 \
  --tdd-dl-symbols 8 \
  --tdd-ul-slots 3 \
  --tdd-awb-timeout 200 \
  --tdd-init-ms 20 \
  --tdd-sparse
```

Record the same counters/log lines as the padded-UL case.

Expected interpretation:

- if padded UL passes and sparse UL fails, the remaining issue is likely gap-handling semantics or sparse submission behavior
- if both fail in the same way, the remaining issue is likely still in timed scheduling / buffer retirement

### 5. Arbiter bypass control test

Use this only as a diagnostic, not as a target mode.

If the binary/build supports arbiter bypass, rerun timed or TDD tests with the arbiter bypassed and compare behavior.

What to compare:

- whether `AWB timeouts` disappear
- whether `Late events` disappear
- whether TX starts behaving like immediate untimed release

Interpretation:

- bypass passes but normal timed mode fails:
  - failure is in timed scheduling / TimedTXArbiter path
- bypass also fails:
  - problem is lower level, likely DMA / stream plumbing / driver interaction

### Suggested capture format

For each run, save:

- full command line
- git revision or bitstream build timestamp
- whether the bitstream is `current` or `m2sdr_previous`
- the final summary block
- the first timeout or status event line, if any

Recommended shell capture:

```bash
mkdir -p /tmp/m2sdr_test_logs
./m2sdr_streaming_test ... 2>&1 | tee /tmp/m2sdr_test_logs/tdd_padded_$(date +%Y%m%d_%H%M%S).log
```

### Decision tree

- untimed TX fails:
  - investigate PCIe/DMA/base TX path first
- timed pulses fail:
  - investigate future-timestamped TX before TDD
- timed pulses pass, padded TDD fails:
  - investigate sustained timed scheduling under TDD load
- padded TDD passes, sparse TDD fails:
  - investigate discontinuous submission semantics relevant to `ocudu-pavo`
