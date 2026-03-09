# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**LiteX-M2SDR** is an open-source FPGA-based M.2 2280 SDR board featuring a Xilinx Artix-7 XC7A200T FPGA and an ADI AD9361 RFIC. It uses the [LiteX](https://github.com/enjoy-digital/litex) HDL framework and connects to the host over PCIe Gen2 x4 (up to ~14 Gbps). The repo contains both the gateware (HDL) and the full Linux software stack.

## Build Commands

### Gateware (FPGA Bitstream)

Requires LiteX installed (`python3 litex_setup.py --config=standard --init --install --user`).

```bash
# Build PCIe variant (M.2 slot)
./litex_m2sdr.py --with-pcie --variant=m2 --build --load

# Build PCIe variant (Acorn baseboard)
./litex_m2sdr.py --with-pcie --variant=baseboard --build --load

# Build Ethernet variant
./litex_m2sdr.py --with-eth --eth-sfp=0 --build --load

# Build with White Rabbit (baseboard only)
./litex_m2sdr.py --with-pcie --with-white-rabbit --variant=baseboard --build
```

### Software (all-in-one)

```bash
cd litex_m2sdr/software
./build.py                        # Build kernel driver + user utilities + SoapySDR driver
sudo ./build.py                   # Also installs kernel module and SoapySDR driver
./build.py --no-install           # Build only, skip install
./build.py --interface=liteeth    # Build for Ethernet (Etherbone) interface
```

### Kernel Driver

```bash
cd litex_m2sdr/software/kernel
make clean all
sudo make install
sudo insmod m2sdr.ko              # Load without reboot
sudo make uninstall
```

### User Utilities

```bash
cd litex_m2sdr/software/user
make clean all
# Or with explicit interface:
make clean INTERFACE=USE_LITEPCIE all
make clean INTERFACE=USE_LITEETH all
```

### SoapySDR Driver

```bash
cd litex_m2sdr/software/soapysdr
cmake -S . -B build
cmake --build build -j$(nproc)
```

### Tests

```bash
# Gateware simulation tests (CI-safe, no hardware needed)
python3 -m pytest -v test

# Run a single test file
python3 -m pytest -v test/test_ad9361.py

# Scripts in scripts/ require a running board/server
python3 scripts/test_xadc.py
python3 scripts/test_dashboard.py
```

## Architecture

### Gateware (`litex_m2sdr/gateware/`)

All gateware is written in Migen/LiteX Python. The top-level SoC is built in `litex_m2sdr.py`.

Key gateware modules:
- **`ad9361/`** — AD9361 RFIC subsystem: PHY (`phy.py` handles DDR LVDS I/O), SPI master, bit-mode conversion (8-bit/12-bit), PRBS test patterns, AGC saturation counter. `core.py` assembles these into `AD9361RFIC` with full TX/RX pipeline.
- **`si5351.py`** — SI5351 clock generator (I2C) for generating the AD9361 reference clock from an internal XO or external 10 MHz input.
- **`pcie.py`** — `PCIeLinkResetWorkaround`: periodically toggles PCIe reset until link-up, used at startup.
- **`qpll.py`** — Shared QPLL for GTP transceivers (used by PCIe/Ethernet/SATA).
- **`time.py`** — Hardware time generator (ns/ps resolution) for timestamping.
- **`pps.py`** — PPS (pulse-per-second) generator.
- **`vrt.py`** — VITA Radio Transport (VRT) packet streamer for Ethernet RX.
- **`rfic.py`** — `RFICDataPacketizer`: frames RFIC data words into packets for DMA.
- **`header.py`** — TX/RX header insertion/extraction.
- **`loopback.py`** — TX→RX loopback crossbar.
- **`gpio.py`** — 4-bit GPIO with CSR and DMA modes.
- **`capability.py`** — Capability register (exposes board features to software).
- **`measurement.py`** — Multi-clock frequency measurement core.

### Software Stack

```
litex_m2sdr/software/
├── kernel/              # Linux PCIe kernel driver (m2sdr.ko)
│   └── main.c           # PCI probe/remove, BAR0 MMAP, DMA, interrupts, /dev/m2sdrX
├── user/                # User-space C utilities
│   ├── ad9361/          # AD9361 driver (from Analog Devices, adapted)
│   ├── liblitepcie/     # LitePCIe DMA library (litepcie_dma.c)
│   ├── libliteeth/      # Etherbone/UDP library
│   ├── libm2sdr/        # Board-level abstraction: SPI (AD9361), I2C (SI5351), flash
│   ├── m2sdr_util.c     # Board info, DMA test, flash ops, clock test
│   ├── m2sdr_rf.c       # RF frontend init/config (sample rate, freq, gain)
│   ├── m2sdr_gen.c      # Real-time tone/noise/PRBS TX generator
│   ├── m2sdr_play.c     # File playback over DMA TX
│   ├── m2sdr_record.c   # IQ capture over DMA RX
│   ├── m2sdr_scan.c     # Wideband scanner (SDL2/OpenGL/ImGui UI)
│   ├── m2sdr_fm_tx.c    # FM modulator (WAV → IQ)
│   ├── m2sdr_fm_rx.c    # FM demodulator (IQ → WAV)
│   ├── m2sdr_sata.c     # SATA streamer control
│   └── m2sdr_gpio.c     # GPIO control
└── soapysdr/            # SoapySDR C++ plugin
    ├── LiteXM2SDRDevice.cpp      # Sample rate/freq/gain, device control
    ├── LiteXM2SDRStreaming.cpp   # activateStream/readStream/writeStream via DMA
    └── LiteXM2SDRRegistration.cpp # Plugin registration/enumeration
```

### Data Flow (PCIe path)

AD9361 LVDS I/O → DDR PHY → 2:1 deserialize → CDC (RFIC clk → sys clk) → bit-mode conversion → `RFICDataPacketizer` → LitePCIe DMA → `/dev/m2sdrX` → user-space

TX is the reverse. The kernel driver manages DMA buffers and interrupts; user-space tools use `liblitepcie` for zero-copy DMA.

### CSR/Register Access

The kernel module maps BAR0 and exposes MMAP + ioctl to user space. `csr.h`, `soc.h`, `config.h`, and `mem.h` in `software/kernel/` are auto-generated by `litex_m2sdr.py` during gateware build. After a gateware rebuild, regenerate them via the software build.

### Key CLI Patterns

```bash
# Board info
./m2sdr_util info

# RF init
./m2sdr_rf -samplerate=30720000 -bandwidth=56000000 -tx_freq=2400000000 -rx_freq=2400000000

# External 10 MHz reference clock
./m2sdr_rf -sync=external -refclk_freq=38400000

# Real-time DMA TX
./m2sdr_gen -s 30720000 -t tone -f 1e6 -a 0.5

# SoapySDR probe
SoapySDRUtil --probe="driver=LiteXM2SDR"

# Enable kernel debug logging
sudo sh -c "echo 'module m2sdr +p' > /sys/kernel/debug/dynamic_debug/control"

# Flash bitstream over PCIe
cd litex_m2sdr/software && ./flash.py ../build/litex_m2sdr_platform/litex_m2sdr/gateware/litex_m2sdr_platform.bin

# LiteX debug bridge (choose JTAG or PCIe)
litex_server --jtag --jtag-config=openocd_xc7_ft2232.cfg
sudo litex_server --pcie --pcie-bar=04:00.0
```

## Important Notes

- **`test/`** — simulation/unit tests, CI-safe (no hardware). **`scripts/`** — board control scripts requiring live hardware.
- **`m2sdr_scan`** requires SDL2/OpenGL (`libsdl2-dev libgl1-mesa-dev`); built conditionally by Makefile.
- **IOMMU**: For PCIe DMA streaming, set `iommu=pt` in kernel cmdline. Without this, IQ data will not flow.
- **Oversampling at 122.88 MSPS** requires PCIe Gen2 x2/x4 (`--pcie-lanes=2|4`).
- **White Rabbit** is baseboard-only. If a stale local `wr-cores/` checkout exists, remove it before building.
- The `--variant=` flag (`m2` vs `baseboard`) selects whether Ethernet/SATA/WR features are available.
- `csr.h` / `soc.h` / `config.h` / `mem.h` in `software/kernel/` are generated artifacts — do not edit manually.
