# OCUDU + M2SDR 3.5 GHz bladeRF PSS baseline

## Validated result

OCUDU `radio_ssb` through Soapy LiteX-M2SDR produces externally captured NR PSS on bladeRF.

Validated profile:

- `radio_ssb -P m2sdr_35g_10MHz`
- DL/RF center: 3500.000 MHz
- SSB center: 3497.150 MHz
- PCI: 1
- Expected PSS identity: `NID2 = PCI mod 3 = 1`
- Sample rate: 11.52 Msps on M2SDR
- bladeRF capture used 5.76 Msps because sensnuc7 enumerated bladeRF as USB Hi-Speed.

## Required safe runtime environment

Do not use Soapy copy-mode on nuc4.

Required:

```bash
unset M2SDR_LITEPCIE_ZERO_COPY
export M2SDR_DIRECT_DEINTERLEAVE=1
export M2SDR_SOAPY_TX_TIME_OFFSET_NS=20000000
export M2SDR_SOAPY_TX_DEDUP_TIME=1
export M2SDR_SOAPY_TX_IDLE_FILL=0
export M2SDR_SOAPY_TX_CS16_TO_SC12=1
```

Rationale:

- Without the +20 ms TX timing offset, `radio_ssb` produced TX late notifications.
- With `M2SDR_SOAPY_TX_TIME_OFFSET_NS=20000000`, `radio_ssb` produced zero TX late/underflow events.
- LitePCIe copy-mode is disabled for nuc4 validation because it hung the Soapy path and held `/dev/m2sdr0`.

## Captures used

Noise baseline:

- bladeRF center: 3497.150 MHz
- sample rate: 5.76 Msps
- duration: 1 s
- rms around -37.46 dBFS

Native M2SDR marker:

- M2SDR LO: 3497.150 MHz
- tone: +500 kHz
- expected RF: 3497.650 MHz
- bladeRF measured peak around +506.8 to +507.0 kHz
- peak over median around 86-88 dB

OCUDU SSB mid-center capture:

- bladeRF center: 3498.575 MHz
- sample rate: 5.76 Msps
- duration: 2 s
- SSB expected offset: -1.425 MHz
- RF center expected offset: +1.425 MHz
- repeated captures showed strong RF energy near 3500.005 MHz.

Offline PSS check:

```bash
python tools/fast_nr_pss_check.py \
  radio_ssb_fc3498575000_fs5760000.sc16 \
  --fs 5760000 \
  --fc 3498575000 \
  --ssb 3497150000 \
  --expected-nid2 1 \
  --cfo-min -30000 \
  --cfo-max 30000 \
  --cfo-step 1000
```

Representative result:

```text
NID2=0: score=0.278193
NID2=1: score=0.977392
NID2=2: score=0.277212
expected_vs_best_other_ratio=3.513
VERDICT=PSS_EXPECTED_NID2_DOMINATES
```

## Open bug

`radio_ssb` still triggers kernel warnings on teardown:

```text
BUG: Bad page map in process radio_ssb
file:m2sdr0 fault:0x0 mmap:litepcie_mmap [m2sdr]
```

This appears after successful TX, not during TX. Treat it as a kernel/driver mmap teardown bug. Do not hide it in validation output.

## Do not do remotely

Do not unload/reload/reinsert the M2SDR kernel module remotely on nuc4 as a debugging shortcut. That previously wedged nuc4/Tailscale.
