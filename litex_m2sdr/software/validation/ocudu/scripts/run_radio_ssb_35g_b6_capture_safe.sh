#!/usr/bin/env bash
# NOTE: LitePCIe copy-mode is disabled on nuc4; use env_m2sdr_ocudu_safe_rf.sh.

set +e
set +u
set +o pipefail

echo "===== refuse if stale test processes are running ====="
if pgrep -a radio_ssb; then
  echo "radio_ssb already running; stop it first."
  exit 2
fi
if pgrep -a bladeRF-cli; then
  echo "bladeRF-cli already running; stop it first."
  exit 2
fi

echo
echo "===== create bladeRF 3.5GHz raw capture helper ====="
cat > /tmp/capture_bladerf_raw_35g_b6.sh <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

OUT="${1:-/tmp/bladerf_35g_b6_raw.sc16}"
N="${2:-4608000}"
CENTER_HZ="${3:-3500000000}"
RX_GAIN="${4:-40}"

SCRIPT="$(mktemp --suffix=.bladerf)"
trap 'rm -f "$SCRIPT"' EXIT

cat > "$SCRIPT" <<EOS
set frequency rx $CENTER_HZ
set samplerate rx 11520000
set bandwidth rx 10000000
set agc rx off
set gain rx $RX_GAIN
rx config file=$OUT format=bin n=$N samples=16384 buffers=32 xfers=16 timeout=10s channel=1
rx start
rx wait 15s
EOS

echo "OUT=$OUT"
echo "N=$N"
echo "CENTER_HZ=$CENTER_HZ"
echo "RX_GAIN=$RX_GAIN"
sudo -n bladeRF-cli -s "$SCRIPT"
ls -lh "$OUT"
EOF
chmod +x /tmp/capture_bladerf_raw_35g_b6.sh

echo
echo "===== start clean 3.5GHz radio_ssb TX, backoff 6 dB ====="
cd /home/stefan/CLionProjects/m2sdr/litex_m2sdr/software/validation/ocudu || exit 1

source ./scripts/env_m2sdr_copy_dma.sh
source ./scripts/env_ocudu_validation.sh

set +e
set +u
set +o pipefail

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
BASE="$PWD/runs/radio_ssb_35g_b6_capture_noidle_${STAMP}"
mkdir -p "$BASE"

STDOUT="$BASE/radio_ssb_stdout.log"
RCFILE="$BASE/radio_ssb_rc.txt"
SUMMARY="$BASE/summary.log"

RADIO_SSB="$HOME/CLionProjects/ocudu/build-radio-ssb-m2sdr/apps/examples/phy/radio_ssb"

{
  echo "BASE=$BASE"
  echo "STDOUT=$STDOUT"
  echo "RADIO_SSB=$RADIO_SSB"
  echo
  echo "===== start radio_ssb 3.5GHz -b6 capture run ====="
} | tee "$SUMMARY"

sudo -n env \
  PATH="$PATH" \
  SOAPY_SDR_ROOT="${SOAPY_SDR_ROOT:-/tmp/empty-soapy-root}" \
  SOAPY_SDR_PLUGIN_PATH="${SOAPY_SDR_PLUGIN_PATH:-}" \
  LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}" \
  # copy-mode assignment disabled on nuc4 \
  M2SDR_DIRECT_DEINTERLEAVE=1 \
  M2SDR_SOAPY_TX_COPY_PRIME_BUFFERS=8 \
  M2SDR_SOAPY_TX_TIME_OFFSET_NS=0 \
  M2SDR_SOAPY_TX_IDLE_FILL=0 \
  M2SDR_SOAPY_TX_DEDUP_TIME=1 \
  M2SDR_SOAPY_TX_CS16_TO_SC12=1 \
  M2SDR_SOAPY_TX_RMS_LOG=0 \
  M2SDR_SOAPY_TX_RMS_LOG_LIMIT=0 \
  M2SDR_SOAPY_TX_DUMP_PATH= \
  M2SDR_SOAPY_TX_DUMP_SAMPLES=0 \
  M2SDR_SOAPY_TX_DMA_DUMP_PATH= \
  M2SDR_SOAPY_TX_DMA_DUMP_BUFFERS=0 \
  timeout 45 "$RADIO_SSB" \
    -P m2sdr_35g_10MHz \
    -D 30000 \
    -b 6 \
    -v info \
  > "$STDOUT" 2>&1 &
TX_PID=$!

echo "TX_PID=$TX_PID" | tee -a "$SUMMARY"

sleep 3

echo
echo "===== capture bladeRF raw IQ ====="
cd /home/stefan/CLionProjects || exit 1

IQ="/tmp/bladerf_ocudu_35g_b6_raw_${STAMP}.sc16"
/tmp/capture_bladerf_raw_35g_b6.sh "$IQ" 4608000 3500000000 40
CAP_RC=$?

echo "CAP_RC=$CAP_RC"
echo "IQ=$IQ"
ls -lh "$IQ"

echo
echo "===== wait for TX ====="
wait "$TX_PID"
TX_RC=$?
echo "radio_ssb_rc=$TX_RC" | tee "$RCFILE" | tee -a "$SUMMARY"

echo
echo "===== TX health ====="
cd /home/stefan/CLionProjects/m2sdr/litex_m2sdr/software/validation/ocudu || exit 1

SSB="$(grep -aci 'SSB: phys_cell_id' "$STDOUT" || true)"
LATE="$(grep -aci 'source=tx type=late' "$STDOUT" || true)"
UNDER="$(grep -aci 'underflow' "$STDOUT" || true)"
BAD="$(grep -aciE 'BUG:|Bad page|kernel BUG|oops|panic' "$STDOUT" || true)"
FINAL_TX="$(grep -ai '\[TX\]' "$STDOUT" | tail -1 || true)"
FINAL_RX="$(grep -ai '\[RX\]' "$STDOUT" | tail -1 || true)"

echo "SSB lines: $SSB"
echo "late notifications: $LATE"
echo "underflow mentions: $UNDER"
echo "kernel bad/bug/oops/panic: $BAD"
echo "$FINAL_TX"
echo "$FINAL_RX"

if [ "$BAD" != "0" ]; then
  echo "VERDICT_TX=KERNEL_OR_DRIVER_BAD_LOGS"
elif [ "$LATE" = "0" ] && [ "$SSB" != "0" ]; then
  echo "VERDICT_TX=CLEAN"
elif [ "$LATE" -le 5 ] && [ "$SSB" != "0" ]; then
  echo "VERDICT_TX=NEAR_CLEAN"
else
  echo "VERDICT_TX=NOT_CLEAN"
fi

echo
echo "===== cheap spectrum check ====="
cd /home/stefan/CLionProjects || exit 1

python3 - <<'PY' "$IQ"
import sys
import numpy as np
from pathlib import Path

iq_path = Path(sys.argv[1])
FS = 11.52e6
SSB_OFFSET = -2.85e6

raw = np.fromfile(iq_path, dtype="<i2")
raw = raw[: raw.size - raw.size % 2]
iq = raw.reshape(-1, 2)
x = (iq[:,0].astype(np.float32) + 1j * iq[:,1].astype(np.float32)) / 2048.0

rms = 20*np.log10(np.sqrt(np.mean(np.abs(x)**2))+1e-12)
peak = 20*np.log10(np.max(np.abs(x))+1e-12)
clip = 100*np.mean((np.abs(x.real) > 0.98) | (np.abs(x.imag) > 0.98))

print(f"IQ={iq_path}")
print(f"samples={x.size} duration_ms={x.size/FS*1e3:.3f}")
print(f"rms_dbfs={rms:.2f}")
print(f"peak_dbfs={peak:.2f}")
print(f"clip_pct={clip:.5f}")

nfft = 65536
nseg = min(64, len(x)//nfft)
win = np.hanning(nfft).astype(np.float32)
acc = np.zeros(nfft, np.float64)

for k in range(nseg):
    s = x[k*nfft:(k+1)*nfft]
    X = np.fft.fftshift(np.fft.fft(s * win))
    acc += np.abs(X)**2

acc /= max(nseg, 1)
f = np.fft.fftshift(np.fft.fftfreq(nfft, 1/FS))
p = 10*np.log10(acc + 1e-30)

def band(lo, hi):
    m = (f >= lo) & (f <= hi)
    return 10*np.log10(np.mean(10**(p[m]/10)) + 1e-30)

for width in [20e3, 100e3, 500e3, 1e6]:
    lo = SSB_OFFSET - width/2
    hi = SSB_OFFSET + width/2
    print(f"ssb_band_width_hz={width:9.0f} power_db={band(lo, hi):8.2f}")

mask = np.abs(f) > 50e3
idx = np.argmax(p[mask])
print(f"max_freq_hz={f[mask][idx]:+.1f} max_psd_db={p[mask][idx]:.2f}")
PY

echo
echo "===== bounded reduced PSS scan, timeout 20s ====="
DET="/home/stefan/CLionProjects/m2sdr/scripts/capture_detect_ssb.py"

timeout 20 python3 - <<'PY' "$IQ" "$DET"
import sys, importlib.util
from pathlib import Path
import numpy as np

iq_path = Path(sys.argv[1])
det_path = Path(sys.argv[2])

spec = importlib.util.spec_from_file_location("ssbdet", det_path)
ssbdet = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ssbdet)

FS = 11.52e6
SSB_OFFSET = -2.85e6
SCS_KHZ = 15
EXPECTED = 1
nfft = int(round(FS / (SCS_KHZ * 1e3)))

raw = np.fromfile(iq_path, dtype="<i2")
raw = raw[: raw.size - raw.size % 2]
iq = raw.reshape(-1, 2)
x = (iq[:,0].astype(np.float32) + 1j * iq[:,1].astype(np.float32)) / 2048.0

win_len = int(round(45e-3 * FS))
starts_ms = [0, 25, 50, 75, 100, 150, 200, 250, 300, 350]

rows = []
for t_ms in starts_ms:
    start = int(round(t_ms * 1e-3 * FS))
    win = x[start:start+win_len]
    if len(win) < win_len:
        continue

    per = ssbdet.search_pss(
        win,
        FS,
        nfft,
        SSB_OFFSET,
        -120000,
        120000,
        30000,
    )

    best = max(range(3), key=lambda n: per[n][0])
    bm, bcfo, bidx, bsnr = per[best]
    em, ecfo, eidx, esnr = per[EXPECTED]

    rows.append((t_ms, best, bm, bcfo, em, ecfo))
    print(
        f"t_ms={t_ms:7.1f} "
        f"best_nid2={best} best_margin={bm:7.2f} best_cfo={bcfo:+7d} "
        f"expected_margin={em:7.2f} expected_cfo={ecfo:+7d}",
        flush=True,
    )

if not rows:
    print("VERDICT=NO_ROWS")
    raise SystemExit(3)

best_exp = max(rows, key=lambda r: r[4])
best_any = max(rows, key=lambda r: r[2])

print()
print("SUMMARY")
print(f"best_any_nid2={best_any[1]} best_any_margin={best_any[2]:.2f} at_t_ms={best_any[0]:.1f}")
print(f"best_expected_margin={best_exp[4]:.2f} best_expected_cfo={best_exp[5]:+d} at_t_ms={best_exp[0]:.1f}")

if best_exp[4] >= 20:
    print("VERDICT=EXPECTED_PCI_VISIBLE")
elif best_exp[4] >= 12:
    print("VERDICT=EXPECTED_PCI_WEAK")
else:
    print("VERDICT=EXPECTED_PCI_NOT_VISIBLE")
PY

SCAN_RC=$?

echo
echo "===== final files ====="
echo "BASE=$BASE"
echo "STDOUT=$STDOUT"
echo "SUMMARY=$SUMMARY"
echo "IQ=$IQ"
echo "CAP_RC=$CAP_RC"
echo "TX_RC=$TX_RC"
echo "SCAN_RC=$SCAN_RC"

exit 0
