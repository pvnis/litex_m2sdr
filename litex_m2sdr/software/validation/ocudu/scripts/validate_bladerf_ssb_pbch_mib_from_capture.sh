#!/usr/bin/env bash
# Validate an OCUDU/M2SDR radio_ssb OTA capture using:
#   1. offline PSS/SSS/PCI detector
#   2. srsRAN ssb_file_test PBCH CRC + MIB decode on a known-good crop
#
# Input: bladeRF SC16 Q11 capture centered on the SSB.
#
# Defaults match the validated Pavonis 3.5 GHz SSB-centered run:
#   fs=5.76 Msps
#   fc=3497.150 MHz
#   ssb=3497.150 MHz
#   PCI=1
#
# No SDR hardware is touched by this script.

set +e +u
set +o pipefail 2>/dev/null || true

RUN="${1:-$(ls -td "$HOME"/pavonis_bladerf_local/runs/bladerf_radio_ssb_ssbcenter_* 2>/dev/null | head -1)}"

FS="${FS:-5760000}"
FC="${FC:-3497150000}"
SSB_FREQ="${SSB_FREQ:-3497150000}"
SCS="${SCS:-15}"
EXPECTED_PCI="${EXPECTED_PCI:-1}"

# Known-good crop for the SSB-centered validation capture.
# This is the first high-energy segment from the offline detector.
CROP_START="${CROP_START:-0}"
CROP_N="${CROP_N:-245760}"

# Optional CFO-corrected variant. Both plain and -10 kHz corrected passed
# on the validated capture; the script accepts either.
CFO_ROT_HZ="${CFO_ROT_HZ:--10000}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLS="${TOOLS:-$SCRIPT_DIR}"
PY="${PY:-$HOME/venvs/pavonis-iq/bin/python}"
SRSRAN_4G_REPO="${SRSRAN_4G_REPO:-$HOME/CLionProjects/srsRAN_4G}"
SSB_BIN="${SSB_BIN:-$SRSRAN_4G_REPO/build-clion/lib/src/phy/sync/test/ssb_file_test}"

RAW="${RAW:-$(ls "$RUN"/radio_ssb_fc${FC}_fs${FS}.sc16 2>/dev/null | head -1)}"

TS="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="$RUN/validation_ssb_pbch_mib_$TS"
mkdir -p "$OUT"

OFFLINE_LOG="$OUT/offline_pss_sss.log"
CROP_LOG="$OUT/crop.log"
SRS_LOG="$OUT/srsran_pbch_mib.log"
PROOF="$OUT/PROOF_srsran_pbch_mib_decode.txt"
LATEST_PROOF="$RUN/PROOF_srsran_pbch_mib_decode_latest.txt"

CROP_CF32="$OUT/knownhit_start${CROP_START}_n${CROP_N}.cf32"
CROP_ROT_CF32="$OUT/knownhit_start${CROP_START}_n${CROP_N}_cfo_${CFO_ROT_HZ}.cf32"

{
  echo "===== validate_bladerf_ssb_pbch_mib_from_capture ====="
  hostname
  date -u --iso-8601=seconds

  echo
  echo "===== configuration ====="
  echo "RUN=$RUN"
  echo "RAW=$RAW"
  echo "FS=$FS"
  echo "FC=$FC"
  echo "SSB_FREQ=$SSB_FREQ"
  echo "SCS=$SCS"
  echo "EXPECTED_PCI=$EXPECTED_PCI"
  echo "CROP_START=$CROP_START"
  echo "CROP_N=$CROP_N"
  echo "CFO_ROT_HZ=$CFO_ROT_HZ"
  echo "TOOLS=$TOOLS"
  echo "PY=$PY"
  echo "SRSRAN_4G_REPO=$SRSRAN_4G_REPO"
  echo "SSB_BIN=$SSB_BIN"
  echo "OUT=$OUT"

  echo
  echo "===== input checks ====="
  ls -lh "$RAW" "$PY" "$SSB_BIN" 2>/dev/null || true
  sha256sum "$RAW" 2>/dev/null || true

  if [ ! -r "$RAW" ]; then
    echo "ERROR: missing readable RAW capture"
    exit 10
  fi

  if [ ! -x "$PY" ]; then
    echo "ERROR: missing Python env: $PY"
    exit 11
  fi

  if [ ! -x "$SSB_BIN" ]; then
    echo "ERROR: missing srsRAN ssb_file_test binary: $SSB_BIN"
    echo "Build with:"
    echo "  cd \"$SRSRAN_4G_REPO\""
    echo "  cmake --build build-clion --target ssb_file_test -j\"\$(nproc)\""
    exit 12
  fi

  if [ ! -r "$TOOLS/detect_nr_pss_sss_from_sc16.py" ]; then
    echo "ERROR: missing offline detector: $TOOLS/detect_nr_pss_sss_from_sc16.py"
    exit 13
  fi

  echo
  echo "===== Python env ====="
  "$PY" - <<'PY'
import sys
print("python =", sys.executable)
try:
    import numpy as np
    print("numpy =", np.__version__)
except Exception as e:
    print("numpy import error =", repr(e))
PY

  echo
  echo "===== offline PSS/SSS/PCI validation ====="
  "$PY" "$TOOLS/detect_nr_pss_sss_from_sc16.py" "$RAW" \
    --fs "$FS" \
    --fc "$FC" \
    --ssb "$SSB_FREQ" \
    --expected-pci "$EXPECTED_PCI" \
    2>&1 | tee "$OFFLINE_LOG"
  OFFLINE_RC=${PIPESTATUS[0]}
  echo "OFFLINE_RC=$OFFLINE_RC"

  echo
  echo "===== make known-good crop CF32 files ====="
  "$PY" - <<'PY' "$RAW" "$CROP_CF32" "$CROP_ROT_CF32" "$CROP_START" "$CROP_N" "$FS" "$CFO_ROT_HZ" | tee "$CROP_LOG"
import sys
from pathlib import Path
import numpy as np

raw_path = Path(sys.argv[1])
out_plain = Path(sys.argv[2])
out_rot = Path(sys.argv[3])
start = int(sys.argv[4])
n = int(sys.argv[5])
fs = float(sys.argv[6])
rot_hz = float(sys.argv[7])

raw = np.fromfile(raw_path, dtype="<i2")
if raw.size % 2:
    raw = raw[:-1]

iq = raw.reshape(-1, 2)
if start < 0 or start + n > iq.shape[0]:
    raise SystemExit(f"crop out of range: start={start} n={n} total={iq.shape[0]}")

seg = iq[start:start+n]
x = (seg[:, 0].astype(np.float32) + 1j * seg[:, 1].astype(np.float32)) / 2048.0
x.astype(np.complex64).tofile(out_plain)

idx = np.arange(x.size, dtype=np.float64)
y = x * np.exp(2j * np.pi * rot_hz * idx / fs).astype(np.complex64)
y.astype(np.complex64).tofile(out_rot)

print(f"raw_samples_total={iq.shape[0]}")
print(f"crop_start={start}")
print(f"crop_samples={x.size}")
print(f"crop_duration_s={x.size / fs:.6f}")
print(f"rot_hz={rot_hz:+.1f}")
print(f"plain={out_plain}")
print(f"plain_size_bytes={out_plain.stat().st_size}")
print(f"rotated={out_rot}")
print(f"rotated_size_bytes={out_rot.stat().st_size}")
print("DONE_CROP")
PY
  CROP_RC=${PIPESTATUS[0]}
  echo "CROP_RC=$CROP_RC"
  ls -lh "$CROP_CF32" "$CROP_ROT_CF32" 2>/dev/null || true

  if [ "$CROP_RC" -ne 0 ]; then
    echo "ERROR: crop generation failed"
    exit "$CROP_RC"
  fi

  echo
  echo "===== srsRAN PBCH/MIB decode on known-good crop ====="
  cd "$SRSRAN_4G_REPO" || exit 14

  : > "$SRS_LOG"

  for FILE_LABEL in crop crop_cfo; do
    if [ "$FILE_LABEL" = "crop" ]; then
      FILE="$CROP_CF32"
    else
      FILE="$CROP_ROT_CF32"
    fi

    echo | tee -a "$SRS_LOG"
    echo "===== TRY FILE=$FILE_LABEL =====" | tee -a "$SRS_LOG"
    "$SSB_BIN" \
      -i "$FILE" \
      -n "$CROP_N" \
      -r "$FS" \
      -s "$SCS" \
      -f "$FC" \
      -F "$SSB_FREQ" \
      -v \
      2>&1 | tee -a "$SRS_LOG"
    RC=${PIPESTATUS[0]}
    echo "TRY_FILE=$FILE_LABEL RC=$RC" | tee -a "$SRS_LOG"
  done

  OFF_OK=0
  PBCH_OK=0

  grep -q 'VERDICT=SSS_EXPECTED_PCI_DOMINATES' "$OFFLINE_LOG" && OFF_OK=1
  grep -q 'crc=OK' "$SRS_LOG" && PBCH_OK=1

  echo
  echo "===== proof bundle ====="
  {
    echo "===== Pavonis OCUDU/M2SDR/bladeRF SSB + PBCH/MIB proof ====="
    hostname
    date -u --iso-8601=seconds

    echo
    echo "===== capture artifact ====="
    echo "RUN=$RUN"
    echo "RAW=$RAW"
    ls -lh "$RAW" "$CROP_CF32" "$CROP_ROT_CF32" 2>/dev/null || true
    sha256sum "$RAW" 2>/dev/null || true

    echo
    echo "===== offline PSS/SSS verdict ====="
    grep -Ei 'EXPECTED_PCI=|BEST_WRONG_PCI=|expected_vs_best_wrong_ratio=|VERDICT=|score=.*pci=1|cfo_Hz=' \
      "$OFFLINE_LOG" 2>/dev/null | sed -n '1,100p'

    echo
    echo "===== srsRAN known-hit PBCH/MIB decode ====="
    grep -Ei 'crop_samples=|crop_duration_s=|TRY FILE=|measure - search|search -|PBCH-MIB|crc=OK|TRY_FILE=' \
      "$CROP_LOG" "$SRS_LOG" 2>/dev/null

    echo
    echo "===== final verdict ====="
    echo "OFFLINE_PSS_SSS_OK=$OFF_OK"
    echo "SRSRAN_PBCH_CRC_OK=$PBCH_OK"

    if [ "$OFF_OK" = "1" ] && [ "$PBCH_OK" = "1" ]; then
      echo "VERDICT=PSS_SSS_PCI1_PBCH_CRC_OK_MIB_DECODED"
    else
      echo "VERDICT=FAIL"
    fi
  } | tee "$PROOF"

  cp -av "$PROOF" "$LATEST_PROOF" 2>/dev/null || true

  echo
  echo "PROOF=$PROOF"
  echo "LATEST_PROOF=$LATEST_PROOF"

  if [ "$OFF_OK" = "1" ] && [ "$PBCH_OK" = "1" ]; then
    exit 0
  fi

  exit 1
} 2>&1 | tee "$OUT/run.log"

exit ${PIPESTATUS[0]}
