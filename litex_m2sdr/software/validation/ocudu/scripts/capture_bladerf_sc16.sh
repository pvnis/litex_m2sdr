#!/usr/bin/env bash
set +e +u
set +o pipefail 2>/dev/null || true

FC="${FC:-1842500000}"
FS="${FS:-11520000}"
BW="${BW:-10000000}"
GAIN="${GAIN:-60}"
N="${N:-11520000}"
OUT="${OUT:-$HOME/pavonis_bladerf/runs/bladerf_capture_$(date -u +%Y%m%dT%H%M%SZ)}"
RAW="${RAW:-$OUT/capture_fc${FC}_fs${FS}.sc16}"
TIMEOUT_S="${TIMEOUT_S:-45}"

mkdir -p "$OUT" || exit 1

TMPROOT="$(mktemp -d /tmp/bladerf_cap.XXXXXX)"
TMPRAW="$TMPROOT/cap.sc16"

LOG="$OUT/capture_bladerf_sc16.log"
SUMMARY="$OUT/SUMMARY.txt"

EXPECTED_BYTES=$((N * 4))
MIN_BYTES=$((EXPECTED_BYTES * 90 / 100))

emit_sc16_stats() {
  RAW_FOR_STATS="$1"

  python3 - <<'PY' "$RAW_FOR_STATS"
import sys
from pathlib import Path

import numpy as np

p = Path(sys.argv[1])
raw = np.fromfile(p, dtype=np.int16)

if raw.size < 4:
    print("RAW_STATS_OK=0")
    print(f"RAW_STATS_ERROR=empty_or_too_small:{p}")
    raise SystemExit(0)

if raw.size % 2:
    raw = raw[:-1]

iq = raw.reshape(-1, 2)
i16 = iq[:, 0]
q16 = iq[:, 1]

i = i16.astype(np.float64)
q = q16.astype(np.float64)
mag = np.sqrt(i * i + q * q)

component_abs_max = int(max(np.max(np.abs(i16)), np.max(np.abs(q16))))
clip2040 = int(np.count_nonzero((np.abs(i16) >= 2040) | (np.abs(q16) >= 2040)))
clip2000 = int(np.count_nonzero((np.abs(i16) >= 2000) | (np.abs(q16) >= 2000)))
clip32760 = int(np.count_nonzero((np.abs(i16) >= 32760) | (np.abs(q16) >= 32760)))

rms_rel_2048 = float(np.sqrt(np.mean(i * i + q * q)) / 2048.0)
rms_rel_32768 = float(np.sqrt(np.mean(i * i + q * q)) / 32768.0)
peak_rel_2048 = float(component_abs_max / 2048.0)

p50_rel_2048 = float(np.percentile(mag / 2048.0, 50))
p95_rel_2048 = float(np.percentile(mag / 2048.0, 95))
p99_rel_2048 = float(np.percentile(mag / 2048.0, 99))
p999_rel_2048 = float(np.percentile(mag / 2048.0, 99.9))

def db20(x):
    return 20.0 * np.log10(float(x) + 1e-15)

print("===== SC16 amplitude stats =====")
print("RAW_STATS_OK=1")
print(f"RAW_SAMPLES={iq.shape[0]}")
print(f"RAW_COMPONENT_ABS_MAX={component_abs_max}")
print(f"RAW_PEAK_REL_2048={peak_rel_2048:.6f}")
print(f"RAW_CLIP2000_COUNT={clip2000}")
print(f"RAW_CLIP2040_COUNT={clip2040}")
print(f"RAW_CLIP32760_COUNT={clip32760}")
print(f"RAW_RMS_REL_2048={rms_rel_2048:.6f}")
print(f"RAW_RMS_REL_2048_DBFS={db20(rms_rel_2048):.2f}")
print(f"RAW_RMS_REL_32768={rms_rel_32768:.6f}")
print(f"RAW_RMS_REL_32768_DBFS={db20(rms_rel_32768):.2f}")
print(f"RAW_MAG_P50_REL_2048={p50_rel_2048:.6f}")
print(f"RAW_MAG_P95_REL_2048={p95_rel_2048:.6f}")
print(f"RAW_MAG_P99_REL_2048={p99_rel_2048:.6f}")
print(f"RAW_MAG_P999_REL_2048={p999_rel_2048:.6f}")

if component_abs_max >= 2040 or clip2040 > 0:
    print("CAPTURE_LEVEL_WARNING=SC16_NEAR_OR_AT_2048_RAIL")
elif component_abs_max >= 2000 or clip2000 > 0:
    print("CAPTURE_LEVEL_WARNING=SC16_CLOSE_TO_2048_RAIL")
else:
    print("CAPTURE_LEVEL_WARNING=OK")
PY
}

run_one() {
  MODE="$1"
  SCRIPT="$TMPROOT/${MODE}.bladerf"
  rm -f "$TMPRAW"

  case "$MODE" in
    manual_gain)
      cat > "$SCRIPT" <<EOC
set frequency rx $FC
set samplerate rx $FS
set bandwidth rx $BW
set agc rx off
set gain rx $GAIN
rx config file=$TMPRAW format=bin n=$N
rx start
rx wait
EOC
      ;;
    agc)
      cat > "$SCRIPT" <<EOC
set frequency rx $FC
set samplerate rx $FS
set bandwidth rx $BW
set agc rx on
rx config file=$TMPRAW format=bin n=$N
rx start
rx wait
EOC
      ;;
    no_gain)
      cat > "$SCRIPT" <<EOC
set frequency rx $FC
set samplerate rx $FS
set bandwidth rx $BW
rx config file=$TMPRAW format=bin n=$N
rx start
rx wait
EOC
      ;;
    *)
      echo "unknown mode: $MODE"
      return 99
      ;;
  esac

  echo
  echo "===== try mode: $MODE ====="
  echo "SCRIPT=$SCRIPT"
  cat "$SCRIPT"

  timeout "$TIMEOUT_S" sudo bladeRF-cli -s "$SCRIPT" 2>&1
  RC=$?
  echo "BLADERF_CLI_RC_$MODE=$RC"

  if [ -s "$TMPRAW" ]; then
    sudo chown "$USER:$USER" "$TMPRAW" 2>/dev/null || true
    BYTES="$(stat -c%s "$TMPRAW" 2>/dev/null || echo 0)"
    echo "RAW_BYTES_$MODE=$BYTES"
    echo "EXPECTED_BYTES=$EXPECTED_BYTES"
    echo "MIN_BYTES=$MIN_BYTES"

    if [ "$BYTES" -ge "$MIN_BYTES" ]; then
      mv -f "$TMPRAW" "$RAW"
      echo "CAPTURE_MODE=$MODE"
      echo "CAPTURE_FILE_PRESENT=1"
      echo "RAW=$RAW"
      ls -lh "$RAW"
      sha256sum "$RAW"
      emit_sc16_stats "$RAW"
      return 0
    fi
  else
    echo "RAW_BYTES_$MODE=0"
  fi

  return 1
}

{
  echo "===== bladeRF SC16 capture v3 ====="
  hostname
  date -u --iso-8601=seconds
  echo "FC=$FC"
  echo "FC_MHz=$(python3 - <<PY
print(float("$FC") / 1e6)
PY
)"
  echo "FC_GHz=$(python3 - <<PY
print(float("$FC") / 1e9)
PY
)"
  echo "FS=$FS"
  echo "BW=$BW"
  echo "GAIN_REQUESTED=$GAIN"
  echo "N=$N"
  echo "OUT=$OUT"
  echo "RAW=$RAW"
  echo "TMPROOT=$TMPROOT"
  echo "EXPECTED_BYTES=$EXPECTED_BYTES"

  echo
  echo "===== bladeRF probe ====="
  timeout 10 sudo bladeRF-cli -p 2>&1 || true

  CAPTURE_OK=0
  for MODE in manual_gain agc no_gain; do
    run_one "$MODE"
    R=$?
    if [ "$R" -eq 0 ]; then
      CAPTURE_OK=1
      break
    fi
  done

  echo
  echo "===== final ====="
  echo "CAPTURE_OK=$CAPTURE_OK"
  echo "RAW=$RAW"

  if [ "$CAPTURE_OK" -ne 1 ]; then
    echo "CAPTURE_FILE_PRESENT=0"
    echo "ERROR: all bladeRF capture modes failed"
    ls -lh "$OUT" "$TMPROOT" 2>/dev/null || true
    exit 10
  fi

  echo "CAPTURE_FILE_PRESENT=1"
  exit 0
} 2>&1 | tee "$LOG"

RC=${PIPESTATUS[0]}

GAIN_READBACK_DB="$(
  awk '
    /Gain RX1 overall:/ {
      for (i = 1; i <= NF; i++) {
        if ($i ~ /^[-+]?[0-9]+$/) {
          val = $i
        }
      }
    }
    END {
      if (val != "") print val
    }
  ' "$LOG" 2>/dev/null
)"

{
  echo "===== compact bladeRF capture summary ====="
  echo "OUT=$OUT"
  echo "LOG=$LOG"
  echo "RAW=$RAW"
  echo "GAIN_REQUESTED=$GAIN"
  if [ -n "$GAIN_READBACK_DB" ]; then
    echo "GAIN_READBACK_DB=$GAIN_READBACK_DB"
    if [ "$GAIN_READBACK_DB" = "$GAIN" ]; then
      echo "GAIN_READBACK_MATCH=1"
    else
      echo "GAIN_READBACK_MATCH=0"
    fi
  else
    echo "GAIN_READBACK_DB=UNKNOWN"
    echo "GAIN_READBACK_MATCH=UNKNOWN"
  fi

  grep -Ei 'try mode|Frequency|sample rate|Bandwidth|Gain|agc|Error|Warning|BLADERF_CLI_RC_|RAW_BYTES_|EXPECTED_BYTES=|MIN_BYTES=|CAPTURE_MODE=|CAPTURE_OK=|CAPTURE_FILE_PRESENT=|RAW=|RAW_STATS_OK=|RAW_SAMPLES=|RAW_COMPONENT_ABS_MAX=|RAW_PEAK_REL_2048=|RAW_CLIP2000_COUNT=|RAW_CLIP2040_COUNT=|RAW_CLIP32760_COUNT=|RAW_RMS_REL_2048=|RAW_RMS_REL_2048_DBFS=|RAW_MAG_P|CAPTURE_LEVEL_WARNING=' "$LOG" | sed -n '1,340p'

  echo "WRAPPER_RC=$RC"
} | tee "$SUMMARY"

rm -rf "$TMPROOT" 2>/dev/null || true

echo
echo "OUT=$OUT"
echo "LOG=$LOG"
echo "SUMMARY=$SUMMARY"
cat "$SUMMARY"

exit "$RC"
