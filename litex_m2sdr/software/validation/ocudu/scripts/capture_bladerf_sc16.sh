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
      return 0
    fi
  else
    echo "RAW_BYTES_$MODE=0"
  fi

  return 1
}

{
  echo "===== bladeRF SC16 capture v2 ====="
  hostname
  date -u --iso-8601=seconds
  echo "FC=$FC"
  echo "FS=$FS"
  echo "BW=$BW"
  echo "GAIN=$GAIN"
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

{
  echo "===== compact bladeRF capture summary ====="
  echo "OUT=$OUT"
  echo "LOG=$LOG"
  echo "RAW=$RAW"
  grep -Ei 'try mode|Frequency|sample rate|Bandwidth|Gain|agc|Error|Warning|BLADERF_CLI_RC_|RAW_BYTES_|EXPECTED_BYTES=|MIN_BYTES=|CAPTURE_MODE=|CAPTURE_OK=|CAPTURE_FILE_PRESENT=|RAW=' "$LOG" | sed -n '1,260p'
  echo "WRAPPER_RC=$RC"
} | tee "$SUMMARY"

rm -rf "$TMPROOT" 2>/dev/null || true

echo
echo "OUT=$OUT"
echo "LOG=$LOG"
echo "SUMMARY=$SUMMARY"
cat "$SUMMARY"

exit "$RC"
