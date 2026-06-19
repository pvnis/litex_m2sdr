#!/usr/bin/env bash
set -euo pipefail

CENTER_HZ="${1:?usage: $0 CENTER_HZ SAMPLE_RATE_HZ N_SAMPLES OUT_DIR}"
FS_HZ="${2:?usage: $0 CENTER_HZ SAMPLE_RATE_HZ N_SAMPLES OUT_DIR}"
N_SAMPLES="${3:?usage: $0 CENTER_HZ SAMPLE_RATE_HZ N_SAMPLES OUT_DIR}"
OUT="${4:?usage: $0 CENTER_HZ SAMPLE_RATE_HZ N_SAMPLES OUT_DIR}"

mkdir -p "$OUT"

RAW="$OUT/capture_fc${CENTER_HZ}_fs${FS_HZ}.sc16"
CMDS="$OUT/bladerf_capture.cmd"
REPORT="$OUT/report.log"

cat > "$CMDS" <<EOF2
set frequency rx $CENTER_HZ
set samplerate rx $FS_HZ
set bandwidth rx 5000000
rx config file=$RAW format=bin n=$N_SAMPLES
rx start
rx wait
EOF2

{
  echo "===== bladeRF minimal SC16 capture ====="
  echo "CENTER_HZ=$CENTER_HZ"
  echo "FS_HZ=$FS_HZ"
  echo "N_SAMPLES=$N_SAMPLES"
  echo "RAW=$RAW"
  echo "CMDS=$CMDS"
  date -u --iso-8601=seconds

  echo
  echo "===== command file ====="
  cat "$CMDS"

  echo
  echo "===== run bladeRF capture ====="
  sudo rm -f "$RAW"
  sudo bladeRF-cli -s "$CMDS" 2>&1
  RC=$?
  echo "bladeRF-cli_rc=$RC"

  sudo chown "$USER:$USER" "$RAW" 2>/dev/null || true

  echo
  echo "===== raw file ====="
  ls -lh "$RAW" 2>/dev/null || true
} 2>&1 | tee "$REPORT"

echo
echo "OUT=$OUT"
echo "RAW=$RAW"
echo "REPORT=$REPORT"
