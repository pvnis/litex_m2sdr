#!/usr/bin/env bash
set -Eeuo pipefail

# Analyze a local bladeRF capture run:
#   1. run offline PSS/SSS/PCI detector
#   2. select a crop from the strongest expected PCI hit
#   3. run the committed PBCH/MIB validator
#
# Usage:
#   ./scripts/analyze_cluster_rf_run.sh /path/to/local/run
#
# Defaults target the current real-gNB Band 3 SSB-centered capture:
#   FS=5760000
#   FC=1839650000
#   SSB_FREQ=1839650000
#   EXPECTED_PCI=1
#
# Override example:
#   FS=5760000 FC=1839650000 SSB_FREQ=1839650000 EXPECTED_PCI=1 ./scripts/analyze_cluster_rf_run.sh "$RUN"

RUN="${1:-}"
if [ -z "$RUN" ]; then
  RUN="$(ls -td "$HOME"/pavonis_bladerf_local/runs/bladerf_gnb_b3_* 2>/dev/null | head -1 || true)"
fi

if [ -z "$RUN" ] || [ ! -d "$RUN" ]; then
  echo "ERROR: missing run directory. Usage: $0 /path/to/local/run" >&2
  exit 2
fi

RAW="${RAW:-$(find "$RUN" -maxdepth 1 -type f -name '*.sc16' | sort | head -1)}"
if [ -z "$RAW" ] || [ ! -f "$RAW" ]; then
  echo "ERROR: no .sc16 capture found in $RUN" >&2
  exit 2
fi

# Infer FC/FS from common filename pattern *_fc<Hz>_fs<Hz>.sc16, unless explicitly set.
B="$(basename "$RAW")"
if [[ "$B" =~ fc([0-9]+)_fs([0-9]+) ]]; then
  INFER_FC="${BASH_REMATCH[1]}"
  INFER_FS="${BASH_REMATCH[2]}"
else
  INFER_FC=""
  INFER_FS=""
fi

FS="${FS:-${INFER_FS:-5760000}}"
FC="${FC:-${INFER_FC:-1839650000}}"
SSB_FREQ="${SSB_FREQ:-$FC}"
EXPECTED_PCI="${EXPECTED_PCI:-1}"
CROP_N="${CROP_N:-245760}"

# For PBCH, leave CFO rotation disabled by default. srsRAN handles small residual CFO;
# in the previous real-gNB run, forced rotation made the residual worse.
CFO_ROT_HZ="${CFO_ROT_HZ:-0}"

PY="${PY:-$HOME/venvs/pavonis-iq/bin/python}"
REPO="${REPO:-$HOME/CLionProjects/litex_m2sdr}"
TOOLS="${TOOLS:-$REPO/litex_m2sdr/software/validation/ocudu/scripts}"
DETECT="${DETECT:-$TOOLS/detect_nr_pss_sss_from_sc16.py}"
VALIDATOR="${VALIDATOR:-$TOOLS/validate_bladerf_ssb_pbch_mib_from_capture.sh}"

TS="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="$RUN/analysis_cluster_${TS}"
mkdir -p "$OUT"

DETECT_LOG="$OUT/offline_pss_sss.log"
SUMMARY="$OUT/SUMMARY.txt"
SELECT_ENV="$OUT/selected_crop.env"
VALIDATOR_LOG="$OUT/validate_pbch_mib.log"

echo "===== analyze_cluster_rf_run =====" | tee "$SUMMARY"
{
  hostname
  date -u --iso-8601=seconds
  echo "RUN=$RUN"
  echo "RAW=$RAW"
  echo "FS=$FS"
  echo "FC=$FC"
  echo "SSB_FREQ=$SSB_FREQ"
  echo "EXPECTED_PCI=$EXPECTED_PCI"
  echo "CROP_N=$CROP_N"
  echo "CFO_ROT_HZ=$CFO_ROT_HZ"
  echo "PY=$PY"
  echo "DETECT=$DETECT"
  echo "VALIDATOR=$VALIDATOR"
  echo "OUT=$OUT"
  echo
  ls -lh "$RAW" "$DETECT" "$VALIDATOR" 2>/dev/null || true
  sha256sum "$RAW" 2>/dev/null || true
} | tee -a "$SUMMARY"

echo | tee -a "$SUMMARY"
echo "===== run offline PSS/SSS detector =====" | tee -a "$SUMMARY"

timeout 180 nice -n 10 "$PY" "$DETECT" "$RAW" \
  --fs "$FS" \
  --fc "$FC" \
  --ssb "$SSB_FREQ" \
  --expected-pci "$EXPECTED_PCI" \
  > "$DETECT_LOG" 2>&1

DETECT_RC=$?

{
  echo "DETECT_RC=$DETECT_RC"
  grep -Ei 'samples=|duration_s|expected_ssb_offset_Hz|expected_pci|expected_nid|VERDICT=|EXPECTED_PCI=|BEST_WRONG_PCI=|expected_vs_best_wrong_ratio=|score=|pci=|nid2=|cfo_Hz|segment samples' "$DETECT_LOG" | sed -n '1,260p'
} | tee -a "$SUMMARY"

if ! grep -q 'VERDICT=SSS_EXPECTED_PCI_DOMINATES' "$DETECT_LOG"; then
  echo | tee -a "$SUMMARY"
  echo "FINAL_VERDICT=PSS_SSS_FAILED_OR_AMBIGUOUS" | tee -a "$SUMMARY"
  echo "SUMMARY=$SUMMARY"
  exit 10
fi

echo | tee -a "$SUMMARY"
echo "===== select crop from detector log =====" | tee -a "$SUMMARY"

"$PY" - <<'PY' "$DETECT_LOG" "$FS" "$CROP_N" "$SELECT_ENV"
import re
import sys
from pathlib import Path

log = Path(sys.argv[1]).read_text(errors="replace")
fs = float(sys.argv[2])
crop_n = int(sys.argv[3])
out = Path(sys.argv[4])

segments = []
for m in re.finditer(r"segment samples\s+(\d+):(\d+)\s+time\s+([0-9.]+)-([0-9.]+)s", log):
    segments.append((int(m.group(1)), int(m.group(2)), float(m.group(3)), float(m.group(4))))

m = re.search(
    r"EXPECTED_PCI=\d+:\s+score=([0-9.]+).*?cfo_Hz=([+-]?[0-9.]+).*?time_s=([0-9.]+)",
    log,
)
if not m:
    raise SystemExit("ERROR: could not parse EXPECTED_PCI best line")

score = float(m.group(1))
observed_cfo = float(m.group(2))
time_s = float(m.group(3))

chosen = None
for start, end, t0, t1 in segments:
    if t0 <= time_s <= t1:
        chosen = start
        break

if chosen is None:
    # Fall back to a crop starting slightly before the detected SSB hit.
    chosen = max(0, int(round(time_s * fs - 0.013 * fs)))

with out.open("w") as f:
    f.write(f"CROP_START={chosen}\n")
    f.write(f"BEST_EXPECTED_SCORE={score}\n")
    f.write(f"BEST_EXPECTED_TIME_S={time_s}\n")
    f.write(f"OBSERVED_CFO_HZ={observed_cfo}\n")

print(f"CROP_START={chosen}")
print(f"BEST_EXPECTED_SCORE={score}")
print(f"BEST_EXPECTED_TIME_S={time_s}")
print(f"OBSERVED_CFO_HZ={observed_cfo}")
print(f"SELECT_ENV={out}")
PY

cat "$SELECT_ENV" | tee -a "$SUMMARY"
# shellcheck disable=SC1090
source "$SELECT_ENV"

echo | tee -a "$SUMMARY"
echo "===== run PBCH/MIB validator =====" | tee -a "$SUMMARY"

set +e
FS="$FS" \
FC="$FC" \
SSB_FREQ="$SSB_FREQ" \
EXPECTED_PCI="$EXPECTED_PCI" \
CROP_START="$CROP_START" \
CROP_N="$CROP_N" \
CFO_ROT_HZ="$CFO_ROT_HZ" \
RAW="$RAW" \
"$VALIDATOR" "$RUN" > "$VALIDATOR_LOG" 2>&1
VALIDATOR_RC=$?
set -e

{
  echo "VALIDATOR_RC=$VALIDATOR_RC"
  grep -Ei 'OFFLINE_PSS_SSS_OK=|SRSRAN_PBCH_CRC_OK=|VERDICT=|crc=OK|crc=KO|PBCH-MIB|EXPECTED_PCI=|BEST_WRONG_PCI=|expected_vs_best_wrong_ratio=|TRY FILE=|measure - search|search -|crop_samples|crop_duration' \
    "$VALIDATOR_LOG" | sed -n '1,260p'
} | tee -a "$SUMMARY"

echo | tee -a "$SUMMARY"
echo "===== final =====" | tee -a "$SUMMARY"

if grep -q 'SRSRAN_PBCH_CRC_OK=1' "$VALIDATOR_LOG"; then
  echo "FINAL_VERDICT=PSS_SSS_PCI_AND_PBCH_MIB_PASS" | tee -a "$SUMMARY"
  RC=0
else
  echo "FINAL_VERDICT=PSS_SSS_PCI_PASS_PBCH_FAIL" | tee -a "$SUMMARY"
  RC=20
fi

echo "OUT=$OUT" | tee -a "$SUMMARY"
echo "SUMMARY=$SUMMARY" | tee -a "$SUMMARY"
echo "DETECT_LOG=$DETECT_LOG" | tee -a "$SUMMARY"
echo "VALIDATOR_LOG=$VALIDATOR_LOG" | tee -a "$SUMMARY"

exit "$RC"
