#!/usr/bin/env bash
# Quiet, bounded PBCH/MIB validator for bladeRF SC16 captures.
#
# Design goals:
#   - never stream detector / ssb_file_test logs to the terminal;
#   - respect RUN / OUT / RAW environment variables;
#   - use detector timing instead of assuming CROP_START=0;
#   - treat ssb_file_test rc=0 as insufficient: require crc=OK and expected PCI;
#   - keep failed CF32 crops out of the tree by default.

set +e +u
set +o pipefail 2>/dev/null || true

TS="$(date -u +%Y%m%dT%H%M%SZ)"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLS="${TOOLS:-$SCRIPT_DIR}"

RUN="${RUN:-${1:-}}"
RAW="${RAW:-}"
FS="${FS:-5760000}"
FC="${FC:-1839650000}"
SSB="${SSB:-${SSB_FREQ:-1839650000}}"
SSB_FREQ="${SSB_FREQ:-$SSB}"
SCS="${SCS:-15}"
SCS_KHZ="${SCS_KHZ:-$SCS}"
EXPECTED_PCI="${EXPECTED_PCI:-1}"

CROP_N="${CROP_N:-245760}"
OFFSETS="${OFFSETS:-140000 160000 180000 190000 192937 200000 210000 220000 240000}"
CFO_LIST="${CFO_LIST:-AUTO 0 -2000 2000 -5000 5000 -10000 10000 -12500 -15000}"
SSB_PATTERN_CANDIDATES="${SSB_PATTERN_CANDIDATES:-auto}"
TIMEOUT_S="${TIMEOUT_S:-25}"

KEEP_FAILURE_CROPS="${KEEP_FAILURE_CROPS:-0}"

PY="${PY:-$HOME/venvs/pavonis-iq/bin/python}"
SRSRAN_4G_REPO="${SRSRAN_4G_REPO:-$HOME/CLionProjects/srsRAN_4G}"
SSB_BIN="${SSB_BIN:-$SRSRAN_4G_REPO/build-clion/lib/src/phy/sync/test/ssb_file_test}"
DET="${DET:-$TOOLS/detect_nr_pss_sss_from_sc16.py}"

if [ -z "$RUN" ]; then
  if [ -n "$RAW" ]; then
    RUN="$(cd "$(dirname "$RAW")/.." 2>/dev/null && pwd)"
  else
    RUN="$(ls -td "$HOME"/pavonis_bladerf_local/runs/* 2>/dev/null | head -1)"
  fi
fi

if [ -z "$RAW" ]; then
  RAW="$(find "$RUN" -type f -name '*.sc16' 2>/dev/null | sort | head -1)"
fi

OUT="${OUT:-$RUN/pbch_mib_quiet_$TS}"
mkdir -p "$OUT/logs" "$OUT/work" "$OUT/winners"

RUN_LOG="$OUT/run.log"
STATUS="$OUT/status.txt"
SUMMARY="$OUT/pbch_grid_summary.tsv"
DET_LOG="$OUT/detector.log"
DET_ENV="$OUT/detector.env"
HELP_LOG="$OUT/ssb_file_test_help.txt"

print_final_summary() {
  echo "===== quiet PBCH validator summary ====="
  echo "OUT=$OUT"
  echo "STATUS=$STATUS"
  echo "SUMMARY=$SUMMARY"
  echo "DET_LOG=$DET_LOG"
  echo "RUN_LOG=$RUN_LOG"

  if [ -f "$OUT/PBCH_OK.txt" ]; then
    echo
    cat "$OUT/PBCH_OK.txt"
  else
    echo
    echo "PBCH_OK=0"
    echo "PBCH_OK_NOT_FOUND=1"
    echo
    echo "===== detector summary ====="
    grep -E 'EXPECTED_PCI=|BEST_WRONG_PCI=|expected_vs_best_wrong_ratio=|VERDICT=|score=.*nid2=.*sample=' "$DET_LOG" 2>/dev/null | tail -30 || true
    echo
    echo "===== last grid tries ====="
    tail -20 "$SUMMARY" 2>/dev/null || true
  fi
}

{
  echo "===== quiet PBCH/MIB validator ====="
  hostname
  date -u --iso-8601=seconds
  echo "RUN=$RUN"
  echo "RAW=$RAW"
  echo "OUT=$OUT"
  echo "FS=$FS"
  echo "FC=$FC"
  echo "SSB_FREQ=$SSB_FREQ"
  echo "SCS=$SCS"
  echo "SCS_KHZ=$SCS_KHZ"
  echo "EXPECTED_PCI=$EXPECTED_PCI"
  echo "CROP_N=$CROP_N"
  echo "OFFSETS=$OFFSETS"
  echo "CFO_LIST=$CFO_LIST"
  echo "SSB_PATTERN_CANDIDATES=$SSB_PATTERN_CANDIDATES"
  echo "PY=$PY"
  echo "SSB_BIN=$SSB_BIN"
  echo "DET=$DET"
} > "$RUN_LOG"

{
  echo "START_UTC=$(date -u --iso-8601=seconds)"
  echo "RUN=$RUN"
  echo "RAW=$RAW"
  echo "OUT=$OUT"
  echo "STATE=initializing"
} > "$STATUS"

FAIL=0

for P in "$RAW" "$PY" "$SSB_BIN" "$DET"; do
  if [ ! -e "$P" ]; then
    echo "ERROR: missing input: $P" | tee -a "$RUN_LOG"
    FAIL=1
  fi
done

if [ "$FAIL" -ne 0 ]; then
  echo "STATE=failed_missing_input" >> "$STATUS"
  print_final_summary
  exit 10
fi

{
  echo
  echo "===== input files ====="
  ls -lh "$RAW" "$PY" "$SSB_BIN" "$DET"
  sha256sum "$RAW"
} >> "$RUN_LOG" 2>&1

"$SSB_BIN" --help > "$HELP_LOG" 2>&1 || true

SUPPORTS_PATTERN=0
if grep -q -- '-P' "$HELP_LOG"; then
  SUPPORTS_PATTERN=1
fi

if [ "$SSB_PATTERN_CANDIDATES" = "auto" ]; then
  if [ "$SUPPORTS_PATTERN" -eq 1 ]; then
    PATTERNS="none A"
  else
    PATTERNS="none"
  fi
else
  PATTERNS="$SSB_PATTERN_CANDIDATES"
fi

{
  echo "SUPPORTS_PATTERN=$SUPPORTS_PATTERN"
  echo "PATTERNS=$PATTERNS"
} >> "$RUN_LOG"

echo "STATE=running_detector" >> "$STATUS"

"$PY" "$DET" \
  "$RAW" \
  --fs "$FS" \
  --fc "$FC" \
  --ssb "$SSB_FREQ" \
  --scs-khz "$SCS_KHZ" \
  --expected-pci "$EXPECTED_PCI" \
  > "$DET_LOG" 2>&1

DET_RC=$?
echo "DETECTOR_RC=$DET_RC" >> "$RUN_LOG"

"$PY" - <<'PY' "$DET_LOG" "$FS" "$EXPECTED_PCI" "$DET_ENV"
import re
import sys
from pathlib import Path

log = Path(sys.argv[1])
fs = float(sys.argv[2])
expected_pci = sys.argv[3]
out = Path(sys.argv[4])

txt = log.read_text(errors="replace")
lines = txt.splitlines()

best_pss_sample = None
best_pss_time = None
best_pss_cfo = None
best_pss_score = None

expected_time = None
expected_cfo = None
expected_score = None
ratio = None
verdict = None

for line in lines:
    if line.startswith("score=") and " nid2=" in line and " sample=" in line and best_pss_sample is None:
        m_sample = re.search(r"sample=([0-9]+)", line)
        m_time = re.search(r"time_s=([-+0-9.eE]+)", line)
        m_cfo = re.search(r"cfo_Hz=([-+0-9.eE]+)", line)
        m_score = re.search(r"score=([-+0-9.eE]+)", line)
        if m_sample:
            best_pss_sample = int(m_sample.group(1))
        if m_time:
            best_pss_time = float(m_time.group(1))
        if m_cfo:
            best_pss_cfo = float(m_cfo.group(1))
        if m_score:
            best_pss_score = float(m_score.group(1))

    if line.startswith(f"EXPECTED_PCI={expected_pci}:") and "time_s=" in line:
        m_time = re.search(r"time_s=([-+0-9.eE]+)", line)
        m_cfo = re.search(r"cfo_Hz=([-+0-9.eE]+)", line)
        m_score = re.search(r"score=([-+0-9.eE]+)", line)
        if m_time:
            expected_time = float(m_time.group(1))
        if m_cfo:
            expected_cfo = float(m_cfo.group(1))
        if m_score:
            expected_score = float(m_score.group(1))

    if line.startswith("expected_vs_best_wrong_ratio="):
        ratio = line.split("=", 1)[1].strip()

    if line.startswith("VERDICT="):
        verdict = line.split("=", 1)[1].strip()

if expected_time is not None:
    anchor_sample = int(round(expected_time * fs))
elif best_pss_sample is not None:
    anchor_sample = int(best_pss_sample)
else:
    anchor_sample = -1

if expected_cfo is not None:
    anchor_cfo = expected_cfo
elif best_pss_cfo is not None:
    anchor_cfo = best_pss_cfo
else:
    anchor_cfo = 0.0

with out.open("w") as f:
    f.write(f"ANCHOR_SAMPLE={anchor_sample}\n")
    f.write(f"ANCHOR_CFO={anchor_cfo:.1f}\n")
    f.write(f"BEST_PSS_SAMPLE={best_pss_sample if best_pss_sample is not None else 'NA'}\n")
    f.write(f"BEST_PSS_SCORE={best_pss_score if best_pss_score is not None else 'NA'}\n")
    f.write(f"EXPECTED_SCORE={expected_score if expected_score is not None else 'NA'}\n")
    f.write(f"EXPECTED_VS_BEST_WRONG_RATIO={ratio if ratio is not None else 'NA'}\n")
    f.write(f"DETECTOR_VERDICT={verdict if verdict is not None else 'NA'}\n")

if anchor_sample < 0:
    raise SystemExit(2)
PY

PARSE_RC=$?
echo "DETECT_PARSE_RC=$PARSE_RC" >> "$RUN_LOG"

if [ "$PARSE_RC" -ne 0 ]; then
  echo "STATE=failed_detector_parse" >> "$STATUS"
  print_final_summary
  exit 20
fi

# shellcheck disable=SC1090
. "$DET_ENV"

{
  echo
  echo "===== detector env ====="
  cat "$DET_ENV"
} >> "$RUN_LOG"

if ! grep -q 'DETECTOR_VERDICT=SSS_EXPECTED_PCI_DOMINATES\|DETECTOR_VERDICT=SSS_EXPECTED_PCI_TOP_LOW_MARGIN' "$DET_ENV"; then
  echo "WARNING: detector did not strongly prove expected PCI; continuing bounded PBCH probe anyway" >> "$RUN_LOG"
fi

printf 'try\tstart\toffset\tcfo\tpattern\tmake_rc\tssb_rc\tcrc\tpci_ok\tsearch_line\n' > "$SUMMARY"

TRIES=0
FOUND=0

echo "STATE=running_pbch_grid" >> "$STATUS"
echo "ANCHOR_SAMPLE=$ANCHOR_SAMPLE" >> "$STATUS"
echo "ANCHOR_CFO=$ANCHOR_CFO" >> "$STATUS"

for OFFSET in $OFFSETS; do
  START=$((ANCHOR_SAMPLE - OFFSET))

  for CFO in $CFO_LIST; do
    if [ "$CFO" = "AUTO" ]; then
      CFO_INT="$(printf '%.0f' "$ANCHOR_CFO" 2>/dev/null || echo 0)"
    else
      CFO_INT="$CFO"
    fi

    for PAT in $PATTERNS; do
      TRIES=$((TRIES + 1))
      LABEL="try_${TRIES}_start_${START}_offset_${OFFSET}_cfo_${CFO_INT}_pat_${PAT}"
      CF="$OUT/work/${LABEL}.cf32"
      MAKE_LOG="$OUT/logs/${LABEL}.make_crop.log"
      SSB_LOG="$OUT/logs/${LABEL}.ssb_file_test.log"

      {
        echo "TRY=$TRIES"
        echo "STATE=running_pbch_grid"
        echo "CURRENT_LABEL=$LABEL"
        echo "CURRENT_START=$START"
        echo "CURRENT_OFFSET=$OFFSET"
        echo "CURRENT_CFO=$CFO_INT"
        echo "CURRENT_PATTERN=$PAT"
      } >> "$STATUS"

      "$PY" - <<'PY' "$RAW" "$CF" "$START" "$CROP_N" "$FS" "$CFO_INT" "$ANCHOR_SAMPLE" > "$MAKE_LOG" 2>&1
import sys
from pathlib import Path
import numpy as np

raw_path = Path(sys.argv[1])
out_path = Path(sys.argv[2])
start = int(sys.argv[3])
n = int(sys.argv[4])
fs = float(sys.argv[5])
cfo = float(sys.argv[6])
anchor_sample = int(float(sys.argv[7]))

raw = np.memmap(raw_path, dtype="<i2", mode="r")
if raw.size % 2:
    raw = raw[:raw.size - 1]

iq = raw.reshape((-1, 2))
total = iq.shape[0]

if start < 0 or start + n > total:
    print(f"ERROR=crop_out_of_range start={start} n={n} total={total}")
    raise SystemExit(2)

seg = iq[start:start+n]
x = (seg[:, 0].astype(np.float32) + 1j * seg[:, 1].astype(np.float32)) / 2048.0

if cfo != 0.0:
    idx = np.arange(x.size, dtype=np.float64)
    x = x * np.exp(2j * np.pi * cfo * idx / fs).astype(np.complex64)

x.astype(np.complex64).tofile(out_path)

print(f"crop_file={out_path}")
print(f"crop_start={start}")
print(f"crop_n={n}")
print(f"crop_anchor_offset={anchor_sample - start}")
print(f"cfo_rot_hz={cfo:+.1f}")
PY
      MAKE_RC=$?

      if [ "$MAKE_RC" -ne 0 ]; then
        printf '%s\t%s\t%s\t%s\t%s\t%s\tNA\tMAKE_FAIL\t0\t%s\n' \
          "$TRIES" "$START" "$OFFSET" "$CFO_INT" "$PAT" "$MAKE_RC" \
          "$(tr '\n\t' '  ' < "$MAKE_LOG" | sed 's/[[:space:]][[:space:]]*/ /g')" >> "$SUMMARY"
        continue
      fi

      if [ "$PAT" = "none" ]; then
        timeout "$TIMEOUT_S" "$SSB_BIN" \
          -i "$CF" \
          -n "$CROP_N" \
          -r "$FS" \
          -s "$SCS" \
          -f "$FC" \
          -F "$SSB_FREQ" \
          -v \
          > "$SSB_LOG" 2>&1
      else
        timeout "$TIMEOUT_S" "$SSB_BIN" \
          -i "$CF" \
          -n "$CROP_N" \
          -r "$FS" \
          -s "$SCS" \
          -f "$FC" \
          -F "$SSB_FREQ" \
          -P "$PAT" \
          -v \
          > "$SSB_LOG" 2>&1
      fi

      SSB_RC=$?
      CRC="$(grep -m1 -o 'crc=OK\|crc=KO' "$SSB_LOG" 2>/dev/null || true)"
      [ -n "$CRC" ] || CRC="NO_CRC_LINE"

      SEARCH_LINE="$(grep -m1 'search -' "$SSB_LOG" 2>/dev/null | sed 's/[[:space:]][[:space:]]*/ /g' || true)"

      PCI_OK=0
      if echo "$SEARCH_LINE" | grep -q "pci=$EXPECTED_PCI" && [ "$CRC" = "crc=OK" ]; then
        PCI_OK=1
      fi

      printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
        "$TRIES" "$START" "$OFFSET" "$CFO_INT" "$PAT" "$MAKE_RC" "$SSB_RC" "$CRC" "$PCI_OK" "$SEARCH_LINE" >> "$SUMMARY"

      if [ "$PCI_OK" -eq 1 ]; then
        FOUND=1
        cp -av "$CF" "$OUT/winners/${LABEL}.cf32" > "$OUT/winners/${LABEL}.copy.log" 2>&1
        {
          echo "PBCH_OK=1"
          echo "BEST_LABEL=$LABEL"
          echo "BEST_TRY=$TRIES"
          echo "BEST_START=$START"
          echo "BEST_OFFSET=$OFFSET"
          echo "BEST_CFO=$CFO_INT"
          echo "BEST_PATTERN=$PAT"
          echo "BEST_CF32=$OUT/winners/${LABEL}.cf32"
          echo "BEST_LOG=$SSB_LOG"
          echo "ANCHOR_SAMPLE=$ANCHOR_SAMPLE"
          echo "ANCHOR_CFO=$ANCHOR_CFO"
          echo
          echo "===== winning ssb_file_test lines ====="
          grep -E 'measure -|search -|PBCH-MIB:|crc=OK|crc=KO' "$SSB_LOG"
        } > "$OUT/PBCH_OK.txt"
        break 3
      fi

      if [ "$KEEP_FAILURE_CROPS" -ne 1 ]; then
        rm -f "$CF" 2>/dev/null || true
      fi
    done
  done
done

{
  echo "END_UTC=$(date -u --iso-8601=seconds)"
  echo "TRIES=$TRIES"
  echo "FOUND=$FOUND"
  if [ "$FOUND" -eq 1 ]; then
    echo "STATE=pbch_ok"
  else
    echo "STATE=pbch_not_found"
  fi
} >> "$STATUS"

if [ "$FOUND" -eq 1 ]; then
  print_final_summary
  exit 0
fi

print_final_summary
exit 1
