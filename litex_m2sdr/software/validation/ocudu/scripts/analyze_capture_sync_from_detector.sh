#!/usr/bin/env bash
set +e +u
set +o pipefail 2>/dev/null || true

RAW="${RAW:-${1:-}}"
FS="${FS:-11520000}"
FC="${FC:-1842500000}"
SSB="${SSB:-1839650000}"
EXPECTED_PCI="${EXPECTED_PCI:-1}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DET="${DET:-$SCRIPT_DIR/detect_nr_pss_sss_from_sc16.py}"

if [ -z "$RAW" ]; then
  echo "ERROR: RAW path required as first arg or RAW=..."
  exit 2
fi

if [ ! -s "$RAW" ]; then
  echo "ERROR: missing/non-empty RAW: $RAW"
  exit 3
fi

OUT="${OUT:-$(dirname "$RAW")/detector_sync_$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "$OUT"

LOG="$OUT/detector.log"
SUMMARY="$OUT/SYNC_SUMMARY.txt"

{
  echo "===== detector sync analysis ====="
  hostname
  date -u --iso-8601=seconds
  echo "RAW=$RAW"
  echo "FS=$FS"
  echo "FC=$FC"
  echo "SSB=$SSB"
  echo "EXPECTED_PCI=$EXPECTED_PCI"
  echo "DET=$DET"

  echo
  echo "===== raw file ====="
  ls -lh "$RAW"
  stat -c 'RAW_BYTES=%s' "$RAW"
  sha256sum "$RAW"

  echo
  echo "===== run detector ====="
  python3 "$DET" "$RAW" \
    --fs "$FS" \
    --fc "$FC" \
    --ssb "$SSB" \
    --expected-pci "$EXPECTED_PCI" \
    --max-pss-candidates 24 \
    --cfo-min -30000 \
    --cfo-max 30000 \
    --cfo-step 2500

  DET_RC=$?
  echo "DETECTOR_RC=$DET_RC"
  date -u --iso-8601=seconds
} 2>&1 | tee "$LOG"

python3 - <<'PY' "$LOG" "$SUMMARY"
import re
import sys
from pathlib import Path

log = Path(sys.argv[1]).read_text(errors="replace")
summary = Path(sys.argv[2])

segments = [
    (float(a), float(b))
    for a, b in re.findall(r"segment samples \d+:\d+ time ([0-9.]+)-([0-9.]+)s", log)
]

pss = []
for m in re.finditer(r"score=([0-9.]+) nid2=(\d+) cfo_Hz=([+-]?[0-9.]+) sample=(\d+) time_s=([0-9.]+)", log):
    pss.append({
        "score": float(m.group(1)),
        "nid2": int(m.group(2)),
        "cfo": float(m.group(3)),
        "sample": int(m.group(4)),
        "time": float(m.group(5)),
    })

expected = re.search(r"EXPECTED_PCI=(\d+): score=([0-9.]+).*?delta=(\d+), cfo_Hz=([+-]?[0-9.]+), time_s=([0-9.]+)", log)
wrong = re.search(r"BEST_WRONG_PCI=(\d+): score=([0-9.]+)", log)
ratio = re.search(r"expected_vs_best_wrong_ratio=([0-9.]+)", log)
verdict = re.search(r"VERDICT=([A-Z0-9_]+)", log)
det_rc = re.search(r"DETECTOR_RC=(\d+)", log)

tx_energy = bool(segments)
pss_seen = bool(pss)
best_pss = max(pss, key=lambda x: x["score"]) if pss else None
expected_ok = verdict and verdict.group(1) == "SSS_EXPECTED_PCI_DOMINATES"

lines = []
lines.append("===== machine-readable sync verdict =====")
lines.append(f"TX_ENERGY_DETECTED={1 if tx_energy else 0}")
lines.append(f"TX_ENERGY_SEGMENT_COUNT={len(segments)}")
if segments:
    lines.append(f"FIRST_ENERGY_START_S={segments[0][0]:.6f}")
    lines.append(f"FIRST_ENERGY_END_S={segments[0][1]:.6f}")
if best_pss:
    lines.append("PSS_DETECTED=1")
    lines.append(f"BEST_PSS_NID2={best_pss['nid2']}")
    lines.append(f"BEST_PSS_SCORE={best_pss['score']:.4f}")
    lines.append(f"BEST_PSS_TIME_S={best_pss['time']:.6f}")
    lines.append(f"BEST_PSS_CFO_HZ={best_pss['cfo']:+.1f}")
else:
    lines.append("PSS_DETECTED=0")

if expected:
    lines.append(f"EXPECTED_PCI={expected.group(1)}")
    lines.append(f"EXPECTED_PCI_SCORE={float(expected.group(2)):.4f}")
    lines.append(f"EXPECTED_PCI_TIME_S={float(expected.group(5)):.6f}")
    lines.append(f"EXPECTED_PCI_CFO_HZ={float(expected.group(4)):+.1f}")

if wrong:
    lines.append(f"BEST_WRONG_PCI={wrong.group(1)}")
    lines.append(f"BEST_WRONG_PCI_SCORE={float(wrong.group(2)):.4f}")

if ratio:
    lines.append(f"EXPECTED_VS_BEST_WRONG_RATIO={float(ratio.group(1)):.3f}")

lines.append(f"PCI1_DOMINATES={1 if expected_ok else 0}")
lines.append(f"DETECTOR_VERDICT={verdict.group(1) if verdict else 'NO_VERDICT'}")
lines.append(f"DETECTOR_RC={det_rc.group(1) if det_rc else 'UNKNOWN'}")

lines.append("")
lines.append("===== human interpretation =====")
if not tx_energy and not pss_seen:
    lines.append("CAPTURE_SYNC_STATUS=NO_TX_SEEN")
    lines.append("Meaning: this capture probably missed the active TX window, or the RF level was below detector threshold.")
elif pss_seen and not expected_ok:
    lines.append("CAPTURE_SYNC_STATUS=TX_SEEN_BUT_PCI_NOT_PROVEN")
    lines.append("Meaning: capture overlapped TX and saw PSS, but SSS/PCI did not cleanly identify expected PCI.")
    lines.append("Recommended next capture: SSB-centered, FC=1839650000, FS=5760000, BW=5600000.")
elif expected_ok:
    lines.append("CAPTURE_SYNC_STATUS=EXPECTED_CELL_PROVEN")
    lines.append("Meaning: capture overlapped TX and expected PCI dominates.")
else:
    lines.append("CAPTURE_SYNC_STATUS=AMBIGUOUS")

summary.write_text("\n".join(lines) + "\n")
print(summary.read_text())
PY

WRAP_RC=0
if grep -q 'PCI1_DOMINATES=1' "$SUMMARY"; then
  WRAP_RC=0
elif grep -q 'PSS_DETECTED=1' "$SUMMARY"; then
  WRAP_RC=20
else
  WRAP_RC=10
fi

echo
echo "OUT=$OUT"
echo "LOG=$LOG"
echo "SUMMARY=$SUMMARY"
cat "$SUMMARY"
exit "$WRAP_RC"
