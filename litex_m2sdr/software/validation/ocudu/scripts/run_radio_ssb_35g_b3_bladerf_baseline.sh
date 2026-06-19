#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

source ./scripts/env_ocudu_validation.sh
source ./scripts/env_m2sdr_ocudu_safe_rf.sh

RADIO_SSB="${RADIO_SSB:-$HOME/CLionProjects/ocudu/build-radio-ssb-m2sdr/apps/examples/phy/radio_ssb}"
RF="${RF:-$HOME/CLionProjects/m2sdr/litex_m2sdr/software/user/m2sdr_rf}"
DURATION_S="${DURATION_S:-25}"
BACKOFF_DB="${BACKOFF_DB:-3}"

STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="$PWD/runs/radio_ssb_35g_b${BACKOFF_DB}_offset20ms_bladerf_${STAMP}"
mkdir -p "$OUT"

STDOUT="$OUT/radio_ssb_stdout.log"
SUMMARY="$OUT/summary.log"
T0_EPOCH="$(date +%s)"

{
  echo "===== OCUDU radio_ssb 3.5 GHz bladeRF baseline ====="
  echo "Profile: m2sdr_35g_10MHz"
  echo "RF center: 3500.000 MHz"
  echo "SSB center: 3497.150 MHz"
  echo "PCI: 1"
  echo "Expected PSS NID2: 1"
  echo "Backoff: b${BACKOFF_DB}"
  echo "Duration: ${DURATION_S}s"
  echo "TX timing offset ns: ${M2SDR_SOAPY_TX_TIME_OFFSET_NS}"
  echo "OUT=$OUT"
  date -u --iso-8601=seconds

  echo
  echo "===== pre-check: no stale RF/OCUDU processes ====="
  pgrep -a -x radio_ssb || true
  pgrep -a -x gnb || true
  pgrep -a -x srsue || true
  pgrep -a -x qcore || true
  pgrep -a -x m2sdr_gen || true
  pgrep -a -x m2sdr_record || true
  pgrep -a -f 'm2sdr_loopback_latency.py' || true
  sudo fuser -v /dev/m2sdr0 2>/dev/null || true

  if pgrep -x radio_ssb >/dev/null || \
     pgrep -x gnb >/dev/null || \
     pgrep -x srsue >/dev/null || \
     pgrep -x qcore >/dev/null || \
     pgrep -x m2sdr_gen >/dev/null || \
     pgrep -x m2sdr_record >/dev/null || \
     pgrep -f 'm2sdr_loopback_latency.py' >/dev/null; then
    echo "FAIL: stale RF/OCUDU process is running."
    exit 2
  fi

  echo
  echo "===== reset timed TX counters ====="
  timeout 5 "$RF" --timed-tx-reset-counters || true
  timeout 5 "$RF" --timed-tx-counters || true

  echo
  echo "===== transmit radio_ssb ====="
  echo "START BLADE RF CAPTURE NOW"
  timeout -s INT -k 3 "$DURATION_S" "$RADIO_SSB" \
    -P m2sdr_35g_10MHz \
    -D 6000 \
    -b "$BACKOFF_DB" \
    -v info \
    > "$STDOUT" 2>&1

  RC=$?
  echo "radio_ssb_rc=$RC"

  echo
  echo "===== TX health summary ====="
  echo "SSB lines: $(grep -aci 'SSB: phys_cell_id' "$STDOUT" || true)"
  echo "late notifications: $(grep -aci 'source=tx type=late' "$STDOUT" || true)"
  echo "copy-mode warnings: $(grep -aci 'zero-copy disabled\|M2SDR_LITEPCIE_ZERO_COPY=0\|write() copy path\|read() copy path' "$STDOUT" || true)"
  grep -ai '\[TX\]' "$STDOUT" | tail -1 || true
  grep -ai '\[RX\]' "$STDOUT" | tail -1 || true

  echo
  echo "===== slip trace ====="
  grep -ai 'TX timed commit slip' "$STDOUT" | head -5 || true

  echo
  echo "===== timed TX counters after ====="
  timeout 5 "$RF" --timed-tx-counters || true

  echo
  echo "===== recent kernel messages since run start ====="
  sudo journalctl -k --since "@$T0_EPOCH" --no-pager | \
    grep -iE 'm2sdr|litepcie|dma|bad page|BUG|oops|panic|ad9361|pcie|underrun|late|dropped|blocked|hung' || true

  echo
  echo "STDOUT=$STDOUT"
  echo "SUMMARY=$SUMMARY"
} 2>&1 | tee "$SUMMARY"

echo
echo "OUT=$OUT"
echo "STDOUT=$STDOUT"
echo "SUMMARY=$SUMMARY"
