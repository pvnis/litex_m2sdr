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

cd /home/stefan/CLionProjects/m2sdr/litex_m2sdr/software/validation/ocudu || exit 1

source ./scripts/env_m2sdr_copy_dma.sh
source ./scripts/env_ocudu_validation.sh

set +e
set +u
set +o pipefail

RADIO_SSB="$HOME/CLionProjects/ocudu/build-radio-ssb-m2sdr/apps/examples/phy/radio_ssb"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
ROOT="$PWD/runs/radio_ssb_35g_backoff_timing_${STAMP}"
mkdir -p "$ROOT"

RESULTS="$ROOT/results.txt"
: > "$RESULTS"

echo "ROOT=$ROOT"
echo "RADIO_SSB=$RADIO_SSB"

for B in 12 9 6 3 0; do
  echo
  echo "===== BACKOFF_DB=$B ====="

  BASE="$ROOT/backoff_${B}"
  mkdir -p "$BASE"
  STDOUT="$BASE/radio_ssb_stdout.log"

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
    timeout 25 "$RADIO_SSB" \
      -P m2sdr_35g_10MHz \
      -D 10000 \
      -b "$B" \
      -v info \
    > "$STDOUT" 2>&1

  RC=$?
  SSB="$(grep -aci 'SSB: phys_cell_id' "$STDOUT" || true)"
  LATE="$(grep -aci 'source=tx type=late' "$STDOUT" || true)"
  UNDER="$(grep -aci 'underflow' "$STDOUT" || true)"
  BAD="$(grep -aciE 'BUG:|Bad page|kernel BUG|oops|panic' "$STDOUT" || true)"
  FINAL_TX="$(grep -ai '\[TX\]' "$STDOUT" | tail -1 || true)"
  FINAL_RX="$(grep -ai '\[RX\]' "$STDOUT" | tail -1 || true)"

  printf "backoff=%2s rc=%3s ssb=%5s late=%5s under=%3s bad=%3s tx='%s' rx='%s'\n" \
    "$B" "$RC" "$SSB" "$LATE" "$UNDER" "$BAD" "$FINAL_TX" "$FINAL_RX" | tee -a "$RESULTS"
done

echo
echo "===== BACKOFF TIMING SUMMARY ====="
cat "$RESULTS"

echo
echo "ROOT=$ROOT"
