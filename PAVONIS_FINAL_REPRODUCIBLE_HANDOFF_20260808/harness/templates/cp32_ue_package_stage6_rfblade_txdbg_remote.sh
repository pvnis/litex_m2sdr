#!/usr/bin/env bash
set -euo pipefail

TAG="${TAG:-codex_stage6_cp32_rfblade_txdbg}"
BASE="${SENS_BASE:-$HOME/pavonis_bladerf/ota_attach_best_${TAG}}"
TAR="${SENS_TAR:-/tmp/${TAG}_ue.tar.gz}"
RUN="$(ls -td "$BASE"/run_* 2>/dev/null | head -1 || true)"

rm -f "$TAR"
test -n "$RUN"
test -d "$RUN"

echo "RUN=$RUN"
echo "RF_TX_DBG_FILES"
find "$RUN" -maxdepth 1 -type f \( -name 'pavonis_rf_blade_tx_dbg_*.sc16' -o -name 'pavonis_rf_blade_tx_dbg_*.meta.txt' \) -print -exec ls -lh {} \;

tar -czf "$TAR" -C / "${RUN#/}"

ls -lh "$TAR"
sha256sum "$TAR"
echo "TAG=$TAG"
echo "BASE=$BASE"
echo "RUN=$RUN"
