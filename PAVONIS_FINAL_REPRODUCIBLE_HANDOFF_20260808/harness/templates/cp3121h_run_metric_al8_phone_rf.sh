#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-@CONTROLLER_HARNESS@}"
BASE="$ART/cp3121h_run_filtered_phone_metrics.sh"
BASE_SHA=adabf67c007fd533f66d9558250c70e85229ffc1b43e80eb377fb0de371a8636
GNB_WRAPPER="$ART/cp3120_fallback_al8_gnb_wrapper.sh"
GNB_WRAPPER_SHA=480a20454431d93516df79e0f9b2833129f4f594077cdba2acdd036ad7239a20
REMOTE_GNB_WRAPPER=@REMOTE_HOME@/pavonis_cp3120_fallback_al8_gnb_wrapper.sh
STAMP="${STAMP:?set a fresh STAMP}"
TAG="${TAG:-cp3121_metric_al8_phone_rf}"

[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ && "$TAG" =~ ^[A-Za-z0-9_.-]+$ ]]
[[ "$(sha256sum "$BASE" | awk '{print $1}')" == "$BASE_SHA" ]]
[[ "$(sha256sum "$GNB_WRAPPER" | awk '{print $1}')" == "$GNB_WRAPPER_SHA" ]]

if [[ "${PAVONIS_CP3121_SEQUENCE_PROBE:-0}" == 1 ]]; then
  cat <<'EOF'
CP3121_BASE=CP3121_FILTERED_PHONE_METRICS
CP3121_RADIO_SHAPE=CP3120_CLEAN_AL8
CP3121_TIMING_COMPENSATION_SAMPLES=230613
CP3121_REPORT_SLOT_OFFSET=0
CP3121_RA_RESP_WINDOW=40
CP3121_PHYSICAL_CHANNEL=1
CP3121_TX_ATTENUATION_DB=16
CP3121_FULL_RX_APPEND=0
CP3121_MSG3_TARGET_CAPTURE=1
CP3121_MSG3_TARGET_MAX=8
CP3121_UL_SYMBOL_RAW_MAX=128
CP3121_CONTEXT_EDGE_TRANSPORT=1
CP3121_EDGE_LATTICE_PERIOD_SAMPLES=823
CP3121_EDGE_LATTICE_ANCHOR_SAMPLES=-138
CP3121_EDGE_LATTICE_CANONICAL_SAMPLES=-6722
CP3121_NON_PRACH_UL_RX_CFO_HZ=0
CP3121_MSG3_PATH_TRACE=1
CP3121_MSG3_PATH_TRACE_LIMIT=1024
CP3121_MSG3_CONRES_SLOT_REBASE_FROM_NON_PRACH=0
CP3121_FALLBACK_DL_DCI_AGGR_LEV=8
CP3121_HTTP_TRANSFER_BYTES=1048576
CP3121_ECHO_REQUESTS=4
CP3121_STARTER=stage6_cp3119_start_ran.sh
CP3121_SEQUENCE_PROBE=PASS
EOF
  exit 0
fi

# CLAUDE-CP3631: an earlier revision exported Soapy hash overrides here. They
# were inert: cp3121h_run_filtered_phone_metrics.sh hardcodes those values in
# the env line that launches the base runner, so anything exported here is
# discarded. The real fix was to revert the deployed Soapy source and align the
# stale build artifact on ran. The overrides are removed so this file does not
# imply an override is active.

[[ "${PAVONIS_CP3121_RF_APPROVED:-0}" == 1 ]] || {
  echo "RF gate closed: set PAVONIS_CP3121_RF_APPROVED=1" >&2
  exit 2
}

exec env \
  PAVONIS_CP3082_RF_APPROVED=1 \
  PAVONIS_OCUDU_GNB_BIN_OVERRIDE="$REMOTE_GNB_WRAPPER" \
  PAVONIS_OCUDU_GNB_SHA_EXPECTED="$GNB_WRAPPER_SHA" \
  STAMP="$STAMP" TAG="$TAG" \
  "$BASE"
