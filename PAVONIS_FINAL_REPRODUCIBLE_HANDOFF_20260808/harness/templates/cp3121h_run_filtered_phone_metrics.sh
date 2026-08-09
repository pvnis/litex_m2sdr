#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-@CONTROLLER_HARNESS@}"
SSH="$ART/ssh_pexpect_run.py"
SCP_GET="$ART/scp_pexpect_get.py"
BASE="$ART/cp3121h_run_continuous_srsenb_to_ocudu_metrics_rf.sh"
BASE_SHA256=ac784e995d2baa6e03cf71c59648cfea0c764223437d7370e7df0976caa41884
DIAG="$ART/cp3061_qcsuper_filtered_remote.sh"
DIAG_SHA256=ebaef596d9a6dc06bd26280f86a05edce43440c07fdf2b69250245df1d342f95
ENTRY="$ART/cp3061_qcsuper_filtered_entry.py"
ENTRY_SHA256=efe1c8031e297b5bdfd143a1b2241115837ff3c3658aebe4d355718575bc3b38
PARSER="$ART/cp3061_count_diag_dlf.py"
PARSER_SHA256=f275a3d0dee1e1138df9280bb86a27e0fa3fc22d7ce7f0a930c29630f3ddd55c
DIAG_REMOTE=@REMOTE_HOME@/pavonis_cp3061_qcsuper_filtered_remote.sh
ENTRY_REMOTE=@REMOTE_HOME@/pavonis_cp3061_qcsuper_filtered_entry.py
STAMP="${STAMP:?set a fresh STAMP}"
TAG="${TAG:-cp3121_filtered_phone_metrics}"
SERIAL="${PAVONIS_PHONE_ADB_SERIAL:?set the authorized lab phone ADB serial}"
PRACH_TIMING_COMPENSATION_SAMPLES=230613
PRACH_REPORT_SLOT_OFFSET=0
RA_RESP_WINDOW=40
MSG3_TARGET_CAPTURE=1
MSG3_TARGET_MAX=8
UL_SYMBOL_RAW_MAX=128
OCUDU_GNB_BIN_OVERRIDE="${PAVONIS_OCUDU_GNB_BIN_OVERRIDE:-@REMOTE_HOME@/pavonis_cp3025_stage1_energy_capture_gnb/gnb}"
OCUDU_GNB_SHA_EXPECTED="${PAVONIS_OCUDU_GNB_SHA_EXPECTED:-dbc7cb7905e51b58a9621e9e6fa93fab81e1286938eb7bd543517c91cc7060b9}"

[[ "$OCUDU_GNB_BIN_OVERRIDE" =~ ^@REMOTE_HOME@/[A-Za-z0-9_./-]+$ ]]
[[ "$OCUDU_GNB_SHA_EXPECTED" =~ ^[0-9a-f]{64}$ ]]

if [[ "${PAVONIS_CP3089_GNB_OVERRIDE_PROBE:-0}" == 1 ]]; then
  echo "CP3089_GNB_BIN_OVERRIDE=$OCUDU_GNB_BIN_OVERRIDE"
  echo "CP3089_GNB_SHA_EXPECTED=$OCUDU_GNB_SHA_EXPECTED"
  echo CP3089_GNB_OVERRIDE_PROBE=PASS
  exit 0
fi

if [[ "${PAVONIS_CP3073_SEQUENCE_PROBE:-0}" == 1 ]]; then
  cat <<'EOF'
CP3062_SEQUENCE=filtered_diag_start,local_M2_srsENB_primer_positive_control,continuous_OCUDU_M2_channel1_ATT16_target,target_activation_and_HTTP_if_available,diag_status_stop_fetch,cleanup
CP3062_DIAG_CODES=b821,b889,b88a
CP3062_PHONE_REBOOT=0
CP3062_PHYSICAL_CHANNEL=1
CP3062_TX_ATTENUATION_DB=16
CP3062_PRACH_COLLECTION_PHASE_MODE=off
CP3062_PRACH_EARLY_PREDEMOD_CAPTURE_MAX=0
CP3073_PRACH_TIMING_COMPENSATION_SAMPLES=230613
CP3073_PRACH_REPORT_SLOT_OFFSET=0
CP3073_RA_RESP_WINDOW=40
CP3080_ALLOW_EXTENDED_RA_RESP_WINDOW=1
CP3073_FULL_RX_APPEND=0
CP3082_MSG3_TARGET_CAPTURE=1
CP3082_MSG3_TARGET_MAX=8
CP3082_UL_SYMBOL_RAW_MAX=128
CP3082_UL_SYMBOL_METADATA_ONLY=1
CP3062_OCUDU_GNB_SHA=dbc7cb7905e51b58a9621e9e6fa93fab81e1286938eb7bd543517c91cc7060b9
CP3062_SEQUENCE_PROBE=PASS
EOF
  exit 0
fi

[[ "${PAVONIS_CP3082_RF_APPROVED:-0}" == 1 ]] || {
  echo "RF gate closed: set PAVONIS_CP3082_RF_APPROVED=1" >&2
  exit 2
}
[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ && "$TAG" =~ ^[A-Za-z0-9_.-]+$ ]]
[[ "$SERIAL" =~ ^[A-Za-z0-9._:-]+$ ]]
[[ "$(sha256sum "$BASE" | cut -d ' ' -f1)" == "$BASE_SHA256" ]]
[[ "$(sha256sum "$DIAG" | cut -d ' ' -f1)" == "$DIAG_SHA256" ]]
[[ "$(sha256sum "$ENTRY" | cut -d ' ' -f1)" == "$ENTRY_SHA256" ]]
[[ "$(sha256sum "$PARSER" | cut -d ' ' -f1)" == "$PARSER_SHA256" ]]

prefix="$ART/cp3082_${STAMP}"
PREFLIGHT="${prefix}_preflight.log"
START_LOG="${prefix}_diag_start.log"
BASE_LOG="${prefix}_base.log"
STATUS_LOG="${prefix}_diag_status.log"
STOP_LOG="${prefix}_diag_stop.log"
GET_DLF_LOG="${prefix}_get_dlf.log"
GET_STDOUT_LOG="${prefix}_get_diag_stdout.log"
DLF="${prefix}_phone_qcsuper_filtered.dlf"
DIAG_STDOUT="${prefix}_phone_qcsuper_filtered.stdout.log"
DIAG_SUMMARY="${prefix}_phone_qcsuper_filtered_summary.json"
SUMMARY="${prefix}_summary.txt"
CAPTURE_GET_LOG="${prefix}_get_msg3_target_capture.log"
CAPTURE_TAR="${prefix}_msg3_target_capture.tar.gz"
for path in "$PREFLIGHT" "$START_LOG" "$BASE_LOG" "$STATUS_LOG" "$STOP_LOG" \
  "$GET_DLF_LOG" "$GET_STDOUT_LOG" "$DLF" "$DIAG_STDOUT" "$DIAG_SUMMARY" "$SUMMARY" \
  "$CAPTURE_GET_LOG" "$CAPTURE_TAR"; do
  [[ ! -e "$path" ]] || { echo "Fresh-stamp guard: $path exists" >&2; exit 2; }
done

diag_started=0
cleanup() {
  local rc=$?
  trap - EXIT INT TERM HUP
  set +e
  if [[ "$diag_started" == 1 ]]; then
    python3 "$SSH" --host ue --out "$STOP_LOG" --timeout 90 -- \
      "$DIAG_REMOTE" stop "$STAMP" "$SERIAL"
  fi
  exit "$rc"
}
trap cleanup EXIT INT TERM HUP

python3 "$SSH" --host ue --out "$PREFLIGHT" --timeout 90 -- bash -lc \
  "set -euo pipefail; test -z \"\$(pgrep -f '[/]qcsuper' || true)\"; test \"\$(/usr/bin/adb -s '$SERIAL' get-state)\" = device; test \"\$(/usr/bin/adb -s '$SERIAL' shell su -c getprop\\ sys.usb.config | tr -d '\\r')\" = adb; test \"\$(sha256sum '$DIAG_REMOTE' | cut -d ' ' -f1)\" = '$DIAG_SHA256'; test \"\$(sha256sum '$ENTRY_REMOTE' | cut -d ' ' -f1)\" = '$ENTRY_SHA256'; echo CP3062_PREFLIGHT=PASS"

python3 "$SSH" --host ue --out "$START_LOG" --timeout 90 -- \
  "$DIAG_REMOTE" start "$STAMP" "$SERIAL"
grep -q '^CP3061_QCSUPER_FILTERED_START=PASS' "$START_LOG"
diag_started=1

set +e
env STAMP="$STAMP" TAG="$TAG" \
  PAVONIS_PHONE_ADB_SERIAL="$SERIAL" \
  PAVONIS_CP2893_RF_APPROVED=1 \
  PAVONIS_CP3020_LOCAL_M2_PRIMER=1 \
  PAVONIS_CP2877_SRS_MCS4=1 \
  PAVONIS_QCORE_RAN_INTERFACE_NAME=lo \
  PAVONIS_SOAPY_SOURCE_SHA_EXPECTED=644e7858ee88d92791a495b7f4a24d762c9e819afb5337c0ef20d02e65ca4e76 \
  PAVONIS_SOAPY_MODULE_SHA_EXPECTED=81b8cb2e079ed050a0056768cc349b0aaf31b25ae145eca359bd24934f4ff97c \
  PAVONIS_OCUDU_GNB_BIN_OVERRIDE="$OCUDU_GNB_BIN_OVERRIDE" \
  PAVONIS_OCUDU_GNB_SHA_EXPECTED="$OCUDU_GNB_SHA_EXPECTED" \
  PAVONIS_SOAPY_PHYSICAL_CHANNEL_OFFSET=1 \
  PAVONIS_PRACH_COLLECTION_PHASE_MODE=off \
  PAVONIS_PRACH_EARLY_PREDEMOD_CAPTURE_MAX=0 \
  PAVONIS_PRACH_TIMING_COMPENSATION_SAMPLES="$PRACH_TIMING_COMPENSATION_SAMPLES" \
  PAVONIS_PRACH_TIMING_COMPENSATION_REPORT_SLOT_OFFSET="$PRACH_REPORT_SLOT_OFFSET" \
  PAVONIS_GNB_RA_RESP_WINDOW="$RA_RESP_WINDOW" \
  PAVONIS_UL_SYMBOL_MSG3_TARGET_CAPTURE="$MSG3_TARGET_CAPTURE" \
  PAVONIS_UL_SYMBOL_MSG3_TARGET_MAX="$MSG3_TARGET_MAX" \
  PAVONIS_UL_SYMBOL_RAW_MAX="$UL_SYMBOL_RAW_MAX" \
  PAVONIS_CP3030_POSTSTART_PHONE_REBOOT=0 \
  PAVONIS_CP2907_POSTSTART_AIRPLANE_CYCLE=0 \
  PAVONIS_CP2904_POSTSTART_RESELECT=0 \
  PAVONIS_CP2901_SERVICE_TRIGGER=0 \
  PAVONIS_CP2920_POSTAIRPLANE_N3_REAPPLY=0 \
  PAVONIS_CP2895_PHONE_N3_PREAPPLY=1 \
  PAVONIS_CP2883_TARGET_ACTIVATION_EVENT=1 \
  PAVONIS_CP2870_DATA_PROBES=1 \
  PAVONIS_CP2887_HTTP_DATA_PROBE=1 \
  PAVONIS_CP3121_HTTP_TRANSFER_BYTES=1048576 \
  PAVONIS_CP2904_TARGET_EVENT_TIMEOUT_SEC=180 \
  PAVONIS_CP2857_SWAP_OBSERVE_SEC=60 \
  "$BASE" >"$BASE_LOG" 2>&1
base_rc=$?
set -e

capture_get_rc=0
set +e
python3 "$SCP_GET" --host ran \
  --remote "@REMOTE_HOME@/pavonis_cp3059_m2sdr_stage_force_att16_runs/${STAMP}_OCUDU/msg3_target_capture.tar.gz" \
  --local "$CAPTURE_TAR" --out "$CAPTURE_GET_LOG" --timeout 180
capture_get_rc=$?
set -e

diag_rc=0
python3 "$SSH" --host ue --out "$STATUS_LOG" --timeout 60 -- \
  "$DIAG_REMOTE" status "$STAMP" "$SERIAL" || diag_rc=$?
python3 "$SSH" --host ue --out "$STOP_LOG" --timeout 90 -- \
  "$DIAG_REMOTE" stop "$STAMP" "$SERIAL" || diag_rc=$?
diag_started=0

python3 "$SCP_GET" --host ue \
  --remote "@REMOTE_HOME@/pavonis_cp2629_runs/$STAMP/phone_qcsuper_filtered.dlf" \
  --local "$DLF" --out "$GET_DLF_LOG" --timeout 120
python3 "$SCP_GET" --host ue \
  --remote "@REMOTE_HOME@/pavonis_cp2629_runs/$STAMP/phone_qcsuper_filtered.stdout.log" \
  --local "$DIAG_STDOUT" --out "$GET_STDOUT_LOG" --timeout 120
python3 "$PARSER" "$DLF" >"$DIAG_SUMMARY" || diag_rc=$?

{
  echo "STAMP=$STAMP"
  echo "TAG=$TAG"
  echo "BASE_SHA256=$BASE_SHA256"
  echo "DIAG_SHA256=$DIAG_SHA256"
  echo "ENTRY_SHA256=$ENTRY_SHA256"
  echo "PARSER_SHA256=$PARSER_SHA256"
  echo "PHYSICAL_CHANNEL=1"
  echo "TX_ATTENUATION_DB=16"
  echo "POSTSTART_PHONE_REBOOT=0"
  echo "DIAG_CODES=b821,b889,b88a"
  echo "PRACH_TIMING_COMPENSATION_SAMPLES=$PRACH_TIMING_COMPENSATION_SAMPLES"
  echo "PRACH_REPORT_SLOT_OFFSET=$PRACH_REPORT_SLOT_OFFSET"
  echo "RA_RESP_WINDOW=$RA_RESP_WINDOW"
  echo "FULL_RX_APPEND=0"
  echo "MSG3_TARGET_CAPTURE=$MSG3_TARGET_CAPTURE"
  echo "MSG3_TARGET_MAX=$MSG3_TARGET_MAX"
  echo "UL_SYMBOL_RAW_MAX=$UL_SYMBOL_RAW_MAX"
  echo "HTTP_TRANSFER_BYTES=1048576"
  echo "BASE_RC=$base_rc"
  echo "CAPTURE_GET_RC=$capture_get_rc"
  echo "DIAG_RC=$diag_rc"
  echo "DLF_BYTES=$(stat -c %s "$DLF")"
  grep -E '^(CP3061_QCSUPER_FILTERED_|CP3062_PREFLIGHT=)' \
    "$PREFLIGHT" "$START_LOG" "$STATUS_LOG" "$STOP_LOG" 2>/dev/null || true
  cat "$DIAG_SUMMARY"
  sha256sum "$DLF" "$DIAG_STDOUT" "$DIAG_SUMMARY"
  if [[ -s "$CAPTURE_TAR" ]]; then sha256sum "$CAPTURE_TAR"; fi
} >"$SUMMARY"

trap - EXIT INT TERM HUP
cat "$SUMMARY"
if ((base_rc != 0)); then
  exit "$base_rc"
fi
if ((capture_get_rc != 0)); then
  exit "$capture_get_rc"
fi
exit "$diag_rc"
