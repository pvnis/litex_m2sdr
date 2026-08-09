#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-@CONTROLLER_HARNESS@}"
SSH="$ART/ssh_pexpect_run.py"
SCP_GET="$ART/scp_pexpect_get.py"
STAMP="${STAMP:?set a fresh STAMP}"
HOLD_BEFORE_TEARDOWN="${PAVONIS_HOLD_BEFORE_TEARDOWN:-0}"
HOLD_MAX_SEC="${PAVONIS_HOLD_MAX_SEC:-3600}"
SERIAL="${PAVONIS_PHONE_ADB_SERIAL:?phone serial must be discovered by the executor}"
SUB_ID="${PAVONIS_PHONE_SUB_ID:-6}"
SLOT_ID="${PAVONIS_PHONE_SLOT_ID:-0}"
SWAP_OBSERVE_SEC="${PAVONIS_CP2857_SWAP_OBSERVE_SEC:-150}"
IDENTITY_AUDIT_ENABLE="${PAVONIS_CP2863_IDENTITY_AUDIT:-0}"
DATA_PROBES_ENABLE="${PAVONIS_CP2870_DATA_PROBES:-0}"
SERVICE_WAIT_SEC="${PAVONIS_CP2874_SRS_SERVICE_WAIT_SEC:-180}"
SRS_MCS4_ENABLE="${PAVONIS_CP2877_SRS_MCS4:-0}"
TARGET_ACTIVATION_EVENT_ENABLE="${PAVONIS_CP2883_TARGET_ACTIVATION_EVENT:-0}"
HTTP_DATA_PROBE_ENABLE="${PAVONIS_CP2887_HTTP_DATA_PROBE:-0}"
HTTP_DATA_PORT="${PAVONIS_CP2887_HTTP_DATA_PORT:-39087}"
HTTP_TRANSFER_BYTES="${PAVONIS_CP3121_HTTP_TRANSFER_BYTES:-1048576}"
PHONE_N3_PREAPPLY="${PAVONIS_CP2895_PHONE_N3_PREAPPLY:-1}"
TARGET_PCI="${PAVONIS_CP2898_TARGET_PCI:-500}"
SERVICE_TRIGGER_ENABLE="${PAVONIS_CP2901_SERVICE_TRIGGER:-0}"
SERVICE_TRIGGER_DURATION_SEC="${PAVONIS_CP2901_SERVICE_TRIGGER_DURATION_SEC:-30}"
POSTSTART_RESELECT_ENABLE="${PAVONIS_CP2904_POSTSTART_RESELECT:-0}"
POSTSTART_RESELECT_OBSERVE_SEC="${PAVONIS_CP2904_POSTSTART_RESELECT_OBSERVE_SEC:-30}"
POSTSTART_AIRPLANE_ENABLE="${PAVONIS_CP2907_POSTSTART_AIRPLANE_CYCLE:-0}"
AIRPLANE_HOLD_SEC="${PAVONIS_CP2907_AIRPLANE_HOLD_SEC:-5}"
AIRPLANE_SETTLE_SEC="${PAVONIS_CP2907_AIRPLANE_SETTLE_SEC:-15}"
POSTAIRPLANE_N3_REAPPLY="${PAVONIS_CP2920_POSTAIRPLANE_N3_REAPPLY:-0}"
POSTSTART_PHONE_REBOOT="${PAVONIS_CP3030_POSTSTART_PHONE_REBOOT:-0}"
TARGET_EVENT_TIMEOUT_SEC="${PAVONIS_CP2904_TARGET_EVENT_TIMEOUT_SEC:-0}"
LOCAL_M2_PRIMER="${PAVONIS_CP3020_LOCAL_M2_PRIMER:-0}"
QCORE_RAN_INTERFACE_NAME="${PAVONIS_QCORE_RAN_INTERFACE_NAME:-@RAN_INTERFACE@}"
SOAPY_SOURCE_SHA_EXPECTED="${PAVONIS_SOAPY_SOURCE_SHA_EXPECTED:-b5131aabe12da43a3e220956b0c8c85ebb32c35bf5f39cbfc364025491eefd89}"
SOAPY_MODULE_SHA_EXPECTED="${PAVONIS_SOAPY_MODULE_SHA_EXPECTED:-b784c400abdb0ae5c3fa9a0c2f09525d9a09bca4f462b138f469c7118f44520e}"
PRACH_COLLECTION_PHASE_MODE="${PAVONIS_PRACH_COLLECTION_PHASE_MODE:-default}"
PRACH_EARLY_CAPTURE_MAX="${PAVONIS_PRACH_EARLY_PREDEMOD_CAPTURE_MAX:-0}"
PRACH_TIMING_COMPENSATION_SAMPLES="${PAVONIS_PRACH_TIMING_COMPENSATION_SAMPLES:-0}"
PRACH_REPORT_SLOT_OFFSET="${PAVONIS_PRACH_TIMING_COMPENSATION_REPORT_SLOT_OFFSET:-0}"
RA_RESP_WINDOW="${PAVONIS_GNB_RA_RESP_WINDOW:-10}"
MSG3_TARGET_CAPTURE="${PAVONIS_UL_SYMBOL_MSG3_TARGET_CAPTURE:-0}"
MSG3_TARGET_MAX="${PAVONIS_UL_SYMBOL_MSG3_TARGET_MAX:-8}"
UL_SYMBOL_RAW_MAX="${PAVONIS_UL_SYMBOL_RAW_MAX:-0}"
OCUDU_GNB_BIN_OVERRIDE="${PAVONIS_OCUDU_GNB_BIN_OVERRIDE:-@REMOTE_HOME@/pavonis_cp3025_stage1_energy_capture_gnb/gnb}"
OCUDU_GNB_SHA_EXPECTED="${PAVONIS_OCUDU_GNB_SHA_EXPECTED:-dbc7cb7905e51b58a9621e9e6fa93fab81e1286938eb7bd543517c91cc7060b9}"
OCUDU_GNB_LIBRARY_PATH_OVERRIDE="${PAVONIS_OCUDU_GNB_LIBRARY_PATH_OVERRIDE:-}"
PHYSICAL_CHANNEL_OFFSET="${PAVONIS_SOAPY_PHYSICAL_CHANNEL_OFFSET:-}"
PHYSICAL_CHANNEL_LABEL="${PHYSICAL_CHANNEL_OFFSET:-0}"

[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ ]]
[[ "$SERIAL" =~ ^[A-Za-z0-9._:-]+$ ]]
[[ "$SUB_ID" =~ ^[0-9]+$ && "$SLOT_ID" =~ ^[0-9]+$ ]]
[[ "$HOLD_BEFORE_TEARDOWN" =~ ^[01]$ ]]
[[ "$HOLD_MAX_SEC" =~ ^[0-9]+$ ]]
(( HOLD_MAX_SEC >= 60 && HOLD_MAX_SEC <= 21600 ))
[[ "$SWAP_OBSERVE_SEC" =~ ^[0-9]+$ ]]
[[ "$IDENTITY_AUDIT_ENABLE" =~ ^[01]$ ]]
[[ "$DATA_PROBES_ENABLE" =~ ^[01]$ ]]
[[ "$SERVICE_WAIT_SEC" =~ ^[0-9]+$ ]]
[[ "$SRS_MCS4_ENABLE" =~ ^[01]$ ]]
[[ "$TARGET_ACTIVATION_EVENT_ENABLE" =~ ^[01]$ ]]
[[ "$HTTP_DATA_PROBE_ENABLE" =~ ^[01]$ && "$HTTP_DATA_PORT" =~ ^[0-9]+$ ]]
[[ "$HTTP_TRANSFER_BYTES" =~ ^[0-9]+$ ]]
((HTTP_TRANSFER_BYTES >= 65536 && HTTP_TRANSFER_BYTES <= 8388608 && HTTP_TRANSFER_BYTES % 65536 == 0))
[[ "$PHONE_N3_PREAPPLY" =~ ^[01]$ ]]
[[ "$TARGET_PCI" =~ ^[0-9]+$ ]] && ((TARGET_PCI <= 1007))
[[ "$SERVICE_TRIGGER_ENABLE" =~ ^[01]$ ]]
[[ "$SERVICE_TRIGGER_DURATION_SEC" =~ ^[0-9]+$ ]]
[[ "$POSTSTART_RESELECT_ENABLE" =~ ^[01]$ ]]
[[ "$POSTSTART_RESELECT_OBSERVE_SEC" =~ ^[0-9]+$ ]]
[[ "$POSTSTART_AIRPLANE_ENABLE" =~ ^[01]$ ]]
[[ "$AIRPLANE_HOLD_SEC" =~ ^[0-9]+$ ]]
[[ "$AIRPLANE_SETTLE_SEC" =~ ^[0-9]+$ ]]
[[ "$POSTAIRPLANE_N3_REAPPLY" =~ ^[01]$ ]]
[[ "$POSTSTART_PHONE_REBOOT" =~ ^[01]$ ]]
[[ "$TARGET_EVENT_TIMEOUT_SEC" =~ ^[0-9]+$ ]]
[[ "$LOCAL_M2_PRIMER" =~ ^[01]$ ]]
[[ "$QCORE_RAN_INTERFACE_NAME" == @RAN_INTERFACE@ || "$QCORE_RAN_INTERFACE_NAME" == lo ]]
[[ "$SOAPY_SOURCE_SHA_EXPECTED" =~ ^[0-9a-f]{64}$ && "$SOAPY_MODULE_SHA_EXPECTED" =~ ^[0-9a-f]{64}$ ]]
[[ "$PRACH_COLLECTION_PHASE_MODE" == default || "$PRACH_COLLECTION_PHASE_MODE" == off ]]
[[ "$PRACH_EARLY_CAPTURE_MAX" =~ ^[0-9]+$ ]] && ((PRACH_EARLY_CAPTURE_MAX <= 128))
[[ "$PRACH_TIMING_COMPENSATION_SAMPLES" =~ ^[0-9]+$ ]] && ((PRACH_TIMING_COMPENSATION_SAMPLES <= 1000000))
[[ "$PRACH_REPORT_SLOT_OFFSET" =~ ^[0-9]+$ ]] && ((PRACH_REPORT_SLOT_OFFSET <= 100))
[[ "$RA_RESP_WINDOW" =~ ^[0-9]+$ ]] && ((RA_RESP_WINDOW >= 1 && RA_RESP_WINDOW <= 80))
[[ "$MSG3_TARGET_CAPTURE" =~ ^[01]$ ]]
[[ "$MSG3_TARGET_MAX" =~ ^[0-9]+$ ]] && ((MSG3_TARGET_MAX >= 1 && MSG3_TARGET_MAX <= 8))
[[ "$UL_SYMBOL_RAW_MAX" =~ ^[0-9]+$ ]] && ((UL_SYMBOL_RAW_MAX <= 128))
if [[ "$MSG3_TARGET_CAPTURE" == 1 ]]; then
  ((UL_SYMBOL_RAW_MAX >= 14))
else
  ((UL_SYMBOL_RAW_MAX == 0))
fi
[[ "$OCUDU_GNB_BIN_OVERRIDE" =~ ^@REMOTE_HOME@/[A-Za-z0-9_./-]+$ && "$OCUDU_GNB_SHA_EXPECTED" =~ ^[0-9a-f]{64}$ ]]
[[ -z "$OCUDU_GNB_LIBRARY_PATH_OVERRIDE" || "$OCUDU_GNB_LIBRARY_PATH_OVERRIDE" =~ ^@REMOTE_HOME@/[A-Za-z0-9_./-]+$ ]]
[[ -z "$PHYSICAL_CHANNEL_OFFSET" || "$PHYSICAL_CHANNEL_OFFSET" == 1 ]]
ALLOW_EXTENDED_RA_RESP_WINDOW=0
if ((RA_RESP_WINDOW > 10)); then
  ALLOW_EXTENDED_RA_RESP_WINDOW=1
fi
((SWAP_OBSERVE_SEC >= 60 && SWAP_OBSERVE_SEC <= 240))
((SERVICE_WAIT_SEC >= 180 && SERVICE_WAIT_SEC <= 420))
((HTTP_DATA_PORT >= 1024 && HTTP_DATA_PORT <= 65535))
((SERVICE_TRIGGER_DURATION_SEC >= 10 && SERVICE_TRIGGER_DURATION_SEC <= 60))
((POSTSTART_RESELECT_OBSERVE_SEC >= 30 && POSTSTART_RESELECT_OBSERVE_SEC <= 60))
((AIRPLANE_HOLD_SEC >= 3 && AIRPLANE_HOLD_SEC <= 15))
((AIRPLANE_SETTLE_SEC >= 10 && AIRPLANE_SETTLE_SEC <= 30))
((SERVICE_TRIGGER_ENABLE + POSTSTART_RESELECT_ENABLE + POSTSTART_AIRPLANE_ENABLE + POSTSTART_PHONE_REBOOT <= 1))
if [[ "$POSTAIRPLANE_N3_REAPPLY" == 1 && \
      ("$POSTSTART_AIRPLANE_ENABLE" != 1 || "$PHONE_N3_PREAPPLY" != 1) ]]; then
  echo "Post-airplane N3 reapply requires airplane cycle and preapply ownership" >&2
  exit 64
fi
if ((TARGET_EVENT_TIMEOUT_SEC == 0)); then
  if [[ "$POSTSTART_RESELECT_ENABLE" == 1 || "$POSTSTART_AIRPLANE_ENABLE" == 1 || "$POSTSTART_PHONE_REBOOT" == 1 ]]; then
    TARGET_EVENT_TIMEOUT_SEC=240
  else
    TARGET_EVENT_TIMEOUT_SEC=90
  fi
fi
((TARGET_EVENT_TIMEOUT_SEC >= 60 && TARGET_EVENT_TIMEOUT_SEC <= 300))
SERVICE_WAIT_SSH_TIMEOUT=$((SERVICE_WAIT_SEC + 120))
if [[ "$TARGET_ACTIVATION_EVENT_ENABLE" == 1 && "$DATA_PROBES_ENABLE" != 1 ]]; then
  echo "Target activation event requires data probes" >&2
  exit 64
fi
if [[ "$HTTP_DATA_PROBE_ENABLE" == 1 && "$TARGET_ACTIVATION_EVENT_ENABLE" != 1 ]]; then
  echo "HTTP data probe requires target activation event" >&2
  exit 64
fi
if [[ "$LOCAL_M2_PRIMER" == 1 &&
      ("$SRS_MCS4_ENABLE" != 1 || "$QCORE_RAN_INTERFACE_NAME" != lo) ]]; then
  echo "Local M2 primer requires the MCS4 fixture and qcore RAN interface lo" >&2
  exit 64
fi

if [[ "${PAVONIS_CP2893_SEQUENCE_PROBE:-0}" == 1 ]]; then
  cat <<'EOF'
CP2893_SEQUENCE=qcore_start,srsenb_mcs4_start,phone_prepare,select_background,wait_qcore_registering,stop_select_restore_mode,require_srs_service,srsenb_stop,prearm_target_activation_and_http,optional_phone_n3_preapply,ocudu_m2sdr_start,one_optional_poststart_action(service_trigger_or_operator_reselect_or_airplane_cycle_or_phone_reboot),event_data_probes,fixed_observe,ocudu_m2sdr_status,optional_phone_n3_restore,cleanup
CP2893_CONTEXT_INVARIANT=qcore_and_phone_remain_live_across_ran_swap
CP2893_TARGET_FIXTURE=cp2819_coherent_ssbplus6_cp2844_n3_ab
CP3120_OCUDU_HELPER=pavonis_cp3120_ocudu_m2sdr_timing230613_ra40_msg3_target_remote.sh
CP3059_STAGE=pavonis_cp3059_m2sdr_force_att16_stage_remote.sh
CP3059_TX_ATTENUATION_DB=16
CP3059_FORCE_ATT_ENV=PAVONIS_SOAPY_TX_FORCE_ATT_DB
CP3020_LOCAL_M2_PRIMER_DEFAULT=0
CP2893_NO_HOT_LOG_POLL=1
CP2893_SEQUENCE_PROBE=PASS
EOF
  exit 0
fi

if [[ "${PAVONIS_CP2874_WAIT_PROBE:-0}" == 1 ]]; then
  echo "CP2874_SERVICE_WAIT_SEC=$SERVICE_WAIT_SEC"
  echo "CP2874_SERVICE_WAIT_SSH_TIMEOUT=$SERVICE_WAIT_SSH_TIMEOUT"
  echo CP2874_WAIT_PROBE=PASS
  exit 0
fi

if [[ "${PAVONIS_CP2877_SRS_PROBE:-0}" == 1 ]]; then
  echo "CP2877_SRS_MCS4_ENABLE=$SRS_MCS4_ENABLE"
  if [[ "$SRS_MCS4_ENABLE" == 1 ]]; then
    echo CP2877_SRS_REMOTE=@REMOTE_HOME@/pavonis_cp2877_srsenb_mcs4_remote.sh
  else
    echo CP2877_SRS_REMOTE=@REMOTE_HOME@/pavonis_cp2656_srsenb_remote.sh
  fi
  echo CP2877_SRS_PROBE=PASS
  exit 0
fi

if [[ "${PAVONIS_CP2883_EVENT_PROBE:-0}" == 1 ]]; then
  echo "CP2883_TARGET_ACTIVATION_EVENT_ENABLE=$TARGET_ACTIVATION_EVENT_ENABLE"
  echo CP2883_EVENT_SOURCE=qcore_incremental_activation_line
  echo CP2883_EVENT_ORDER=prearm_before_ocudu_then_probe_before_fixed_observe
  echo CP2883_EVENT_PROBE=PASS
  exit 0
fi

if [[ "${PAVONIS_CP2887_HTTP_PROBE:-0}" == 1 ]]; then
  echo "CP2887_HTTP_DATA_PROBE_ENABLE=$HTTP_DATA_PROBE_ENABLE"
  echo "CP2887_HTTP_DATA_PORT=$HTTP_DATA_PORT"
  echo "CP3121_HTTP_TRANSFER_BYTES=$HTTP_TRANSFER_BYTES"
  echo CP2887_HTTP_ORDER=prearm_server_before_ocudu_then_phone_request_on_activation
  echo CP2887_HTTP_PROBE=PASS
  exit 0
fi

if [[ "${PAVONIS_CP2895_N3_PROBE:-0}" == 1 ]]; then
  echo "CP2895_PHONE_N3_PREAPPLY=$PHONE_N3_PREAPPLY"
  echo CP2895_N3_PROBE=PASS
  exit 0
fi

if [[ "${PAVONIS_CP2898_PCI_PROBE:-0}" == 1 ]]; then
  echo "CP2898_TARGET_PCI=$TARGET_PCI"
  echo CP2898_PCI_PROBE=PASS
  exit 0
fi

if [[ "${PAVONIS_CP2901_TRIGGER_PROBE:-0}" == 1 ]]; then
  echo "CP2901_SERVICE_TRIGGER_ENABLE=$SERVICE_TRIGGER_ENABLE"
  echo "CP2901_SERVICE_TRIGGER_DURATION_SEC=$SERVICE_TRIGGER_DURATION_SEC"
  echo CP2901_TRIGGER_ORDER=after_ocudu_start_before_target_activation_wait
  echo CP2901_TRIGGER_PROBE=PASS
  exit 0
fi

if [[ "${PAVONIS_CP2904_RESELECT_PROBE:-0}" == 1 ]]; then
  echo "CP2904_POSTSTART_RESELECT_ENABLE=$POSTSTART_RESELECT_ENABLE"
  echo "CP2904_POSTSTART_RESELECT_OBSERVE_SEC=$POSTSTART_RESELECT_OBSERVE_SEC"
  echo "CP2904_TARGET_EVENT_TIMEOUT_SEC=$TARGET_EVENT_TIMEOUT_SEC"
  echo CP2904_RESELECT_ORDER=after_ocudu_start_before_target_activation_wait
  echo CP2904_RESELECT_HELPER=cp2595_oplus_operator_select_remote
  echo CP2904_RESELECT_PROBE=PASS
  exit 0
fi

if [[ "${PAVONIS_CP2907_AIRPLANE_PROBE:-0}" == 1 ]]; then
  echo "CP2907_POSTSTART_AIRPLANE_ENABLE=$POSTSTART_AIRPLANE_ENABLE"
  echo "CP2907_AIRPLANE_HOLD_SEC=$AIRPLANE_HOLD_SEC"
  echo "CP2907_AIRPLANE_SETTLE_SEC=$AIRPLANE_SETTLE_SEC"
  echo "CP2907_TARGET_EVENT_TIMEOUT_SEC=$TARGET_EVENT_TIMEOUT_SEC"
  echo CP2907_AIRPLANE_ORDER=after_ocudu_start_before_target_activation_wait
  echo CP2907_AIRPLANE_PROBE=PASS
  exit 0
fi

if [[ "${PAVONIS_CP2920_POSTAIRPLANE_REAPPLY_PROBE:-0}" == 1 ]]; then
  echo "CP2920_POSTAIRPLANE_N3_REAPPLY=$POSTAIRPLANE_N3_REAPPLY"
  echo CP2920_REAPPLY_HELPER=cp2552_n3_radioinfo_phone_rf_remote
  echo CP2920_REAPPLY_ORDER=after_airplane_cycle_before_target_activation_wait
  echo CP2920_RESTORE_OWNER=existing_phone_n3_applied_cleanup
  echo CP2920_POSTAIRPLANE_REAPPLY_PROBE=PASS
  exit 0
fi

if [[ "${PAVONIS_CP3042_PHASE_MODE_PROBE:-0}" == 1 ]]; then
  echo "CP3042_WRAPPER_PHASE_MODE=$PRACH_COLLECTION_PHASE_MODE"
  echo "CP3044_WRAPPER_EARLY_CAPTURE_MAX=$PRACH_EARLY_CAPTURE_MAX"
  echo "CP3082_WRAPPER_MSG3_TARGET_CAPTURE=$MSG3_TARGET_CAPTURE"
  echo "CP3082_WRAPPER_MSG3_TARGET_MAX=$MSG3_TARGET_MAX"
  echo "CP3082_WRAPPER_UL_SYMBOL_RAW_MAX=$UL_SYMBOL_RAW_MAX"
  echo CP3042_WRAPPER_PHASE_MODE_FORWARD=PASS
  exit 0
fi

[[ "${PAVONIS_CP2893_RF_APPROVED:-0}" == 1 ]] || {
  echo "RF gate closed" >&2
  exit 2
}

QSTAMP="${STAMP}_QCORE"
SSTAMP="${STAMP}_SRSENB"
PSTAMP="${STAMP}_PHONE"
OSTAMP="${STAMP}_OCUDU"
prefix="$ART/cp2893_${STAMP}"

NUC_PREFLIGHT="${prefix}_ran_preflight.log"
SENS_PREFLIGHT="${prefix}_sens_preflight.log"
QCORE_START="${prefix}_qcore_start.log"
SRS_START="${prefix}_srsenb_start.log"
PHONE_PREPARE="${prefix}_phone_prepare.log"
RADIO_CLEAR="${prefix}_phone_radio_clear.log"
SELECT_START="${prefix}_select_start.log"
REG_WAIT="${prefix}_qcore_registration_wait.log"
SERVICE_WAIT="${prefix}_srs_service_wait.log"
SRS_STATUS="${prefix}_srsenb_status.log"
QCORE_SRS_STATUS="${prefix}_qcore_srs_status.log"
PHONE_SRS_STATUS="${prefix}_phone_srs_status.log"
SRS_STOP="${prefix}_srsenb_stop.log"
OCUDU_START="${prefix}_ocudu_start.log"
OCUDU_STATUS="${prefix}_ocudu_status.log"
QCORE_OCUDU_STATUS="${prefix}_qcore_ocudu_status.log"
QCORE_OCUDU_EVENT_STATUS="${prefix}_qcore_ocudu_event_status.log"
PHONE_OCUDU_STATUS="${prefix}_phone_ocudu_status.log"
QCORE_IDENTITY_AUDIT="${prefix}_qcore_identity_audit.log"
PHONE_OCUDU_DATA="${prefix}_phone_ocudu_data.log"
QCORE_OCUDU_DATA="${prefix}_qcore_ocudu_data.log"
TARGET_ACTIVATION_WAIT="${prefix}_target_activation_wait.log"
CORE_HTTP_SERVER="${prefix}_core_http_server.log"
PHONE_HTTP_DATA="${prefix}_phone_http_data.log"
ATTACH_METRICS="${prefix}_attach_metrics.log"
PHONE_N3_APPLY="${prefix}_phone_n3_apply.log"
PHONE_N3_RESTORE="${prefix}_phone_n3_restore.log"
PHONE_SERVICE_TRIGGER_LOG="${prefix}_phone_service_trigger.log"
PHONE_POSTSTART_RESELECT="${prefix}_phone_poststart_reselect.log"
PHONE_POSTSTART_AIRPLANE="${prefix}_phone_poststart_airplane.log"
PHONE_POSTAIRPLANE_N3_REAPPLY="${prefix}_phone_postairplane_n3_reapply.log"
PHONE_POSTSTART_REBOOT="${prefix}_phone_poststart_reboot.log"
SELECT_STOP="${prefix}_select_stop.log"
OCUDU_STOP="${prefix}_ocudu_stop.log"
PHONE_RESTORE="${prefix}_phone_restore.log"
QCORE_STOP="${prefix}_qcore_stop.log"
PACKAGE_NUC="${prefix}_package_ran.log"
PACKAGE_SENS="${prefix}_package_sens.log"
NUC_TAR="${prefix}_ran.tar.gz"
SENS_TAR="${prefix}_ue.tar.gz"
SUMMARY="${prefix}_summary.txt"

for path in "$NUC_PREFLIGHT" "$SENS_PREFLIGHT" "$QCORE_START" "$SRS_START" \
  "$PHONE_PREPARE" "$RADIO_CLEAR" "$SELECT_START" "$REG_WAIT" "$SERVICE_WAIT" "$SRS_STATUS" \
  "$QCORE_SRS_STATUS" "$PHONE_SRS_STATUS" "$SRS_STOP" "$OCUDU_START" \
  "$OCUDU_STATUS" "$QCORE_OCUDU_STATUS" "$QCORE_OCUDU_EVENT_STATUS" "$PHONE_OCUDU_STATUS" "$QCORE_IDENTITY_AUDIT" \
  "$PHONE_OCUDU_DATA" "$QCORE_OCUDU_DATA" "$SELECT_STOP" \
  "$TARGET_ACTIVATION_WAIT" \
  "$CORE_HTTP_SERVER" "$PHONE_HTTP_DATA" "$ATTACH_METRICS" \
  "$PHONE_N3_APPLY" "$PHONE_N3_RESTORE" \
  "$PHONE_SERVICE_TRIGGER_LOG" \
  "$PHONE_POSTSTART_RESELECT" \
  "$PHONE_POSTSTART_AIRPLANE" \
  "$PHONE_POSTAIRPLANE_N3_REAPPLY" \
  "$PHONE_POSTSTART_REBOOT" \
  "$OCUDU_STOP" "$PHONE_RESTORE" "$QCORE_STOP" "$PACKAGE_NUC" "$PACKAGE_SENS" \
  "$NUC_TAR" "$SENS_TAR" "$SUMMARY"; do
  [[ ! -e "$path" ]] || { echo "Fresh-stamp guard: $path exists" >&2; exit 2; }
done

QCORE=@REMOTE_HOME@/pavonis_cp2989_qcore_remote.sh
if [[ "$LOCAL_M2_PRIMER" == 1 ]]; then
  SRS=@REMOTE_HOME@/pavonis_cp2985_srsenb_m2sdr_ch1_att16_rx40_pdu_session_dl_retx2_remote.sh
  SRS_LOCAL="$ART/cp2985_srsenb_m2sdr_ch1_att16_rx40_pdu_session_dl_retx2_remote.sh"
  SRS_HOST=ran
elif [[ "$SRS_MCS4_ENABLE" == 1 ]]; then
  SRS=@REMOTE_HOME@/pavonis_cp2877_srsenb_mcs4_remote.sh
  SRS_LOCAL="$ART/cp2877_srsenb_mcs4_remote.sh"
  SRS_HOST=ue
else
  SRS=@REMOTE_HOME@/pavonis_cp2656_srsenb_remote.sh
  SRS_LOCAL="$ART/cp2656_srsenb_remote.sh"
  SRS_HOST=ue
fi
PHONE=@REMOTE_HOME@/pavonis_cp2629_phone_remote.sh
OCUDU=@REMOTE_HOME@/pavonis_cp3120_ocudu_m2sdr_timing230613_ra40_msg3_target_remote.sh
OCUDU_LOCAL="$ART/cp3120_ocudu_m2sdr_timing230613_ra40_msg3_target_remote.sh"
OCUDU_STAGE=@REMOTE_HOME@/pavonis_cp3059_m2sdr_force_att16_stage_remote.sh
OCUDU_STAGE_LOCAL="$ART/cp3059_m2sdr_force_att16_stage_remote.sh"
PHONE_N3_CONTROL=@REMOTE_HOME@/pavonis_cp2552_n3_radioinfo_phone_rf_remote.sh
PHONE_N3_CONTROL_SHA=ac872c07c884f869c3eea8bef9afb5c90bc75f27e3dcd2fc2b9b0760cb5d88e8
SELECT=@REMOTE_HOME@/pavonis_cp2595_oplus_operator_select_remote.sh
IDENTITY_AUDIT=@REMOTE_HOME@/pavonis_cp2863_qcore_identity_audit.py
PHONE_DATA_PROBE=@REMOTE_HOME@/pavonis_cp3121_phone_data_metrics_remote.sh
CORE_DATA_PROBE=@REMOTE_HOME@/pavonis_cp2870_core_data_probe_remote.sh
TARGET_ACTIVATION_WAITER=@REMOTE_HOME@/pavonis_cp2883_qcore_target_activation_wait_remote.py
CORE_HTTP_ECHO=@REMOTE_HOME@/pavonis_cp3121_core_http_metrics_remote.py
PHONE_HTTP_PROBE=@REMOTE_HOME@/pavonis_cp3121_phone_http_metrics_remote.sh
PHONE_SERVICE_TRIGGER=@REMOTE_HOME@/pavonis_cp2901_phone_service_trigger_remote.sh
PHONE_AIRPLANE_CYCLE=@REMOTE_HOME@/pavonis_cp2907_phone_airplane_cycle_remote.sh
PHONE_REBOOT_UNDER_TARGET=@REMOTE_HOME@/pavonis_cp3030_phone_reboot_under_target_remote.sh

hold_done=0
qcore_started=0
srs_started=0
phone_prepared=0
select_started=0
ocudu_started=0
phone_n3_applied=0
data_probe_gate_pass=1
http_data_gate_pass=1
final_data_gate_pass=1
data_probes_ran=0
target_activation_wait_pid=''
core_http_server_pid=''
target_start_ns=0

run_data_probes() {
  local qcore_status="$1"
  local phone_probe_pid core_probe_pid phone_probe_transport_rc core_probe_transport_rc
  local srs_activations ocudu_activations

  srs_activations="$(sed -n 's/.*userplane_activate=\([0-9][0-9]*\).*/\1/p' "$QCORE_SRS_STATUS" | tail -n 1)"
  ocudu_activations="$(sed -n 's/.*userplane_activate=\([0-9][0-9]*\).*/\1/p' "$qcore_status" | tail -n 1)"
  [[ "$srs_activations" =~ ^[0-9]+$ && "$ocudu_activations" =~ ^[0-9]+$ ]]
  ((ocudu_activations > srs_activations))

  set +e
  python3 "$SSH" --host ue --out "$PHONE_OCUDU_DATA" --timeout 90 -- "$PHONE_DATA_PROBE" "$SERIAL" &
  phone_probe_pid=$!
  python3 "$SSH" --host ran --out "$QCORE_OCUDU_DATA" --timeout 90 -- "$CORE_DATA_PROBE" &
  core_probe_pid=$!
  wait "$phone_probe_pid"
  phone_probe_transport_rc=$?
  wait "$core_probe_pid"
  core_probe_transport_rc=$?
  set -e

  printf 'CP2880_DATA_PROBE_TRANSPORT phone_rc=%s core_rc=%s\n' \
    "$phone_probe_transport_rc" "$core_probe_transport_rc" >>"$PHONE_OCUDU_DATA"
  if ((phone_probe_transport_rc == 0)) && \
      grep -q '^CP3121_PHONE_ECHO_PASS=1' "$PHONE_OCUDU_DATA"; then
    data_probe_gate_pass=1
  else
    data_probe_gate_pass=0
  fi
  data_probes_ran=1
}

run_identity_audit() {
  if [[ "$IDENTITY_AUDIT_ENABLE" == 1 && ! -e "$QCORE_IDENTITY_AUDIT" ]]; then
    python3 "$SSH" --host ran --out "$QCORE_IDENTITY_AUDIT" --timeout 90 -- bash -lc \
      "set -euo pipefail; sudo python3 '$IDENTITY_AUDIT' --log '@REMOTE_HOME@/pavonis_cp2629_runs/$QSTAMP/qcore.log' --sim '@REMOTE_HOME@/CLionProjects/m2sdr/litex_m2sdr/software/validation/ocudu/configs/sims-pavonis-ota.private.toml'"
  fi
}

stop_select() {
  python3 "$SSH" --host ue --out "$SELECT_STOP" --timeout 60 -- bash -lc \
    "set -euo pipefail; root='@REMOTE_HOME@/pavonis_cp2629_runs/$PSTAMP'; pid=''; [[ -s \"\$root/continuous_select.pid\" ]] && pid=\"\$(cat \"\$root/continuous_select.pid\")\"; if [[ \"\$pid\" =~ ^[0-9]+$ ]] && kill -0 \"\$pid\" 2>/dev/null; then kill -TERM \"\$pid\" 2>/dev/null || true; for _ in \$(seq 1 100); do kill -0 \"\$pid\" 2>/dev/null || break; sleep 0.1; done; kill -0 \"\$pid\" 2>/dev/null && kill -KILL \"\$pid\" 2>/dev/null || true; fi; echo CP2857_SELECT_STOP=PASS"
}

restore_phone_n3() {
  python3 "$SSH" --host ue --out "$PHONE_N3_RESTORE" --timeout 180 -- bash -lc \
    "set -euo pipefail; test \"\$(sha256sum '$PHONE_N3_CONTROL' | awk '{print \$1}')\" = '$PHONE_N3_CONTROL_SHA'; '$PHONE_N3_CONTROL' restore '$SERIAL' '$SUB_ID'"
  local rc=$?
  if [[ "$rc" == 0 ]]; then
    phone_n3_applied=0
  fi
  return "$rc"
}

hold_before_teardown_gate() {
  local rc="$1"
  if [[ "$HOLD_BEFORE_TEARDOWN" == 1 && "$hold_done" == 0 ]]; then
    hold_done=1
    local release="$ART/cp3121h_${STAMP}_RELEASE"
    local deadline=$(( $(date +%s) + HOLD_MAX_SEC ))
    echo
    echo "=============================================================="
    echo " HOLDING BEFORE TEARDOWN - the stack is still up."
    echo " rc=$rc   Test the phone now."
    echo
    echo " Release and tear down with:"
    echo "   touch $release"
    echo " or press Ctrl-C once."
    echo " Automatic release after ${HOLD_MAX_SEC}s."
    echo "=============================================================="
    local interrupted=0
    trap 'interrupted=1' INT TERM HUP
    while [[ ! -e "$release" ]] && (( $(date +%s) < deadline )) && [[ "$interrupted" == 0 ]]; do
      sleep 5
    done
    trap - INT TERM HUP
    rm -f "$release"
    echo " hold released; tearing down"
  fi
}

cleanup() {
  local rc=$?
  trap - EXIT INT TERM HUP
  set +e
  hold_before_teardown_gate "$rc"
  if [[ "$core_http_server_pid" =~ ^[0-9]+$ ]] && kill -0 "$core_http_server_pid" 2>/dev/null; then
    kill -TERM "$core_http_server_pid" 2>/dev/null || true
    wait "$core_http_server_pid" 2>/dev/null || true
  fi
  if [[ "$target_activation_wait_pid" =~ ^[0-9]+$ ]] && kill -0 "$target_activation_wait_pid" 2>/dev/null; then
    kill -TERM "$target_activation_wait_pid" 2>/dev/null || true
    wait "$target_activation_wait_pid" 2>/dev/null || true
  fi
  if [[ "$select_started" == 1 ]]; then stop_select; fi
  if [[ "$ocudu_started" == 1 ]]; then
    python3 "$SSH" --host ran --out "$OCUDU_STOP" --timeout 120 -- "$OCUDU" stop "$OSTAMP"
  fi
  if [[ "$srs_started" == 1 ]]; then
    python3 "$SSH" --host "$SRS_HOST" --out "$SRS_STOP" --timeout 90 -- "$SRS" stop "$SSTAMP"
  fi
  if [[ "$phone_n3_applied" == 1 ]]; then restore_phone_n3; fi
  if [[ "$phone_prepared" == 1 ]]; then
    python3 "$SSH" --host ue --out "$PHONE_RESTORE" --timeout 240 -- "$PHONE" restore "$PSTAMP" "$SERIAL" "$SUB_ID"
  fi
  if [[ "$qcore_started" == 1 ]]; then
    run_identity_audit
    python3 "$SSH" --host ran --out "$QCORE_STOP" --timeout 90 -- "$QCORE" stop "$QSTAMP"
  fi
  exit "$rc"
}
trap cleanup EXIT INT TERM HUP

qcore_sha="$(sha256sum "$ART/cp2989_qcore_remote.sh" | cut -d ' ' -f1)"
srs_sha="$(sha256sum "$SRS_LOCAL" | cut -d ' ' -f1)"
phone_sha="$(sha256sum "$ART/cp2629_phone_remote.sh" | cut -d ' ' -f1)"
ocudu_sha="$(sha256sum "$OCUDU_LOCAL" | cut -d ' ' -f1)"
ocudu_stage_sha="$(sha256sum "$OCUDU_STAGE_LOCAL" | cut -d ' ' -f1)"
select_sha=0b1fdaf23bbf64abd498dc27e020050bb6e3a5b802195ab8fc6c448c826a8d7e
target_activation_waiter_sha="$(sha256sum "$ART/cp2883_qcore_target_activation_wait_remote.py" | cut -d ' ' -f1)"
core_http_echo_sha="$(sha256sum "$ART/cp3121_core_http_metrics_remote.py" | cut -d ' ' -f1)"
phone_http_probe_sha="$(sha256sum "$ART/cp3121_phone_http_metrics_remote.sh" | cut -d ' ' -f1)"
phone_data_probe_sha="$(sha256sum "$ART/cp3121_phone_data_metrics_remote.sh" | cut -d ' ' -f1)"
phone_service_trigger_sha="$(sha256sum "$ART/cp2901_phone_service_trigger_remote.sh" | cut -d ' ' -f1)"
phone_airplane_cycle_sha="$(sha256sum "$ART/cp2907_phone_airplane_cycle_remote.sh" | cut -d ' ' -f1)"
phone_reboot_under_target_sha="$(sha256sum "$ART/cp3030_phone_reboot_under_target_remote.sh" | cut -d ' ' -f1)"

python3 "$SSH" --host ran --out "$NUC_PREFLIGHT" --timeout 60 -- bash -lc \
  "set -euo pipefail; test \"\$(ps -C gnb -C qcore -C srsenb --no-headers | wc -l)\" -eq 0; test -e /dev/m2sdr0; test \"\$(cat /sys/module/m2sdr/parameters/dma_reader_program_mode)\" = N; test \"\$(sha256sum '$QCORE' | cut -d ' ' -f1)\" = '$qcore_sha'; test \"\$(sha256sum '$OCUDU' | cut -d ' ' -f1)\" = '$ocudu_sha'; test \"\$(sha256sum '$OCUDU_STAGE' | cut -d ' ' -f1)\" = '$ocudu_stage_sha'; if [[ '$LOCAL_M2_PRIMER' == 1 ]]; then test \"\$(sha256sum '$SRS' | cut -d ' ' -f1)\" = '$srs_sha'; fi; if [[ '$TARGET_ACTIVATION_EVENT_ENABLE' == 1 ]]; then test \"\$(sha256sum '$TARGET_ACTIVATION_WAITER' | cut -d ' ' -f1)\" = '$target_activation_waiter_sha'; fi; if [[ '$HTTP_DATA_PROBE_ENABLE' == 1 ]]; then test \"\$(sha256sum '$CORE_HTTP_ECHO' | cut -d ' ' -f1)\" = '$core_http_echo_sha'; fi; test \"\$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort -u)\" = performance; echo CP2893_NUC4_PREFLIGHT=PASS"
python3 "$SSH" --host ue --out "$SENS_PREFLIGHT" --timeout 60 -- bash -lc \
  "set -euo pipefail; test \"\$(ps -C gnb -C srsenb -C srsue --no-headers | wc -l)\" -eq 0; test \"\$(adb -s '$SERIAL' get-state)\" = device; if [[ '$LOCAL_M2_PRIMER' != 1 ]]; then test \"\$(sha256sum '$SRS' | cut -d ' ' -f1)\" = '$srs_sha'; fi; test \"\$(sha256sum '$PHONE' | cut -d ' ' -f1)\" = '$phone_sha'; test \"\$(sha256sum '$PHONE_N3_CONTROL' | cut -d ' ' -f1)\" = '$PHONE_N3_CONTROL_SHA'; test \"\$(sha256sum '$SELECT' | cut -d ' ' -f1)\" = '$select_sha'; test \"\$(sha256sum '$PHONE_DATA_PROBE' | cut -d ' ' -f1)\" = '$phone_data_probe_sha'; if [[ '$HTTP_DATA_PROBE_ENABLE' == 1 ]]; then test \"\$(sha256sum '$PHONE_HTTP_PROBE' | cut -d ' ' -f1)\" = '$phone_http_probe_sha'; fi; if [[ '$SERVICE_TRIGGER_ENABLE' == 1 ]]; then test \"\$(sha256sum '$PHONE_SERVICE_TRIGGER' | cut -d ' ' -f1)\" = '$phone_service_trigger_sha'; fi; if [[ '$POSTSTART_AIRPLANE_ENABLE' == 1 ]]; then test \"\$(sha256sum '$PHONE_AIRPLANE_CYCLE' | cut -d ' ' -f1)\" = '$phone_airplane_cycle_sha'; fi; if [[ '$POSTSTART_PHONE_REBOOT' == 1 ]]; then test \"\$(sha256sum '$PHONE_REBOOT_UNDER_TARGET' | cut -d ' ' -f1)\" = '$phone_reboot_under_target_sha'; fi; bladeRF-cli -p 2>&1 | grep -q 5c750e8ef0e44a068c7c99fedb9b35fd; echo CP2893_SENS_PREFLIGHT=PASS"

python3 "$SSH" --host ran --out "$QCORE_START" --timeout 90 -- \
  env PAVONIS_QCORE_RAN_INTERFACE_NAME="$QCORE_RAN_INTERFACE_NAME" \
  "$QCORE" start "$QSTAMP"
grep -q 'CP2989_QCORE_START=PASS' "$QCORE_START"
grep -q " ran_interface=$QCORE_RAN_INTERFACE_NAME " "$QCORE_START"
qcore_started=1
if [[ "$LOCAL_M2_PRIMER" == 1 ]]; then
  python3 "$SSH" --host ran --out "$SRS_START" --timeout 120 -- \
    env PAVONIS_CP2985_RF_APPROVED=1 \
    PAVONIS_NR_SETUP_FOLLOWUP_DL_HARQ_RETX_ACKS=2 \
    PAVONIS_SOAPY_SOURCE_SHA_EXPECTED="$SOAPY_SOURCE_SHA_EXPECTED" \
    PAVONIS_SOAPY_MODULE_SHA_EXPECTED="$SOAPY_MODULE_SHA_EXPECTED" \
    "$SRS" start "$SSTAMP"
  grep -q 'CP2985_SRSENB_M2SDR_CH1_ATT16_RX40_PDU_SESSION_DL_RETX2_START=PASS' "$SRS_START"
else
  python3 "$SSH" --host ue --out "$SRS_START" --timeout 120 -- "$SRS" start "$SSTAMP"
  grep -q 'CP2629_SRSENB_START=PASS' "$SRS_START"
fi
srs_started=1
python3 "$SSH" --host ue --out "$PHONE_PREPARE" --timeout 240 -- "$PHONE" prepare "$PSTAMP" "$SERIAL" "$SUB_ID"
grep -q 'CP2629_PHONE_PREPARE=PASS' "$PHONE_PREPARE"
phone_prepared=1
python3 "$SSH" --host ue --out "$RADIO_CLEAR" --timeout 60 -- bash -lc \
  "adb -s '$SERIAL' logcat -b radio -c; echo CP2857_RADIO_CLEAR=PASS"

python3 "$SSH" --host ue --out "$SELECT_START" --timeout 60 -- bash -lc \
  "set -euo pipefail; root='@REMOTE_HOME@/pavonis_cp2629_runs/$PSTAMP'; test ! -e \"\$root/continuous_select.pid\"; nohup '$SELECT' '$SERIAL' '$SUB_ID' '$SLOT_ID' '$PSTAMP' 150 >\"\$root/continuous_select.log\" 2>&1 </dev/null & echo \$! >\"\$root/continuous_select.pid\"; echo CP2857_SELECT_START=PASS"
grep -q 'CP2857_SELECT_START=PASS' "$SELECT_START"
select_started=1

python3 "$SSH" --host ran --out "$REG_WAIT" --timeout 300 -- bash -lc \
  "set -euo pipefail; q='@REMOTE_HOME@/pavonis_cp2629_runs/$QSTAMP/qcore.log'; timeout 240 sh -c 'tail -n 100 -F \"\$1\" | grep -m1 \"Registering imsi-\"' sh \"\$q\" >/dev/null; echo CP2857_QCORE_REGISTERING=PASS"
grep -q 'CP2857_QCORE_REGISTERING=PASS' "$REG_WAIT"

# The selector's trap restores automatic/full RAT. Keep srsENB and qcore live
# while that restore releases the pending RegistrationComplete.
stop_select
select_started=0

set +e
python3 "$SSH" --host ue --out "$SERVICE_WAIT" --timeout "$SERVICE_WAIT_SSH_TIMEOUT" -- bash -lc \
  "set -euo pipefail; deadline=\$((\$(date +%s)+$SERVICE_WAIT_SEC)); while ((\$(date +%s)<deadline)); do line=\$(adb -s '$SERIAL' shell ip -o -4 addr show 2>/dev/null | tr -d '\\r' | awk '\$2 ~ /^(rmnet|ccmni|pdp|v4-rmnet)/ && \$4 ~ /^10\\.255\\.0\\.2\// {print \$2 \" \" \$4; exit}'); if [[ -n \"\$line\" ]]; then echo \"CP2857_SRS_SERVICE=PASS \$line\"; exit 0; fi; sleep 2; done; echo CP2857_SRS_SERVICE=TIMEOUT; exit 1"
service_wait_ssh_rc=$?
set -e
printf 'CP2857_SRS_SERVICE_SSH_RC=%s\n' "$service_wait_ssh_rc" >>"$SERVICE_WAIT"
grep -q 'CP2857_SRS_SERVICE=PASS' "$SERVICE_WAIT"

python3 "$SSH" --host "$SRS_HOST" --out "$SRS_STATUS" --timeout 90 -- "$SRS" status "$SSTAMP"
python3 "$SSH" --host ran --out "$QCORE_SRS_STATUS" --timeout 90 -- "$QCORE" status "$QSTAMP"
python3 "$SSH" --host ue --out "$PHONE_SRS_STATUS" --timeout 90 -- "$PHONE" status "$PSTAMP" "$SERIAL" "$SUB_ID"
awk -F= '/^RACH_COUNT=/{ok=($2+0)>=1} END{exit !ok}' "$SRS_STATUS"
grep -Eq 'userplane_activate=[1-9][0-9]*' "$QCORE_SRS_STATUS"
grep -q 'DATA_WATCH_INTERFACE=' "$PHONE_SRS_STATUS"

python3 "$SSH" --host "$SRS_HOST" --out "$SRS_STOP" --timeout 90 -- "$SRS" stop "$SSTAMP"
if [[ "$LOCAL_M2_PRIMER" == 1 ]]; then
  grep -q 'CP2985_SRSENB_M2SDR_CH1_ATT16_RX40_PDU_SESSION_DL_RETX2_STOP=PASS' "$SRS_STOP"
else
  grep -q 'CP2629_SRSENB_STOP=PASS' "$SRS_STOP"
fi
srs_started=0
if [[ "$TARGET_ACTIVATION_EVENT_ENABLE" == 1 ]]; then
  python3 "$SSH" --host ran --out "$TARGET_ACTIVATION_WAIT" \
    --timeout "$((TARGET_EVENT_TIMEOUT_SEC + 30))" -- \
    python3 "$TARGET_ACTIVATION_WAITER" "@REMOTE_HOME@/pavonis_cp2629_runs/$QSTAMP/qcore.log" \
    --timeout "$TARGET_EVENT_TIMEOUT_SEC" &
  target_activation_wait_pid=$!
  target_activation_armed=0
  for _ in $(seq 1 200); do
    if grep -q '^CP2883_TARGET_ACTIVATION_WATCHER=ARMED' "$TARGET_ACTIVATION_WAIT" 2>/dev/null; then
      target_activation_armed=1
      break
    fi
    kill -0 "$target_activation_wait_pid" 2>/dev/null || break
    sleep 0.05
  done
  [[ "$target_activation_armed" == 1 ]]
fi
if [[ "$HTTP_DATA_PROBE_ENABLE" == 1 ]]; then
  http_nonce="CP2887_${STAMP//./_}"
  python3 "$SSH" --host ran --out "$CORE_HTTP_SERVER" \
    --timeout "$((TARGET_EVENT_TIMEOUT_SEC + 60))" -- \
    python3 "$CORE_HTTP_ECHO" --port "$HTTP_DATA_PORT" --nonce "$http_nonce" \
    --transfer-bytes "$HTTP_TRANSFER_BYTES" --timeout "$TARGET_EVENT_TIMEOUT_SEC" &
  core_http_server_pid=$!
  core_http_armed=0
  for _ in $(seq 1 200); do
    if grep -q '^CP3121_CORE_HTTP_SERVER=ARMED' "$CORE_HTTP_SERVER" 2>/dev/null; then
      core_http_armed=1
      break
    fi
    kill -0 "$core_http_server_pid" 2>/dev/null || break
    sleep 0.05
  done
  [[ "$core_http_armed" == 1 ]]
fi

# CP2894 proved the continuous target can be measured strongly with preapply
# on. CP2895 makes the control optional so one run can test whether forcing it
# after live primer service suppresses fresh access.
if [[ "$PHONE_N3_PREAPPLY" == 1 ]]; then
  phone_n3_applied=1
  python3 "$SSH" --host ue --out "$PHONE_N3_APPLY" --timeout 180 -- bash -lc \
    "set -euo pipefail; test \"\$(sha256sum '$PHONE_N3_CONTROL' | awk '{print \$1}')\" = '$PHONE_N3_CONTROL_SHA'; '$PHONE_N3_CONTROL' apply '$SERIAL' '$SUB_ID'"
  grep -q '^PHONE_N3_APPLY=PASS' "$PHONE_N3_APPLY"
fi

target_start_ns="$(date +%s%N)"
python3 "$SSH" --host ran --out "$OCUDU_START" --timeout 240 -- \
  env PAVONIS_SOAPY_SOURCE_SHA_EXPECTED="$SOAPY_SOURCE_SHA_EXPECTED" \
  PAVONIS_SOAPY_MODULE_SHA_EXPECTED="$SOAPY_MODULE_SHA_EXPECTED" \
  PAVONIS_OCUDU_GNB_BIN_OVERRIDE="$OCUDU_GNB_BIN_OVERRIDE" \
  PAVONIS_OCUDU_GNB_SHA_EXPECTED="$OCUDU_GNB_SHA_EXPECTED" \
  PAVONIS_OCUDU_GNB_LIBRARY_PATH_OVERRIDE="$OCUDU_GNB_LIBRARY_PATH_OVERRIDE" \
  PAVONIS_SOAPY_PHYSICAL_CHANNEL_OFFSET="$PHYSICAL_CHANNEL_OFFSET" \
  PAVONIS_PRACH_COLLECTION_PHASE_MODE="$PRACH_COLLECTION_PHASE_MODE" \
  PAVONIS_PRACH_EARLY_PREDEMOD_CAPTURE_MAX="$PRACH_EARLY_CAPTURE_MAX" \
  PAVONIS_PRACH_TIMING_COMPENSATION_SAMPLES="$PRACH_TIMING_COMPENSATION_SAMPLES" \
  PAVONIS_PRACH_TIMING_COMPENSATION_REPORT_SLOT_OFFSET="$PRACH_REPORT_SLOT_OFFSET" \
  PAVONIS_GNB_RA_RESP_WINDOW="$RA_RESP_WINDOW" \
  PAVONIS_UL_SYMBOL_MSG3_TARGET_CAPTURE="$MSG3_TARGET_CAPTURE" \
  PAVONIS_UL_SYMBOL_MSG3_TARGET_MAX="$MSG3_TARGET_MAX" \
  PAVONIS_UL_SYMBOL_RAW_MAX="$UL_SYMBOL_RAW_MAX" \
  "$OCUDU" start "$OSTAMP" "$TARGET_PCI"
grep -q 'CP3059_OCUDU_M2SDR_STAGE_FORCE_ATT16_START=PASS' "$OCUDU_START"
grep -q " soapy_source_sha=$SOAPY_SOURCE_SHA_EXPECTED " "$OCUDU_START"
grep -q " soapy_module_sha=$SOAPY_MODULE_SHA_EXPECTED" "$OCUDU_START"
grep -q " gnb_sha=$OCUDU_GNB_SHA_EXPECTED " "$OCUDU_START"
grep -q " physical_channel=$PHYSICAL_CHANNEL_LABEL " "$OCUDU_START"
grep -q ' tx_attenuation_db=16 ' "$OCUDU_START"
grep -q ' tx_attenuation_readback_db=16 ' "$OCUDU_START"
grep -q ' stage_sha=7752f0dd049e7a7382cd16d1180699a422a77b8ac574c211fd5bda98d4e5e566 ' "$OCUDU_START"
grep -q " prach_collection_phase_mode=$PRACH_COLLECTION_PHASE_MODE" "$OCUDU_START"
grep -q " prach_early_capture_max=$PRACH_EARLY_CAPTURE_MAX" "$OCUDU_START"
grep -q " prach_timing_compensation_samples=$PRACH_TIMING_COMPENSATION_SAMPLES" "$OCUDU_START"
grep -q " prach_report_slot_offset=$PRACH_REPORT_SLOT_OFFSET" "$OCUDU_START"
grep -q " ra_resp_window=$RA_RESP_WINDOW" "$OCUDU_START"
grep -q " allow_extended_ra_resp_window=$ALLOW_EXTENDED_RA_RESP_WINDOW" "$OCUDU_START"
grep -q " msg3_target_capture=$MSG3_TARGET_CAPTURE" "$OCUDU_START"
grep -q " msg3_target_max=$MSG3_TARGET_MAX" "$OCUDU_START"
grep -q " ul_symbol_raw_max=$UL_SYMBOL_RAW_MAX" "$OCUDU_START"
ocudu_started=1

if [[ "$SERVICE_TRIGGER_ENABLE" == 1 ]]; then
  python3 "$SSH" --host ue --out "$PHONE_SERVICE_TRIGGER_LOG" \
    --timeout "$((SERVICE_TRIGGER_DURATION_SEC + 60))" -- \
    "$PHONE_SERVICE_TRIGGER" "$SERIAL" "$SERVICE_TRIGGER_DURATION_SEC"
  grep -q '^CP2901_PHONE_SERVICE_TRIGGER_PASS=1' "$PHONE_SERVICE_TRIGGER_LOG"
fi

if [[ "$POSTSTART_RESELECT_ENABLE" == 1 ]]; then
  python3 "$SSH" --host ue --out "$PHONE_POSTSTART_RESELECT" --timeout 300 -- \
    "$SELECT" "$SERIAL" "$SUB_ID" "$SLOT_ID" "${PSTAMP}_TARGET" \
    "$POSTSTART_RESELECT_OBSERVE_SEC"
  grep -q '^OPLUS_OPERATOR_SELECT_REMOTE=PASS' "$PHONE_POSTSTART_RESELECT"
  grep -q '^OPLUS_OPERATOR_SELECT_STATE_RESTORE=PASS' "$PHONE_POSTSTART_RESELECT"
fi

if [[ "$POSTSTART_AIRPLANE_ENABLE" == 1 ]]; then
  python3 "$SSH" --host ue --out "$PHONE_POSTSTART_AIRPLANE" \
    --timeout "$((AIRPLANE_HOLD_SEC + AIRPLANE_SETTLE_SEC + 60))" -- \
    "$PHONE_AIRPLANE_CYCLE" "$SERIAL" "$AIRPLANE_HOLD_SEC" "$AIRPLANE_SETTLE_SEC" run n3
  grep -q '^CP2907_PHONE_AIRPLANE_CYCLE_PASS=1' "$PHONE_POSTSTART_AIRPLANE"
fi

if [[ "$POSTAIRPLANE_N3_REAPPLY" == 1 ]]; then
  python3 "$SSH" --host ue --out "$PHONE_POSTAIRPLANE_N3_REAPPLY" \
    --timeout 180 -- bash -lc \
    "set -euo pipefail; test \"\$(sha256sum '$PHONE_N3_CONTROL' | awk '{print \$1}')\" = '$PHONE_N3_CONTROL_SHA'; '$PHONE_N3_CONTROL' apply '$SERIAL' '$SUB_ID'"
  grep -q '^PHONE_N3_APPLY=PASS' "$PHONE_POSTAIRPLANE_N3_REAPPLY"
fi

if [[ "$POSTSTART_PHONE_REBOOT" == 1 ]]; then
  python3 "$SSH" --host ue --out "$PHONE_POSTSTART_REBOOT" --timeout 480 -- \
    "$PHONE_REBOOT_UNDER_TARGET" "$SERIAL" "$SUB_ID"
  grep -q '^CP3030_PHONE_REBOOT_UNDER_TARGET=PASS' "$PHONE_POSTSTART_REBOOT"
fi

if [[ "$TARGET_ACTIVATION_EVENT_ENABLE" == 1 ]]; then
  set +e
  wait "$target_activation_wait_pid"
  target_activation_wait_rc=$?
  set -e
  target_activation_wait_pid=''
  ((target_activation_wait_rc == 0))
  grep -q '^CP2883_TARGET_ACTIVATION=PASS' "$TARGET_ACTIVATION_WAIT"
  target_activation_ns="$(date +%s%N)"
  time_to_attach_ms=$(((target_activation_ns - target_start_ns) / 1000000))
  printf 'CP3121_ATTACH_METRICS target_start_ns=%s target_activation_ns=%s time_to_attach_ms=%s\n' \
    "$target_start_ns" "$target_activation_ns" "$time_to_attach_ms" >"$ATTACH_METRICS"
  python3 "$SSH" --host ran --out "$QCORE_OCUDU_EVENT_STATUS" --timeout 90 -- "$QCORE" status "$QSTAMP"
  if [[ "$HTTP_DATA_PROBE_ENABLE" == 1 ]]; then
    set +e
    python3 "$SSH" --host ue --out "$PHONE_HTTP_DATA" --timeout 240 -- \
      "$PHONE_HTTP_PROBE" "$SERIAL" "$http_nonce" "$HTTP_DATA_PORT" "$HTTP_TRANSFER_BYTES"
    phone_http_transport_rc=$?
    wait "$core_http_server_pid"
    core_http_transport_rc=$?
    set -e
    core_http_server_pid=''
    if ((phone_http_transport_rc == 0 && core_http_transport_rc == 0)) && \
        grep -q '^CP3121_PHONE_HTTP_PASS=1' "$PHONE_HTTP_DATA" && \
        grep -q '^CP3121_CORE_HTTP_PASS=1' "$CORE_HTTP_SERVER"; then
      http_data_gate_pass=1
    else
      http_data_gate_pass=0
    fi
  fi
  run_data_probes "$QCORE_OCUDU_EVENT_STATUS"
fi

# Fixed observation avoids CP1901-style polling of a growing gNB log.
sleep "$SWAP_OBSERVE_SEC"
python3 "$SSH" --host ran --out "$OCUDU_STATUS" --timeout 120 -- "$OCUDU" status "$OSTAMP"
python3 "$SSH" --host ran --out "$QCORE_OCUDU_STATUS" --timeout 90 -- "$QCORE" status "$QSTAMP"
python3 "$SSH" --host ue --out "$PHONE_OCUDU_STATUS" --timeout 90 -- "$PHONE" status "$PSTAMP" "$SERIAL" "$SUB_ID"
if [[ "$IDENTITY_AUDIT_ENABLE" == 1 ]]; then
  run_identity_audit
  grep -q '^QCORE_IDENTITY_AUDIT=PASS' "$QCORE_IDENTITY_AUDIT"
fi
if [[ "$DATA_PROBES_ENABLE" == 1 ]]; then
  ((data_probes_ran == 1)) || run_data_probes "$QCORE_OCUDU_STATUS"
fi
if [[ "$HTTP_DATA_PROBE_ENABLE" == 1 ]]; then
  final_data_gate_pass=$((http_data_gate_pass && data_probe_gate_pass))
else
  final_data_gate_pass="$data_probe_gate_pass"
fi

hold_before_teardown_gate 0
python3 "$SSH" --host ran --out "$OCUDU_STOP" --timeout 120 -- "$OCUDU" stop "$OSTAMP"
grep -q 'CP3059_OCUDU_M2SDR_STAGE_FORCE_ATT16_STOP=PASS' "$OCUDU_STOP"
ocudu_started=0
if [[ "$phone_n3_applied" == 1 ]]; then
  restore_phone_n3
  grep -q '^PHONE_N3_RESTORE=PASS' "$PHONE_N3_RESTORE"
fi
python3 "$SSH" --host ue --out "$PHONE_RESTORE" --timeout 240 -- "$PHONE" restore "$PSTAMP" "$SERIAL" "$SUB_ID"
grep -q 'CP2629_PHONE_RESTORE=PASS' "$PHONE_RESTORE"
phone_prepared=0
python3 "$SSH" --host ran --out "$QCORE_STOP" --timeout 90 -- "$QCORE" stop "$QSTAMP"
grep -q 'CP2989_QCORE_STOP=PASS' "$QCORE_STOP"
qcore_started=0

if [[ "$LOCAL_M2_PRIMER" == 1 ]]; then
  python3 "$SSH" --host ran --out "$PACKAGE_NUC" --timeout 120 -- bash -lc \
    "set -euo pipefail; tar -C @REMOTE_HOME@ -czf /tmp/cp2893_${STAMP}_ran.tar.gz 'pavonis_cp2629_runs/$QSTAMP' 'pavonis_cp2985_srsenb_pdu_session_dl_retx2_runs/$SSTAMP' 'pavonis_cp3059_m2sdr_stage_force_att16_runs/$OSTAMP'; sha256sum /tmp/cp2893_${STAMP}_ran.tar.gz"
  python3 "$SSH" --host ue --out "$PACKAGE_SENS" --timeout 120 -- bash -lc \
    "set -euo pipefail; tar -C @REMOTE_HOME@/pavonis_cp2629_runs -czf /tmp/cp2893_${STAMP}_ue.tar.gz '$PSTAMP'; sha256sum /tmp/cp2893_${STAMP}_ue.tar.gz"
else
  python3 "$SSH" --host ran --out "$PACKAGE_NUC" --timeout 120 -- bash -lc \
    "set -euo pipefail; tar -C @REMOTE_HOME@ -czf /tmp/cp2893_${STAMP}_ran.tar.gz 'pavonis_cp2629_runs/$QSTAMP' 'pavonis_cp3059_m2sdr_stage_force_att16_runs/$OSTAMP'; sha256sum /tmp/cp2893_${STAMP}_ran.tar.gz"
  python3 "$SSH" --host ue --out "$PACKAGE_SENS" --timeout 120 -- bash -lc \
    "set -euo pipefail; tar -C @REMOTE_HOME@/pavonis_cp2629_runs -czf /tmp/cp2893_${STAMP}_ue.tar.gz '$SSTAMP' '$PSTAMP'; sha256sum /tmp/cp2893_${STAMP}_ue.tar.gz"
fi
python3 "$SCP_GET" --host ran --remote "/tmp/cp2893_${STAMP}_ran.tar.gz" --local "$NUC_TAR" --out "${prefix}_get_ran.log" --timeout 180
python3 "$SCP_GET" --host ue --remote "/tmp/cp2893_${STAMP}_ue.tar.gz" --local "$SENS_TAR" --out "${prefix}_get_sens.log" --timeout 180

{
  echo "STAMP=$STAMP"
  echo "SWAP_OBSERVE_SEC=$SWAP_OBSERVE_SEC"
  echo "SERVICE_WAIT_SEC=$SERVICE_WAIT_SEC"
  echo "SRS_MCS4_ENABLE=$SRS_MCS4_ENABLE"
  echo "LOCAL_M2_PRIMER=$LOCAL_M2_PRIMER"
  echo "QCORE_RAN_INTERFACE_NAME=$QCORE_RAN_INTERFACE_NAME"
  echo "SOAPY_SOURCE_SHA_EXPECTED=$SOAPY_SOURCE_SHA_EXPECTED"
  echo "SOAPY_MODULE_SHA_EXPECTED=$SOAPY_MODULE_SHA_EXPECTED"
  echo "OCUDU_GNB_BIN_OVERRIDE=$OCUDU_GNB_BIN_OVERRIDE"
  echo "OCUDU_GNB_SHA_EXPECTED=$OCUDU_GNB_SHA_EXPECTED"
  echo "OCUDU_GNB_LIBRARY_PATH_OVERRIDE=$OCUDU_GNB_LIBRARY_PATH_OVERRIDE"
  echo "PHYSICAL_CHANNEL=$PHYSICAL_CHANNEL_LABEL"
  echo "PRACH_COLLECTION_PHASE_MODE=$PRACH_COLLECTION_PHASE_MODE"
  echo "PRACH_EARLY_CAPTURE_MAX=$PRACH_EARLY_CAPTURE_MAX"
  echo "PRACH_TIMING_COMPENSATION_SAMPLES=$PRACH_TIMING_COMPENSATION_SAMPLES"
  echo "PRACH_REPORT_SLOT_OFFSET=$PRACH_REPORT_SLOT_OFFSET"
  echo "RA_RESP_WINDOW=$RA_RESP_WINDOW"
  echo "ALLOW_EXTENDED_RA_RESP_WINDOW=$ALLOW_EXTENDED_RA_RESP_WINDOW"
  echo "MSG3_TARGET_CAPTURE=$MSG3_TARGET_CAPTURE"
  echo "MSG3_TARGET_MAX=$MSG3_TARGET_MAX"
  echo "UL_SYMBOL_RAW_MAX=$UL_SYMBOL_RAW_MAX"
  echo "TARGET_ACTIVATION_EVENT_ENABLE=$TARGET_ACTIVATION_EVENT_ENABLE"
  echo "HTTP_DATA_PROBE_ENABLE=$HTTP_DATA_PROBE_ENABLE"
  echo "HTTP_TRANSFER_BYTES=$HTTP_TRANSFER_BYTES"
  echo "TARGET_RADIO=M2SDR_COHERENT_SSBPLUS6"
  echo "TARGET_PCI=$TARGET_PCI"
  echo "PHONE_N3_PREAPPLY=$PHONE_N3_PREAPPLY"
  echo "SERVICE_TRIGGER_ENABLE=$SERVICE_TRIGGER_ENABLE"
  echo "SERVICE_TRIGGER_DURATION_SEC=$SERVICE_TRIGGER_DURATION_SEC"
  echo "POSTSTART_RESELECT_ENABLE=$POSTSTART_RESELECT_ENABLE"
  echo "POSTSTART_RESELECT_OBSERVE_SEC=$POSTSTART_RESELECT_OBSERVE_SEC"
  echo "TARGET_EVENT_TIMEOUT_SEC=$TARGET_EVENT_TIMEOUT_SEC"
  echo "POSTSTART_AIRPLANE_ENABLE=$POSTSTART_AIRPLANE_ENABLE"
  echo "AIRPLANE_HOLD_SEC=$AIRPLANE_HOLD_SEC"
  echo "AIRPLANE_SETTLE_SEC=$AIRPLANE_SETTLE_SEC"
  echo "POSTAIRPLANE_N3_REAPPLY=$POSTAIRPLANE_N3_REAPPLY"
  echo "POSTSTART_PHONE_REBOOT=$POSTSTART_PHONE_REBOOT"
  echo "CP2880_DATA_PROBE_GATE_PASS=$data_probe_gate_pass"
  echo "CP2887_HTTP_DATA_GATE_PASS=$http_data_gate_pass"
  echo "PAVONIS_FINAL_DATA_GATE_PASS=$final_data_gate_pass"
  echo "CP2883_DATA_PROBES_RAN=$data_probes_ran"
  echo "CP2893_CONTEXT_INVARIANT=PASS"
  grep -E 'CP2857_QCORE_REGISTERING=|CP2857_SRS_SERVICE=|PHONE_N3_(APPLY|RESTORE)=|CP3059_OCUDU_M2SDR_STAGE_FORCE_ATT16_|RACH_COUNT=|CP2989_QCORE_STATUS=|PRACH_EVENT_COUNT=|RAR_TRACE_COUNT=|PUSCH_COUNT=|CRC_OK_COUNT=|SOAPY_TX_TIMEOUT_COUNT=|DOWNLINK_LATE_COUNT=|DATA_WATCH_|CP2880_|CP2883_|CP2887_|CP3121_|CP2901_|CP2907_|OPLUS_OPERATOR_SELECT_(SCAN_COMPLETE|TARGET_COUNT|TAP_ISSUED|REMOTE|STATE_RESTORE)=' \
    "$REG_WAIT" "$SERVICE_WAIT" "$SRS_STATUS" "$QCORE_SRS_STATUS" "$PHONE_N3_APPLY" "$PHONE_POSTAIRPLANE_N3_REAPPLY" "$PHONE_POSTSTART_REBOOT" "$TARGET_ACTIVATION_WAIT" "$ATTACH_METRICS" "$CORE_HTTP_SERVER" "$PHONE_HTTP_DATA" "$OCUDU_STATUS" "$QCORE_OCUDU_EVENT_STATUS" "$QCORE_OCUDU_STATUS" "$PHONE_OCUDU_STATUS" "$QCORE_IDENTITY_AUDIT" "$PHONE_OCUDU_DATA" "$QCORE_OCUDU_DATA" "$PHONE_N3_RESTORE" "$PHONE_SERVICE_TRIGGER_LOG" "$PHONE_POSTSTART_RESELECT" "$PHONE_POSTSTART_AIRPLANE" 2>/dev/null || true
  sha256sum "$NUC_TAR" "$SENS_TAR"
} >"$SUMMARY"

trap - EXIT INT TERM HUP
cat "$SUMMARY"
((final_data_gate_pass == 1))
