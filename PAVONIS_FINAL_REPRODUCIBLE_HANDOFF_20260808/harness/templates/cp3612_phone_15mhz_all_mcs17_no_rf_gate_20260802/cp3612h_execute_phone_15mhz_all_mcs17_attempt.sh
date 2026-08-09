#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-@CONTROLLER_HARNESS@}"
WORKSPACE="$ART"
CP="$ART/cp3612_phone_15mhz_all_mcs17_no_rf_gate_20260802"
HELPERS="$ART/cp3465_phone_mimo_2x2_no_rf_gate_20260731"
SSH="$ART/ssh_pexpect_run.py"
SCP_PUT="$ART/scp_pexpect_put.py"
SCP_GET="$ART/scp_pexpect_get.py"
RUNNER="$CP/cp3612h_run_phone_15mhz_all_mcs17_rf.sh"
RUNNER_SHA=c718b1f8e604cf03be887977c1de4ac4502fdbef02057606385148593d166575
GNB_WRAPPER="$CP/pavonis_cp3612_phone_15mhz_all_mcs17_gnb_wrapper.sh"
GNB_WRAPPER_SHA=7d43f47faaeeb9bca81195b6123ed66664d1233053c2822810c881424736f0f6
GNB_WRAPPER_REMOTE=@REMOTE_HOME@/pavonis_cp3612_phone_15mhz_all_mcs17_gnb_wrapper.sh
PRIMER_HELPER="$ART/cp3585_phone_15mhz_1x1_ota_20260802/cp3585_srsenb_m2sdr_primer_runtime_pinned_remote.sh"
PRIMER_HELPER_SHA=99d7ea55e002f6574a482717eb95dc9ff7e5582582491fa306f30cf4e11f4fb1
PRIMER_HELPER_REMOTE=@REMOTE_HOME@/pavonis_cp3585_srsenb_m2sdr_primer_runtime_pinned_remote.sh
OCUDU_HELPER="$ART/cp3586_phone_15mhz_1x1_ota_20260802/cp3586_ocudu_runtime_pinned_remote.sh"
OCUDU_HELPER_SHA=0e8d88c7b24f743841c0a4ba3251d0288b241271bdefb44c55fa4a7a0593ecc4
OCUDU_HELPER_REMOTE=@REMOTE_HOME@/pavonis_cp3586_ocudu_runtime_pinned_remote.sh
SERVER="$HELPERS/cp3465_sustained_http_server.py"
PHONE="$HELPERS/cp3465_phone_sustained_http.sh"
ANALYZER="$HELPERS/cp3465_analyze_sustained.py"
BASELINE_MANIFEST="$ART/portable_baseline.sha256"

STAMP="${STAMP:?set a fresh STAMP}"
TAG="${TAG:-cp3612_phone_15mhz_all_mcs17}"
DURATION="${PAVONIS_CP3612_DURATION_SEC:-45}"
BENCH_PORT="${PAVONIS_CP3612_PORT:-39088}"
RESULT_ROOT="${PAVONIS_CP3612_RESULT_ROOT:-$CP}"

[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ && "$TAG" =~ ^[A-Za-z0-9_.-]+$ ]]
[[ "$DURATION" =~ ^[0-9]+$ ]] && ((DURATION >= 30 && DURATION <= 60))
[[ "$BENCH_PORT" =~ ^[0-9]+$ ]] && ((BENCH_PORT >= 1024 && BENCH_PORT <= 65535))
[[ "$RESULT_ROOT" == "$ART/"* ]]
[[ "$(sha256sum "$RUNNER" | awk '{print $1}')" == "$RUNNER_SHA" ]]
[[ "$(sha256sum "$GNB_WRAPPER" | awk '{print $1}')" == "$GNB_WRAPPER_SHA" ]]
[[ "$(sha256sum "$PRIMER_HELPER" | awk '{print $1}')" == "$PRIMER_HELPER_SHA" ]]
[[ "$(sha256sum "$OCUDU_HELPER" | awk '{print $1}')" == "$OCUDU_HELPER_SHA" ]]
[[ "${PAVONIS_CP3612_EXECUTE_RF_APPROVED:-0}" == 1 ]] || {
  echo "RF gate closed: set PAVONIS_CP3612_EXECUTE_RF_APPROVED=1" >&2
  exit 2
}
(cd "$WORKSPACE" && sha256sum -c "$BASELINE_MANIFEST" >/dev/null)

ATTEMPT="$RESULT_ROOT/attempt_${STAMP}"
[[ ! -e "$ATTEMPT" ]]
mkdir -m 700 "$ATTEMPT"
TMP="$(mktemp -d /tmp/pavonis_cp3612_execute.XXXXXX)"

SERVER_REMOTE=/tmp/pavonis_cp3612_sustained_http_server.py
PHONE_REMOTE=/tmp/pavonis_cp3612_phone_sustained_http.sh
SERVER_RESULT_REMOTE="/tmp/pavonis_cp3612_${STAMP}_core.json"
PHONE_RESULT_REMOTE="/tmp/pavonis_cp3612_${STAMP}_phone.json"
SERVER_LOG_REMOTE="/tmp/pavonis_cp3612_${STAMP}_core.log"
PHONE_RESULT="$ATTEMPT/phone_sustained.json"
CORE_RESULT="$ATTEMPT/core_sustained.json"
ANALYSIS="$ATTEMPT/sustained_analysis.json"
RUN_LOG="$ATTEMPT/runner.log"
LIFECYCLE="$ATTEMPT/lifecycle_summary.txt"
echo 'STAGE=role_discovery' >"$LIFECYCLE"

cleanup() {
  local rc=$?
  trap - EXIT INT TERM HUP
  set +e
  python3 "$SSH" --host ran --out "$TMP/core_cleanup.raw" --timeout 30 -- \
    bash -lc "pkill -f '[p]avonis_cp3612_sustained_http_server.py.*${STAMP}' || true"
  python3 "$SSH" --host ue --out "$TMP/phone_cleanup.raw" --timeout 30 -- \
    bash -lc "pkill -f '[c]p3157_phone_sustained_http.sh.*${STAMP}' || true"
  rm -rf "$TMP"
  exit "$rc"
}
trap cleanup EXIT INT TERM HUP

python3 "$SSH" --host ue --out "$TMP/roles_before.raw" --timeout 30 -- \
  bash -lc 'set -euo pipefail; serial=$(adb get-serialno | tr -d "\r"); sub=$(adb -s "$serial" shell settings get global multi_sim_data_call | tr -d "\r[:space:]"); row=$(adb -s "$serial" shell su -c "content query --uri content://telephony/siminfo --projection sim_id --where _id=$sub" 2>/dev/null | tr -d "\r"); slot=$(sed -n "s/.*sim_id=\\([0-9][0-9]*\\).*/\\1/p" <<<"$row"); printf "SERIAL=%s\nSUB_ID=%s\nSLOT_ID=%s\n" "$serial" "$sub" "$slot"'
serial="$(tr -d '\r' <"$TMP/roles_before.raw" | sed -n 's/^SERIAL=//p')"
sub_id="$(tr -d '\r' <"$TMP/roles_before.raw" | sed -n 's/^SUB_ID=//p')"
slot_id="$(tr -d '\r' <"$TMP/roles_before.raw" | sed -n 's/^SLOT_ID=//p')"
[[ -n "$serial" && "$sub_id" =~ ^[0-9]+$ && "$slot_id" =~ ^[0-9]+$ ]]

echo 'STAGE=lifecycle_pins' >>"$LIFECYCLE"
fresh_sha=93630dc71ba58969c7dfce9b90ead81faffa4c429a47a14ce09c661902b63ff6
sim_sha=f37becd7da794647cd889178818325a01173f1021350de6ea41d578871b6de0d
policy_sha=ac24833190ae12277ec9600b1fa5ab9bcc9e04689d6fd56872ab952d21ef86d2
python3 "$SSH" --host ue --out "$TMP/lifecycle_pins.raw" --timeout 30 -- \
  bash -lc "set -euo pipefail; test \"\$(sha256sum @REMOTE_HOME@/pavonis_cp2969_fresh_phone_epoch_remote.sh | cut -d ' ' -f1)\" = '$fresh_sha'; test \"\$(sha256sum @REMOTE_HOME@/cp3086_phone_sim_power_cycle_remote.sh | cut -d ' ' -f1)\" = '$sim_sha'; test \"\$(sha256sum @REMOTE_HOME@/cp3087_phone_policy_audit_remote.sh | cut -d ' ' -f1)\" = '$policy_sha'; echo PINS=PASS"

set +e
echo 'STAGE=fresh_phone_epoch' >>"$LIFECYCLE"
python3 "$SSH" --host ue --out "$TMP/fresh.raw" --timeout 600 -- \
  @REMOTE_HOME@/pavonis_cp2969_fresh_phone_epoch_remote.sh \
  "${STAMP}_FRESH" "$serial" "$sub_id"
fresh_rc=$?
sim_rc=1
policy_rc=1
if ((fresh_rc == 0)); then
  echo 'STAGE=sim_power_cycle' >>"$LIFECYCLE"
  python3 "$SSH" --host ue --out "$TMP/sim.raw" --timeout 180 -- \
    env PAVONIS_CP3086_SIM_POWER_CYCLE_APPROVED=1 \
    @REMOTE_HOME@/cp3086_phone_sim_power_cycle_remote.sh cycle "${STAMP}_SIM"
  sim_rc=$?
fi
if ((sim_rc == 0)); then
  sleep 15
  echo 'STAGE=policy_audit' >>"$LIFECYCLE"
  python3 "$SSH" --host ue --out "$TMP/policy.raw" --timeout 180 -- \
    @REMOTE_HOME@/cp3087_phone_policy_audit_remote.sh "${STAMP}_POLICY"
  policy_rc=$?
  if ((policy_rc != 0)); then
    sleep 15
    python3 "$SSH" --host ue --out "$TMP/policy_retry.raw" --timeout 180 -- \
      @REMOTE_HOME@/cp3087_phone_policy_audit_remote.sh "${STAMP}_POLICY_RETRY"
    policy_rc=$?
  fi
fi
set -e
printf 'FRESH_RC=%s\nSIM_RC=%s\nPOLICY_RC=%s\n' \
  "$fresh_rc" "$sim_rc" "$policy_rc" >>"$LIFECYCLE"
((fresh_rc == 0 && sim_rc == 0 && policy_rc == 0))

echo 'STAGE=helper_deploy' >>"$LIFECYCLE"
set +e
python3 "$SCP_PUT" --host ran --local "$SERVER" --remote "$SERVER_REMOTE" \
  --out "$TMP/server_put.raw" --timeout 60
server_put_rc=$?
python3 "$SCP_PUT" --host ue --local "$PHONE" --remote "$PHONE_REMOTE" \
  --out "$TMP/phone_put.raw" --timeout 60
phone_put_rc=$?
python3 "$SCP_PUT" --host ran --local "$PRIMER_HELPER" --remote "$PRIMER_HELPER_REMOTE" \
  --out "$TMP/primer_put.raw" --timeout 60
primer_put_rc=$?
python3 "$SCP_PUT" --host ran --local "$OCUDU_HELPER" --remote "$OCUDU_HELPER_REMOTE" \
  --out "$TMP/ocudu_helper_put.raw" --timeout 60
ocudu_helper_put_rc=$?
python3 "$SCP_PUT" --host ran --local "$GNB_WRAPPER" --remote "$GNB_WRAPPER_REMOTE" \
  --out "$TMP/gnb_wrapper_put.raw" --timeout 60
gnb_wrapper_put_rc=$?
set -e
printf 'SERVER_PUT_RC=%s\nPHONE_PUT_RC=%s\nPRIMER_PUT_RC=%s\nOCUDU_HELPER_PUT_RC=%s\nGNB_WRAPPER_PUT_RC=%s\n' \
  "$server_put_rc" "$phone_put_rc" "$primer_put_rc" "$ocudu_helper_put_rc" \
  "$gnb_wrapper_put_rc" >>"$LIFECYCLE"
server_sha="$(sha256sum "$SERVER" | cut -d ' ' -f1)"
phone_sha="$(sha256sum "$PHONE" | cut -d ' ' -f1)"

echo 'STAGE=primer_helper_ready' >>"$LIFECYCLE"
python3 "$SSH" --host ran --out "$TMP/primer_ready.raw" --timeout 30 -- \
  bash -lc "set -euo pipefail; test \"\$(sha256sum '$PRIMER_HELPER_REMOTE' | cut -d ' ' -f1)\" = '$PRIMER_HELPER_SHA'; chmod 700 '$PRIMER_HELPER_REMOTE'; echo PRIMER_HELPER_GATE=PASS"
grep -q '^PRIMER_HELPER_GATE=PASS' < <(tr -d '\r' <"$TMP/primer_ready.raw")
echo 'PRIMER_HELPER_PROOF=1' >>"$LIFECYCLE"

echo 'STAGE=ocudu_helper_ready' >>"$LIFECYCLE"
python3 "$SSH" --host ran --out "$TMP/ocudu_helper_ready.raw" --timeout 30 -- \
  bash -lc "set -euo pipefail; test \"\$(sha256sum '$OCUDU_HELPER_REMOTE' | cut -d ' ' -f1)\" = '$OCUDU_HELPER_SHA'; chmod 700 '$OCUDU_HELPER_REMOTE'; echo OCUDU_HELPER_GATE=PASS"
grep -q '^OCUDU_HELPER_GATE=PASS' < <(tr -d '\r' <"$TMP/ocudu_helper_ready.raw")
echo 'OCUDU_HELPER_PROOF=1' >>"$LIFECYCLE"

echo 'STAGE=gnb_wrapper_ready' >>"$LIFECYCLE"
python3 "$SSH" --host ran --out "$TMP/gnb_wrapper_ready.raw" --timeout 30 -- \
  bash -lc "set -euo pipefail; test \"\$(sha256sum '$GNB_WRAPPER_REMOTE' | cut -d ' ' -f1)\" = '$GNB_WRAPPER_SHA'; chmod 700 '$GNB_WRAPPER_REMOTE'; PAVONIS_CP3612_GNB_WRAPPER_PROBE=1 '$GNB_WRAPPER_REMOTE'; PAVONIS_CP3612_GNB_WRAPPER_ENV_PROBE=1 '$GNB_WRAPPER_REMOTE'; echo GNB_WRAPPER_GATE=PASS"
grep -q '^GNB_WRAPPER_GATE=PASS' < <(tr -d '\r' <"$TMP/gnb_wrapper_ready.raw")
echo 'GNB_WRAPPER_PROOF=1' >>"$LIFECYCLE"

echo 'STAGE=server_start' >>"$LIFECYCLE"
set +e
python3 "$SSH" --host ran --out "$TMP/server_start.raw" --timeout 30 -- \
  bash -lc "set -euo pipefail; test \"\$(sha256sum '$SERVER_REMOTE' | cut -d ' ' -f1)\" = '$server_sha'; rm -f '$SERVER_RESULT_REMOTE' '$SERVER_LOG_REMOTE'; nohup python3 '$SERVER_REMOTE' --port '$BENCH_PORT' --result '$SERVER_RESULT_REMOTE' --timeout 600 >'$SERVER_LOG_REMOTE' 2>&1 </dev/null &"
server_start_rc=$?
set -e
printf 'SERVER_START_RC=%s\n' "$server_start_rc" >>"$LIFECYCLE"

echo 'STAGE=server_armed' >>"$LIFECYCLE"
set +e
python3 "$SSH" --host ran --out "$TMP/server_armed.raw" --timeout 30 -- \
  bash -lc "set -euo pipefail; for _ in \$(seq 1 100); do if grep -q '^CP3153_SUSTAINED_SERVER_ARMED=1' '$SERVER_LOG_REMOTE'; then echo SERVER_ARMED_GATE=PASS; exit 0; fi; sleep 0.1; done; exit 1"
server_armed_rc=$?
set -e
grep -q '^SERVER_ARMED_GATE=PASS' < <(tr -d '\r' <"$TMP/server_armed.raw")
printf 'SERVER_ARMED_RC=%s\nSERVER_ARMED_PROOF=1\n' \
  "$server_armed_rc" >>"$LIFECYCLE"

echo 'STAGE=phone_helper_ready' >>"$LIFECYCLE"
set +e
python3 "$SSH" --host ue --out "$TMP/phone_ready.raw" --timeout 30 -- \
  bash -lc "set -euo pipefail; test \"\$(sha256sum '$PHONE_REMOTE' | cut -d ' ' -f1)\" = '$phone_sha'; chmod 700 '$PHONE_REMOTE'; rm -f '$PHONE_RESULT_REMOTE'; echo PHONE_HELPER_GATE=PASS"
phone_ready_rc=$?
set -e
grep -q '^PHONE_HELPER_GATE=PASS' < <(tr -d '\r' <"$TMP/phone_ready.raw")
printf 'PHONE_READY_RC=%s\nPHONE_READY_PROOF=1\n' \
  "$phone_ready_rc" >>"$LIFECYCLE"

echo 'STAGE=radio_runner' >>"$LIFECYCLE"
set +e
env \
  PAVONIS_CP3612_RF_APPROVED=1 \
  PAVONIS_PHONE_ADB_SERIAL="$serial" \
  PAVONIS_PHONE_SUB_ID="$sub_id" \
  PAVONIS_PHONE_SLOT_ID="$slot_id" \
  PAVONIS_CP3153_SUSTAINED_ENABLE=1 \
  PAVONIS_CP3153_SUSTAINED_DURATION_SEC="$DURATION" \
  PAVONIS_CP3153_SUSTAINED_PORT="$BENCH_PORT" \
  PAVONIS_CP3153_SUSTAINED_REMOTE="$PHONE_REMOTE" \
  PAVONIS_CP3153_SUSTAINED_RESULT_REMOTE="$PHONE_RESULT_REMOTE" \
  STAMP="$STAMP" \
  TAG="$TAG" \
  "$RUNNER" >"$RUN_LOG" 2>&1
runner_rc=$?
set -e
printf 'RUNNER_RC=%s\n' "$runner_rc" >>"$LIFECYCLE"

if ((runner_rc != 0)); then
  if grep -q 'CP2857_SRS_SERVICE=TIMEOUT' \
      "$ART/cp2893_${STAMP}_srs_service_wait.log" 2>/dev/null; then
    echo 'CLASS=primer_supply_void' >>"$LIFECYCLE"
    exit 20
  fi
  echo 'CLASS=runner_failure' >>"$LIFECYCLE"
  exit "$runner_rc"
fi

echo 'STAGE=result_proof' >>"$LIFECYCLE"
set +e
python3 "$SSH" --host ran --out "$TMP/core_wait.raw" --timeout 90 -- \
  bash -lc "set -euo pipefail; for _ in \$(seq 1 90); do if test -s '$SERVER_RESULT_REMOTE'; then echo CORE_RESULT_GATE=PASS; exit 0; fi; sleep 1; done; exit 1"
core_wait_rc=$?
python3 "$SSH" --host ue --out "$TMP/phone_wait.raw" --timeout 90 -- \
  bash -lc "set -euo pipefail; for _ in \$(seq 1 90); do if test -s '$PHONE_RESULT_REMOTE'; then echo PHONE_RESULT_GATE=PASS; exit 0; fi; sleep 1; done; exit 1"
phone_wait_rc=$?
set -e
grep -q '^CORE_RESULT_GATE=PASS' < <(tr -d '\r' <"$TMP/core_wait.raw")
grep -q '^PHONE_RESULT_GATE=PASS' < <(tr -d '\r' <"$TMP/phone_wait.raw")
printf 'CORE_WAIT_RC=%s\nPHONE_WAIT_RC=%s\nRESULT_PROOF=1\n' \
  "$core_wait_rc" "$phone_wait_rc" >>"$LIFECYCLE"

set +e
python3 "$SCP_GET" --host ran --remote "$SERVER_RESULT_REMOTE" \
  --local "$CORE_RESULT" --out "$TMP/core_get.raw" --timeout 60
core_get_rc=$?
python3 "$SCP_GET" --host ue --remote "$PHONE_RESULT_REMOTE" \
  --local "$PHONE_RESULT" --out "$TMP/phone_get.raw" --timeout 60
phone_get_rc=$?
set -e
jq -e '.complete == true' "$CORE_RESULT" >/dev/null
jq -e '.complete == true' "$PHONE_RESULT" >/dev/null
printf 'CORE_GET_RC=%s\nPHONE_GET_RC=%s\n' \
  "$core_get_rc" "$phone_get_rc" >>"$LIFECYCLE"
python3 "$ANALYZER" --phone "$PHONE_RESULT" --core "$CORE_RESULT" \
  --output "$ANALYSIS"
jq -e '.verdict == "pass"' "$ANALYSIS" >/dev/null
(cd "$WORKSPACE" && sha256sum -c "$BASELINE_MANIFEST" >/dev/null)
echo 'CLASS=valid_success' >>"$LIFECYCLE"
echo 'CP3612_PHONE_15MHZ_ALL_MCS17_ATTEMPT=PASS selector_enable=1 mcs=17'
