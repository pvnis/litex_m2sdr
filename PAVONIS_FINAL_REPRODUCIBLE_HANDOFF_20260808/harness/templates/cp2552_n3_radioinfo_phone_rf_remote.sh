#!/usr/bin/env bash
set -euo pipefail

ACTION=${1:?apply or restore}
SERIAL=${2:?pass the authorized phone ADB serial}
SUB_ID=${3:-6}
MODE_HELPER=@REMOTE_HOME@/pavonis_cp2531_radioinfo_set_mode_remote.sh
MODE_HELPER_SHA256=4bbb5a4a56dd1cb7d3254d85b8ba4c056db63b9c16093dc31dc9d1882f142b34
SYSTEM_JAR=/data/local/tmp/pavonis_system_selection.cp2551.jar
SYSTEM_JAR_SHA256=66e43df91f68412b9ed6c6e5346de869f0aef469c927cf773ae491cc492c21cf
MANUAL_JAR=/data/local/tmp/pavonis_manual_plmn.jar
MANUAL_JAR_SHA256=66e82f1f7f8ac01e282d49eb0239a59e05b8c98342daf4e19375f9723f5693f8
FULL_MASK_TEXT='GPRS|EDGE|UMTS|CDMA|CDMA - EvDo rev. 0|CDMA - EvDo rev. A|CDMA - 1xRTT|HSDPA|HSUPA|HSPA|CDMA - EvDo rev. B|LTE|CDMA - eHRPD|HSPA+|GSM|TD_SCDMA|LTE_CA|NR'
ADB=(/usr/bin/adb -s "$SERIAL")

run_system() {
  "${ADB[@]}" shell \
    "su -c 'CLASSPATH=$SYSTEM_JAR app_process /system/bin PavonisSystemSelection $1 $SUB_ID'" |
    tr -d '\r'
}

run_manual() {
  "${ADB[@]}" shell \
    "su -c 'CLASSPATH=$MANUAL_JAR app_process /system/bin PavonisManualPlmn $1 $SUB_ID'" |
    tr -d '\r'
}

verify_tools() {
  [[ $(sha256sum "$MODE_HELPER" | awk '{print $1}') == "$MODE_HELPER_SHA256" ]]
  local system_hash manual_hash
  system_hash=$("${ADB[@]}" shell "su -c 'sha256sum $SYSTEM_JAR'" | tr -d '\r' | awk '{print $1}')
  manual_hash=$("${ADB[@]}" shell "su -c 'sha256sum $MANUAL_JAR'" | tr -d '\r' | awk '{print $1}')
  [[ "$system_hash" == "$SYSTEM_JAR_SHA256" ]]
  [[ "$manual_hash" == "$MANUAL_JAR_SHA256" ]]
  echo "MODE_HELPER_SHA256=$MODE_HELPER_SHA256"
  echo "SYSTEM_JAR_SHA256=$system_hash"
  echo "MANUAL_JAR_SHA256=$manual_hash"
}

restore_state() {
  local clear_rc=0 automatic_rc=0 full_rc=0 current_mode
  set +e
  run_system clear
  clear_rc=$?
  current_mode=$(run_manual get | sed -n 's/.* mode=\([0-9][0-9]*\).*/\1/p' | tail -n 1)
  if [[ "$current_mode" == 1 ]]; then
    echo "RESTORE_AUTOMATIC_SKIPPED=already_mode1"
  else
    run_manual automatic
    automatic_rc=$?
  fi
  sleep 10
  full_rc=1
  for attempt in 1 2 3; do
    echo "RESTORE_FULL_ATTEMPT=$attempt"
    echo "RESTORE_FULL_ENTRY=direct"
    "$MODE_HELPER" "$SERIAL" full direct
    full_rc=$?
    if [[ "$full_rc" == 0 ]]; then
      break
    fi
    sleep 10
  done
  if [[ "$full_rc" != 0 ]]; then
    for attempt in 1 2; do
      echo "RESTORE_FULL_FALLBACK_ATTEMPT=$attempt"
      echo "RESTORE_FULL_ENTRY=via_app"
      "$MODE_HELPER" "$SERIAL" full via_app
      full_rc=$?
      if [[ "$full_rc" == 0 ]]; then
        break
      fi
      sleep 10
    done
  fi
  set -e

  local mode mask mode6 oplus_mode6
  mode=$(run_manual get | tee /dev/stderr | sed -n 's/.* mode=\([0-9][0-9]*\).*/\1/p' | tail -n 1)
  mask=$("${ADB[@]}" shell cmd phone get-allowed-network-types-for-users -s 0 | tr -d '\r')
  mode6=$("${ADB[@]}" shell settings get global preferred_network_mode6 | tr -d '\r')
  oplus_mode6=$("${ADB[@]}" shell settings get global oplus_user_preferred_network_mode6 | tr -d '\r')
  echo "RESTORE_SYSTEM_CLEAR_RC=$clear_rc"
  echo "RESTORE_AUTOMATIC_RC=$automatic_rc"
  echo "RESTORE_FULL_MODE_RC=$full_rc"
  echo "RESTORE_SELECTION_MODE=$mode"
  echo "RESTORE_MASK=$mask"
  echo "RESTORE_MODE6=$mode6"
  echo "RESTORE_OPLUS_MODE6=$oplus_mode6"
  [[ "$clear_rc" == 0 && "$automatic_rc" == 0 && "$full_rc" == 0 ]]
  [[ "$mode" == 1 && "$mask" == "$FULL_MASK_TEXT" ]]
  [[ "$mode6" == 33 && "$oplus_mode6" == 33 ]]
  echo "PHONE_N3_RESTORE=PASS"
}

verify_tools
case "$ACTION" in
  apply)
    echo "PHONE_N3_APPLY_START_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    set +e
    "$MODE_HELPER" "$SERIAL" nr_only via_app
    mode_rc=$?
    system_rc=99
    if [[ "$mode_rc" == 0 ]]; then
      run_system set-n3
      system_rc=$?
    fi
    set -e
    if [[ "$mode_rc" != 0 || "$system_rc" != 0 ]]; then
      echo "PHONE_N3_APPLY_FAILED mode_rc=$mode_rc system_rc=$system_rc" >&2
      restore_state
      exit 1
    fi
    echo "PHONE_N3_APPLY_MODE_RC=$mode_rc"
    echo "PHONE_N3_APPLY_SYSTEM_RC=$system_rc"
    echo "PHONE_N3_APPLY_END_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "PHONE_N3_APPLY=PASS"
    ;;
  restore)
    echo "PHONE_N3_RESTORE_START_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    restore_state
    echo "PHONE_N3_RESTORE_END_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    ;;
  *)
    echo "usage: $0 apply|restore [SERIAL [SUB_ID]]" >&2
    exit 64
    ;;
esac
