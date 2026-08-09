#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:?prepare, watch-data, status, or restore}"
STAMP="${2:?fresh stamp}"
SERIAL="${3:?pass the authorized phone ADB serial}"
SUB_ID="${4:-6}"
[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ ]]
[[ "$SERIAL" =~ ^[A-Za-z0-9._:-]+$ ]]
[[ "$SUB_ID" =~ ^[0-9]+$ ]]

ROOT="@REMOTE_HOME@/pavonis_cp2629_runs/$STAMP"
MONITOR=@REMOTE_HOME@/pavonis_cp2395_phone_adb_monitor_remote.sh
N3=@REMOTE_HOME@/pavonis_cp2552_n3_radioinfo_phone_rf_remote.sh
ROAM=@REMOTE_HOME@/pavonis_cp2574_phone_data_roaming_remote.sh
UPDATE_MODAL=@REMOTE_HOME@/pavonis_cp3032_dismiss_system_update_modal_remote.sh
UPDATE_MODAL_SHA=827ba47ace9a33af300b71bc24a9b1af1d16668f578d2b9bd2af1d8df99859e7
VOICE_JAR=/data/local/tmp/pavonis_voice_policy_v2.cp2563.jar
MANUAL_JAR=/data/local/tmp/pavonis_manual_plmn.jar
ADB=(/usr/bin/adb -s "$SERIAL")

run_voice() {
  "${ADB[@]}" shell \
    "su -c 'CLASSPATH=$VOICE_JAR app_process /system/bin PavonisVoicePolicy $*'" |
    tr -d '\r'
}

run_manual() {
  "${ADB[@]}" shell \
    "su -c 'CLASSPATH=$MANUAL_JAR app_process /system/bin PavonisManualPlmn $1 $SUB_ID'" |
    tr -d '\r'
}

stop_pid_file() {
  local path="$1" pid=''
  [[ -s "$path" ]] && pid="$(cat "$path")"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
  fi
  rm -f "$path"
}

case "$ACTION" in
  prepare)
    mkdir -p "$ROOT"
    chmod 700 "$ROOT"
    [[ $("${ADB[@]}" get-state) == device ]]
    [[ $(sha256sum "$MONITOR" | cut -d ' ' -f1) == 39bc31c45c91d8d68ea3947e3681b6d948ba45b9e5cb67721b600bc10b919258 ]]
    [[ $(sha256sum "$N3" | cut -d ' ' -f1) == c6bfd5e3f6393fd6f7152b393782da364ea9e4a6f6040eb52bb692215da40e22 ]]
    [[ $(sha256sum "$ROAM" | cut -d ' ' -f1) == ab33218f52ff07f3bdc144da4eb9d4961c0d4c3ffc0c6f92a884898c05cff109 ]]
    [[ $(sha256sum "$UPDATE_MODAL" | cut -d ' ' -f1) == "$UPDATE_MODAL_SHA" ]]
    [[ $("${ADB[@]}" shell sha256sum "$VOICE_JAR" | tr -d '\r' | cut -d ' ' -f1) == 208d7e72d76a272e0fea1a41af8da3fc873dc2d9fd1315fb6fa234c1f1e4dcdf ]]
    "$UPDATE_MODAL" "$SERIAL" | tee "$ROOT/phone_update_modal.log"
    [[ $(run_manual get | sed -n 's/.* mode=\([0-9][0-9]*\).*/\1/p') == 1 ]]
    "$ROAM" get "$SERIAL" "$SUB_ID" | grep -q 'value=0'
    run_voice get "$SUB_ID" | grep -q 'vonr=true advanced_calling=true'

    nohup env PAVONIS_PHONE_MONITOR_INTERVAL_SEC=3 \
      "$MONITOR" "$SERIAL" 300 "$ROOT/phone_monitor.tsv" \
      >"$ROOT/phone_monitor.stdout.log" 2>&1 </dev/null &
    echo $! >"$ROOT/phone_monitor.pid"
    nohup "$0" watch-data "$STAMP" "$SERIAL" "$SUB_ID" \
      >"$ROOT/phone_data_watch.log" 2>&1 </dev/null &
    echo $! >"$ROOT/phone_data_watch.pid"

    "$ROAM" set "$SERIAL" "$SUB_ID" 1
    run_voice set "$SUB_ID" false false
    "$N3" apply "$SERIAL" "$SUB_ID"
    echo CP2629_PHONE_PREPARE=PASS
    ;;
  watch-data)
    deadline=$(( $(date +%s) + 300 ))
    echo "DATA_WATCH_START_UTC=$(date -u +%FT%TZ)"
    while (( $(date +%s) < deadline )); do
      cell_line="$("${ADB[@]}" shell ip -o -4 addr show 2>/dev/null | tr -d '\r' |
        awk '$2 ~ /^(rmnet|ccmni|pdp|v4-rmnet)/ {print $2 " " $4; exit}' || true)"
      if [[ -n "$cell_line" ]]; then
        read -r iface cidr <<<"$cell_line"
        echo "DATA_WATCH_INTERFACE=$iface"
        echo "DATA_WATCH_ADDRESS=$cidr"
        set +e
        "${ADB[@]}" shell ping -I "$iface" -c 10 -W 2 10.255.0.1 | tr -d '\r'
        ping_rc=${PIPESTATUS[0]}
        set -e
        echo "DATA_WATCH_PING_RC=$ping_rc"
        echo "DATA_WATCH_END_UTC=$(date -u +%FT%TZ)"
        exit "$ping_rc"
      fi
      sleep 2
    done
    echo DATA_WATCH_RESULT=NO_CELLULAR_INTERFACE
    exit 3
    ;;
  status)
    echo PHONE_MONITOR_ROWS=$(awk 'BEGIN{n=0} /^[0-9][0-9][0-9][0-9]-/{n++} END{print n}' "$ROOT/phone_monitor.tsv" 2>/dev/null || echo 0)
    tail -n 12 "$ROOT/phone_monitor.tsv" 2>/dev/null || true
    echo PHONE_DATA_WATCH
    tail -n 30 "$ROOT/phone_data_watch.log" 2>/dev/null || true
    echo PHONE_CURRENT_IP
    "${ADB[@]}" shell ip -o -4 addr show 2>/dev/null | tr -d '\r' |
      awk '$2 ~ /^(rmnet|ccmni|pdp|v4-rmnet)/ {print; found=1} END{if(!found) print "none"}'
    echo CP2629_PHONE_STATUS=PASS
    ;;
  restore)
    stop_pid_file "$ROOT/phone_data_watch.pid"
    stop_pid_file "$ROOT/phone_monitor.pid"
    set +e
    "$N3" restore "$SERIAL" "$SUB_ID" >"$ROOT/phone_n3_restore.log" 2>&1
    n3_rc=$?
    "$ROAM" set "$SERIAL" "$SUB_ID" 0 >"$ROOT/phone_roaming_restore.log" 2>&1
    roam_rc=$?
    run_voice set "$SUB_ID" true true >"$ROOT/phone_voice_restore.log" 2>&1
    voice_rc=$?
    "${ADB[@]}" shell input keyevent 3 >/dev/null 2>&1
    set -e
    n3_state_ok=0
    if grep -q '^RESTORE_SELECTION_MODE=1$' "$ROOT/phone_n3_restore.log" &&
       grep -q '^RESTORE_MODE6=33$' "$ROOT/phone_n3_restore.log" &&
       grep -q '^RESTORE_OPLUS_MODE6=33$' "$ROOT/phone_n3_restore.log"; then
      n3_state_ok=1
    fi
    mode="$(run_manual get | sed -n 's/.* mode=\([0-9][0-9]*\).*/\1/p')"
    roam="$($ROAM get "$SERIAL" "$SUB_ID" | sed -n 's/.* value=\([01]\).*/\1/p' | head -n1)"
    voice="$(run_voice get "$SUB_ID" | grep 'PAVONIS_VOICE_POLICY sub_id=')"
    echo "PHONE_RESTORE_RC n3=$n3_rc n3_state_ok=$n3_state_ok roaming=$roam_rc voice=$voice_rc"
    echo "PHONE_RESTORE_STATE mode=$mode roaming=$roam voice=$voice"
    [[ ("$n3_rc" == 0 || "$n3_state_ok" == 1) && "$roam_rc" == 0 && "$voice_rc" == 0 ]]
    [[ "$mode" == 1 && "$roam" == 0 && "$voice" == *'vonr=true advanced_calling=true'* ]]
    echo CP2629_PHONE_RESTORE=PASS
    ;;
  *)
    echo "usage: $0 prepare|watch-data|status|restore STAMP [SERIAL [SUB_ID]]" >&2
    exit 64
    ;;
esac
