#!/usr/bin/env bash
set -euo pipefail

SELF="$$"

collect_pids() {
  ps -eo pid=,cmd= | awk -v self="$SELF" '
    $1 == self { next }
    /\/runs\/codex_stage6_.*\/run_tx\.sh/ ||
    /scripts\/run_ota_stage_template\.sh --i-understand-rf-test/ ||
    /\/ocudu\/build-clion\/apps\/gnb\/gnb -c .*\/gnb-run\.yml/ ||
    /\/qcore\/target\/debug\/qcore --mcc 001 --mnc 01/ {
      print $1
    }
  ' | sort -n | uniq
}

show_state() {
  local label="$1"
  echo "$label"
  ps -eo pid,ppid,stat,etime,cmd | grep -Ei 'codex_stage6_|run_ota_stage_template|/ocudu/build-clion/apps/gnb/gnb|/qcore/target/debug/qcore' | grep -v grep || true
}

kill_stage() {
  local signal="$1"
  shift
  local pids=("$@")
  if [ "${#pids[@]}" -eq 0 ]; then
    echo "${signal}_PIDS="
    return 0
  fi
  echo "${signal}_PIDS=${pids[*]}"
  sudo -n kill "-${signal}" "${pids[@]}" 2>/dev/null || kill "-${signal}" "${pids[@]}" 2>/dev/null || true
}

date -u +%FT%TZ
show_state "BEFORE"

mapfile -t PIDS < <(collect_pids)
kill_stage INT "${PIDS[@]}"
sleep 3
show_state "AFTER_INT"

mapfile -t PIDS < <(collect_pids)
kill_stage TERM "${PIDS[@]}"
sleep 3
show_state "AFTER_TERM"

mapfile -t PIDS < <(collect_pids)
kill_stage KILL "${PIDS[@]}"
sleep 1
show_state "AFTER_KILL"

mapfile -t LEFT < <(collect_pids)
echo "CLEANUP_LEFTOVER_COUNT=${#LEFT[@]}"
if [ "${#LEFT[@]}" -ne 0 ]; then
  echo "CLEANUP_LEFTOVER_PIDS=${LEFT[*]}"
  exit 1
fi

echo "CLEANUP_PASS"
