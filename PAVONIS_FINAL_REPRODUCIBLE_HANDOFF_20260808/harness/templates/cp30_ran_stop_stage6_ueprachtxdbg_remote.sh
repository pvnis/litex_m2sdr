#!/usr/bin/env bash
set -euo pipefail

SELF="$$"
TAG="${TAG:-codex_stage6_cp30_ueprachtxdbg}"

collect_ancestor_pids() {
  local pid="$SELF"
  while [[ "$pid" =~ ^[0-9]+$ ]] && ((pid >= 1)); do
    printf '%s ' "$pid"
    ((pid == 1)) && break
    pid=$(awk '/^PPid:/ {print $2}' "/proc/$pid/status" 2>/dev/null || true)
  done
}

ANCESTOR_PIDS=$(collect_ancestor_pids)

collect_pids() {
  ps -eo pid=,ppid=,comm=,args= | awk -v ancestors="$ANCESTOR_PIDS" -v self="$SELF" -v tag="$TAG" '
    index(" " ancestors, " " $1 " ") { next }
    $2 == self { next }
    $3 ~ /^(ps|awk|sort|uniq|grep|sudo)$/ { next }
    index($0, tag) ||
    /\/ocudu\/build-clion\/apps\/gnb\/gnb -c .*\/gnb-run.yml/ ||
    /\/gnb[^[:space:]]*[[:space:]]+-c[[:space:]].*\/gnb-run.yml/ ||
    /\/qcore\/target\/debug\/qcore --mcc 001 --mnc 01/ {
      print $1
    }
  ' | sort -n | uniq
}

date -u +%FT%TZ
echo "TAG=$TAG"
echo "BEFORE"
ps -eo pid,ppid,stat,etime,cmd | grep -Ei "$TAG|/ocudu/build-clion/apps/gnb/gnb|/qcore/target/debug/qcore" | grep -v grep || true

mapfile -t PIDS < <(collect_pids)
if [ "${#PIDS[@]}" -gt 0 ]; then
  echo "INT_PIDS=${PIDS[*]}"
  sudo -n kill -INT "${PIDS[@]}" 2>/dev/null || kill -INT "${PIDS[@]}" 2>/dev/null || true
  sleep 5
fi

echo "AFTER_INT"
ps -eo pid,ppid,stat,etime,cmd | grep -Ei "$TAG|/ocudu/build-clion/apps/gnb/gnb|/qcore/target/debug/qcore" | grep -v grep || true

mapfile -t PIDS2 < <(collect_pids)
if [ "${#PIDS2[@]}" -gt 0 ]; then
  echo "TERM_PIDS=${PIDS2[*]}"
  sudo -n kill -TERM "${PIDS2[@]}" 2>/dev/null || kill -TERM "${PIDS2[@]}" 2>/dev/null || true
  sleep 5
fi

echo "AFTER_TERM"
ps -eo pid,ppid,stat,etime,cmd | grep -Ei "$TAG|/ocudu/build-clion/apps/gnb/gnb|/qcore/target/debug/qcore" | grep -v grep || true

mapfile -t PIDS3 < <(collect_pids)
if [ "${#PIDS3[@]}" -gt 0 ]; then
  echo "KILL_PIDS=${PIDS3[*]}"
  sudo -n kill -KILL "${PIDS3[@]}" 2>/dev/null || kill -KILL "${PIDS3[@]}" 2>/dev/null || true
  sleep 1
fi

mapfile -t PIDS4 < <(collect_pids)
if [ "${#PIDS4[@]}" -gt 0 ]; then
  echo "FINAL_REMAINING_PIDS=${PIDS4[*]}" >&2
  exit 1
fi
echo "FINAL_STOP=PASS"
