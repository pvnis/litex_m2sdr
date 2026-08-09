#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:?start, status, or stop}"
STAMP="${2:?fresh stamp}"
SERIAL="${3:?pass the authorized phone ADB serial}"
[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ && "$SERIAL" =~ ^[A-Za-z0-9._:-]+$ ]]

ROOT="@REMOTE_HOME@/pavonis_cp2629_runs/$STAMP"
PID_FILE="$ROOT/qcsuper_filtered.pid"
USB_FILE="$ROOT/qcsuper_filtered.original_usb_config"
DLF="$ROOT/phone_qcsuper_filtered.dlf"
LOG="$ROOT/phone_qcsuper_filtered.stdout.log"
ADB=/usr/bin/adb
ENTRY=@REMOTE_HOME@/pavonis_cp3061_qcsuper_filtered_entry.py
ENTRY_SHA=efe1c8031e297b5bdfd143a1b2241115837ff3c3658aebe4d355718575bc3b38

restore_usb() {
  local original current
  original=adb
  [[ -s "$USB_FILE" ]] && original="$(tr -d '\r\n' <"$USB_FILE")"
  "$ADB" wait-for-device
  current="$("$ADB" -s "$SERIAL" shell su -c getprop\ sys.usb.config | tr -d '\r')"
  if [[ "$current" != "$original" ]]; then
    "$ADB" -s "$SERIAL" shell su -c setprop\ sys.usb.config\ "$original" || true
    sleep 4
    "$ADB" wait-for-device
  fi
  [[ "$("$ADB" -s "$SERIAL" get-state)" == device ]]
  current="$("$ADB" -s "$SERIAL" shell su -c getprop\ sys.usb.config | tr -d '\r')"
  [[ "$current" == "$original" ]]
}

stop_capture() {
  local pid=''
  [[ -s "$PID_FILE" ]] && pid="$(cat "$PID_FILE")"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 100); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -0 "$pid" 2>/dev/null && kill -TERM "$pid" 2>/dev/null || true
  fi
  restore_usb
}

case "$ACTION" in
  start)
    mkdir -p "$ROOT"
    chmod 700 "$ROOT"
    [[ ! -e "$PID_FILE" && ! -e "$DLF" && ! -e "$LOG" ]]
    [[ "$(sha256sum "$ENTRY" | cut -d ' ' -f1)" == "$ENTRY_SHA" ]]
    [[ "$("$ADB" -s "$SERIAL" get-state)" == device ]]
    original="$("$ADB" -s "$SERIAL" shell su -c getprop\ sys.usb.config | tr -d '\r')"
    [[ "$original" == adb ]]
    printf '%s\n' "$original" >"$USB_FILE"
    : >"$LOG"
    pid="$(python3 - "$ENTRY" "$DLF" "$LOG" <<'PY'
import signal
import subprocess
import sys

entry, dlf, log_path = sys.argv[1:]

def reset_signals():
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)

with open(log_path, "ab", buffering=0) as log:
    proc = subprocess.Popen(
        [entry, "--adb", "--dlf-dump", dlf],
        stdin=subprocess.DEVNULL,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        close_fds=True,
        preexec_fn=reset_signals,
    )
print(proc.pid)
PY
)"
    printf '%s\n' "$pid" >"$PID_FILE"
    ready=0
    for _ in $(seq 1 300); do
      if kill -0 "$pid" 2>/dev/null && [[ -f "$DLF" ]] &&
         grep -q 'PAVONIS_QCSUPER_FILTERED_CODES=b821,b889,b88a' "$LOG" &&
         grep -q 'Enabled logging for:' "$LOG"; then
        ready=1
        break
      fi
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    if [[ "$ready" != 1 ]]; then
      stop_capture
      tail -n 40 "$LOG"
      exit 1
    fi
    "$ADB" wait-for-device
    [[ "$("$ADB" -s "$SERIAL" get-state)" == device ]]
    echo "CP3061_QCSUPER_FILTERED_START=PASS pid=$pid dlf=$DLF codes=b821,b889,b88a"
    ;;
  status)
    pid="$(cat "$PID_FILE")"
    kill -0 "$pid"
    size="$(stat -c %s "$DLF")"
    echo "CP3061_QCSUPER_FILTERED_STATUS=RUNNING pid=$pid dlf_size=$size"
    ;;
  stop)
    stop_capture
    size="$(stat -c %s "$DLF")"
    echo "CP3061_QCSUPER_FILTERED_STOP=PASS dlf_size=$size usb_config=adb"
    ;;
  *)
    echo "usage: $0 start|status|stop STAMP [SERIAL]" >&2
    exit 64
    ;;
esac
