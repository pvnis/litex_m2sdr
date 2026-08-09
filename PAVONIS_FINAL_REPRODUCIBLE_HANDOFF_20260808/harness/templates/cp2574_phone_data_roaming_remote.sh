#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "usage: $0 get SERIAL SUB_ID" >&2
  echo "       $0 set SERIAL SUB_ID 0|1" >&2
  exit 64
}

[[ $# -eq 3 || $# -eq 4 ]] || usage
ACTION="$1"
SERIAL="$2"
SUB_ID="$3"
ADB="${ADB:-/usr/bin/adb}"
KEY="data_roaming${SUB_ID}"

[[ "$SUB_ID" =~ ^[0-9]+$ ]] || usage
STATE="$($ADB devices | awk -v serial="$SERIAL" '$1 == serial {print $2; exit}')"
[[ "$STATE" == device ]] || {
  echo "ADB device is not authorized: serial=$SERIAL state=${STATE:-missing}" >&2
  exit 2
}

read_state() {
  local value
  value="$($ADB -s "$SERIAL" shell settings get global "$KEY" | tr -d '\r')"
  [[ "$value" == 0 || "$value" == 1 ]] || {
    echo "Unexpected $KEY value: $value" >&2
    exit 3
  }
  printf '%s' "$value"
}

if [[ "$ACTION" == get && $# -eq 3 ]]; then
  echo "PAVONIS_DATA_ROAMING sub_id=$SUB_ID key=$KEY value=$(read_state)"
  echo "PAVONIS_DATA_ROAMING_GET=PASS"
  exit 0
fi

if [[ "$ACTION" == set && $# -eq 4 ]]; then
  TARGET="$4"
  [[ "$TARGET" == 0 || "$TARGET" == 1 ]] || usage
  BEFORE="$(read_state)"
  $ADB -s "$SERIAL" shell "su -c 'settings put global $KEY $TARGET'"
  sleep 1
  AFTER="$(read_state)"
  [[ "$AFTER" == "$TARGET" ]] || {
    echo "Data-roaming readback mismatch: target=$TARGET after=$AFTER" >&2
    exit 4
  }
  echo "PAVONIS_DATA_ROAMING_SET sub_id=$SUB_ID key=$KEY before=$BEFORE target=$TARGET after=$AFTER"
  echo "PAVONIS_DATA_ROAMING_SET=PASS"
  exit 0
fi

usage
