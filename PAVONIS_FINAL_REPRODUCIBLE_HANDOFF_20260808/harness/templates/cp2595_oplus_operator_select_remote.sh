#!/usr/bin/env bash
set -euo pipefail

SERIAL="${1:?serial}"
SUB_ID="${2:-6}"
SLOT_ID="${3:-0}"
STAMP="${4:?stamp}"
OBSERVE_SEC="${5:-90}"
TARGET_TITLE='901 70 5G'
ADB=/usr/bin/adb
MANUAL_JAR=/data/local/tmp/pavonis_manual_plmn.jar
MODE_HELPER=@REMOTE_HOME@/pavonis_cp2531_radioinfo_set_mode_remote.sh
MODE_HELPER_SHA256=4bbb5a4a56dd1cb7d3254d85b8ba4c056db63b9c16093dc31dc9d1882f142b34
FULL_MASK_TEXT='GPRS|EDGE|UMTS|CDMA|CDMA - EvDo rev. 0|CDMA - EvDo rev. A|CDMA - 1xRTT|HSDPA|HSUPA|HSPA|CDMA - EvDo rev. B|LTE|CDMA - eHRPD|HSPA+|GSM|TD_SCDMA|LTE_CA|NR'
SCAN_XML="/tmp/pavonis_oplus_operator_select_scan_${STAMP}.xml"
STATE_XML="/tmp/pavonis_oplus_operator_select_state_${STAMP}.xml"

[[ "$SERIAL" =~ ^[A-Za-z0-9._:-]+$ ]]
[[ "$SUB_ID" =~ ^[0-9]+$ ]]
[[ "$SLOT_ID" =~ ^[0-9]+$ ]]
[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ ]]
[[ "$OBSERVE_SEC" =~ ^[0-9]+$ ]]
(( OBSERVE_SEC >= 30 && OBSERVE_SEC <= 150 ))

run_manual() {
  "$ADB" -s "$SERIAL" shell \
    "su -c 'CLASSPATH=$MANUAL_JAR app_process /system/bin PavonisManualPlmn $1 $SUB_ID'" |
    tr -d '\r'
}

finish_operator_activity() {
  "$ADB" -s "$SERIAL" shell \
    "su -c 'am start -W -n com.android.phone/.OplusNetworkSetting --ei subscription $SLOT_ID'" \
    >/dev/null 2>&1 || true
  sleep 1
  "$ADB" -s "$SERIAL" shell input keyevent 4 >/dev/null 2>&1 || true
  sleep 1
  "$ADB" -s "$SERIAL" shell input keyevent 4 >/dev/null 2>&1 || true
  sleep 1
  "$ADB" -s "$SERIAL" shell input keyevent 3 >/dev/null 2>&1 || true
}

state_may_change=0
restore_state() {
  if [[ "$state_may_change" != 1 ]]; then
    return 0
  fi
  run_manual automatic
  finish_operator_activity
  [[ $(sha256sum "$MODE_HELPER" | awk '{print $1}') == "$MODE_HELPER_SHA256" ]]
  "$MODE_HELPER" "$SERIAL" full via_app
  "$ADB" -s "$SERIAL" shell input keyevent 3 >/dev/null 2>&1 || true

  local mode mask mode6 oplus_mode6
  mode=$(run_manual get | tee /dev/stderr | sed -n 's/.* mode=\([0-9][0-9]*\).*/\1/p' | tail -n 1)
  mask=$("$ADB" -s "$SERIAL" shell cmd phone get-allowed-network-types-for-users -s 0 | tr -d '\r')
  mode6=$("$ADB" -s "$SERIAL" shell settings get global preferred_network_mode6 | tr -d '\r')
  oplus_mode6=$("$ADB" -s "$SERIAL" shell settings get global oplus_user_preferred_network_mode6 | tr -d '\r')
  echo "OPLUS_OPERATOR_SELECT_FINAL_MODE=$mode"
  echo "OPLUS_OPERATOR_SELECT_FINAL_MASK=$mask"
  echo "OPLUS_OPERATOR_SELECT_FINAL_MODE6=$mode6"
  echo "OPLUS_OPERATOR_SELECT_FINAL_OPLUS_MODE6=$oplus_mode6"
  [[ "$mode" == 1 ]]
  [[ "$mask" == "$FULL_MASK_TEXT" ]]
  [[ "$mode6" == 33 && "$oplus_mode6" == 33 ]]
  state_may_change=0
  echo OPLUS_OPERATOR_SELECT_STATE_RESTORE=PASS
}

on_exit() {
  local rc=$?
  trap - EXIT INT TERM HUP
  set +e
  restore_state
  local restore_rc=$?
  if [[ "$rc" == 0 && "$restore_rc" != 0 ]]; then
    rc=$restore_rc
  fi
  exit "$rc"
}
trap on_exit EXIT INT TERM HUP

[[ $("$ADB" -s "$SERIAL" get-state) == device ]]
finish_operator_activity
before=$(run_manual get)
echo "$before"
grep -q ' mode=1$' <<<"$before"

"$ADB" -s "$SERIAL" shell input keyevent 224 >/dev/null
"$ADB" -s "$SERIAL" shell wm dismiss-keyguard >/dev/null 2>&1 || true
"$ADB" -s "$SERIAL" shell logcat -b all -c
state_may_change=1

echo "OPLUS_OPERATOR_SELECT_SCAN_START_UTC=$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)"
"$ADB" -s "$SERIAL" shell \
  "su -c 'am start -W -n com.android.phone/.OplusNetworkSetting --ei subscription $SLOT_ID'" |
  tr -d '\r'

complete=0
for _ in $(seq 1 36); do
  sleep 5
  "$ADB" -s "$SERIAL" shell uiautomator dump "/sdcard/pavonis_oplus_operator_select_${STAMP}.xml" >/dev/null
  "$ADB" -s "$SERIAL" pull "/sdcard/pavonis_oplus_operator_select_${STAMP}.xml" "$SCAN_XML" >/dev/null
  title_count=$(grep -o 'resource-id="android:id/title"' "$SCAN_XML" | wc -l || true)
  if grep -q 'text="Available networks"' "$SCAN_XML" &&
     ! grep -q 'text="Searching' "$SCAN_XML" &&
     (( title_count >= 3 )); then
    complete=1
    break
  fi
done
echo "OPLUS_OPERATOR_SELECT_SCAN_END_UTC=$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)"
echo "OPLUS_OPERATOR_SELECT_SCAN_COMPLETE=$complete"
[[ "$complete" == 1 ]]

python3 - "$SCAN_XML" <<'PY'
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
for node in root.iter("node"):
    if node.get("resource-id") == "android:id/title" and node.get("text"):
        print("OPLUS_OPERATOR_SELECT_UI_TITLE=" + node.get("text"))
PY

coords=$(python3 - "$SCAN_XML" "$TARGET_TITLE" <<'PY'
import re
import sys
import xml.etree.ElementTree as ET

root = ET.parse(sys.argv[1]).getroot()
target = sys.argv[2]
parents = {child: parent for parent in root.iter() for child in parent}
matches = [
    node for node in root.iter("node")
    if node.get("resource-id") == "android:id/title"
    and node.get("text") == target
    and node.get("enabled") == "true"
]
if len(matches) != 1 or "Forbidden" in target:
    raise SystemExit(f"expected one enabled non-forbidden target, got {len(matches)}")
node = matches[0]
row = node
while row in parents and row.get("clickable") != "true":
    row = parents[row]
if row.get("clickable") != "true" or row.get("enabled") != "true":
    raise SystemExit("target has no enabled clickable ancestor")
match = re.fullmatch(r"\[(\d+),(\d+)\]\[(\d+),(\d+)\]", row.get("bounds", ""))
if not match:
    raise SystemExit("invalid target row bounds")
x1, y1, x2, y2 = map(int, match.groups())
print((x1 + x2) // 2, (y1 + y2) // 2)
PY
)
read -r tap_x tap_y <<<"$coords"
echo OPLUS_OPERATOR_SELECT_TARGET_COUNT=1
echo "OPLUS_OPERATOR_SELECT_TARGET_TITLE=$TARGET_TITLE"
echo "OPLUS_OPERATOR_SELECT_TARGET_TAP=$tap_x,$tap_y"
echo "OPLUS_OPERATOR_SELECT_TAP_UTC=$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)"
"$ADB" -s "$SERIAL" shell input tap "$tap_x" "$tap_y"
echo OPLUS_OPERATOR_SELECT_TAP_ISSUED=1

start_epoch=$(date +%s)
last_mode=''
last_top=''
activity_finished=0
while (( $(date +%s) - start_epoch < OBSERVE_SEC )); do
  sleep 5
  mode=$(run_manual get | sed -n 's/.* mode=\([0-9][0-9]*\).*/\1/p' | tail -n 1)
  top=$("$ADB" -s "$SERIAL" shell dumpsys activity activities |
    sed -n 's/.*topResumedActivity=//p' | head -n 1 | tr -d '\r')
  if [[ "$mode" != "$last_mode" ]]; then
    echo "OPLUS_OPERATOR_SELECT_MODE_TRANSITION elapsed=$(( $(date +%s) - start_epoch )) mode=$mode"
    last_mode=$mode
  fi
  if [[ "$top" != "$last_top" ]]; then
    echo "OPLUS_OPERATOR_SELECT_TOP_TRANSITION elapsed=$(( $(date +%s) - start_epoch )) top=$top"
    last_top=$top
  fi
  if [[ "$top" != *OplusNetworkSetting* ]]; then
    activity_finished=1
  fi
done

"$ADB" -s "$SERIAL" shell uiautomator dump "/sdcard/pavonis_oplus_operator_select_state_${STAMP}.xml" >/dev/null 2>&1 || true
"$ADB" -s "$SERIAL" pull "/sdcard/pavonis_oplus_operator_select_state_${STAMP}.xml" "$STATE_XML" >/dev/null 2>&1 || true
if [[ -s "$STATE_XML" ]]; then
  python3 - "$STATE_XML" <<'PY'
import sys
import xml.etree.ElementTree as ET
root = ET.parse(sys.argv[1]).getroot()
texts = []
for node in root.iter("node"):
    text = node.get("text", "")
    if text and text not in texts:
        texts.append(text)
for text in texts[:30]:
    print("OPLUS_OPERATOR_SELECT_FINAL_UI_TEXT=" + text)
PY
fi

final_before_restore=$(run_manual get)
echo "$final_before_restore"
echo "OPLUS_OPERATOR_SELECT_ACTIVITY_FINISHED=$activity_finished"
echo "OPLUS_OPERATOR_SELECT_OBSERVE_END_UTC=$(date -u +%Y-%m-%dT%H:%M:%S.%NZ)"
"$ADB" -s "$SERIAL" shell logcat -b all -d -v epoch 2>/dev/null |
  grep -Ei 'manual network selection|setNetworkSelection|SET_NETWORK_SELECTION_MANUAL|INTERNAL_ERR|ILLEGAL_SIM|registration done' |
  tail -n 200 || true
echo OPLUS_OPERATOR_SELECT_REMOTE=PASS

restore_state
trap - EXIT INT TERM HUP
