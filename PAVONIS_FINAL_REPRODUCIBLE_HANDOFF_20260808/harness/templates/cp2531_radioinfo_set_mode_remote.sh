#!/usr/bin/env bash
set -euo pipefail

SERIAL="${1:?usage: cp2531_radioinfo_set_mode_remote.sh SERIAL nr_only|full [via_app]}"
MODE="${2:?usage: cp2531_radioinfo_set_mode_remote.sh SERIAL nr_only|full [via_app]}"
ENTRY="${3:-direct}"
ADB=/usr/bin/adb
BASE_XML="/sdcard/pavonis_cp2531_radioinfo_base_${$}.xml"
POPUP_XML="/sdcard/pavonis_cp2531_radioinfo_popup_${$}.xml"
FULL_MASK_TEXT='GPRS|EDGE|UMTS|CDMA|CDMA - EvDo rev. 0|CDMA - EvDo rev. A|CDMA - 1xRTT|HSDPA|HSUPA|HSPA|CDMA - EvDo rev. B|LTE|CDMA - eHRPD|HSPA+|GSM|TD_SCDMA|LTE_CA|NR'

state=$("$ADB" devices | awk -v s="$SERIAL" '$1 == s {print $2; exit}')
[[ "$state" == device ]]
"$ADB" -s "$SERIAL" shell input keyevent 224 >/dev/null
"$ADB" -s "$SERIAL" shell wm dismiss-keyguard >/dev/null 2>&1 || true

case "$MODE" in
  nr_only)
    target='NR only'
    ;;
  full)
    target='NR/LTE/TDSCDMA/CDMA/EvDo/GSM/WCDMA'
    ;;
  *)
    echo "unsupported mode: $MODE" >&2
    exit 2
    ;;
esac

if [[ "$ENTRY" == via_app ]]; then
  "$ADB" -s "$SERIAL" shell am force-stop com.sladjan.sava.petg
  "$ADB" -s "$SERIAL" shell am start -W \
    -n com.sladjan.sava.petg/.MainActivity >/dev/null
  sleep 2
  "$ADB" -s "$SERIAL" shell input tap 540 1232
  sleep 1
  "$ADB" -s "$SERIAL" shell input tap 540 754
else
  "$ADB" -s "$SERIAL" shell am start -W \
    -n com.android.phone/.settings.RadioInfo >/dev/null
fi
sleep 2
"$ADB" -s "$SERIAL" shell dumpsys window |
  grep -q 'com.android.phone.settings.RadioInfo'

if "$ADB" -s "$SERIAL" shell dumpsys window |
   grep -q 'mCurrentFocus=.*PopupWindow'; then
  "$ADB" -s "$SERIAL" shell input keyevent 4
  sleep 1
fi

"$ADB" -s "$SERIAL" shell uiautomator dump "$BASE_XML" >/dev/null
spinner_coords=$(
  "$ADB" -s "$SERIAL" exec-out cat "$BASE_XML" |
    python3 -c '
import re
import sys
import xml.etree.ElementTree as ET

root = ET.fromstring(sys.stdin.read())
matches = [
    node
    for node in root.iter("node")
    if node.get("resource-id") == "com.android.phone:id/preferredNetworkType"
    and node.get("class") == "android.widget.Spinner"
]
if len(matches) != 1:
    raise SystemExit(f"expected one preferred-network spinner, got {len(matches)}")
match = re.fullmatch(r"\[(\d+),(\d+)\]\[(\d+),(\d+)\]", matches[0].get("bounds", ""))
if not match:
    raise SystemExit("invalid spinner bounds")
x1, y1, x2, y2 = map(int, match.groups())
print((x1 + x2) // 2, (y1 + y2) // 2)
'
)
read -r spinner_x spinner_y <<<"$spinner_coords"
"$ADB" -s "$SERIAL" shell input tap "$spinner_x" "$spinner_y"
sleep 1
"$ADB" -s "$SERIAL" shell uiautomator dump "$POPUP_XML" >/dev/null

coords=$(
  "$ADB" -s "$SERIAL" exec-out cat "$POPUP_XML" |
    python3 -c '
import re
import sys
import xml.etree.ElementTree as ET

target = sys.argv[1]
root = ET.fromstring(sys.stdin.read())
matches = [
    node
    for node in root.iter("node")
    if node.get("text") == target
    and node.get("class") == "android.widget.CheckedTextView"
]
if len(matches) != 1:
    raise SystemExit(f"expected one {target!r} row, got {len(matches)}")
match = re.fullmatch(r"\[(\d+),(\d+)\]\[(\d+),(\d+)\]", matches[0].get("bounds", ""))
if not match:
    raise SystemExit("invalid row bounds")
x1, y1, x2, y2 = map(int, match.groups())
print((x1 + x2) // 2, (y1 + y2) // 2)
' "$target"
)
read -r tap_x tap_y <<<"$coords"
"$ADB" -s "$SERIAL" shell input tap "$tap_x" "$tap_y"
sleep 4
"$ADB" -s "$SERIAL" shell rm -f "$BASE_XML" "$POPUP_XML"

actual=$("$ADB" -s "$SERIAL" shell \
  cmd phone get-allowed-network-types-for-users -s 0 | tr -d '\r')
mode6=$("$ADB" -s "$SERIAL" shell \
  settings get global preferred_network_mode6 | tr -d '\r')
oplus_mode6=$("$ADB" -s "$SERIAL" shell \
  settings get global oplus_user_preferred_network_mode6 | tr -d '\r')

if [[ "$MODE" == nr_only ]]; then
  [[ "$actual" == NR ]]
  [[ "$mode6" == 23 ]]
  [[ "$oplus_mode6" == 33 ]]
else
  [[ "$actual" == "$FULL_MASK_TEXT" ]]
  [[ "$mode6" == 33 ]]
  [[ "$oplus_mode6" == 33 ]]
fi

echo "RADIOINFO_MODE=$MODE"
echo "RADIOINFO_ENTRY=$ENTRY"
echo "RADIOINFO_TARGET=$target"
echo "RADIOINFO_SPINNER_TAP=$spinner_x,$spinner_y"
echo "RADIOINFO_TAP=$tap_x,$tap_y"
echo "RADIOINFO_ALLOWED=$actual"
echo "RADIOINFO_MODE6=$mode6"
echo "RADIOINFO_OPLUS_MODE6=$oplus_mode6"
echo "RADIOINFO_SET_MODE=PASS"
