#!/usr/bin/env bash
set -euo pipefail

SERIAL="${1:-${PAVONIS_PHONE_ADB_SERIAL:?pass the authorized phone ADB serial}}"
PROBE="${PAVONIS_CP3032_PROBE:-0}"
ADB=(/usr/bin/adb -s "$SERIAL")
UI_XML="/sdcard/pavonis_cp3032_update_modal_${$}.xml"

[[ "$SERIAL" =~ ^[A-Za-z0-9._:-]+$ && "$PROBE" =~ ^[01]$ ]]
[[ "$("${ADB[@]}" get-state)" == device ]]
command -v python3 >/dev/null

cleanup() {
  "${ADB[@]}" shell rm -f "$UI_XML" >/dev/null 2>&1 || true
}
trap cleanup EXIT

if [[ "$PROBE" == 1 ]]; then
  echo "CP3032_UPDATE_MODAL_PROBE=PASS adb=device parser=xml_exact"
  exit 0
fi

"${ADB[@]}" shell input keyevent 224 >/dev/null
"${ADB[@]}" shell wm dismiss-keyguard >/dev/null 2>&1 || true
"${ADB[@]}" shell am force-stop com.sladjan.sava.petg >/dev/null
"${ADB[@]}" shell am start -W \
  -n com.sladjan.sava.petg/.MainActivity >/dev/null
sleep 2

dismissed=0
for _ in 1 2; do
  "${ADB[@]}" shell uiautomator dump "$UI_XML" >/dev/null
  result=$(
    "${ADB[@]}" exec-out cat "$UI_XML" |
      python3 -c '
import re
import sys
import xml.etree.ElementTree as ET

root = ET.fromstring(sys.stdin.read())
matches = [
    node
    for node in root.iter("node")
    if node.get("text") == "Remind me later"
    and node.get("clickable") == "true"
]
if not matches:
    print("ABSENT")
    raise SystemExit(0)
if len(matches) != 1:
    raise SystemExit(f"expected one safe update defer action, got {len(matches)}")
match = re.fullmatch(r"\[(\d+),(\d+)\]\[(\d+),(\d+)\]", matches[0].get("bounds", ""))
if not match:
    raise SystemExit("invalid safe update defer bounds")
x1, y1, x2, y2 = map(int, match.groups())
print((x1 + x2) // 2, (y1 + y2) // 2)
'
  )
  if [[ "$result" == ABSENT ]]; then
    break
  fi
  read -r tap_x tap_y <<<"$result"
  "${ADB[@]}" shell input tap "$tap_x" "$tap_y" >/dev/null
  dismissed=1
  sleep 2
done

"${ADB[@]}" shell uiautomator dump "$UI_XML" >/dev/null
remaining=$(
  "${ADB[@]}" exec-out cat "$UI_XML" |
    python3 -c '
import sys
import xml.etree.ElementTree as ET

root = ET.fromstring(sys.stdin.read())
print(sum(1 for node in root.iter("node") if node.get("text") == "Remind me later"))
'
)
[[ "$remaining" == 0 ]]
echo "CP3032_UPDATE_MODAL=PASS dismissed=$dismissed safe_action_absent=1"
