#!/usr/bin/env bash
set -euo pipefail

SERIAL="${1:?ADB serial}"
DURATION_SEC="${2:-30}"
MODE="${3:-run}"

[[ "$SERIAL" =~ ^[A-Za-z0-9._:-]+$ ]]
[[ "$DURATION_SEC" =~ ^[0-9]+$ ]] && ((DURATION_SEC >= 10 && DURATION_SEC <= 60))
[[ "$MODE" == run || "$MODE" == probe ]]

ADB=(/usr/bin/adb -s "$SERIAL")
[[ "$("${ADB[@]}" get-state)" == device ]]

if [[ "$MODE" == probe ]]; then
  "${ADB[@]}" shell "su -c 'command -v ping >/dev/null'" >/dev/null
  echo "CP2901_PHONE_SERVICE_TRIGGER_PROBE=PASS duration_sec=$DURATION_SEC"
  exit 0
fi

iface="$("${ADB[@]}" shell ip -o -4 addr show | tr -d '\r' |
  awk '$4 ~ /^10\.255\.0\.2\// {print $2; exit}')"
if [[ ! "$iface" =~ ^[A-Za-z0-9_.-]+$ ]]; then
  echo "CP2901_PHONE_SERVICE_TRIGGER status=no_private_iface attempts=0 replies=0 tx_delta=0 rx_delta=0"
  echo CP2901_PHONE_SERVICE_TRIGGER_PASS=0
  exit 0
fi

read_counter() {
  local name="$1"
  "${ADB[@]}" shell "su -c 'cat /sys/class/net/$iface/statistics/$name'" | tr -d '\r'
}

tx_before="$(read_counter tx_bytes)"
rx_before="$(read_counter rx_bytes)"
[[ "$tx_before" =~ ^[0-9]+$ && "$rx_before" =~ ^[0-9]+$ ]]

deadline=$((SECONDS + DURATION_SEC))
attempts=0
replies=0
while ((SECONDS < deadline)); do
  attempts=$((attempts + 1))
  set +e
  "${ADB[@]}" shell "su -c 'ping -I $iface -c 1 -W 1 10.255.0.1 >/dev/null 2>&1'" >/dev/null 2>&1
  ping_rc=$?
  set -e
  ((ping_rc == 0)) && replies=$((replies + 1))
done

tx_after="$(read_counter tx_bytes)"
rx_after="$(read_counter rx_bytes)"
[[ "$tx_after" =~ ^[0-9]+$ && "$rx_after" =~ ^[0-9]+$ ]]
tx_delta=$((tx_after - tx_before))
rx_delta=$((rx_after - rx_before))

pass=0
((attempts >= 5)) && pass=1
printf 'CP2901_PHONE_SERVICE_TRIGGER status=sent iface=%s attempts=%s replies=%s tx_delta=%s rx_delta=%s\n' \
  "$iface" "$attempts" "$replies" "$tx_delta" "$rx_delta"
echo "CP2901_PHONE_SERVICE_TRIGGER_PASS=$pass"
exit 0
