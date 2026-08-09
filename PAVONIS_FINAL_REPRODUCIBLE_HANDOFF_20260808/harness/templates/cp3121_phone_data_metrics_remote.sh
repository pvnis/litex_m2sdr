#!/usr/bin/env bash
set -euo pipefail

SERIAL="${1:?ADB serial}"
ADB=(/usr/bin/adb -s "$SERIAL")

[[ "$("${ADB[@]}" get-state)" == device ]]
iface="$("${ADB[@]}" shell ip -o -4 addr show | tr -d '\r' | awk '$4 ~ /^10\.255\.0\.2\// {print $2; exit}')"
[[ "$iface" =~ ^[A-Za-z0-9_.-]+$ ]]

read_counter() {
  local name="$1"
  "${ADB[@]}" shell "su -c 'cat /sys/class/net/$iface/statistics/$name'" | tr -d '\r'
}

rx_before="$(read_counter rx_bytes)"
tx_before="$(read_counter tx_bytes)"
[[ "$rx_before" =~ ^[0-9]+$ && "$tx_before" =~ ^[0-9]+$ ]]

set +e
route_output="$("${ADB[@]}" shell "su -c 'ip route get 10.255.0.1 oif $iface'" 2>&1 | tr -d '\r')"
route_rc=$?
ping_output="$("${ADB[@]}" shell "su -c 'ping -I $iface -c 4 -W 2 10.255.0.1'" 2>&1 | tr -d '\r')"
ping_rc=$?
set -e

rx_after="$(read_counter rx_bytes)"
tx_after="$(read_counter tx_bytes)"
[[ "$rx_after" =~ ^[0-9]+$ && "$tx_after" =~ ^[0-9]+$ ]]
rx_delta=$((rx_after - rx_before))
tx_delta=$((tx_after - tx_before))
received="$(awk -F, '/packets transmitted/ {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); split($2, fields, " "); print fields[1]; exit}' <<<"$ping_output" || true)"
[[ "$received" =~ ^[0-9]+$ ]] || received=0

rtt_values="$(awk -F= '/^(rtt|round-trip)/ {
  gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2);
  split($2, fields, "/");
  if (length(fields) >= 4) {
    gsub(/[[:space:]]+/, "", fields[1]);
    gsub(/[[:space:]]+/, "", fields[2]);
    gsub(/[[:space:]]+/, "", fields[3]);
    gsub(/[[:space:]]+.*/, "", fields[4]);
    print fields[1], fields[2], fields[3], fields[4];
  }
  exit
}' <<<"$ping_output")"
read -r rtt_min_ms rtt_avg_ms rtt_max_ms rtt_mdev_ms <<<"$rtt_values"
for value in "$rtt_min_ms" "$rtt_avg_ms" "$rtt_max_ms" "$rtt_mdev_ms"; do
  [[ "$value" =~ ^[0-9]+([.][0-9]+)?$ ]]
done

route_class=other
if ((route_rc == 0)) && grep -Eq "(^|[[:space:]])dev[[:space:]]+$iface([[:space:]]|$)" <<<"$route_output"; then
  route_class=ok
elif grep -Eqi 'network is unreachable|unreachable' <<<"$route_output"; then
  route_class=network_unreachable
fi

failure_class=other
if ((ping_rc == 0 && received >= 1 && rx_delta > 0 && tx_delta > 0)); then
  failure_class=none
elif grep -Eqi 'network is unreachable|connect:.*unreachable' <<<"$ping_output"; then
  failure_class=network_unreachable
elif grep -Eqi 'permission denied|operation not permitted' <<<"$ping_output"; then
  failure_class=bind_permission
elif ((received == 0)); then
  failure_class=packet_loss
fi

pass=0
[[ "$failure_class" == none && "$route_class" == ok ]] && pass=1

printf 'CP3121_PHONE_ECHO iface=%s route_rc=%s route_class=%s ping_rc=%s failure_class=%s received=%s rx_delta=%s tx_delta=%s rtt_min_ms=%s rtt_avg_ms=%s rtt_max_ms=%s rtt_mdev_ms=%s\n' \
  "$iface" "$route_rc" "$route_class" "$ping_rc" "$failure_class" "$received" "$rx_delta" "$tx_delta" \
  "$rtt_min_ms" "$rtt_avg_ms" "$rtt_max_ms" "$rtt_mdev_ms"
echo "CP3121_PHONE_ECHO_PASS=$pass"
exit "$((pass == 0))"

