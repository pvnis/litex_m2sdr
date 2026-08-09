#!/usr/bin/env bash
set -euo pipefail

route_output="$(ip route get 10.255.0.2 2>&1)"
route_rc=$?
iface="$(awk '{for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1); exit}}' <<<"$route_output")"
source_ip="$(awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}' <<<"$route_output")"
[[ "$iface" =~ ^[A-Za-z0-9_.-]+$ && "$source_ip" == 10.255.0.1 ]]
ip link show "$iface" >/dev/null
rx_before="$(cat "/sys/class/net/$iface/statistics/rx_bytes")"
tx_before="$(cat "/sys/class/net/$iface/statistics/tx_bytes")"
[[ "$rx_before" =~ ^[0-9]+$ && "$tx_before" =~ ^[0-9]+$ ]]

set +e
ping_output="$(ping -I "$source_ip" -c 4 -W 2 10.255.0.2 2>&1)"
ping_rc=$?
set -e

rx_after="$(cat "/sys/class/net/$iface/statistics/rx_bytes")"
tx_after="$(cat "/sys/class/net/$iface/statistics/tx_bytes")"
[[ "$rx_after" =~ ^[0-9]+$ && "$tx_after" =~ ^[0-9]+$ ]]
rx_delta=$((rx_after - rx_before))
tx_delta=$((tx_after - tx_before))
received="$(awk -F, '/packets transmitted/ {gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2); split($2, fields, " "); print fields[1]; exit}' <<<"$ping_output" || true)"
[[ "$received" =~ ^[0-9]+$ ]] || received=0

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

printf 'CP2880_CORE_TO_PHONE iface=%s source=%s route_rc=%s route_class=%s ping_rc=%s failure_class=%s received=%s rx_delta=%s tx_delta=%s\n' \
  "$iface" "$source_ip" "$route_rc" "$route_class" "$ping_rc" "$failure_class" "$received" "$rx_delta" "$tx_delta"
echo "CP2880_CORE_TO_PHONE_PASS=$pass"
((pass == 0)) || echo CP2870_CORE_TO_PHONE=PASS
