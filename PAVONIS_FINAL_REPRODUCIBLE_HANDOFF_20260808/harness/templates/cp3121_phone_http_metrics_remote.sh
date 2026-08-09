#!/usr/bin/env bash
set -euo pipefail

SERIAL="${1:?ADB serial}"
NONCE="${2:?nonce}"
PORT="${3:-39087}"
TRANSFER_BYTES="${4:-1048576}"
[[ "$SERIAL" =~ ^[A-Za-z0-9._:-]+$ ]]
[[ "$NONCE" =~ ^[A-Za-z0-9_-]+$ && "$PORT" =~ ^[0-9]+$ ]]
[[ "$TRANSFER_BYTES" =~ ^[0-9]+$ ]] && ((TRANSFER_BYTES >= 65536 && TRANSFER_BYTES <= 8388608))

ADB=(/usr/bin/adb -s "$SERIAL")
[[ "$("${ADB[@]}" get-state)" == device ]]
iface="$("${ADB[@]}" shell ip -o -4 addr show | tr -d '\r' | awk '$4 ~ /^10\.255\.0\.2\// {print $2; exit}')"
[[ "$iface" =~ ^[A-Za-z0-9_.-]+$ ]]
upload_path="/data/local/tmp/pavonis_cp3121_upload.bin"

cleanup() {
  "${ADB[@]}" shell "su -c 'rm -f $upload_path'" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM HUP

read_counter() {
  local name="$1"
  "${ADB[@]}" shell "su -c 'cat /sys/class/net/$iface/statistics/$name'" | tr -d '\r'
}

parse_metric() {
  local line="$1" name="$2"
  sed -n "s/.*${name}=\\([^[:space:]]*\\).*/\\1/p" <<<"$line"
}

rx_before="$(read_counter rx_bytes)"
tx_before="$(read_counter tx_bytes)"
[[ "$rx_before" =~ ^[0-9]+$ && "$tx_before" =~ ^[0-9]+$ ]]

"${ADB[@]}" shell "su -c 'dd if=/dev/zero of=$upload_path bs=65536 count=$((TRANSFER_BYTES / 65536)) 2>/dev/null; chmod 600 $upload_path'"
phone_upload_size="$("${ADB[@]}" shell "su -c 'wc -c < $upload_path'" | tr -d '\r[:space:]')"
[[ "$phone_upload_size" == "$TRANSFER_BYTES" ]]

set +e
route_output="$("${ADB[@]}" shell "su -c 'ip route get 10.255.0.1 oif $iface'" 2>&1 | tr -d '\r')"
route_rc=$?
echo_output="$("${ADB[@]}" shell "su -c 'curl --interface $iface --noproxy \"*\" --connect-timeout 5 --max-time 15 --silent --show-error --write-out \"\\nCP3121_CURL time_total=%{time_total} size_download=%{size_download} speed_download=%{speed_download}\" http://10.255.0.1:$PORT/pavonis/$NONCE'" 2>&1 | tr -d '\r')"
echo_rc=$?
download_output="$("${ADB[@]}" shell "su -c 'curl --interface $iface --noproxy \"*\" --connect-timeout 5 --max-time 120 --silent --show-error --output /dev/null --write-out \"CP3121_CURL time_total=%{time_total} size_download=%{size_download} speed_download=%{speed_download}\" http://10.255.0.1:$PORT/pavonis/$NONCE/download'" 2>&1 | tr -d '\r')"
download_rc=$?
upload_output="$("${ADB[@]}" shell "su -c 'curl --interface $iface --noproxy \"*\" --connect-timeout 5 --max-time 180 --silent --show-error --data-binary @$upload_path --write-out \"\\nCP3121_CURL time_total=%{time_total} size_upload=%{size_upload} speed_upload=%{speed_upload}\" http://10.255.0.1:$PORT/pavonis/$NONCE/upload'" 2>&1 | tr -d '\r')"
upload_rc=$?
set -e

rx_after="$(read_counter rx_bytes)"
tx_after="$(read_counter tx_bytes)"
[[ "$rx_after" =~ ^[0-9]+$ && "$tx_after" =~ ^[0-9]+$ ]]
rx_delta=$((rx_after - rx_before))
tx_delta=$((tx_after - tx_before))

route_class=other
if ((route_rc == 0)) && grep -Eq "(^|[[:space:]])dev[[:space:]]+$iface([[:space:]]|$)" <<<"$route_output"; then
  route_class=ok
elif grep -Eqi 'network is unreachable|unreachable' <<<"$route_output"; then
  route_class=network_unreachable
fi

echo_body="$(sed '$d' <<<"$echo_output")"
echo_metrics="$(tail -n 1 <<<"$echo_output")"
upload_body="$(sed '$d' <<<"$upload_output")"
upload_metrics="$(tail -n 1 <<<"$upload_output")"
echo_match=0
upload_match=0
[[ "$echo_body" == "PAVONIS_CP3121_ECHO=$NONCE" ]] && echo_match=1
[[ "$upload_body" == "PAVONIS_CP3121_UPLOAD=PASS" ]] && upload_match=1

echo_time_s="$(parse_metric "$echo_metrics" time_total)"
download_time_s="$(parse_metric "$download_output" time_total)"
download_bytes="$(parse_metric "$download_output" size_download)"
download_bps="$(parse_metric "$download_output" speed_download)"
upload_time_s="$(parse_metric "$upload_metrics" time_total)"
upload_bytes="$(parse_metric "$upload_metrics" size_upload)"
upload_bps="$(parse_metric "$upload_metrics" speed_upload)"
for value in "$echo_time_s" "$download_time_s" "$download_bps" "$upload_time_s" "$upload_bps"; do
  [[ "$value" =~ ^[0-9]+([.][0-9]+)?$ ]]
done
[[ "$download_bytes" =~ ^[0-9]+$ && "$upload_bytes" =~ ^[0-9]+$ ]]

pass=0
if [[ "$route_class" == ok ]] &&
   ((echo_rc == 0 && download_rc == 0 && upload_rc == 0 && echo_match == 1 && upload_match == 1)) &&
   ((download_bytes == TRANSFER_BYTES && upload_bytes == TRANSFER_BYTES && rx_delta > 0 && tx_delta > 0)); then
  pass=1
fi

printf 'CP3121_PHONE_HTTP iface=%s route_rc=%s route_class=%s echo_rc=%s echo_match=%s echo_time_s=%s download_rc=%s download_bytes=%s download_time_s=%s download_Bps=%s upload_rc=%s upload_match=%s upload_bytes=%s upload_time_s=%s upload_Bps=%s rx_delta=%s tx_delta=%s\n' \
  "$iface" "$route_rc" "$route_class" "$echo_rc" "$echo_match" "$echo_time_s" \
  "$download_rc" "$download_bytes" "$download_time_s" "$download_bps" \
  "$upload_rc" "$upload_match" "$upload_bytes" "$upload_time_s" "$upload_bps" \
  "$rx_delta" "$tx_delta"
echo "CP3121_PHONE_HTTP_PASS=$pass"
exit "$((pass == 0))"
