#!/usr/bin/env bash
set -euo pipefail

mapfile -t serials < <(adb devices | awk '$2 == "device" {print $1}')
[[ "${#serials[@]}" == 1 ]]
serial="${serials[0]}"
adb_cmd=(adb -s "$serial")
iface="$("${adb_cmd[@]}" shell ip -o -4 addr show | tr -d '\r' |
  awk '$4 ~ /^10\.255\.0\.2\// {print $2; exit}')"
[[ "$iface" =~ ^[A-Za-z0-9_.-]+$ ]]

ping_count() {
  local target="$1" output received
  output="$("${adb_cmd[@]}" shell "su -c 'ping -I $iface -c 3 -W 3 $target'" |
    tr -d '\r')"
  received="$(awk -F, '/packets transmitted/ {
    gsub(/^[[:space:]]+|[[:space:]]+$/, "", $2)
    split($2, fields, " ")
    print fields[1]
    exit
  }' <<<"$output")"
  [[ "$received" =~ ^[0-9]+$ && "$received" -ge 1 ]]
  printf '%s\n' "$received"
}

gateway_received="$(ping_count 10.255.0.1)"
public_ip_received="$(ping_count 8.8.8.8)"
public_dns_received="$(ping_count google.com)"
http_code="$("${adb_cmd[@]}" shell \
  "su -c 'curl --interface $iface --silent --show-error --location --output /dev/null --write-out %{http_code} --connect-timeout 10 --max-time 30 https://www.google.com/generate_204'" |
  tr -d '\r\n')"
[[ "$http_code" == 204 ]]

printf 'PAVONIS_PHONE_PUBLIC_EGRESS=PASS iface=%s gateway_received=%s public_ip_received=%s public_dns_received=%s https_code=%s\n' \
  "$iface" "$gateway_received" "$public_ip_received" "$public_dns_received" "$http_code"
