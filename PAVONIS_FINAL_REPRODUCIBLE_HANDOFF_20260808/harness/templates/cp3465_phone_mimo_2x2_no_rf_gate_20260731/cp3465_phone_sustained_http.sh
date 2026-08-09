#!/usr/bin/env bash
set -euo pipefail

DURATION="${1:-45}"
BENCH_PORT="${2:-39088}"
RESULT="${3:?result path required}"
[[ "$DURATION" =~ ^[0-9]+$ ]] && ((DURATION >= 30 && DURATION <= 60))
[[ "$BENCH_PORT" =~ ^[0-9]+$ ]]

mapfile -t devices < <(
  /usr/bin/adb devices |
    tail -n +2 |
    grep -E '[[:space:]]device$' |
    cut -f1
)
[[ "${#devices[@]}" == 1 ]]
serial="${devices[0]}"
ADB=(/usr/bin/adb -s "$serial")
[[ "$("${ADB[@]}" get-state)" == device ]]

upload_path="/data/local/tmp/pavonis_sustained_upload.bin"
metrics_dir="$(mktemp -d /tmp/pavonis_cp3155_phone.XXXXXX)"
cleanup() {
  "${ADB[@]}" shell "su -c 'rm -f $upload_path'" >/dev/null 2>&1 || true
  rm -rf "$metrics_dir"
}
trap cleanup EXIT INT TERM HUP

discover_path() {
  local deadline=$((SECONDS + 240))
  local line iface cidr gateway
  while ((SECONDS < deadline)); do
    while read -r line; do
      [[ -n "$line" ]] || continue
      iface="${line%% *}"
      cidr="${line#* }"
      [[ "$iface" =~ ^(rmnet|ccmni)[A-Za-z0-9_.-]*$ ]] || continue
      gateway="$(
        python3 - "$cidr" <<'PY'
import ipaddress
import sys

interface = ipaddress.ip_interface(sys.argv[1])
print(interface.network.network_address + 1)
PY
      )"
      if "${ADB[@]}" shell \
          "su -c 'curl --interface $iface --noproxy \"*\" --connect-timeout 1 --max-time 2 --silent http://$gateway:$BENCH_PORT/ready >/dev/null'" \
          </dev/null >/dev/null 2>&1; then
        printf '%s %s\n' "$iface" "$gateway"
        return 0
      fi
    done < <(
      "${ADB[@]}" shell "su -c 'ip -o -4 addr show'" |
        tr -d '\r' |
        awk '{print $2, $4}'
    )
    sleep 1
  done
  return 1
}

read -r iface gateway < <(discover_path)
echo "CP3153_PHONE_PATH_DISCOVERED=1"
echo "CP3153_BENCHMARK_PATH_READY=1"

bench_ready=0
for _ in $(seq 1 60); do
  if "${ADB[@]}" shell \
      "su -c 'curl --interface $iface --noproxy \"*\" --connect-timeout 1 --max-time 2 --silent http://$gateway:$BENCH_PORT/ready >/dev/null'" \
      </dev/null >/dev/null 2>&1; then
    bench_ready=1
    break
  fi
  sleep 1
done
[[ "$bench_ready" == 1 ]]

"${ADB[@]}" shell \
  "su -c 'dd if=/dev/zero of=$upload_path bs=1048576 count=16 2>/dev/null; chmod 600 $upload_path'" \
  </dev/null

read_counter() {
  local name="$1"
  "${ADB[@]}" shell "su -c 'cat /sys/class/net/$iface/statistics/$name'" |
    tr -d '\r[:space:]'
}

rx_before="$(read_counter rx_bytes)"
set +e
"${ADB[@]}" shell \
  "su -c 'curl --interface $iface --noproxy \"*\" --connect-timeout 5 --max-time $DURATION --silent --output /dev/null --write-out \"size=%{size_download} speed=%{speed_download} time=%{time_total}\" http://$gateway:$BENCH_PORT/download'" \
  </dev/null >"$metrics_dir/downlink.txt" 2>"$metrics_dir/downlink.err"
downlink_rc=$?
set -e
rx_after="$(read_counter rx_bytes)"

tx_before="$(read_counter tx_bytes)"
set +e
"${ADB[@]}" shell \
  "su -c 'curl --interface $iface --noproxy \"*\" --connect-timeout 5 --max-time $((DURATION + 10)) --silent -H \"Expect:\" -H \"X-Pavonis-Duration: $DURATION\" --data-binary @$upload_path --write-out \"size=%{size_upload} speed=%{speed_upload} time=%{time_total}\" http://$gateway:$BENCH_PORT/upload'" \
  </dev/null >"$metrics_dir/uplink.txt" 2>"$metrics_dir/uplink.err"
uplink_rc=$?
set -e
tx_after="$(read_counter tx_bytes)"

downlink_metrics="$(tr -d '\r\n' <"$metrics_dir/downlink.txt")"
uplink_metrics="$(tr -d '\r\n' <"$metrics_dir/uplink.txt")"
parse_metric() {
  local text="$1" key="$2"
  sed -n "s/.*${key}=\\([0-9.]*\\).*/\\1/p" <<<"$text"
}
downlink_bytes="$(parse_metric "$downlink_metrics" size)"
downlink_speed="$(parse_metric "$downlink_metrics" speed)"
downlink_time="$(parse_metric "$downlink_metrics" time)"
uplink_bytes="$(parse_metric "$uplink_metrics" size)"
uplink_speed="$(parse_metric "$uplink_metrics" speed)"
uplink_time="$(parse_metric "$uplink_metrics" time)"
for value in "$downlink_bytes" "$downlink_speed" "$downlink_time" \
  "$uplink_bytes" "$uplink_speed" "$uplink_time" "$rx_before" "$rx_after" \
  "$tx_before" "$tx_after"; do
  [[ "$value" =~ ^[0-9]+([.][0-9]+)?$ ]]
done
((downlink_rc == 28))
case "$uplink_rc" in
  0|18|28|52|55|56) ;;
  *) exit 1 ;;
esac

python3 - "$RESULT" "$DURATION" "$downlink_rc" "$downlink_bytes" \
  "$downlink_speed" "$downlink_time" "$((rx_after - rx_before))" \
  "$uplink_rc" "$uplink_bytes" "$uplink_speed" "$uplink_time" \
  "$((tx_after - tx_before))" <<'PY'
import json
from pathlib import Path
import sys

(
    output,
    duration,
    dl_rc,
    dl_bytes,
    dl_speed,
    dl_time,
    dl_rx_delta,
    ul_rc,
    ul_bytes,
    ul_speed,
    ul_time,
    ul_tx_delta,
) = sys.argv[1:]
payload = {
    "schema": 1,
    "complete": (
        int(dl_rc) == 28
        and int(ul_rc) in {0, 18, 28, 52, 55, 56}
        and float(dl_time) >= int(duration) - 1
        and float(ul_time) >= int(duration) - 1
    ),
    "requested_seconds_per_direction": int(duration),
    "server_enforced_uplink_deadline": True,
    "downlink": {
        "curl_rc": int(dl_rc),
        "bytes": int(float(dl_bytes)),
        "bytes_per_second": float(dl_speed),
        "seconds": float(dl_time),
        "interface_rx_delta": int(dl_rx_delta),
    },
    "uplink": {
        "curl_rc": int(ul_rc),
        "bytes": int(float(ul_bytes)),
        "bytes_per_second": float(ul_speed),
        "seconds": float(ul_time),
        "interface_tx_delta": int(ul_tx_delta),
    },
}
Path(output).write_text(
    json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
)
PY

echo "CP3153_PHONE_SUSTAINED_COMPLETE=1"
