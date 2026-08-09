#!/usr/bin/env bash
set -u

NETNS="${PAVONIS_DATA_PROBE_NETNS:-ue1}"
DEV="${PAVONIS_DATA_PROBE_DEV:-tun_ue1}"
UE_ADDR="${PAVONIS_DATA_PROBE_UE_ADDR:-10.255.0.2}"
GATEWAY="${PAVONIS_DATA_PROBE_GATEWAY:-10.255.0.1}"
MAX_WAIT_SECONDS="${PAVONIS_DATA_PROBE_MAX_WAIT_SECONDS:-380}"

if [[ "${1:-}" == "--self-test" ]]; then
  for cmd in awk grep ip ping python3 sudo; do
    command -v "$cmd" >/dev/null || {
      echo "SELF_TEST=FAIL missing=$cmd"
      exit 1
    }
  done
  echo "SELF_TEST=PASS"
  exit 0
fi

read_counters()
{
  sudo -n ip netns exec "$NETNS" ip -s link show dev "$DEV" 2>/dev/null |
    awk '/RX:/{getline; rx=$2} /TX:/{getline; tx=$2} END{print rx+0, tx+0}'
}

address_is_ready()
{
  sudo -n ip netns exec "$NETNS" ip -4 -o addr show dev "$DEV" 2>/dev/null |
    grep -Fq "$UE_ADDR/"
}

echo "PROBE_MODE=event_driven_netlink_add_only"
echo "PROBE_START=$(date -u +%Y-%m-%dT%H:%M:%SZ)"

deadline=$((SECONDS + MAX_WAIT_SECONDS))
while [[ ! -e "/var/run/netns/$NETNS" ]]; do
  if ((SECONDS >= deadline)); then
    echo "DATA_PROBE=NO_NETNS"
    exit 42
  fi
  sleep 0.5
done
echo "NETNS_SEEN_AT=$(date -u +%Y-%m-%dT%H:%M:%SZ)"

event_file="/tmp/pavonis_data_probe_address_event_$$.log"
trap 'rm -f "$event_file"' EXIT

if address_is_ready; then
  echo "ADDRESS_SOURCE=initial_snapshot"
  sudo -n ip netns exec "$NETNS" ip -4 -o addr show dev "$DEV" >"$event_file"
else
  remaining=$((deadline - SECONDS))
  if ((remaining <= 0)); then
    echo "DATA_PROBE=NO_ADDRESS"
    exit 42
  fi

  if ! sudo -n ip netns exec "$NETNS" \
    python3 - "$UE_ADDR" "$remaining" >"$event_file" <<'PY'
import select
import socket
import struct
import sys
import time

target = socket.inet_aton(sys.argv[1])
deadline = time.monotonic() + float(sys.argv[2])
sock = socket.socket(socket.AF_NETLINK, socket.SOCK_RAW, socket.NETLINK_ROUTE)
sock.bind((0, 0x10))  # RTMGRP_IPV4_IFADDR

while True:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise SystemExit(42)
    readable, _, _ = select.select([sock], [], [], remaining)
    if not readable:
        raise SystemExit(42)

    data = sock.recv(65535)
    offset = 0
    while offset + 16 <= len(data):
        length, msg_type, _, _, _ = struct.unpack_from("IHHII", data, offset)
        if length < 16:
            break
        end = offset + length
        if msg_type == 20 and offset + 24 <= end:  # RTM_NEWADDR
            family, prefix, _, _, ifindex = struct.unpack_from(
                "BBBBI", data, offset + 16
            )
            attr_offset = offset + 24
            while attr_offset + 4 <= end:
                attr_len, attr_type = struct.unpack_from("HH", data, attr_offset)
                if attr_len < 4:
                    break
                value = data[attr_offset + 4 : attr_offset + attr_len]
                if family == socket.AF_INET and attr_type in (1, 2) and value[:4] == target:
                    print(
                        "RTM_NEWADDR"
                        f" ifindex={ifindex}"
                        f" ifname={socket.if_indextoname(ifindex)}"
                        f" address={sys.argv[1]}"
                        f" prefixlen={prefix}",
                        flush=True,
                    )
                    raise SystemExit(0)
                attr_offset += (attr_len + 3) & ~3
        offset += (length + 3) & ~3
PY
  then
    echo "DATA_PROBE=NO_ADDRESS_ADD_EVENT"
    exit 42
  fi
  echo "ADDRESS_SOURCE=netlink_add"
fi

echo "ADDRESS_EVENT=$(sed -n '1p' "$event_file")"
echo "ADDRESS_SEEN_AT=$(date -u +%Y-%m-%dT%H:%M:%SZ)"

ready=0
for ((attempt = 0; attempt < 40; attempt++)); do
  if address_is_ready; then
    ready=1
    break
  fi
  sleep 0.05
done
if ((ready == 0)); then
  echo "DATA_PROBE=ADDRESS_EVENT_WITHOUT_LIVE_DEVICE"
  exit 42
fi

sudo -n ip netns exec "$NETNS" ip -4 -o addr show dev "$DEV"
read -r rx0 tx0 < <(read_counters)
echo "COUNTERS_BEFORE_RX=$rx0 TX=$tx0"

sudo -n ip netns exec "$NETNS" \
  ping -q -I "$DEV" -c 10 -i 0.1 -W 1 "$GATEWAY"
ping_rc=$?

read -r rx1 tx1 < <(read_counters)
echo "COUNTERS_AFTER_RX=$rx1 TX=$tx1"
echo "COUNTER_DELTA_RX=$((rx1 - rx0)) TX=$((tx1 - tx0))"
echo "PING_RC=$ping_rc"

if ((ping_rc == 0)); then
  echo "DATA_PROBE=PASS"
else
  echo "DATA_PROBE=PING_FAIL"
fi
exit "$ping_rc"
