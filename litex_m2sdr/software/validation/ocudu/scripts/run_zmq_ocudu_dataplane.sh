#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_ocudu_validation.sh
source "$SCRIPT_DIR/env_ocudu_validation.sh"

UE_CONF=""
SIM_FILE=""
GNB_CONF="$VALIDATION_ROOT/configs/gnb_zmq_tdd_n78_20mhz.yml"
EXTERNAL_IFACE="${OCUDU_VALIDATION_EXTERNAL_IFACE:-wlp5s0}"
PING_TARGET="${OCUDU_VALIDATION_PING_TARGET:-8.8.8.8}"
ATTACH_TIMEOUT="${OCUDU_VALIDATION_ATTACH_TIMEOUT:-90}"
PING_COUNT="${OCUDU_VALIDATION_PING_COUNT:-5}"
ALLOWED_LOSS="${OCUDU_VALIDATION_ALLOWED_PACKET_LOSS:-0}"
TAG="zmq-dataplane"

usage() {
    echo "usage: $0 --ue-conf PRIVATE_CONF --sim-file PRIVATE_TOML [--gnb-conf FILE] [--external-iface IFACE] [--ping-target IP] [--attach-timeout SEC] [--allowed-loss PERCENT] [--tag TAG]" >&2
}
while (($#)); do
    case "$1" in
        --ue-conf) UE_CONF="${2:?missing path}"; shift 2 ;;
        --sim-file) SIM_FILE="${2:?missing path}"; shift 2 ;;
        --gnb-conf) GNB_CONF="${2:?missing path}"; shift 2 ;;
        --external-iface) EXTERNAL_IFACE="${2:?missing interface}"; shift 2 ;;
        --ping-target) PING_TARGET="${2:?missing address}"; shift 2 ;;
        --attach-timeout) ATTACH_TIMEOUT="${2:?missing seconds}"; shift 2 ;;
        --allowed-loss) ALLOWED_LOSS="${2:?missing percentage}"; shift 2 ;;
        --tag) TAG="${2:?missing tag}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) usage; exit 2 ;;
    esac
done

[[ -n "$UE_CONF" && -f "$UE_CONF" ]] || { echo "A private, untracked UE config is required via --ue-conf." >&2; exit 2; }
[[ -n "$SIM_FILE" && -f "$SIM_FILE" ]] || { echo "A private, untracked qcore SIM file is required via --sim-file." >&2; exit 2; }
[[ -f "$GNB_CONF" ]] || { echo "Missing gNB config: $GNB_CONF" >&2; exit 2; }
[[ "$ATTACH_TIMEOUT" =~ ^[0-9]+$ ]] && ((ATTACH_TIMEOUT > 0 && ATTACH_TIMEOUT <= 600)) || {
    echo "Attach timeout must be an integer from 1 to 600 seconds." >&2; exit 2;
}
[[ "$PING_COUNT" =~ ^[0-9]+$ ]] && ((PING_COUNT > 0 && PING_COUNT <= 100)) || {
    echo "Ping count must be an integer from 1 to 100." >&2; exit 2;
}
[[ "$ALLOWED_LOSS" =~ ^[0-9]+$ ]] && ((ALLOWED_LOSS >= 0 && ALLOWED_LOSS <= 100)) || {
    echo "Allowed loss must be an integer from 0 to 100 percent." >&2; exit 2;
}
[[ "$TAG" =~ ^[A-Za-z0-9._-]+$ ]] || { echo "Tag contains unsupported characters." >&2; exit 2; }
for binary in "$QCORE_BIN" "$OCUDU_GNB_BIN" "$SRSUE_BIN"; do
    [[ -x "$binary" ]] || { echo "Missing required executable: $binary" >&2; exit 1; }
done
[[ -x "$QCORE_REPO/setup-routing" ]] || { echo "Missing qcore routing setup: $QCORE_REPO/setup-routing" >&2; exit 1; }

RUN_DIR="$VALIDATION_RUN_ROOT/$(date +%Y%m%d_%H%M%S)_$TAG"
mkdir -p "$RUN_DIR"
touch "$RUN_DIR"/{setup.log,qcore.log,gnb.log,srsue.log,checks.txt,dataplane.txt,pids.txt}
cat > "$RUN_DIR/notes.md" <<EOF
# Objective
Reproduce the OCUDU ZMQ attach and namespace dataplane baseline.

# Hypothesis
The known ZMQ cell and matching private subscriber provisioning complete attach and ping.

# Result
See RESULT.md and summary.json.
EOF
{
    printf 'qcore command: %q --mcc 001 --mnc 01 --sim-cred-file <redacted> --local-ip 127.0.0.1 --no-dhcp\n' "$QCORE_BIN"
    printf 'gNB command: %q -c %q\n' "$OCUDU_GNB_BIN" "$GNB_CONF"
    printf 'srsUE command: %q <private-config>\n' "$SRSUE_BIN"
    printf 'gNB config sha256: %s\n' "$(sha256sum "$GNB_CONF" | awk '{print $1}')"
    printf 'UE config sha256: %s\n' "$(sha256sum "$UE_CONF" | awk '{print $1}')"
    printf 'qcore SIM file sha256: %s\n' "$(sha256sum "$SIM_FILE" | awk '{print $1}')"
} > "$RUN_DIR/commands-and-checksums.txt"

cleanup() {
    set +e
    for pid in ${SRSUE_PID:-} ${GNB_PID:-} ${QCORE_PID:-}; do
        sudo -n kill -TERM -- "-$pid" 2>/dev/null || kill -TERM -- "-$pid" 2>/dev/null || true
    done
    "$SCRIPT_DIR/cleanup_zmq_ocudu.sh" --external-iface "$EXTERNAL_IFACE" >> "$RUN_DIR/setup.log" 2>&1 || true
    [[ -f "$RUN_DIR/RESULT.md" ]] || "$SCRIPT_DIR/summarize_zmq_run.sh" "$RUN_DIR" >> "$RUN_DIR/setup.log" 2>&1 || true
}
trap cleanup EXIT INT TERM

sudo -v
"$SCRIPT_DIR/cleanup_zmq_ocudu.sh" --external-iface "$EXTERNAL_IFACE" >> "$RUN_DIR/setup.log" 2>&1
(cd "$QCORE_REPO" && sudo -n ./setup-routing "$EXTERNAL_IFACE") >> "$RUN_DIR/setup.log" 2>&1
sudo -n ip netns add ue1 >> "$RUN_DIR/setup.log" 2>&1

setsid sudo -n env RUST_LOG=info "$QCORE_BIN" \
    --mcc 001 --mnc 01 --sim-cred-file "$SIM_FILE" --local-ip 127.0.0.1 \
    --no-dhcp < /dev/null > "$RUN_DIR/qcore.log" 2>&1 &
QCORE_PID=$!
echo "qcore $QCORE_PID" >> "$RUN_DIR/pids.txt"
sleep 3

setsid "$OCUDU_GNB_BIN" -c "$GNB_CONF" < /dev/null > "$RUN_DIR/gnb.log" 2>&1 &
GNB_PID=$!
echo "gnb $GNB_PID" >> "$RUN_DIR/pids.txt"
sleep 2

setsid sudo -n "$SRSUE_BIN" "$UE_CONF" < /dev/null > "$RUN_DIR/srsue.log" 2>&1 &
SRSUE_PID=$!
echo "srsue $SRSUE_PID" >> "$RUN_DIR/pids.txt"

deadline=$((SECONDS + ATTACH_TIMEOUT))
while ((SECONDS < deadline)); do
    grep -q 'PDU Session Establishment successful' "$RUN_DIR/srsue.log" && break
    kill -0 "$SRSUE_PID" 2>/dev/null || break
    sleep 2
done

{
    sudo -n ip netns exec ue1 ip address show
    sudo -n ip netns exec ue1 ip -o -4 address show dev tun_ue1
    sudo -n ip netns exec ue1 ip route show
} > "$RUN_DIR/checks.txt" 2>&1 || true
sudo -n ip netns exec ue1 ip route replace default dev tun_ue1 >> "$RUN_DIR/setup.log" 2>&1 || true
sudo -n ip netns exec ue1 ping -c "$PING_COUNT" -W 2 "$PING_TARGET" > "$RUN_DIR/dataplane.txt" 2>&1 || true

loss="$(sed -nE 's/.* ([0-9]+)% packet loss.*/\1/p' "$RUN_DIR/dataplane.txt" | tail -1)"
if [[ -n "$loss" && "$loss" -le "$ALLOWED_LOSS" ]]; then
    echo "VALIDATION_PING_OK=1 (loss $loss% <= allowed $ALLOWED_LOSS%)" >> "$RUN_DIR/dataplane.txt"
fi

summary_rc=0
"$SCRIPT_DIR/summarize_zmq_run.sh" "$RUN_DIR" || summary_rc=$?
echo "Run directory: $RUN_DIR"
exit "$summary_rc"
