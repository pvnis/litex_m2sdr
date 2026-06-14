#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_ocudu_validation.sh
source "$SCRIPT_DIR/env_ocudu_validation.sh"

EXTERNAL_IFACE="${OCUDU_VALIDATION_EXTERNAL_IFACE:-wlp5s0}"
VERBOSE=0
while (($#)); do
    case "$1" in
        --external-iface) EXTERNAL_IFACE="${2:?missing interface}"; shift 2 ;;
        --verbose) VERBOSE=1; shift ;;
        *) echo "usage: $0 [--external-iface IFACE] [--verbose]" >&2; exit 2 ;;
    esac
done

snapshot() {
    echo "Processes:"
    pgrep -a -x qcore || true
    pgrep -a -x gnb || true
    pgrep -a -x srsue || true
    echo "Validation links/namespaces:"
    ip netns list 2>/dev/null | grep -E '^ue1([[:space:]]|$)' || true
    ip -brief link 2>/dev/null | grep -E '^(qcoretun2?|veth[0-3]|qcore_br0|veth_ue_[12]_[ab])[[:space:]]' || true
}

stop_matching_processes() {
    local name pid cmd
    for name in srsue gnb qcore; do
        while read -r pid; do
            [[ -n "$pid" ]] || continue
            cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
            case "$name:$cmd" in
                srsue:*"$SRSUE_BIN"*|srsue:*"$VALIDATION_ROOT"*) sudo -n kill -TERM "$pid" 2>/dev/null || true ;;
                gnb:*"$OCUDU_GNB_BIN"*|gnb:*"$VALIDATION_ROOT"*) sudo -n kill -TERM "$pid" 2>/dev/null || true ;;
                qcore:*"$QCORE_BIN"*|qcore:*"$VALIDATION_ROOT"*) sudo -n kill -TERM "$pid" 2>/dev/null || true ;;
            esac
        done < <(pgrep -x "$name" 2>/dev/null || true)
    done
    sleep 1
}

((VERBOSE)) && { echo "Before cleanup:"; snapshot; }
stop_matching_processes

sudo -n ip netns del ue1 2>/dev/null || true
for link in qcoretun qcoretun2 veth0 veth1 veth2 veth3 qcore_br0 \
    veth_ue_1_a veth_ue_1_b veth_ue_2_a veth_ue_2_b; do
    sudo -n ip link del "$link" 2>/dev/null || true
done

while sudo -n iptables -C FORWARD -i qcoretun -j ACCEPT 2>/dev/null; do
    sudo -n iptables -D FORWARD -i qcoretun -j ACCEPT
done
while sudo -n iptables -C FORWARD -i qcoretun2 -j ACCEPT 2>/dev/null; do
    sudo -n iptables -D FORWARD -i qcoretun2 -j ACCEPT
done
while sudo -n iptables -C FORWARD -i "$EXTERNAL_IFACE" -j ACCEPT -d 10.255.0.0/24 2>/dev/null; do
    sudo -n iptables -D FORWARD -i "$EXTERNAL_IFACE" -j ACCEPT -d 10.255.0.0/24
done
while sudo -n iptables -t nat -C POSTROUTING -o "$EXTERNAL_IFACE" -s 10.255.0.0/24 -j MASQUERADE 2>/dev/null; do
    sudo -n iptables -t nat -D POSTROUTING -o "$EXTERNAL_IFACE" -s 10.255.0.0/24 -j MASQUERADE
done

((VERBOSE)) && { echo "After cleanup:"; snapshot; }
