#!/usr/bin/env bash
set -euo pipefail

NETNS="${PAVONIS_UE_NETNS:-ue1}"
ACTION="${1:-status}"

case "$ACTION" in
  --self-test)
    for cmd in grep ip sudo; do
      command -v "$cmd" >/dev/null || {
        echo "SELF_TEST=FAIL missing=$cmd"
        exit 1
      }
    done
    echo "SELF_TEST=PASS"
    ;;
  prepare)
    if ip netns list 2>/dev/null | grep -Eq "^${NETNS}([[:space:]]|$)"; then
      sudo -n ip netns delete "$NETNS"
    fi
    sudo -n ip netns add "$NETNS"
    sudo -n ip netns exec "$NETNS" ip link set lo up
    test -e "/run/netns/$NETNS"
    ip netns list | grep -E "^${NETNS}([[:space:]]|$)"
    sudo -n ip netns exec "$NETNS" ip -o link show lo
    echo "NETNS_PREPARE=PASS name=$NETNS"
    ;;
  cleanup)
    sudo -n ip netns delete "$NETNS" 2>/dev/null || true
    if [[ -e "/run/netns/$NETNS" ]]; then
      echo "NETNS_CLEANUP=FAIL name=$NETNS"
      exit 1
    fi
    echo "NETNS_CLEANUP=PASS name=$NETNS"
    ;;
  status)
    if [[ -e "/run/netns/$NETNS" ]]; then
      ip netns list | grep -E "^${NETNS}([[:space:]]|$)"
      sudo -n ip netns exec "$NETNS" ip -o link show lo
      echo "NETNS_STATUS=present name=$NETNS"
    else
      echo "NETNS_STATUS=absent name=$NETNS"
      exit 1
    fi
    ;;
  *)
    echo "usage: $0 {prepare|cleanup|status|--self-test}" >&2
    exit 2
    ;;
esac
