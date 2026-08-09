#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:?start, status, or stop}"
STAMP="${2:?fresh stamp}"
[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ ]]

ROOT="@REMOTE_HOME@/pavonis_cp2629_runs/$STAMP"
PID_FILE="$ROOT/srsenb.pid"
LOG="$ROOT/srsenb.stdout.log"
BIN=@REMOTE_HOME@/pavonis_cp2656_srsenb/srsenb
PLUGIN_DIR=@REMOTE_HOME@/pavonis_cp2639_bladerf_gnb
PLUGIN="$PLUGIN_DIR/libsrsran_rf_blade.so.25.10.0"
CONFIG=@REMOTE_HOME@/pavonis_cp2642_bladerf_gnb/config

stop_gnb() {
  local pid=''
  [[ -s "$PID_FILE" ]] && pid="$(cat "$PID_FILE")"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 100); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
  fi
  rm -f "$PID_FILE"
}

case "$ACTION" in
  start)
    mkdir -p "$ROOT"
    chmod 700 "$ROOT"
    [[ ! -e "$PID_FILE" ]]
    [[ -z $(pgrep -x srsenb || true) ]]
    [[ -z $(pgrep -x srsue || true) ]]
    [[ $(sha256sum "$BIN" | cut -d ' ' -f1) == 288cfcbd3af8171e4054b88b0ba12352e9daa7e9065fd61c349c2882e2982015 ]]
    [[ $(sha256sum "$PLUGIN" | cut -d ' ' -f1) == 89b8ed5b609865c67298606f7b90daa97fa98c61ee7290feb602b62308289b4a ]]
    [[ $(sha256sum @REMOTE_HOME@/CLionProjects/srsRAN_4G/lib/src/phy/rf/rf_blade_imp.c | cut -d ' ' -f1) == 18dffea9d147a964312c0c4051303d8e07651a53b4edcc52a9b01e72f69ceed9 ]]
    [[ $(sha256sum "$CONFIG/enb.conf" | cut -d ' ' -f1) == 3f4039d0ece26fda026bb061f5bb4414e036b0c0af4dcb7de309ce2f8c295d72 ]]
    [[ $(sha256sum "$CONFIG/rr.conf" | cut -d ' ' -f1) == 975feb78150cb512d354cab0f5b721dd59c56c174dfc01d928c1a958a48cc5de ]]
    [[ $(sha256sum "$CONFIG/sib.conf" | cut -d ' ' -f1) == b781724b1c199e610e7f4d12b7963120c02d85da4389bccb9c5f8124ee0b57e9 ]]
    [[ $(sha256sum "$CONFIG/rb.conf" | cut -d ' ' -f1) == 5964b3011286730e294f75b913dd2b827773c50500c606b92d9c5664d559f976 ]]
    bladeRF-cli -p 2>&1 | grep -q '5c750e8ef0e44a068c7c99fedb9b35fd'
    ping -c 1 -W 2 @CORE_N2_IP@ >/dev/null
    : >"$LOG"
    cd "$CONFIG"
    nohup env \
      LD_LIBRARY_PATH="$PLUGIN_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
      PAVONIS_RF_BLADE_PHYSICAL_CHANNEL=1 \
      PAVONIS_RF_BLADE_SINGLE_CHANNEL_LAYOUT=1 \
      PAVONIS_RF_BLADE_STATE_DBG=1 \
      PAVONIS_RRC_REMOVE_PRE_NGAP_ON_RELEASE_MISS=1 \
      PAVONIS_NR_PUCCH_ACK_TRACE=1 \
      PAVONIS_NR_SETUP_UL_TRACE=1 \
      PAVONIS_NR_SETUP_ACK_UL_GRANT=1 \
      PAVONIS_NR_SR_RESOURCE0_COMPAT=1 \
      PAVONIS_NR_DISABLE_SA_CSI_SETUP=1 \
      stdbuf -oL -eL "$BIN" enb.conf \
      --log.all_level=warning --log.filename=stdout \
      --pcap.enable=true --pcap.nr_filename="$ROOT/srsenb_nr_mac.pcap" \
      --pcap.ngap_enable=true --pcap.ngap_filename="$ROOT/srsenb_ngap.pcap" \
      >"$LOG" 2>&1 </dev/null &
    pid=$!
    printf '%s\n' "$pid" >"$PID_FILE"
    ready=0
    for _ in $(seq 1 300); do
      if grep -q 'PAVONIS_RF_BLADE_PHYSICAL_CHANNEL active index=1 rx=2 tx=3' "$LOG" &&
         grep -q 'PAVONIS_RF_BLADE_SINGLE_CHANNEL_LAYOUT active=1 rx_layout=0 tx_layout=1' "$LOG" &&
         grep -q 'NG connection successful' "$LOG" &&
         grep -q '==== eNodeB started ===' "$LOG"; then
        ready=1
        break
      fi
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    if [[ "$ready" != 1 ]]; then
      stop_gnb
      tail -n 100 "$LOG"
      exit 1
    fi
    tr '\0' '\n' <"/proc/$pid/environ" | grep -qx 'PAVONIS_RRC_REMOVE_PRE_NGAP_ON_RELEASE_MISS=1'
    tr '\0' '\n' <"/proc/$pid/environ" | grep -qx 'PAVONIS_NR_PUCCH_ACK_TRACE=1'
    tr '\0' '\n' <"/proc/$pid/environ" | grep -qx 'PAVONIS_NR_SETUP_UL_TRACE=1'
    tr '\0' '\n' <"/proc/$pid/environ" | grep -qx 'PAVONIS_NR_SETUP_ACK_UL_GRANT=1'
    tr '\0' '\n' <"/proc/$pid/environ" | grep -qx 'PAVONIS_NR_SR_RESOURCE0_COMPAT=1'
    tr '\0' '\n' <"/proc/$pid/environ" | grep -qx 'PAVONIS_NR_DISABLE_SA_CSI_SETUP=1'
    grep -q 'PAVONIS_NR_SR_RESOURCE0_COMPAT active cc=0 resource_id=0' "$LOG"
    grep -q 'PAVONIS_NR_DISABLE_SA_CSI_SETUP active cc=0' "$LOG"
    echo "CP2629_SRSENB_START=PASS pid=$pid root=$ROOT"
    echo CP2656_TRACE_ENV=PASS
    grep -E 'PAVONIS_RF_BLADE_(PHYSICAL_CHANNEL|SINGLE_CHANNEL_LAYOUT) active|PAVONIS_NR_(SR_RESOURCE0_COMPAT|DISABLE_SA_CSI_SETUP) active|NG connection successful|eNodeB started|Setting frequency' "$LOG" | head -n 24
    ;;
  status)
    pid="$(cat "$PID_FILE")"
    kill -0 "$pid"
    echo "CP2629_SRSENB_STATUS=RUNNING pid=$pid"
    awk '
      /RACH:/ {rach++}
      /User 0x[0-9a-fA-F]+ connected/ {connected++}
      /Radio-Link Failure|disconnected/ {released++}
      END {printf "RACH_COUNT=%d\nCONNECTED_COUNT=%d\nRELEASE_COUNT=%d\n", rach, connected, released}
    ' "$LOG"
    grep -E 'RACH:|User 0x[0-9a-fA-F]+ connected|Radio-Link Failure|disconnected|ERROR|Failed' "$LOG" | tail -n 80 || true
    ;;
  stop)
    stop_gnb
    [[ -z $(pgrep -x srsenb || true) ]]
    echo CP2629_SRSENB_STOP=PASS
    ;;
  *)
    echo "usage: $0 start|status|stop STAMP" >&2
    exit 64
    ;;
esac
