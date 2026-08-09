#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:?start, status, or stop}"
STAMP="${2:?fresh stamp}"
SETUP_DL_RETX_ACKS="${PAVONIS_NR_SETUP_FOLLOWUP_DL_HARQ_RETX_ACKS:-1}"
SOAPY_MODULE_SHA="${PAVONIS_SOAPY_MODULE_SHA_EXPECTED:-b784c400abdb0ae5c3fa9a0c2f09525d9a09bca4f462b138f469c7118f44520e}"
SOAPY_SOURCE_SHA="${PAVONIS_SOAPY_SOURCE_SHA_EXPECTED:-b5131aabe12da43a3e220956b0c8c85ebb32c35bf5f39cbfc364025491eefd89}"
[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ ]]
[[ "$SETUP_DL_RETX_ACKS" =~ ^[0-9]+$ ]]
[[ "$SOAPY_MODULE_SHA" =~ ^[0-9a-f]{64}$ && "$SOAPY_SOURCE_SHA" =~ ^[0-9a-f]{64}$ ]]
((SETUP_DL_RETX_ACKS <= 16))

CONFIG_BASE=@REMOTE_HOME@/pavonis_cp2924_srsenb_m2sdr
BIN_BASE=@REMOTE_HOME@/pavonis_cp2985_srsenb_pdu_session_dl_harq_retx
ROOT="@REMOTE_HOME@/pavonis_cp2985_srsenb_pdu_session_dl_retx2_runs/$STAMP"
PID_FILE="$ROOT/srsenb.pid"
LOG="$ROOT/srsenb.stdout.log"
BIN="$BIN_BASE/srsenb"
CONFIG="$CONFIG_BASE/config"
RF_PLUGIN_DIR=@REMOTE_HOME@/pavonis_cp2939_rf_plugin
RF_PLUGIN="$RF_PLUGIN_DIR/libsrsran_rf_soapy.so"
RF_BASE_DIR=@REMOTE_HOME@/pavonis_cp2962_srsran_native_build/lib/src/phy/rf
SOAPY_MODULE=/usr/lib/x86_64-linux-gnu/SoapySDR/modules0.8/libSoapyLiteXM2SDR.so
SOAPY_SOURCE=@REMOTE_HOME@/CLionProjects/m2sdr/litex_m2sdr/software/soapysdr/LiteXM2SDRStreaming.cpp
PROGRAM_MODE=/sys/module/m2sdr/parameters/dma_reader_program_mode
LOCAL_ADDR=128.178.122.52/32

BIN_SHA=fb830086dbb69836e44eefcbb3ce49faa885f1e7d80f31fc62c32248e7b2b089
ENB_SHA=4d0e67a967403d9b801cdd080f86cc36f2a0de507cd352299f1e01770f582094
RR_SHA=975feb78150cb512d354cab0f5b721dd59c56c174dfc01d928c1a958a48cc5de
SIB_SHA=b781724b1c199e610e7f4d12b7963120c02d85da4389bccb9c5f8124ee0b57e9
RB_SHA=5964b3011286730e294f75b913dd2b827773c50500c606b92d9c5664d559f976
RF_PLUGIN_SHA=142a2efc2c8309d510f6cad06b5f0280b8ed08ddaedd2a70c72affe952b3ca56
KMOD_SHA=3ded84bbc2ac7ff26a8192666c4083041ae54ebdfd808b48633ccedc91261ead

restore_program_mode() {
  if [[ -e "$PROGRAM_MODE" ]]; then
    echo N | sudo -n tee "$PROGRAM_MODE" >/dev/null
  fi
}

remove_local_addr() {
  if ip -4 addr show dev @RAN_INTERFACE@ | grep -qF "$LOCAL_ADDR"; then
    sudo -n ip addr del "$LOCAL_ADDR" dev @RAN_INTERFACE@
  fi
}

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
  remove_local_addr
  restore_program_mode
}

compact_status() {
  awk '
    /RACH:/ {rach++}
    /User 0x[0-9a-fA-F]+ connected/ {connected++}
    /PAVONIS_NR_DISABLE_SA_CSI_SETUP active/ {csi_marker++}
    /PAVONIS_NR_GTPU_PDU_SESSION_CONTAINER/ {gtpu_ext++}
    /PAVONIS_NR_SETUP_FOLLOWUP_UL_GRANT armed/ {followup_armed++}
    /PAVONIS_NR_SETUP_FOLLOWUP_UL_GRANT injected/ {followup_injected++}
    /PAVONIS_NR_SETUP_FOLLOWUP_DL_HARQ_RETX armed/ {setup_retx_armed++}
    /PAVONIS_NR_PDU_SESSION_DL_HARQ_RETX armed/ {pdu_retx_armed++}
    /PAVONIS_NR_SETUP_FOLLOWUP_DL_HARQ_RETX forced_nack/ {dl_retx_forced++}
    END {
      printf "RACH_COUNT=%d\nCONNECTED_COUNT=%d\nCSI_MARKER_COUNT=%d\nGTPU_EXT_MARKER_COUNT=%d\nFOLLOWUP_GRANT_ARMED_COUNT=%d\nFOLLOWUP_GRANT_INJECTED_LOG_COUNT=%d\nSETUP_DL_HARQ_RETX_ARMED_COUNT=%d\nPDU_SESSION_DL_HARQ_RETX_ARMED_COUNT=%d\nDL_HARQ_RETX_FORCED_COUNT=%d\n", rach, connected, csi_marker, gtpu_ext, followup_armed, followup_injected, setup_retx_armed, pdu_retx_armed, dl_retx_forced
    }
  ' "$LOG"
  printf 'SOAPY_TIMEOUT_COUNT=%s\n' "$(grep -aEc 'SoapySDRDevice_(write|read)Stream returned.*TIMEOUT|Couldn.t write all samples|SoapySDR TX: writeStream timeout' "$LOG" || true)"
  printf 'SOAPY_TIME_ERROR_COUNT=%s\n' "$(grep -aEc 'TIME_ERROR|time error' "$LOG" || true)"
  printf 'UNDERFLOW_COUNT=%s\n' "$(grep -aFc 'UNDERFLOW' "$LOG" || true)"
  printf 'LATE_COUNT=%s\n' "$(grep -aEc 'late|Late' "$LOG" || true)"
}

case "$ACTION" in
  start)
    if [[ "${PAVONIS_CP2985_RF_APPROVED:-0}" != 1 ]]; then
      echo "RF gate closed: set PAVONIS_CP2985_RF_APPROVED=1" >&2
      exit 2
    fi
    [[ ! -e "$ROOT" ]]
    mkdir -p "$ROOT"
    chmod 700 "$ROOT"
    sudo -n true
    [[ -e /dev/m2sdr0 ]]
    [[ -z $(pgrep -x gnb || true) ]]
    [[ -z $(pgrep -x srsenb || true) ]]
    pgrep -x qcore >/dev/null
    sudo -n ss -A sctp -ln | grep -q '@CORE_N2_IP@:38412'
    [[ $(sha256sum "$BIN" | awk '{print $1}') == "$BIN_SHA" ]]
    [[ $(sha256sum "$CONFIG/enb.conf" | awk '{print $1}') == "$ENB_SHA" ]]
    [[ $(sha256sum "$CONFIG/rr.conf" | awk '{print $1}') == "$RR_SHA" ]]
    [[ $(sha256sum "$CONFIG/sib.conf" | awk '{print $1}') == "$SIB_SHA" ]]
    [[ $(sha256sum "$CONFIG/rb.conf" | awk '{print $1}') == "$RB_SHA" ]]
    [[ $(sha256sum "$RF_PLUGIN" | awk '{print $1}') == "$RF_PLUGIN_SHA" ]]
    [[ $(sha256sum "$SOAPY_MODULE" | awk '{print $1}') == "$SOAPY_MODULE_SHA" ]]
    [[ $(sha256sum "$SOAPY_SOURCE" | awk '{print $1}') == "$SOAPY_SOURCE_SHA" ]]
    [[ $(sha256sum "$(modinfo -n m2sdr)" | awk '{print $1}') == "$KMOD_SHA" ]]
    [[ $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort -u) == performance ]]
    [[ $(cat "$PROGRAM_MODE") == N ]]
    ! ip -4 addr show | grep -qF "$LOCAL_ADDR"
    sudo -n ip addr add "$LOCAL_ADDR" dev @RAN_INTERFACE@
    ip -4 addr show dev @RAN_INTERFACE@ | grep -qF "$LOCAL_ADDR"
    echo Y | sudo -n tee "$PROGRAM_MODE" >/dev/null
    [[ $(cat "$PROGRAM_MODE") == Y ]]

    start_failed=1
    on_start_exit() {
      local rc=$?
      trap - EXIT INT TERM HUP
      if [[ "$start_failed" == 1 ]]; then
        set +e
        stop_gnb
      fi
      exit "$rc"
    }
    trap on_start_exit EXIT INT TERM HUP

    : >"$LOG"
    cd "$CONFIG"
    nohup env \
      LD_LIBRARY_PATH="$RF_PLUGIN_DIR:$RF_BASE_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
      M2SDR_SOAPY_TX_ALIGN_HAS_TIME_REMAINDER=1 \
      PAVONIS_SRSRAN_SOAPY_PHYSICAL_CHANNEL=1 \
      PAVONIS_RRC_REMOVE_PRE_NGAP_ON_RELEASE_MISS=1 \
      PAVONIS_NR_PUCCH_ACK_TRACE=1 \
      PAVONIS_NR_SETUP_UL_TRACE=1 \
      PAVONIS_NR_SETUP_ACK_UL_GRANT=1 \
      PAVONIS_NR_SETUP_FOLLOWUP_UL_GRANTS=512 \
      PAVONIS_NR_SETUP_FOLLOWUP_DL_HARQ_RETX_ACKS="$SETUP_DL_RETX_ACKS" \
      PAVONIS_NR_PDU_SESSION_DL_HARQ_RETX_ACKS=2 \
      PAVONIS_NR_SR_RESOURCE0_COMPAT=1 \
      PAVONIS_NR_DISABLE_SA_CSI_SETUP=1 \
      PAVONIS_NR_GTPU_PDU_SESSION_CONTAINER=1 \
      stdbuf -oL -eL "$BIN" enb.conf \
      --rf.tx_gain=16.0 \
      --log.all_level=warning --log.filename=stdout \
      --pcap.enable=true --pcap.nr_filename="$ROOT/srsenb_nr_mac.pcap" \
      --pcap.ngap_enable=true --pcap.ngap_filename="$ROOT/srsenb_ngap.pcap" \
      >"$LOG" 2>&1 </dev/null &
    pid=$!
    printf '%s\n' "$pid" >"$PID_FILE"
    ready=0
    for _ in $(seq 1 400); do
      if grep -q 'NG connection successful' "$LOG" &&
         grep -q '==== eNodeB started ===' "$LOG" &&
         grep -q 'Selecting Soapy device: 0' "$LOG" &&
         grep -q 'PAVONIS_SRSRAN_SOAPY_PHYSICAL_CHANNEL active=1 requested_channels=1' "$LOG" &&
         grep -q 'RX setupStream: Selected channel 1' "$LOG"; then
        ready=1
        break
      fi
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    if [[ "$ready" != 1 ]]; then
      tail -n 60 "$LOG"
      exit 1
    fi
    tr '\0' '\n' <"/proc/$pid/environ" | grep -qx 'M2SDR_SOAPY_TX_ALIGN_HAS_TIME_REMAINDER=1'
    tr '\0' '\n' <"/proc/$pid/environ" | grep -qx 'PAVONIS_SRSRAN_SOAPY_PHYSICAL_CHANNEL=1'
    tr '\0' '\n' <"/proc/$pid/environ" | grep -qx 'PAVONIS_NR_SETUP_FOLLOWUP_UL_GRANTS=512'
    tr '\0' '\n' <"/proc/$pid/environ" | grep -qx "PAVONIS_NR_SETUP_FOLLOWUP_DL_HARQ_RETX_ACKS=$SETUP_DL_RETX_ACKS"
    tr '\0' '\n' <"/proc/$pid/environ" | grep -qx 'PAVONIS_NR_PDU_SESSION_DL_HARQ_RETX_ACKS=2'
    tr '\0' '\n' <"/proc/$pid/environ" | grep -qx 'PAVONIS_NR_DISABLE_SA_CSI_SETUP=1'
    tr '\0' '\n' <"/proc/$pid/environ" | grep -qx 'PAVONIS_NR_GTPU_PDU_SESSION_CONTAINER=1'
    tr '\0' '\n' <"/proc/$pid/cmdline" | grep -qx -- '--rf.tx_gain=16.0'
    ! grep -q 'TX gain was not set' "$LOG"
    start_failed=0
    trap - EXIT INT TERM HUP
    echo "CP2985_SRSENB_M2SDR_CH1_ATT16_RX40_PDU_SESSION_DL_RETX2_START=PASS pid=$pid root=$ROOT setup_dl_retx_acks=$SETUP_DL_RETX_ACKS soapy_module_sha=$SOAPY_MODULE_SHA soapy_source_sha=$SOAPY_SOURCE_SHA"
    grep -E 'Selecting Soapy device|RX setupStream|TX setupStream|Setting frequency|NG connection successful|eNodeB started|PAVONIS_SRSRAN_SOAPY_PHYSICAL_CHANNEL|PAVONIS_NR_(DISABLE_SA_CSI_SETUP|GTPU_PDU_SESSION_CONTAINER)' "$LOG" | head -n 30
    ;;
  status)
    pid="$(cat "$PID_FILE")"
    kill -0 "$pid"
    echo "CP2985_SRSENB_M2SDR_CH1_ATT16_RX40_PDU_SESSION_DL_RETX2_STATUS=RUNNING pid=$pid"
    compact_status
    ;;
  stop)
    stop_gnb
    [[ -z $(pgrep -x srsenb || true) ]]
    [[ $(cat "$PROGRAM_MODE") == N ]]
    ! ip -4 addr show dev @RAN_INTERFACE@ | grep -qF "$LOCAL_ADDR"
    echo CP2985_SRSENB_M2SDR_CH1_ATT16_RX40_PDU_SESSION_DL_RETX2_STOP=PASS
    ;;
  *)
    echo "usage: $0 start|status|stop STAMP" >&2
    exit 64
    ;;
esac
