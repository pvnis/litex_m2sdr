#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:?start, status, stop, or probe-phase-mode}"
STAMP="${2:?fresh stamp}"
TARGET_PCI="${3:-500}"
SOAPY_SOURCE_SHA="${PAVONIS_SOAPY_SOURCE_SHA_EXPECTED:-b5131aabe12da43a3e220956b0c8c85ebb32c35bf5f39cbfc364025491eefd89}"
SOAPY_MODULE_SHA="${PAVONIS_SOAPY_MODULE_SHA_EXPECTED:-b784c400abdb0ae5c3fa9a0c2f09525d9a09bca4f462b138f469c7118f44520e}"
PRACH_COLLECTION_PHASE_MODE="${PAVONIS_PRACH_COLLECTION_PHASE_MODE:-default}"
PRACH_EARLY_CAPTURE_MAX="${PAVONIS_PRACH_EARLY_PREDEMOD_CAPTURE_MAX:-0}"
PRACH_TIMING_COMPENSATION_SAMPLES="${PAVONIS_PRACH_TIMING_COMPENSATION_SAMPLES:-0}"
PRACH_REPORT_SLOT_OFFSET="${PAVONIS_PRACH_TIMING_COMPENSATION_REPORT_SLOT_OFFSET:-0}"
RA_RESP_WINDOW="${PAVONIS_GNB_RA_RESP_WINDOW:-10}"
MSG3_TARGET_CAPTURE="${PAVONIS_UL_SYMBOL_MSG3_TARGET_CAPTURE:-0}"
MSG3_TARGET_MAX="${PAVONIS_UL_SYMBOL_MSG3_TARGET_MAX:-8}"
UL_SYMBOL_RAW_MAX="${PAVONIS_UL_SYMBOL_RAW_MAX:-0}"
GNB_BIN="${PAVONIS_OCUDU_GNB_BIN_OVERRIDE:-@REMOTE_HOME@/pavonis_cp3025_stage1_energy_capture_gnb/gnb}"
GNB_SHA="${PAVONIS_OCUDU_GNB_SHA_EXPECTED:-dbc7cb7905e51b58a9621e9e6fa93fab81e1286938eb7bd543517c91cc7060b9}"
GNB_LIBRARY_PATH="${PAVONIS_OCUDU_GNB_LIBRARY_PATH_OVERRIDE:-}"
PHYSICAL_CHANNEL_OFFSET="${PAVONIS_SOAPY_PHYSICAL_CHANNEL_OFFSET:-}"
PHYSICAL_CHANNEL_LABEL="${PHYSICAL_CHANNEL_OFFSET:-0}"
[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ ]]
[[ "$TARGET_PCI" =~ ^[0-9]+$ ]] && ((TARGET_PCI <= 1007))
[[ "$SOAPY_SOURCE_SHA" =~ ^[0-9a-f]{64}$ && "$SOAPY_MODULE_SHA" =~ ^[0-9a-f]{64}$ ]]
[[ "$PRACH_COLLECTION_PHASE_MODE" == default || "$PRACH_COLLECTION_PHASE_MODE" == off ]]
[[ "$PRACH_EARLY_CAPTURE_MAX" =~ ^[0-9]+$ ]] && ((PRACH_EARLY_CAPTURE_MAX <= 128))
[[ "$PRACH_TIMING_COMPENSATION_SAMPLES" =~ ^[0-9]+$ ]] && ((PRACH_TIMING_COMPENSATION_SAMPLES <= 1000000))
[[ "$PRACH_REPORT_SLOT_OFFSET" =~ ^[0-9]+$ ]] && ((PRACH_REPORT_SLOT_OFFSET <= 100))
[[ "$RA_RESP_WINDOW" =~ ^[0-9]+$ ]] && ((RA_RESP_WINDOW >= 1 && RA_RESP_WINDOW <= 80))
[[ "$MSG3_TARGET_CAPTURE" =~ ^[01]$ ]]
[[ "$MSG3_TARGET_MAX" =~ ^[0-9]+$ ]] && ((MSG3_TARGET_MAX >= 1 && MSG3_TARGET_MAX <= 8))
[[ "$UL_SYMBOL_RAW_MAX" =~ ^[0-9]+$ ]] && ((UL_SYMBOL_RAW_MAX <= 128))
if [[ "$MSG3_TARGET_CAPTURE" == 1 ]]; then
  ((UL_SYMBOL_RAW_MAX >= 14))
else
  ((UL_SYMBOL_RAW_MAX == 0))
fi
[[ "$GNB_BIN" =~ ^@REMOTE_HOME@/[A-Za-z0-9_./-]+$ && "$GNB_SHA" =~ ^[0-9a-f]{64}$ ]]
[[ -z "$GNB_LIBRARY_PATH" || "$GNB_LIBRARY_PATH" =~ ^@REMOTE_HOME@/[A-Za-z0-9_./-]+$ ]]
[[ -z "$PHYSICAL_CHANNEL_OFFSET" || "$PHYSICAL_CHANNEL_OFFSET" == 1 ]]
ALLOW_EXTENDED_RA_RESP_WINDOW=0
if ((RA_RESP_WINDOW > 10)); then
  ALLOW_EXTENDED_RA_RESP_WINDOW=1
fi
if [[ -n "$GNB_LIBRARY_PATH" ]]; then
  export LD_LIBRARY_PATH="$GNB_LIBRARY_PATH${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
if [[ "$PRACH_COLLECTION_PHASE_MODE" == default ]]; then
  PRACH_COLLECTION_PHASE_SEQUENCE=0:0,1474560:160,1647360:160
else
  PRACH_COLLECTION_PHASE_SEQUENCE=
fi

ROOT="@REMOTE_HOME@/pavonis_cp3059_m2sdr_stage_force_att16_runs/$STAMP"
OUT="$ROOT/launch"
STATE="$ROOT/state.env"
START_LOG="$ROOT/cp2730_start.log"
SNAPSHOT="$ROOT/gnb.snapshot.log"
UL_SYMBOL_DUMP_ARM_FILE="/tmp/pavonis_ul_symbol_${STAMP}_arm"
MSG3_CAPTURE_DIR="$ROOT/msg3_target_capture"
MSG3_CAPTURE_TAR="$ROOT/msg3_target_capture.tar.gz"
VALIDATION=@REMOTE_HOME@/CLionProjects/m2sdr/litex_m2sdr/software/validation/ocudu
STARTER=@REMOTE_HOME@/stage6_cp3119_start_ran.sh
STARTER_SHA=a377e31ddc7fb58a69d5900add9cbd9a9655414fbf951330a5c23c6bd173f46f
STAGE=@REMOTE_HOME@/pavonis_cp3059_m2sdr_force_att16_stage_remote.sh
STAGE_SHA=7752f0dd049e7a7382cd16d1180699a422a77b8ac574c211fd5bda98d4e5e566
SOAPY_SOURCE=@REMOTE_HOME@/CLionProjects/m2sdr/litex_m2sdr/software/soapysdr/LiteXM2SDRStreaming.cpp
SOAPY_BUILD=@REMOTE_HOME@/CLionProjects/m2sdr/litex_m2sdr/software/soapysdr/build/libSoapyLiteXM2SDR.so
SOAPY_DEPLOYED=/usr/lib/x86_64-linux-gnu/SoapySDR/modules0.8/libSoapyLiteXM2SDR.so
KMOD_SHA=3ded84bbc2ac7ff26a8192666c4083041ae54ebdfd808b48633ccedc91261ead
PROGRAM_MODE=/sys/module/m2sdr/parameters/dma_reader_program_mode

load_state() {
  [[ -s "$STATE" ]]
  # shellcheck source=/dev/null
  source "$STATE"
}

snapshot_log() {
  if [[ -n "${GNB_LOG:-}" && -f "$GNB_LOG" ]]; then
    cp "$GNB_LOG" "$SNAPSHOT"
  fi
}

package_msg3_capture() {
  [[ "$MSG3_TARGET_CAPTURE" == 1 ]] || return 0

  local manifest=/tmp/pavonis_ul_symbol_manifest.tsv
  local files=()
  local file manifest_rows=0
  shopt -s nullglob
  files=(/tmp/pavonis_puxch_target_*.cf32)
  shopt -u nullglob

  rm -rf "$MSG3_CAPTURE_DIR"
  mkdir -p "$MSG3_CAPTURE_DIR"
  if [[ -s "$manifest" ]]; then
    cp -p "$manifest" "$MSG3_CAPTURE_DIR/"
    manifest_rows="$(awk 'END { print (NR > 0 ? NR - 1 : 0) }' "$manifest")"
  fi
  for file in "${files[@]}"; do
    cp -p "$file" "$MSG3_CAPTURE_DIR/"
  done
  {
    printf 'MSG3_TARGET_CAPTURE=%s\n' "$MSG3_TARGET_CAPTURE"
    printf 'MSG3_TARGET_MAX=%s\n' "$MSG3_TARGET_MAX"
    printf 'UL_SYMBOL_RAW_MAX=%s\n' "$UL_SYMBOL_RAW_MAX"
    printf 'TARGET_RAW_FILE_COUNT=%s\n' "${#files[@]}"
    printf 'TARGET_MANIFEST_ROWS=%s\n' "$manifest_rows"
  } >"$MSG3_CAPTURE_DIR/capture_summary.env"
  tar -C "$ROOT" -czf "$MSG3_CAPTURE_TAR" msg3_target_capture
  sha256sum "$MSG3_CAPTURE_TAR"
}

restore_program_mode() {
  if [[ -e "$PROGRAM_MODE" ]]; then
    echo N | sudo -n tee "$PROGRAM_MODE" >/dev/null
    [[ $(cat "$PROGRAM_MODE") == N ]]
  fi
}

stop_processes() {
  local pid
  if [[ -s "$STATE" ]]; then
    load_state || true
  elif [[ -s "$OUT/TX_READY.env" ]]; then
    HARNESS_PID="$(awk -F= '$1=="HARNESS_PID"{print $2}' "$OUT/TX_READY.env" | tail -n 1)"
    OTA="$(awk -F= '$1=="OTA"{print $2}' "$OUT/TX_READY.env" | tail -n 1)"
    GNB_LOG="$(awk -F= '$1=="GNB_LOG"{print $2}' "$OUT/TX_READY.env" | tail -n 1)"
    if [[ -s "$OTA/pids.txt" ]]; then
      GNB_PID="$(awk '$1=="gnb"{print $2}' "$OTA/pids.txt" | tail -n 1)"
      STAGE_PID="$(awk '$1=="stage"{print $2}' "$OTA/pids.txt" | tail -n 1)"
    fi
  fi

  pid="${GNB_PID:-}"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  fi
  pid="${STAGE_PID:-}"
  [[ "$pid" =~ ^[0-9]+$ ]] && kill -TERM "$pid" 2>/dev/null || true
  pid="${HARNESS_PID:-}"
  [[ "$pid" =~ ^[0-9]+$ ]] && kill -TERM "$pid" 2>/dev/null || true

  for _ in $(seq 1 80); do
    [[ -z $(pgrep -x gnb || true) ]] && break
    sleep 0.1
  done
  for pid in $(pgrep -x gnb 2>/dev/null || true); do
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  done
  snapshot_log || true
  restore_program_mode
}

compact_status() {
  local log="$1"
  awk '
    /prach\(ra-rnti=/ {prach++}
    /PAVONIS_GNB_RAR_PDU_TRACE/ {rar++}
    /PUSCH: rnti=/ {pusch++}
    /CRC=OK|crc=OK|CRC OK/ {crc_ok++}
    /RRCSetupRequest|RRC Setup Request/ {rrc_setup++}
    END {
      printf "PRACH_EVENT_COUNT=%d\nRAR_TRACE_COUNT=%d\nPUSCH_COUNT=%d\nCRC_OK_COUNT=%d\nRRC_SETUP_COUNT=%d\n", prach, rar, pusch, crc_ok, rrc_setup
    }
  ' "$log"
  printf 'SOAPY_TX_TIMEOUT_COUNT=%s\n' "$(grep -aEc 'SoapySDR TX: writeStream timeout|Soapy TX timeout' "$log" || true)"
  printf 'DOWNLINK_LATE_COUNT=%s\n' "$(grep -aFc 'Downlink data late' "$log" || true)"
  printf 'UNDERFLOW_COUNT=%s\n' "$(grep -aFc 'UNDERFLOW' "$log" || true)"
  printf 'TIME_ERROR_COUNT=%s\n' "$(grep -aFc 'TIME_ERROR' "$log" || true)"
  printf 'BACKFILL_MARKER_COUNT=%s\n' "$(grep -aFc 'backfilled contiguous untimed write' "$log" || true)"
}

case "$ACTION" in
  start)
    [[ ! -e "$ROOT" ]]
    mkdir -p "$ROOT"
    chmod 700 "$ROOT"
    sudo -n true
    [[ -z $(pgrep -x gnb || true) ]]
    pgrep -x qcore >/dev/null
    sudo -n ss -A sctp -ln | grep -q '@CORE_N2_IP@:38412'
    [[ -e /dev/m2sdr0 ]]
    [[ $(sha256sum "$STARTER" | awk '{print $1}') == "$STARTER_SHA" ]]
    [[ $(sha256sum "$STAGE" | awk '{print $1}') == "$STAGE_SHA" ]]
    [[ $(sha256sum "$GNB_BIN" | awk '{print $1}') == "$GNB_SHA" ]]
    [[ $(sha256sum "$SOAPY_DEPLOYED" | awk '{print $1}') == "$SOAPY_MODULE_SHA" ]]
    KMOD="$(modinfo -n m2sdr)"
    [[ $(sha256sum "$KMOD" | awk '{print $1}') == "$KMOD_SHA" ]]
    [[ $(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor | sort -u) == performance ]]
    [[ $(cat "$PROGRAM_MODE") == N ]]
    if [[ "$MSG3_TARGET_CAPTURE" == 1 ]]; then
      rm -f /tmp/pavonis_puxch_target_*.cf32 /tmp/pavonis_ul_symbol_manifest.tsv "$UL_SYMBOL_DUMP_ARM_FILE"
    fi
    PROGRAM_MARKER_BEFORE="$(sudo -n dmesg | grep -F -c 'Reader one-shot program mode active' || true)"
    echo Y | sudo -n tee "$PROGRAM_MODE" >/dev/null
    [[ $(cat "$PROGRAM_MODE") == Y ]]

    start_failed=1
    on_start_exit() {
      local rc=$?
      trap - EXIT INT TERM HUP
      if [[ "$start_failed" == 1 ]]; then
        set +e
        stop_processes
      fi
      exit "$rc"
    }
    trap on_start_exit EXIT INT TERM HUP

    set +e
    (
      cd "$VALIDATION"
      env \
        TAG=cp3059_ocudu_m2sdr_stage_force_att16_target STAMP="$STAMP" OUT="$OUT" \
        DURATION=420 TX_BACKOFF=10 TX_ATT=16 GNB_RX_GAIN=40 IFACE=@RAN_INTERFACE@ \
        PAVONIS_SOAPY_TX_FORCE_ATT_DB=16 \
        PAVONIS_SOAPY_PHYSICAL_CHANNEL_OFFSET="$PHYSICAL_CHANNEL_OFFSET" \
        PAVONIS_OTA_STAGE_TEMPLATE_OVERRIDE="$STAGE" \
        OCUDU_GNB_BIN="$GNB_BIN" \
        PAVONIS_GNB_CLOCK_PPM=-0.548059701 PAVONIS_GNB_PLMN=90170 \
        PAVONIS_GNB_ID=411 PAVONIS_GNB_ID_BIT_LENGTH=28 PAVONIS_GNB_SECTOR_ID=1 \
        PAVONIS_GNB_PCI="$TARGET_PCI" PAVONIS_GNB_TAC=7 \
        PAVONIS_GNB_PRACH_FREQUENCY_START=1 PAVONIS_GNB_PRACH_CONFIG_INDEX=0 \
        PAVONIS_GNB_TOTAL_NOF_RA_PREAMBLES=8 PAVONIS_GNB_PREAMBLE_RX_TARGET_PW=-110 \
        PAVONIS_GNB_MIB_INTRAFREQ_RESELECTION_ALLOWED=1 \
        PAVONIS_GNB_SRSENB_SIB1_COMPAT=1 PAVONIS_GNB_SIB1_RETX_PERIOD_MS=20 \
        PAVONIS_GNB_SIB1_USE_N0=1 PAVONIS_GNB_MSG3_DELTA_PREAMBLE=-1 \
        PAVONIS_GNB_CORESET0_INDEX=6 PAVONIS_GNB_SS0_INDEX=0 \
        PAVONIS_GNB_SS1_N_CANDIDATES=0,0,1,1,0 PAVONIS_GNB_MAX_UE_MCS=9 \
        PAVONIS_GNB_MAX_CONRES_MCS=2 PAVONIS_GNB_FALLBACK_DL_DCI_AGGR_LEV=8 \
        PAVONIS_GNB_SSB_BLOCK_POWER_DBM=0 PAVONIS_GNB_SSB_AMPLITUDE_DB=6 \
        PAVONIS_GNB_POWER_CONTROL_OFFSET_SS_DB=6 PAVONIS_GNB_SI_PDSCH_POWER_SIGN_COMPAT=1 \
        PAVONIS_GNB_RA_RESP_WINDOW="$RA_RESP_WINDOW" \
        PAVONIS_ALLOW_EXTENDED_RA_RESP_WINDOW="$ALLOW_EXTENDED_RA_RESP_WINDOW" \
        PAVONIS_GNB_PUSCH_DEC_MAX_ITERATIONS=10 \
        PAVONIS_GNB_INITIAL_CONTEXT_TRACE=1 \
        PAVONIS_GNB_SI_OCCASION_TRACE=1 PAVONIS_GNB_SI_OCCASION_TRACE_LIMIT=4096 \
        PAVONIS_GNB_RAR_PDU_TRACE=1 PAVONIS_GNB_RAR_SCHED_TRACE=1 \
        PAVONIS_GNB_RAR_SCHED_TRACE_LIMIT=256 PAVONIS_GNB_RAR_FAPI_TRACE=1 \
        PAVONIS_GNB_RAR_FAPI_TRACE_LIMIT=256 PAVONIS_GNB_RAR_TX_WORKER_TRACE_ARM=1 \
        PAVONIS_PRACH_COLLECTION_DELAY_SAMPLES=0 PAVONIS_PRACH_COLLECTION_EXTRA_SAMPLES=12000 \
        PAVONIS_PRACH_COLLECTION_PHASE_SEQUENCE="$PRACH_COLLECTION_PHASE_SEQUENCE" \
        PAVONIS_PRACH_TIMING_COMPENSATION_SAMPLES="$PRACH_TIMING_COMPENSATION_SAMPLES" \
        PAVONIS_PRACH_TIMING_COMPENSATION_REPORT_SLOT_OFFSET="$PRACH_REPORT_SLOT_OFFSET" \
        PAVONIS_LOWER_PHY_UL_RX_TIMESTAMP_OFFSET_SAMPLES=0 \
        PAVONIS_PRACH_DEMOD_INPUT_STAGE1_POLICY=1 \
        PAVONIS_PRACH_DEMOD_INPUT_STAGE1_CANDIDATE_LIST=-192:-0.5:20000,-192:-0.5:35000,-192:-0.5:45000,64:-0.5:5000,64:-0.5:10000,64:-0.5:15000,64:-0.5:20000 \
        PAVONIS_PRACH_DEMOD_INPUT_STAGE1_RMS_DB_FLOOR=-70 \
        PAVONIS_PRACH_DEMOD_INPUT_STAGE1_PEAK_DB_FLOOR=-55 \
        PAVONIS_PRACH_DEMOD_INPUT_STAGE1_MAX_CANDIDATES=8 \
        PAVONIS_PRACH_DEMOD_INPUT_STAGE1_STOP_ON_ACCEPT=1 \
        PAVONIS_PRACH_DEMOD_INPUT_STAGE1_TRACE=1 \
        PAVONIS_PRACH_DEMOD_INPUT_STAGE1_TRACE_MAX=8192 \
        PAVONIS_PRACH_STAGE1_TA_COMPENSATE_SELECTED_SHIFT=1 \
        PAVONIS_PRACH_PREDEMOD_SUMMARY_MAX=1700 PAVONIS_PRACH_PREDEMOD_RAW_MAX=0 \
        PAVONIS_PRACH_STRONG_PREDEMOD_CAPTURE=1 PAVONIS_PRACH_STRONG_PREDEMOD_RAW_MAX=64 \
        PAVONIS_PRACH_STRONG_PREDEMOD_CAPTURE_STAGE1_ENERGY=1 \
        PAVONIS_PRACH_EARLY_PREDEMOD_CAPTURE_MAX="$PRACH_EARLY_CAPTURE_MAX" \
        PAVONIS_PRACH_STRONG_PREDEMOD_RMS_DB_FLOOR=-70 \
        PAVONIS_PRACH_STRONG_PREDEMOD_PEAK_DB_FLOOR=-60 \
        PAVONIS_NON_PRACH_UL_RX_TIMESTAMP_OFFSET_SAMPLES=-241118 \
        PAVONIS_NON_PRACH_UL_RX_PIPELINE_DEPTH_SLOTS=40 \
        PAVONIS_NON_PRACH_UL_RX_OFFSET_FROM_STAGE1=1 \
        PAVONIS_NON_PRACH_UL_RX_OFFSET_NEGATIVE_SHIFT_SAMPLES=-241118 \
        PAVONIS_NON_PRACH_UL_RX_OFFSET_POSITIVE_SHIFT_SAMPLES=-237798 \
        PAVONIS_NON_PRACH_UL_RX_OFFSET_FROM_PRACH_EDGE=1 \
        PAVONIS_NON_PRACH_UL_RX_OFFSET_EDGE_INCLUDE_RAR_TA=1 \
        PAVONIS_NON_PRACH_UL_RX_OFFSET_EDGE_CALIBRATION_SAMPLES=-237332 \
        PAVONIS_NON_PRACH_UL_RX_OFFSET_EDGE_SCHEDULER_ERROR_SAMPLES=0 \
        PAVONIS_NON_PRACH_UL_RX_OFFSET_EDGE_BLOCK_SIZE=8 \
        PAVONIS_NON_PRACH_UL_RX_OFFSET_EDGE_HOST_ACTIVE_STOP_SAMPLES=10404 \
        PAVONIS_NON_PRACH_UL_RX_CFO_HZ=-4000 \
        PAVONIS_UL_SYMBOL_DUMP="$MSG3_TARGET_CAPTURE" \
        PAVONIS_UL_SYMBOL_DUMP_ARM_MODE=scheduled_msg3 \
        PAVONIS_UL_SYMBOL_DUMP_ARM_FILE="$UL_SYMBOL_DUMP_ARM_FILE" \
        PAVONIS_UL_SYMBOL_SUMMARY_MAX=260000 \
        PAVONIS_UL_SYMBOL_METADATA_ONLY=1 \
        PAVONIS_UL_SYMBOL_RAW_MAX="$UL_SYMBOL_RAW_MAX" \
        PAVONIS_UL_SYMBOL_RAW_PEAK_MIN=1e9 \
        PAVONIS_UL_SYMBOL_RAW_RMS_MIN=1e9 \
        PAVONIS_UL_SYMBOL_MSG3_TARGET_CAPTURE="$MSG3_TARGET_CAPTURE" \
        PAVONIS_UL_SYMBOL_MSG3_TARGET_MAX="$MSG3_TARGET_MAX" \
        PAVONIS_SOAPY_ALLOW_M2SDR_CONTINUOUS_TX_MODE=1 \
        PAVONIS_SOAPY_TX_FORCE_CONTINUOUS_STREAM=1 \
        PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS=1 PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS_OVERRIDE=1 \
        PAVONIS_SOAPY_TX_FORCE_ATT_DB=0 PAVONIS_SOAPY_TX_REANCHOR_INTERVAL_SAMPLES=0 \
        PAVONIS_SOAPY_TX_DEADLINE_WRITE_OVERRIDE=1 \
        PAVONIS_SOAPY_TX_DEADLINE_WRITE_GUARD_US_OVERRIDE=2000 \
        PAVONIS_SOAPY_TX_DEADLINE_WRITE_MAX_TIMEOUT_US_OVERRIDE=50000 \
        PAVONIS_SOAPY_TX_TIMEOUT_RETRY_OVERRIDE=0 \
        M2SDR_LITEPCIE_ZERO_COPY=1 M2SDR_SOAPY_TX_COPY_PRIME_BUFFERS=8 \
        M2SDR_SOAPY_TX_ZERO_COPY_PRIME_BUFFERS=128 M2SDR_SOAPY_TX_ZERO_STALE_PAYLOAD=0 \
        M2SDR_SOAPY_TX_BACKFILL_CONTIGUOUS_TIME=1 \
        M2SDR_SOAPY_TX_TIME_OFFSET_NS=20000000 M2SDR_SOAPY_TX_FULL_RING_REFRESH_US=50 \
        M2SDR_SOAPY_TX_WORKER_PKT_COUNT=64 M2SDR_SOAPY_TX_WORKER_TRACE_LIMIT=0 \
        M2SDR_SOAPY_TX_CS16_TO_SC12=1 M2SDR_SOAPY_TX_ALIGN_HAS_TIME_REMAINDER=1 \
        OCUDU_SOAPY_TX_WRITE_TIMEOUT_US=1000 OCUDU_SOAPY_TX_SUMMARY=1 \
        OCUDU_SOAPY_TX_SUMMARY_PERIOD=100 OCUDU_SOAPY_TX_SUPPRESS_EMPTY_EOB=1 \
        OCUDU_SOAPY_RX_USE_READSTREAM=1 PAVONIS_SOAPY_RX_APPEND_ENABLE=0 \
        SOAPY_SDR_ROOT=/tmp/empty-soapy-root-cp2892 \
        SOAPY_SDR_PLUGIN_PATH=@REMOTE_HOME@/CLionProjects/m2sdr/litex_m2sdr/software/soapysdr/build \
        SOAPY_SDR_LOG_LEVEL=INFO \
        PAVONIS_QCORE_SIM_FILE="$VALIDATION/configs/sims-pavonis-ota.private.toml" \
        "$STARTER"
    ) >"$START_LOG" 2>&1
    start_rc=$?
    set -e
    if [[ "$start_rc" != 0 ]]; then
      tail -n 100 "$START_LOG"
      exit "$start_rc"
    fi

    READY="$OUT/TX_READY.env"
    [[ -s "$READY" ]]
    HARNESS_PID="$(awk -F= '$1=="HARNESS_PID"{print $2}' "$READY" | tail -n 1)"
    OTA="$(awk -F= '$1=="OTA"{print $2}' "$READY" | tail -n 1)"
    GNB_LOG="$(awk -F= '$1=="GNB_LOG"{print $2}' "$READY" | tail -n 1)"
    TX_READBACK="$(awk -F= '$1=="TX_READBACK_PATH"{print $2}' "$READY" | tail -n 1)"
    GNB_PID="$(awk '$1=="gnb"{print $2}' "$OTA/pids.txt" | tail -n 1)"
    STAGE_PID="$(awk '$1=="stage"{print $2}' "$OTA/pids.txt" | tail -n 1)"
    [[ "$HARNESS_PID" =~ ^[0-9]+$ && "$GNB_PID" =~ ^[0-9]+$ && "$STAGE_PID" =~ ^[0-9]+$ ]]
    kill -0 "$GNB_PID"
    grep -q 'N2: Connection to AMF on @CORE_N2_IP@:38412 completed' "$GNB_LOG"
    [[ -s "$TX_READBACK" ]]
    grep -q '^configured_discontinuous_tx=0$' "$TX_READBACK"
    grep -q '^tx_force_continuous_stream=1$' "$TX_READBACK"
    grep -q '^tx_all_timed_chunks=1$' "$TX_READBACK"
    grep -q '^env_PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS=1$' "$TX_READBACK"
    tr '\0' '\n' <"/proc/$GNB_PID/environ" | grep -qx 'PAVONIS_SOAPY_TX_FORCE_ATT_DB=16'
    awk -F '\t' '
      $1 ~ /^[0-9]+$/ && $6 == 1 && ($7 + 0) >= 15.99 && ($7 + 0) <= 16.01 { ok=1 }
      END { exit !ok }
    ' "$TX_READBACK"
    PROGRAM_MARKER_AFTER="$(sudo -n dmesg | grep -F -c 'Reader one-shot program mode active' || true)"
    ((PROGRAM_MARKER_AFTER > PROGRAM_MARKER_BEFORE))
    [[ $(cat "$PROGRAM_MODE") == Y ]]

    {
      printf 'ROOT=%q\n' "$ROOT"
      printf 'OUT=%q\n' "$OUT"
      printf 'READY=%q\n' "$READY"
      printf 'OTA=%q\n' "$OTA"
      printf 'GNB_LOG=%q\n' "$GNB_LOG"
      printf 'TX_READBACK=%q\n' "$TX_READBACK"
      printf 'HARNESS_PID=%q\n' "$HARNESS_PID"
      printf 'STAGE_PID=%q\n' "$STAGE_PID"
      printf 'GNB_PID=%q\n' "$GNB_PID"
      printf 'PROGRAM_MARKER_BEFORE=%q\n' "$PROGRAM_MARKER_BEFORE"
      printf 'PROGRAM_MARKER_AFTER=%q\n' "$PROGRAM_MARKER_AFTER"
      printf 'TARGET_PCI=%q\n' "$TARGET_PCI"
      printf 'PRACH_COLLECTION_PHASE_MODE=%q\n' "$PRACH_COLLECTION_PHASE_MODE"
      printf 'PRACH_EARLY_CAPTURE_MAX=%q\n' "$PRACH_EARLY_CAPTURE_MAX"
      printf 'PRACH_TIMING_COMPENSATION_SAMPLES=%q\n' "$PRACH_TIMING_COMPENSATION_SAMPLES"
      printf 'PRACH_REPORT_SLOT_OFFSET=%q\n' "$PRACH_REPORT_SLOT_OFFSET"
      printf 'RA_RESP_WINDOW=%q\n' "$RA_RESP_WINDOW"
      printf 'ALLOW_EXTENDED_RA_RESP_WINDOW=%q\n' "$ALLOW_EXTENDED_RA_RESP_WINDOW"
      printf 'MSG3_TARGET_CAPTURE=%q\n' "$MSG3_TARGET_CAPTURE"
      printf 'MSG3_TARGET_MAX=%q\n' "$MSG3_TARGET_MAX"
      printf 'UL_SYMBOL_RAW_MAX=%q\n' "$UL_SYMBOL_RAW_MAX"
    } >"$STATE"
    start_failed=0
    trap - EXIT INT TERM HUP
    echo "CP3586_OCUDU_RUNTIME_ARTIFACT_PINS=PASS"
    echo "CP3059_OCUDU_M2SDR_STAGE_FORCE_ATT16_START=PASS pid=$GNB_PID root=$ROOT gnb_sha=$GNB_SHA soapy_source_gate=not_runtime_artifact soapy_build_gate=not_runtime_artifact soapy_module_sha=$SOAPY_MODULE_SHA stage_sha=$STAGE_SHA physical_channel=$PHYSICAL_CHANNEL_LABEL tx_attenuation_db=16 tx_attenuation_readback_db=16 prach_collection_phase_mode=$PRACH_COLLECTION_PHASE_MODE prach_early_capture_max=$PRACH_EARLY_CAPTURE_MAX prach_timing_compensation_samples=$PRACH_TIMING_COMPENSATION_SAMPLES prach_report_slot_offset=$PRACH_REPORT_SLOT_OFFSET ra_resp_window=$RA_RESP_WINDOW allow_extended_ra_resp_window=$ALLOW_EXTENDED_RA_RESP_WINDOW msg3_target_capture=$MSG3_TARGET_CAPTURE msg3_target_max=$MSG3_TARGET_MAX ul_symbol_raw_max=$UL_SYMBOL_RAW_MAX"
    echo "TARGET_PCI=$TARGET_PCI"
    printf 'PROGRAM_MARKER_BEFORE=%s\nPROGRAM_MARKER_AFTER=%s\n' "$PROGRAM_MARKER_BEFORE" "$PROGRAM_MARKER_AFTER"
    grep -m1 'N2: Connection to AMF on @CORE_N2_IP@:38412 completed' "$GNB_LOG"
    grep -E '^(configured_discontinuous_tx|tx_force_continuous_stream|tx_all_timed_chunks|env_PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS)=' "$TX_READBACK"
    ;;
  status)
    load_state
    kill -0 "$GNB_PID"
    [[ $(cat "$PROGRAM_MODE") == Y ]]
    snapshot_log
    echo "CP3059_OCUDU_M2SDR_STAGE_FORCE_ATT16_STATUS=RUNNING pid=$GNB_PID"
    compact_status "$SNAPSHOT"
    ;;
  stop)
    sudo -n true
    stop_processes
    package_msg3_capture
    [[ -z $(pgrep -x gnb || true) ]]
    [[ $(cat "$PROGRAM_MODE") == N ]]
    echo CP3059_OCUDU_M2SDR_STAGE_FORCE_ATT16_STOP=PASS
    ;;
  probe-phase-mode)
    printf 'CP3042_PRACH_COLLECTION_PHASE_MODE=%s\n' "$PRACH_COLLECTION_PHASE_MODE"
    printf 'CP3042_PRACH_COLLECTION_PHASE_SEQUENCE=%s\n' "$PRACH_COLLECTION_PHASE_SEQUENCE"
    printf 'CP3044_PRACH_EARLY_CAPTURE_MAX=%s\n' "$PRACH_EARLY_CAPTURE_MAX"
    printf 'CP3073_PRACH_TIMING_COMPENSATION_SAMPLES=%s\n' "$PRACH_TIMING_COMPENSATION_SAMPLES"
    printf 'CP3073_PRACH_REPORT_SLOT_OFFSET=%s\n' "$PRACH_REPORT_SLOT_OFFSET"
    printf 'CP3073_RA_RESP_WINDOW=%s\n' "$RA_RESP_WINDOW"
    printf 'CP3080_ALLOW_EXTENDED_RA_RESP_WINDOW=%s\n' "$ALLOW_EXTENDED_RA_RESP_WINDOW"
    printf 'CP3082_MSG3_TARGET_CAPTURE=%s\n' "$MSG3_TARGET_CAPTURE"
    printf 'CP3082_MSG3_TARGET_MAX=%s\n' "$MSG3_TARGET_MAX"
    printf 'CP3082_UL_SYMBOL_RAW_MAX=%s\n' "$UL_SYMBOL_RAW_MAX"
    printf 'CP3082_UL_SYMBOL_DUMP=%s\n' "$MSG3_TARGET_CAPTURE"
    printf 'CP3082_UL_SYMBOL_DUMP_ARM_MODE=%s\n' scheduled_msg3
    printf 'CP3082_UL_SYMBOL_METADATA_ONLY=%s\n' 1
    printf 'CP3047_PHYSICAL_CHANNEL=%s\n' "$PHYSICAL_CHANNEL_LABEL"
    printf 'CP3059_TX_ATTENUATION_DB=%s\n' 16
    printf 'CP3059_FORCE_ATT_ENV=%s\n' PAVONIS_SOAPY_TX_FORCE_ATT_DB
    printf 'CP3059_STAGE_SHA256=%s\n' "$STAGE_SHA"
    echo CP3042_PRACH_COLLECTION_PHASE_PROBE=PASS
    ;;
  *)
    echo 'usage: helper start|status|stop|probe-phase-mode STAMP' >&2
    exit 64
    ;;
esac
