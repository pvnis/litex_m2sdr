#!/usr/bin/env bash
set +e +u
set +o pipefail 2>/dev/null || true

if [ "$(hostname)" != "ran" ]; then
  echo "ERROR: this must run on ran, current host is: $(hostname)"
  exit 2
fi

sudo -v || exit 3

TAG="${TAG:-codex_stage6_cp1660_txatt5_nativecontinuous_alltimed_lead30ms_stage6_prachtrace}"
STAMP="${STAMP:-$(date -u +%Y%m%dT%H%M%SZ)}"
ARM="${ARM:-/tmp/pavonis_prach_stage6_cp1660_nativecontinuous_alltimed_lead30ms_${STAMP}_arm}"
PRACH_SEGMENT_ARM="${PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE_ARM_FILE:-$ARM}"
PRACH_LATE_CONTEXT_TRACE_PATH="${PAVONIS_PRACH_LATE_CONTEXT_TRACE_PATH:-/tmp/pavonis_prach_late_context_trace.tsv}"
RX_APPEND_ARM="${PAVONIS_SOAPY_RX_APPEND_ARM_FILE:-${RX_APPEND_ARM:-$ARM}}"
PAVONIS_SOAPY_RX_APPEND_IMMEDIATE="${PAVONIS_SOAPY_RX_APPEND_IMMEDIATE:-0}"
PAVONIS_SOAPY_RX_FORCE_CENTER_HZ="${PAVONIS_SOAPY_RX_FORCE_CENTER_HZ:-}"
OCUDU_SOAPY_RX_BW_HZ="${OCUDU_SOAPY_RX_BW_HZ:-}"
UL_SYMBOL_ARM="${PAVONIS_UL_SYMBOL_DUMP_ARM_FILE:-$ARM}"
DURATION="${DURATION:-420}"
TX_BACKOFF="${TX_BACKOFF:-10}"
TX_ATT="${TX_ATT:-5}"
GNB_RX_GAIN="${GNB_RX_GAIN:-40}"
M2SDR_CLOCK_SOURCE="${PAVONIS_M2SDR_CLOCK_SOURCE:-}"
GNB_CLOCK_PPM="${PAVONIS_GNB_CLOCK_PPM:-}"
GNB_PLMN="${PAVONIS_GNB_PLMN:-}"
GNB_ID="${PAVONIS_GNB_ID:-}"
GNB_ID_BIT_LENGTH="${PAVONIS_GNB_ID_BIT_LENGTH:-}"
GNB_SECTOR_ID="${PAVONIS_GNB_SECTOR_ID:-}"
GNB_PCI="${PAVONIS_GNB_PCI:-}"
GNB_TAC="${PAVONIS_GNB_TAC:-}"
GNB_PRACH_FREQUENCY_START="${PAVONIS_GNB_PRACH_FREQUENCY_START:-}"
GNB_PRACH_CONFIG_INDEX="${PAVONIS_GNB_PRACH_CONFIG_INDEX:-}"
GNB_TOTAL_NOF_RA_PREAMBLES="${PAVONIS_GNB_TOTAL_NOF_RA_PREAMBLES:-}"
GNB_PREAMBLE_RX_TARGET_PW="${PAVONIS_GNB_PREAMBLE_RX_TARGET_PW:-}"
GNB_Q_QUAL_MIN="${PAVONIS_GNB_Q_QUAL_MIN:-}"
OTA_STAGE_TEMPLATE="${PAVONIS_OTA_STAGE_TEMPLATE_OVERRIDE:-./scripts/run_ota_stage_template.sh}"
OCUDU_GNB_BIN_OVERRIDE="${OCUDU_GNB_BIN:-}"
GNB_MIB_INTRAFREQ_RESELECTION_ALLOWED="${PAVONIS_GNB_MIB_INTRAFREQ_RESELECTION_ALLOWED:-0}"
GNB_SIB1_RETX_PERIOD_MS="${PAVONIS_GNB_SIB1_RETX_PERIOD_MS:-0}"
GNB_SIB1_USE_N0="${PAVONIS_GNB_SIB1_USE_N0:-0}"
GNB_SRSENB_SIB1_COMPAT="${PAVONIS_GNB_SRSENB_SIB1_COMPAT:-0}"
GNB_MSG3_DELTA_PREAMBLE="${PAVONIS_GNB_MSG3_DELTA_PREAMBLE:-}"
PRACH_STRONG_CAPTURE_STAGE1_ENERGY="${PAVONIS_PRACH_STRONG_PREDEMOD_CAPTURE_STAGE1_ENERGY:-0}"
PRACH_STAGE1_CANDIDATE_LIST="${PAVONIS_PRACH_DEMOD_INPUT_STAGE1_CANDIDATE_LIST:-}"
PRACH_STAGE1_MAX_CANDIDATES="${PAVONIS_PRACH_DEMOD_INPUT_STAGE1_MAX_CANDIDATES:-1024}"
PHYSICAL_CHANNEL_OFFSET="${PAVONIS_SOAPY_PHYSICAL_CHANNEL_OFFSET:-}"
ROOT="$PWD"

case "$GNB_MIB_INTRAFREQ_RESELECTION_ALLOWED" in
  0|1) ;;
  *) echo "ERROR: PAVONIS_GNB_MIB_INTRAFREQ_RESELECTION_ALLOWED must be 0 or 1" >&2; exit 2 ;;
esac
case "$GNB_SIB1_RETX_PERIOD_MS" in
  0|20) ;;
  *) echo "ERROR: PAVONIS_GNB_SIB1_RETX_PERIOD_MS must be 0 or 20" >&2; exit 2 ;;
esac
case "$GNB_SIB1_USE_N0" in
  0|1) ;;
  *) echo "ERROR: PAVONIS_GNB_SIB1_USE_N0 must be 0 or 1" >&2; exit 2 ;;
esac
case "$GNB_SRSENB_SIB1_COMPAT" in
  0|1) ;;
  *) echo "ERROR: PAVONIS_GNB_SRSENB_SIB1_COMPAT must be 0 or 1" >&2; exit 2 ;;
esac
case "$PRACH_STRONG_CAPTURE_STAGE1_ENERGY" in
  0|1) ;;
  *) echo "ERROR: PAVONIS_PRACH_STRONG_PREDEMOD_CAPTURE_STAGE1_ENERGY must be 0 or 1" >&2; exit 2 ;;
esac
if [[ ! "$PRACH_STAGE1_MAX_CANDIDATES" =~ ^[0-9]+$ ]] || ((PRACH_STAGE1_MAX_CANDIDATES > 4096)); then
  echo "ERROR: PAVONIS_PRACH_DEMOD_INPUT_STAGE1_MAX_CANDIDATES must be 0..4096" >&2
  exit 2
fi
case "$GNB_MSG3_DELTA_PREAMBLE" in
  ''|-1|0|1|2|3|4|5|6) ;;
  *) echo "ERROR: PAVONIS_GNB_MSG3_DELTA_PREAMBLE must be empty or -1 through 6" >&2; exit 2 ;;
esac

IFACE="${IFACE:-$(ip route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="dev"){print $(i+1); exit}}')}"
[ -n "$IFACE" ] || IFACE="@RAN_INTERFACE@"

ALLOW_CONTINUOUS_MODE="${PAVONIS_SOAPY_ALLOW_M2SDR_CONTINUOUS_TX_MODE:-1}"
FORCE_CONTINUOUS_STREAM="${PAVONIS_SOAPY_TX_FORCE_CONTINUOUS_STREAM:-0}"
ALL_TIMED_CHUNKS="${PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS_OVERRIDE:-${PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS:-1}}"
REANCHOR_INTERVAL_SAMPLES="${PAVONIS_SOAPY_TX_REANCHOR_INTERVAL_SAMPLES:-0}"
ALIGN_HAS_TIME_REMAINDER="${M2SDR_SOAPY_TX_ALIGN_HAS_TIME_REMAINDER:-1}"
LITEPCIE_ZERO_COPY="${M2SDR_LITEPCIE_ZERO_COPY:-1}"
TX_COPY_PRIME_BUFFERS="${M2SDR_SOAPY_TX_COPY_PRIME_BUFFERS:-8}"
TX_ZERO_COPY_PRIME_BUFFERS="${M2SDR_SOAPY_TX_ZERO_COPY_PRIME_BUFFERS:-1}"
case "$ALL_TIMED_CHUNKS" in
  0|1) ;;
  *) echo "ERROR: PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS_OVERRIDE must resolve to 0 or 1" >&2; exit 2 ;;
esac
case "$LITEPCIE_ZERO_COPY" in
  0|1) ;;
  *) echo "ERROR: M2SDR_LITEPCIE_ZERO_COPY must resolve to 0 or 1" >&2; exit 2 ;;
esac
[[ "$TX_COPY_PRIME_BUFFERS" =~ ^[0-9]+$ ]] &&
  (( TX_COPY_PRIME_BUFFERS >= 1 && TX_COPY_PRIME_BUFFERS <= 16 )) || {
  echo "ERROR: M2SDR_SOAPY_TX_COPY_PRIME_BUFFERS must resolve to 1..16" >&2
  exit 2
}
[[ "$TX_ZERO_COPY_PRIME_BUFFERS" =~ ^[0-9]+$ ]] &&
  (( TX_ZERO_COPY_PRIME_BUFFERS >= 1 && TX_ZERO_COPY_PRIME_BUFFERS <= 256 )) || {
  echo "ERROR: M2SDR_SOAPY_TX_ZERO_COPY_PRIME_BUFFERS must resolve to 1..256" >&2
  exit 2
}
if [ -n "$GNB_PLMN" ] && [[ ! "$GNB_PLMN" =~ ^[0-9]{5,6}$ ]]; then
  echo "ERROR: PAVONIS_GNB_PLMN must contain five or six digits" >&2
  exit 2
fi
if [ -n "$GNB_ID" ] && { [[ ! "$GNB_ID" =~ ^[0-9]+$ ]] || (( GNB_ID > 4294967295 )); }; then
  echo "ERROR: PAVONIS_GNB_ID must be an integer from 0 through 4294967295" >&2
  exit 2
fi
if [ -n "$GNB_ID_BIT_LENGTH" ] &&
   { [[ ! "$GNB_ID_BIT_LENGTH" =~ ^[0-9]+$ ]] ||
     (( GNB_ID_BIT_LENGTH < 22 || GNB_ID_BIT_LENGTH > 32 )); }; then
  echo "ERROR: PAVONIS_GNB_ID_BIT_LENGTH must be an integer from 22 through 32" >&2
  exit 2
fi
if [ -n "$GNB_SECTOR_ID" ] && [[ ! "$GNB_SECTOR_ID" =~ ^[0-9]+$ ]]; then
  echo "ERROR: PAVONIS_GNB_SECTOR_ID must be a non-negative integer" >&2
  exit 2
fi
if [ -n "$GNB_ID_BIT_LENGTH" ] || [ -n "$GNB_SECTOR_ID" ]; then
  if [ -z "$GNB_ID" ] || [ -z "$GNB_ID_BIT_LENGTH" ] || [ -z "$GNB_SECTOR_ID" ]; then
    echo "ERROR: PAVONIS_GNB_ID, PAVONIS_GNB_ID_BIT_LENGTH, and PAVONIS_GNB_SECTOR_ID must be set together" >&2
    exit 2
  fi
  if (( GNB_ID >= (1 << GNB_ID_BIT_LENGTH) )); then
    echo "ERROR: PAVONIS_GNB_ID does not fit PAVONIS_GNB_ID_BIT_LENGTH" >&2
    exit 2
  fi
  if (( GNB_SECTOR_ID >= (1 << (36 - GNB_ID_BIT_LENGTH)) )); then
    echo "ERROR: PAVONIS_GNB_SECTOR_ID does not fit the remaining NCI bits" >&2
    exit 2
  fi
fi
if [ -n "$GNB_PCI" ] && { [[ ! "$GNB_PCI" =~ ^[0-9]+$ ]] || (( GNB_PCI > 1007 )); }; then
  echo "ERROR: PAVONIS_GNB_PCI must be an integer from 0 through 1007" >&2
  exit 2
fi
if [ -n "$GNB_TAC" ] && { [[ ! "$GNB_TAC" =~ ^[0-9]+$ ]] || (( GNB_TAC > 16777215 )); }; then
  echo "ERROR: PAVONIS_GNB_TAC must be an integer from 0 through 16777215" >&2
  exit 2
fi
if [ -n "$GNB_PRACH_FREQUENCY_START" ] &&
   { [[ ! "$GNB_PRACH_FREQUENCY_START" =~ ^[0-9]+$ ]] || (( GNB_PRACH_FREQUENCY_START > 274 )); }; then
  echo "ERROR: PAVONIS_GNB_PRACH_FREQUENCY_START must be an integer from 0 through 274" >&2
  exit 2
fi
if [ -n "$GNB_PRACH_CONFIG_INDEX" ] &&
   { [[ ! "$GNB_PRACH_CONFIG_INDEX" =~ ^[0-9]+$ ]] || (( GNB_PRACH_CONFIG_INDEX > 255 )); }; then
  echo "ERROR: PAVONIS_GNB_PRACH_CONFIG_INDEX must be an integer from 0 through 255" >&2
  exit 2
fi
if [ -n "$GNB_TOTAL_NOF_RA_PREAMBLES" ] &&
   { [[ ! "$GNB_TOTAL_NOF_RA_PREAMBLES" =~ ^[0-9]+$ ]] ||
     (( GNB_TOTAL_NOF_RA_PREAMBLES < 1 || GNB_TOTAL_NOF_RA_PREAMBLES > 64 )); }; then
  echo "ERROR: PAVONIS_GNB_TOTAL_NOF_RA_PREAMBLES must be an integer from 1 through 64" >&2
  exit 2
fi
if [ -n "$GNB_PREAMBLE_RX_TARGET_PW" ] &&
   { [[ ! "$GNB_PREAMBLE_RX_TARGET_PW" =~ ^-?[0-9]+$ ]] ||
     (( GNB_PREAMBLE_RX_TARGET_PW < -202 || GNB_PREAMBLE_RX_TARGET_PW > -60 || GNB_PREAMBLE_RX_TARGET_PW % 2 != 0 )); }; then
  echo "ERROR: PAVONIS_GNB_PREAMBLE_RX_TARGET_PW must be an even integer from -202 through -60" >&2
  exit 2
fi
if [ -n "$GNB_Q_QUAL_MIN" ] &&
   { [[ ! "$GNB_Q_QUAL_MIN" =~ ^-?[0-9]+$ ]] ||
     (( GNB_Q_QUAL_MIN < -43 || GNB_Q_QUAL_MIN > -12 )); }; then
  echo "ERROR: PAVONIS_GNB_Q_QUAL_MIN must be an integer from -43 through -12" >&2
  exit 2
fi
if [ ! -x "$OTA_STAGE_TEMPLATE" ]; then
  echo "ERROR: OTA stage template is not executable: $OTA_STAGE_TEMPLATE" >&2
  exit 2
fi
GNB_SI_OCCASION_TRACE="${PAVONIS_GNB_SI_OCCASION_TRACE:-0}"
GNB_SI_OCCASION_TRACE_PATH="${PAVONIS_GNB_SI_OCCASION_TRACE_PATH:-/tmp/pavonis_gnb_si_occasion_trace.tsv}"
GNB_SI_OCCASION_TRACE_LIMIT="${PAVONIS_GNB_SI_OCCASION_TRACE_LIMIT:-4096}"
GNB_RAR_SCHED_TRACE="${PAVONIS_GNB_RAR_SCHED_TRACE:-0}"
GNB_RAR_SCHED_TRACE_LIMIT="${PAVONIS_GNB_RAR_SCHED_TRACE_LIMIT:-128}"
GNB_RAR_FAPI_TRACE_PATH="${PAVONIS_GNB_RAR_FAPI_TRACE_PATH:-/tmp/pavonis_gnb_rar_fapi_trace.tsv}"
TX_WORKER_TRACE_PATH="${M2SDR_SOAPY_TX_WORKER_TRACE_PATH:-}"
TX_WORKER_TRACE_LIMIT="${M2SDR_SOAPY_TX_WORKER_TRACE_LIMIT:-65536}"
TX_WORKER_TRACE_FLUSH_PERIOD="${M2SDR_SOAPY_TX_WORKER_TRACE_FLUSH_PERIOD:-256}"
TX_WORKER_PKT_COUNT="${M2SDR_SOAPY_TX_WORKER_PKT_COUNT:-16}"
DEADLINE_WRITE_OVERRIDE="${PAVONIS_SOAPY_TX_DEADLINE_WRITE_OVERRIDE:-0}"
DEADLINE_WRITE_GUARD_US_OVERRIDE="${PAVONIS_SOAPY_TX_DEADLINE_WRITE_GUARD_US_OVERRIDE:-2000}"
DEADLINE_WRITE_MAX_TIMEOUT_US_OVERRIDE="${PAVONIS_SOAPY_TX_DEADLINE_WRITE_MAX_TIMEOUT_US_OVERRIDE:-50000}"
TX_WRITE_DUMP_CF32_PATH="${PAVONIS_SOAPY_TX_WRITE_DUMP_CF32_PATH:-}"
TX_WRITE_DUMP_META_PATH="${PAVONIS_SOAPY_TX_WRITE_DUMP_META_PATH:-}"
TX_WRITE_DUMP_MAX_SAMPLES="${PAVONIS_SOAPY_TX_WRITE_DUMP_MAX_SAMPLES:-0}"
TX_WRITE_DUMP_PORT="${PAVONIS_SOAPY_TX_WRITE_DUMP_PORT:-0}"
TX_PLUGIN_DUMP_PATH="${M2SDR_SOAPY_TX_DUMP_PATH:-}"
TX_PLUGIN_DUMP_SAMPLES="${M2SDR_SOAPY_TX_DUMP_SAMPLES:-0}"
TX_DMA_DUMP_PATH="${M2SDR_SOAPY_TX_DMA_DUMP_PATH:-}"
TX_DMA_DUMP_BUFFERS="${M2SDR_SOAPY_TX_DMA_DUMP_BUFFERS:-0}"
TX_PROBE_ARM_FILE="${M2SDR_SOAPY_TX_PROBE_ARM_FILE:-}"
TX_PROBE_ARM_POLL_PERIOD="${M2SDR_SOAPY_TX_PROBE_ARM_POLL_PERIOD:-1024}"
[[ "$TX_PROBE_ARM_POLL_PERIOD" =~ ^[1-9][0-9]*$ ]] || {
  echo "ERROR: M2SDR_SOAPY_TX_PROBE_ARM_POLL_PERIOD must be a positive integer" >&2
  exit 2
}
RAR_WINDOW_OFFSET_SLOTS="${PAVONIS_RACH_RAR_WINDOW_OFFSET_SLOTS:-}"
GNB_RA_RESP_WINDOW="${PAVONIS_GNB_RA_RESP_WINDOW:-}"
OUT="${OUT:-$ROOT/runs/${TAG}_${STAMP}_txatt${TX_ATT}_rxgain${GNB_RX_GAIN}_nativecontinuous_alltimed_lead30ms_stage6}"
mkdir -p "$OUT"

GNB_CONF="$OUT/gnb_m2sdr_band3_ota.txatt${TX_ATT}.backoff${TX_BACKOFF}.rxgain${GNB_RX_GAIN}.yml"
SIM_FILE="${PAVONIS_QCORE_SIM_FILE:-$ROOT/configs/sims-ocudu-zmq.example.toml}"
UE_CONF="$ROOT/configs/ue-bladerf-band3-ota.template.conf"

HARNESS_LOG="$OUT/harness.log"
WATCH_LOG="$OUT/watch.log"
RUNNER="$OUT/run_harness.sh"
READY="$OUT/TX_READY.env"
SUMMARY="$OUT/SUMMARY.txt"
BEFORE="$OUT/ota_dirs_before.txt"
NOW="$OUT/ota_dirs_now.txt"
TX_READBACK="${PAVONIS_SOAPY_TX_READBACK_PATH:-$OUT/pavonis_soapy_tx_readback.txt}"

rm -f "$ARM"
[ "$PRACH_SEGMENT_ARM" = "$ARM" ] || rm -f "$PRACH_SEGMENT_ARM"
[ "$RX_APPEND_ARM" = "$ARM" ] || rm -f "$RX_APPEND_ARM"
if [ "$PAVONIS_SOAPY_RX_APPEND_IMMEDIATE" = "1" ]; then
  touch "$RX_APPEND_ARM"
fi
[ "$UL_SYMBOL_ARM" = "$ARM" ] || rm -f "$UL_SYMBOL_ARM"
rm -f "$TX_READBACK"
[ -n "$TX_PROBE_ARM_FILE" ] && rm -f "$TX_PROBE_ARM_FILE"
rm -f /tmp/pavonis_prachdet_trace.log
rm -f /tmp/pavonis_prach_predemod_manifest.tsv
rm -f /tmp/pavonis_prach_collection_segments.tsv
[ -n "$PRACH_LATE_CONTEXT_TRACE_PATH" ] && rm -f "$PRACH_LATE_CONTEXT_TRACE_PATH"
rm -f /tmp/pavonis_prach_predemod_*.cf32
rm -f /tmp/pavonis_ul_symbol_manifest.tsv
rm -f /tmp/pavonis_ul_symbol_*.cf32
rm -f /tmp/pavonis_pucch_f1_grid_*.cf32
rm -f /tmp/pavonis_soapy_rx_manifest.tsv
rm -f /tmp/pavonis_soapy_rx_append*.cf32
rm -f /tmp/pavonis_soapy_rx_append_meta.tsv
rm -f /tmp/pavonis_soapy_rx_readback.txt
rm -f /tmp/pavonis_soapy_rx_*.cf32
rm -f /tmp/gnb_m2sdr_band3_ota.log
[ -n "$GNB_SI_OCCASION_TRACE_PATH" ] && rm -f "$GNB_SI_OCCASION_TRACE_PATH"
[ -n "$GNB_RAR_FAPI_TRACE_PATH" ] && rm -f "$GNB_RAR_FAPI_TRACE_PATH"
[ -n "$TX_WORKER_TRACE_PATH" ] && rm -f "$TX_WORKER_TRACE_PATH"

find "$ROOT/runs" -maxdepth 1 -type d -name '*_ota-stage' 2>/dev/null | sort > "$BEFORE"

cp -av "$ROOT/configs/gnb_m2sdr_band3_ota.template.yml" "$GNB_CONF" >/dev/null

python3 - <<'PY' "$GNB_CONF" "$TX_BACKOFF" "$GNB_RX_GAIN" "$M2SDR_CLOCK_SOURCE"
import os
from pathlib import Path
import re
import sys
import yaml

p = Path(sys.argv[1])
backoff = sys.argv[2]
rx_gain = sys.argv[3]
m2sdr_clock_source = sys.argv[4].strip()
s = p.read_text()
s = re.sub(r'(\btx_gain:\s*)[-+0-9.]+\b', r'\g<1>0', s)
s = re.sub(r'(\btx_gain_backoff:\s*)[-+0-9.]+\b', rf'\g<1>{backoff}', s)
s = re.sub(r'(\brx_gain:\s*)[-+0-9.]+\b', rf'\g<1>{rx_gain}', s)
s = re.sub(r'(\btx_mode:\s*)\S+\b', r'\g<1>continuous', s)


def set_m2sdr_clock_source(doc, clock_source):
    if not clock_source:
        return doc
    aliases = {
        "internal": "internal",
        "external": "external",
        "fpga": "fpga",
        "si5351c-fpga": "fpga",
        "si5351c_fpga": "fpga",
        "pll": "fpga",
    }
    normalized = aliases.get(clock_source)
    if normalized is None:
        allowed = ", ".join(sorted(aliases))
        raise SystemExit(f"invalid PAVONIS_M2SDR_CLOCK_SOURCE={clock_source!r}; expected one of {allowed}")

    def repl(match):
        prefix = match.group(1)
        quote = match.group(2) or ""
        args = match.group(3).strip()
        close_quote = quote if quote else (match.group(4) or "")
        suffix = match.group(5) or ""
        parts = [part.strip() for part in args.split(",") if part.strip()]
        parts = [part for part in parts if not part.startswith("clock_source=")]
        parts.append(f"clock_source={normalized}")
        return f"{prefix}{quote}{','.join(parts)}{close_quote}{suffix}"

    updated, count = re.subn(r'(?m)^(\s*device_args:\s*)(["\']?)([^"\'#\n]*)(["\']?)(.*)$', repl, doc, count=1)
    if count != 1:
        raise SystemExit("could not update ru_sdr.device_args for PAVONIS_M2SDR_CLOCK_SOURCE")
    return updated

s = set_m2sdr_clock_source(s, m2sdr_clock_source)


def set_cell_prach_key(doc, key, value):
    lines = doc.splitlines()
    out = []
    in_prach = False
    inserted = False
    for line in lines:
        if line == "  prach:":
            in_prach = True
            inserted = False
            out.append(line)
            continue
        if in_prach:
            if line.strip() and not line.startswith("    "):
                if not inserted:
                    out.append(f"    {key}: {value}")
                    inserted = True
                in_prach = False
            elif re.match(rf"\s*{re.escape(key)}:\s*", line):
                if not inserted:
                    out.append(f"    {key}: {value}")
                    inserted = True
                continue
        out.append(line)
    if in_prach and not inserted:
        out.append(f"    {key}: {value}")
    return "\n".join(out) + ("\n" if doc.endswith("\n") else "")

demarg_enabled = os.environ.get("PAVONIS_GNB_SRSUE_PDCCH_COMPAT", "0") == "1"
fixed_sib1_mcs = os.environ.get("PAVONIS_GNB_FIXED_SIB1_MCS", "")
max_ue_mcs = os.environ.get("PAVONIS_GNB_MAX_UE_MCS", "")
ssb_power = os.environ.get("PAVONIS_GNB_SSB_BLOCK_POWER_DBM", "")
power_control_offset_ss_db = os.environ.get("PAVONIS_GNB_POWER_CONTROL_OFFSET_SS_DB", "")
pusch_dec_max_iterations = os.environ.get("PAVONIS_GNB_PUSCH_DEC_MAX_ITERATIONS", "")
coreset0_index = os.environ.get("PAVONIS_GNB_CORESET0_INDEX", "12" if demarg_enabled else "")
ss0_index = os.environ.get("PAVONIS_GNB_SS0_INDEX", "0" if demarg_enabled else "")
max_coreset0_duration = os.environ.get("PAVONIS_GNB_MAX_CORESET0_DURATION", "")
ss1_n_candidates = os.environ.get("PAVONIS_GNB_SS1_N_CANDIDATES", "")
ra_resp_window = os.environ.get("PAVONIS_GNB_RA_RESP_WINDOW", "")
prach_frequency_start = os.environ.get("PAVONIS_GNB_PRACH_FREQUENCY_START", "")
prach_config_index = os.environ.get("PAVONIS_GNB_PRACH_CONFIG_INDEX", "")
total_nof_ra_preambles = os.environ.get("PAVONIS_GNB_TOTAL_NOF_RA_PREAMBLES", "")
preamble_rx_target_pw = os.environ.get("PAVONIS_GNB_PREAMBLE_RX_TARGET_PW", "")
q_qual_min = os.environ.get("PAVONIS_GNB_Q_QUAL_MIN", "")
clock_ppm = os.environ.get("PAVONIS_GNB_CLOCK_PPM", "")
gnb_plmn = os.environ.get("PAVONIS_GNB_PLMN", "")
gnb_id = os.environ.get("PAVONIS_GNB_ID", "")
gnb_id_bit_length = os.environ.get("PAVONIS_GNB_ID_BIT_LENGTH", "")
gnb_sector_id = os.environ.get("PAVONIS_GNB_SECTOR_ID", "")
gnb_pci = os.environ.get("PAVONIS_GNB_PCI", "")
gnb_tac = os.environ.get("PAVONIS_GNB_TAC", "")
srsenb_sib1_n0_20ms = os.environ.get("PAVONIS_GNB_SRSENB_SIB1_COMPAT", "0")
msg3_delta_preamble = os.environ.get("PAVONIS_GNB_MSG3_DELTA_PREAMBLE", "")
if srsenb_sib1_n0_20ms not in {"0", "1"}:
    raise SystemExit("PAVONIS_GNB_SRSENB_SIB1_COMPAT must be 0 or 1")
if msg3_delta_preamble and (
    not re.fullmatch(r"-?[0-9]+", msg3_delta_preamble) or not -1 <= int(msg3_delta_preamble) <= 6
):
    raise SystemExit("PAVONIS_GNB_MSG3_DELTA_PREAMBLE must be empty or -1 through 6")

if clock_ppm:
    try:
        clock_ppm_value = float(clock_ppm)
    except ValueError as exc:
        raise SystemExit("PAVONIS_GNB_CLOCK_PPM must be numeric") from exc
    if not (-100.0 <= clock_ppm_value <= 100.0):
        raise SystemExit("PAVONIS_GNB_CLOCK_PPM must be within +/-100 ppm")
    s, n = re.subn(
        r'(?m)^(  rx_gain:\s*[-+0-9.]+\s*)$',
        rf'\1\n  clock_ppm: {clock_ppm_value:.9g}',
        s,
        count=1,
    )
    if n != 1:
        raise SystemExit("could not insert ru_sdr.clock_ppm after rx_gain")

if ra_resp_window:
    if not re.fullmatch(r"[0-9]+", ra_resp_window):
        raise SystemExit(f"invalid PAVONIS_GNB_RA_RESP_WINDOW={ra_resp_window!r}")
    s = set_cell_prach_key(s, "ra_resp_window", ra_resp_window)

if prach_frequency_start:
    if not re.fullmatch(r"[0-9]+", prach_frequency_start) or not 0 <= int(prach_frequency_start) <= 274:
        raise SystemExit(
            f"invalid PAVONIS_GNB_PRACH_FREQUENCY_START={prach_frequency_start!r}; expected 0 through 274"
        )
    s = set_cell_prach_key(s, "prach_frequency_start", prach_frequency_start)

if prach_config_index:
    if not re.fullmatch(r"[0-9]+", prach_config_index) or not 0 <= int(prach_config_index) <= 255:
        raise SystemExit(
            f"invalid PAVONIS_GNB_PRACH_CONFIG_INDEX={prach_config_index!r}; expected 0 through 255"
        )
    s = set_cell_prach_key(s, "prach_config_index", prach_config_index)

if total_nof_ra_preambles:
    if not re.fullmatch(r"[0-9]+", total_nof_ra_preambles) or not 1 <= int(total_nof_ra_preambles) <= 64:
        raise SystemExit(
            "invalid PAVONIS_GNB_TOTAL_NOF_RA_PREAMBLES="
            f"{total_nof_ra_preambles!r}; expected 1 through 64"
        )
    s = set_cell_prach_key(s, "total_nof_ra_preambles", total_nof_ra_preambles)
    s = set_cell_prach_key(s, "nof_cb_preambles_per_ssb", total_nof_ra_preambles)

if preamble_rx_target_pw:
    if not re.fullmatch(r"-?[0-9]+", preamble_rx_target_pw):
        raise SystemExit(f"invalid PAVONIS_GNB_PREAMBLE_RX_TARGET_PW={preamble_rx_target_pw!r}")
    value = int(preamble_rx_target_pw)
    if not -202 <= value <= -60 or value % 2:
        raise SystemExit(
            "invalid PAVONIS_GNB_PREAMBLE_RX_TARGET_PW="
            f"{preamble_rx_target_pw!r}; expected an even integer from -202 through -60"
        )
    s = set_cell_prach_key(s, "preamble_rx_target_pw", preamble_rx_target_pw)

extra = []
if q_qual_min:
    if not re.fullmatch(r"-?[0-9]+", q_qual_min) or not -43 <= int(q_qual_min) <= -12:
        raise SystemExit(
            f"invalid PAVONIS_GNB_Q_QUAL_MIN={q_qual_min!r}; expected -43 through -12"
        )
    extra.append(f"  q_qual_min: {q_qual_min}\n")
if demarg_enabled or coreset0_index or ss0_index or max_coreset0_duration or ss1_n_candidates:
    common_lines = []
    if ss0_index:
        common_lines.append(f"      ss0_index: {ss0_index}\n")
    if coreset0_index:
        common_lines.append(f"      coreset0_index: {coreset0_index}\n")
    if max_coreset0_duration:
        common_lines.append(f"      max_coreset0_duration: {max_coreset0_duration}\n")
    if ss1_n_candidates:
        candidates = [part.strip() for part in ss1_n_candidates.split(",")]
        allowed_candidate_counts = {"0", "1", "2", "3", "4", "5", "6", "8"}
        if len(candidates) != 5 or any(part not in allowed_candidate_counts for part in candidates):
            raise SystemExit(
                f"invalid PAVONIS_GNB_SS1_N_CANDIDATES={ss1_n_candidates!r}; "
                "expected five comma-separated values from {0,1,2,3,4,5,6,8}"
            )
        common_lines.append(f"      ss1_n_candidates: [{', '.join(candidates)}]\n")
    pdcch_lines = ["  pdcch:\n"]
    if demarg_enabled:
        pdcch_lines.extend(
            [
                "    dedicated:\n",
                "      ss2_type: common\n",
                "      dci_format_0_1_and_1_1: false\n",
            ]
        )
    pdcch_lines.append("    common:\n")
    pdcch_lines.extend(common_lines)
    extra.append("".join(pdcch_lines))
pdsch_lines = []
if fixed_sib1_mcs:
    pdsch_lines.append(f"    fixed_sib1_mcs: {fixed_sib1_mcs}\n")
if max_ue_mcs:
    try:
        max_ue_mcs_value = int(max_ue_mcs)
    except ValueError as exc:
        raise SystemExit("PAVONIS_GNB_MAX_UE_MCS must be an integer") from exc
    if not 0 <= max_ue_mcs_value <= 28:
        raise SystemExit("PAVONIS_GNB_MAX_UE_MCS must be between 0 and 28")
    pdsch_lines.append(f"    max_ue_mcs: {max_ue_mcs_value}\n")
if pdsch_lines:
    extra.append("  pdsch:\n" + "".join(pdsch_lines))
if ssb_power:
    extra.append("  ssb:\n" f"    ssb_block_power_dbm: {ssb_power}\n")
if extra:
    s, n = re.subn(r'(?m)^  prach:\n', "".join(extra) + "  prach:\n", s, count=1)
    if n != 1:
        raise SystemExit("could not insert Pavonis gNB demarginalization YAML before cell_cfg.prach")

if pusch_dec_max_iterations:
    try:
        pusch_dec_max_iterations_value = int(pusch_dec_max_iterations)
    except ValueError as exc:
        raise SystemExit("PAVONIS_GNB_PUSCH_DEC_MAX_ITERATIONS must be an integer") from exc
    if pusch_dec_max_iterations_value <= 0:
        raise SystemExit("PAVONIS_GNB_PUSCH_DEC_MAX_ITERATIONS must be positive")
    s, n = re.subn(
        r'(?m)^cell_cfg:\n',
        "expert_phy:\n"
        f"  pusch_dec_max_iterations: {pusch_dec_max_iterations_value}\n\n"
        "cell_cfg:\n",
        s,
        count=1,
    )
    if n != 1:
        raise SystemExit("could not insert Pavonis expert_phy YAML before cell_cfg")

if gnb_plmn:
    if not re.fullmatch(r"[0-9]{5,6}", gnb_plmn):
        raise SystemExit("PAVONIS_GNB_PLMN must contain five or six digits")
    config = yaml.safe_load(s)
    try:
        config["cu_cp"]["amf"]["supported_tracking_areas"][0]["plmn_list"][0]["plmn"] = gnb_plmn
        config["cell_cfg"]["plmn"] = gnb_plmn
    except (KeyError, IndexError, TypeError) as exc:
        raise SystemExit("could not update both generated gNB PLMN fields") from exc
    s = yaml.safe_dump(config, sort_keys=False)

if srsenb_sib1_n0_20ms == "1":
    config = yaml.safe_load(s)
    cell = config["cell_cfg"]
    pdcch_common = cell.setdefault("pdcch", {}).setdefault("common", {})
    if not ss1_n_candidates:
        pdcch_common["ss1_n_candidates"] = [0, 0, 1, 0, 0]
    cell.setdefault("ul_common", {})["p_max"] = 10
    cell.setdefault("sib", {})["t311"] = 30000
    s = yaml.safe_dump(config, sort_keys=False)

if msg3_delta_preamble:
    config = yaml.safe_load(s)
    config["cell_cfg"].setdefault("pusch", {})["msg3_delta_preamble"] = int(msg3_delta_preamble)
    s = yaml.safe_dump(config, sort_keys=False)

if gnb_id or gnb_id_bit_length or gnb_sector_id or gnb_pci or gnb_tac:
    config = yaml.safe_load(s)
    if gnb_id:
        try:
            gnb_id_value = int(gnb_id)
        except ValueError as exc:
            raise SystemExit("PAVONIS_GNB_ID must be an integer") from exc
        if not 0 <= gnb_id_value <= 4294967295:
            raise SystemExit("PAVONIS_GNB_ID must be from 0 through 4294967295")
        config["gnb_id"] = gnb_id_value
    if gnb_id_bit_length:
        try:
            gnb_id_bit_length_value = int(gnb_id_bit_length)
        except ValueError as exc:
            raise SystemExit("PAVONIS_GNB_ID_BIT_LENGTH must be an integer") from exc
        if not 22 <= gnb_id_bit_length_value <= 32:
            raise SystemExit("PAVONIS_GNB_ID_BIT_LENGTH must be from 22 through 32")
        config["gnb_id_bit_length"] = gnb_id_bit_length_value
    if gnb_sector_id:
        try:
            gnb_sector_id_value = int(gnb_sector_id)
        except ValueError as exc:
            raise SystemExit("PAVONIS_GNB_SECTOR_ID must be an integer") from exc
        if gnb_id_bit_length:
            max_sector_id = 1 << (36 - int(gnb_id_bit_length))
            if not 0 <= gnb_sector_id_value < max_sector_id:
                raise SystemExit("PAVONIS_GNB_SECTOR_ID does not fit the remaining NCI bits")
        config["cell_cfg"]["sector_id"] = gnb_sector_id_value
    if gnb_pci:
        try:
            pci_value = int(gnb_pci)
        except ValueError as exc:
            raise SystemExit("PAVONIS_GNB_PCI must be an integer") from exc
        if not 0 <= pci_value <= 1007:
            raise SystemExit("PAVONIS_GNB_PCI must be from 0 through 1007")
        config["cell_cfg"]["pci"] = pci_value
    if gnb_tac:
        try:
            tac_value = int(gnb_tac)
        except ValueError as exc:
            raise SystemExit("PAVONIS_GNB_TAC must be an integer") from exc
        if not 0 <= tac_value <= 16777215:
            raise SystemExit("PAVONIS_GNB_TAC must be from 0 through 16777215")
        config["cell_cfg"]["tac"] = tac_value
        try:
            config["cu_cp"]["amf"]["supported_tracking_areas"][0]["tac"] = tac_value
        except (KeyError, IndexError, TypeError) as exc:
            raise SystemExit("could not update CU-CP supported tracking-area TAC") from exc
    s = yaml.safe_dump(config, sort_keys=False)

p.write_text(s)
print(f"patched_private_run_config={p}")
print(f"PAVONIS_M2SDR_CLOCK_SOURCE={m2sdr_clock_source}")
if m2sdr_clock_source:
    print("pavonis_m2sdr_clock_source_yaml=1")
print(f"PAVONIS_GNB_POWER_CONTROL_OFFSET_SS_DB={power_control_offset_ss_db}")
print(f"PAVONIS_GNB_PUSCH_DEC_MAX_ITERATIONS={pusch_dec_max_iterations}")
print(f"PAVONIS_GNB_RA_RESP_WINDOW={ra_resp_window}")
print(f"PAVONIS_GNB_PRACH_FREQUENCY_START={prach_frequency_start}")
print(f"PAVONIS_GNB_PRACH_CONFIG_INDEX={prach_config_index}")
print(f"PAVONIS_GNB_TOTAL_NOF_RA_PREAMBLES={total_nof_ra_preambles}")
print(f"PAVONIS_GNB_PREAMBLE_RX_TARGET_PW={preamble_rx_target_pw}")
print(f"PAVONIS_GNB_Q_QUAL_MIN={q_qual_min}")
print(f"PAVONIS_GNB_CLOCK_PPM={clock_ppm}")
print(f"PAVONIS_GNB_PLMN={gnb_plmn}")
print(f"PAVONIS_GNB_ID={gnb_id}")
print(f"PAVONIS_GNB_ID_BIT_LENGTH={gnb_id_bit_length}")
print(f"PAVONIS_GNB_SECTOR_ID={gnb_sector_id}")
print(f"PAVONIS_GNB_PCI={gnb_pci}")
print(f"PAVONIS_GNB_TAC={gnb_tac}")
print(f"PAVONIS_GNB_SRSENB_SIB1_COMPAT={srsenb_sib1_n0_20ms}")
print(f"PAVONIS_GNB_MSG3_DELTA_PREAMBLE={msg3_delta_preamble}")
if gnb_plmn:
    print("pavonis_gnb_plmn_yaml=1")
if ra_resp_window:
    print("pavonis_gnb_ra_resp_window_yaml=1")
if prach_frequency_start:
    print("pavonis_gnb_prach_frequency_start_yaml=1")
if prach_config_index:
    print("pavonis_gnb_prach_config_index_yaml=1")
if total_nof_ra_preambles:
    print("pavonis_gnb_total_nof_ra_preambles_yaml=1")
if preamble_rx_target_pw:
    print("pavonis_gnb_preamble_rx_target_pw_yaml=1")
if q_qual_min:
    print("pavonis_gnb_q_qual_min_yaml=1")
if extra:
    print("pavonis_gnb_demarginalization_yaml=1")
    print(f"PAVONIS_GNB_SRSUE_PDCCH_COMPAT={int(demarg_enabled)}")
    print(f"PAVONIS_GNB_CORESET0_INDEX={coreset0_index}")
    print(f"PAVONIS_GNB_SS0_INDEX={ss0_index}")
    print(f"PAVONIS_GNB_MAX_CORESET0_DURATION={max_coreset0_duration}")
    print(f"PAVONIS_GNB_SS1_N_CANDIDATES={ss1_n_candidates}")
    print(f"PAVONIS_GNB_FIXED_SIB1_MCS={fixed_sib1_mcs}")
    print(f"PAVONIS_GNB_MAX_UE_MCS={max_ue_mcs}")
    print(f"PAVONIS_GNB_SSB_BLOCK_POWER_DBM={ssb_power}")
PY
CONFIG_PATCH_RC=$?
if [ "$CONFIG_PATCH_RC" -ne 0 ]; then
  echo "ERROR: gNB config patch failed (rc=$CONFIG_PATCH_RC)" >&2
  exit "$CONFIG_PATCH_RC"
fi

cat > "$RUNNER" <<EOR
#!/usr/bin/env bash
set +e +u
set +o pipefail 2>/dev/null || true

cd "$ROOT" || exit 1

export PAVONIS_SOAPY_TX_READBACK_PATH="$TX_READBACK"
export OCUDU_GNB_BIN="$OCUDU_GNB_BIN_OVERRIDE"
export PAVONIS_GNB_MIB_INTRAFREQ_RESELECTION_ALLOWED="$GNB_MIB_INTRAFREQ_RESELECTION_ALLOWED"
export PAVONIS_GNB_SIB1_RETX_PERIOD_MS="$GNB_SIB1_RETX_PERIOD_MS"
export PAVONIS_GNB_SIB1_USE_N0="$GNB_SIB1_USE_N0"
export PAVONIS_GNB_SRSENB_SIB1_COMPAT="$GNB_SRSENB_SIB1_COMPAT"
export PAVONIS_GNB_MSG3_DELTA_PREAMBLE="$GNB_MSG3_DELTA_PREAMBLE"
export PAVONIS_SOAPY_PHYSICAL_CHANNEL_OFFSET="$PHYSICAL_CHANNEL_OFFSET"
export PAVONIS_SOAPY_RX_FORCE_CENTER_HZ="$PAVONIS_SOAPY_RX_FORCE_CENTER_HZ"
export OCUDU_SOAPY_RX_BW_HZ="$OCUDU_SOAPY_RX_BW_HZ"
export PAVONIS_SOAPY_TX_FORCE_CONTINUOUS_STREAM="$FORCE_CONTINUOUS_STREAM"
export PAVONIS_SOAPY_ALLOW_M2SDR_CONTINUOUS_TX_MODE="$ALLOW_CONTINUOUS_MODE"
export PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS="$ALL_TIMED_CHUNKS"
export PAVONIS_SOAPY_TX_REANCHOR_INTERVAL_SAMPLES="$REANCHOR_INTERVAL_SAMPLES"
export OCUDU_SOAPY_TX_TIMEOUT_RETRY="\${PAVONIS_SOAPY_TX_TIMEOUT_RETRY_OVERRIDE:-0}"
export OCUDU_SOAPY_TX_TIMEOUT_RETRY_MAX="\${PAVONIS_SOAPY_TX_TIMEOUT_RETRY_MAX_OVERRIDE:-0}"
export OCUDU_SOAPY_TX_TIMEOUT_RETRY_US="\${PAVONIS_SOAPY_TX_TIMEOUT_RETRY_US_OVERRIDE:-1000}"
export OCUDU_SOAPY_TX_DEADLINE_WRITE_TIMEOUT="$DEADLINE_WRITE_OVERRIDE"
export OCUDU_SOAPY_TX_DEADLINE_WRITE_GUARD_US="$DEADLINE_WRITE_GUARD_US_OVERRIDE"
export OCUDU_SOAPY_TX_DEADLINE_WRITE_MAX_TIMEOUT_US="$DEADLINE_WRITE_MAX_TIMEOUT_US_OVERRIDE"
export OCUDU_SOAPY_TX_DEADLINE_WRITE_CARRY_ON_TIMEOUT=0
export OCUDU_SOAPY_TX_DEADLINE_WRITE_DIRECT_POLL_US=0
export PAVONIS_SOAPY_TX_WRITE_DUMP_CF32_PATH="$TX_WRITE_DUMP_CF32_PATH"
export PAVONIS_SOAPY_TX_WRITE_DUMP_META_PATH="$TX_WRITE_DUMP_META_PATH"
export PAVONIS_SOAPY_TX_WRITE_DUMP_MAX_SAMPLES="$TX_WRITE_DUMP_MAX_SAMPLES"
export PAVONIS_SOAPY_TX_WRITE_DUMP_PORT="$TX_WRITE_DUMP_PORT"
export M2SDR_SOAPY_TX_DUMP_PATH="$TX_PLUGIN_DUMP_PATH"
export M2SDR_SOAPY_TX_DUMP_SAMPLES="$TX_PLUGIN_DUMP_SAMPLES"
export M2SDR_SOAPY_TX_DMA_DUMP_PATH="$TX_DMA_DUMP_PATH"
export M2SDR_SOAPY_TX_DMA_DUMP_BUFFERS="$TX_DMA_DUMP_BUFFERS"
export M2SDR_SOAPY_TX_PROBE_ARM_FILE="$TX_PROBE_ARM_FILE"
export M2SDR_SOAPY_TX_PROBE_ARM_POLL_PERIOD="$TX_PROBE_ARM_POLL_PERIOD"
export PAVONIS_GNB_POWER_CONTROL_OFFSET_SS_DB="\${PAVONIS_GNB_POWER_CONTROL_OFFSET_SS_DB:-}"
export PAVONIS_GNB_CLOCK_PPM="$GNB_CLOCK_PPM"
export PAVONIS_GNB_PLMN="$GNB_PLMN"
export PAVONIS_GNB_PUSCH_DEC_MAX_ITERATIONS="\${PAVONIS_GNB_PUSCH_DEC_MAX_ITERATIONS:-}"
export PAVONIS_GNB_SI_OCCASION_TRACE="$GNB_SI_OCCASION_TRACE"
export PAVONIS_GNB_SI_OCCASION_TRACE_PATH="$GNB_SI_OCCASION_TRACE_PATH"
export PAVONIS_GNB_SI_OCCASION_TRACE_LIMIT="$GNB_SI_OCCASION_TRACE_LIMIT"
export PAVONIS_GNB_RAR_SCHED_TRACE="$GNB_RAR_SCHED_TRACE"
export PAVONIS_GNB_RAR_SCHED_TRACE_LIMIT="$GNB_RAR_SCHED_TRACE_LIMIT"
export M2SDR_SOAPY_TX_WORKER_TRACE_PATH="$TX_WORKER_TRACE_PATH"
export M2SDR_SOAPY_TX_WORKER_TRACE_LIMIT="$TX_WORKER_TRACE_LIMIT"
export M2SDR_SOAPY_TX_WORKER_TRACE_FLUSH_PERIOD="$TX_WORKER_TRACE_FLUSH_PERIOD"
export M2SDR_SOAPY_TX_WORKER_PKT_COUNT="$TX_WORKER_PKT_COUNT"
export M2SDR_LITEPCIE_ZERO_COPY="$LITEPCIE_ZERO_COPY"
export M2SDR_SOAPY_TX_COPY_PRIME_BUFFERS="$TX_COPY_PRIME_BUFFERS"
export M2SDR_SOAPY_TX_ZERO_COPY_PRIME_BUFFERS="$TX_ZERO_COPY_PRIME_BUFFERS"
export PAVONIS_ALLOW_EXTENDED_RA_RESP_WINDOW="\${PAVONIS_ALLOW_EXTENDED_RA_RESP_WINDOW:-}"
export PAVONIS_RACH_RAR_WINDOW_OFFSET_SLOTS="$RAR_WINDOW_OFFSET_SLOTS"
export M2SDR_SOAPY_TX_ALIGN_HAS_TIME_REMAINDER="$ALIGN_HAS_TIME_REMAINDER"
export PAVONIS_PRACHDET_TRACE=1
export PAVONIS_PRACHDET_TRACE_ARM_FILE="$ARM"
export PAVONIS_PRACH_PREDEMOD_DUMP=1
export PAVONIS_PRACH_DUMP_ARM_FILE="$ARM"
export PAVONIS_PRACH_PREDEMOD_SUMMARY_MAX="\${PAVONIS_PRACH_PREDEMOD_SUMMARY_MAX:-1200}"
export PAVONIS_PRACH_PREDEMOD_RAW_MAX="\${PAVONIS_PRACH_PREDEMOD_RAW_MAX:-512}"
export PAVONIS_PRACH_STRONG_PREDEMOD_CAPTURE_STAGE1_ENERGY="$PRACH_STRONG_CAPTURE_STAGE1_ENERGY"
export PAVONIS_PRACH_DEMOD_INPUT_STAGE1_CANDIDATE_LIST="$PRACH_STAGE1_CANDIDATE_LIST"
export PAVONIS_PRACH_DEMOD_INPUT_STAGE1_MAX_CANDIDATES="$PRACH_STAGE1_MAX_CANDIDATES"
export PAVONIS_PRACH_COLLECTION_EXTRA_SAMPLES="\${PAVONIS_PRACH_COLLECTION_EXTRA_SAMPLES:-0}"
export PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE="\${PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE:-0}"
export PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE_ARM_MODE="\${PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE_ARM_MODE:-pre_ue}"
export PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE_ARM_FILE="\${PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE_ARM_FILE:-$ARM}"
export PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE_MAX="\${PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE_MAX:-4096}"
export PAVONIS_PRACH_LATE_CONTEXT_TRACE="\${PAVONIS_PRACH_LATE_CONTEXT_TRACE:-0}"
export PAVONIS_PRACH_LATE_CONTEXT_TRACE_PATH="\${PAVONIS_PRACH_LATE_CONTEXT_TRACE_PATH:-$PRACH_LATE_CONTEXT_TRACE_PATH}"
export PAVONIS_PRACH_LATE_CONTEXT_TRACE_LIMIT="\${PAVONIS_PRACH_LATE_CONTEXT_TRACE_LIMIT:-64}"
export PAVONIS_UL_SYMBOL_DUMP="\${PAVONIS_UL_SYMBOL_DUMP:-1}"
export PAVONIS_UL_SYMBOL_DUMP_ARM_MODE="\${PAVONIS_UL_SYMBOL_DUMP_ARM_MODE:-pre_ue}"
export PAVONIS_UL_SYMBOL_DUMP_ARM_FILE="\${PAVONIS_UL_SYMBOL_DUMP_ARM_FILE:-$UL_SYMBOL_ARM}"
export PAVONIS_UL_SYMBOL_SUMMARY_MAX="\${PAVONIS_UL_SYMBOL_SUMMARY_MAX:-260000}"
export PAVONIS_UL_SYMBOL_METADATA_ONLY="\${PAVONIS_UL_SYMBOL_METADATA_ONLY:-1}"
export PAVONIS_UL_SYMBOL_RAW_MAX="\${PAVONIS_UL_SYMBOL_RAW_MAX:-0}"
export PAVONIS_UL_SYMBOL_RAW_PEAK_MIN="\${PAVONIS_UL_SYMBOL_RAW_PEAK_MIN:-0.006}"
export PAVONIS_UL_SYMBOL_RAW_RMS_MIN="\${PAVONIS_UL_SYMBOL_RAW_RMS_MIN:-0.0015}"
export PAVONIS_UL_SYMBOL_RAW_TARGET_SFN_MOD="\${PAVONIS_UL_SYMBOL_RAW_TARGET_SFN_MOD:-0}"
export PAVONIS_UL_SYMBOL_RAW_TARGET_SFN_REM="\${PAVONIS_UL_SYMBOL_RAW_TARGET_SFN_REM:--1}"
export PAVONIS_UL_SYMBOL_RAW_TARGET_SLOT="\${PAVONIS_UL_SYMBOL_RAW_TARGET_SLOT:--1}"
export PAVONIS_UL_SYMBOL_ARM_POLL_PERIOD="\${PAVONIS_UL_SYMBOL_ARM_POLL_PERIOD:-1024}"

if [ "\${PAVONIS_SOAPY_RX_APPEND_ENABLE:-0}" = "1" ]; then
  export PAVONIS_SOAPY_RX_DUMP=1
  export PAVONIS_SOAPY_RX_DUMP_ARM_FILE="$RX_APPEND_ARM"
  export PAVONIS_SOAPY_RX_ARM_POLL_PERIOD="\${PAVONIS_SOAPY_RX_ARM_POLL_PERIOD:-16}"
  export PAVONIS_SOAPY_RX_SUMMARY_MAX="\${PAVONIS_SOAPY_RX_SUMMARY_MAX:-600000}"
  export PAVONIS_SOAPY_RX_APPEND_CF32_PATH="\${PAVONIS_SOAPY_RX_APPEND_CF32_PATH:-/tmp/pavonis_soapy_rx_append_p0.cf32}"
  export PAVONIS_SOAPY_RX_APPEND_META_PATH="\${PAVONIS_SOAPY_RX_APPEND_META_PATH:-/tmp/pavonis_soapy_rx_append_meta.tsv}"
  export PAVONIS_SOAPY_RX_APPEND_MAX_SAMPLES="\${PAVONIS_SOAPY_RX_APPEND_MAX_SAMPLES:-230400000}"
  export PAVONIS_SOAPY_RX_APPEND_PORT="\${PAVONIS_SOAPY_RX_APPEND_PORT:-0}"
  export PAVONIS_SOAPY_RX_READBACK_PATH="\${PAVONIS_SOAPY_RX_READBACK_PATH:-/tmp/pavonis_soapy_rx_readback.txt}"
  export OCUDU_SOAPY_RX_USE_READSTREAM="\${OCUDU_SOAPY_RX_USE_READSTREAM:-1}"
fi

echo "===== detached OTA harness ====="
hostname
date -u --iso-8601=seconds
echo "TAG=$TAG"
echo "STAMP=$STAMP"
echo "GNB_CONF=$GNB_CONF"
echo "SIM_FILE=$SIM_FILE"
echo "UE_CONF=$UE_CONF"
echo "DURATION=$DURATION"
echo "IFACE=$IFACE"
echo "ARM=$ARM"
echo "RX_APPEND_ARM=$RX_APPEND_ARM"
echo "UL_SYMBOL_ARM=$UL_SYMBOL_ARM"
echo "PAVONIS_SOAPY_TX_READBACK_PATH=\$PAVONIS_SOAPY_TX_READBACK_PATH"
echo "OCUDU_GNB_BIN=\$OCUDU_GNB_BIN"
echo "PAVONIS_GNB_MIB_INTRAFREQ_RESELECTION_ALLOWED=\$PAVONIS_GNB_MIB_INTRAFREQ_RESELECTION_ALLOWED"
echo "PAVONIS_GNB_SIB1_RETX_PERIOD_MS=\$PAVONIS_GNB_SIB1_RETX_PERIOD_MS"
echo "PAVONIS_GNB_SIB1_USE_N0=\$PAVONIS_GNB_SIB1_USE_N0"
echo "PAVONIS_GNB_SRSENB_SIB1_COMPAT=\$PAVONIS_GNB_SRSENB_SIB1_COMPAT"
echo "PAVONIS_GNB_MSG3_DELTA_PREAMBLE=\$PAVONIS_GNB_MSG3_DELTA_PREAMBLE"
echo "PAVONIS_SOAPY_PHYSICAL_CHANNEL_OFFSET=\$PAVONIS_SOAPY_PHYSICAL_CHANNEL_OFFSET"
echo "PAVONIS_SOAPY_TX_FORCE_CONTINUOUS_STREAM=\$PAVONIS_SOAPY_TX_FORCE_CONTINUOUS_STREAM"
echo "PAVONIS_SOAPY_ALLOW_M2SDR_CONTINUOUS_TX_MODE=\$PAVONIS_SOAPY_ALLOW_M2SDR_CONTINUOUS_TX_MODE"
echo "PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS=\$PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS"
echo "PAVONIS_SOAPY_TX_REANCHOR_INTERVAL_SAMPLES=\$PAVONIS_SOAPY_TX_REANCHOR_INTERVAL_SAMPLES"
echo "M2SDR_SOAPY_TX_ALIGN_HAS_TIME_REMAINDER=\$M2SDR_SOAPY_TX_ALIGN_HAS_TIME_REMAINDER"
echo "PAVONIS_GNB_SI_OCCASION_TRACE=\$PAVONIS_GNB_SI_OCCASION_TRACE"
echo "PAVONIS_GNB_SI_OCCASION_TRACE_PATH=\$PAVONIS_GNB_SI_OCCASION_TRACE_PATH"
echo "PAVONIS_GNB_SI_OCCASION_TRACE_LIMIT=\$PAVONIS_GNB_SI_OCCASION_TRACE_LIMIT"
echo "PAVONIS_GNB_RAR_SCHED_TRACE=\$PAVONIS_GNB_RAR_SCHED_TRACE"
echo "PAVONIS_GNB_RAR_SCHED_TRACE_LIMIT=\$PAVONIS_GNB_RAR_SCHED_TRACE_LIMIT"
echo "M2SDR_SOAPY_TX_WORKER_TRACE_PATH=\$M2SDR_SOAPY_TX_WORKER_TRACE_PATH"
echo "M2SDR_SOAPY_TX_WORKER_TRACE_LIMIT=\$M2SDR_SOAPY_TX_WORKER_TRACE_LIMIT"
echo "M2SDR_SOAPY_TX_WORKER_TRACE_FLUSH_PERIOD=\$M2SDR_SOAPY_TX_WORKER_TRACE_FLUSH_PERIOD"
echo "M2SDR_LITEPCIE_ZERO_COPY=\$M2SDR_LITEPCIE_ZERO_COPY"
echo "M2SDR_SOAPY_TX_COPY_PRIME_BUFFERS=\$M2SDR_SOAPY_TX_COPY_PRIME_BUFFERS"
echo "M2SDR_SOAPY_TX_ZERO_COPY_PRIME_BUFFERS=\$M2SDR_SOAPY_TX_ZERO_COPY_PRIME_BUFFERS"
echo "M2SDR_SOAPY_TX_PROBE_ARM_FILE=\$M2SDR_SOAPY_TX_PROBE_ARM_FILE"
echo "M2SDR_SOAPY_TX_PROBE_ARM_POLL_PERIOD=\$M2SDR_SOAPY_TX_PROBE_ARM_POLL_PERIOD"
echo "PAVONIS_GNB_SRSUE_PDCCH_COMPAT=\${PAVONIS_GNB_SRSUE_PDCCH_COMPAT:-}"
echo "PAVONIS_GNB_CORESET0_INDEX=\${PAVONIS_GNB_CORESET0_INDEX:-}"
echo "PAVONIS_GNB_SS0_INDEX=\${PAVONIS_GNB_SS0_INDEX:-}"
echo "PAVONIS_GNB_MAX_CORESET0_DURATION=\${PAVONIS_GNB_MAX_CORESET0_DURATION:-}"
echo "PAVONIS_GNB_FIXED_SIB1_MCS=\${PAVONIS_GNB_FIXED_SIB1_MCS:-}"
echo "PAVONIS_GNB_MAX_UE_MCS=\${PAVONIS_GNB_MAX_UE_MCS:-}"
echo "PAVONIS_GNB_MAX_MSG4_MCS=\${PAVONIS_GNB_MAX_MSG4_MCS:-}"
echo "PAVONIS_GNB_MAX_CONRES_MCS=\${PAVONIS_GNB_MAX_CONRES_MCS:-}"
echo "PAVONIS_GNB_FALLBACK_DL_DCI_AGGR_LEV=\${PAVONIS_GNB_FALLBACK_DL_DCI_AGGR_LEV:-}"
echo "PAVONIS_GNB_SS1_N_CANDIDATES=\${PAVONIS_GNB_SS1_N_CANDIDATES:-}"
echo "PAVONIS_QCORE_SIM_FILE=$SIM_FILE"
echo "PAVONIS_GNB_SSB_BLOCK_POWER_DBM=\${PAVONIS_GNB_SSB_BLOCK_POWER_DBM:-}"
echo "PAVONIS_GNB_SIB1_DCI_AGGR_LEV=\${PAVONIS_GNB_SIB1_DCI_AGGR_LEV:-}"
echo "PAVONIS_GNB_POWER_CONTROL_OFFSET_SS_DB=\${PAVONIS_GNB_POWER_CONTROL_OFFSET_SS_DB:-}"
echo "PAVONIS_GNB_CLOCK_PPM=\$PAVONIS_GNB_CLOCK_PPM"
echo "PAVONIS_GNB_PLMN=\$PAVONIS_GNB_PLMN"
echo "PAVONIS_OTA_STAGE_TEMPLATE=$OTA_STAGE_TEMPLATE"
echo "PAVONIS_GNB_PUSCH_DEC_MAX_ITERATIONS=\${PAVONIS_GNB_PUSCH_DEC_MAX_ITERATIONS:-}"
echo "PAVONIS_GNB_RA_RESP_WINDOW=${GNB_RA_RESP_WINDOW:-}"
echo "PAVONIS_ALLOW_EXTENDED_RA_RESP_WINDOW=\${PAVONIS_ALLOW_EXTENDED_RA_RESP_WINDOW:-}"
echo "PAVONIS_UL_SYMBOL_DUMP=\${PAVONIS_UL_SYMBOL_DUMP:-}"
echo "PAVONIS_UL_SYMBOL_DUMP_ARM_MODE=\${PAVONIS_UL_SYMBOL_DUMP_ARM_MODE:-}"
echo "PAVONIS_UL_SYMBOL_DUMP_ARM_FILE=\${PAVONIS_UL_SYMBOL_DUMP_ARM_FILE:-}"
echo "PAVONIS_UL_SYMBOL_SUMMARY_MAX=\${PAVONIS_UL_SYMBOL_SUMMARY_MAX:-}"
echo "PAVONIS_UL_SYMBOL_METADATA_ONLY=\${PAVONIS_UL_SYMBOL_METADATA_ONLY:-}"
echo "PAVONIS_UL_SYMBOL_RAW_MAX=\${PAVONIS_UL_SYMBOL_RAW_MAX:-}"
echo "PAVONIS_UL_SYMBOL_RAW_PEAK_MIN=\${PAVONIS_UL_SYMBOL_RAW_PEAK_MIN:-}"
echo "PAVONIS_UL_SYMBOL_RAW_RMS_MIN=\${PAVONIS_UL_SYMBOL_RAW_RMS_MIN:-}"
echo "PAVONIS_UL_SYMBOL_RAW_TARGET_SFN_MOD=\${PAVONIS_UL_SYMBOL_RAW_TARGET_SFN_MOD:-}"
echo "PAVONIS_UL_SYMBOL_RAW_TARGET_SFN_REM=\${PAVONIS_UL_SYMBOL_RAW_TARGET_SFN_REM:-}"
echo "PAVONIS_UL_SYMBOL_RAW_TARGET_SLOT=\${PAVONIS_UL_SYMBOL_RAW_TARGET_SLOT:-}"
echo "PAVONIS_PUCCH_F1_GRID_CAPTURE=\${PAVONIS_PUCCH_F1_GRID_CAPTURE:-0}"
echo "PAVONIS_PUCCH_F1_GRID_CAPTURE_MAX=\${PAVONIS_PUCCH_F1_GRID_CAPTURE_MAX:-4}"
echo "PAVONIS_PUCCH_F1_GRID_CAPTURE_MIN_EPRE_DB=\${PAVONIS_PUCCH_F1_GRID_CAPTURE_MIN_EPRE_DB:--60}"
echo "PAVONIS_PUCCH_F1_SYMBOL_PHASE_STEP_DEG=\${PAVONIS_PUCCH_F1_SYMBOL_PHASE_STEP_DEG:-0}"
echo "PAVONIS_PUCCH_F1_SUBCARRIER_PHASE_STEP_DEG=\${PAVONIS_PUCCH_F1_SUBCARRIER_PHASE_STEP_DEG:-0}"
echo "PAVONIS_PUCCH_F1_PHASE_SEARCH_CANDIDATES=\${PAVONIS_PUCCH_F1_PHASE_SEARCH_CANDIDATES:-}"
echo "PAVONIS_PUCCH_F1_PHASE_SEARCH_TRACE_LIMIT=\${PAVONIS_PUCCH_F1_PHASE_SEARCH_TRACE_LIMIT:-64}"
echo "PAVONIS_RACH_RAR_WINDOW_OFFSET_SLOTS=\$PAVONIS_RACH_RAR_WINDOW_OFFSET_SLOTS"
echo "PAVONIS_PRACHDET_TRACE_ARM_FILE=\$PAVONIS_PRACHDET_TRACE_ARM_FILE"
echo "PAVONIS_PRACH_DUMP_ARM_FILE=\$PAVONIS_PRACH_DUMP_ARM_FILE"
echo "PAVONIS_PRACH_STRONG_PREDEMOD_CAPTURE_STAGE1_ENERGY=\$PAVONIS_PRACH_STRONG_PREDEMOD_CAPTURE_STAGE1_ENERGY"
echo "PAVONIS_PRACH_DEMOD_INPUT_STAGE1_CANDIDATE_LIST=\$PAVONIS_PRACH_DEMOD_INPUT_STAGE1_CANDIDATE_LIST"
echo "PAVONIS_PRACH_DEMOD_INPUT_STAGE1_MAX_CANDIDATES=\$PAVONIS_PRACH_DEMOD_INPUT_STAGE1_MAX_CANDIDATES"
echo "PAVONIS_PRACH_COLLECTION_EXTRA_SAMPLES=\$PAVONIS_PRACH_COLLECTION_EXTRA_SAMPLES"
echo "PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE=\$PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE"
echo "PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE_ARM_MODE=\$PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE_ARM_MODE"
echo "PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE_ARM_FILE=\$PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE_ARM_FILE"
echo "PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE_MAX=\$PAVONIS_PRACH_COLLECTION_SEGMENT_TRACE_MAX"
echo "PAVONIS_PRACH_LATE_CONTEXT_TRACE=\$PAVONIS_PRACH_LATE_CONTEXT_TRACE"
echo "PAVONIS_PRACH_LATE_CONTEXT_TRACE_PATH=\$PAVONIS_PRACH_LATE_CONTEXT_TRACE_PATH"
echo "PAVONIS_PRACH_LATE_CONTEXT_TRACE_LIMIT=\$PAVONIS_PRACH_LATE_CONTEXT_TRACE_LIMIT"
echo "PAVONIS_UL_SYMBOL_DUMP_ARM_FILE=\$PAVONIS_UL_SYMBOL_DUMP_ARM_FILE"
echo "PAVONIS_SOAPY_RX_APPEND_ENABLE=\${PAVONIS_SOAPY_RX_APPEND_ENABLE:-0}"
echo "PAVONIS_SOAPY_RX_APPEND_IMMEDIATE=$PAVONIS_SOAPY_RX_APPEND_IMMEDIATE"
echo "PAVONIS_SOAPY_RX_DUMP=\${PAVONIS_SOAPY_RX_DUMP:-}"
echo "PAVONIS_SOAPY_RX_DUMP_ARM_FILE=\${PAVONIS_SOAPY_RX_DUMP_ARM_FILE:-}"
echo "PAVONIS_SOAPY_RX_ARM_POLL_PERIOD=\${PAVONIS_SOAPY_RX_ARM_POLL_PERIOD:-}"
echo "PAVONIS_SOAPY_RX_SUMMARY_MAX=\${PAVONIS_SOAPY_RX_SUMMARY_MAX:-}"
echo "PAVONIS_SOAPY_RX_APPEND_CF32_PATH=\${PAVONIS_SOAPY_RX_APPEND_CF32_PATH:-}"
echo "PAVONIS_SOAPY_RX_APPEND_META_PATH=\${PAVONIS_SOAPY_RX_APPEND_META_PATH:-}"
echo "PAVONIS_SOAPY_RX_APPEND_MAX_SAMPLES=\${PAVONIS_SOAPY_RX_APPEND_MAX_SAMPLES:-}"
echo "PAVONIS_SOAPY_RX_APPEND_PORT=\${PAVONIS_SOAPY_RX_APPEND_PORT:-}"
echo "PAVONIS_SOAPY_RX_READBACK_PATH=\${PAVONIS_SOAPY_RX_READBACK_PATH:-}"
echo "OCUDU_SOAPY_RX_USE_READSTREAM=\${OCUDU_SOAPY_RX_USE_READSTREAM:-}"

"$OTA_STAGE_TEMPLATE" \
  --i-understand-rf-test \
  --gnb-conf "$GNB_CONF" \
  --sim-file "$SIM_FILE" \
  --ue-conf "$UE_CONF" \
  --duration "$DURATION" \
  --external-iface "$IFACE"

RC=\$?
echo "run_ota_stage_template_rc=\$RC"
date -u --iso-8601=seconds
exit "\$RC"
EOR

chmod +x "$RUNNER"

{
  echo "===== detached marked OCUDU gNB Band 3 native-continuous Stage 6 window ====="
  hostname
  date -u --iso-8601=ns
  echo "TAG=$TAG"
  echo "STAMP=$STAMP"
  echo "OUT=$OUT"
  echo "DURATION=$DURATION"
  echo "TX_BACKOFF=$TX_BACKOFF"
  echo "TX_ATT=$TX_ATT"
  echo "GNB_RX_GAIN=$GNB_RX_GAIN"
  echo "PAVONIS_M2SDR_CLOCK_SOURCE=$M2SDR_CLOCK_SOURCE"
  echo "PAVONIS_GNB_CLOCK_PPM=$GNB_CLOCK_PPM"
  echo "PAVONIS_GNB_PLMN=$GNB_PLMN"
  echo "PAVONIS_OTA_STAGE_TEMPLATE=$OTA_STAGE_TEMPLATE"
  echo "ARM=$ARM"
  echo "TX_READBACK=$TX_READBACK"
  echo "PAVONIS_SOAPY_TX_FORCE_CONTINUOUS_STREAM=$FORCE_CONTINUOUS_STREAM"
  echo "PAVONIS_SOAPY_ALLOW_M2SDR_CONTINUOUS_TX_MODE=$ALLOW_CONTINUOUS_MODE"
  echo "PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS=$ALL_TIMED_CHUNKS"
  echo "PAVONIS_SOAPY_TX_REANCHOR_INTERVAL_SAMPLES=$REANCHOR_INTERVAL_SAMPLES"
  echo "M2SDR_SOAPY_TX_ALIGN_HAS_TIME_REMAINDER=$ALIGN_HAS_TIME_REMAINDER"
  echo "PAVONIS_GNB_SI_OCCASION_TRACE=$GNB_SI_OCCASION_TRACE"
  echo "PAVONIS_GNB_SI_OCCASION_TRACE_PATH=$GNB_SI_OCCASION_TRACE_PATH"
  echo "PAVONIS_GNB_SI_OCCASION_TRACE_LIMIT=$GNB_SI_OCCASION_TRACE_LIMIT"
  echo "PAVONIS_GNB_RAR_SCHED_TRACE=$GNB_RAR_SCHED_TRACE"
  echo "PAVONIS_GNB_RAR_SCHED_TRACE_LIMIT=$GNB_RAR_SCHED_TRACE_LIMIT"
  echo "M2SDR_SOAPY_TX_WORKER_TRACE_PATH=$TX_WORKER_TRACE_PATH"
  echo "M2SDR_SOAPY_TX_WORKER_TRACE_LIMIT=$TX_WORKER_TRACE_LIMIT"
  echo "M2SDR_SOAPY_TX_WORKER_TRACE_FLUSH_PERIOD=$TX_WORKER_TRACE_FLUSH_PERIOD"
  echo "M2SDR_SOAPY_TX_PROBE_ARM_FILE=$TX_PROBE_ARM_FILE"
  echo "M2SDR_SOAPY_TX_PROBE_ARM_POLL_PERIOD=$TX_PROBE_ARM_POLL_PERIOD"
  echo "PAVONIS_SOAPY_RX_APPEND_ENABLE=${PAVONIS_SOAPY_RX_APPEND_ENABLE:-0}"
  echo "PAVONIS_SOAPY_RX_APPEND_IMMEDIATE=$PAVONIS_SOAPY_RX_APPEND_IMMEDIATE"
  echo "PAVONIS_M2SDR_CLOCK_SOURCE=$M2SDR_CLOCK_SOURCE"
  echo "PAVONIS_GNB_SRSUE_PDCCH_COMPAT=${PAVONIS_GNB_SRSUE_PDCCH_COMPAT:-}"
  echo "PAVONIS_GNB_CORESET0_INDEX=${PAVONIS_GNB_CORESET0_INDEX:-}"
  echo "PAVONIS_GNB_SS0_INDEX=${PAVONIS_GNB_SS0_INDEX:-}"
  echo "PAVONIS_GNB_MAX_CORESET0_DURATION=${PAVONIS_GNB_MAX_CORESET0_DURATION:-}"
  echo "PAVONIS_GNB_FIXED_SIB1_MCS=${PAVONIS_GNB_FIXED_SIB1_MCS:-}"
  echo "PAVONIS_GNB_MAX_UE_MCS=${PAVONIS_GNB_MAX_UE_MCS:-}"
  echo "PAVONIS_GNB_MAX_MSG4_MCS=${PAVONIS_GNB_MAX_MSG4_MCS:-}"
  echo "PAVONIS_GNB_MAX_CONRES_MCS=${PAVONIS_GNB_MAX_CONRES_MCS:-}"
  echo "PAVONIS_GNB_FALLBACK_DL_DCI_AGGR_LEV=${PAVONIS_GNB_FALLBACK_DL_DCI_AGGR_LEV:-}"
  echo "PAVONIS_GNB_SS1_N_CANDIDATES=${PAVONIS_GNB_SS1_N_CANDIDATES:-}"
  echo "PAVONIS_QCORE_SIM_FILE=$SIM_FILE"
  echo "PAVONIS_GNB_SSB_BLOCK_POWER_DBM=${PAVONIS_GNB_SSB_BLOCK_POWER_DBM:-}"
  echo "PAVONIS_GNB_Q_QUAL_MIN=${PAVONIS_GNB_Q_QUAL_MIN:-}"
  echo "PAVONIS_GNB_SIB1_DCI_AGGR_LEV=${PAVONIS_GNB_SIB1_DCI_AGGR_LEV:-}"
  echo "PAVONIS_GNB_POWER_CONTROL_OFFSET_SS_DB=${PAVONIS_GNB_POWER_CONTROL_OFFSET_SS_DB:-}"
  echo "PAVONIS_GNB_PUSCH_DEC_MAX_ITERATIONS=${PAVONIS_GNB_PUSCH_DEC_MAX_ITERATIONS:-}"
  echo "PAVONIS_GNB_RA_RESP_WINDOW=${GNB_RA_RESP_WINDOW:-}"
  echo "PAVONIS_ALLOW_EXTENDED_RA_RESP_WINDOW=${PAVONIS_ALLOW_EXTENDED_RA_RESP_WINDOW:-}"
  echo "PAVONIS_RACH_RAR_WINDOW_OFFSET_SLOTS=$RAR_WINDOW_OFFSET_SLOTS"
  echo "PAVONIS_SOAPY_RX_APPEND_ENABLE=${PAVONIS_SOAPY_RX_APPEND_ENABLE:-0}"
  echo "PAVONIS_SOAPY_RX_APPEND_IMMEDIATE=$PAVONIS_SOAPY_RX_APPEND_IMMEDIATE"
  echo "IFACE=$IFACE"
  echo "GNB_CONF=$GNB_CONF"
  echo "HARNESS_LOG=$HARNESS_LOG"
  echo "RUNNER=$RUNNER"
  echo "READY=$READY"

  echo
  echo "===== RF config ====="
  grep -nE 'gnb_id:|gnb_id_bit_length:|sector_id:|device_args|clock_source|srate:|tx_gain:|rx_gain:|clock_ppm:|tx_gain_backoff|expert_phy|pusch_dec_max_iterations|dl_arfcn|band:|channel_bandwidth_MHz|plmn:|pci:|q_qual_min|pdcch|ss0_index|coreset0_index|max_coreset0_duration|pdsch|fixed_sib1_mcs|max_ue_mcs|ssb|ssb_block_power_dbm|prach_config_index|total_nof_ra_preambles|nof_cb_preambles_per_ssb|ra_resp_window|prach|broadcast_enabled' "$GNB_CONF"

  if [ "${PAVONIS_GNB_CONFIG_ONLY:-0}" = "1" ]; then
    echo
    echo "===== config-only exit ====="
    echo "PAVONIS_GNB_CONFIG_ONLY=1"
    echo "GNB_CONF=$GNB_CONF"
    echo "MARKED_TX_DETACHED_RC=0"
    exit 0
  fi

  echo
  echo "===== starting detached harness ====="
  START_EPOCH="$(date -u +%s)"
  START_UTC="$(date -u --iso-8601=ns)"
  echo "TX_START_REQUEST_UTC=$START_UTC"
  echo "START_EPOCH=$START_EPOCH"

  nohup "$RUNNER" > "$HARNESS_LOG" 2>&1 < /dev/null &
  HPID=$!
  echo "HARNESS_PID=$HPID"

  echo
  echo "===== waiting for NEW ota-stage gnb.log marker ====="

  OTA=""
  GNB_LOG=""
  READY_SEEN=0

  for i in $(seq 1 180); do
    OTA_FROM_LOG="$(grep -aoE "$ROOT/runs/[0-9]{8}_[0-9]{6}_ota-stage" "$HARNESS_LOG" 2>/dev/null | tail -1 || true)"
    find "$ROOT/runs" -maxdepth 1 -type d -name '*_ota-stage' 2>/dev/null | sort > "$NOW"
    OTA_NEW="$(comm -13 "$BEFORE" "$NOW" | tail -1 || true)"

    if [ -n "$OTA_FROM_LOG" ]; then
      OTA="$OTA_FROM_LOG"
    elif [ -n "$OTA_NEW" ]; then
      OTA="$OTA_NEW"
    fi

    if [ -n "$OTA" ]; then
      GNB_LOG="$OTA/gnb.log"
    fi

    if [ -n "$GNB_LOG" ] && [ -f "$GNB_LOG" ]; then
      GNB_MTIME="$(stat -c %Y "$GNB_LOG" 2>/dev/null || echo 0)"
      if [ "$GNB_MTIME" -ge "$START_EPOCH" ] && grep -q '==== gNB started ===' "$GNB_LOG"; then
        READY_SEEN=1
        TX_READY_UTC="$(date -u --iso-8601=ns)"
        CELL_LINE="$(grep -m1 'Cell pci=' "$GNB_LOG" || true)"
        cp -f "$GNB_LOG" /tmp/gnb_m2sdr_band3_ota.log 2>/dev/null || true

        cat > "$READY" <<EOR2
RF_TX_WINDOW_OPEN=1
RUN_UE_NOW=1
TX_READY_UTC=$TX_READY_UTC
OUT=$OUT
OTA=$OTA
GNB_LOG=$GNB_LOG
HARNESS_PID=$HPID
HARNESS_LOG=$HARNESS_LOG
ARM=$ARM
TX_READBACK_PATH=$TX_READBACK
DL_CENTER_HZ=1842500000
SSB_CENTER_HZ=1839650000
EXPECTED_PCI=1
TX_ATT_DB=$TX_ATT
TX_BACKOFF_DB=$TX_BACKOFF
GNB_RX_GAIN_DB=$GNB_RX_GAIN
PAVONIS_SOAPY_TX_FORCE_CONTINUOUS_STREAM=$FORCE_CONTINUOUS_STREAM
PAVONIS_SOAPY_ALLOW_M2SDR_CONTINUOUS_TX_MODE=$ALLOW_CONTINUOUS_MODE
PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS=$ALL_TIMED_CHUNKS
PAVONIS_SOAPY_TX_REANCHOR_INTERVAL_SAMPLES=$REANCHOR_INTERVAL_SAMPLES
M2SDR_SOAPY_TX_ALIGN_HAS_TIME_REMAINDER=$ALIGN_HAS_TIME_REMAINDER
PAVONIS_GNB_SI_OCCASION_TRACE=$GNB_SI_OCCASION_TRACE
PAVONIS_GNB_SI_OCCASION_TRACE_PATH=$GNB_SI_OCCASION_TRACE_PATH
PAVONIS_GNB_SI_OCCASION_TRACE_LIMIT=$GNB_SI_OCCASION_TRACE_LIMIT
PAVONIS_GNB_RAR_SCHED_TRACE=$GNB_RAR_SCHED_TRACE
PAVONIS_GNB_RAR_SCHED_TRACE_LIMIT=$GNB_RAR_SCHED_TRACE_LIMIT
M2SDR_SOAPY_TX_WORKER_TRACE_PATH=$TX_WORKER_TRACE_PATH
M2SDR_SOAPY_TX_WORKER_TRACE_LIMIT=$TX_WORKER_TRACE_LIMIT
M2SDR_SOAPY_TX_WORKER_TRACE_FLUSH_PERIOD=$TX_WORKER_TRACE_FLUSH_PERIOD
M2SDR_LITEPCIE_ZERO_COPY=$LITEPCIE_ZERO_COPY
M2SDR_SOAPY_TX_COPY_PRIME_BUFFERS=$TX_COPY_PRIME_BUFFERS
M2SDR_SOAPY_TX_ZERO_COPY_PRIME_BUFFERS=$TX_ZERO_COPY_PRIME_BUFFERS
M2SDR_SOAPY_TX_PROBE_ARM_FILE=$TX_PROBE_ARM_FILE
M2SDR_SOAPY_TX_PROBE_ARM_POLL_PERIOD=$TX_PROBE_ARM_POLL_PERIOD
PAVONIS_GNB_SRSUE_PDCCH_COMPAT=${PAVONIS_GNB_SRSUE_PDCCH_COMPAT:-}
PAVONIS_GNB_CORESET0_INDEX=${PAVONIS_GNB_CORESET0_INDEX:-}
PAVONIS_GNB_SS0_INDEX=${PAVONIS_GNB_SS0_INDEX:-}
PAVONIS_GNB_MAX_CORESET0_DURATION=${PAVONIS_GNB_MAX_CORESET0_DURATION:-}
PAVONIS_GNB_FIXED_SIB1_MCS=${PAVONIS_GNB_FIXED_SIB1_MCS:-}
PAVONIS_GNB_MAX_UE_MCS=${PAVONIS_GNB_MAX_UE_MCS:-}
PAVONIS_GNB_MAX_MSG4_MCS=${PAVONIS_GNB_MAX_MSG4_MCS:-}
PAVONIS_GNB_MAX_CONRES_MCS=${PAVONIS_GNB_MAX_CONRES_MCS:-}
PAVONIS_GNB_FALLBACK_DL_DCI_AGGR_LEV=${PAVONIS_GNB_FALLBACK_DL_DCI_AGGR_LEV:-}
PAVONIS_GNB_SS1_N_CANDIDATES=${PAVONIS_GNB_SS1_N_CANDIDATES:-}
PAVONIS_QCORE_SIM_FILE=$SIM_FILE
PAVONIS_GNB_SSB_BLOCK_POWER_DBM=${PAVONIS_GNB_SSB_BLOCK_POWER_DBM:-}
PAVONIS_GNB_Q_QUAL_MIN=${PAVONIS_GNB_Q_QUAL_MIN:-}
PAVONIS_GNB_SIB1_DCI_AGGR_LEV=${PAVONIS_GNB_SIB1_DCI_AGGR_LEV:-}
PAVONIS_GNB_POWER_CONTROL_OFFSET_SS_DB=${PAVONIS_GNB_POWER_CONTROL_OFFSET_SS_DB:-}
PAVONIS_GNB_CLOCK_PPM=$GNB_CLOCK_PPM
PAVONIS_GNB_PLMN=$GNB_PLMN
PAVONIS_OTA_STAGE_TEMPLATE=$OTA_STAGE_TEMPLATE
PAVONIS_GNB_PUSCH_DEC_MAX_ITERATIONS=${PAVONIS_GNB_PUSCH_DEC_MAX_ITERATIONS:-}
PAVONIS_GNB_RA_RESP_WINDOW=${GNB_RA_RESP_WINDOW:-}
PAVONIS_ALLOW_EXTENDED_RA_RESP_WINDOW=${PAVONIS_ALLOW_EXTENDED_RA_RESP_WINDOW:-}
PAVONIS_RACH_RAR_WINDOW_OFFSET_SLOTS=$RAR_WINDOW_OFFSET_SLOTS
OCUDU_GNB_BIN=$OCUDU_GNB_BIN_OVERRIDE
PAVONIS_GNB_MIB_INTRAFREQ_RESELECTION_ALLOWED=$GNB_MIB_INTRAFREQ_RESELECTION_ALLOWED
PAVONIS_GNB_SIB1_RETX_PERIOD_MS=$GNB_SIB1_RETX_PERIOD_MS
PAVONIS_GNB_SIB1_USE_N0=$GNB_SIB1_USE_N0
PAVONIS_GNB_MSG3_DELTA_PREAMBLE=$GNB_MSG3_DELTA_PREAMBLE
PAVONIS_SOAPY_PHYSICAL_CHANNEL_OFFSET=$PHYSICAL_CHANNEL_OFFSET
EOR2

        echo
        echo "######################################################################"
        echo "RF_TX_WINDOW_OPEN=1"
        echo "RUN_UE_NOW=1"
        echo "TX_READY_UTC=$TX_READY_UTC"
        echo "TX_READY_FILE=$READY"
        echo "TX_READBACK_PATH=$TX_READBACK"
        echo "ARM=$ARM"
        echo "HARNESS_PID=$HPID"
        echo "OTA=$OTA"
        echo "GNB_LOG=$GNB_LOG"
        echo "$CELL_LINE"
        echo "######################################################################"
        echo
        break
      fi
    fi

    if ! kill -0 "$HPID" 2>/dev/null; then
      echo "ERROR: harness exited before a fresh gNB started marker"
      break
    fi

    sleep 1
  done

  echo
  echo "===== verdict ====="
  echo "READY_SEEN=$READY_SEEN"
  echo "OUT=$OUT"
  echo "READY=$READY"
  echo "ARM=$ARM"
  echo "TX_READBACK=$TX_READBACK"
  echo "PAVONIS_SOAPY_TX_FORCE_CONTINUOUS_STREAM=$FORCE_CONTINUOUS_STREAM"
  echo "OCUDU_GNB_BIN=$OCUDU_GNB_BIN_OVERRIDE"
  echo "PAVONIS_GNB_MIB_INTRAFREQ_RESELECTION_ALLOWED=$GNB_MIB_INTRAFREQ_RESELECTION_ALLOWED"
  echo "PAVONIS_GNB_SIB1_RETX_PERIOD_MS=$GNB_SIB1_RETX_PERIOD_MS"
  echo "PAVONIS_GNB_SIB1_USE_N0=$GNB_SIB1_USE_N0"
  echo "PAVONIS_GNB_MSG3_DELTA_PREAMBLE=$GNB_MSG3_DELTA_PREAMBLE"
  echo "PAVONIS_SOAPY_PHYSICAL_CHANNEL_OFFSET=$PHYSICAL_CHANNEL_OFFSET"
  echo "PAVONIS_SOAPY_ALLOW_M2SDR_CONTINUOUS_TX_MODE=$ALLOW_CONTINUOUS_MODE"
  echo "PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS=$ALL_TIMED_CHUNKS"
  echo "PAVONIS_SOAPY_TX_REANCHOR_INTERVAL_SAMPLES=$REANCHOR_INTERVAL_SAMPLES"
  echo "M2SDR_SOAPY_TX_ALIGN_HAS_TIME_REMAINDER=$ALIGN_HAS_TIME_REMAINDER"
  echo "PAVONIS_RACH_RAR_WINDOW_OFFSET_SLOTS=$RAR_WINDOW_OFFSET_SLOTS"
  echo "PAVONIS_GNB_RA_RESP_WINDOW=${GNB_RA_RESP_WINDOW:-}"
  echo "PAVONIS_ALLOW_EXTENDED_RA_RESP_WINDOW=${PAVONIS_ALLOW_EXTENDED_RA_RESP_WINDOW:-}"
  echo "HARNESS_PID=$HPID"
  echo "HARNESS_LOG=$HARNESS_LOG"

  if [ "$READY_SEEN" -eq 1 ]; then
    echo "MARKED_TX_DETACHED_RC=0"
    exit 0
  fi

  echo "MARKED_TX_DETACHED_RC=1"
  exit 1
} 2>&1 | tee "$WATCH_LOG"

RC=${PIPESTATUS[0]}

{
  echo "===== compact detached marked TX summary ====="
  echo "OUT=$OUT"
  echo "WATCH_LOG=$WATCH_LOG"
  echo "HARNESS_LOG=$HARNESS_LOG"
  echo "READY=$READY"
  echo "ARM=$ARM"
  echo "TX_READBACK=$TX_READBACK"
  grep -Ei 'RF_TX_WINDOW_OPEN=|RUN_UE_NOW=|TX_READY_UTC=|TX_READY_FILE=|TX_READBACK_PATH=|TX_READBACK=|ARM=|HARNESS_PID=|OTA=|GNB_LOG=|Cell pci=|dl_arfcn|dl_ssb_arfcn|==== gNB started|READY_SEEN=|MARKED_TX_DETACHED_RC=|PAVONIS_SOAPY_TX_READBACK_PATH=|PAVONIS_SOAPY_TX_FORCE_CONTINUOUS_STREAM=|PAVONIS_SOAPY_ALLOW_M2SDR_CONTINUOUS_TX_MODE=|PAVONIS_SOAPY_TX_ALL_TIMED_CHUNKS=|PAVONIS_SOAPY_TX_REANCHOR_INTERVAL_SAMPLES=|PAVONIS_SOAPY_RX_|OCUDU_SOAPY_RX_USE_READSTREAM=|M2SDR_SOAPY_TX_ALIGN_HAS_TIME_REMAINDER=|M2SDR_SOAPY_TX_WORKER_TRACE|PAVONIS_GNB_|PAVONIS_ALLOW_EXTENDED_RA_RESP_WINDOW=|PAVONIS_RACH_RAR|pavonis_gnb_demarginalization_yaml=|pavonis_gnb_ra_resp_window_yaml=|PAVONIS_PRACH|PAVONIS_UL_SYMBOL' "$WATCH_LOG" "$HARNESS_LOG" 2>/dev/null | sed -n '1,480p'
  if [ -f "$TX_READBACK" ]; then
    echo "===== TX readback sidecar ====="
    sed -n '1,160p' "$TX_READBACK"
  else
    echo "TX_READBACK_MISSING=1"
  fi
  echo "WRAPPER_RC=$RC"
} | tee "$SUMMARY"
