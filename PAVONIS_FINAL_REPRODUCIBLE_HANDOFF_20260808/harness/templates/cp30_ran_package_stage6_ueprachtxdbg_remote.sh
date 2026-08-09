#!/usr/bin/env bash
set -euo pipefail

TAG="${TAG:-codex_stage6_cp30_ueprachtxdbg}"
TAR="${NUC4_TAR:-/tmp/${TAG}_ran.tar.gz}"
MAN="/tmp/${TAG}_ran_tar_manifest.txt"
REL="/tmp/${TAG}_ran_tar_manifest.rel.txt"
RUNS="$HOME/CLionProjects/m2sdr/litex_m2sdr/software/validation/ocudu/runs"

rm -f "$TAR" "$MAN" "$REL"

add_path() {
  if [ -e "$1" ]; then
    printf '%s\n' "$1" >> "$MAN"
  else
    printf 'MISSING %s\n' "$1" >&2
  fi
}

TX="$(ls -td "$RUNS"/${TAG}_* 2>/dev/null | head -1 || true)"
OTA="$(ls -td "$RUNS"/*_ota-stage 2>/dev/null | head -1 || true)"

add_path /tmp/pavonis_prachdet_trace.log
add_path /tmp/pavonis_prach_predemod_manifest.tsv
add_path /tmp/pavonis_prach_strong_predemod_manifest.tsv
add_path /tmp/pavonis_prach_demod_input_cfo_search_trace.tsv
add_path /tmp/pavonis_prach_demod_input_stage1_trace.tsv
add_path /tmp/pavonis_prach_collection_segments.tsv
add_path /tmp/pavonis_prach_late_context_trace.tsv
add_path /tmp/pavonis_ul_symbol_manifest.tsv
add_path /tmp/pavonis_soapy_rx_manifest.tsv
add_path /tmp/pavonis_soapy_rx_append_meta.tsv
add_path /tmp/pavonis_soapy_rx_readback.txt
add_path /tmp/pavonis_gnb_si_occasion_trace.tsv
add_path /tmp/pavonis_gnb_rar_fapi_trace.tsv
add_path /tmp/pavonis_m2sdr_tx_worker_trace.tsv
add_path /tmp/pavonis_dl_slot_capture_manifest.tsv
add_path /tmp/gnb_m2sdr_band3_ota.log
if [ -n "${PAVONIS_SOAPY_TX_WRITE_DUMP_CF32_PATH:-}" ]; then
  add_path "$PAVONIS_SOAPY_TX_WRITE_DUMP_CF32_PATH"
fi
if [ -n "${PAVONIS_SOAPY_TX_WRITE_DUMP_META_PATH:-}" ]; then
  add_path "$PAVONIS_SOAPY_TX_WRITE_DUMP_META_PATH"
fi
if [ -n "${M2SDR_SOAPY_TX_DUMP_PATH:-}" ]; then
  add_path "$M2SDR_SOAPY_TX_DUMP_PATH"
  add_path "${M2SDR_SOAPY_TX_DUMP_PATH}.meta.jsonl"
fi
if [ -n "${M2SDR_SOAPY_TX_DMA_DUMP_PATH:-}" ]; then
  add_path "$M2SDR_SOAPY_TX_DMA_DUMP_PATH"
  add_path "${M2SDR_SOAPY_TX_DMA_DUMP_PATH}.meta.jsonl"
fi

if [ -n "$TX" ]; then add_path "$TX"; else echo "MISSING_TX_FOR_TAG=$TAG" >&2; fi
if [ -n "$OTA" ]; then add_path "$OTA"; else echo "MISSING_OTA_STAGE" >&2; fi

shopt -s nullglob
for raw in /tmp/pavonis_prach_predemod_*.cf32; do
  printf '%s\n' "$raw" >> "$MAN"
done

for raw in /tmp/pavonis_prach_strong_predemod_*.cf32; do
  printf '%s\n' "$raw" >> "$MAN"
done

for raw in /tmp/pavonis_ul_symbol_*.cf32; do
  printf '%s\n' "$raw" >> "$MAN"
done

for raw in /tmp/pavonis_pucch_f1_grid_*.cf32; do
  printf '%s\n' "$raw" >> "$MAN"
done

for raw in /tmp/pavonis_soapy_rx_*.cf32; do
  printf '%s\n' "$raw" >> "$MAN"
done

for raw in /tmp/pavonis_dl_slot_capture_*.ci16; do
  printf '%s\n' "$raw" >> "$MAN"
done

test -s "$MAN"
sed 's#^/##' "$MAN" > "$REL"
tar -czf "$TAR" -C / -T "$REL"

ls -lh "$TAR"
sha256sum "$TAR"
echo "TAG=$TAG"
echo "TX=$TX"
echo "OTA=$OTA"
echo "MANIFEST_BEGIN"
cat "$MAN"
echo "MANIFEST_END"
