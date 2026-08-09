#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
STAMP="${STAMP:-$(date -u +%Y%m%dT%H%M%SZ)CP2362_PUCCH_ACK_ORDER_TRACE}"
TAG="${TAG:-codex_stage6_cp2362_pucch_ack_order_trace}"

export ART STAMP TAG
export PAVONIS_PUXCH_ORDER_TRACE=1
export PAVONIS_PUXCH_ORDER_TRACE_LIMIT=2048
export PAVONIS_PUXCH_ORDER_TRACE_LATE_ARM=1
export PAVONIS_NON_PRACH_UL_RX_PIPELINE_DEPTH_SLOTS=40
# Empty disables the diagnostic selector; numeric 0 actively targets phase zero.
export PAVONIS_GNB_RAR_TARGET_SLOT_MOD160=""
export PAVONIS_DL_SLOT_CAPTURE="${PAVONIS_DL_SLOT_CAPTURE:-0}"

exec bash "$ART/cp2353_run_initial_context_trace.sh" "$@"
