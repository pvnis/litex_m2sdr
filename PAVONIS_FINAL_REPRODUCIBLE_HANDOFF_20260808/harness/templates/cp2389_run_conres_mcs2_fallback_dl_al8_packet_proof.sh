#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
STAMP="${STAMP:-$(date -u +%Y%m%dT%H%M%SZ)CONRES_MCS2_FALLBACK_DL_AL8_PACKET_PROOF}"
TAG="${TAG:-codex_stage6_conres_mcs2_fallback_dl_al8_packet_proof_${STAMP}}"

export ART STAMP TAG
unset GNB_MAX_MSG4_MCS
export GNB_MAX_CONRES_MCS=2
export GNB_FALLBACK_DL_DCI_AGGR_LEV=8
export GNB_SS1_N_CANDIDATES=0,0,1,1,0

exec bash "$ART/cp2362_run_pucch_ack_order_trace.sh" "$@"
