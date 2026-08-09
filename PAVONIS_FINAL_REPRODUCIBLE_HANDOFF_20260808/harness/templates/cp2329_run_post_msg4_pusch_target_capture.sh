#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
STAMP="${STAMP:-$(date -u +%Y%m%dT%H%M%SZ)CP2329_POST_MSG4_PUSCH_TARGET_CAPTURE}"
TAG="${TAG:-codex_stage6_cp2329_post_msg4_pusch_target_capture}"

export ART STAMP TAG
export PAVONIS_UL_SYMBOL_MSG3_TARGET_CAPTURE=1
export PAVONIS_UL_SYMBOL_MSG3_TARGET_MAX=8
export PAVONIS_UL_SYMBOL_RAW_MAX_OVERRIDE=128
export PAVONIS_PUSCH_CSI_TRACE=1

exec bash "$ART/cp2324_run_msg4_crnti_trace.sh"
