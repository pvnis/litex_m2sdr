#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
STAMP="${STAMP:-$(date -u +%Y%m%dT%H%M%SZ)CP2324_MSG4_CRNTI_TRACE}"
TAG="${TAG:-codex_stage6_cp2324_msg4_crnti_trace}"

export ART STAMP TAG
export PAVONIS_SRSUE_CRNTI_CANDIDATE_TRACE=1
export PAVONIS_SRSUE_CRNTI_CANDIDATE_TRACE_LIMIT=256
export PAVONIS_SRSUE_CRNTI_CANDIDATE_TRACE_MIN_CORR=0.0

exec bash "$ART/cp2318_run_stage8_ue_tx_gain66.sh"
