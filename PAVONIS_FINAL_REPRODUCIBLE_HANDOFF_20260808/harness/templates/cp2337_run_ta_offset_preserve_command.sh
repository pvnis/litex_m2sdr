#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
STAMP="${STAMP:-$(date -u +%Y%m%dT%H%M%SZ)CP2337_TA_OFFSET_PRESERVE_COMMAND}"
TAG="${TAG:-codex_stage6_cp2337_ta_offset_preserve_command}"

export ART STAMP TAG
export PAVONIS_SRSUE_TA_HANDOFF_TRACE=1
export PAVONIS_SRSUE_TA_HANDOFF_APPLY_CURRENT=1
export PAVONIS_SRSUE_TA_HANDOFF_TRACE_LIMIT=64
export PAVONIS_SRSUE_TA_OFFSET_PRESERVE_COMMAND=1

exec bash "$ART/cp2329_run_post_msg4_pusch_target_capture.sh"
