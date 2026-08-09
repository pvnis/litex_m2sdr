#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
STAMP="${STAMP:-$(date -u +%Y%m%dT%H%M%SZ)CP2351_SECURITY_DL_AFTER_UL_ARM}"
TAG="${TAG:-codex_stage6_cp2351_security_dl_after_ul_arm}"

export ART STAMP TAG
export PAVONIS_UL_SYMBOL_MSG3_TARGET_ARM_ON_DL_AFTER_UL=1

exec bash "$ART/cp2347_run_security_pusch_late_arm.sh" "$@"
