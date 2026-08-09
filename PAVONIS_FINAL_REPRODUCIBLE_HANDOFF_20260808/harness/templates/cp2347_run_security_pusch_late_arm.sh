#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
STAMP="${STAMP:-$(date -u +%Y%m%dT%H%M%SZ)CP2347_SECURITY_PUSCH_LATE_ARM}"
TAG="${TAG:-codex_stage6_cp2347_security_pusch_late_arm}"

export ART STAMP TAG
export PAVONIS_UL_SYMBOL_MSG3_TARGET_LATE_ARM=1

exec bash "$ART/cp2345_run_qcore_ota_subscriber.sh" "$@"
