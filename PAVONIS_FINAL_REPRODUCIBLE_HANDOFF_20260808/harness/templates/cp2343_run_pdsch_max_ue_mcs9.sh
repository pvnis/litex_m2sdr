#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
STAMP="${STAMP:-$(date -u +%Y%m%dT%H%M%SZ)CP2343_PDSCH_MAX_UE_MCS9}"
TAG="${TAG:-codex_stage6_cp2343_pdsch_max_ue_mcs9}"

export ART STAMP TAG
export GNB_MAX_UE_MCS=9

exec bash "$ART/cp2341_run_cp2339_stage1_candidate.sh"
