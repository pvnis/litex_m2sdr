#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
STAMP="${STAMP:-$(date -u +%Y%m%dT%H%M%SZ)CP2318_STAGE8_UE_TX_GAIN66}"
TAG="${TAG:-codex_stage6_cp2318_stage8_ue_tx_gain66}"

export ART STAMP TAG
export UE_TX_GAIN_DB=66

exec bash "$ART/cp2313_run_pucch_f1_phase_search_full_bank.sh"
