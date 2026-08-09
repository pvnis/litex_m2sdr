#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
STAMP="${STAMP:-$(date -u +%Y%m%dT%H%M%SZ)CP2353_INITIAL_CONTEXT_TRACE}"
TAG="${TAG:-codex_stage6_cp2353_initial_context_trace}"

export ART STAMP TAG
export PAVONIS_GNB_INITIAL_CONTEXT_TRACE=1

exec bash "$ART/cp2351_run_security_dl_after_ul_arm.sh" "$@"
