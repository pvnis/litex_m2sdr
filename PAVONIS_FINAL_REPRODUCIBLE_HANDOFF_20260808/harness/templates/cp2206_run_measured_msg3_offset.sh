#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-@CONTROLLER_HARNESS@}"
STAMP="${STAMP:-$(date -u +%Y%m%dT%H%M%SZ)CP2206_MEASURED_MSG3_OFFSET}"
TAG="${TAG:-codex_stage6_cp2206_measured_msg3_offset}"

export ART STAMP TAG
export PAVONIS_NON_PRACH_UL_RX_TIMESTAMP_OFFSET_SAMPLES="${PAVONIS_NON_PRACH_UL_RX_TIMESTAMP_OFFSET_SAMPLES:--241118}"

exec bash "$ART/cp2201_run_scheduled_msg3_target_capture.sh"
