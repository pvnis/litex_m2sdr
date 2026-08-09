#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-@CONTROLLER_HARNESS@}"
STAMP="${STAMP:-$(date -u +%Y%m%dT%H%M%SZ)CP2246_RAR_WORKER_PUSCH_RUNTIME_DUAL_TRACE}"
TAG="${TAG:-codex_stage6_cp2246_rar_worker_pusch_runtime_dual_trace}"

export ART STAMP TAG
export PAVONIS_GNB_RAR_TX_WORKER_TRACE_ARM_OVERRIDE=1
export M2SDR_SOAPY_TX_WORKER_TRACE_ARM_FILE_OVERRIDE="${M2SDR_SOAPY_TX_WORKER_TRACE_ARM_FILE_OVERRIDE-/tmp/pavonis_cp2246_rar_tx_worker_${STAMP}.arm}"

exec bash "$ART/cp2232_run_msg3_edge_calibration.sh"
