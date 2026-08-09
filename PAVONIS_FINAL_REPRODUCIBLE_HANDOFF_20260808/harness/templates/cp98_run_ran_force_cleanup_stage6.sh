#!/usr/bin/env bash
set -euo pipefail

ART="@CONTROLLER_HARNESS@"
SSH="$ART/ssh_pexpect_run.py"
SCP_PUT="$ART/scp_pexpect_put.py"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
LOG="$ART/cp98_${STAMP}_sequence.log"

: > "$LOG"

log() {
  printf '%s %s\n' "$(date -u +%FT%TZ)" "$*" | tee -a "$LOG"
}

run_cmd() {
  local label="$1"
  shift
  log "START $label"
  "$@"
  local rc=$?
  log "DONE $label rc=$rc"
  return "$rc"
}

log "cp98_force_cleanup_sequence_start stamp=$STAMP"

run_cmd "put_ran_cp98_force_cleanup" \
  python3 "$SCP_PUT" \
    --host ran \
    --local "$ART/cp98_ran_force_cleanup_stage6_remote.sh" \
    --remote @REMOTE_HOME@/stage6_cp98_force_cleanup.sh \
    --out "$ART/cp98_${STAMP}_scp_put_ran_force_cleanup.log" \
    --timeout 120

run_cmd "chmod_ran_cp98_force_cleanup" \
  python3 "$SSH" \
    --host ran \
    --out "$ART/cp98_${STAMP}_ran_chmod_force_cleanup.log" \
    --timeout 30 \
    -- chmod +x @REMOTE_HOME@/stage6_cp98_force_cleanup.sh

run_cmd "run_ran_cp98_force_cleanup" \
  python3 "$SSH" \
    --host ran \
    --out "$ART/cp98_${STAMP}_ran_force_cleanup.log" \
    --timeout 60 \
    -- @REMOTE_HOME@/stage6_cp98_force_cleanup.sh
CLEANUP_RC=$?

log "cp98_force_cleanup_sequence_done stamp=$STAMP cleanup_rc=$CLEANUP_RC"
exit "$CLEANUP_RC"
