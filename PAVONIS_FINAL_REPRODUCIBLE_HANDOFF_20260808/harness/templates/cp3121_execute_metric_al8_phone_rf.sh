#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-@CONTROLLER_HARNESS@}"
SSH="$ART/ssh_pexpect_run.py"
RUNNER="$ART/cp3121_run_metric_al8_phone_rf.sh"
RUNNER_SHA=a5484a606e7a2af8910903a9f90be75360e70149df755d6a16fcdede6720699b
STAMP="${STAMP:?set a fresh STAMP}"
TAG="${TAG:-cp3121_metric_al8_phone_rf}"
SERIAL_TMP=$(mktemp /tmp/pavonis_cp3121_phone.XXXXXX.local)
trap 'rm -f "$SERIAL_TMP"' EXIT
chmod 600 "$SERIAL_TMP"

[[ "$(sha256sum "$RUNNER" | awk '{print $1}')" == "$RUNNER_SHA" ]]

python3 "$SSH" --host ue --out "$SERIAL_TMP" --timeout 30 -- bash -lc \
  'set -euo pipefail; /usr/bin/adb devices'
mapfile -t serials < <(awk '{sub(/\r$/, "", $2)} $2 == "device" {print $1}' "$SERIAL_TMP")
[[ "${#serials[@]}" == 1 ]]

exec env \
  PAVONIS_PHONE_ADB_SERIAL="${serials[0]}" \
  PAVONIS_CP3121_RF_APPROVED=1 \
  STAMP="$STAMP" TAG="$TAG" \
  "$RUNNER"
