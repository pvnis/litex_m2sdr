#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-audit}"
STAMP="${2:-cp3086_audit}"
JAR=@REMOTE_HOME@/pavonis_sim_power_cycle.jar
PHONE_JAR=/data/local/tmp/pavonis_sim_power_cycle.jar
EXPECTED_JAR_SHA256=6ee9fee93c89a183bd385802824404747280b473fcf1b72402fa7eacca599da2

[[ "$MODE" == audit || "$MODE" == cycle ]]
[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ ]]
[[ -z $(pgrep -x srsenb || true) && -z $(pgrep -x srsue || true) ]]
[[ $(adb get-state 2>/dev/null) == device ]]
[[ $(sha256sum "$JAR" | cut -d ' ' -f1) == "$EXPECTED_JAR_SHA256" ]]

sub_id=$(adb shell settings get global multi_sim_data_call | tr -d '\r')
[[ "$sub_id" =~ ^[0-9]+$ ]]
slot_row=$(adb shell su -c \
  "content query --uri content://telephony/siminfo --projection sim_id --where _id=$sub_id" \
  2>/dev/null | tr -d '\r')
slot=$(sed -n 's/.*sim_id=\([0-9][0-9]*\).*/\1/p' <<<"$slot_row")
[[ "$slot" =~ ^[0-9]+$ ]]
phone_sha=$(adb shell sha256sum "$PHONE_JAR" | tr -d '\r' | cut -d ' ' -f1)
[[ "$phone_sha" == "$EXPECTED_JAR_SHA256" ]]

if [[ "$MODE" == audit ]]; then
  [[ "${PAVONIS_CP3086_SIM_POWER_CYCLE_APPROVED:-0}" == 0 ]]
  adb shell su 0 sh -c \
    "CLASSPATH=$PHONE_JAR app_process / PavonisSimPowerCycle $slot audit"
  exit 0
fi

[[ "${PAVONIS_CP3086_SIM_POWER_CYCLE_APPROVED:-0}" == 1 ]]
ROOT="@REMOTE_HOME@/pavonis_cp3086_sim_power_runs/$STAMP"
[[ ! -e "$ROOT" ]]
mkdir -m 700 -p "$ROOT"

set +e
adb shell su 0 sh -c \
  "CLASSPATH=$PHONE_JAR app_process / PavonisSimPowerCycle $slot cycle PAVONIS_CP3086_SIM_POWER_CYCLE_APPROVED" \
  >"$ROOT/cycle.log" 2>&1
cycle_rc=$?
set -e
chmod 600 "$ROOT/cycle.log"
[[ "$cycle_rc" == 0 ]]
grep -q '^PAVONIS_SIM_POWER_CYCLE mode=cycle step=down result=PASS ' "$ROOT/cycle.log"
grep -q '^PAVONIS_SIM_POWER_CYCLE mode=cycle step=up result=PASS ' "$ROOT/cycle.log"

[[ $(adb get-state 2>/dev/null) == device ]]
final_sub_id=$(adb shell settings get global multi_sim_data_call | tr -d '\r')
[[ "$final_sub_id" =~ ^[0-9]+$ ]]
[[ "$final_sub_id" == "$sub_id" ]]
state=$(adb shell su -c \
  "content query --uri content://telephony/siminfo --projection uicc_applications_enabled --where _id=$sub_id" \
  2>/dev/null | tr -d '\r')
[[ "$state" == *'uicc_applications_enabled=1'* ]]

echo CP3086_PHONE_SIM_POWER_CYCLE=PASS
