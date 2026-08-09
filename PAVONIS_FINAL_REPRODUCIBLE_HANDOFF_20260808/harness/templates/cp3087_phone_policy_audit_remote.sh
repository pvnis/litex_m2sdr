#!/usr/bin/env bash
set -euo pipefail

STAMP="${1:?fresh stamp}"
[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ ]]

ROOT="@REMOTE_HOME@/pavonis_cp3087_policy_audit/$STAMP"
FPLMN=@REMOTE_HOME@/cp2661_fplmn_repair_remote.sh
ROAM=@REMOTE_HOME@/pavonis_cp2574_phone_data_roaming_remote.sh
SIM_POWER=@REMOTE_HOME@/cp3086_phone_sim_power_cycle_remote.sh
MANUAL_JAR=/data/local/tmp/pavonis_manual_plmn.jar
VOICE_JAR=/data/local/tmp/pavonis_voice_policy_v2.cp2563.jar

[[ ! -e "$ROOT" ]]
mkdir -m 700 -p "$ROOT"
[[ -z $(pgrep -x srsenb || true) && -z $(pgrep -x srsue || true) ]]
[[ $(adb get-state 2>/dev/null) == device ]]

serial=$(adb get-serialno | tr -d '\r')
sub_id=$(adb shell settings get global multi_sim_data_call | tr -d '\r')
[[ "$serial" =~ ^[A-Za-z0-9._:-]+$ && "$sub_id" =~ ^[0-9]+$ ]]
ADB=(adb -s "$serial")

bash "$FPLMN" audit "$STAMP" >"$ROOT/fplmn.log" 2>&1
grep -q '^CP2661_FPLMN_AUDIT=PASS' "$ROOT/fplmn.log"

manual="$("${ADB[@]}" shell \
  "su -c 'CLASSPATH=$MANUAL_JAR app_process /system/bin PavonisManualPlmn get $sub_id'" | tr -d '\r')"
mask="$("${ADB[@]}" shell cmd phone get-allowed-network-types-for-users -s 0 | tr -d '\r')"
mode6="$("${ADB[@]}" shell settings get global preferred_network_mode6 | tr -d '\r')"
oplus_mode6="$("${ADB[@]}" shell settings get global oplus_user_preferred_network_mode6 | tr -d '\r')"
roaming="$("$ROAM" get "$serial" "$sub_id" | tr -d '\r')"
voice="$("${ADB[@]}" shell \
  "su -c 'CLASSPATH=$VOICE_JAR app_process /system/bin PavonisVoicePolicy get $sub_id'" | tr -d '\r')"
stale_private="$("${ADB[@]}" shell ip -o -4 addr show 2>/dev/null | tr -d '\r' | \
  awk '$4 ~ /^10\.255\.0\.2\// {n++} END {print n+0}')"

[[ "$manual" == *' mode=1'* ]]
[[ "$mask" == *'NR'* ]]
[[ "$mode6" == 33 && "$oplus_mode6" == 33 ]]
[[ "$roaming" == *' value=0'* ]]
[[ "$voice" == *'vonr=true advanced_calling=true'* ]]
[[ "$stale_private" == 0 ]]

env -u PAVONIS_CP3086_SIM_POWER_CYCLE_APPROVED \
  "$SIM_POWER" audit "${STAMP}_SIM_AUDIT" >"$ROOT/sim_state.log" 2>&1
grep -q '^PAVONIS_SIM_POWER_CYCLE mode=audit result=PASS changed=0$' "$ROOT/sim_state.log"
chmod 600 "$ROOT"/*.log

echo CP3087_PHONE_POLICY_AUDIT=PASS fplmn=1 selection=1 rat=1 roaming=0 voice=1 stale_private=0 sim=1
