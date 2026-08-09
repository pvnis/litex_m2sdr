#!/usr/bin/env bash
set -euo pipefail

SERIAL="${PAVONIS_PHONE_ADB_SERIAL:?set PAVONIS_PHONE_ADB_SERIAL}"
SUB_ID="${PAVONIS_PHONE_SUB_ID:-6}"
FPLMN_JAR=/data/local/tmp/pavonis_fplmn_repair.cp2661.jar
MANUAL_JAR=/data/local/tmp/pavonis_manual_plmn.jar
VOICE_JAR=/data/local/tmp/pavonis_voice_policy_v2.cp2563.jar
ROAM=@REMOTE_HOME@/pavonis_cp2574_phone_data_roaming_remote.sh
EXPECTED_FPLMN_HASH=20825f15d52cb0f59cc8ffc4f83879e50b9c729a552b6084a4497535daf3ddab
FULL_MASK='GPRS|EDGE|UMTS|CDMA|CDMA - EvDo rev. 0|CDMA - EvDo rev. A|CDMA - 1xRTT|HSDPA|HSUPA|HSPA|CDMA - EvDo rev. B|LTE|CDMA - eHRPD|HSPA+|GSM|TD_SCDMA|LTE_CA|NR'
ADB=(/usr/bin/adb -s "$SERIAL")

[[ "$SERIAL" =~ ^[A-Za-z0-9._:-]+$ && "$SUB_ID" =~ ^[0-9]+$ ]]
[[ -z $(pgrep -x srsenb || true) ]]
[[ -z $(pgrep -x srsue || true) ]]
[[ "$("${ADB[@]}" get-state)" == device ]]

echo "CP2662_REBOOT_START_UTC=$(date -u +%FT%TZ)"
"${ADB[@]}" reboot

offline=0
for _ in $(seq 1 60); do
  if ! "${ADB[@]}" get-state >/dev/null 2>&1; then
    offline=1
    break
  fi
  sleep 1
done
[[ "$offline" == 1 ]]

online=0
for _ in $(seq 1 240); do
  if [[ $("${ADB[@]}" get-state 2>/dev/null || true) == device ]]; then
    online=1
    break
  fi
  sleep 1
done
[[ "$online" == 1 ]]

booted=0
for _ in $(seq 1 180); do
  if [[ $("${ADB[@]}" shell getprop sys.boot_completed 2>/dev/null | tr -d '\r') == 1 ]]; then
    booted=1
    break
  fi
  sleep 1
done
[[ "$booted" == 1 ]]
sleep 15
"${ADB[@]}" shell input keyevent 82 >/dev/null
"${ADB[@]}" shell input keyevent 3 >/dev/null

run_root_java() {
  local jar="$1"
  shift
  "${ADB[@]}" shell "su -c 'CLASSPATH=$jar app_process /system/bin $*'" | tr -d '\r'
}

fplmn="$(run_root_java "$FPLMN_JAR" PavonisFplmnRepair audit "$SUB_ID" 2 90170)"
printf '%s\n' "$fplmn"
[[ "$fplmn" == *'count=13'* ]]
[[ "$fplmn" == *'occurrences=0'* ]]
[[ "$fplmn" == *"hash=$EXPECTED_FPLMN_HASH"* ]]

selection="$(run_root_java "$MANUAL_JAR" PavonisManualPlmn get "$SUB_ID")"
printf '%s\n' "$selection"
[[ "$selection" == *' mode=1'* ]]

mask="$("${ADB[@]}" shell cmd phone get-allowed-network-types-for-users -s 0 | tr -d '\r')"
mode6="$("${ADB[@]}" shell settings get global preferred_network_mode6 | tr -d '\r')"
oplus_mode6="$("${ADB[@]}" shell settings get global oplus_user_preferred_network_mode6 | tr -d '\r')"
roaming="$("$ROAM" get "$SERIAL" "$SUB_ID" | tr -d '\r')"
voice="$(run_root_java "$VOICE_JAR" PavonisVoicePolicy get "$SUB_ID")"

[[ "$mask" == "$FULL_MASK" ]]
[[ "$mode6" == 33 && "$oplus_mode6" == 33 ]]
[[ "$roaming" == *' value=0'* ]]
[[ "$voice" == *'vonr=true advanced_calling=true'* ]]

printf 'CP2662_PHONE_BASELINE selection=1 full_rat=1 mode6=%s oplus_mode6=%s roaming=0 voice=1 adb=device\n' \
  "$mode6" "$oplus_mode6"
echo "CP2662_REBOOT_END_UTC=$(date -u +%FT%TZ)"
echo CP2662_PHONE_REBOOT_REAUDIT=PASS
