#!/usr/bin/env bash
set -euo pipefail

STAMP="${1:?fresh stamp}"
SERIAL="${2:?pass the authorized phone ADB serial}"
SUB_ID="${3:-6}"
[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ && "$SERIAL" =~ ^[A-Za-z0-9._:-]+$ && "$SUB_ID" =~ ^[0-9]+$ ]]

ROOT="@REMOTE_HOME@/pavonis_cp2969_runs/$STAMP"
REBOOT=@REMOTE_HOME@/cp2662_phone_reboot_reaudit_remote.sh
PHONE=@REMOTE_HOME@/pavonis_cp2629_phone_remote.sh
FPLMN=@REMOTE_HOME@/cp2661_fplmn_repair_remote.sh
ROAM=@REMOTE_HOME@/pavonis_cp2574_phone_data_roaming_remote.sh
MANUAL_JAR=/data/local/tmp/pavonis_manual_plmn.jar
VOICE_JAR=/data/local/tmp/pavonis_voice_policy_v2.cp2563.jar
ADB=(/usr/bin/adb -s "$SERIAL")

[[ ! -e "$ROOT" ]]
mkdir -p "$ROOT"
chmod 700 "$ROOT"
[[ -z $(pgrep -x srsenb || true) && -z $(pgrep -x srsue || true) ]]

set +e
env PAVONIS_PHONE_ADB_SERIAL="$SERIAL" PAVONIS_PHONE_SUB_ID="$SUB_ID" \
  "$REBOOT" >"$ROOT/reboot_reaudit.log" 2>&1
reboot_rc=$?
set -e
[[ "$reboot_rc" == 0 || "$reboot_rc" == 1 ]]
[[ $("${ADB[@]}" get-state) == device ]]
[[ $("${ADB[@]}" shell getprop sys.boot_completed | tr -d '\r') == 1 ]]

restore_stamp="${STAMP}_RESTORE"
mkdir -p "@REMOTE_HOME@/pavonis_cp2629_runs/$restore_stamp"
chmod 700 "@REMOTE_HOME@/pavonis_cp2629_runs/$restore_stamp"
"$PHONE" restore "$restore_stamp" "$SERIAL" "$SUB_ID" >"$ROOT/restore.log" 2>&1

bash "$FPLMN" audit "$STAMP" >"$ROOT/fplmn.log" 2>&1
grep -q '^CP2661_FPLMN_AUDIT=PASS' "$ROOT/fplmn.log"
manual="$("${ADB[@]}" shell "su -c 'CLASSPATH=$MANUAL_JAR app_process /system/bin PavonisManualPlmn get $SUB_ID'" | tr -d '\r')"
mask="$("${ADB[@]}" shell cmd phone get-allowed-network-types-for-users -s 0 | tr -d '\r')"
mode6="$("${ADB[@]}" shell settings get global preferred_network_mode6 | tr -d '\r')"
oplus_mode6="$("${ADB[@]}" shell settings get global oplus_user_preferred_network_mode6 | tr -d '\r')"
roaming="$("$ROAM" get "$SERIAL" "$SUB_ID" | tr -d '\r')"
voice="$("${ADB[@]}" shell "su -c 'CLASSPATH=$VOICE_JAR app_process /system/bin PavonisVoicePolicy get $SUB_ID'" | tr -d '\r')"
stale_private="$("${ADB[@]}" shell ip -o -4 addr show 2>/dev/null | tr -d '\r' | awk '$4 ~ /^10\.255\.0\.2\// {n++} END {print n+0}')"

[[ "$manual" == *' mode=1'* ]]
[[ "$mask" == *'NR'* ]]
[[ "$mode6" == 33 && "$oplus_mode6" == 33 ]]
[[ "$roaming" == *' value=0'* ]]
[[ "$voice" == *'vonr=true advanced_calling=true'* ]]
[[ "$stale_private" == 0 ]]

printf 'CP2969_PHONE_FRESH_EPOCH=PASS reboot_rc=%s selection=1 mode6=%s oplus_mode6=%s roaming=0 voice=1 stale_private=%s adb=device\n' \
  "$reboot_rc" "$mode6" "$oplus_mode6" "$stale_private"
