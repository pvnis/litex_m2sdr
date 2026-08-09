#!/usr/bin/env bash
set -euo pipefail

SERIAL="${1:-${PAVONIS_PHONE_ADB_SERIAL:?pass the authorized phone ADB serial}}"
SUB_ID="${2:-${PAVONIS_PHONE_SUB_ID:-6}}"
PROBE="${PAVONIS_CP3030_PROBE:-0}"
REBOOT=@REMOTE_HOME@/pavonis_cp2662_phone_reboot_reaudit_remote.sh
REBOOT_SHA=b71f598df639321800fe96c0766b93f05d7f5e066976d3aa433cc95aaf827cef
N3=@REMOTE_HOME@/pavonis_cp2552_n3_radioinfo_phone_rf_remote.sh
N3_SHA=c6bfd5e3f6393fd6f7152b393782da364ea9e4a6f6040eb52bb692215da40e22
UPDATE_MODAL=@REMOTE_HOME@/pavonis_cp3032_dismiss_system_update_modal_remote.sh
UPDATE_MODAL_SHA=827ba47ace9a33af300b71bc24a9b1af1d16668f578d2b9bd2af1d8df99859e7
VOICE_JAR=/data/local/tmp/pavonis_voice_policy_v2.cp2563.jar
FPLMN_JAR=/data/local/tmp/pavonis_fplmn_repair.cp2661.jar
MANUAL_JAR=/data/local/tmp/pavonis_manual_plmn.jar
EXPECTED_FPLMN_HASH=20825f15d52cb0f59cc8ffc4f83879e50b9c729a552b6084a4497535daf3ddab
ADB=(/usr/bin/adb -s "$SERIAL")

[[ "$SERIAL" =~ ^[A-Za-z0-9._:-]+$ && "$SUB_ID" =~ ^[0-9]+$ ]]
[[ "$PROBE" =~ ^[01]$ ]]
[[ $(sha256sum "$REBOOT" | awk '{print $1}') == "$REBOOT_SHA" ]]
[[ $(sha256sum "$N3" | awk '{print $1}') == "$N3_SHA" ]]
[[ $(sha256sum "$UPDATE_MODAL" | awk '{print $1}') == "$UPDATE_MODAL_SHA" ]]
[[ -z $(pgrep -x srsenb || true) ]]
[[ -z $(pgrep -x srsue || true) ]]
[[ "$("${ADB[@]}" get-state)" == device ]]

if [[ "$PROBE" == 1 ]]; then
  echo "CP3030_PHONE_REBOOT_UNDER_TARGET_PROBE=PASS reboot_hash=1 n3_hash=1 update_modal_hash=1 adb=device idle=1"
  exit 0
fi

set +e
"$REBOOT"
reboot_rc=$?
set -e
# This handset normally returns 1 because reboot resets VoNR. Every state is
# re-audited below, so no other RC1 failure is accepted implicitly.
[[ "$reboot_rc" == 0 || "$reboot_rc" == 1 ]]

[[ "$("${ADB[@]}" get-state)" == device ]]
[[ "$("${ADB[@]}" shell getprop sys.boot_completed | tr -d '\r')" == 1 ]]
"$UPDATE_MODAL" "$SERIAL"

run_root_java() {
  local jar="$1"
  shift
  "${ADB[@]}" shell "su -c 'CLASSPATH=$jar app_process /system/bin $*'" | tr -d '\r'
}

voice_set="$(run_root_java "$VOICE_JAR" PavonisVoicePolicy set "$SUB_ID" true true)"
[[ "$voice_set" == *'PAVONIS_VOICE_POLICY_SET=PASS'* ]]

"$N3" apply "$SERIAL" "$SUB_ID"

fplmn="$(run_root_java "$FPLMN_JAR" PavonisFplmnRepair audit "$SUB_ID" 2 90170)"
[[ "$fplmn" == *'count=13'* ]]
[[ "$fplmn" == *'occurrences=0'* ]]
[[ "$fplmn" == *"hash=$EXPECTED_FPLMN_HASH"* ]]
selection="$(run_root_java "$MANUAL_JAR" PavonisManualPlmn get "$SUB_ID")"
[[ "$selection" == *' mode=1'* ]]
voice="$(run_root_java "$VOICE_JAR" PavonisVoicePolicy get "$SUB_ID")"
[[ "$voice" == *'vonr=true advanced_calling=true'* ]]
[[ "$("${ADB[@]}" shell settings get global preferred_network_mode6 | tr -d '\r')" == 23 ]]
[[ "$("${ADB[@]}" shell settings get global oplus_user_preferred_network_mode6 | tr -d '\r')" == 33 ]]
allowed="$("${ADB[@]}" shell cmd phone get-allowed-network-types-for-users -s 0 | tr -d '\r')"
[[ "$allowed" == NR ]]
[[ -z $("${ADB[@]}" shell ip -o addr show | tr -d '\r' | awk '$4 ~ /^10[.]255[.]0[.]2\//{print}') ]]

echo "CP3030_PHONE_REBOOT_UNDER_TARGET=PASS reboot_rc=$reboot_rc adb=device boot=1 automatic=1 n3=1 voice=1 stale_private=0"
