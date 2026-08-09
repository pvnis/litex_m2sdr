#!/usr/bin/env bash
set -euo pipefail

SERIAL="${1:?ADB serial}"
HOLD_SEC="${2:-5}"
SETTLE_SEC="${3:-15}"
MODE="${4:-run}"
EXPECTED_PROFILE="${5:-full}"

[[ "$SERIAL" =~ ^[A-Za-z0-9._:-]+$ ]]
[[ "$HOLD_SEC" =~ ^[0-9]+$ ]] && ((HOLD_SEC >= 3 && HOLD_SEC <= 15))
[[ "$SETTLE_SEC" =~ ^[0-9]+$ ]] && ((SETTLE_SEC >= 10 && SETTLE_SEC <= 30))
[[ "$MODE" == run || "$MODE" == probe ]]
[[ "$EXPECTED_PROFILE" == full || "$EXPECTED_PROFILE" == n3 ]]

ADB=(/usr/bin/adb -s "$SERIAL")
MANUAL_JAR=/data/local/tmp/pavonis_manual_plmn.jar
FULL_MASK='GPRS|EDGE|UMTS|CDMA|CDMA - EvDo rev. 0|CDMA - EvDo rev. A|CDMA - 1xRTT|HSDPA|HSUPA|HSPA|CDMA - EvDo rev. B|LTE|CDMA - eHRPD|HSPA+|GSM|TD_SCDMA|LTE_CA|NR'

[[ "$("${ADB[@]}" get-state)" == device ]]

airplane_state() {
  "${ADB[@]}" shell cmd connectivity airplane-mode | tr -d '\r'
}

private_count() {
  "${ADB[@]}" shell ip -o -4 addr show | tr -d '\r' |
    awk '$4 ~ /^10\.255\.0\.2\// {n++} END{print n+0}'
}

restore_airplane_off() {
  if [[ "$(airplane_state 2>/dev/null || true)" != disabled ]]; then
    "${ADB[@]}" shell cmd connectivity airplane-mode disable >/dev/null 2>&1 || true
  fi
}

if [[ "$MODE" == probe ]]; then
  [[ "$(airplane_state)" == disabled ]]
  echo "CP2907_PHONE_AIRPLANE_PROBE=PASS hold_sec=$HOLD_SEC settle_sec=$SETTLE_SEC expected_profile=$EXPECTED_PROFILE"
  exit 0
fi

trap restore_airplane_off EXIT INT TERM HUP
[[ "$(airplane_state)" == disabled ]]
before_private="$(private_count)"
echo "CP2907_PHONE_AIRPLANE_STAGE=baseline before_private=$before_private"

"${ADB[@]}" shell cmd connectivity airplane-mode enable >/dev/null
for _ in $(seq 1 40); do
  [[ "$(airplane_state)" == enabled ]] && break
  sleep 0.25
done
[[ "$(airplane_state)" == enabled ]]
sleep "$HOLD_SEC"
on_private="$(private_count)"
echo "CP2907_PHONE_AIRPLANE_STAGE=enabled on_private=$on_private"

"${ADB[@]}" shell cmd connectivity airplane-mode disable >/dev/null
for _ in $(seq 1 40); do
  [[ "$(airplane_state)" == disabled ]] && break
  sleep 0.25
done
[[ "$(airplane_state)" == disabled ]]
echo CP2907_PHONE_AIRPLANE_STAGE=disabled
sleep "$SETTLE_SEC"

mode="$("${ADB[@]}" shell \
  "su -c 'CLASSPATH=$MANUAL_JAR app_process /system/bin PavonisManualPlmn get 6'" |
  tr -d '\r' | sed -n 's/.* mode=\([0-9][0-9]*\).*/\1/p')"
mask="$("${ADB[@]}" shell cmd phone get-allowed-network-types-for-users -s 0 | tr -d '\r')"
mode6="$("${ADB[@]}" shell settings get global preferred_network_mode6 | tr -d '\r')"
oplus_mode6="$("${ADB[@]}" shell settings get global oplus_user_preferred_network_mode6 | tr -d '\r')"
after_private="$(private_count)"
printf 'CP2907_PHONE_AIRPLANE_STAGE=post_settle profile=%s selection=%s mode6=%s oplus_mode6=%s after_private=%s\n' \
  "$EXPECTED_PROFILE" "$mode" "$mode6" "$oplus_mode6" "$after_private"

[[ "$mode" == 1 ]]
if [[ "$EXPECTED_PROFILE" == n3 ]]; then
  [[ "$mask" == NR ]]
  [[ "$mode6" == 23 && "$oplus_mode6" == 33 ]]
else
  [[ "$mask" == "$FULL_MASK" ]]
  [[ "$mode6" == 33 && "$oplus_mode6" == 33 ]]
fi
printf 'CP2907_PHONE_AIRPLANE_CYCLE profile=%s before_private=%s on_private=%s after_private=%s hold_sec=%s settle_sec=%s selection=%s mode6=%s oplus_mode6=%s\n' \
  "$EXPECTED_PROFILE" \
  "$before_private" "$on_private" "$after_private" "$HOLD_SEC" "$SETTLE_SEC" \
  "$mode" "$mode6" "$oplus_mode6"
echo CP2907_PHONE_AIRPLANE_CYCLE_PASS=1

trap - EXIT INT TERM HUP
