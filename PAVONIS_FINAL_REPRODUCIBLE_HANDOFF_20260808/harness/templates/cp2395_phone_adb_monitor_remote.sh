#!/usr/bin/env bash
set -euo pipefail

SERIAL="${1:?ADB serial required}"
DURATION="${2:?duration required}"
OUT="${3:?output path required}"
INTERVAL="${PAVONIS_PHONE_MONITOR_INTERVAL_SEC:-3}"

[[ "$DURATION" =~ ^[0-9]+$ ]] && ((DURATION >= 10 && DURATION <= 300))
[[ "$INTERVAL" =~ ^[0-9]+$ ]] && ((INTERVAL >= 1 && INTERVAL <= 15))

state="$(adb devices | awk -v serial="$SERIAL" '$1 == serial {print $2; exit}')"
if [[ "$state" != "device" ]]; then
  printf 'MONITOR_ERROR adb_state=%q\n' "${state:-missing}" > "$OUT"
  exit 2
fi

start="$(date +%s)"
deadline=$((start + DURATION))
private_plmn_seen=0
in_service_seen=0
nr_seen=0
cellular_ip_seen=0
nr_signal_valid_seen=0
nr_quality_valid_seen=0
nr_camped_seen=0

{
  printf 'MONITOR_START_UTC=%s serial=%s duration_sec=%s interval_sec=%s\n' \
    "$(date -u +%FT%TZ)" "$SERIAL" "$DURATION" "$INTERVAL"
  printf 'utc\telapsed_sec\toperator_numeric\tnetwork_type\tvoice_reg\tdata_reg\tvoice_rat\tdata_rat\tnr_available\tdata_connection_state\tcellular_iface\tcellular_ipv4\tnr_ss_rsrp\tnr_level\tlte_rsrp\tlte_level\twcdma_ss\twcdma_level\tprimary_signal\tnr_ss_rsrq\tnr_ss_sinr\tnr_csi_rsrp\tnr_csi_rsrq\tnr_timing_advance\tnr_ps_reg_state\tnr_ps_reject_cause\tnr_ps_emergency\tnr_ps_registered_plmn\tnr_cell_pci\tnr_cell_tac\tnr_cell_nci\tnr_cell_arfcn\tnr_cell_bands\n'
} > "$OUT"

while (( $(date +%s) < deadline )); do
  now="$(date +%s)"
  dump="$(adb -s "$SERIAL" shell dumpsys telephony.registry 2>/dev/null || true)"
  service="$(printf '%s\n' "$dump" | grep -m1 'mServiceState=' || true)"
  data_line="$(printf '%s\n' "$dump" | grep -m1 'mDataConnectionState=' || true)"
  signal="$(printf '%s\n' "$dump" | grep -m1 'mSignalStrength=' || true)"
  nr_ps="$(printf '%s\n' "$service" | sed 's/NetworkRegistrationInfo{/\nNetworkRegistrationInfo{/g' | grep 'domain=PS' | grep -m1 'accessNetworkTechnology=NR' || true)"
  nr_cell="$(printf '%s\n' "$dump" | grep -m1 'mCellIdentity=CellIdentityNr' || true)"

  operator_numeric="$(adb -s "$SERIAL" shell getprop gsm.operator.numeric 2>/dev/null | tr -d '\r\n' | tr ' ' '_' || true)"
  network_type="$(adb -s "$SERIAL" shell getprop gsm.network.type 2>/dev/null | tr -d '\r\n' | tr ' ' '_' || true)"
  voice_reg="$(printf '%s\n' "$service" | sed -n 's/.*mVoiceRegState=\([^,}]*\).*/\1/p' | tr ' ' '_' || true)"
  data_reg="$(printf '%s\n' "$service" | sed -n 's/.*mDataRegState=\([^,}]*\).*/\1/p' | tr ' ' '_' || true)"
  voice_rat="$(printf '%s\n' "$service" | sed -n 's/.*getRilVoiceRadioTechnology=\([^,}]*\).*/\1/p' | tr ' ' '_' || true)"
  data_rat="$(printf '%s\n' "$service" | sed -n 's/.*getRilDataRadioTechnology=\([^,}]*\).*/\1/p' | tr ' ' '_' || true)"
  nr_available="$(printf '%s\n' "$service" | sed -n 's/.*isNrAvailable = \([^ }]*\).*/\1/p' || true)"
  data_state="$(printf '%s\n' "$data_line" | sed -n 's/.*mDataConnectionState=\([^ ]*\).*/\1/p' || true)"
  cell_line="$(adb -s "$SERIAL" shell ip -o -4 addr show 2>/dev/null | tr -d '\r' | awk '$2 ~ /^(rmnet|ccmni|pdp|v4-rmnet)/ {print $2 "\t" $4; exit}' || true)"
  cell_iface="$(printf '%s' "$cell_line" | cut -f1)"
  cell_ip="$(printf '%s' "$cell_line" | cut -f2)"
  nr_ss_rsrp="$(printf '%s\n' "$signal" | sed -n 's/.*ssRsrp = \([^ ]*\).*/\1/p' || true)"
  nr_ss_rsrq="$(printf '%s\n' "$signal" | sed -n 's/.*ssRsrq = \([^ ]*\).*/\1/p' || true)"
  nr_ss_sinr="$(printf '%s\n' "$signal" | sed -n 's/.*ssSinr = \([^ ]*\).*/\1/p' || true)"
  nr_csi_rsrp="$(printf '%s\n' "$signal" | sed -n 's/.*csiRsrp = \([^ ]*\).*/\1/p' || true)"
  nr_csi_rsrq="$(printf '%s\n' "$signal" | sed -n 's/.*csiRsrq = \([^ ]*\).*/\1/p' || true)"
  nr_timing_advance="$(printf '%s\n' "$signal" | sed -n 's/.*timingAdvance = \([^ ]*\).*/\1/p' || true)"
  nr_level="$(printf '%s\n' "$signal" | sed -n 's/.* level = \([^ ]*\) parametersUseForLevel.*/\1/p' || true)"
  lte_fields="$(printf '%s\n' "$signal" | sed -n 's/.*mLte=CellSignalStrengthLte: \(.*\),mNr=.*/\1/p' || true)"
  lte_rsrp="$(printf '%s\n' "$lte_fields" | sed -n 's/.*rsrp=\([^ ]*\).*/\1/p' || true)"
  lte_level="$(printf '%s\n' "$lte_fields" | sed -n 's/.*level=\([^ ]*\).*/\1/p' || true)"
  wcdma_fields="$(printf '%s\n' "$signal" | sed -n 's/.*mWcdma=CellSignalStrengthWcdma: \(.*\),mTdscdma=.*/\1/p' || true)"
  wcdma_ss="$(printf '%s\n' "$wcdma_fields" | sed -n 's/.*ss=\([^ ]*\).*/\1/p' || true)"
  wcdma_level="$(printf '%s\n' "$wcdma_fields" | sed -n 's/.*level=\([^ ]*\).*/\1/p' || true)"
  primary_signal="$(printf '%s\n' "$signal" | sed -n 's/.*primary=\([^}]*\).*/\1/p' || true)"
  nr_ps_reg_state="$(printf '%s\n' "$nr_ps" | sed -n 's/.*registrationState=\([^ ,}]*\).*/\1/p' || true)"
  nr_ps_reject_cause="$(printf '%s\n' "$nr_ps" | sed -n 's/.*rejectCause=\([^ ,}]*\).*/\1/p' || true)"
  nr_ps_emergency="$(printf '%s\n' "$nr_ps" | sed -n 's/.*emergencyEnabled=\([^ ,}]*\).*/\1/p' || true)"
  nr_ps_registered_plmn="$(printf '%s\n' "$nr_ps" | sed -n 's/.*registeredPlmn=\([^ ,}]*\).*/\1/p' || true)"
  nr_cell_pci="$(printf '%s\n' "$nr_cell" | sed -n 's/.*mPci = \([^ ]*\).*/\1/p' || true)"
  nr_cell_tac="$(printf '%s\n' "$nr_cell" | sed -n 's/.*mTac = \([^ ]*\).*/\1/p' || true)"
  nr_cell_nci="$(printf '%s\n' "$nr_cell" | sed -n 's/.*mNci = \([^ ]*\).*/\1/p' || true)"
  nr_cell_arfcn="$(printf '%s\n' "$nr_cell" | sed -n 's/.*mNrArfcn = \([^ ]*\).*/\1/p' || true)"
  nr_cell_bands="$(printf '%s\n' "$nr_cell" | sed -n 's/.*mBands = \(\[[^]]*\]\).*/\1/p' | tr ' ' '_' || true)"

  [[ "$operator_numeric" == *00101* ]] && private_plmn_seen=1
  [[ "$voice_reg" == *IN_SERVICE* || "$data_reg" == *IN_SERVICE* ]] && in_service_seen=1
  [[ "$network_type" == *NR* || "$voice_rat" == *NR* || "$data_rat" == *NR* || "$nr_available" == true ]] && nr_seen=1
  [[ -n "$cell_ip" ]] && cellular_ip_seen=1
  [[ -n "$nr_ss_rsrp" && "$nr_ss_rsrp" != 2147483647 ]] && nr_signal_valid_seen=1
  [[ -n "$nr_ss_rsrq" && "$nr_ss_rsrq" != 2147483647 ]] && nr_quality_valid_seen=1
  [[ "$nr_ps_reg_state" == HOME || "$nr_ps_reg_state" == ROAMING ]] && nr_camped_seen=1

  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t' \
    "$(date -u +%FT%TZ)" "$((now - start))" "$operator_numeric" "$network_type" \
    "$voice_reg" "$data_reg" "$voice_rat" "$data_rat" "$nr_available" \
    "$data_state" "$cell_iface" "$cell_ip" "$nr_ss_rsrp" "$nr_level" \
    "$lte_rsrp" "$lte_level" "$wcdma_ss" "$wcdma_level" "$primary_signal" >> "$OUT"
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$nr_ss_rsrq" "$nr_ss_sinr" "$nr_csi_rsrp" "$nr_csi_rsrq" "$nr_timing_advance" \
    "$nr_ps_reg_state" "$nr_ps_reject_cause" "$nr_ps_emergency" "$nr_ps_registered_plmn" \
    "$nr_cell_pci" "$nr_cell_tac" "$nr_cell_nci" "$nr_cell_arfcn" "$nr_cell_bands" >> "$OUT"
  sleep "$INTERVAL"
done

{
  printf 'MONITOR_SUMMARY private_plmn_seen=%s in_service_seen=%s nr_seen=%s cellular_ip_seen=%s nr_signal_valid_seen=%s nr_quality_valid_seen=%s nr_camped_seen=%s\n' \
    "$private_plmn_seen" "$in_service_seen" "$nr_seen" "$cellular_ip_seen" "$nr_signal_valid_seen" \
    "$nr_quality_valid_seen" "$nr_camped_seen"
  final_iface="$(adb -s "$SERIAL" shell ip -o -4 addr show 2>/dev/null | tr -d '\r' | awk '$2 ~ /^(rmnet|ccmni|pdp|v4-rmnet)/ {print $2; exit}' || true)"
  if [[ "$private_plmn_seen" == 1 && -n "$final_iface" ]]; then
    printf 'PRIVATE_GATEWAY_PING_BEGIN iface=%s\n' "$final_iface"
    adb -s "$SERIAL" shell ping -I "$final_iface" -c 5 -W 2 10.255.0.1 2>&1 | tr -d '\r'
    printf 'PRIVATE_GATEWAY_PING_END\n'
  else
    printf 'PRIVATE_GATEWAY_PING_SKIPPED private_plmn_seen=%s cellular_iface=%q\n' \
      "$private_plmn_seen" "$final_iface"
  fi
  printf 'MONITOR_END_UTC=%s\n' "$(date -u +%FT%TZ)"
} >> "$OUT"
