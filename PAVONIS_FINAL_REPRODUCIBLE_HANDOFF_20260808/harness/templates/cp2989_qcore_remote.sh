#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:?start, status, or stop}"
STAMP="${2:?fresh stamp}"
RAN_INTERFACE_NAME="${PAVONIS_QCORE_RAN_INTERFACE_NAME:-@RAN_INTERFACE@}"
[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ ]]
[[ "$RAN_INTERFACE_NAME" == @RAN_INTERFACE@ || "$RAN_INTERFACE_NAME" == lo ]]

ROOT="@REMOTE_HOME@/pavonis_cp2629_runs/$STAMP"
PID_FILE="$ROOT/qcore.pid"
LOG="$ROOT/qcore.log"
QCORE=@REMOTE_HOME@/pavonis_cp2989_qcore_unknown_update_fallback/qcore
QCORE_SHA=f2bb11379bc91ff1f443b6f67f854181b07a76106c71c4583007e071e92102d3
SIM=@REMOTE_HOME@/CLionProjects/m2sdr/litex_m2sdr/software/validation/ocudu/configs/sims-pavonis-ota.private.toml

redact() {
  sed -E 's/imsi-[0-9]+/imsi-<redacted>/g; s/[0-9]{15,20}/<redacted_numeric_id>/g'
}

sanitize_log() {
  local tmp
  [[ -f "$LOG" ]] || return 0
  tmp="$(mktemp "$ROOT/qcore.redacted.XXXXXX")"
  redact <"$LOG" >"$tmp"
  chmod 600 "$tmp"
  mv "$tmp" "$LOG"
}

stop_qcore() {
  local pid=''
  [[ -s "$PID_FILE" ]] && pid="$(cat "$PID_FILE")"
  if [[ "$pid" =~ ^[0-9]+$ ]] && sudo kill -0 "$pid" 2>/dev/null; then
    sudo kill -TERM "$pid" 2>/dev/null || true
    for _ in $(seq 1 50); do
      sudo kill -0 "$pid" 2>/dev/null || break
      sleep 0.1
    done
    sudo kill -0 "$pid" 2>/dev/null && sudo kill -KILL "$pid" 2>/dev/null || true
  fi
  rm -f "$PID_FILE"
}

count_log() {
  local pattern="$1"
  grep -cF "$pattern" "$LOG" 2>/dev/null || true
}

case "$ACTION" in
  start)
    mkdir -p "$ROOT"
    chmod 700 "$ROOT"
    [[ ! -e "$PID_FILE" ]]
    [[ "$(sha256sum "$QCORE" | cut -d ' ' -f1)" == "$QCORE_SHA" ]]
    [[ $(stat -c %a "$SIM") == 600 ]]
    [[ -z $(pgrep -x qcore || true) ]]
    ip -4 addr show dev @RAN_INTERFACE@ | grep -q '@CORE_N2_IP@/24'
    ip link show "$RAN_INTERFACE_NAME" >/dev/null
    ip link show qcoretun >/dev/null
    sudo true
    : >"$LOG"
    chmod 600 "$LOG"
    nohup sudo env \
      PAVONIS_QCORE_ALLOW_FOREIGN_GUTI_IDENTITY_FALLBACK=1 \
      PAVONIS_QCORE_ALLOW_FOREIGN_SUPI_HOME_PLMN=1 \
      PAVONIS_QCORE_ALLOW_UNKNOWN_UPDATE_IDENTITY_FALLBACK=1 \
      "$QCORE" \
      --local-ip @CORE_N2_IP@ \
      --ran-interface-name "$RAN_INTERFACE_NAME" \
      --sim-cred-file "$SIM" \
      --mcc 901 --mnc 70 \
      --no-dhcp --ue-subnet 10.255.0.0 \
      --n6-interface-name veth1 --tun-interface-name qcoretun \
      >"$LOG" 2>&1 </dev/null &
    launcher_pid=$!
    ready=0
    for _ in $(seq 1 100); do
      if grep -q 'My AMF NGAP port    : @CORE_N2_IP@:38412' "$LOG" &&
         grep -q "Interface to RAN    : $RAN_INTERFACE_NAME (from command line)" "$LOG"; then
        ready=1
        break
      fi
      sudo kill -0 "$launcher_pid" 2>/dev/null || break
      sleep 0.1
    done
    if [[ "$ready" != 1 ]]; then
      stop_qcore
      redact <"$LOG" | sed -n '1,80p'
      sanitize_log
      exit 1
    fi
    qcore_pid="$(pgrep -n -x qcore)"
    printf '%s\n' "$qcore_pid" >"$PID_FILE"
    sudo ss -A sctp -ln | grep -q '@CORE_N2_IP@:38412'
    echo "CP2989_QCORE_START=PASS pid=$qcore_pid root=$ROOT binary_sha256=$QCORE_SHA ran_interface=$RAN_INTERFACE_NAME guti_fallback=1 supi_home_plmn=1 unknown_update_fallback=1"
    ;;
  status)
    sudo true
    pid="$(cat "$PID_FILE")"
    sudo kill -0 "$pid"
    printf 'CP2989_QCORE_STATUS=RUNNING pid=%s registration_request=%s guti_fallback=%s supi_home_plmn=%s unknown_update_fallback=%s reject=%s identity_request=%s identity_response=%s auth_request=%s auth_response=%s nas_smc=%s nas_smc_complete=%s rrc_smc=%s rrc_smc_complete=%s reg_accept=%s reg_complete=%s userplane_activate=%s\n' \
      "$pid" \
      "$(count_log '>> Nas RegistrationRequest')" \
      "$(count_log 'PAVONIS_QCORE_ALLOW_FOREIGN_GUTI_IDENTITY_FALLBACK active')" \
      "$(count_log 'PAVONIS_QCORE_ALLOW_FOREIGN_SUPI_HOME_PLMN active')" \
      "$(count_log 'PAVONIS_QCORE_ALLOW_UNKNOWN_UPDATE_IDENTITY_FALLBACK active')" \
      "$(count_log 'Reject registration')" \
      "$(count_log '<< Nas IdentityRequest')" \
      "$(count_log '>> Nas IdentityResponse')" \
      "$(count_log '<< NasAuthenticationRequest')" \
      "$(count_log '>> Nas AuthenticationResponse')" \
      "$(count_log '<< NasSecurityModeCommand')" \
      "$(count_log '>> Nas SecurityModeComplete')" \
      "$(count_log '<< Rrc SecurityModeCommand')" \
      "$(count_log '>> Rrc SecurityModeComplete')" \
      "$(count_log '<< Nas RegistrationAccept')" \
      "$(count_log 'Registration Complete')" \
      "$(count_log 'Activate userplane session')"
    grep -E 'PAVONIS_QCORE_ALLOW_(FOREIGN_(GUTI|SUPI)|UNKNOWN_UPDATE)|Nas RegistrationRequest|Nas Identity|NasAuthentication|Nas Authentication|NasSecurityMode|Nas SecurityMode|Rrc SecurityMode|RegistrationAccept|Registration Complete|Activate userplane session|Reject registration|WARN|ERROR' "$LOG" \
      | redact | tail -n 100 || true
    ;;
  stop)
    sudo true
    stop_qcore
    [[ -z $(pgrep -x qcore || true) ]]
    sanitize_log
    echo CP2989_QCORE_STOP=PASS
    ;;
  *)
    echo "usage: $0 start|status|stop STAMP" >&2
    exit 64
    ;;
esac
