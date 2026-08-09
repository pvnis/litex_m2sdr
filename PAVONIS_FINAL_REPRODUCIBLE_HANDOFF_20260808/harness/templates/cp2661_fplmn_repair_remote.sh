#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:?deploy-audit, audit, remove, retrieve-backup, or restore}"
STAMP="${2:?fresh stamp}"
EXPECTED_HASH="${3:-}"
SERIAL="${PAVONIS_PHONE_ADB_SERIAL:?set PAVONIS_PHONE_ADB_SERIAL}"
SUB_ID="${PAVONIS_PHONE_SUB_ID:-6}"
APP_TYPE=2
TARGET=90170
HOST_JAR=@REMOTE_HOME@/pavonis_fplmn_repair.cp2661.jar
DEVICE_JAR=/data/local/tmp/pavonis_fplmn_repair.cp2661.jar
JAR_SHA=cc6c2c8ea415b504a2cdbfa9620eeb656b25fcbe9091663972ffcfbba77aa21e
DEVICE_BACKUP="/data/local/tmp/pavonis_fplmn_backup_${STAMP}.txt"
HOST_BACKUP_DIR=@REMOTE_HOME@/pavonis_cp2661_fplmn_backups
HOST_BACKUP="$HOST_BACKUP_DIR/${STAMP}.txt"
ADB=(/usr/bin/adb -s "$SERIAL")

[[ "$STAMP" =~ ^[A-Za-z0-9_.-]+$ ]]
[[ "$SERIAL" =~ ^[A-Za-z0-9._:-]+$ && "$SUB_ID" =~ ^[0-9]+$ ]]

run_java() {
  "${ADB[@]}" shell \
    "su -c 'CLASSPATH=$DEVICE_JAR app_process /system/bin PavonisFplmnRepair $*'" |
    tr -d '\r'
}

verify_device_jar() {
  local actual
  actual="$("${ADB[@]}" shell "su -c 'sha256sum $DEVICE_JAR'" | tr -d '\r' | awk '{print $1}')"
  [[ "$actual" == "$JAR_SHA" ]]
  echo "CP2661_DEVICE_JAR_SHA256=$actual"
}

case "$ACTION" in
  deploy-audit)
    [[ "$("${ADB[@]}" get-state)" == device ]]
    [[ "$(sha256sum "$HOST_JAR" | awk '{print $1}')" == "$JAR_SHA" ]]
    "${ADB[@]}" push "$HOST_JAR" "$DEVICE_JAR" >/dev/null
    "${ADB[@]}" shell "su -c 'chown root:shell $DEVICE_JAR; chmod 600 $DEVICE_JAR'"
    verify_device_jar
    run_java audit "$SUB_ID" "$APP_TYPE" "$TARGET"
    echo CP2661_FPLMN_DEPLOY_AUDIT=PASS
    ;;
  audit)
    verify_device_jar
    run_java audit "$SUB_ID" "$APP_TYPE" "$TARGET"
    echo CP2661_FPLMN_AUDIT=PASS
    ;;
  remove)
    [[ "$EXPECTED_HASH" =~ ^[0-9a-f]{64}$ ]]
    verify_device_jar
    mkdir -p "$HOST_BACKUP_DIR"
    chmod 700 "$HOST_BACKUP_DIR"
    [[ ! -e "$HOST_BACKUP" ]]
    if "${ADB[@]}" shell "su -c 'test -e $DEVICE_BACKUP'"; then
      echo "Device backup already exists: $DEVICE_BACKUP" >&2
      exit 2
    fi
    run_java remove "$SUB_ID" "$APP_TYPE" "$TARGET" "$EXPECTED_HASH" "$DEVICE_BACKUP"
    "${ADB[@]}" shell "su -c 'chown root:shell $DEVICE_BACKUP; chmod 600 $DEVICE_BACKUP'"
    "${ADB[@]}" pull "$DEVICE_BACKUP" "$HOST_BACKUP" >/dev/null
    chmod 600 "$HOST_BACKUP"
    printf 'CP2661_FPLMN_BACKUP host=%s device=%s sha256=%s mode=%s\n' \
      "$HOST_BACKUP" "$DEVICE_BACKUP" "$(sha256sum "$HOST_BACKUP" | awk '{print $1}')" \
      "$(stat -c %a "$HOST_BACKUP")"
    run_java audit "$SUB_ID" "$APP_TYPE" "$TARGET"
    echo CP2661_FPLMN_REMOVE=PASS
    ;;
  retrieve-backup)
    verify_device_jar
    mkdir -p "$HOST_BACKUP_DIR"
    chmod 700 "$HOST_BACKUP_DIR"
    [[ ! -e "$HOST_BACKUP" ]]
    "${ADB[@]}" shell "su -c 'test -f $DEVICE_BACKUP'"
    "${ADB[@]}" exec-out "su -c 'cat $DEVICE_BACKUP'" >"$HOST_BACKUP"
    chmod 600 "$HOST_BACKUP"
    [[ "$(stat -c %s "$HOST_BACKUP")" -gt 0 ]]
    printf 'CP2661_FPLMN_BACKUP_RETRIEVE=PASS host=%s device=%s sha256=%s mode=%s size=%s\n' \
      "$HOST_BACKUP" "$DEVICE_BACKUP" "$(sha256sum "$HOST_BACKUP" | awk '{print $1}')" \
      "$(stat -c %a "$HOST_BACKUP")" "$(stat -c %s "$HOST_BACKUP")"
    ;;
  restore)
    [[ "$EXPECTED_HASH" =~ ^[0-9a-f]{64}$ ]]
    verify_device_jar
    [[ -f "$HOST_BACKUP" && "$(stat -c %a "$HOST_BACKUP")" == 600 ]]
    [[ $("${ADB[@]}" shell "su -c 'test -f $DEVICE_BACKUP'; echo \$?" | tr -d '\r') == 0 ]]
    run_java restore "$SUB_ID" "$APP_TYPE" "$DEVICE_BACKUP" "$EXPECTED_HASH"
    run_java audit "$SUB_ID" "$APP_TYPE" "$TARGET"
    echo CP2661_FPLMN_RESTORE=PASS
    ;;
  *)
    echo "usage: $0 deploy-audit|audit|remove|retrieve-backup|restore STAMP [EXPECTED_HASH]" >&2
    exit 64
    ;;
esac
