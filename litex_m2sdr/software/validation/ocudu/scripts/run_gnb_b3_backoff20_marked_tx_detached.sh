#!/usr/bin/env bash
set +e +u
set +o pipefail 2>/dev/null || true

if [ "$(hostname)" != "nuc4" ]; then
  echo "ERROR: this must run on nuc4, current host is: $(hostname)"
  exit 2
fi

sudo -v || exit 3

DURATION="${DURATION:-300}"
TX_BACKOFF="${TX_BACKOFF:-20}"
ROOT="$PWD"

IFACE="${IFACE:-$(ip route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="dev"){print $(i+1); exit}}')}"
[ -n "$IFACE" ] || IFACE="wlp5s0"

OUT="${OUT:-$ROOT/runs/gnb_b3_backoff20_marked_tx_detached_$(date -u +%Y%m%dT%H%M%SZ)}"
mkdir -p "$OUT"

GNB_CONF="$OUT/gnb_m2sdr_band3_ota.backoff${TX_BACKOFF}.yml"
SIM_FILE="$ROOT/configs/sims-ocudu-zmq.example.toml"
UE_CONF="$ROOT/configs/ue-bladerf-band3-ota.template.conf"

HARNESS_LOG="$OUT/harness.log"
WATCH_LOG="$OUT/watch.log"
RUNNER="$OUT/run_harness.sh"
READY="$OUT/TX_READY.env"
SUMMARY="$OUT/SUMMARY.txt"
BEFORE="$OUT/ota_dirs_before.txt"
NOW="$OUT/ota_dirs_now.txt"

find "$ROOT/runs" -maxdepth 1 -type d -name '*_ota-stage' 2>/dev/null | sort > "$BEFORE"

cp -av "$ROOT/configs/gnb_m2sdr_band3_ota.template.yml" "$GNB_CONF" >/dev/null

python3 - <<'PY' "$GNB_CONF" "$TX_BACKOFF"
from pathlib import Path
import re
import sys

p = Path(sys.argv[1])
backoff = sys.argv[2]
s = p.read_text()
s = re.sub(r'(\btx_gain_backoff:\s*)30\b', rf'\g<1>{backoff}', s)
p.write_text(s)
print(f"patched_private_run_config={p}")
PY

cat > "$RUNNER" <<EOR
#!/usr/bin/env bash
set +e +u
set +o pipefail 2>/dev/null || true

cd "$ROOT" || exit 1

echo "===== detached OTA harness ====="
hostname
date -u --iso-8601=seconds
echo "GNB_CONF=$GNB_CONF"
echo "SIM_FILE=$SIM_FILE"
echo "UE_CONF=$UE_CONF"
echo "DURATION=$DURATION"
echo "IFACE=$IFACE"

./scripts/run_ota_stage_template.sh \
  --i-understand-rf-test \
  --gnb-conf "$GNB_CONF" \
  --sim-file "$SIM_FILE" \
  --ue-conf "$UE_CONF" \
  --duration "$DURATION" \
  --external-iface "$IFACE"

RC=\$?
echo "run_ota_stage_template_rc=\$RC"
date -u --iso-8601=seconds
exit "\$RC"
EOR

chmod +x "$RUNNER"

{
  echo "===== detached marked OCUDU gNB Band 3 TX window ====="
  hostname
  date -u --iso-8601=ns
  echo "OUT=$OUT"
  echo "DURATION=$DURATION"
  echo "TX_BACKOFF=$TX_BACKOFF"
  echo "IFACE=$IFACE"
  echo "GNB_CONF=$GNB_CONF"
  echo "HARNESS_LOG=$HARNESS_LOG"
  echo "RUNNER=$RUNNER"
  echo "READY=$READY"

  echo
  echo "===== RF config ====="
  grep -nE 'device_args|srate:|tx_gain:|rx_gain:|tx_gain_backoff|dl_arfcn|band:|channel_bandwidth_MHz|pci:|broadcast_enabled' "$GNB_CONF"

  echo
  echo "===== starting detached harness ====="
  START_EPOCH="$(date -u +%s)"
  START_UTC="$(date -u --iso-8601=ns)"
  echo "TX_START_REQUEST_UTC=$START_UTC"
  echo "START_EPOCH=$START_EPOCH"

  nohup "$RUNNER" > "$HARNESS_LOG" 2>&1 < /dev/null &
  HPID=$!
  echo "HARNESS_PID=$HPID"

  echo
  echo "===== waiting for NEW ota-stage gnb.log marker ====="

  OTA=""
  GNB_LOG=""
  READY_SEEN=0

  for i in $(seq 1 180); do
    # Prefer paths printed by this harness log.
    OTA_FROM_LOG="$(grep -aoE "$ROOT/runs/[0-9]{8}_[0-9]{6}_ota-stage" "$HARNESS_LOG" 2>/dev/null | tail -1 || true)"

    # Also detect newly-created ota-stage dirs, excluding pre-existing ones.
    find "$ROOT/runs" -maxdepth 1 -type d -name '*_ota-stage' 2>/dev/null | sort > "$NOW"
    OTA_NEW="$(comm -13 "$BEFORE" "$NOW" | tail -1 || true)"

    if [ -n "$OTA_FROM_LOG" ]; then
      OTA="$OTA_FROM_LOG"
    elif [ -n "$OTA_NEW" ]; then
      OTA="$OTA_NEW"
    fi

    if [ -n "$OTA" ]; then
      GNB_LOG="$OTA/gnb.log"
    fi

    if [ -n "$GNB_LOG" ] && [ -f "$GNB_LOG" ]; then
      GNB_MTIME="$(stat -c %Y "$GNB_LOG" 2>/dev/null || echo 0)"
      if [ "$GNB_MTIME" -ge "$START_EPOCH" ] && grep -q '==== gNB started ===' "$GNB_LOG"; then
        READY_SEEN=1
        TX_READY_UTC="$(date -u --iso-8601=ns)"
        CELL_LINE="$(grep -m1 'Cell pci=' "$GNB_LOG" || true)"

        cat > "$READY" <<EOR2
RF_TX_WINDOW_OPEN=1
RUN_CAPTURE_NOW=1
TX_READY_UTC=$TX_READY_UTC
OUT=$OUT
OTA=$OTA
GNB_LOG=$GNB_LOG
HARNESS_PID=$HPID
HARNESS_LOG=$HARNESS_LOG
DL_CENTER_HZ=1842500000
SSB_CENTER_HZ=1839650000
EXPECTED_PCI=1
RECOMMENDED_CAPTURE_FC=1839650000
RECOMMENDED_CAPTURE_FS=5760000
RECOMMENDED_CAPTURE_BW=5600000
RECOMMENDED_CAPTURE_N=11520000
EOR2

        echo
        echo "######################################################################"
        echo "RF_TX_WINDOW_OPEN=1"
        echo "RUN_CAPTURE_NOW=1"
        echo "TX_READY_UTC=$TX_READY_UTC"
        echo "TX_READY_FILE=$READY"
        echo "HARNESS_PID=$HPID"
        echo "OTA=$OTA"
        echo "GNB_LOG=$GNB_LOG"
        echo "$CELL_LINE"
        echo
        echo "Run the sensnuc7 SSB-centered capture now."
        echo "######################################################################"
        echo
        break
      fi
    fi

    if ! kill -0 "$HPID" 2>/dev/null; then
      echo "ERROR: harness exited before a fresh gNB started marker"
      break
    fi

    sleep 1
  done

  echo
  echo "===== verdict ====="
  echo "READY_SEEN=$READY_SEEN"
  echo "OUT=$OUT"
  echo "READY=$READY"
  echo "HARNESS_PID=$HPID"
  echo "HARNESS_LOG=$HARNESS_LOG"

  if [ "$READY_SEEN" -eq 1 ]; then
    echo "MARKED_TX_DETACHED_RC=0"
    exit 0
  fi

  echo "MARKED_TX_DETACHED_RC=1"
  exit 1
} 2>&1 | tee "$WATCH_LOG"

RC=${PIPESTATUS[0]}

{
  echo "===== compact detached marked TX summary ====="
  echo "OUT=$OUT"
  echo "WATCH_LOG=$WATCH_LOG"
  echo "HARNESS_LOG=$HARNESS_LOG"
  echo "READY=$READY"
  grep -Ei 'RF_TX_WINDOW_OPEN=|RUN_CAPTURE_NOW=|TX_READY_UTC=|TX_READY_FILE=|HARNESS_PID=|OTA=|GNB_LOG=|Cell pci=|dl_arfcn|dl_ssb_arfcn|==== gNB started|READY_SEEN=|MARKED_TX_DETACHED_RC=' "$WATCH_LOG" "$HARNESS_LOG" 2>/dev/null | sed -n '1,260p'
  echo "WRAPPER_RC=$RC"
} | tee "$SUMMARY"

echo
echo "OUT=$OUT"
echo "WATCH_LOG=$WATCH_LOG"
echo "HARNESS_LOG=$HARNESS_LOG"
echo "SUMMARY=$SUMMARY"
cat "$SUMMARY"

exit "$RC"
