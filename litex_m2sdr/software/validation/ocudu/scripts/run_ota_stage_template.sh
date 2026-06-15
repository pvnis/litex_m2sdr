#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_ocudu_validation.sh
source "$SCRIPT_DIR/env_ocudu_validation.sh"

ACK=0; START_UE=0; UE_CONF=""; GNB_CONF=""; SIM_FILE=""
EXTERNAL_IFACE="${OCUDU_VALIDATION_EXTERNAL_IFACE:-wlp5s0}"
DURATION="${OCUDU_VALIDATION_OTA_DURATION:-60}"
usage() {
    echo "usage: $0 --i-understand-rf-test --gnb-conf RUN_COPY.yml --sim-file PRIVATE_TOML --ue-conf PRIVATE_CONF [--start-ue] [--duration SEC] [--external-iface IFACE]" >&2
}
while (($#)); do
    case "$1" in
        --i-understand-rf-test) ACK=1; shift ;;
        --start-ue) START_UE=1; shift ;;
        --ue-conf) UE_CONF="${2:?missing path}"; shift 2 ;;
        --gnb-conf) GNB_CONF="${2:?missing path}"; shift 2 ;;
        --sim-file) SIM_FILE="${2:?missing path}"; shift 2 ;;
        --duration) DURATION="${2:?missing seconds}"; shift 2 ;;
        --external-iface) EXTERNAL_IFACE="${2:?missing interface}"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) usage; exit 2 ;;
    esac
done

((ACK)) || { echo "Refusing OTA stage without --i-understand-rf-test." >&2; exit 2; }
[[ "$DURATION" =~ ^[0-9]+$ ]] && ((DURATION > 0 && DURATION <= 600)) || {
    echo "OTA duration must be an integer from 1 to 600 seconds." >&2; exit 2;
}
[[ -f "$UE_CONF" ]] || { echo "A private UE config is required via --ue-conf." >&2; exit 2; }
[[ -f "$SIM_FILE" ]] || { echo "A private qcore SIM file is required via --sim-file." >&2; exit 2; }
[[ -f "$GNB_CONF" ]] || { echo "A user-provided hardware gNB config is required via --gnb-conf." >&2; exit 2; }
[[ "$(readlink -f "$GNB_CONF")" != "$(readlink -f "$VALIDATION_ROOT/configs/gnb_m2sdr_band3_ota.template.yml")" ]] || {
    echo "Copy the committed gNB template to a private/run config before use." >&2; exit 2;
}
for binary in "$QCORE_BIN" "$OCUDU_GNB_BIN"; do
    [[ -x "$binary" ]] || { echo "Missing required executable: $binary" >&2; exit 1; }
done
((START_UE == 0)) || [[ -x "$SRSUE_BIN" ]] || { echo "Missing required executable: $SRSUE_BIN" >&2; exit 1; }

RUN_DIR="$VALIDATION_RUN_ROOT/$(date +%Y%m%d_%H%M%S)_ota-stage"
mkdir -p "$RUN_DIR"
touch "$RUN_DIR"/{qcore.log,gnb.log,srsue.log,setup.log,pids.txt}
cp "$GNB_CONF" "$RUN_DIR/gnb-run.yml"
{
    printf 'gNB config sha256: %s\n' "$(sha256sum "$GNB_CONF" | awk '{print $1}')"
    printf 'UE config sha256: %s\n' "$(sha256sum "$UE_CONF" | awk '{print $1}')"
    printf 'qcore SIM file sha256: %s\n' "$(sha256sum "$SIM_FILE" | awk '{print $1}')"
} > "$RUN_DIR/commands-and-checksums.txt"
NETWORK_SETUP_STARTED=0

cleanup() {
    set +e
    for pid in ${SRSUE_PID:-} ${GNB_PID:-} ${QCORE_PID:-}; do
        [[ -n "${pid:-}" ]] || continue
        sudo -n kill -TERM -- "-$pid" 2>/dev/null || kill -TERM -- "-$pid" 2>/dev/null || true
    done
    if [[ "${NETWORK_SETUP_STARTED:-0}" == "1" || -n "${QCORE_PID:-}" || -n "${GNB_PID:-}" || -n "${SRSUE_PID:-}" ]]; then
        "$SCRIPT_DIR/cleanup_zmq_ocudu.sh" --external-iface "$EXTERNAL_IFACE" >> "$RUN_DIR/setup.log" 2>&1 || true
    else
        echo "Skipping qcore network cleanup: qcore/routing/gNB startup was not reached." >> "$RUN_DIR/setup.log" 2>&1 || true
    fi
    if [[ ! -f "$RUN_DIR/RESULT.md" ]]; then
        cat > "$RUN_DIR/RESULT.md" <<EOF
# OTA stage result

Date: $(date --iso-8601=seconds)
Host: $(hostname)
Run directory: $RUN_DIR
Result: FAIL
First failed stage: preflight or bounded stage execution
EOF
    fi
}
trap cleanup EXIT INT TERM

"$SCRIPT_DIR/check_m2sdr_clock_gate.sh" "$RUN_DIR/clk-test.log" | tee "$RUN_DIR/clock-gate.log"
"$SCRIPT_DIR/collect_host_state.sh" "$RUN_DIR" > "$RUN_DIR/collect-host-state.log" 2>&1

# Non-interactive: the caller must prime sudo before detached launch.
sudo -n true
NETWORK_SETUP_STARTED=1
{
    echo "===== pre-clean qcore/OCUDU network state ====="
    date --iso-8601=seconds
} >> "$RUN_DIR/setup.log" 2>&1
if ! "$SCRIPT_DIR/cleanup_zmq_ocudu.sh" --external-iface "$EXTERNAL_IFACE" >> "$RUN_DIR/setup.log" 2>&1; then
    echo "WARN: cleanup_zmq_ocudu.sh returned nonzero during pre-clean; continuing to setup-routing." >> "$RUN_DIR/setup.log" 2>&1
fi
{
    echo "===== qcore setup-routing ====="
    date --iso-8601=seconds
} >> "$RUN_DIR/setup.log" 2>&1
(cd "$QCORE_REPO" && sudo -n ./setup-routing "$EXTERNAL_IFACE") >> "$RUN_DIR/setup.log" 2>&1

setsid sudo -n env RUST_LOG=info "$QCORE_BIN" --mcc 001 --mnc 01 \
    --sim-cred-file "$SIM_FILE" --local-ip 127.0.0.1 --no-dhcp \
    < /dev/null > "$RUN_DIR/qcore.log" 2>&1 &
QCORE_PID=$!; echo "qcore $QCORE_PID" >> "$RUN_DIR/pids.txt"
sleep 3
setsid "$OCUDU_GNB_BIN" -c "$RUN_DIR/gnb-run.yml" < /dev/null > "$RUN_DIR/gnb.log" 2>&1 &
GNB_PID=$!; echo "gnb $GNB_PID" >> "$RUN_DIR/pids.txt"
sleep 5
if ((START_UE)); then
    setsid sudo -n "$SRSUE_BIN" "$UE_CONF" < /dev/null > "$RUN_DIR/srsue.log" 2>&1 &
    SRSUE_PID=$!; echo "srsue $SRSUE_PID" >> "$RUN_DIR/pids.txt"
fi

echo "Independently prove downlink SSB before debugging PRACH."
sleep "$DURATION"
highest="Stage C: OTA gNB"
((START_UE)) && highest="Stage E: srsUE started"
cat > "$RUN_DIR/RESULT.md" <<EOF
# OTA stage result

Date: $(date --iso-8601=seconds)
Host: $(hostname)
Run directory: $RUN_DIR
Bounded duration: $DURATION seconds
UE automatically started: $START_UE
Highest attempted stage: $highest
gNB config sha256: $(sha256sum "$GNB_CONF" | awk '{print $1}')
UE config sha256: $(sha256sum "$UE_CONF" | awk '{print $1}')
qcore SIM file sha256: $(sha256sum "$SIM_FILE" | awk '{print $1}')

Independently prove downlink SSB before debugging PRACH. Record the result and
the first failed ladder stage before changing another variable.
EOF
echo "Bounded OTA stage complete: $RUN_DIR"
