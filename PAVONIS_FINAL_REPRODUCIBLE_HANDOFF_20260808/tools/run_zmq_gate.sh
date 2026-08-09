#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${1:?Usage: run_zmq_gate.sh WORKSPACE}"
WORKSPACE="$(cd "$WORKSPACE" && pwd)"
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VALIDATION_ROOT="$WORKSPACE/litex_m2sdr/litex_m2sdr/software/validation/ocudu"
LOCAL_DIR="$WORKSPACE/pavonis-local"
RUNNER="$VALIDATION_ROOT/scripts/run_zmq_ocudu_dataplane.sh"

for path in \
  "$RUNNER" \
  "$LOCAL_DIR/ue-zmq.local.conf" \
  "$LOCAL_DIR/sims-zmq.local.toml" \
  "$PACKAGE_DIR/config/zmq/gnb_zmq_tdd_n78_20mhz.yml"; do
  [[ -e "$path" ]] || {
    echo "ERROR: missing ZMQ input: $path" >&2
    exit 1
  }
done

if grep -q 'REPLACE_WITH_' \
  "$LOCAL_DIR/ue-zmq.local.conf" "$LOCAL_DIR/sims-zmq.local.toml"; then
  echo "ERROR: private ZMQ templates still contain placeholders" >&2
  exit 1
fi

sudo -n true 2>/dev/null || {
  echo "ERROR: prime sudo with 'sudo -v' before running the ZMQ gate" >&2
  exit 1
}

export WORKSPACE_ROOT="$WORKSPACE"
export VALIDATION_ROOT
export VALIDATION_RUN_ROOT="$WORKSPACE/pavonis-runs/zmq"
# shellcheck disable=SC1090
source "$VALIDATION_ROOT/scripts/env_ocudu_validation.sh"

external_iface="${PAVONIS_EXTERNAL_IFACE:-$(ip route show default | awk 'NR==1 {print $5}')}"
core_gateway="${PAVONIS_CORE_GATEWAY:-$(awk '/^sudo ip addr add .*\\/24 dev veth2$/ {sub(/\\/.*/, "", $5); print $5; exit}' "$WORKSPACE/qcore/setup-routing")}"
[[ -n "$external_iface" && -n "$core_gateway" ]] || {
  echo "ERROR: could not derive external interface or private core gateway" >&2
  exit 1
}

stamp="${PAVONIS_STAGE_STAMP:-$(date -u +%Y%m%dT%H%M%SZ)}"
[[ "$stamp" =~ ^[A-Za-z0-9_.-]+$ ]] || {
  echo "ERROR: invalid ZMQ stage stamp" >&2
  exit 64
}
gate_dir="${PAVONIS_STAGE_OUTPUT:-$WORKSPACE/pavonis-gates/$stamp-zmq}"
[[ ! -e "$gate_dir" ]] || {
  echo "ERROR: ZMQ gate output already exists: $gate_dir" >&2
  exit 73
}
mkdir -p "$gate_dir"
controller_log="$gate_dir/controller.log"

set +e
"$RUNNER" \
  --ue-conf "$LOCAL_DIR/ue-zmq.local.conf" \
  --sim-file "$LOCAL_DIR/sims-zmq.local.toml" \
  --gnb-conf "$PACKAGE_DIR/config/zmq/gnb_zmq_tdd_n78_20mhz.yml" \
  --external-iface "$external_iface" \
  --ping-target "$core_gateway" \
  --attach-timeout 120 \
  --allowed-loss 0 \
  --tag "supervisor-$stamp" >"$controller_log" 2>&1
runner_rc=$?
set -e

cat "$controller_log"
run_dir="$(awk -F': ' '/^Run directory:/ {value=$2} END {print value}' "$controller_log")"
[[ -n "$run_dir" && -f "$run_dir/summary.json" ]] || {
  echo "ERROR: ZMQ runner did not produce summary.json" >&2
  exit 1
}
cp "$run_dir/summary.json" "$gate_dir/summary.json"
sha256sum "$gate_dir/summary.json" >"$gate_dir/summary.sha256"

python3 - "$gate_dir/summary.json" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as handle:
    summary = json.load(handle)
if summary.get("result") != "PASS":
    raise SystemExit("ZMQ result is not PASS")
if summary.get("highest_milestone") != "PING_OK":
    raise SystemExit("ZMQ highest milestone is not PING_OK")
if summary.get("first_failed_milestone") != "NONE":
    raise SystemExit("ZMQ reports a failed milestone")
if summary.get("milestones", {}).get("ping_ok") != 1:
    raise SystemExit("ZMQ ping gate is not set")
PY

[[ "$runner_rc" -eq 0 ]] || {
  echo "ERROR: ZMQ markers passed but runner exited $runner_rc; inspect $controller_log" >&2
  exit 1
}

echo "ZMQ gate: $gate_dir"
echo "PAVONIS_ZMQ_SUMMARY=$gate_dir/summary.json"
echo "PAVONIS_SUPERVISOR_ZMQ=PASS"
