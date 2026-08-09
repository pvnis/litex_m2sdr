#!/usr/bin/env bash
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'EOF'
usage:
  ./pavonis-final.sh verify
  ./pavonis-final.sh prepare-private --workspace SOURCE_WORKSPACE
  ./pavonis-final.sh zmq --workspace SOURCE_WORKSPACE --stamp FRESH
  ./pavonis-final.sh materialize --site FILE --output DIR
  ./pavonis-final.sh install-ue-aux --site FILE
  ./pavonis-final.sh doctor --site FILE [--local-only]
  ./pavonis-final.sh deploy --site FILE --harness DIR
  ./pavonis-final.sh dry-run slow|fast --site FILE
  ./pavonis-final.sh run slow|fast --site FILE --workspace SOURCE_WORKSPACE \
    --rf-approved --stamp FRESH [--hold]
EOF
}

site=''
output=''
workspace=''
stamp=''
rf_approved=0
hold=0
local_only=0
command="${1:-}"
[[ -n "$command" ]] || { usage; exit 64; }
shift
profile=''
if [[ "$command" == run || "$command" == dry-run ]]; then
  profile="${1:-}"
  [[ "$profile" == slow || "$profile" == fast ]] || { usage; exit 64; }
  shift
fi
while (($#)); do
  case "$1" in
    --site) site="${2:?missing site file}"; shift 2 ;;
    --output|--harness) output="${2:?missing output path}"; shift 2 ;;
    --workspace) workspace="${2:?missing source workspace}"; shift 2 ;;
    --stamp) stamp="${2:?missing stamp}"; shift 2 ;;
    --rf-approved) rf_approved=1; shift ;;
    --hold) hold=1; shift ;;
    --local-only) local_only=1; shift ;;
    *) usage; exit 64 ;;
  esac
done

case "$command" in
  verify)
    exec python3 "$ROOT/tools/verify_package.py"
    ;;
  prepare-private)
    [[ -n "$workspace" ]] || { usage; exit 64; }
    exec python3 "$ROOT/tools/prepare_private.py" --workspace "$workspace"
    ;;
  zmq)
    [[ -n "$workspace" && "$stamp" =~ ^[A-Za-z0-9_.-]+$ ]] || { usage; exit 64; }
    run_root="$ROOT/work/zmq/$stamp"
    [[ ! -e "$run_root" ]] || { echo "fresh stamp required: $stamp" >&2; exit 73; }
    python3 "$ROOT/tools/verify_package.py"
    PAVONIS_STAGE_STAMP="${stamp}_ZMQ" \
      PAVONIS_STAGE_OUTPUT="$run_root" \
      exec "$ROOT/tools/run_zmq_gate.sh" "$workspace"
    ;;
  materialize)
    [[ -n "$site" && -n "$output" ]] || { usage; exit 64; }
    exec python3 "$ROOT/tools/materialize_harness.py" --site "$site" --output "$output"
    ;;
  install-ue-aux)
    [[ -n "$site" ]] || { usage; exit 64; }
    exec python3 "$ROOT/tools/install_ue_aux.py" \
      --site "$site" --log-dir "$ROOT/work/install-ue-aux"
    ;;
  doctor)
    [[ -n "$site" ]] || { usage; exit 64; }
    args=(--site "$site" --log-dir "$ROOT/work/doctor")
    ((local_only == 0)) || args+=(--local-only)
    exec python3 "$ROOT/tools/doctor.py" "${args[@]}"
    ;;
  deploy)
    [[ -n "$site" && -n "$output" ]] || { usage; exit 64; }
    exec python3 "$ROOT/tools/deploy_harness.py" --site "$site" --harness "$output" --log-dir "$ROOT/work/deploy"
    ;;
  dry-run)
    [[ -n "$site" ]] || { usage; exit 64; }
    run_root="$ROOT/work/dry-run-${profile}-$(date -u +%Y%m%dT%H%M%SZ)"
    harness="$run_root/harness"
    mkdir -p "$run_root"
    chmod 700 "$run_root"
    python3 "$ROOT/tools/materialize_harness.py" --site "$site" --output "$harness"
    export PAVONIS_SITE_CONFIG="$site" ART="$harness" STAMP="DRYRUN_$(date -u +%Y%m%dT%H%M%SZ)"
    set +e
    if [[ "$profile" == slow ]]; then
      "$harness/cp3121h_run_metric_al8_phone_rf.sh" >"$run_root/gate.log" 2>&1
    else
      "$harness/cp3612_phone_15mhz_all_mcs17_no_rf_gate_20260802/cp3612h_execute_phone_15mhz_all_mcs17_attempt.sh" >"$run_root/gate.log" 2>&1
    fi
    rc=$?
    set -e
    [[ "$rc" == 2 ]] || { echo "DRY_RUN_FAIL rc=$rc" >&2; exit 1; }
    echo "PAVONIS_RF_GATE_CLOSED=PASS profile=$profile rc=$rc"
    ;;
  run)
    [[ -n "$site" && -n "$workspace" && "$rf_approved" == 1 && "$stamp" =~ ^[A-Za-z0-9_.-]+$ ]] || { usage; exit 64; }
    run_root="$ROOT/work/runs/$stamp"
    [[ ! -e "$run_root" ]] || { echo "fresh stamp required: $stamp" >&2; exit 73; }
    mkdir -p "$run_root"
    chmod 700 "$run_root"
    harness="$run_root/harness"
    python3 "$ROOT/tools/verify_package.py"
    python3 "$ROOT/tools/materialize_harness.py" --site "$site" --output "$harness"
    PAVONIS_STAGE_STAMP="${stamp}_ZMQ" \
      PAVONIS_STAGE_OUTPUT="$run_root/stages/zmq" \
      "$ROOT/tools/run_zmq_gate.sh" "$workspace"
    python3 "$ROOT/tools/deploy_harness.py" --site "$site" --harness "$harness" --log-dir "$run_root/deploy"
    python3 "$ROOT/tools/doctor.py" --site "$site" --log-dir "$run_root/doctor"
    export PAVONIS_RF_APPROVED=1
    bounded_stamp="${stamp//[^A-Za-z0-9_]/_}_BOUNDED"
    python3 "$ROOT/tools/run_bounded_gate.py" \
      --site "$site" --harness "$harness" \
      --output-dir "$run_root/stages/bounded" --stamp "$bounded_stamp"
    phone_args=(
      "$profile" --site "$site" --harness "$harness"
      --output-dir "$run_root/stages/phone" --stamp "${stamp}_PHONE"
      --public-proof
    )
    ((hold == 0)) || phone_args+=(--hold)
    python3 "$ROOT/tools/run_phone_gate.py" "${phone_args[@]}"
    python3 "$ROOT/tools/write_staged_summary.py" \
      --stamp "$stamp" --profile "$profile" \
      --zmq "$run_root/stages/zmq/summary.json" \
      --bounded "$run_root/stages/bounded/summary.json" \
      --phone "$run_root/stages/phone/summary.json" \
      --output "$run_root/STAGED_SUMMARY.json"
    ;;
  *) usage; exit 64 ;;
esac
