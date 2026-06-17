#!/usr/bin/env bash
set -u

VALIDATION_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$VALIDATION_ROOT/scripts/env_ocudu_validation.sh"
source "$HOME/CLionProjects/env_m2sdr_current.sh" >/dev/null 2>&1

RUN_ROOT="${VALIDATION_RUN_ROOT:-$VALIDATION_ROOT/runs}"
OUT="$RUN_ROOT/$(date -u +%Y%m%dT%H%M%SZ)_soapy_rx_smoke_logged"
mkdir -p "$OUT"

LOG="$OUT/run.log"
SUMMARY="$OUT/SUMMARY.txt"

write_summary() {
  {
    echo "OUT=$OUT"
    grep -m1 '^python_used=' "$LOG" || echo "python_used=missing"
    grep -m1 '^precheck_result=' "$LOG" || echo "precheck_result=missing"
    grep -m1 '^record_exit=' "$LOG" || echo "record_exit=missing"
    grep -m1 '^bytes=' "$LOG" || echo "bytes=missing"
    echo -n "kernel_red_flags="
    awk '/===== kernel warnings\/errors =====/{k=1; next} /===== end =====/{k=0} k {print}' "$LOG" | grep -qiE 'bad page|BUG|oops|panic|dma|timeout|error|segfault|blocked|hung' && echo "YES" || echo "NO"
    echo
    echo "===== last 120 log lines ====="
    tail -120 "$LOG" 2>/dev/null || true
  } > "$SUMMARY"
}

trap write_summary EXIT

{
  echo "===== start ====="
  date -u
  uname -a
  echo "OUT=$OUT"

  echo
  echo "===== env ====="
  echo "VALIDATION_ROOT=$VALIDATION_ROOT"
  echo "M2SDR_ROOT=${M2SDR_ROOT:-unset}"
  echo "M2SDR_SW=${M2SDR_SW:-unset}"
  echo "SOAPY_SDR_ROOT=${SOAPY_SDR_ROOT:-unset}"
  echo "SOAPY_SDR_PLUGIN_PATH=${SOAPY_SDR_PLUGIN_PATH:-unset}"
  echo "LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-unset}"
  echo "M2SDR_DIRECT_DEINTERLEAVE=${M2SDR_DIRECT_DEINTERLEAVE:-unset}"

  echo
  echo "===== find Python with SoapySDR binding ====="
  PY_FOUND=""
  for py in \
    "$HOME/venvs/litex-m2sdr/bin/python3" \
    "$HOME/venvs/litex-m2sdr/bin/python" \
    "$(command -v python3 2>/dev/null)" \
    "/usr/bin/python3"
  do
    [ -n "$py" ] || continue
    [ -x "$py" ] || continue
    echo "checking $py"
    if "$py" - <<'PY'
import SoapySDR
print("OK SoapySDR", SoapySDR)
PY
    then
      PY_FOUND="$py"
      break
    fi
  done

  if [ -z "$PY_FOUND" ]; then
    echo "python_used=none"
    echo "precheck_result=FAIL_NO_PYTHON_SOAPYSDR_BINDING"
    echo "No usable Python SoapySDR binding found. RX smoke not run."
    echo
    echo "===== possible package/module hints ====="
    python3 -V 2>&1 || true
    python3 -c 'import sys; print(sys.path)' 2>&1 || true
    echo
    echo "===== end ====="
    date -u
    exit 20
  fi

  echo "python_used=$PY_FOUND"
  echo "precheck_result=PASS"

  echo
  echo "===== Soapy find/probe precheck ====="
  timeout -k 2s 20s SoapySDRUtil --find="driver=LiteXM2SDR"
  find_rc=$?
  echo "soapy_find_exit=$find_rc"

  timeout -k 2s 30s SoapySDRUtil --probe="driver=LiteXM2SDR"
  probe_rc=$?
  echo "soapy_probe_exit=$probe_rc"

  if [ "$find_rc" -ne 0 ] || [ "$probe_rc" -ne 0 ]; then
    echo "record_exit=not_run_probe_failed"
    echo "bytes=missing"
    exit 21
  fi

  echo
  echo "===== Soapy RX-only 200 ms smoke ====="
  cd "$M2SDR_SW" || exit 22

  timeout -k 2s 8s "$PY_FOUND" soapysdr/test_record.py \
    --samplerate 11520000 \
    --bandwidth 10000000 \
    --freq 1842500000 \
    --gain 20 \
    --channel 0 \
    --secs 0.2 \
    "$OUT/rx_200ms.cf32"
  rc=$?
  echo "record_exit=$rc"

  echo
  echo "===== output file ====="
  ls -lh "$OUT/rx_200ms.cf32" 2>&1 || true
  stat -c 'bytes=%s' "$OUT/rx_200ms.cf32" 2>/dev/null || echo "bytes=missing"

  echo
  echo "===== kernel warnings/errors ====="
  journalctl -k -b --since '5 minutes ago' --no-pager | \
    grep -iE 'm2sdr|litepcie|bad page|BUG|oops|panic|dma|timeout|error|fail|segfault|blocked|hung' || true

  echo
  echo "===== end ====="
  date -u
} > "$LOG" 2>&1

echo "$OUT"
