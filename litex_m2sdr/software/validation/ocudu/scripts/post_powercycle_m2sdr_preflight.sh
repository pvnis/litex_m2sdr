#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_ocudu_validation.sh
source "$SCRIPT_DIR/env_ocudu_validation.sh"
RUN_DIR="${1:-$VALIDATION_RUN_ROOT/$(date +%Y%m%d_%H%M%S)_m2sdr-preflight}"
mkdir -p "$RUN_DIR"

"$SCRIPT_DIR/collect_host_state.sh" "$RUN_DIR" > "$RUN_DIR/collect-host-state.log" 2>&1
lspci -nn > "$RUN_DIR/lspci.txt" 2>&1 || true
ls -l /dev/m2sdr* > "$RUN_DIR/devices.txt" 2>&1 || true
lsmod | grep '^m2sdr' > "$RUN_DIR/module.txt" 2>&1 || true

device=FAIL; scratch=FAIL; clock=FAIL; soapy=FAIL
compgen -G '/dev/m2sdr*' >/dev/null && device=PASS
timeout 30 "$M2SDR_UTIL" scratch-test > "$RUN_DIR/scratch-test.log" 2>&1 && scratch=PASS
"$SCRIPT_DIR/check_m2sdr_clock_gate.sh" "$RUN_DIR/clk-test.log" > "$RUN_DIR/clock-gate.log" 2>&1 && clock=PASS
timeout 30 SoapySDRUtil --info > "$RUN_DIR/soapy-info.log" 2>&1 || true
timeout 30 SoapySDRUtil '--find=driver=LiteXM2SDR' > "$RUN_DIR/soapy-find.log" 2>&1 || true
grep -q 'LiteXM2SDR' "$RUN_DIR/soapy-find.log" && soapy=PASS
command -v bladeRF-cli >/dev/null && timeout 30 bladeRF-cli -e 'version; info' > "$RUN_DIR/bladerf-info.log" 2>&1 || true

result=FAIL
[[ "$device$scratch$clock$soapy" == PASSPASSPASSPASS ]] && result=PASS
cat > "$RUN_DIR/RESULT.md" <<EOF
# M2SDR post-power-cycle preflight

Date: $(date --iso-8601=seconds)
Host: $(hostname)
Run directory: $RUN_DIR

- device present: $device
- scratch-test passes: $scratch
- clock test completes with required values: $clock
- Soapy find sees LiteXM2SDR: $soapy

Result: $result
EOF
echo "$result: preflight results written to $RUN_DIR"
[[ "$result" == PASS ]]
