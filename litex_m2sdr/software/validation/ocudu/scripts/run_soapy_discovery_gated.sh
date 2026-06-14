#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_ocudu_validation.sh
source "$SCRIPT_DIR/env_ocudu_validation.sh"
PROBE=0
[[ "${1:-}" == --probe ]] && PROBE=1
[[ $# -le 1 ]] || { echo "usage: $0 [--probe]" >&2; exit 2; }
RUN_DIR="$VALIDATION_RUN_ROOT/$(date +%Y%m%d_%H%M%S)_soapy-discovery"
mkdir -p "$RUN_DIR"

"$SCRIPT_DIR/check_m2sdr_clock_gate.sh" "$RUN_DIR/clk-test.log" | tee "$RUN_DIR/clock-gate.log"
timeout 30 SoapySDRUtil --info > "$RUN_DIR/soapy-info.log" 2>&1
timeout 30 SoapySDRUtil '--find=driver=LiteXM2SDR' > "$RUN_DIR/soapy-find.log" 2>&1
if ((PROBE)); then
    if timeout "${SOAPY_PROBE_TIMEOUT:-30}" SoapySDRUtil '--probe=driver=LiteXM2SDR' > "$RUN_DIR/soapy-probe.log" 2>&1; then
        echo "PASS: bounded Soapy probe"
    else
        echo "FAIL: bounded Soapy probe; see $RUN_DIR/soapy-probe.log" >&2
        exit 1
    fi
fi
cat > "$RUN_DIR/RESULT.md" <<EOF
# Gated Soapy discovery result

Date: $(date --iso-8601=seconds)
Host: $(hostname)
Run directory: $RUN_DIR
Clock gate: PASS
Soapy info: PASS
Soapy find: PASS
Probe requested: $PROBE
Result: PASS
EOF
echo "PASS: gated Soapy discovery written to $RUN_DIR"
