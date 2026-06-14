#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_ocudu_validation.sh
source "$SCRIPT_DIR/env_ocudu_validation.sh"
TIMEOUT_SECONDS="${M2SDR_CLOCK_GATE_TIMEOUT:-20}"
LOG="${1:-$(mktemp /tmp/m2sdr-clock-gate.XXXXXX.log)}"

[[ -x "$M2SDR_UTIL" ]] || { echo "FAIL: missing executable $M2SDR_UTIL" >&2; exit 1; }
if ! timeout "$TIMEOUT_SECONDS" "$M2SDR_UTIL" clk-test > "$LOG" 2>&1; then
    echo "FAIL: bounded clk-test failed; see $LOG" >&2
    exit 1
fi
for clock in "Sys Clk" "PCIe Clk" "AD9361 Ref Clk" "AD9361 Dat Clk" "Time Ref Clk"; do
    grep -Fq "$clock" "$LOG" || { echo "FAIL: missing $clock in $LOG" >&2; exit 1; }
done
if ! awk '
    /^[[:space:]]*[0-9]+[[:space:]]/ {
        measurements++
        if ($5 !~ /^[0-9]+([.][0-9]+)?$/ || ($5 + 0) <= 0) {
            exit 1
        }
    }
    END {
        if (measurements == 0) {
            exit 1
        }
    }
' "$LOG"; then
    echo "FAIL: AD9361 Dat Clk is zero or unparsable; see $LOG" >&2
    exit 1
fi
echo "PASS: required M2SDR clocks reported ($LOG)"
