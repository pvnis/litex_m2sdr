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
    function valid_positive(value) {
        return value ~ /^[0-9]+([.][0-9]+)?$/ && (value + 0) > 0
    }
    $1 == "AD9361" && $2 == "Dat" && $3 == "Clk" {
        measurements++
        if (NF != 4 || !valid_positive($4)) {
            invalid = 1
        }
        next
    }
    /^[[:space:]]*[0-9]+[[:space:]]/ {
        measurements++
        if (NF < 6 || !valid_positive($5)) {
            invalid = 1
        }
    }
    END {
        if (measurements == 0 || invalid) {
            exit 1
        }
    }
' "$LOG"; then
    echo "FAIL: AD9361 Dat Clk is zero or unparsable; see $LOG" >&2
    exit 1
fi
echo "PASS: required M2SDR clocks reported ($LOG)"
