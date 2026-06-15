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
    /^[[:space:]]*[0-9]+[[:space:]]/ {
        measurements++

        # Columns:
        # 1 measurement index
        # 2 Sys Clk
        # 3 PCIe Clk
        # 4 AD9361 Ref Clk
        # 5 AD9361 Dat Clk
        # 6 Time Ref Clk
        #
        # AD9361 Dat Clk may be 0.00 before the RF/data path is initialized.
        # Do not block pre-gNB startup on it. Runtime streaming checks catch
        # data-path failures later.
        if (NF < 6 ||
            !valid_positive($2) ||
            !valid_positive($3) ||
            !valid_positive($4) ||
            !valid_positive($6)) {
            invalid = 1
        }
    }
    END {
        if (measurements == 0 || invalid) {
            exit 1
        }
    }
' "$LOG"; then
    echo "FAIL: required non-data M2SDR clocks are zero or unparsable; see $LOG" >&2
    exit 1
fi

dat_clk="$(awk "/^[[:space:]]*[0-9]+[[:space:]]/ {print \$5; exit}" "$LOG")"
echo "PASS: required M2SDR clocks reported; AD9361 Dat Clk pre-init=${dat_clk:-unknown} MHz ($LOG)"
