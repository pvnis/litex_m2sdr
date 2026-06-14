#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
required=(
    README.md HANDOFF.md
    configs/gnb_zmq_tdd_n78_20mhz.yml configs/ue-zmq-netns.template.conf
    configs/sims-ocudu-zmq.example.toml configs/gnb_m2sdr_band3_ota.template.yml
    configs/ue-bladerf-band3-ota.template.conf configs/cell_equivalence_checklist.md
    templates/RESULT_TEMPLATE.md templates/HARDWARE_RESULT_TEMPLATE.md
)
for file in "${required[@]}"; do
    [[ -f "$ROOT/$file" ]] || { echo "FAIL: missing $file" >&2; exit 1; }
done
for script in "$SCRIPT_DIR"/*.sh; do
    [[ -x "$script" ]] || { echo "FAIL: not executable $script" >&2; exit 1; }
done
for forbidden in 'rm''mod' 'modprobe ''-r' 'ins''mod' 'git clean ''-fdx'; do
if grep -RInF "$forbidden" "$ROOT/scripts"; then
    echo "FAIL: forbidden command found" >&2
    exit 1
fi
done
if grep -En 'set -[^[:space:]]*[eu]|pipefail' "$SCRIPT_DIR/env_ocudu_validation.sh"; then
    echo "FAIL: sourceable environment enables strict mode" >&2
    exit 1
fi
grep -q '<SECRET_OPC>' "$ROOT/configs/ue-zmq-netns.template.conf"
grep -q '<PRIVATE_IMEI>' "$ROOT/configs/ue-bladerf-band3-ota.template.conf"

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
touch "$tmp"/{qcore.log,gnb.log,srsue.log,checks.txt,dataplane.txt}
printf 'started\nNGAP setup response\n' > "$tmp/qcore.log"
printf 'gNB started\n' > "$tmp/gnb.log"
printf 'RF opened\nRandom Access Complete\nRRC Connected\nPDU Session Establishment successful\n' > "$tmp/srsue.log"
printf 'inet 10.255.0.2/24 scope global tun_ue1\n' > "$tmp/checks.txt"
printf '5 packets transmitted, 5 received, 0%% packet loss\n' > "$tmp/dataplane.txt"
"$SCRIPT_DIR/summarize_zmq_run.sh" "$tmp" >/dev/null
grep -q '"result":"PASS"' "$tmp/summary.json"

check_clock_fixture() {
    local name="$1"
    local expected="$2"
    local fixture="$tmp/$name"
    shift 2
    {
        echo '#!/usr/bin/env bash'
        printf "printf '%%s\\\\n'"
        printf " %q" "$@"
        echo
    } > "$fixture"
    chmod +x "$fixture"
    if M2SDR_UTIL="$fixture" "$SCRIPT_DIR/check_m2sdr_clock_gate.sh" "$tmp/$name.log" > "$tmp/$name-result.log" 2>&1; then
        actual=PASS
    else
        actual=FAIL
    fi
    [[ "$actual" == "$expected" ]] || {
        echo "FAIL: $name clock fixture expected $expected, got $actual" >&2
        cat "$tmp/$name-result.log" >&2
        exit 1
    }
}
table_header='Meas. Sys Clk PCIe Clk AD9361 Ref Clk AD9361 Dat Clk Time Ref Clk'
check_clock_fixture named-line-non-zero PASS \
    'Sys Clk 125.00' 'PCIe Clk 124.71' 'AD9361 Ref Clk 38.40' \
    'AD9361 Dat Clk 122.88' 'Time Ref Clk 100.00'
check_clock_fixture named-line-zero FAIL \
    'Sys Clk 125.00' 'PCIe Clk 124.71' 'AD9361 Ref Clk 38.40' \
    'AD9361 Dat Clk 0.00' 'Time Ref Clk 100.00'
check_clock_fixture table-non-zero PASS \
    "$table_header" '1 100.00 125.00 40.00 122.88 100.00'
check_clock_fixture table-zero FAIL \
    "$table_header" '1 100.00 125.00 40.00 0.00 100.00'

if "$SCRIPT_DIR/run_zmq_ocudu_dataplane.sh" > "$tmp/refusal.log" 2>&1; then
    echo "FAIL: ZMQ runner accepted missing private configs" >&2
    exit 1
fi
grep -q 'private, untracked UE config' "$tmp/refusal.log"
echo "PASS: validation harness self-check"
