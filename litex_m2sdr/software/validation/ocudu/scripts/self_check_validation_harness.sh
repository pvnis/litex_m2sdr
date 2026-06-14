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

cat > "$tmp/fake-m2sdr-util" <<'EOF'
#!/usr/bin/env bash
cat <<'CLOCKS'
Meas.     Sys Clk          PCIe Clk         AD9361 Ref Clk   AD9361 Dat Clk   Time Ref Clk (MHz)
1                  100.00           125.00            40.00             0.00           100.00
CLOCKS
EOF
chmod +x "$tmp/fake-m2sdr-util"
if M2SDR_UTIL="$tmp/fake-m2sdr-util" "$SCRIPT_DIR/check_m2sdr_clock_gate.sh" "$tmp/zero-clock.log" > "$tmp/zero-clock-result.log" 2>&1; then
    echo "FAIL: clock gate accepted zero AD9361 Dat Clk" >&2
    exit 1
fi
grep -q 'AD9361 Dat Clk is zero or unparsable' "$tmp/zero-clock-result.log"

if "$SCRIPT_DIR/run_zmq_ocudu_dataplane.sh" > "$tmp/refusal.log" 2>&1; then
    echo "FAIL: ZMQ runner accepted missing private configs" >&2
    exit 1
fi
grep -q 'private, untracked UE config' "$tmp/refusal.log"
echo "PASS: validation harness self-check"
