#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env_ocudu_validation.sh
source "$SCRIPT_DIR/env_ocudu_validation.sh"
RUN_DIR="${1:-$VALIDATION_RUN_ROOT/$(date +%Y%m%d_%H%M%S)_host-state}"
mkdir -p "$RUN_DIR/host-state"
OUT="$RUN_DIR/host-state"

capture() {
    local file="$1"; shift
    { echo "\$ $*"; "$@"; } > "$OUT/$file" 2>&1 || true
}
capture hostname.txt hostname
capture date.txt date --iso-8601=seconds
capture uptime.txt uptime
capture uname.txt uname -a
capture processes.txt ps auxww
capture sockets.txt ss -lntup
capture netns.txt ip netns list
capture links.txt ip -details link show
capture addresses.txt ip address show
capture iptables.txt sudo -n iptables-save
capture soapy-info.txt timeout 30 SoapySDRUtil --info
capture soapy-find.txt timeout 30 SoapySDRUtil '--find=driver=LiteXM2SDR'
command -v bladeRF-cli >/dev/null && capture bladerf-info.txt timeout 30 bladeRF-cli -e 'version; info'

for entry in "litex_m2sdr:$M2SDR_REPO" "ocudu:$OCUDU_REPO" "srsRAN_4G:$SRSRAN_4G_REPO" "qcore:$QCORE_REPO"; do
    name="${entry%%:*}"; repo="${entry#*:}"
    {
        git -C "$repo" rev-parse HEAD
        git -C "$repo" status --short --branch
    } > "$OUT/repo-$name.txt" 2>&1 || true
done
echo "Host state written to $OUT"
