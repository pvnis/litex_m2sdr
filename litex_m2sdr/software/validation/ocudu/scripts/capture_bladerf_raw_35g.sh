#!/usr/bin/env bash
set -euo pipefail

OUT="${1:-/tmp/bladerf_35g_raw.sc16}"
N="${2:-4608000}"             # 0.4 s at 11.52 Msps
CENTER_HZ="${3:-3500000000}"
RX_GAIN="${4:-40}"

SCRIPT="$(mktemp --suffix=.bladerf)"
trap 'rm -f "$SCRIPT"' EXIT

cat > "$SCRIPT" <<EOS
set frequency rx $CENTER_HZ
set samplerate rx 11520000
set bandwidth rx 10000000
set agc rx off
set gain rx $RX_GAIN
rx config file=$OUT format=bin n=$N samples=16384 buffers=32 xfers=16 timeout=10s channel=1
rx start
rx wait 15s
EOS

echo "OUT=$OUT"
echo "N=$N"
echo "CENTER_HZ=$CENTER_HZ"
echo "RX_GAIN=$RX_GAIN"
sudo -n bladeRF-cli -s "$SCRIPT"
ls -lh "$OUT"
