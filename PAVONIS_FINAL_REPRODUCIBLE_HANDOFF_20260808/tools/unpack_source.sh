#!/usr/bin/env bash
set -euo pipefail

DESTINATION="${1:?usage: unpack_source.sh EMPTY_DESTINATION}"
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
[[ ! -e "$DESTINATION" ]] || { echo "destination exists: $DESTINATION" >&2; exit 73; }
mkdir -p "$DESTINATION"
DESTINATION="$(cd "$DESTINATION" && pwd)"

while read -r repo branch expected; do
  git clone --quiet --branch "$branch" --single-branch \
    "$ROOT/source/bundles/$repo.bundle" "$DESTINATION/$repo"
  actual="$(git -C "$DESTINATION/$repo" rev-parse HEAD)"
  [[ "$actual" == "$expected" ]] || {
    echo "$repo checkout mismatch: $actual" >&2
    exit 1
  }
done <<'EOF'
litex_m2sdr pavonis/supervisor-snapshot-litex-m2sdr b4d3dbae113ed8a6a33fac177dab0d85381677d6
ocudu pavonis/supervisor-snapshot-ocudu d12a7f10ec4edfb57364e7be31343da34fafb81d
qcore pavonis/supervisor-snapshot-qcore d84e2f433c8ff9f5fb10e6fcc05d25b6c15a7c67
srsRAN_4G pavonis/supervisor-snapshot-srsran4g d3a772af794650cd2544a88ee0781f7af0bb219a
EOF

cp -a "$ROOT/source/overlays/ocudu-cp3406/lib/." "$DESTINATION/ocudu/lib/"
cp -a "$ROOT/source/overlays/ocudu-cp3406/tests/." "$DESTINATION/ocudu/tests/"
[[ "$(sha256sum "$DESTINATION/ocudu/lib/radio/soapy/radio_soapy_tx_stream.cpp" | awk '{print $1}')" == \
   50fee9094caedf2990faf0ac3771b9ca0ae9964d1d8904e72fbd336e2c16a0e7 ]]
[[ "$(sha256sum "$DESTINATION/ocudu/lib/radio/soapy/radio_soapy_tx_deadline.h" | awk '{print $1}')" == \
   79345305fc5bca4b3437e9453db32acf03da8b4a69478899fda89580e67feaf8 ]]

cat >"$DESTINATION/SOURCE_REVISIONS" <<EOF
litex_m2sdr $(git -C "$DESTINATION/litex_m2sdr" rev-parse HEAD)
ocudu $(git -C "$DESTINATION/ocudu" rev-parse HEAD)
qcore $(git -C "$DESTINATION/qcore" rev-parse HEAD)
srsRAN_4G $(git -C "$DESTINATION/srsRAN_4G" rev-parse HEAD)
EOF
echo "PAVONIS_SOURCE_UNPACK=PASS destination=$DESTINATION"
