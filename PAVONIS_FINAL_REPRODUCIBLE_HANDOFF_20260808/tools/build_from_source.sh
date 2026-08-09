#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${1:?usage: build_from_source.sh SOURCE_WORKSPACE}"
WORKSPACE="$(cd "$WORKSPACE" && pwd)"
JOBS="${PAVONIS_BUILD_JOBS:-$(nproc)}"
[[ "$JOBS" =~ ^[0-9]+$ ]] && ((JOBS > 0))

for command in cargo cmake git sha256sum; do
  command -v "$command" >/dev/null || { echo "missing build command: $command" >&2; exit 69; }
done
if command -v ninja >/dev/null; then
  CMAKE_GENERATOR=Ninja
else
  command -v make >/dev/null || { echo "missing build command: ninja or make" >&2; exit 69; }
  CMAKE_GENERATOR='Unix Makefiles'
fi

while read -r repo expected; do
  actual="$(git -C "$WORKSPACE/$repo" rev-parse HEAD)"
  [[ "$actual" == "$expected" ]] || {
    echo "$repo revision mismatch: $actual" >&2
    exit 1
  }
done <<'EOF'
litex_m2sdr b4d3dbae113ed8a6a33fac177dab0d85381677d6
ocudu d12a7f10ec4edfb57364e7be31343da34fafb81d
qcore d84e2f433c8ff9f5fb10e6fcc05d25b6c15a7c67
srsRAN_4G d3a772af794650cd2544a88ee0781f7af0bb219a
EOF

cargo clippy --manifest-path "$WORKSPACE/qcore/Cargo.toml" --all-targets -- -D warnings
cargo build --release --locked --manifest-path "$WORKSPACE/qcore/Cargo.toml"

cmake -S "$WORKSPACE/ocudu" -B "$WORKSPACE/ocudu/build-pavonis" -G "$CMAKE_GENERATOR" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DENABLE_SOAPY=ON -DENABLE_ZEROMQ=ON -DENABLE_UHD=OFF
cmake --build "$WORKSPACE/ocudu/build-pavonis" -j"$JOBS" --target radio_soapy_tx_deadline_test gnb
ctest --test-dir "$WORKSPACE/ocudu/build-pavonis" -R '^radio_soapy_tx_deadline_test$' --output-on-failure

cmake -S "$WORKSPACE/srsRAN_4G" -B "$WORKSPACE/srsRAN_4G/build-pavonis" -G "$CMAKE_GENERATOR" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo -DENABLE_ZEROMQ=ON -DENABLE_UHD=OFF \
  -DENABLE_BLADERF=ON -DENABLE_SOAPYSDR=ON -DENABLE_RF_PLUGINS=ON
cmake --build "$WORKSPACE/srsRAN_4G/build-pavonis" -j"$JOBS" \
  --target srsue srsenb srsran_rf_soapy

mkdir -p "$WORKSPACE/pavonis-build"
hashes="$WORKSPACE/pavonis-build/SHA256SUMS"
sha256sum \
  "$WORKSPACE/qcore/target/release/qcore" \
  "$WORKSPACE/ocudu/build-pavonis/apps/gnb/gnb" \
  "$WORKSPACE/srsRAN_4G/build-pavonis/srsue/src/srsue" \
  "$WORKSPACE/srsRAN_4G/build-pavonis/srsenb/src/srsenb" >"$hashes"
chmod 600 "$hashes"
echo "PAVONIS_SOURCE_BUILD=PASS jobs=$JOBS hashes=$hashes"
