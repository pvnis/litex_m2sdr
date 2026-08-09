#!/usr/bin/env bash
set -euo pipefail

ROLE="${1:?usage: install_runtime_layout.sh ran|ue SOURCE_WORKSPACE}"
WORKSPACE="${2:?usage: install_runtime_layout.sh ran|ue SOURCE_WORKSPACE}"
[[ "$ROLE" == ran || "$ROLE" == ue ]]
WORKSPACE="$(cd "$WORKSPACE" && pwd)"
[[ -f "$WORKSPACE/SOURCE_REVISIONS" ]]

link_once() {
  local target="$1" link="$2"
  [[ -e "$target" ]] || {
    echo "runtime layout target is missing: $target" >&2
    exit 69
  }
  mkdir -p "$(dirname "$link")"
  if [[ -L "$link" && "$(readlink -f "$link")" == "$(readlink -f "$target")" ]]; then
    return
  fi
  [[ ! -e "$link" && ! -L "$link" ]] || {
    echo "runtime layout collision: $link" >&2
    exit 73
  }
  ln -s "$target" "$link"
}

install_once() {
  local source="$1" destination="$2"
  [[ -x "$source" ]] || {
    echo "built executable is missing: $source" >&2
    exit 69
  }
  if [[ -f "$destination" ]] && cmp -s "$source" "$destination"; then
    chmod 700 "$destination"
    return
  fi
  [[ ! -e "$destination" && ! -L "$destination" ]] || {
    echo "runtime alias already exists: $destination" >&2
    exit 73
  }
  install -D -m 700 "$source" "$destination"
}

mkdir -p "$HOME/CLionProjects"
link_once "$WORKSPACE/litex_m2sdr" "$HOME/CLionProjects/m2sdr"

hashes=()
if [[ "$ROLE" == ran ]]; then
  gnb="$WORKSPACE/ocudu/build-pavonis/apps/gnb/gnb"
  qcore="$WORKSPACE/qcore/target/release/qcore"
  srsenb="$WORKSPACE/srsRAN_4G/build-pavonis/srsenb/src/srsenb"
  rf_soapy="$WORKSPACE/srsRAN_4G/build-pavonis/lib/src/phy/rf/libsrsran_rf_soapy.so"
  for source in "$gnb" "$qcore" "$srsenb" "$rf_soapy"; do
    [[ -f "$source" ]] || { echo "built RAN artifact is missing: $source" >&2; exit 69; }
  done
  link_once "$WORKSPACE/ocudu" "$HOME/CLionProjects/ocudu"
  link_once "$WORKSPACE/ocudu/build-pavonis" "$WORKSPACE/ocudu/build-clion"
  link_once "$WORKSPACE/qcore" "$HOME/CLionProjects/qcore"
  mkdir -p "$WORKSPACE/qcore/target/debug"
  link_once "$qcore" "$WORKSPACE/qcore/target/debug/qcore"

  install_once "$qcore" "$HOME/pavonis_cp2989_qcore_unknown_update_fallback/qcore"
  install_once "$gnb" "$HOME/pavonis_cp3025_stage1_energy_capture_gnb/gnb"
  install_once "$gnb" "$HOME/pavonis_cp3109_scheduler_conres_rebase_gnb/gnb"
  install_once "$gnb" "$HOME/pavonis_cp3597_msg4_geometry_trace_gnb/gnb"
  install_once "$srsenb" "$HOME/pavonis_cp2985_srsenb_pdu_session_dl_harq_retx/srsenb"
  install_once "$srsenb" "$HOME/pavonis_cp2656_srsenb/srsenb"
  install_once "$rf_soapy" "$HOME/pavonis_cp2939_rf_plugin/libsrsran_rf_soapy.so"
  hashes+=(
    "$HOME/pavonis_cp2989_qcore_unknown_update_fallback/qcore"
    "$HOME/pavonis_cp3025_stage1_energy_capture_gnb/gnb"
    "$HOME/pavonis_cp3109_scheduler_conres_rebase_gnb/gnb"
    "$HOME/pavonis_cp3597_msg4_geometry_trace_gnb/gnb"
    "$HOME/pavonis_cp2985_srsenb_pdu_session_dl_harq_retx/srsenb"
    "$HOME/pavonis_cp2656_srsenb/srsenb"
    "$HOME/pavonis_cp2939_rf_plugin/libsrsran_rf_soapy.so"
  )
else
  srsue="$WORKSPACE/srsRAN_4G/build-pavonis/srsue/src/srsue"
  [[ -x "$srsue" ]] || { echo "built UE artifact is missing: $srsue" >&2; exit 69; }
  link_once "$WORKSPACE/srsRAN_4G" "$HOME/CLionProjects/srsRAN_4G"
  link_once "$WORKSPACE/srsRAN_4G/build-pavonis" "$WORKSPACE/srsRAN_4G/build-clion"
  link_once "$WORKSPACE/srsRAN_4G/build-pavonis" \
    "$WORKSPACE/srsRAN_4G/build-ue-bladerf-wno-gcc13-rrc"
  hashes+=("$srsue")
fi

manifest="$HOME/PAVONIS_RUNTIME_LAYOUT_${ROLE}_SHA256SUMS"
manifest_new="$(mktemp "$HOME/.pavonis-runtime-layout.${ROLE}.XXXXXX")"
trap 'rm -f "$manifest_new"' EXIT
sha256sum "${hashes[@]}" >"$manifest_new"
if [[ -e "$manifest" ]]; then
  cmp -s "$manifest_new" "$manifest" || {
    echo "runtime manifest collision: $manifest" >&2
    exit 73
  }
  rm -f "$manifest_new"
else
  install -m 600 "$manifest_new" "$manifest"
  rm -f "$manifest_new"
fi
sha256sum -c "$manifest" >/dev/null
printf 'PAVONIS_RUNTIME_LAYOUT=PASS role=%s manifest=%s\n' "$ROLE" "$manifest"
