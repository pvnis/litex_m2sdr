#!/usr/bin/env bash
# Source this file. It intentionally does not enable shell strict mode.

_ocudu_validation_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export VALIDATION_ROOT="${VALIDATION_ROOT:-$(cd "$_ocudu_validation_script_dir/.." && pwd)}"
export M2SDR_REPO="${M2SDR_REPO:-$(cd "$VALIDATION_ROOT/../../../.." && pwd)}"
export WORKSPACE_ROOT="${WORKSPACE_ROOT:-$(cd "$M2SDR_REPO/.." && pwd)}"
export OCUDU_REPO="${OCUDU_REPO:-$WORKSPACE_ROOT/ocudu}"
export QCORE_REPO="${QCORE_REPO:-$WORKSPACE_ROOT/qcore}"
export SRSRAN_4G_REPO="${SRSRAN_4G_REPO:-$WORKSPACE_ROOT/srsRAN_4G}"
export OCUDU_GNB_BIN="${OCUDU_GNB_BIN:-$OCUDU_REPO/build/apps/gnb/gnb}"
export QCORE_BIN="${QCORE_BIN:-$QCORE_REPO/target/debug/qcore}"
export SRSUE_BIN="${SRSUE_BIN:-$SRSRAN_4G_REPO/build/srsue/src/srsue}"
export SRSRAN_RF_PLUGIN_DIR="${SRSRAN_RF_PLUGIN_DIR:-$SRSRAN_4G_REPO/build/lib/src/phy/rf}"
export M2SDR_SW="${M2SDR_SW:-$M2SDR_REPO/litex_m2sdr/software}"
export M2SDR_UTIL="${M2SDR_UTIL:-$M2SDR_SW/user/m2sdr_util}"
export VALIDATION_RUN_ROOT="${VALIDATION_RUN_ROOT:-$VALIDATION_ROOT/runs}"

export SOAPY_SDR_ROOT="${SOAPY_SDR_ROOT:-/tmp/empty-soapy-root}"
export SOAPY_SDR_PLUGIN_PATH="${SOAPY_SDR_PLUGIN_PATH:-$M2SDR_SW/soapysdr/build}"
export M2SDR_DIRECT_DEINTERLEAVE="${M2SDR_DIRECT_DEINTERLEAVE:-1}"

_ocudu_validation_prepend_path() {
    case ":${LD_LIBRARY_PATH:-}:" in
        *":$1:"*) ;;
        *) LD_LIBRARY_PATH="$1${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
    esac
}
_ocudu_validation_prepend_path "$M2SDR_SW/user/libm2sdr"
_ocudu_validation_prepend_path "$SRSRAN_RF_PLUGIN_DIR"
export LD_LIBRARY_PATH
unset -f _ocudu_validation_prepend_path
unset _ocudu_validation_script_dir

ocudu_validation_print_env() {
    printf '%-24s %s\n' \
        VALIDATION_ROOT "$VALIDATION_ROOT" \
        M2SDR_REPO "$M2SDR_REPO" \
        WORKSPACE_ROOT "$WORKSPACE_ROOT" \
        OCUDU_REPO "$OCUDU_REPO" \
        QCORE_REPO "$QCORE_REPO" \
        SRSRAN_4G_REPO "$SRSRAN_4G_REPO" \
        OCUDU_GNB_BIN "$OCUDU_GNB_BIN" \
        QCORE_BIN "$QCORE_BIN" \
        SRSUE_BIN "$SRSUE_BIN" \
        M2SDR_UTIL "$M2SDR_UTIL" \
        SOAPY_SDR_PLUGIN_PATH "$SOAPY_SDR_PLUGIN_PATH"
}
