#!/usr/bin/env bash
# Safe OCUDU/M2SDR RF validation environment.
# Source this before running Soapy/OCUDU RF tests on nuc4.

_m2sdr_safe_fail() {
  echo "ERROR: $*" >&2
  return 2 2>/dev/null || exit 2
}

if [[ "${M2SDR_LITEPCIE_ZERO_COPY:-}" == "0" ]]; then
  _m2sdr_safe_fail "LitePCIe copy-mode is disabled for nuc4 validation; it hung /dev/m2sdr0 during testing."
fi

unset M2SDR_LITEPCIE_ZERO_COPY

export SOAPY_SDR_ROOT="${SOAPY_SDR_ROOT:-/tmp/empty-soapy-root}"
export SOAPY_SDR_PLUGIN_PATH="${SOAPY_SDR_PLUGIN_PATH:-$HOME/CLionProjects/m2sdr/litex_m2sdr/software/soapysdr/build}"

_M2SDR_USER="$HOME/CLionProjects/m2sdr/litex_m2sdr/software/user"
_M2SDR_LIB="$HOME/CLionProjects/m2sdr/litex_m2sdr/software/user/libm2sdr"

case ":${LD_LIBRARY_PATH:-}:" in
  *":$_M2SDR_USER:"*) ;;
  *) export LD_LIBRARY_PATH="$_M2SDR_USER:${LD_LIBRARY_PATH:-}" ;;
esac
case ":${LD_LIBRARY_PATH:-}:" in
  *":$_M2SDR_LIB:"*) ;;
  *) export LD_LIBRARY_PATH="$_M2SDR_LIB:${LD_LIBRARY_PATH:-}" ;;
esac

export M2SDR_DIRECT_DEINTERLEAVE="${M2SDR_DIRECT_DEINTERLEAVE:-1}"

# Required for OCUDU radio_ssb on the current nuc4/M2SDR path.
export M2SDR_SOAPY_TX_TIME_OFFSET_NS="${M2SDR_SOAPY_TX_TIME_OFFSET_NS:-20000000}"
export M2SDR_SOAPY_TX_DEDUP_TIME="${M2SDR_SOAPY_TX_DEDUP_TIME:-1}"
export M2SDR_SOAPY_TX_IDLE_FILL="${M2SDR_SOAPY_TX_IDLE_FILL:-0}"
export M2SDR_SOAPY_TX_CS16_TO_SC12="${M2SDR_SOAPY_TX_CS16_TO_SC12:-1}"

echo "M2SDR safe RF env:"
echo "  SOAPY_SDR_PLUGIN_PATH=$SOAPY_SDR_PLUGIN_PATH"
echo "  M2SDR_DIRECT_DEINTERLEAVE=$M2SDR_DIRECT_DEINTERLEAVE"
echo "  M2SDR_SOAPY_TX_TIME_OFFSET_NS=$M2SDR_SOAPY_TX_TIME_OFFSET_NS"
echo "  M2SDR_LITEPCIE_ZERO_COPY=${M2SDR_LITEPCIE_ZERO_COPY-unset}"
