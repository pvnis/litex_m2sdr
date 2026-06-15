#!/usr/bin/env bash
set -euo pipefail

source ~/CLionProjects/env_m2sdr_current.sh

export M2SDR_LITEPCIE_ZERO_COPY=0
export M2SDR_DIRECT_DEINTERLEAVE=1
