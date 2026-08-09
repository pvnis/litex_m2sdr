#!/usr/bin/env bash
set -euo pipefail

ART="${ART:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"

export QCORE_SIM_FILE=@REMOTE_HOME@/CLionProjects/m2sdr/litex_m2sdr/software/validation/ocudu/configs/sims-pavonis-ota.private.toml

exec bash "$ART/cp2343_run_pdsch_max_ue_mcs9.sh" "$@"
