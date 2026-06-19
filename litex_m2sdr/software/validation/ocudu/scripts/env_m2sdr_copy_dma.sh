#!/usr/bin/env bash
echo "ERROR: deprecated copy-mode env is disabled on nuc4." >&2
echo "Reason: LitePCIe copy-mode hung the Soapy/M2SDR path and held /dev/m2sdr0 during validation." >&2
echo "Use scripts/env_m2sdr_ocudu_safe_rf.sh instead." >&2
return 2 2>/dev/null || exit 2
