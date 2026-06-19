#!/usr/bin/env bash
set -Eeuo pipefail

# Fetch bladeRF capture artifacts from sensnuc7 and related gNB/qcore logs from nuc4.
#
# Defaults are for the current real-gNB Band 3 SSB-centered tests.
#
# Override examples:
#   CAP_PATTERN='bladerf_gnb_b3_ssbcenter_delayed_*' ./scripts/fetch_cluster_rf_run.sh
#   CAP_PATTERN='bladerf_gnb_b3_ssbcenter_backoff20_*' NUC_PATTERN='gnb_b3_downlink_no_ue_backoff20_*' ./scripts/fetch_cluster_rf_run.sh
#   CAP_HOST='stefan@sensnuc7' NUC_HOST='stefan@nuc4ts' ./scripts/fetch_cluster_rf_run.sh

CAP_HOST="${CAP_HOST:-stefan@sensnuc7}"
NUC_HOST="${NUC_HOST:-stefan@nuc4}"

CAP_ROOT="${CAP_ROOT:-~/pavonis_bladerf/runs}"
NUC_VALIDATION_ROOT="${NUC_VALIDATION_ROOT:-~/CLionProjects/m2sdr/litex_m2sdr/software/validation/ocudu}"

CAP_PATTERN="${CAP_PATTERN:-bladerf_gnb_b3_ssbcenter_backoff20_*}"
NUC_PATTERN="${NUC_PATTERN:-gnb_b3_downlink_no_ue_backoff20_*}"

DEST_ROOT="${DEST_ROOT:-$HOME/pavonis_bladerf_local/runs}"

echo "===== fetch_cluster_rf_run ====="
hostname
date -u --iso-8601=seconds
echo "CAP_HOST=$CAP_HOST"
echo "NUC_HOST=$NUC_HOST"
echo "CAP_PATTERN=$CAP_PATTERN"
echo "NUC_PATTERN=$NUC_PATTERN"
echo "DEST_ROOT=$DEST_ROOT"

mkdir -p "$DEST_ROOT"

echo
echo "===== find latest capture run on capture host ====="
REMOTE_CAP_RUN="$(ssh "$CAP_HOST" "ls -td $CAP_ROOT/$CAP_PATTERN 2>/dev/null | head -1" || true)"
if [ -z "$REMOTE_CAP_RUN" ]; then
  echo "ERROR: no capture run matched on $CAP_HOST: $CAP_ROOT/$CAP_PATTERN" >&2
  exit 2
fi

LOCAL_RUN="$DEST_ROOT/$(basename "$REMOTE_CAP_RUN")"
mkdir -p "$LOCAL_RUN"

echo "REMOTE_CAP_RUN=$REMOTE_CAP_RUN"
echo "LOCAL_RUN=$LOCAL_RUN"

echo
echo "===== rsync capture run ====="
rsync -av --progress "$CAP_HOST:$REMOTE_CAP_RUN/" "$LOCAL_RUN/"

echo
echo "===== find latest nuc4 gNB wrapper run ====="
REMOTE_NUC_RUN="$(ssh "$NUC_HOST" "cd $NUC_VALIDATION_ROOT/runs 2>/dev/null && ls -td $NUC_PATTERN 2>/dev/null | head -1" || true)"
echo "REMOTE_NUC_RUN=${REMOTE_NUC_RUN:-NONE}"

mkdir -p "$LOCAL_RUN/nuc4_logs"

if [ -n "$REMOTE_NUC_RUN" ]; then
  echo
  echo "===== rsync nuc4 wrapper run ====="
  rsync -av "$NUC_HOST:$NUC_VALIDATION_ROOT/runs/$REMOTE_NUC_RUN/" "$LOCAL_RUN/nuc4_logs/$REMOTE_NUC_RUN/"
fi

echo
echo "===== find latest nuc4 ota-stage run ====="
REMOTE_OTA_RUN="$(ssh "$NUC_HOST" "cd $NUC_VALIDATION_ROOT/runs 2>/dev/null && ls -td *_ota-stage 2>/dev/null | head -1" || true)"
echo "REMOTE_OTA_RUN=${REMOTE_OTA_RUN:-NONE}"

if [ -n "$REMOTE_OTA_RUN" ]; then
  echo
  echo "===== rsync nuc4 ota-stage run ====="
  rsync -av "$NUC_HOST:$NUC_VALIDATION_ROOT/runs/$REMOTE_OTA_RUN/" "$LOCAL_RUN/nuc4_logs/$REMOTE_OTA_RUN/"
fi

echo
echo "===== fetch /tmp gNB runtime log if present ====="
rsync -av "$NUC_HOST:/tmp/gnb_m2sdr_band3_ota.log" "$LOCAL_RUN/nuc4_logs/" 2>/dev/null || true

echo
echo "===== local artifact summary ====="
{
  echo "FETCH_TIME_UTC=$(date -u --iso-8601=seconds)"
  echo "CAP_HOST=$CAP_HOST"
  echo "NUC_HOST=$NUC_HOST"
  echo "REMOTE_CAP_RUN=$REMOTE_CAP_RUN"
  echo "REMOTE_NUC_RUN=${REMOTE_NUC_RUN:-}"
  echo "REMOTE_OTA_RUN=${REMOTE_OTA_RUN:-}"
  echo "LOCAL_RUN=$LOCAL_RUN"
  echo
  echo "===== files ====="
  find "$LOCAL_RUN" -maxdepth 3 -type f -printf '%P %s bytes\n' | sort
  echo
  echo "===== sha256 sc16 ====="
  find "$LOCAL_RUN" -maxdepth 1 -type f -name '*.sc16' -print0 | xargs -0 -r sha256sum
} | tee "$LOCAL_RUN/FETCH_MANIFEST.txt"

echo
echo "LOCAL_RUN=$LOCAL_RUN"
echo "Next:"
echo "  ./scripts/analyze_cluster_rf_run.sh \"$LOCAL_RUN\""
