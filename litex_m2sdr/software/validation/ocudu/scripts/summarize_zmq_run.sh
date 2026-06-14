#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${1:-}"
[[ -d "$RUN_DIR" ]] || { echo "usage: $0 RUN_DIR" >&2; exit 2; }

has() { grep -Eiq "$2" "$RUN_DIR/$1" 2>/dev/null; }
qcore_started=0; ngap=0; gnb_started=0; ue_rf=0; rach=0; rrc=0; pdu=0; ue_ip=0; ping_ok=0
[[ -s "$RUN_DIR/qcore.log" ]] && qcore_started=1
has qcore.log 'NGAP.*(setup|connection)|setup response|gNB.*connect' && ngap=1
[[ -s "$RUN_DIR/gnb.log" ]] && gnb_started=1
has srsue.log 'RF.*(open|device)|Opening.*RF|device_name' && ue_rf=1
has srsue.log 'Random Access Complete' && rach=1
has srsue.log 'RRC Connected' && rrc=1
has srsue.log 'PDU Session Establishment successful' && pdu=1
has checks.txt 'tun_ue1.*inet |inet .*tun_ue1' && ue_ip=1
has dataplane.txt '0% packet loss|VALIDATION_PING_OK=1' && ping_ok=1

highest=QCORE_STARTED
((ngap)) && highest=NGAP_SETUP
((gnb_started)) && highest=GNB_STARTED
((ue_rf)) && highest=UE_RF_OPENED
((rach)) && highest=RANDOM_ACCESS_COMPLETE
((rrc)) && highest=RRC_CONNECTED
((pdu)) && highest=PDU_SESSION
((ue_ip)) && highest=UE_IP_ASSIGNED
((ping_ok)) && highest=PING_OK
result=FAIL
((pdu && ping_ok)) && result=PASS

first_failed=NONE
for item in "QCORE_STARTED:$qcore_started" "NGAP_SETUP:$ngap" "GNB_STARTED:$gnb_started" \
    "UE_RF_OPENED:$ue_rf" "RANDOM_ACCESS_COMPLETE:$rach" "RRC_CONNECTED:$rrc" \
    "PDU_SESSION:$pdu" "UE_IP_ASSIGNED:$ue_ip" "PING_OK:$ping_ok"; do
    [[ "${item##*:}" == 1 ]] || { first_failed="${item%%:*}"; break; }
done

cat > "$RUN_DIR/summary.txt" <<EOF
Result: $result
Highest achieved milestone: $highest
First failed milestone: $first_failed
QCORE_STARTED=$qcore_started
NGAP_SETUP=$ngap
GNB_STARTED=$gnb_started
UE_RF_OPENED=$ue_rf
RANDOM_ACCESS_COMPLETE=$rach
RRC_CONNECTED=$rrc
PDU_SESSION=$pdu
UE_IP_ASSIGNED=$ue_ip
PING_OK=$ping_ok
EOF

cat > "$RUN_DIR/summary.json" <<EOF
{"result":"$result","highest_milestone":"$highest","first_failed_milestone":"$first_failed","milestones":{"qcore_started":$qcore_started,"ngap_setup":$ngap,"gnb_started":$gnb_started,"ue_rf_opened":$ue_rf,"random_access_complete":$rach,"rrc_connected":$rrc,"pdu_session":$pdu,"ue_ip_assigned":$ue_ip,"ping_ok":$ping_ok}}
EOF

{
    echo "# Validation result"
    echo
    echo "Date: $(date --iso-8601=seconds)"
    echo "Host: $(hostname)"
    echo "Run directory: $RUN_DIR"
    echo
    echo "## Result"
    echo
    echo "- PASS/FAIL: $result"
    echo "- Highest achieved milestone: $highest"
    echo "- First failed milestone: $first_failed"
    echo
    echo "## Commands and configuration checksums"
    echo
    if [[ -s "$RUN_DIR/commands-and-checksums.txt" ]]; then
        sed 's/^/- /' "$RUN_DIR/commands-and-checksums.txt"
    else
        echo "- Not recorded"
    fi
    echo
    echo "## Artifacts"
    echo
    echo "- qcore log: qcore.log"
    echo "- gNB log: gnb.log"
    echo "- srsUE log: srsue.log"
    echo "- dataplane: dataplane.txt"
    echo "- summary: summary.json"
} > "$RUN_DIR/RESULT.md"

cat "$RUN_DIR/summary.txt"
[[ "$result" == PASS ]]
