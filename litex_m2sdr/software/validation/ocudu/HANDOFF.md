# OCUDU Validation Harness Handoff

## Scope and assumptions

The harness lives under `litex_m2sdr/software/validation/ocudu/` and assumes
sibling `ocudu`, `qcore`, and `srsRAN_4G` checkouts. Paths and timeouts are
environment-overridable. It assumes the binaries and the local M2SDR Soapy
plugin have already been built.

Prepare private UE and qcore SIM configs from the committed templates. Keep them
outside version control with mode `0600`; IMSI, K, OPc, PLMN, and MNC must
match. The ZMQ baseline is n78, ARFCN 626000, SSB ARFCN 625632, 20 MHz, SCS 30,
PCI 1, and PRACH index 159. The OTA starting point is band 3, ARFCN 368500, SSB
ARFCN 367930, 10 MHz, SCS 15, PCI 1, PRACH index 1, and 11.52 Msps.

## Routine commands

```bash
# ZMQ attach plus namespace ping
scripts/run_zmq_ocudu_dataplane.sh \
  --ue-conf /private/ue-zmq.local.conf \
  --sim-file /private/sims.local.toml \
  --external-iface IFACE

# Read-only host snapshot
scripts/collect_host_state.sh

# Non-streaming post-power-cycle checks
scripts/post_powercycle_m2sdr_preflight.sh

# Clock-gated discovery; probe remains explicit
scripts/run_soapy_discovery_gated.sh
scripts/run_soapy_discovery_gated.sh --probe

# Bounded OTA qcore/gNB stage
scripts/run_ota_stage_template.sh --i-understand-rf-test \
  --gnb-conf /private/gnb-band3.yml \
  --ue-conf /private/ue-band3.local.conf \
  --sim-file /private/sims.local.toml
```

Run commands from this directory or use their repository-relative paths. The
environment script infers the repository root. `RESULT.md`, logs, checks, and
summaries are written into timestamped run directories.

## Validation order

Re-run ZMQ before OTA after protocol or PHY changes. For hardware testing,
advance only through the first passing stages: M2SDR hardware/stream, qcore,
OTA gNB, independent SSB detection, srsUE, Msg1, Msg2, Msg3, RRC/NAS, PDU/IP.
Independently prove downlink SSB before debugging PRACH. Keep
`M2SDR_DIRECT_DEINTERLEAVE=1` and change one experiment variable at a time.

## Safety

Confirm permitted frequencies, RF power, port roles, and attenuation before
transmission. Stop transmitters before cable changes. Do not commit subscriber
credentials, logs, captures, pcaps, or run directories. Do not add kernel
module unload/reload commands to the harness.
