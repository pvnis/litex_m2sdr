# OCUDU + LiteX-M2SDR Validation Harness

This directory turns the OCUDU, qcore, srsUE, and LiteX-M2SDR validation
ladder into repeatable, logged runs. Start with the ZMQ control case, then move
through bounded hardware and OTA stages without changing unrelated variables.

## Expected layout

The defaults assume sibling checkouts below one workspace:

```text
workspace/
  litex_m2sdr/
  ocudu/
  qcore/
  srsRAN_4G/
```

All important paths can be overridden before sourcing
`scripts/env_ocudu_validation.sh`. Run
`ocudu_validation_print_env` afterward to inspect the resolved paths. The
sourceable file deliberately does not enable strict shell mode because doing so
would silently change the behavior of the caller's interactive shell or script.

The scripts select the local M2SDR Soapy plugin and export
`M2SDR_DIRECT_DEINTERLEAVE=1`. Background qcore, gNB, and srsUE processes use
`setsid ... < /dev/null` so an SSH or terminal disconnect does not leave jobs
attached to a dead controlling terminal. Every runner still installs cleanup
traps and records process IDs.

## Private configuration

Copy the UE templates to private, untracked files and replace placeholders with
lab credentials. Create a private qcore SIM file matching those credentials.
Use mode `0600` for both. Never edit or commit the example SIM file as a real
subscriber database.

```bash
cp litex_m2sdr/software/validation/ocudu/configs/ue-zmq-netns.template.conf \
  /path/to/private/ue-zmq.local.conf
chmod 0600 /path/to/private/ue-zmq.local.conf /path/to/private/sims.local.toml
```

The ZMQ namespace template expects `ue1` and `tun_ue1`. Keep the baseline cell
parameters aligned with `configs/cell_equivalence_checklist.md`.

## ZMQ regression

Run the ZMQ full attach before OTA after protocol, scheduler, RRC, NAS, or UE
PHY changes. It proves the non-RF chain before radio variables are introduced.

```bash
sudo -v
litex_m2sdr/software/validation/ocudu/scripts/run_zmq_ocudu_dataplane.sh \
  --ue-conf /path/to/private/ue-zmq.local.conf \
  --sim-file /path/to/private/sims.local.toml \
  --external-iface wlp5s0
```

The runner starts qcore, gNB, and srsUE in that order, waits a bounded time,
adds the namespace default route, runs a bounded ping, writes a timestamped run
directory under `runs/`, summarizes it, and cleans qcore/ZMQ network state.
Override `VALIDATION_RUN_ROOT` to store runs elsewhere.

Manual cleanup is narrowly scoped to expected validation processes, links,
namespace, and iptables rules:

```bash
sudo -v
litex_m2sdr/software/validation/ocudu/scripts/cleanup_zmq_ocudu.sh \
  --external-iface wlp5s0 --verbose
```

## Hardware and OTA ladder

Run the non-streaming post-power-cycle preflight first:

```bash
litex_m2sdr/software/validation/ocudu/scripts/post_powercycle_m2sdr_preflight.sh
litex_m2sdr/software/validation/ocudu/scripts/run_soapy_discovery_gated.sh
litex_m2sdr/software/validation/ocudu/scripts/run_soapy_discovery_gated.sh --probe
```

The clock gate is mandatory before Soapy probe, streaming, or OTA gNB startup.
The preflight does not run DMA, Soapy probe, RF streaming, AD9361 register
operations, gNB, or srsUE.

For OTA, copy `configs/gnb_m2sdr_band3_ota.template.yml` to a private run
configuration, review RF power and attenuation, and run:

```bash
sudo -v
litex_m2sdr/software/validation/ocudu/scripts/run_ota_stage_template.sh \
  --i-understand-rf-test \
  --gnb-conf /path/to/private/gnb-band3.yml \
  --ue-conf /path/to/private/ue-band3.local.conf \
  --sim-file /path/to/private/sims.local.toml
```

This starts only qcore and the OTA gNB for a bounded interval. Add `--start-ue`
only after independently proving downlink SSB. Follow the ladder in
`templates/HARDWARE_RESULT_TEMPLATE.md`: hardware/stream, qcore, gNB,
independent downlink, srsUE, Msg1, Msg2, Msg3, RRC/NAS, then PDU/IP. Stop at the
first failed stage.

## Results and repository hygiene

`RESULT.md` records the highest achieved and first failed milestones.
`summary.json` is intended for automated comparison. Treat repeated radio
overflow/underflow/late reports, USB or DMA errors, slow real-time warnings,
stale artifacts, missing direct deinterleave, or multiple changed variables as
invalid-run markers.

Do not commit private credentials, local configs, run directories, logs, pcaps,
IQ captures, build directories, or generated hardware files. Do not add driver
unload/reload recovery to this harness. Run
`scripts/self_check_validation_harness.sh` before submitting changes.
