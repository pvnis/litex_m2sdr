# Pavonis Cold-Lab Reproduction Runbook

Status: final release. This runbook starts from this package and two clean
Ubuntu hosts. It never requires a pre-existing personal account name or files
under one. `docs/RUNTIME_DEPENDENCIES.md` is the authoritative included versus
site-owned inventory.

## 1. Success boundary

The required final milestone is a real handset attached OTA to OCUDU through
the LiteX M2SDR, with a private PDU session, bidirectional application data,
DNS, and public-internet egress. Cell visibility, RRC connection, or core
registration alone are partial results.

The proven phone sequence uses a bounded srsENB primer before OCUDU. A cold
OCUDU-only sequence is not yet equivalent: it reaches RF access, RRC, and core
registration but not this handset's private PDU session. Do not remove the
primer during reproduction.

## 2. Topology

| Role | Required components |
|---|---|
| Controller | This package, Python 3.11+, Git, SSH/SCP, 60 GB free for builds |
| `ran` host | qcore, srsENB primer, OCUDU, LiteX M2SDR PCIe, N3/N6 routing |
| `ue` host | ADB, one authorized rooted real handset, bladeRF/srsUE bounded-gate hardware |
| RF | Authorized private-lab antenna path, documented M2SDR channel and attenuation |

The controller's measured traffic is not carried by ADB. ADB changes phone
state and launches probes; bytes under test traverse the radio, private core,
and N6 path.

## 3. Recorded software/hardware epoch

The final August 5 runs used:

- Ubuntu 24.04, x86-64, kernel `6.8.0-101-lowlatency` on both role hosts;
- performance CPU governor on both hosts;
- SoapySDR library `0.8.1`, API `0.8.0`, ABI `0.8`;
- M2SDR module SHA-256
  `3ded84bbc2ac7ff26a8192666c4083041ae54ebdfd808b48633ccedc91261ead`;
- M2SDR M.2 gateware built 2026-06-15 12:00:51, PCIe Gen2 x1, PTM enabled,
  internal-XO clocking;
- bladeRF CLI `1.10.0`, libbladeRF `2.6.0`, firmware `2.6.0`, FPGA `0.16.0`,
  USB SuperSpeed;
- ADB `1.0.41` with exactly one authorized handset.

The recorded Soapy source SHA is
`644e7858ee88d92791a495b7f4a24d762c9e819afb5337c0ef20d02e65ca4e76`;
the build and deployed module SHA is
`81b8cb2e079ed050a0056768cc349b0aaf31b25ae145eca359bd24934f4ff97c`.
All runtime pins are listed in `config/runtime-pin-map.toml`.

A new kernel, gateware, driver, module, or library is a new hardware epoch.
Keep the prior kernel bootable and re-run the producer and OTA gates before
calling the new epoch equivalent.

## 4. Verify before extracting

From the package root:

```bash
./pavonis-final.sh verify
```

This verifies the package manifest, four Git bundles, privacy rules, and all
shell/Python syntax. Stop on any failure.

The source bundle heads are:

| Repository | Commit |
|---|---|
| LiteX M2SDR | `b4d3dbae113ed8a6a33fac177dab0d85381677d6` |
| OCUDU | `d12a7f10ec4edfb57364e7be31343da34fafb81d` |
| qcore | `d84e2f433c8ff9f5fb10e6fcc05d25b6c15a7c67` |
| srsRAN 4G | `d3a772af794650cd2544a88ee0781f7af0bb219a` |

Each bundle is a privacy-clean, one-commit snapshot. The OCUDU CP3406 overlay
is applied by the unpack tool and leaves the bundle itself unchanged.

## 5. Create service accounts and SSH

Create a dedicated unprivileged lab account on both role hosts. The example
uses `pavonis`; another name is valid. The home path must be the same on both
hosts because one rendered, hash-pinned harness is deployed to both.

```bash
sudo useradd --create-home --shell /bin/bash pavonis
sudo usermod -a -G dialout,plugdev,pavonisrt pavonis
```

Install the controller's public key in that account. Keep normal SSH host-key
verification. The package defaults to `StrictHostKeyChecking=yes`; use
`accept-new` only for first enrollment and switch back to `yes` afterward.
Password transport is supported only through a separate mode-600 password
file and requires `pexpect`; key transport is preferred.

The preserved proof harness runs a service-account-owned real-time tuning
helper as root and launches qcore/srsUE through dynamic environment wrappers.
Consequently, the unchanged harness does **not** have a genuinely narrow sudo
boundary. On dedicated, single-purpose private-lab hosts, install the explicit
example profile after reviewing and, if needed, changing its account name:

```bash
sudo install -o root -g root -m 440 \
  config/sudoers.pavonis-lab.example /etc/sudoers.d/pavonis-lab
sudo visudo -cf /etc/sudoers.d/pavonis-lab
sudo -u pavonis sudo -n true
```

This is a known POC security limitation, not a least-privilege claim. A
productized deployment must replace user-owned privileged helpers with
root-owned fixed-function wrappers and a narrow command policy. Do not use
this profile on a shared or production host.

## 6. Private site file

Create it outside this package:

```bash
cp config/site.toml.example ../pavonis-site.toml
chmod 600 ../pavonis-site.toml
```

Fill the two role hosts/users/homes, SSH key, known-hosts file, core N2
address, primer S1 address, and interface names. The file is rejected if it is
group/world readable. Never place credentials, SIM values, a handset serial,
or a private key in the package tree.

Validate parsing without SSH:

```bash
./pavonis-final.sh doctor --site ../pavonis-site.toml --local-only
```

The full remote doctor is intentionally deferred until the runtime aliases,
rendered harness, QCSuper environment, and helper JARs are installed.

## 7. Host packages and real-time state

On Debian/Ubuntu install:

```bash
sudo apt update
sudo apt install \
  build-essential cmake ninja-build pkg-config git jq rsync python3 python3-pexpect \
  libfftw3-dev libmbedtls-dev libboost-program-options-dev \
  libconfig++-dev libsctp-dev libyaml-cpp-dev libzmq3-dev \
  libsoapysdr-dev soapysdr-tools libbladerf-dev libusb-1.0-0-dev \
  iproute2 iptables pciutils usbutils adb python3-venv \
  linux-headers-"$(uname -r)"
```

Install the qcore-pinned Rust toolchain:

```bash
rustup toolchain install nightly-2026-05-20 --component rust-src
cargo install bpf-linker --version 0.10.3 --locked
```

Use the persistent real-time group/limits setup described by the project:
effective `RLIMIT_RTPRIO >= 98`, successful bounded `chrt --fifo 10 true`,
performance governor, performance EPP, and no startup warning from the three
lower-PHY real-time threads. A reboot invalidates those assumptions until
read back.

## 8. Unpack and build

Keep the verified release on the controller and copy the same immutable bytes
to both role hosts. `RAN_HOST` and `UE_HOST` below are site values, not literal
names:

```bash
rsync -a --exclude work/ ./ \
  pavonis@RAN_HOST:pavonis-release/
rsync -a --exclude work/ ./ \
  pavonis@UE_HOST:pavonis-release/
```

On each destination, run `./pavonis-final.sh verify` before unpacking. Build
the full snapshot on the controller for the ZMQ gate and natively on each role
host for its runtime artifacts; do not copy controller binaries into a role
host merely because all three machines are x86-64:

```bash
cd "$HOME/pavonis-release"
./pavonis-final.sh verify
./tools/unpack_source.sh "$HOME/pavonis-source"
PAVONIS_BUILD_JOBS="$(nproc)" \
  ./tools/build_from_source.sh "$HOME/pavonis-source"
```

The build tool verifies all four commits, applies the exact CP3406 OCUDU
overlay, keeps OCUDU's warning-as-error behavior, runs qcore clippy with
warnings denied, builds qcore/OCUDU/srsUE/srsENB, and runs the focused OCUDU
Soapy deadline test.

Build LiteX M2SDR user tools, kernel module, and Soapy plugin natively on the
`ran` host. The exact ordinary software build/install sequence for the bundled
snapshot is:

```bash
cd "$HOME/pavonis-source/litex_m2sdr/software"
./build.py --clean
sudo make -C kernel install
sudo make -C user install_dev PREFIX=/usr/local
sudo make -C soapysdr/build install
sudo depmod -a
sudo ldconfig
sudo modprobe m2sdr
SoapySDRUtil --probe='driver=LiteXM2SDR'
```

The optional GUI dependencies are not required. Do not cross-deploy the
controller build. Do not flash gateware as part of a routine source build;
record the current gateware first and treat a flash as a new hardware epoch.
The bundled repository README and `source/README.md` remain the detailed
reference.

The clean snapshot build is review source, not a promise of byte-identical
compiler output. Generate explicit SHA replacements for rebuilt artifacts:

```bash
python3 tools/make_pin_overrides.py \
  --qcore PATH --gnb PATH --srsenb PATH --rf-soapy PATH \
  --soapy-module PATH --soapy-source PATH --m2sdr-module PATH \
  --enb-conf PATH --rr-conf PATH --sib-conf PATH --rb-conf PATH
```

Place the emitted entries under `[runtime.pin_overrides]` in the private site
file. This does not declare the rebuild equivalent; it lets the harness name
the new bytes so the staged gates can judge them.

On each destination host, from its new service account, create the exact
compatibility layout without hand-written aliases:

```bash
# On the RAN role host:
./tools/install_runtime_layout.sh ran "$HOME/pavonis-source"

# On the UE role host:
./tools/install_runtime_layout.sh ue "$HOME/pavonis-source"
```

The installer is restartable when every existing alias has the expected target
or bytes. It refuses any mismatched path and writes a mode-600 hash manifest,
which the remote doctor verifies before a staged run. It does not install a
kernel module or Soapy module system-wide.

Create the private ZMQ inputs after unpacking:

```bash
./pavonis-final.sh prepare-private --workspace "$HOME/pavonis-source"
```

Populate `$HOME/pavonis-source/pavonis-local/ue-zmq.local.conf` and
`sims-zmq.local.toml` with one matching project-owned test-subscriber role.
Both files are created mode 600. Never paste their contents into a checkpoint
or package artifact.

## 9. Runtime layout

The layout installer creates these compatibility aliases on the roles:

```text
$REMOTE_HOME/pavonis_cp2989_qcore_unknown_update_fallback/qcore
$REMOTE_HOME/pavonis_cp3025_stage1_energy_capture_gnb/gnb
$REMOTE_HOME/pavonis_cp3109_scheduler_conres_rebase_gnb/gnb
$REMOTE_HOME/pavonis_cp3597_msg4_geometry_trace_gnb/gnb
$REMOTE_HOME/CLionProjects/ocudu/build-clion/apps/gnb/gnb
$REMOTE_HOME/CLionProjects/qcore/target/debug/qcore
$REMOTE_HOME/CLionProjects/srsRAN_4G/build-clion/srsue/src/srsue
$REMOTE_HOME/CLionProjects/srsRAN_4G/build-ue-bladerf-wno-gcc13-rrc/srsue/src/srsue
$REMOTE_HOME/pavonis_cp2985_srsenb_pdu_session_dl_harq_retx/srsenb
$REMOTE_HOME/pavonis_cp2656_srsenb/srsenb
$REMOTE_HOME/pavonis_cp2939_rf_plugin/libsrsran_rf_soapy.so
$REMOTE_HOME/CLionProjects/m2sdr/litex_m2sdr/
```

The gNB aliases may point to the same clean current gNB only when the
runtime pin overrides name its actual SHA and the no-RF feature/default probes
pass. Do not symlink mismatched historical binaries under a familiar path.

The deploy command installs the rendered helper scripts in `$REMOTE_HOME` and
the materialized non-secret primer configs in:

```text
$REMOTE_HOME/pavonis_cp2924_srsenb_m2sdr/config/
```

Install the Soapy module at the system ABI path reported by
`SoapySDRUtil --info`; install/rebuild the M2SDR module for the running kernel.

Create the private subscriber role file, mode 600, at the path required by
the rendered qcore and OCUDU helpers beneath the local LiteX validation
directory. Start from the example role only. The real values must remain
outside version control and this package.

Install the active phone chain's non-secret UE prerequisites into the new
service-account home:

```bash
./pavonis-final.sh install-ue-aux --site ../pavonis-site.toml
```

This uploads the two packaged project helper JARs with their recorded hashes
and creates `$REMOTE_HOME/pavonis_qcsuper_cp2741` with QCSuper `2.1.0.post4`,
`crcmod 1.7`, `pycrate 0.8.1`, `pyserial 3.5`, and `pyusb 1.3.1`. It does not
change phone settings or transmit RF.

## 10. Network and phone prerequisites

Recreate the private N2/N3/N6 topology described by the validation scripts:

- qcore N2 on the configured core address/interface;
- `qcoretun` for the private UE subnet;
- the host-local N3 path expected by the gNB profile;
- N6 forwarding and MASQUERADE on the configured external interface;
- no stale route from a prior run.

On the `ue` role, `adb devices` must show exactly one authorized device and
`adb shell su -c id` must return root. The handset must have mobile data and
data roaming enabled. The private test cell is presented as roaming on this
device. ADB cannot grant the vendor-protected roaming setting; approve it in
the phone UI once.

The phone may retain 5GS state after the core is restarted. Preserve the
fresh-epoch, SIM-cycle, and policy-audit sequence in the executor. Do not
interpret stale retained state as a new radio defect.

## 11. Materialize, gate, deploy

Use a new output path each time:

```bash
render="$PWD/work/manual-render-$(date -u +%Y%m%dT%H%M%SZ)"
./pavonis-final.sh materialize \
  --site ../pavonis-site.toml \
  --output "$render"
./pavonis-final.sh deploy \
  --site ../pavonis-site.toml --harness "$render"
./pavonis-final.sh doctor --site ../pavonis-site.toml
```

Materialization substitutes site paths and addresses, applies explicit
runtime pin replacements, then recomputes the complete included script/config
SHA chain to a fixed point. `RENDERED_SHA256SUMS` records the output.

Before any RF, prove both gates closed:

```bash
./pavonis-final.sh dry-run slow --site ../pavonis-site.toml
./pavonis-final.sh dry-run fast --site ../pavonis-site.toml
```

Both must report `PAVONIS_RF_GATE_CLOSED=PASS ... rc=2` without SSH.

## 12. Staged validation

A clean rebuild must pass in order:

1. ZMQ qcore + OCUDU + srsUE attach and user-plane ping, no RF.
2. Bounded M2SDR + bladeRF attach and bidirectional data.
3. The selected phone OTA profile, including private sustained data and a
   hold-time public-IP, DNS, and HTTPS proof bound to the NR interface.

The final package carries and executes the source/controllers for all three
gates. The bounded stage is the CP3424-confirmed 45-second, four-stream
M2SDR/bladeRF shape. It requires cell selection, at least one CRC-OK PUSCH,
clean Soapy timeout/downlink-late health, and a passing sustained-data
analysis. Do not let a later stage run after an earlier stage fails or is
void. Use one exact retry for a single upstream void; never count a no-attach
roll as a throughput result.

The no-RF stage can be run independently:

```bash
./pavonis-final.sh zmq \
  --workspace "$HOME/pavonis-source" \
  --stamp "$(date -u +%Y%m%dT%H%M%SZ)_ZMQ"
```

It must produce a `summary.json` with result `PASS`, highest milestone
`PING_OK`, no failed milestone, and `ping_ok=1`.

## 13. Run the phone profiles

These commands transmit only after rerunning the ZMQ gate and passing the
bounded RF gate. Confirm the private lab path, use a fresh stamp, and keep the
operator present:

```bash
./pavonis-final.sh run slow \
  --site ../pavonis-site.toml \
  --workspace "$HOME/pavonis-source" --rf-approved \
  --stamp "$(date -u +%Y%m%dT%H%M%SZ)_SLOW"

./pavonis-final.sh run fast \
  --site ../pavonis-site.toml \
  --workspace "$HOME/pavonis-source" --rf-approved \
  --stamp "$(date -u +%Y%m%dT%H%M%SZ)_FAST"
```

Each command creates distinct `_ZMQ`, `_BOUNDED`, and `_PHONE` run identifiers
under one fresh top-level stamp. The final `STAGED_SUMMARY.json` records paths
and SHA-256 for all stage receipts. The fast executor runs the 45-second
sustained benchmark. The adapter briefly enables the proven hold, runs the
public-egress helper, then creates the release sentinel automatically. A
valid result must have successful attach, target-era session activation,
complete endpoint JSON on both sides, bidirectional byte growth, gateway,
public-IP, DNS and HTTPS proof, and clean teardown. Report the
measured result even when it is below the August 5 value.

Use `--hold` only to keep the already-proven stack up for an interactive demo
after the automatic public-internet proof.
Launch held runs inside `tmux` or another persistent controller session. The
release sentinel is created inside that run's private harness directory:

```text
slow: cp3121h_<TOPLEVEL_STAMP>_PHONE_RELEASE
fast: cp3612h_<TOPLEVEL_STAMP>_PHONE_RELEASE
```

If the controller session dies during a hold, the remote stack can be
orphaned. Run the matching qcore stop, OCUDU stop, and phone restore helpers
from the rendered harness before starting anything else.

## 14. Failure classification

- **Void:** an upstream event never occurred. Retry the exact shape once.
- **First post-reboot cold class:** huge TX timeouts, zero downlink-late, deaf
  acquisition. Discard once and rerun unchanged after host-state readback.
- **Clean zero-RACH:** no Msg1 makes RAR/Msg3/data questions void.
- **Two-state good/floor SINR:** a run-start timing/transport presentation
  class, not evidence that power changed.
- **Native/cross ABI mismatch:** rebuild on the destination host.
- **Pipefail false negative:** inspect the bounded summary before treating a
  wrapper return code as a radio failure.
- **Stale routes or phone identity:** restore the host network and fresh phone
  epoch before another RF run.

Never enable full-rate same-device RX append, per-sample hot-path scans, or a
growing-log poller during a detector-bearing run. Those observer effects were
reproduced repeatedly.

## 15. Teardown and records

Every run uses a fresh directory under `work/runs/<STAMP>`. Teardown must stop
the gNB/primer/core, restore phone and N3 state, release the M2SDR stream, and
leave both roles idle. Do not package raw run logs. Create a sanitized summary
with paths and SHA-256, run the privacy scanner, and add it as a new numeric
checkpoint rather than editing an earlier report.
