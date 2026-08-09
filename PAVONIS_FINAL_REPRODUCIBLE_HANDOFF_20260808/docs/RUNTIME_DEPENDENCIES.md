# Pavonis Runtime Dependency Boundary

This inventory separates what the release carries from what a lab operator
must supply. A missing site-owned item is a failed prerequisite, not a reason
to weaken a hash or RF gate.

## Included and pinned

- privacy-clean snapshots of LiteX M2SDR, OCUDU, qcore, and srsRAN 4G;
- the CP3406 OCUDU deadline overlay and focused test;
- the complete active phone controller chain and its non-secret primer configs;
- the CP3424-confirmed M2SDR/bladeRF bounded-data controller chain;
- role-based SSH/SCP implementations and all controller dependency hashes;
- ZMQ gNB/UE templates and its attach/user-plane gate;
- project-owned handset helpers
  `pavonis_fplmn_repair.cp2661.jar` and
  `pavonis_sim_power_cycle.jar`, SHA-256 pinned in the scripts;
- exact QCSuper Python package versions installed by `install-ue-aux`.

## Private and deliberately external

- the private test subscriber identity and authentication values;
- qcore/UE subscriber TOML files populated with that same role;
- SSH private keys, passwords, known-host contents, and physical host names;
- the handset serial and any retained mobile-network state;
- site addresses, interface names, RF cabling/antenna placement, and attenuation
  documentation.

Private files must remain mode 600 outside the release. The package accepts
their paths or deploy roles; it never reads their values into a report.

## Host-provided

The RAN role provides the M2SDR PCIe device, running-kernel module, matching
Soapy plugin, recorded gateware class, qcore routing privileges, performance
governor/EPP, and effective real-time scheduling permission. The UE role
provides bladeRF USB SuperSpeed for the bounded gate, ADB, and exactly one
authorized rooted handset. Both roles need the packages listed in the
runbook, a dedicated service account, and bounded passwordless sudo for the
named lab operations.

The phone baseline uses root only for controlled SIM-state handling and a
filtered Qualcomm diagnostic observer. Measured traffic still traverses NR;
ADB and QCSuper do not carry benchmark payload.

The unchanged proof harness also requires passwordless root operations. Its
service-account-owned real-time helper means the supplied private-lab sudoers
example is intentionally broad and must be used only on dedicated hosts. This
is a documented POC security limitation; least-privilege conversion requires
root-owned fixed-function helpers and is future hardening work.

## Built artifacts and aliases

Native builds are intentionally not bundled as portable binaries. After
building on the destination ABI/kernel, place or link the resulting files at
the compatibility paths with `tools/install_runtime_layout.sh`, generate
explicit SHA replacements with `tools/make_pin_overrides.py`, and rerender the
harness. The installer and remote doctor verify a per-role runtime hash
manifest; the controller rejects old hashes or missing aliases.

## Verification status

The release verifies source-bundle integrity, applies the OCUDU overlay,
parses every shell/Python file, materializes and fixed-point re-pins the full
controller graph, executes the bounded controller's validate-only and closed
RF paths, checks both phone RF gates closed, and tests stage-receipt creation
without network or RF.

On the packaging workstation, the clean qcore snapshot passed clippy with
warnings denied and built in release mode. The full OCUDU build was not
possible there because its SoapySDR development package was absent and the
workstation had no noninteractive package-install privilege. This is an
environment-limited partial, not evidence that the packaged RAN build passed
on a clean workstation. The runbook therefore retains SoapySDR as a mandatory
native-host prerequisite and never claims a fresh-host RF rerun was performed
during packaging.
