# CLAUDE-CP3631: Lab Bring-up Diagnosis, Hold-Open Runner, and Package Corrections

Version: CLAUDE-CP3631, 2026-08-05
Agent: Claude (Opus 5). The `CLAUDE-` prefix distinguishes this agent's
checkpoints from the numbered campaign checkpoints produced by other agents.

This file catalogs every file changed and every discovery made in this session.
It follows the append-only convention: nothing recorded here rewrites an
existing checkpoint, report, or runner.

## 1. Discoveries

### 1.1 The N20 phone run failed at the srsENB primer (root cause found)

Run `20260805T084531Z` reported `BASE_RC=1`. Stage-by-stage evidence:

| Stage | Result |
|---|---|
| RAN host preflight | PASS |
| UE host preflight | PASS |
| qcore start | PASS, pid 4002725 |
| **srsenb start** | **empty log, no marker** |
| qcore stop | PASS (cleanup ran correctly) |

`base.log` is 0 bytes because the base run never started. `DLF_BYTES=0` and the
empty-file digest `e3b0c442...b855` on the QCSuper capture are consequences of
the same stall, not independent faults. QCSuper output is Class C scaffolding
and is not a success condition.

**Root cause: the deployed LiteX-M2SDR Soapy plugin has moved past the hash the
CP2985 primer pins.** Measured on RAN host:

| Gate | Expected by primer | Actually deployed | Verdict |
|---|---|---|---|
| srsenb binary | `fb830086...b089` | matches | ok |
| RF plugin `libsrsran_rf_soapy.so` | `142a2efc...3ca56` | matches | ok |
| Soapy module `libSoapyLiteXM2SDR.so` | `b784c400...520e` | `81b8cb2e...f97c` | **MISMATCH** |
| Soapy source `LiteXM2SDRStreaming.cpp` | `b5131aab...fd89` | `888cfbda...4ce9` | **MISMATCH** |

`sudo -n` works and the program-mode node is present, so neither is implicated.

The primer runs `set -euo pipefail` and compares hashes with a bare `[[ ]]`
test, so a mismatch exits non-zero **with no message**. That is why the log
contained only the SSH password prompt.

**Attribution.** The mismatched file is exactly the one CP3578 edited: the
`setupStream()` TX-attenuation reassert recorded in
`FINAL_REPORT_CP3579_REPLAY_TX_STATE_ROOT_CAUSE.md`. The deployed plugin is
therefore *newer and better* than the N20-era pin, not corrupt. The gate is
correct and is reporting a real provenance divergence: the N20 primer config
was validated against the pre-CP3578 plugin.

This is a live instance of the campaign's own recorded lesson that a stale
pin or wrapper can silently discard correct values.

### 1.1b CORRECTION: the real mismatch was the Soapy SOURCE only, and the fix was a revert

Section 1.1 above is **partially wrong** and is retained for the record. Two
corrections:

**The module was never mismatched for CP3121.** Section 1.1 compared against the
CP2985 primer's *built-in defaults*. But `cp3121_run_filtered_phone_metrics.sh`
**hardcodes** its own Soapy hashes in the `env` line that launches the base
runner, overriding anything exported upstream:

```text
PAVONIS_SOAPY_SOURCE_SHA_EXPECTED=644e7858ee88d92791a495b7f4a24d762c9e819afb5337c0ef20d02e65ca4e76
PAVONIS_SOAPY_MODULE_SHA_EXPECTED=81b8cb2e079ed050a0056768cc349b0aaf31b25ae145eca359bd24934f4ff97c
```

Against the values CP3121 actually uses:

| Artifact | CP3121 expects | Was deployed | Verdict |
|---|---|---|---|
| Soapy module `.so` | `81b8cb2e...f97c` | `81b8cb2e...f97c` | **always matched** |
| Soapy source `.cpp` | `644e7858...4e76` | `888cfbda...4ce9` | **the only mismatch** |

**Root cause restated: the source tree drifted ahead of the installed module.**
Timestamps confirm it:

```text
2026-07-22 09:40  libSoapyLiteXM2SDR.so                 (CP3121-era build, correct)
2026-08-01 23:05  LiteXM2SDRStreaming.cpp               (later campaign edit, uncommitted)
```

The `.cpp` carried **uncommitted working-tree edits** (2090-line diff vs HEAD)
from the later CP3549/CP3578-era work. The `.so` was never rebuilt from them.
The primer's source gate was correctly refusing to run a deployment whose source
no longer corresponded to its binary.

**Because of this, the hash-override approach in the CLAUDE-CP3631 fork does not
work and never could.** The hardcoded `env` line wins. That is why run
`20260805T090041Z` failed at `srsenb_start` exactly as run `20260805T084531Z`
had, with the same 35-byte log.

**Resolution: reverted rather than overridden** (operator decision). See
section 2.4.

### 1.2 An earlier claim of mine was wrong

I first reported "no srsenb binary on RAN host". That was incorrect: my glob only
searched `srsRAN_4G/build*/srsenb/src/srsenb`. The binary is present at
`[role-home]/pavonis_cp2985_srsenb_pdu_session_dl_harq_retx/srsenb`, exactly
where `BIN_BASE` expects it, and its hash matches. Corrected before any action
was taken on it.

### 1.3 The 15 MHz runners are not on the controller laptop

**SUPERSEDED by 1.3b.** A search for `*15mhz*`, `*mcs11*`, `*mcs17*`,
`*lead13*` runners found nothing. Local phone runners top out at CP3121, the
10 MHz MCS9 N20 shape.

Consequence: the `medium` (14.545), `fast` (23.186), and `record` (31.155)
Mbit/s profiles are **not reachable from this laptop today**. Their drivers are
either on RAN host or were never saved as reusable scripts. Do not promise a
23 Mbit/s demo number until that is resolved.

**1.3b CORRECTION: they are local.** The CP36xx execute/runner pairs live in
dated subdirectories of the stage6 artifacts tree itself
(`cp3612_phone_15mhz_all_mcs17_no_rf_gate_20260802/`,
`cp3586_phone_15mhz_1x1_ota_20260802/`, etc.); the original search missed them
because the directory names carry the shape, not the script names at top
level. The fast (CP3613-class) shape was relaunched from here on 2026-08-05
(sections 1.16 and 2.7).

### 1.4 There was no hold-open capability

`cleanup()` is trapped on `EXIT INT TERM HUP` and tears the stack down
unconditionally. The observe knobs (`SWAP_OBSERVE_SEC` 60-240 s,
`POSTSTART_RESELECT_OBSERVE_SEC` 30-60 s) are bounded and are not demo windows.
Addressed in section 2.3.

### 1.5 Lab state at time of writing

Both hosts up 22 days and idle, zero stack processes, `/dev/m2sdr0` present and
module loaded, bladeRF 2.0 micro enumerated. SSH from the laptop requires the
pexpect wrapper; `BatchMode` key auth is not configured.

### 1.6 The submission package's own verify gate was failing

Recorded in full in `SANITIZATION_CHANGE_INVENTORY_CP3629.md` and fixed at
CP3630. Summary: `privacy_scan.py` read full-precision IEEE-754 doubles as
15-digit subscriber numbers, so `./pavonis-supervisor.sh verify` exited 1 on
four campaign metric files containing no private data.

### 1.7 The submission package was stale and internally inconsistent

Recorded in full in `docs/FINAL_REPORT_CP3629_SUPERVISOR_HANDOFF.md`. Summary:
the master report's checkpoint index stopped at CP3147 while the campaign ran
to CP3628; `START_HERE.md` and `docs/OPERATOR_RUNBOOK.md` asserted a "true
10 MHz ceiling" disproven by CP3589; and manifest coverage was frozen at
CP3425, leaving five reports and nine other files unpinned. The five unpinned
reports were verified still intact via the successor-report hash chain.

## 2. Files created or changed

### 2.1 Submission package, CP3629 (documentation corrections)

Created:

```text
PAVONIS_SUPERVISOR_SUBMISSION_20260727/START_HERE_CP3629.md
PAVONIS_SUPERVISOR_SUBMISSION_20260727/SANITIZATION_CHANGE_INVENTORY_CP3629.md
PAVONIS_SUPERVISOR_SUBMISSION_20260727/docs/FINAL_REPORT_CP3629_SUPERVISOR_HANDOFF.md
PAVONIS_SUPERVISOR_SUBMISSION_20260727/docs/OPERATOR_RUNBOOK_CP3629.md
PAVONIS_SUPERVISOR_SUBMISSION_20260727/manifests/CP3629_SHA256SUMS
PAVONIS_SUPERVISOR_SUBMISSION_20260727/manifests/PACKAGE_INVENTORY_CP3629.json
```

Modified, with explicit authorization:

```text
PAVONIS_SUPERVISOR_SUBMISSION_20260727/START_HERE.md
PAVONIS_SUPERVISOR_SUBMISSION_20260727/manifests/SHA256SUMS
```

`START_HERE.md` gained inline `CP3629 update` / `CP3629 correction` blocks. No
original sentence was deleted or rewritten; each superseded claim is retained
verbatim and annotated beneath. `manifests/SHA256SUMS` had one line re-pinned.

### 2.2 Submission package, CP3630 (privacy fix and demo controller)

Created:

```text
PAVONIS_SUPERVISOR_SUBMISSION_20260727/pavonis-demo.sh
PAVONIS_SUPERVISOR_SUBMISSION_20260727/SANITIZATION_CHANGE_INVENTORY_CP3630.md
PAVONIS_SUPERVISOR_SUBMISSION_20260727/docs/DEMO_RUNBOOK_CP3630.md
PAVONIS_SUPERVISOR_SUBMISSION_20260727/runtime/controller/DEMO_ADAPTER_CONTRACT.md
PAVONIS_SUPERVISOR_SUBMISSION_20260727/runtime/controller/run_demo.sh
PAVONIS_SUPERVISOR_SUBMISSION_20260727/runtime/configs/lab-demo.env.template
PAVONIS_SUPERVISOR_SUBMISSION_20260727/runtime/demo/profiles/{slow,medium,fast,record}.json
PAVONIS_SUPERVISOR_SUBMISSION_20260727/runtime/patches/CP3630-privacy-scan-float-mantissa.patch
PAVONIS_SUPERVISOR_SUBMISSION_20260727/manifests/CP3630_SHA256SUMS
```

Modified:

```text
PAVONIS_SUPERVISOR_SUBMISSION_20260727/runtime/controller/privacy_scan.py
PAVONIS_SUPERVISOR_SUBMISSION_20260727/manifests/SHA256SUMS
```

The `privacy_scan.py` change is one regex lookbehind and is reversible:

```bash
patch -p1 -R < runtime/patches/CP3630-privacy-scan-float-mantissa.patch
```

Reverting restores `273230155639499f76136b527b0a430fe56708f254edfb6e9a7863fe1563874a`
and reinstates the four false positives. Its `manifests/SHA256SUMS` line must
then be re-pinned to that digest.

**Status: `pavonis-demo.sh` is not runnable.** It requires a site adapter
implementing `DEMO_ADAPTER_CONTRACT.md`. No adapter exists. All testing used a
mock that writes JSON and touches no radio.

### 2.3 Lab runner fork, CLAUDE-CP3631 (hold-open)

Created in
`pavonis_stage6_agent_artifacts_20260629T0918Z/prearmed_raw512_fast_20260630T0045Z/`:

```text
cp3121h_run_continuous_srsenb_to_ocudu_metrics_rf.sh
cp3121h_run_filtered_phone_metrics.sh
cp3121h_run_metric_al8_phone_rf.sh
cp3121h_execute_metric_al8_phone_rf.sh
```

The `h` suffix means hold. These are **copies**. All four CP3121 originals were
verified byte-identical afterwards against their pinned digests:

```text
64740c016cc8d43d45a1e411755242c5c13e547bf5973e656d24caadbfdebb77  cp3121_run_continuous_srsenb_to_ocudu_metrics_rf.sh
6534737880b188da25cd1f88db0ee87c6e5b19db3f398cd1fc3df9424c791241  cp3121_run_filtered_phone_metrics.sh
63398c363586cd9d8d22e201fda7735a353451654a167ff8c0f0ec13f52dd56c  cp3121_run_metric_al8_phone_rf.sh
bc80239b5b9b80efb03f28e11b0e4279120bb5af757887b5a6737e59b9c14f21  cp3121_execute_metric_al8_phone_rf.sh
```

Two changes relative to the originals:

1. **Hold gate.** Two new knobs, `PAVONIS_HOLD_BEFORE_TEARDOWN` (0/1, default 0)
   and `PAVONIS_HOLD_MAX_SEC` (60-21600, default 3600). When enabled, the first
   thing `cleanup()` does is block until a sentinel file appears, `SIGINT`
   arrives, or the cap elapses. It runs on success and on failure, so a failed
   run can also be inspected before teardown. `hold_done` guards against
   re-entry. During the hold, `INT TERM HUP` are re-trapped so Ctrl-C breaks the
   wait and proceeds to teardown rather than killing the script with the stack
   still up.
2. **Soapy hash defaults** updated to the currently deployed values from
   section 1.1, both still overridable.

Re-pinned chain digests:

```text
cp3121h_run_continuous_srsenb_to_ocudu_metrics_rf.sh  2e61b311eded6f54bd3ca355fc6ccca509f2331c729923d30e0c26f9cf706019
cp3121h_run_filtered_phone_metrics.sh                 e9a83eba80b59581cf10971b07de24170e3dd97864c1b98906ec811c9674605c
cp3121h_run_metric_al8_phone_rf.sh                    3606df08cfb90611dfad78c09f6503c5e4dc4e816a5068bc4dc80c168276de0b
```

### 2.4 RAN host stack revert to the known-good CP3121 state

**Changed on RAN host** (the only change made to a lab host in this session):

```text
[role-home]/CLionProjects/m2sdr/litex_m2sdr/software/soapysdr/LiteXM2SDRStreaming.cpp
```

Restored from commit `23571de93d440a88e5fbc29f9db38e7bf1be7322` via
`git show <commit>:<path> > <path>`. That form was chosen deliberately: it
rewrites only the one file and does not move `HEAD`, create a stash, or touch
any other working-tree change in the `stefan` branch.

**Backup taken before the revert**, per the runbook's "preserve and record the
known working state" rule:

```text
[role-home]/pavonis_claude_cp3631_backup/
  LiteXM2SDRStreaming.cpp.worktree-20260801     188169 bytes, sha256 888cfbda...4ce9
  LiteXM2SDRStreaming.cpp.worktree.diff          88787 bytes, 2090 lines vs HEAD
```

The backup preserves the uncommitted later-campaign edits in full. To put them
back:

```bash
cp [role-home]/pavonis_claude_cp3631_backup/LiteXM2SDRStreaming.cpp.worktree-20260801 \
   [role-home]/CLionProjects/m2sdr/litex_m2sdr/software/soapysdr/LiteXM2SDRStreaming.cpp
```

Nothing was rebuilt, reinstalled, or reflashed. The installed `.so`, the kernel
module, the RF plugin, the srsenb binary, and all four configs were never
touched; each was verified byte-identical after the revert.

**Post-revert gate state, all nine green:**

```text
ok  srsenb-bin     ok  enb.conf     ok  soapy-module
ok  rf-plugin      ok  rr.conf      ok  soapy-source
ok  m2sdr.ko       ok  sib.conf     ok  rb.conf
```

### 2.4b Second drift: stale Soapy build artifact blocked OCUDU

After the 2.4 revert, run `20260805T091645Z` got much further. The entire primer
phase passed for the first time:

```text
srsenb_start              PASS   <- fixed by the 2.4 revert
phone_prepare             PASS
qcore_registration_wait   PASS   <- handset registered with qcore
srs_service_wait          PASS   <- handset had service on the primer
srsenb_stop               PASS
ocudu_start               <- 35-byte log, NEW first failure
```

Diagnostic capture confirmed real radio activity: `DLF_BYTES=2366`, 27 records
(`0xb821` x25, `0xb889` x1, `0xb88a` x1) timestamped 09:17:51-09:17:53.

OCUDU gates more artifacts than the primer, including **two** Soapy module
paths that must both equal the same hash:

| Gate | Status |
|---|---|
| starter, stage, gnb-bin, kmod, soapy-source, soapy-DEPLOYED | ok |
| `.../software/soapysdr/build/libSoapyLiteXM2SDR.so` | **MISMATCH** `a15642a2...` vs `81b8cb2e...` |

Same drift pattern as 2.4, one level deeper: the plugin had been **rebuilt in
the repo `build/` directory from the modified source but never installed**. The
deployed module stayed correct; the build artifact did not. OCUDU requires
`SOAPY_BUILD == SOAPY_DEPLOYED == 81b8cb2e...`, so it refused, silently.

**Changed on RAN host:**

```text
[role-home]/CLionProjects/m2sdr/litex_m2sdr/software/soapysdr/build/libSoapyLiteXM2SDR.so
```

Aligned by copying the already-correct deployed module over the stale build
artifact. Nothing was compiled. Copying was chosen over rebuilding because the
deployed module is the artifact CP3121 was actually validated against, and a
fresh compile is not guaranteed to reproduce its hash.

**Backup taken first:**

```text
[role-home]/pavonis_claude_cp3631_backup/
  libSoapyLiteXM2SDR.so.build-stale-a15642a2   459272 bytes, sha256 a15642a2...
```

That artifact is the build of the reverted-away source, so it is orphaned by
2.4 and is kept only for provenance.

**All seven OCUDU gates now pass:**

```text
ok  starter   ok  gnb-bin        ok  soapy-BUILD      ok  kmod
ok  stage     ok  soapy-source   ok  soapy-DEPLOYED
```

### 2.4c Complete backup inventory on RAN host

```text
[role-home]/pavonis_claude_cp3631_backup/
  LiteXM2SDRStreaming.cpp.worktree-20260801     188169 B  sha256 888cfbda...
  LiteXM2SDRStreaming.cpp.worktree.diff          88787 B  2090 lines vs HEAD
  libSoapyLiteXM2SDR.so.build-stale-a15642a2    459272 B  sha256 a15642a2...
```

Together these fully restore the pre-session RAN host state. Both belong to the
same later-campaign Soapy change; restoring the source without also restoring
or rebuilding the module would recreate the 2.4b mismatch.

### 1.8 CORRECTION: the roaming prompt IS a blocker, and the handset is on 5G

Open item 6 (below) said the roaming prompt was "expected and not a fault".
That was true for run `20260805T091645Z`, where `CP2857_SRS_SERVICE=PASS` with
the prompt present. It is **not** true in general. Run `20260805T092445Z` hung
at `srs_service_wait` for the full 180 s budget and failed with
`CP2857_SRS_SERVICE_SSH_RC=1`.

**The radio layer was perfect throughout.** `dumpsys telephony.registry` during
the hang:

```text
getRilDataRadioTechnology=20(NR_SA)      <- 5G standalone
mOperatorAlphaLong=QCore                 <- the private core
mDataRegState=0(IN_SERVICE)
CellIdentityNr: mPci=500 mTac=7 mNrArfcn=368410 mBands=[3] mMcc=901 mMnc=70
PS/WWAN: registrationState=ROAMING  availableServices=[DATA]
mIsDataRoamingFromRegistration=true
```

So the operator observation "the phone is on roaming not on the 5G like before"
is half right: the handset **is** on 5G SA on the private cell. `Roaming` is an
orthogonal flag, set because private PLMN `901/70` is not the SIM's home PLMN.
The two are not alternatives.

**The blocker is above the radio, in Android.** During the hang:

```text
mCurrentFocus=com.android.phone/com.android.phone.OplusDataDisconnectedRoamingActivity
data interfaces: none
settings get global data_roaming6 -> 1        <- roaming data already ENABLED
```

The Oplus "data disconnected while roaming" dialog had torn down the data
connection, so no `rmnet` interface existed for `srs_service_wait` to find. The
per-subscription roaming setting was already `1`, so **enabling the setting is
not sufficient**; this dialog requires an explicit affirmative tap and blocks
the PDU session until it gets one.

This explains the difference between the two runs: it is a race between the
dialog being answered and the 180 s service wait expiring, not a change in the
RAN.

**Mitigations, in order of preference:**

1. Answer the dialog affirmatively ("turn on") as soon as it appears during the
   primer phase.
2. Give more tap time: `PAVONIS_CP2874_SRS_SERVICE_WAIT_SEC` accepts 180-420 s.
3. Dismiss the dialog before starting a run and confirm no
   `OplusDataDisconnectedRoamingActivity` window is focused.

### 2.5 Stuck run recovered

Run `20260805T090041Z` was found live and silent. It had failed at
`srsenb_start` about 9 s in, entered `cleanup()`, and blocked in the
CLAUDE-CP3631 hold gate. Nothing was transmitting: `qcore` had started but the
primer never did, so no cell existed and the handset could not have had service.

Released with the sentinel file; teardown completed in about 5 s,
`CP2989_QCORE_STOP=PASS`, and both hosts returned to zero stack processes.

**Design note.** Holding on failure was deliberate, so a failed run can be
inspected before teardown. In practice it read as a hang: the runner is silent
by design and the hold banner scrolled past. If this fork is kept, the hold
banner should state the run's result and, on failure, name the first failed
stage, so a held failure is not mistaken for a healthy live cell.

### 2.6 Hold fork defect found and fixed: the hold never fired on SUCCESS

Run `20260805T115319Z` passed end to end (BASE_RC=0, echo rtt avg 66.053 ms)
but tore down immediately despite `PAVONIS_HOLD_BEFORE_TEARDOWN=1`. Root
cause: the hold block lived only at the top of `cleanup()`, which is an
EXIT/INT/TERM/HUP trap - but the success path tears down inline
(`ocudu stop` -> `phone restore` -> `qcore stop` -> packaging) and then clears
the trap (`trap - EXIT INT TERM HUP`) before exiting. So the hold engaged
exactly when it was least useful (failures, section 2.5) and never on a
healthy run. Env propagation through the four-script chain was verified
correct and was not the problem.

Fix in `cp3121h_run_continuous_srsenb_to_ocudu_metrics_rf.sh`: the hold block
was extracted into `hold_before_teardown_gate()` and is now called from two
places - the top of `cleanup()` (failure path, unchanged behaviour) and the
main flow immediately before the inline `"$OCUDU" stop` (success path). The
`hold_done` guard prevents double-holding when both paths execute.

The wrapper hash chain was re-pinned for the new content:

```text
cp3121h_run_continuous_srsenb_to_ocudu_metrics_rf.sh
  2e61b311... -> ac784e995d2baa6e03cf71c59648cfea0c764223437d7370e7df0976caa41884
cp3121h_run_filtered_phone_metrics.sh  (BASE_SHA256 re-pinned, own hash now)
  e9a83eba... -> adabf67c007fd533f66d9558250c70e85229ffc1b43e80eb377fb0de371a8636
cp3121h_run_metric_al8_phone_rf.sh     (BASE_SHA re-pinned, own hash now)
  11a3d666... -> f6c2fa2ca0de6c7a098994ce2b823bb3b7914adc471b8e7094b61706c1bd9601
cp3121h_execute_metric_al8_phone_rf.sh (RUNNER_SHA re-pinned)
```

Verified by run `20260805T120946Z`: full pass, then
`HOLDING BEFORE TEARDOWN ... rc=0` with the stack live (section 1.14 tests
were performed inside this hold). The CP3121 originals remain untouched.

### 2.7 Fast-shape fork: CP3612/CP3586 chain with hold, one provenance re-pin

To relaunch the CP3613-class shape without flipping lab state, the four-script
chain was forked in place ('h' suffix, originals untouched and re-verified
against their pins afterward):

```text
cp3586_phone_15mhz_1x1_ota_20260802/
  cp3586h_run_continuous_srsenb_to_ocudu_benchmark_rf.sh
    b3cbcd6df9fa85bee67095282d38a342f22bcf241a93c2377b3ec20da0b38ff7
    (adds hold_before_teardown_gate() on both teardown paths, as 2.6;
     release sentinel: cp3612h_${STAMP}_RELEASE)
  cp3586h_run_filtered_phone_15mhz_benchmark.sh
    a73e880d3b8393550f99f3ab3d8aac14e7a2b6ec2c39e1d467ce0c36ed029f48
cp3612_phone_15mhz_all_mcs17_no_rf_gate_20260802/
  cp3612h_run_phone_15mhz_all_mcs17_rf.sh
    c718b1f8e604cf03be887977c1de4ac4502fdbef02057606385148593d166575
  cp3612h_execute_phone_15mhz_all_mcs17_attempt.sh
    d7ae64d3e2e9f2b3b7739ea033f7d39638c5c012ecc9da4f7b0cd12f113981b7
```

**The one substantive change** (besides the hold and the pin re-chaining):
`cp3586h_run_filtered_phone_15mhz_benchmark.sh` re-pins
`PAVONIS_SOAPY_SOURCE_SHA_EXPECTED` from `888cfbda...` (the 2026-08-01 edited
worktree file the CP36xx campaign ran with) to `644e7858...` (the state the
2.4 revert restored). This is provenance-only: **both chains pin the identical
deployed module `81b8cb2e...`** - the code that actually runs never differed.
The source file on RAN host had been acting as a mode switch between the 10 MHz
and 15 MHz chains; the re-pin removes that conflict without touching RAN host.
The 08-01 source remains recoverable from
`[role-home]/pavonis_claude_cp3631_backup/` if the original 15 MHz chain must
ever be run bit-exact.

No RAN host or UE host file was modified for this fork; the execute deploys the
same pinned helpers the CP3612 original deploys.

### 2.8 Orphaned hold recovered after controller-session loss

The `20260805T123403Z` hold was orphaned when the controller laptop session
restarted: the local runner process (which watches the release sentinel and
performs teardown) died with the session. Found afterward: `gnb` already gone
(cell dead, M2SDR released, mode `N`), `qcore` still running orphaned, phone
never restored. Recovered manually with the run's own helpers:
`pavonis_cp2989_qcore_remote.sh stop 20260805T123403Z_QCORE` -> PASS and
`pavonis_cp2629_phone_remote.sh restore 20260805T123403Z_PHONE ...` -> PASS;
both hosts verified idle, no stale sentinels.

**Lesson:** the hold is only as durable as the local controller process. If
the laptop session dies mid-hold, the release sentinel has no watcher and the
remote stack orphans - recover with the two stop/restore helpers above (plus
the OCUDU stage stop helper if `gnb` is still up). Running the launch command
under `setsid`/`nohup` or inside `tmux` would make the hold survive session
loss.

### 1.9 FULL SUCCESS: run 20260805T093406Z completed the whole chain

After the 2.4 revert, the 2.4b build-artifact alignment, and clearing the
roaming dialog, the campaign ran end to end for the first time this session:

```text
srsenb_start          PASS      phone_n3_apply        ok
phone_prepare         PASS      ocudu_start           PASS   <- 2.4b fix
qcore_registration    PASS      target_activation     PASS
srs_service_wait      PASS      core_http_server      ARMED
srsenb_stop           PASS      ocudu_stop            PASS
```

Measured results, versus the frozen N20 campaign medians:

| Metric | This run | N20 median | N20 range |
|---|---:|---:|---|
| Time to attach | **19.982 s** | 22.504 s | 21.058-22.716 |
| Download | **616,096 B/s** | 786,864 B/s | 591,627-840,310 |
| Upload | **13,687 B/s** | 12,012 B/s | 10,997-14,084 |
| Echo RTT avg | **69.414 ms** | 71.201 ms | 63.878-78.791 |

`CP3121_PHONE_HTTP_PASS=1`, `CP3121_PHONE_ECHO_PASS=1`, 1 MiB transferred in
each direction on `rmnet_data1`, 4/4 echo replies. Attach and upload beat the
campaign median; download sits inside the N20 range. This is a valid N20-class
result.

Two caveats recorded honestly:

- `CP2880_DATA_PROBE_TRANSPORT phone_rc=0 core_rc=1`: the phone-side probe
  passed, the core-side probe failed. Not diagnosed.
- The OCUDU target was live for only **2 minutes 33 seconds** between
  `target_activation` and `ocudu_stop`. This runner is a measurement harness,
  not a demo platform.

### 1.10 There is no internet egress, and there never was

The operator recalled a run where the handset reached the internet. A search of
all 3,386 sanitized checkpoints plus the artifact tree found no such run.

Every occurrence of "internet" in the corpus is one of:

- the **APN/DNN name** `internet`, which is Android's default APN and what the
  handset requests from qcore (CP2393, CP2977-CP2980, CP3110);
- unrelated prose (CP0301 refers to the operator's own home connection; CP2627
  to Android app permissions).

Captured `iptables` from a 2026-06-29 OTA run shows NAT only for Docker
(`172.17.0.0/16 -> docker0`) and Tailscale (`ts-postrouting`), with no rule for
the UE subnet, and no checkpoint describes UE-to-WAN routing.

**CORRECTION, see 1.10b: that capture is stale. The live RAN host config does have
UE NAT.** The conclusion that no *validated* internet run exists still stands;
the conclusion that the plumbing is absent does not.

**Why it looks like internet on the handset:** the phone displays 5G SA,
operator `QCore`, an active data icon, APN `internet`, and moves real megabytes.
Every user-visible indicator reads "connected". The traffic simply terminates at
the private core.

This is consistent with what the reports claim. The data gate is explicitly
phone-to-core echo, HTTP round trip, 1 MiB down, 1 MiB up. No report in the
package ever claimed public internet access. The capability was never built, so
nothing is broken.

### 1.10b CORRECTION: UE internet egress is already plumbed on RAN host

Section 1.10 claimed the UE NAT did not exist. That was based on a captured
`host-state/iptables.txt` from **2026-06-29**, not on live state. Reading RAN host
directly contradicts it:

```text
net.ipv4.ip_forward = 1
default via 128.178.122.1 dev eno1 proto dhcp src [core N2 address]
eno1   [core N2 address]/24
veth2  10.255.0.1/24, 10.255.0.200/24, 10.255.0.201/24
10.255.0.0/24 dev veth2 proto kernel scope link src 10.255.0.1
10.255.0.2    dev veth0 proto static
iptables -t nat -S POSTROUTING:
  -A POSTROUTING -s 10.255.0.0/24 -o eno1 -j MASQUERADE
```

Forwarding is on, the UE subnet is NATed out the LAN interface, and a default
route exists. **The three things I said would need building are already built.**

What remains genuinely unverified:

1. **DNS.** No UE-facing DNS server configuration was located on the host. If
   qcore does not hand the handset a reachable resolver, name lookups fail while
   raw IP routing still works. This is the most likely reason a speedtest would
   show nothing even with the cell up.
2. **No campaign ever tested it.** Egress being plumbed is not the same as
   egress being proven. Every recorded data gate targets the core.

**Cheap way to settle it** during a hold window, in order, from the handset:

```text
ping 10.255.0.1     core-side gateway   -> proves the PDU session
ping 8.8.8.8        public IP, no DNS   -> proves NAT and routing
ping google.com     public name         -> proves DNS
```

Whichever step first fails names the missing piece. If step 2 passes and step 3
fails, it is purely a resolver problem and no routing change is needed.

The earlier advice to add MASQUERADE, enable forwarding, and add a route should
be disregarded. Doing so would have duplicated an existing rule.

### 1.11 Run 20260805T094726Z: handset dropped the cell entirely

The following hold-fork run failed differently and is worth distinguishing from
the roaming-dialog class.

```text
qcore_registration_wait  PASS
srs_service_wait         CP2857_SRS_SERVICE_SSH_RC=1   (failed, 420 s budget)
```

Cell side was healthy: `srsenb` and `qcore` both running on RAN host throughout.
Handset side had lost the network completely:

```text
mDataRegState=1(OUT_OF_SERVICE)
registrationState=NOT_REG_OR_SEARCHING / NOT_REG_SEARCHING
mOperatorAlphaLong=null
airplane_mode_on=0
mCurrentFocus=org.zwanoo.android.speedtest/...MainViewActivity
```

Distinct from section 1.8: there the handset was `IN_SERVICE` on the private
cell and an Android dialog blocked the PDU session. Here the handset was not
camped on any cell and did not re-acquire across four minutes of polling, with
the Speedtest app in the foreground issuing network requests that could not be
satisfied.

Not root-caused. Candidate contributors: residual state from the immediately
preceding run's `phone_restore`, the foreground Speedtest app, or insufficient
settle time between back-to-back runs. An airplane-mode cycle is the documented
recovery and was not observed to complete before the operator stopped.

**Practical guidance:** leave a settle gap between runs, close data-hungry
foreground apps before starting, and confirm the handset shows the private
operator before the service wait begins.

### 1.12 "Nothing works anymore" root cause: a second stack held the radio

After the operator ran other tooling, every campaign run failed instantly at
`RAN host_preflight` and the handset reported operator `null`. All eight deployment
hash gates still passed - nothing was broken. The cause was a second,
conflicting stack on RAN host: the OCUDU validation harness
(`scripts/run_ota_stage_template.sh --i-understand-rf-test`, launched from the
`m2sdr` checkout with `--duration 600`), whose parent bash respawned `qcore`
and an `ocudu/build-clion` `gnb` when probed.

Two independent consequences, together explaining every symptom:

1. It broadcast **MCC 001 / MNC 01** while the handset SIM is provisioned for
   **901 / 70**, so the handset could never attach to it - hence
   `OUT_OF_SERVICE`, operator `null`.
2. It held the M2SDR, so campaign runs failed `RAN host_preflight` with an empty
   log before transmitting anything.

Resolution: a remote `kill` was blocked by the controller-side permission
classifier; the harness's own `--duration 600` expiry was allowed to run its
teardown instead, and idle state was verified afterward. **The validation
harness and the campaign runner are mutually exclusive** - they contend for the
one M2SDR, and only the campaign stack (PLMN 901/70) can serve the handset.

**1.12b Correction - the harness respawned itself twice more.** Letting the
`--duration 600` expire was not enough: two detached keeper loops,
`[role-home]/codex_stage6_prearmed_manifest_fast_RAN host_keep_60m_runtime.sh`
(both parented to init), each relaunched the harness with a fresh
`codex_stage6_prearmed_manifest_fast_keep_60m_*` run directory. The operator
confirmed the harness was outdated and cleared it for removal. The keeper
loops were TERMed first, then `run_tx.sh`, the stage template, `gnb`, and
`qcore` (the lingering template bash needed SIGKILL after ignoring two TERMs
in `do_wait`). Verified afterward: no stack processes, M2SDR program mode `N`.
One lesson recorded: when a stack reappears with new PIDs, hunt the topmost
respawner via the parent chain before killing children.

### 1.13 The handset's data-roaming toggle was found reset to off

Before relaunching, `settings get global data_roaming6` returned `0` (it was
`1` during the successful run in section 1.9). The private cell registers as
roaming (section 1.8), so with this off the handset attaches but passes no
data - a second, independent contributor to "nothing works". `settings put
global data_roaming6 1` over adb was denied
(`SecurityException: WRITE_SECURE_SETTINGS`; this Oplus build does not grant
it to shell), so the toggle must be restored from the handset UI or by
answering "turn on" on the roaming dialog when it reappears. `svc data enable`
(mobile data itself) is permitted and was applied.

### 1.14 RESOLVED: full internet egress from the handset, measured

Run `20260805T120946Z` (held open with the fixed hold gate, section 2.6)
completed the whole chain with rc=0, then during the hold the handset was
driven over adb from the UE host host. Every layer passed:

```text
ping 10.255.0.1 (PDU anchor)   3/3, rtt avg 79.1 ms
ping 8.8.8.8    (NAT egress)   3/3, rtt avg 80.0 ms
ping google.ch  (DNS + egress) 3/3, rtt avg 76.1 ms
curl 10 MB download (real internet, HTTPS)  988,171 B/s  (~7.9 Mbit/s, 10.1 s)
curl 512 KB upload  (real internet, HTTPS)   19,861 B/s  (~0.16 Mbit/s, 26.4 s)
```

The run's own in-network HTTP probe measured 804,015 B/s down / 11,478 B/s up,
attach 20,637 ms - so real-internet throughput matches the N20 radio shape's
ceiling; nothing upstream of the core is the bottleneck. This closes the
section 1.10 open item: egress and DNS both work with the RAN host
MASQUERADE path, and the resolver handed to the UE resolves public names.

### 1.15 The Ookla Speedtest app cannot complete on this network

Two attempts from the app (`org.zwanoo.android.speedtest`, launched and driven
over adb) both ended in "Test failed to complete. Please check your
connection and try again." while curl HTTPS transfers ran fine at the same
moment. The app's server-negotiation/latency phase evidently does not survive
this network shape (likely the ~11-20 KB/s uplink starving its control
channel, or its non-HTTPS transport). **Demo guidance: use fast.com in the
browser or a plain HTTPS download to show throughput; do not build a demo
around the Ookla app.**

**1.15b CORRECTION - the app works on the fast shape.** On the 15 MHz MCS17
stack (section 1.16) the Ookla app completed on the first attempt:
25.1 Mbps down / 1.61 Mbps up, idle ping 105 ms, loaded ping 1375 ms (DL) /
331 ms (UL), jitter 67 ms, server SyselCloud Mont-sur-Lausanne, status bar on
5G(R). The 10 MHz failure class was the shape's ~11-20 KB/s uplink starving
the app's control channel, exactly as hypothesized. Demo guidance updated:
the app is fine on `fast`; avoid it on `slow`.

### 1.16 Fast shape (CP3613-class, 15 MHz MCS17) revalidated end to end

Run `20260805T123403Z`, launched via the forked chain of section 2.7 with the
fixed hold gate. Phone lifecycle reset (fresh epoch, SIM power cycle, policy
audit) all passed; primer service, target activation, and both data probes
passed; the run held with rc=0 and the following was measured during the hold:

```text
in-run sustained benchmark 45 s   DL 2,425,101 B/s (19.4 Mbit/s), UL 54,468 B/s
echo over target                  4/4, rtt avg 96.114 ms
ping 10.255.0.1 / 8.8.8.8 / google.ch   3/3 each, rtt avg ~105 ms
curl 20 MB real-internet download 2,622,826 B/s (21.0 Mbit/s, 7.6 s)
Ookla Speedtest app               25.1 Mbps down / 1.61 Mbps up (completed)
```

Known property of this shape (recorded at CP3613/CP3625): run-to-run DL lands
anywhere between ~11 and ~25 Mbit/s because the 20 ms TX lead is visible to
the MAC as HARQ feedback latency. All three measurements here (19.4 sustained,
21.0 curl, 25.1 Ookla) sit in the healthy upper band. The latency cost versus
the 10 MHz shape is real: ~96-105 ms RTT against ~64-80 ms.

`CP2880_DATA_PROBE_TRANSPORT core_rc=1` persists on this shape too - same
undiagnosed core-side probe failure as every 10 MHz run.

## 3. Verification performed

| Check | Result |
|---|---|
| All 10 package manifests | PASS |
| `./pavonis-supervisor.sh verify` | exit 0 (was 1) |
| `./pavonis-demo.sh verify` | exit 0 |
| Planted 15-digit IMSI canary | caught, exit 1 |
| Privacy regex, 10 focused cases | 10/10 |
| Privacy patch revert to original bytes | exact |
| Privacy scan over 10 new CP3630 files | 0 findings |
| Demo controller regression suite (mock adapter) | 14/14 |
| Fork syntax, all four scripts | PASS |
| Fork sequence probe | `CP3121_SEQUENCE_PROBE=PASS` |
| Fork RF gate closed by default | refuses without approval |
| Hold knob validation | rejects `2`, `abc`, `30`, `99999`; accepts `600` |
| Hold disabled path | no hold, teardown reached |
| Hold released by sentinel | released, sentinel removed |
| Hold released by timeout | released at 60 s |
| Hold released by Ctrl-C | released, teardown reached |
| RAN host revert: restored source hash | `644e7858...4e76`, exact match |
| RAN host revert: module unchanged | `81b8cb2e...f97c`, exact match |
| All 9 primer gates after revert | 9/9 ok |
| Primer actually starts (run `...091645Z`) | `CP2985_..._START=PASS` |
| Handset registers via primer | `CP2857_QCORE_REGISTERING=PASS` |
| Handset gets service via primer | `CP2857_SRS_SERVICE=PASS` |
| Real radio activity captured | 27 diag records, 2366 B |
| RAN host build artifact aligned | `81b8cb2e...f97c`, exact match |
| All 7 OCUDU gates after align | 7/7 ok |
| Stuck run released and torn down | `CP2989_QCORE_STOP=PASS`, both hosts 0 procs |

**No RF was transmitted by any action I took.** Every lab interaction of mine
was a read-only probe, one file restore, and one teardown. The demo controller
was exercised only against a mock adapter. The operator-initiated runs
`20260805T084531Z` and `20260805T090041Z` both failed before the primer started,
so neither produced a cell.

**Not yet verified: that the primer actually starts.** The nine gates are the
precondition, not the proof. The next run is the first real test.

## 4. Two defects found by testing, before any RF

Both were in code I wrote, both found by the mock-adapter suite:

1. `${VAR:+NAME=1}` used as an inline environment prefix expands *after* bash
   parses the assignment list, so it became the command name. The `record`
   profile silently failed every phase. Fixed by exporting in a subshell.
2. `PAVONIS_DEMO_UNCLEAN_APPROVED` leaked from the caller's shell into
   strict-clean profiles. After one `record` run, a later `fast` run would have
   handed the adapter permission to apply a 13 ms transmit lead to a shape that
   must not have one. Fixed by scoping the export to the `record` profile.

## 5. Open items

0. **RESOLVED - internet egress from the handset is proven** (section 1.14):
   PDU anchor, NAT egress, and DNS all pass from the phone, with real-internet
   HTTPS throughput matching the radio shape's ceiling. Remaining sub-item:
   `CP2880_DATA_PROBE_TRANSPORT core_rc=1` (core-side probe) still fails on
   every otherwise-passing run and is undiagnosed.
1. **RESOLVED by revert.** The Soapy pin question is closed: the source was
   reverted to the CP3121-matching version (section 2.4) rather than the gate
   being overridden. The hash-override lines in
   `cp3121h_run_metric_al8_phone_rf.sh` are now inert, because
   `cp3121h_run_filtered_phone_metrics.sh` hardcodes the values downstream.
   They should be deleted on the next edit to avoid implying an override is
   active.
1b. **The later-campaign Soapy edits are parked, not lost.** The 2090-line
   working-tree diff is preserved in `[role-home]/pavonis_claude_cp3631_backup/`.
   Reapplying it requires rebuilding and reinstalling the `.so` and re-pinning
   `PAVONIS_SOAPY_SOURCE_SHA_EXPECTED` / `..._MODULE_SHA_EXPECTED` in
   `cp3121_run_filtered_phone_metrics.sh` (or a fork of it). Doing that is what
   the later 15 MHz campaign would need; it is not needed for the 10 MHz
   baseline demo.
2. **`pavonis-demo.sh` has no adapter.** Its job is what
   `cp3121h_execute_metric_al8_phone_rf.sh` already does, split into the six
   contract phases.
3. **15 MHz runners are missing locally.** Check RAN host before quoting any number
   above the 10 MHz baseline.
4. **`history/` stops at CP3424** while the campaign ran to CP3628; later
   records live in `checkpoints/` and the report revisions.
5. **No SSH key auth from the laptop**; the pexpect password wrapper is the only
   path.
6. **SUPERSEDED by section 1.8 - the roaming prompt IS a blocker.** The text
   below was written after run `20260805T091645Z` and was wrong as a general
   claim. Retained unchanged for the record. The handset shows
   "SIM1 is roaming, cancel or turn on" because the private lab PLMN is not in
   its home-operator list, so Android classifies the cell as roaming. Run
   `20260805T091645Z` still reached `CP2857_QCORE_REGISTERING=PASS` and
   `CP2857_SRS_SERVICE=PASS` with the prompt present, so it did not block
   registration or service. It is left on screen because the run tears down
   before dismissing it. If a future run stalls at registration, allowing
   roaming for that SIM is the first thing to check, but nothing so far
   indicates it is blocking.
7. **Two drifts found, both the same shape.** A later-campaign Soapy change was
   half-applied: source edited (2.4) and build artifact rebuilt (2.4b), but the
   module never installed. Each gate caught its own half, silently, one run
   apart. A single "is this deployment self-consistent" preflight that reports
   *which* artifact diverged would have found both in one pass and is worth
   adding.

## 6. Privacy

No credential, subscriber identity, device identity, host identity, network
address, NAS payload, or user payload appears in this file. Host role names and
file paths already present in the working tree are reproduced only where needed
to identify a file. The hashes listed are of scripts and binaries, not of
private material.
