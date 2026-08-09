# CLAUDE-CP3631 - In-person demo TLDR (validated 2026-08-05)

Both recipes below were run end-to-end today and held open for live phone use.
Full detail: `CLAUDE_CP3631_LAB_BRINGUP_AND_PACKAGE_CORRECTIONS.md` §1.14-1.16, §2.6-2.7.

## 0. Preflight (2 min)

```bash
ART=[controller-harness]
# RAN host must be idle - THIS is what killed everything today:
python3 "$ART/ssh_pexpect_run.py" --host RAN host --out /tmp/pf.log --timeout 30 -- bash -lc \
  'pgrep -a -f "[k]eep_60m|[r]un_tx|[r]un_ota_stage_template|[q]core|[g]nb|[s]rsenb" || echo IDLE; cat /sys/module/m2sdr/parameters/dma_reader_program_mode'
cat /tmp/pf.log   # want: IDLE and N
```

- Anything listed → kill the **topmost** parent first (keeper loops respawn children).
- The OCUDU validation harness (`run_ota_stage_template.sh`) broadcasts MCC 001/01:
  the phone can NEVER attach to it, and it hogs the radio. It is mutually
  exclusive with everything below.
- Phone check: `adb devices` on UE host shows `[device-redacted]`.

## 1. Pick a shape and launch (one command each)

Both hold the stack up after a full pass (`PAVONIS_HOLD_*`), so you can demo
by hand. Timings: **slow ≈ 8 min** to holding; **fast ≈ 18 min** (it reboots
the phone and power-cycles the SIM first - that lifecycle reset is why it's
reliable; don't shortcut it).

**SLOW - 10 MHz MCS9 (the N20 baseline). ~8 Mbit/s down, 64 ms RTT.**
```bash
cd "$ART" && env STAMP=$(date -u +%Y%m%dT%H%M%SZ) \
  PAVONIS_HOLD_BEFORE_TEARDOWN=1 PAVONIS_HOLD_MAX_SEC=7200 \
  bash "$ART/cp3121h_execute_metric_al8_phone_rf.sh"
```

**FAST - 15 MHz MCS17 (CP3613 class). 19-25 Mbit/s down, ~100 ms RTT.**
```bash
cd "$ART" && env STAMP=$(date -u +%Y%m%dT%H%M%SZ) \
  PAVONIS_CP3612_EXECUTE_RF_APPROVED=1 \
  PAVONIS_HOLD_BEFORE_TEARDOWN=1 PAVONIS_HOLD_MAX_SEC=7200 \
  bash "$ART/cp3612_phone_15mhz_all_mcs17_no_rf_gate_20260802/cp3612h_execute_phone_15mhz_all_mcs17_attempt.sh"
```

(Medium 15 MHz MCS11 exists at `cp3605_*/` but has no hold fork yet - apply
the §2.7 recipe if needed. The `record` 13 ms-lead shape fails producer
health: do not demo it.)

Wait for the banner:
```
HOLDING BEFORE TEARDOWN - the stack is still up.  rc=0
```
**Check rc.** `rc=0` = healthy cell, go demo. `rc!=0` = the run FAILED and is
holding for inspection - there is no working cell, don't demo it.

## 2. During the demo

- If the phone shows the **"SIM1 is roaming" popup → tap "Turn on"**. The
  private PLMN registers as roaming; Cancel kills data (this bit us today).
- Phone should show **5G(R)** and get `10.255.0.2`. Full internet works:
  browsing, `google.ch`, DNS - qcore NATs out via RAN host.
- **Speedtest app: fast shape only** (today: 25.1 down / 1.61 up). On slow its
  uplink starves the app's control channel and it errors - use **fast.com** or
  a plain HTTPS download there.
- Talking points: DL ceiling moves 8→25 Mbit/s between shapes; latency is the
  price of the fast shape's 20 ms TX lead (64→100 ms); fast-shape DL varies
  run-to-run 11-25 Mbit/s by design (HARQ latency), so quote "up to 25".

## 3. Teardown when done

```bash
touch "$ART/cp3121h_<STAMP>_RELEASE"    # slow
touch "$ART/cp3612h_<STAMP>_RELEASE"    # fast
```
(exact path is printed in the banner; auto-release after 2 h). Teardown takes
~2 min and restores the phone. Verify idle with the §0 check before any next run.

**The hold dies with your terminal.** Run the launch inside `tmux` (or under
`setsid`) if there is any chance the session closes - if the controller
process dies mid-hold, nothing watches the sentinel and the remote stack
orphans. Recovery (with the run's STAMP):
```bash
python3 "$ART/ssh_pexpect_run.py" --host RAN host --out /tmp/q.log --timeout 90 -- \
  [role-home]/pavonis_cp2989_qcore_remote.sh stop "<STAMP>_QCORE"
python3 "$ART/ssh_pexpect_run.py" --host UE host --out /tmp/p.log --timeout 240 -- \
  [role-home]/pavonis_cp2629_phone_remote.sh restore "<STAMP>_PHONE" [device-redacted] 6
```
then the §0 idle check. (This happened on 2026-08-05; catalog §2.8.)

## 4. If it breaks

| Symptom | Cause seen today | Fix |
|---|---|---|
| Instant fail at `RAN host_preflight` | something else holds the M2SDR | §0 check, kill topmost parent |
| Phone `OUT_OF_SERVICE`, operator null | MCC 001/01 harness broadcasting | kill it (§0) |
| Attached but no data | roaming toggle off | popup "Turn on", or Settings → SIM1 → Data roaming |
| Fails after back-to-back runs | phone state accumulation | use fast (has lifecycle reset), or wait ~2 min between runs |
| "Stuck", no output for minutes | it's HOLDING (check for the banner) | read rc; release sentinel to tear down |
