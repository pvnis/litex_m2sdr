# srsUE ⇄ OCUDU over Band‑78 SCS‑30 (TDD): Findings & Remaining Gap

**Status:** 2026‑06‑03. Goal: get **srsRAN_4G srsUE** to attach to the **OCUDU (srsRAN Project) gNB** running on a **LiteX‑M2SDR**, band 78, **SCS‑30 kHz, TDD**, validated over **ZMQ** (sample‑clocked, no RF artifacts) before moving to real RF.

This documents the root causes found and fixed, plus the precise remaining gap. The underlying theme: **srsRAN_4G's NR stack was built and tested for SCS‑15 / FDD, so the LTE‑derived code reuses SCS‑15 (slot == subframe, 15 kHz RB) assumptions that break at SCS‑30 (2 slots/subframe, 30 kHz RB, 2× sample rate).**

---

## TL;DR progress

The UE now goes: **cell search → MIB → SIB1 (CRC=OK) → PRACH detected → RAR (Msg2) → Msg3 PUSCH transmitted & received by the gNB**. The chain of SCS‑30 bugs below was fixed one layer at a time, each verified. **Next blocker: Msg3 PUSCH decode — it reaches the gNB but at low SINR (≈‑16 dB, crc=KO) due to a residual ~75 µs UL timing offset.**

A **clean FDD band‑3 (10 MHz, SCS‑15) ZMQ baseline completes RACH fully** (Msg1→Msg2→Msg3→Msg4), which proved the format‑0 PRACH, OCUDU's detection, and the ZMQ UL path all work — isolating every remaining failure to SCS‑30/TDD specifics.

---

## Environment / how to reproduce

- gNB (TDD band 78): `ocudu-pavo/configs/m2sdr_zmq.yml` — ZMQ, band 78, 20 MHz, `common_scs: 30`, `prach_config_index: 0` (see "PRACH format" note), `phy_level: debug`, `mac_level: debug`, `broadcast_enabled: true` (logs every PRACH opportunity).
- UE (TDD band 78): `/home/dmd/ue-zmq.conf` — ZMQ, band 78, 51 PRB, `dl_nr_arfcn 626000`, `ssb_nr_arfcn 625632`, `scs 30`.
- FDD baseline gNB: `ocudu-pavo/configs/m2sdr_fdd_zmq.yml` (10 MHz, SCS‑15, `prach_config_index: 1`). UE: `/home/dmd/ue-fdd-zmq.conf`.

**Run pattern (important — see "testing traps"):**
```bash
pkill -9 -x gnb; pkill -9 -x srsue; sleep 3          # exact-name kill (see trap #1)
rm -f /tmp/gnb_zmq.log /tmp/ue-zmq.log
cd ocudu-pavo/build; ./apps/gnb/gnb -c ../configs/m2sdr_zmq.yml >/tmp/gnb.out 2>&1 &
sleep 6                                               # UE must connect quickly (gNB starves w/o RX in ~10s)
srsRAN_4G/build/srsue/src/srsue /home/dmd/ue-zmq.conf >/tmp/ue.out 2>&1 &
sleep 26; pkill -9 -x srsue; pkill -9 -x gnb
```

---

## Fixes (each root‑caused and verified)

### 1. SIB1 PDSCH never decoded — OFDM cyclic‑prefix layout (the multi‑day blocker)
- **Symptom:** UE camped, decoded MIB + SIB1‑PDCCH (DCI corr 1.00), but SIB1 **PDSCH failed: CRC=KO, evm≈0.46**, even over clean ZMQ (snr +69).
- **Root cause:** `lib/src/phy/dft/ofdm.c` builds the 14‑symbol slot as **two 7‑symbol halves** (`SRSRAN_CP_NSYMB(NORM)=7`), each with a long CP at its start → long CP at symbols **0 and 7**. Correct for SCS‑15 (μ=0: long CP at l=0 and l=7); **wrong for SCS‑30 (μ=1: long CP only at l=0)**. The spurious long CP at symbol 7 shifts the FFT window of symbols 7–13 by `cp1−cp2 = 6` samples → a linear phase ramp across subcarriers. SIB1 DMRS is at symbols 2/7/11: symbol 2 stayed flat, 7 & 11 ramped → the time‑averaged channel estimate tilted → all PDSCH data decoded against a tilted estimate. PDCCH/SSB/MIB (symbols 0–1) never cross symbol 7, which is why only the PDSCH failed.
- **Fix:** numerology‑aware CP layout. Added `nr_scs_hz` to `srsran_ofdm_cfg_t`; in the guru init, for `nr_scs_hz > 15000` the second 7‑symbol half advances by the uniform normal‑CP stride (`slot_advance = SRSRAN_CP_NSYMB(cp) * (symbol_sz + cp2)`) instead of `slot_sz`, so symbols 7–13 keep the continuous CP layout. Also fixed the phase‑comp table SCS (was hardcoded 15 kHz) and its CP index. Plumbed `nr_scs_hz = SRSRAN_SUBC_SPACING_NR(carrier->scs)` via `ue_dl_nr.c` set_carrier.
  - **Plumbing trap:** `ofdm_init_mbsfn_` has a "already‑initialised (q->max_prb>0)" branch that copies only cp/nof_prb/symbol_sz on a resize re‑init — had to add `q->cfg.nr_scs_hz = cfg->nr_scs_hz;` there (set_carrier re‑inits via this path) or the field is silently dropped. Also made set_carrier re‑init when `carrier->scs` changes, not just nof_prb.
- **Files:** `lib/src/phy/dft/ofdm.c`, `lib/include/srsran/phy/dft/ofdm.h`, `lib/src/phy/ue/ue_dl_nr.c`.
- **Verified:** DMRS sym 7/11 flat, `ce` flat, **SIB1 PDSCH CRC=OK, evm=0.00, snr +69**, RRC "SIB1 received".

### 2. RACH config parse aborted — `l139` ASN.1 missing `break;`
- **Root cause:** `lib/src/asn1/rrc_nr_utils.cc:429` — the `l139` (short‑format root sequence) case set `root_seq_idx` but had **no `break;`**, falling through to the `default:` "Not‑implemented" path which `return false`. So the UE could not parse `rach-ConfigCommon` and never started RACH (stuck re‑reading SIB1).
- **Fix:** add `break;`.
- **Verified:** UE proceeds "Preparing RRC Setup Request → Starting PRACH by MAC order → transmits Msg1".

### 3. PRACH fired at the wrong slot — occasion timing (slot vs subframe)
- **Root cause:** `srsran_prach_nr_tti_opportunity_fr1_unpaired` uses `sf_idx = current_tti % 10` (subframe‑indexed). The slot‑granular camping rework makes `tti` **slot‑indexed (20/frame)** at SCS‑30, so `%10` picked the wrong subframe. At SCS‑15 slot==subframe so it's correct (FDD works).
- **Fix:** added `mu` to `srsran_prach_t`/`srsran_prach_cfg_t`; in `srsran_prach_tti_opportunity`, for NR convert `sf_tti = current_tti >> mu` and only fire on `current_tti % (1<<mu) == 0` (first slot of the PRACH subframe). No‑op at μ=0.
- **Files:** `lib/src/phy/phch/prach.c`, `prach.h`, `srsue/src/phy/nr/worker_pool.cc` (`prach_cfg.mu = carrier.scs`).
- **Verified:** UE fires Msg1 on even slots at the correct config‑0 occasion (sfn%16==1, slot 18).

### 4. PRACH preamble half‑duration — sample‑rate (μ scaling)
- **Root cause:** the LTE PRACH library generates the preamble at the **SCS‑15 reference rate** (`N_ifft_ul = srsran_symbol_sz(cell.nof_prb)`). Measured `len=10404, N_seq=9216 (=768×12)` = ~451 µs at the SCS‑30 carrier's 23.04 MHz, but format‑0 needs ~900 µs. So the preamble was **half‑duration / double‑SCS → undetectable**.
- **Fix:** scale the working IFFT size by `2^mu` in `srsran_prach_set_cell_` (`N_ifft_ul <<= cfg->mu`), and give the init 2× headroom (`srsran_prach_init(..., MIN(2048, srsran_symbol_sz(max_prb)*2))` in `srsue/src/phy/prach.cc`). No‑op at μ=0.
- **Verified:** UE PRACH `nof_sf=2` (full 1 ms, spread across 2 SCS‑30 slots by the existing chunk loop).

### 5. PRACH landed 4 slots late — DL→UL TX advance (slot vs ms) — *the "ZMQ TDD UL alignment" bug*
- **Symptom:** UE radiated the preamble at **full power** at the right slot index, but the gNB's detector at the correct occasion saw `rssi=-inf` (zero energy).
- **Root cause:** in the slot‑granular rework `srsue/src/phy/sync_sa.cc run_state_cell_camping`, the DL→UL TX advance was added as `FDD_HARQ_DELAY_DL_MS * 1e-3` (**4 ms**), but `TTI_TX` advances the opportunity by `FDD_HARQ_DELAY_DL_MS` **slots** (4). At SCS‑15, 4 slots = 4 ms (consistent); at **SCS‑30, 4 ms = 8 slots**, so the preamble for opportunity slot N was radiated at the time of slot N+4 and landed **4 slots past the gNB RX window**.
- **Fix:** advance by 4 *slots* of time: `slot_tx_time.add((double)FDD_HARQ_DELAY_DL_MS * (double)slot_len / srate_hz)` — 4 ms at SCS‑15, 2 ms at SCS‑30.
- **Verified:** gNB PRACH energy went `rssi=-inf → -98 dB` and the detector began correlating (`t 2.7µs → 308µs`) — preamble now arrives in the right slot.
- **Diagnosis aids:** UE side `SRSUE_PRACH_TX_DBG=1` (env, logs PRACH max‑abs + tx_time + sf_idx). gNB side `broadcast_enabled: true` (logs every opportunity: `[PHY] PRACH: rsi=.. rssi=.. detected_preambles=[..]`; the OCUDU detector decorator `lib/phy/.../prach/factories.cpp:155` only logs on detection otherwise).

### 6. PRACH at the wrong frequency — `k_0` IFFT‑bin placement (μ / RB‑size)
- **Symptom:** with timing fixed, gNB rssi sat at the noise floor (`-98 dB`; FDD was +5 dB), `detected_preambles=[]` — preamble in the right slot but the wrong frequency band.
- **Root cause:** `srsran_prach_gen` (prach.c:768) places the preamble at IFFT bin `k_0 = freq_offset*N_RB_SC - srsran_nof_prb(N_ifft_ul)*N_RB_SC/2 + N_ifft_ul/2` using the **SCS‑15 RB grid** (12 sc/RB at 180 kHz) and `srsran_nof_prb(N_ifft_ul)` for centering. An SCS‑30 NR RB is 360 kHz (24 sc in the 15 kHz IFFT grid), and the NR carrier doesn't fill the doubled FFT, so the preamble ended up at ~half the intended frequency offset → outside the gNB PRACH band.
- **Fix:** for `is_nr && mu>0`, compute `k_0 = freq_offset*(N_RB_SC<<mu) - (carrier_nof_prb*(N_RB_SC<<mu))/2 + N_ifft_ul/2` using the real NR carrier PRB count (plumbed `carrier_nof_prb = carrier.nof_prb`). No‑op at μ=0.
- **Files:** `lib/src/phy/phch/prach.c`, `prach.h`, `srsue/src/phy/nr/worker_pool.cc`.
- **Verified:** **gNB DETECTS the preamble** — `rssi=+7.8 dB, detected_preambles=[{idx=0 ... detection_metric=2.5}]`, and the gNB transmits the **RAR (Msg2)** `RAR PDSCH ra-rnti=0x7f tc-rnti=0x4601`.

### 7. RA‑RNTI mismatch — t_id (slot vs subframe)  *(in progress at time of writing)*
- **Symptom:** gNB detects the preamble and sends the RAR on `ra-rnti=0x7f` (=127 ⇒ t_id=9), but the UE searches `ra-rnti` computed with **t_id=18** → it never sees the RAR (`RAR Timer expired`).
- **Root cause:** RA‑RNTI = `1 + s_id + 14*t_id + ...` (`proc_ra_nr.cc:355`). `t_id` is the PRACH‑occasion slot; for a long‑format preamble (one occasion per subframe) it is **subframe‑granular**. The UE passed the slot index (18); the gNB uses the subframe index (9 = 18>>μ). Same slot‑vs‑subframe family as #3/#5.
- **Fix (applied):** in `worker_pool.cc` prach_sent, `t_id = SRSRAN_SLOT_NR_MOD(scs, TTI_TX(tti)) >> mu`.
- **Verified:** UE now **receives the RAR (Msg2)** — `DL RAPID: 0, Temp C-RNTI: 0x4601, TA: 288, UL Grant: [...]`, matching the gNB's `tc-rnti=0x4601`.

### 8. Msg3 scheduled one slot early — PUSCH numerology (Msg3 Δ)
- **Root cause:** `rrc_nr_procedures.cc` sets `phy_cfg.pdsch.scs_cfg = mib.scs_common` but never set `phy_cfg.pusch.scs_cfg`, so it stayed 0 (SCS‑15). The Msg3 RAR‑grant slot delay Δ (TS 38.214 Table 6.1.2.1.1‑5: Δ=2/3/4/6 for μ=0/1/2/3) is indexed by `cfg->scs_cfg`, so it used Δ=2 instead of Δ=3 → `k = k2(4)+2 = 6` → Msg3 landed on slot 696 (the TDD **special** slot, no UL) instead of 697.
- **Fix:** add `phy_cfg.pusch.scs_cfg = mib.scs_common;`.
- **Verified:** `SET-UL-GRANT slot_rx=690 k=7 -> tti_tx=697` (UL slot 17).

### 9. UE never transmits scheduled UL — TDD pattern not applied to PHY (`period=0`)
- **Root cause:** `rrc_nr.cc handle_sib1` applied PDSCH/PUSCH/PUCCH/RACH/PDCCH/carrier/SSB configs but **never applied `tdd-UL-DL-ConfigCommon` to `phy_cfg.duplex`** (it only read `tdd_ul_dl_cfg_common_present` for the RACH duplex mode). So `phy_cfg.duplex.tdd.pattern1.period_ms` stayed 0 → `srsran_duplex_nr_is_ul()` returned false for every slot → `cc_worker::work_ul` returned before fetching the grant → no Msg3 (and no PUCCH/PUSCH ever). PRACH worked because it bypasses the duplex check; DL worked because of the earlier `tdd_nr_is_dl` period==0→true workaround.
- **Fix:** call `make_phy_tdd_cfg(sib1...tdd_ul_dl_cfg_common, &phy_cfg.duplex)` in `handle_sib1` when present.
- **Verified:** `WORK-UL ul_idx=697 is_ul=1 period=5`, `GET-UL-GRANT tti_tx=697 HIT`, and the gNB now receives the Msg3 PUSCH with real signal (sinr `inf → -16 dB`).

### 10. Time‑alignment 2× too large — TA command not numerology‑scaled
- **Root cause:** `ta_control::add_ta_cmd_rar/new` used `srsran_N_ta_new_rar(ta_cmd) (= ta_cmd*16) * SRSRAN_LTE_TS` — the SCS‑15 reference with no μ scaling. NR `N_TA = ta_cmd·16·64/2^μ` (TS 38.213 §4.2), so at SCS‑30 the TA was 2× too large (~163 µs vs ~75 µs) → Msg3 over‑advanced.
- **Fix:** added a `numerology` to `ta_control` (set from carrier SCS in `sync_sa.cc`); scale **only the command contribution** by `>> μ` (the fixed `N_TA_offset` base from `add_ta_offset` is μ‑independent and must not scale).
- **Verified:** `Set TA RAR ... ta_usec ≈ 88` (75 µs command + 13 µs offset). Msg3 SINR improved slightly but still ‑16 dB → the dominant remaining error is the ~75 µs residual UL offset, not the TA magnitude (see "Current result").

---

## Current result (2026‑06‑04, after fixes #1–#10)

**Msg1 → Msg2 → Msg3‑reaches‑gNB over ZMQ, SCS‑30 band‑78 TDD:**

```
gNB:  PRACH detected_preambles=[{idx=0 ta=75us}]  →  RAR PDSCH tc-rnti=0x4601 ta=288
UE:   DL RAPID:0, Temp C-RNTI:0x4601, TA:288, UL Grant   ← RAR received
UE:   WORK-UL ul_idx=697 is_ul=1   GET-UL-GRANT tti_tx=697 HIT   ← Msg3 generated in UL slot 17
gNB:  PUSCH: rnti=0x4601 prb=[8,11) ... crc=KO sinr=-16dB         ← Msg3 reaches gNB, decode fails
```

Chain status: **cell search ✓ · MIB ✓ · SIB1 ✓ · Msg1 PRACH ✓ · Msg2 RAR ✓ · Msg3 PUSCH reaches gNB (sinr ≈ ‑16 dB, crc=KO) · Msg4 — not reached.**

**Remaining Msg3 blocker — two interrelated precise‑alignment issues (long‑format PRACH workaround over SCS‑30 TDD).** The gNB receives the Msg3 PUSCH at the right slot/PRBs but at `sinr≈‑16 dB, crc=KO`. Two findings from a UL‑advance sweep (`SRSUE_UL_ADV_US`, env‑gated diagnostic in `sync_sa.cc`):

1. **~75 µs systematic UL timing offset.** The gNB detects the PRACH at `ta=75 µs` (FDD baseline was `ta=0`). Adding **‑75 µs** to the camping `tx_time` advance moves it to `ta=0.00 µs`. 75 µs ≈ 2 SCS‑30 OFDM symbols ≈ the SSB start‑symbol offset (Case C, SSB at symbol 2), so the UE's UL timing reference is likely ~2 symbols off — probably the slot‑sync establishing `last_rx_time` relative to the SSB position rather than the slot boundary, or a sub‑slot residual in the slot‑granular UL `tx_time`. (Note the captured `rx` times also carry a constant +13 µs = 300 samples = the FR1 N_TA_offset of 25600 Tc.)
2. **Weak PRACH correlation.** Even at `ta=0`, the gNB's `detection_metric` is only ~2.2 vs **287** for the FDD baseline (~130× weaker). `rssi=+7.8 dB` (power is in the PRACH band), so it's not energy — it's a **frequency/sequence quality** issue: a residual sub‑RB error in the `k_0` placement (#6) or a rate residual in the long‑format preamble degrades the Zadoff‑Chu correlation. This (not just timing) is why Msg3, which depends on the same UL chain, also decodes poorly.

These are characteristic of forcing the **long‑format (config 0) PRACH** through an SCS‑30 TDD cell; the real‑deployment **short‑format (config 159)** path would avoid the long‑format‑specific frequency/rate adaptations entirely (but needs NR short‑format PRACH support in srsRAN_4G — see Open items). Next steps: (a) trace the 75 µs to the slot‑sync UL reference and fix at source (not via the advance hack); (b) verify `k_0` to the exact subcarrier and the long‑format CP/start‑symbol against the gNB's expectation to recover the correlation metric.

---

## Remaining gaps / open items

- **TA = 75 µs in the detection.** Over ZMQ (no propagation) TA should be ~0. There is a residual ~75 µs (~1.5 SCS‑30 symbols) fine‑timing offset in the PRACH placement (start symbol / CP position). It is within the format‑0 detection window so detection succeeds, but it should be understood/zeroed for robustness and for short‑format PRACH.
- **Long‑format vs the real deployment.** All of the above uses **`prach_config_index: 0` (format 0, long 839)** as a validation shortcut, because srsRAN_4G's PRACH library rejects NR **short‑format** config indices (`config_idx ≥ 64`, e.g. the band‑78 default **159**) in `srsran_prach_set_cell_`, and `srsran_prach_get_preamble_format = config_idx/16` is meaningless for them. The real band‑78 deployment uses short‑format PRACH (formats A/B/C, N_zc=139). Supporting it in srsRAN_4G is a separate, larger task.
- **Msg3 PUSCH (current blocker).** The UE receives the RAR and transmits Msg3 on the grant, but the gNB never decodes it (keeps re‑granting). Likely the same SCS‑15‑assumption family applied to *scheduled* UL: the RAR UL‑grant **k2** slot offset, the **TX‑advance/slot‑vs‑subframe** timing (cf. #5/#7) for the scheduled PUSCH, and PUSCH frequency/time placement at SCS‑30. After Msg3: Msg4 contention resolution and the rest of RRC setup remain to be exercised.
- **Residual ~75 µs PRACH timing offset** (see below) means the very first Msg3 is sent before any TA is applied; the gNB issues `ta_cmd=288`, but if the UE's Msg3 timing is off by the same ~75 µs the gNB may miss it — worth checking alongside the Msg3 work.
- **Cleanup before merge.** Remove env‑gated debug: `SRSUE_DUMP_PDSCH_D` dumps in `pdsch_nr.c`/`dmrs_sch.c`, `SRSUE_PRACH_TX_DBG` in `sf_worker.cc`, `SRSUE_NO_SYNC_PRECOMP`/`SRSUE_FFT_WOFF`/`SRSUE_NO_PHASE_COMP` toggles. Revert experimental config knobs: gNB `prach_config_index: 0` (→ default/159 for real deployment), `phy_level/mac_level: debug` + `broadcast_enabled`, and UE `continuous_tx = yes` if not wanted.

---

## Testing traps that cost real time

1. **`pkill -f 'gnb -c'` self‑kill.** The pattern `gnb -c` also matches the *bash script's own command line*, so `pkill -f 'gnb -c'` killed the interpreter running the launch/cleanup script → silent failures and "flaky" gNB. **Use `pkill -x gnb` (exact process name).**
2. **gNB starvation over ZMQ.** With no UE connected, the gNB's RX has nothing to pull and it stops after ~10 s. Start the UE within a few seconds of the gNB binding `:2000` (the FDD 20 MHz case also stalled for unrelated 106‑PRB reasons — use 10 MHz for the FDD baseline).
3. **Stale dump file.** `/tmp/pdsch_d.txt` (and similar) accumulate across runs; if a chained‑`;` cleanup line silently failed, `awk c==1` read **stale top‑of‑file** data → false "no effect" conclusions. Always verify truncation and read the *freshest* record.

---

## Key files touched (srsRAN_4G)

| File | Fix |
|---|---|
| `lib/src/phy/dft/ofdm.c`, `lib/include/srsran/phy/dft/ofdm.h` | #1 SCS‑30 CP layout, `nr_scs_hz`, phase‑comp SCS |
| `lib/src/phy/ue/ue_dl_nr.c` | #1 plumb `nr_scs_hz`, re‑init on scs change |
| `lib/src/asn1/rrc_nr_utils.cc` | #2 `l139` missing `break;` |
| `lib/src/phy/phch/prach.c`, `lib/include/srsran/phy/phch/prach.h` | #3 occasion timing (`mu`), #4 sample‑rate (`<<mu`), #6 `k_0` freq (`carrier_nof_prb`) |
| `srsue/src/phy/prach.cc` | #4 init 2× headroom |
| `srsue/src/phy/sync_sa.cc` | #5 TX advance in slots, #10 `ta.set_numerology` |
| `srsue/src/phy/nr/worker_pool.cc` | plumb mu/carrier_nof_prb, #7 RA‑RNTI t_id |
| `srsue/src/stack/rrc_nr/rrc_nr_procedures.cc` | #8 `pusch.scs_cfg` |
| `srsue/src/stack/rrc_nr/rrc_nr.cc` | #9 apply `tdd-UL-DL-ConfigCommon` to `phy_cfg.duplex` |
| `srsue/hdr/phy/ta_control.h` | #10 numerology‑scaled TA |

See also the auto‑memory note `project_srsue_scs30_cp_bug.md`.

---

# UPDATE 2026-06-04 — NR short-format PRACH (config 159 / B4) WORKS; RACH now reaches Msg3

Switched from the long-format (config 0) workaround to the **real band-78 short-format PRACH (config 159 = format B4, L_RA=139)** and implemented it end-to-end in srsRAN_4G. The RACH now runs **Msg1 (PRACH) → Msg2 (RAR) → Msg3 (transmitted)**. PRACH detection metric jumped from ~2.2 (long-format workaround) to **90** at the OCUDU gNB. Two decisive root-cause fixes this session:

## Fix #11 — PRACH short-format `begin` off-by-1 (even-FFT DC centering)  ★ unblocked PRACH detection
`srsran_prach_gen` short-format frequency placement was **1 subcarrier too high**. With the mirror IFFT (bin N/2 = DC) and an **even** `N_ifft_prach`, the carrier's lowest subcarrier sits one bin higher than the symmetric `-carrier/2` term implies.

**How it was found (definitive method):** added `OCUDU_PRACHDET_DBG`/`OCUDU_PRACHDET_DUMP` to the gNB detector (`prach_detector_generic_impl.cpp`) to print per-sequence `peak/threshold/delay` and to dump the extracted `combined_symbols` and expected `root` (139 cf each) to `/tmp/prachdet_{comb,root}.cf`. numpy showed `|c|` had constant magnitude (a proper ZC ✓) but `c ≈ roll(root, 1)` (98.8% corr) → exactly a **1-subcarrier shift**. On a ZC this maps to a correlation-peak **delay ∝ u (=138)**, far outside the detection window → `peak 0.034 < thr 0.458` → not detected. `begin -= 1` collapses the peak to `delay=0`, `seq=0 peak 0.034 → 41.2` → **detected, metric 90**.
- File: `lib/src/phy/phch/prach.c` (short-format branch in `srsran_prach_gen`), with comment. Also confirmed the NR L_RA=139 root order equals srsRAN's `prach_zc_roots_format4` table (so `4==p->f || IS_SHORT(p->f)` selects it in `srsran_prach_gen_seqs`).

## Fix #12 — RA-RNTI t_id must be the full slot-in-frame index (not `>> mu`)  ★ unblocked RAR reception
With PRACH detected, the gNB sent the RAR with **`ra-rnti=0x10b` (267)** but the UE searched **`0x7f` (127)** → never found Msg2 → endless Msg1 retransmission. `267 = 1 + 14·t_id ⇒ t_id=19` (the actual SCS-30 slot index); `127 ⇒ t_id=9 = 19>>1`. My earlier "#7" applied `>> scs`, halving t_id. The OCUDU gNB (reference) uses the **full slot-in-frame index** per TS 38.321 §5.1.3.
- File: `srsue/src/phy/nr/worker_pool.cc` — `t_id = SRSRAN_SLOT_NR_MOD(scs, TTI_TX(tti))` (removed `>> scs`). UE now computes `0x10b`, receives the RAR PDCCH (`crc=OK`), gets the Msg3 grant + TA, and transmits Msg3.

## Short-format PRACH implementation (config 159 / B4) — supporting changes
- `prach.h`: NR format codes (A1..C2 = 10..16), `IS_SHORT()`, extended `srsran_prach_t`/`_cfg_t` (`mu`, `carrier_nof_prb`, `is_nr`, `prach_nof_sym`, `delta_f_ra`).
- `prach.c`: `cfg_159` (B4, occasion in last slot of subframe → slot 19), short-format `N_zc=139`, `N_cs` from TS 38.211 6.3.3.1-5, `N_ifft_prach = N_ifft_ul >> mu_ra`, `N_cp = (cp_kappa>>mu_ra)·N_ifft_ul/2048` (B4 cp_kappa=936), `N_seq = 12·N_ifft_prach`, ZC DFT replanned to 139.
- `prach_tables.h`: `#pragma GCC diagnostic ignored "-Wmissing-field-initializers"` around the NR config tables.
- gNB config `m2sdr_zmq.yml`: `prach_config_index: 159`.

## CURRENT BLOCKER — Msg3 PUSCH decodes as `crc=KO` (`sinr=-10.7 dB`)
The UE transmits Msg3 on the grant; the gNB sees signal but cannot decode (retransmits 4× → discards → UE's contention-resolution timer expires → restart). **Everything verified to MATCH** between UE and gNB: PRBs (8‑10), symbols (0‑13), DMRS positions `{2,7,11}`, `dmrs_type=1`, `n_id=1`, scrambling `cinit=0x23008001` (n_RNTI=0x4601), slot 17, qam64 TBS=11, **no transform precoding** (absent in SIB1), and **`n-TimingAdvanceOffset=n25600` (13 µs) matches** the UE's `n_ta=400→13 µs`. The `-10.7 dB` is **identical across every retransmission** (deterministic, not noise), so it's a signal-level mismatch — most likely **fine UL timing** (offset > CP corrupts the DMRS channel estimate) or a **DMRS-sequence** mismatch despite matching parameters.
- **Next diagnostic:** dump the gNB's received Msg3 PUSCH (frequency-domain symbols + estimated time offset) the same way the PRACH dump cracked Fix #11; compare the measured DMRS time-alignment against 0 to confirm/deny a fine-timing offset, and the received DMRS sequence against the expected one.

## Debug instrumentation added this session (remove before merge)
- gNB `lib/phy/upper/channel_processors/prach/prach_detector_generic_impl.cpp`: `OCUDU_PRACHDET_DBG` (peak/thr/delay print) and `OCUDU_PRACHDET_DUMP` (combined/root dump).
- UE `lib/src/phy/phch/prach.c`: `SRSUE_PRACH_BEGIN_OFF` env override on `begin` (the `begin -= 1` base fix is permanent; the env is now only a tuning aid).
- UE `ue-zmq.conf`: `phy_level=debug`, `mac_level=debug`, log `filename=/tmp/ue_dbg.log`.

---

# UPDATE 2026-06-04 (cont.) — RACH reaches Msg3-decode; Msg2/RAR fixed; Msg3 blocked by residual UL CFO

After fixes #11/#12 the full RACH runs **Msg1 → RAR → Msg3 (transmitted, TA applied) → Msg3 PUSCH reaches gNB**. Two more fixes + one remaining blocker:

## Fix #13 — UL TX OFDM CP layout (`nr_scs_hz` on the transmit path)  ★ Msg3 snr −10 → +0.5 dB
`srsran_ue_ul_nr_set_carrier` (`lib/src/phy/ue/ue_ul_nr.c`) built the UL TX OFDM `srsran_ofdm_cfg_t` **without `nr_scs_hz`**, so the modulator used the LTE/SCS-15 CP layout (long CP wrongly at symbol 7) for the SCS-30 PUSCH — the transmit-side twin of the SIB1-PDSCH RX bug (fix #1). Added `fft_cfg.nr_scs_hz = SRSRAN_SUBC_SPACING_NR(carrier->scs)`. Verified via a gNB-side PUSCH channel-estimator dump (`OCUDU_PUSCHCE_DBG`): Msg3 `snr_dB` jumped **−10.1 → +0.5**, `ta_us ≈ 0`.
- Also corrected the manual-CP loop in `ofdm.c ofdm_tx_slot` (`SRSRAN_CP_LEN_NORM(slot_in_sf*nof_symbols+i)` for `nr_scs_hz>15000`) for consistency, though that path turned out not to be the dominant error.

## Diagnostic method (reusable)
Instrumented the **OCUDU gNB** PUSCH chain (all env-gated):
- `OCUDU_PUSCHCE_DBG` in `port_channel_estimator_average_impl.cpp::do_compute` → prints `ta_us, snr_dB, epre, cfo_Hz` per PUSCH (Msg3 = `nof_dmrs_pilots=54`).
- `OCUDU_PUSCHCE_DMP` → per-DMRS-symbol channel coeff `H = mean(rx·conj(expected))`: magnitude consistency = DMRS-sequence match; phase ramp = CFO.
- `OCUDU_PRACHDET_DBG`/`OCUDU_PRACHDET_DUMP` (PRACH, used for fix #11).

## CURRENT BLOCKER — residual UL CFO on Msg3 (`snr` pinned at +0.5 dB, `cfo_Hz ≈ −1184`)
With fix #13, Msg3 `ta≈0` and DMRS magnitudes are **consistent** (`|H|≈2.0` across symbols 2/7/11 → DMRS sequence is correct), but the per-DMRS-symbol **phases are 101.3° / 25.3° / 36.6°** — a per-symbol phase progression the gNB reads (aliased over only 3 DMRS symbols, gaps 5 and 4) as `cfo≈−1184 Hz` and mis-compensates, capping `snr` at +0.5 dB — just below the rate-0.131 QPSK LDPC threshold → `crc=KO` → contention-resolution timeout → RACH restart.
Ruled out:
- **UL CFO pre-comp** (`phy.get_ul_cfo()` from noisy connected-mode SSB tracking, rsrp −119/snr −21): forcing it to 0 (`SRSUE_NO_UL_CFO`) left `cfo` unchanged.
- **CP half-boundary step**: the `ofdm_tx_slot` cp_len fix changed nothing (phases byte-identical) — the ramp is uniform, not a step.
- **Phase-comp absence**: disabling UL phase-comp (`SRSUE_NO_UL_PHASECOMP`) made it worse (snr −9.2) — so phase-comp is needed and ~correct; the residual is a small epoch/center-freq mismatch.
- UE & gNB **center freq match** (3390.000 MHz, ARFCN 626000, pointA 625388); `n-TimingAdvanceOffset=n25600` matches; no transform precoding; PRB/sym/DMRS-pos/scrambling all match.
- Empirical `SRSUE_UL_CFO_HZ` sweep via `srsran_vec_apply_cfo` did **not** cleanly cancel it (the `apply_cfo` uses `1000*sf_sz` as the rate, suspect for the NR slot length, and 3-DMRS aliasing makes the gNB CFO readout non-monotonic) — CRC never passed.
- **Next:** the residual is a sub-sample per-symbol phase-comp epoch discrepancy between srsRAN's LTE-derived OFDM and OCUDU's NR OFDM (likely the long-CP duration 60 vs NR 66 samples interacting with the per-slot phase-comp reference, or the odd-slot/subframe phase-comp reference for Msg3 in slot 17). Best next step: dump the **raw** received Msg3 DMRS REs and the UE's transmitted IQ for one slot, and compare the per-symbol phase against an ideal CP-OFDM reference to pin the exact epoch error; or fix the `apply_cfo` scale and brute-force the cancelling offset to confirm the magnitude, then correct the phase-comp epoch.

## Env-gated knobs added this session (all default OFF; for cleanup/continuation)
UE: `SRSUE_PRACH_BEGIN_OFF` (prach.c), `SRSUE_NO_UL_CFO` / `SRSUE_UL_CFO_HZ` (cc_worker.cc), `SRSUE_NO_UL_PHASECOMP` (ue_ul_nr.c). gNB: `OCUDU_PRACHDET_DBG`, `OCUDU_PRACHDET_DUMP`, `OCUDU_PUSCHCE_DBG`, `OCUDU_PUSCHCE_DMP`. UE log: `phy_level/mac_level=debug`, `filename=/tmp/ue_dbg.log` in `ue-zmq.conf`.

---

# ✅ RESOLVED 2026-06-04 — FULL 5G ATTACH COMPLETE (Msg1→Msg4→RRC→NAS→PDU session)

srsUE now performs a **complete band-78 SCS-30 TDD attach** to the OCUDU gNB over ZMQ:
`Msg1 (B4 PRACH) → Msg2 (RAR) → Msg3 (PUSCH, crc=OK) → Msg4 (Contention Resolution) → Random Access Complete → rrcSetup → rrcSetupComplete → Registration Accept → PDU Session Establishment successful, **IP 10.255.0.3**`. gNB↔AMF NGAP (InitialContextSetup, PDUSessionResourceSetup) all complete.

## Fix #14 — UL carrier-centre frequency hardcoded SCS-15 (`get_center_freq_from_abs_freq_point_a`)  ★ Msg3 snr 0.5 → 97 dB
`lib/src/common/band_helper.cc::get_center_freq_from_abs_freq_point_a` computed the carrier centre as `pointA + nof_prb/2 · (12·**15000**)` with the comment *"always 180 kHz regardless of actual SCS"*. For an SCS-30 carrier an RB is 360 kHz, so for 51 PRB the UL centre came out **3385.41 MHz instead of 3390 MHz (4.59 MHz low)**. The DL centre escaped this (set directly from the ARFCN in `rrc_nr.cc`), which is why the DL worked but the UL didn't. `apply_ul_common_cfg` overwrote `ul_center_frequency_hz` with the buggy value, so the UL TX OFDM phase compensation (`ue_ul_nr.c`, center = `ul_center_frequency_hz`) ran 4.59 MHz off → a **−87°/symbol phase ramp** on the Msg3 PUSCH (`4.59e6 · (768+54)/23.04e6 ≈ 163.76 cycles/sym → mod 1 → −87°`), which the gNB read (aliased over 3 DMRS symbols) as `cfo≈−1184 Hz` and mis-compensated → snr pinned at +0.5 dB.

**Fix:** added a `carrier_scs` parameter (default 15 kHz for legacy/SCS-15 callers & tests) and use `SRSRAN_SUBC_SPACING_NR(carrier_scs)` for the RB width; passed the real carrier SCS from all three callers:
- `lib/include/srsran/common/band_helper.h` + `lib/src/common/band_helper.cc` (signature + impl)
- `srsue/src/stack/rrc_nr/rrc_nr.cc::apply_ul_common_cfg` (UL, `phy_cfg.carrier.scs`)
- `lib/src/asn1/rrc_nr_utils.cc` ×2 (DL `make_phy_carrier_cfg`, UL `fill_phy_carrier_cfg`, via `make_subcarrier_spacing`)

Result: UL phase-comp centre = 3390 MHz, Msg3 `cfo_Hz=0.0`, `snr_dB=97`, `crc=OK` → handshake completes.

## How #13/#14 were diagnosed
gNB PUSCH channel-estimator dumps: `OCUDU_PUSCHCE_DBG` (ta/snr/cfo) localized it to a CFO (not timing/DMRS-sequence, since `|H|` was consistent); `OCUDU_PUSCHCE_DMP` (per-symbol channel phase) gave 101.3°/25.3°/36.6° = a −87°/sym ramp; a UE-side `TXOFDM` print exposed `phasecomp_hz=3385410000` ≠ the 3390 MHz carrier centre, and `4.59 MHz = (51·12·15000)/2` pinned the SCS-15 hardcode.

## Complete fix list (this multi-session effort)
#1 SIB1 CP layout (RX) · #2 l139 ASN.1 · #3 PRACH occasion timing · #4 PRACH sample-rate · #5 TX advance · #6 PRACH k_0 · #7→#12 RA-RNTI t_id (full slot index) · #8 PUSCH scs_cfg · #9 TDD duplex · #10 TA numerology · #11 PRACH short-format begin off-by-1 · **#13 UL TX OFDM CP layout (nr_scs_hz)** · **#14 UL carrier-centre SCS (phase-comp)**. Plus full NR short-format (B4/config-159) PRACH support in srsRAN_4G.

## Remaining (optional)
- **OTA**: the protocol stack is now correct end-to-end over ZMQ; OTA on the M2SDR is the next step (was the standing goal) — the SCS-30/band-78 bugs that blocked it are fixed.
- **Cleanup before merge**: env-gated debug knobs (UE: `SRSUE_PRACH_BEGIN_OFF`, `SRSUE_NO_UL_CFO`, `SRSUE_UL_CFO_HZ`, `SRSUE_NO_UL_PHASECOMP`; gNB: `OCUDU_PRACHDET_DBG/DUMP`, `OCUDU_PUSCHCE_DBG/DMP`) all default OFF; revert UE `ue-zmq.conf` debug log levels if desired.

## Data-plane verified 2026-06-04
User-plane confirmed end-to-end: `ping -I tun_srsue_zmq 10.255.0.1` → **5/5, 0% loss, ~12 ms RTT** (1000-byte payload also OK). Proof it traversed the air interface: UE `RLC-NR DRB1: Tx PDU`, gNB `PUSCH rnti=0x4601 tbs=544 crc=OK`. Path: UE app → DRB1 → PUSCH/ZMQ → gNB → GTP → qcore UPF → gateway 10.255.0.1 (veth2) and back. So the attach is not just signalling-complete — IP data flows both ways.
