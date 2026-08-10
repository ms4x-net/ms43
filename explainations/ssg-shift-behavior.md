## 1. The general MS43 torque model

MS43 is a torque-structure ECU, but a fairly early and pragmatic one: everything is anchored on **PVS** (the virtual/filling-equivalent throttle angle, °PVS), not on charge mass directly.

**Driver demand → torque** (Chapter D.4, p. 1269):

```
TQI = IP_TQI_PVS__N__PVS(PVS_DRIV, N) × FAC_TQI_COR × IP_FAC_IGA_TQR__IGA_DIF
```

- `IP_TQI_PVS__N__PVS` — 12×12 map, indicated torque over filling pre-control and engine speed, valid for *optimum ignition angle and reference ambient conditions*.
- `FAC_TQI_COR = FAC_AMP × IP_MAF_ADD_TIA_FAC__TIA × IP_FAC_TCO_TQI__TCO` — ambient pressure (altitude), intake air temp, coolant temp.
- `IP_FAC_IGA_TQR__IGA_DIF` — the "typischer Zündhaken" (ignition efficiency hook): torque factor as a function of retard from MBT.
- In PUC (overrun fuel cut) `TQI = 0`.

Two derived reference values matter for the interventions:
- `TQI_PVS_REF` — driver-demand torque **without** altitude correction (reference conditions). This is the arbitration reference.
- `TQI_PVS_COR` — corrected to current ambient.

**Losses** (D.1, p. 1261): `TQ_LOSS = TQFR(N, MAF) + TQFR_ADD(TCO, TOIL) + TQFR_ST + TQ_LOSS_ACC + TQ_LOSS_ECF + C_TQ_LOSS_HPA`. Note the last term — a constant loss offset for the **SSG hydraulic pump**, added only while `LV_HPA = 1` and CAN is healthy. That's the SSG-specific load in the friction model.

**Actual torque** is modelled twice, because Level-1 monitoring needs redundancy (D.6):
- `TQI_TPS_AV_REF` — from throttle + ISA position via a fictitious summed throttle angle `TPS_SUM_AV`, a plenum pressure ratio map `IP_PQ_MAIN_COL_TQ__N__TPS_SUM`, resulting in a substitute air mass fed to `IP_TQI_MAF__N__MAF`. Filtered with `IP_TQI_MTC_OPEN/CLOSE_CRLC…` to represent manifold/air-mass inertia.
- `TQI_TPS_MAF` — from the measured/modelled MAF directly via the same `IP_TQI_MAF__N__MAF`.

**The inverse path** (how a torque request becomes an actuator command):
- Filling: `PVS_TQI = IP_PVS_TQI__N__TQI(N, TQI_request)` — the inverse of the torque map. Then MIN/MAX arbitration against driver `PVS`, then `TPS_SP_PVS` → `TPS_SP`.
- Ignition: `TQD_FAC` (total reduction factor) → `IP_IGA_DIF_TQR__TQD_FAC` (inverse ignition hook) → retard from MBT, hard-limited late by `IP_IGA_TQR__N__MAF`.
- Cylinder cut-off: injection suppression, used for very fast reduction and for the rev limiter's 13 cut-off patterns.

**On CAN everything is relative** to `C_TQ_STND` (norm torque, Nm): `TQI_CAN = TQI / C_TQ_STND × 100 %`. The DME publishes `TQI_CAN` (driver demand, no interventions), `TQI_TQR_CAN` (including ASR/MSR/ETCU/LIM/AMT/GEAR interventions), `TQI_MAF_CAN` (from air mass and IGA), `TQ_LOSS_CAN`, plus `SF_TQD`, `LV_ERR_GC` and `ERR_AMT` in DME1.

## 2. The SSG interface (SSG1 message, 10 ms)

Read in 4.19.5 / nomenclature p. 195–196. The relevant labels:

| Signal | Meaning |
|---|---|
| `TQI_AMT_CAN` (MD_IND_SSG) | target torque, % of `C_TQ_STND` |
| `TQI_MAF_AMT_CAN` (MD_IND_SSG_LM) | portion of the target to be set **via filling** |
| `TQI_AMT_CPL_CAN` | complement of the target (checksum-style) |
| `CTR_AMT` | message counter |
| `STATE_AMT` (B_SSG) | **shift phase 0…3** |
| `LV_GS` (S_SCHALT) | shift active |
| `STATE_CLU_AMT` (B_SSG_KUP) | clutch state: 0 open, 1 creep, 2 launch, 3 closed |
| `N_MAX_AMT_CAN` | max engine speed requested during the shift |
| `LV_AMT_ES` (S_SSG_MS) | request "engine stop" (injection cut-off) |
| `LV_HPA`, `LV_GP`, `LV_RSC`, `LV_CITY`, `GEAR_INFO` | pump load, fault/limit flag, gear info |

Two torque values, not one — that's the key architectural point. The SSG explicitly splits its request into "do this much with air" (`TQI_MAF_AMT_CAN`) and "the rest with spark" (the delta down to `TQI_AMT_CAN`). Internally:

```
TQI_AMT     = C_TQ_STND × TQI_AMT_CAN / 100%
TQI_MAF_AMT = C_TQ_STND × TQI_MAF_AMT_CAN / 100%
TQI_MAF_AMT = MAX(TQI_AMT, TQI_MAF_AMT)     ! filling path never below total target
```

`LV_CS` (clutch switch, used by cruise control etc.) is synthesised: `LV_CS = 0` only when `STATE_CLU_AMT = 3` (closed), otherwise 1.

## 3. The shift sequence — phases 0…3

The spec never writes a prose description of "upshift" vs "downshift". It defines a **phase state machine driven entirely by the SSG**, and the DME's behaviour is defined per phase. The semantics fall out of the plausibility checks (9.11.4.1.1) and the arbitration matrix (9.11.4.2.3):

**Phase 0 — no shift.** `LV_GS = 0`. Requirement: `TQI_AMT < C_TQI_AMT_COND_DIAG`, i.e. the SSG must not be asking for anything meaningful. Violation → `STATE_ERR_AMT = 01H`. If a shift is signalled (`LV_GS = 1`) but `STATE_AMT = 0`, that's `11H` "Schalteingriff ohne Schaltphase".

**Phase 1 — torque handover, clutch still closed.** Timer `T_STATE_AMT_1_DIAG` runs, limited by `C_T_MAX_STATE_AMT_1_DIAG`. Plausibility here: `TQI_AMT` may not exceed `MAX(TQI, TQ_LOSS) + C_TQI_AMT_DIAG` for longer than `C_T_MAX_TQI_AMT_DIAG`, else `21H` "Momentenanforderung zu groß". So a limited torque *increase* above driver demand is tolerated, but only briefly. This is the phase where the driveline is unloaded before the clutch opens.

**Phase 2 — clutch open, gear change, speed synchronisation.** The monitored quantity changes: instead of the torque check, the ECU counts `T_STATE_CLU_AMT_DIAG` while `STATE_CLU_AMT = 3` (closed). If the clutch stays closed longer than `C_T_MAX_STATE_CLU_AMT_DIAG` → `22H` "Kupplung öffnet nicht". Two things are special in this phase:
- **Arbitration is exclusive.** Per the matrix: `LV_AMT_ACT = 1` and `STATE_AMT = 2` → the ECU uses `TQI_COR_AMT`, and *"alle anderen Momentenanforderungen werden nicht berücksichtigt"*. ASR, MSR, LIM, GEAR are all ignored. The SSG owns the engine.
- **Rev limit is handed to the gearbox.** In 9.7 (p. 601): if `LV_GS > 0`, `STATE_AMT = 2` and `ERR_AMT = 0`, then `N_MAX = N_MAX_AMT_CAN + C_N_MAX_AMT_ADD`. Each limiter event during a shift increments the non-volatile counter `CTR_N_MAX_AMT_GS` (readable via DS2 — effectively a shift-abuse counter). Worth noting: the formula gates on phase 2 only, while the calibration label for `C_N_MAX_AMT_ADD` says "während Schaltphase 1 und 2" — an inconsistency in the document itself.

**Phase 3 — clutch closing, torque build-up.** Symmetric to phase 1: same timer structure (`T_STATE_AMT_3_DIAG`) and the same `MAX(TQI, TQ_LOSS) + C_TQI_AMT_DIAG` ceiling on the request.

**Enable condition:** `LV_AMT_ACT = 1` iff `ERR_AMT = 0` AND `LV_GS = 1` AND `STATE_AMT ≠ 0`. Deactivation waits for any running intervention to finish.

## 4. Upshift vs. downshift — where the asymmetry actually lives

The phase machine is identical; the difference is the **sign of the request** and how arbitration treats it.

**Upshift** (torque cut): `TQI_AMT_CAN` drops below driver demand in phase 1. Arbitration in phases 1/3:

```
AMT + (ASR or LIM or GEAR): MIN(TQI_COR_ASR, TQI_COR_MSR, TQI_COR_AMT, TQI_LIM, TQI_PVS_REF)
```

so the lowest request wins, exactly as for a traction-control cut. Reduction is executed by closing the throttle (`PVS_TQI`) plus ignition retard for the fast part. If the SSG asks for `TQI_AMT_CAN = 0` and `TCO > C_TCO_MIN_TI_AMT`, the ECU performs **cylinder cut-off**: injection is suppressed from the next injection onward (already-running injections finish). On release, injectors are reactivated immediately — including injections whose SOI has already passed, respecting the latest possible SOI of 504 °CA after firing TDC. That is the mechanism behind the hard, near-instant torque hole of an SMG upshift.

**Downshift** (blip): the SSG requests a torque *increase* to spin the engine up to the target gear's speed. The spec is explicit about why AMT is treated differently from every other requester:

> `TQI_COR_AMT` is **not** limited to `TQI_PVS_REF`, because the SSG ECU can request both a torque increase and a torque reduction.

Every other intervention is clamped against driver demand (ASR: MIN, MSR: MAX, LIM/GEAR: MIN). AMT is not. Concretely:
- Phases 1/3 with MSR present: `MAX(TQI_COR_AMT, TQI_COR_MSR, TQI_PVS_REF)` — max-selection, so a blip can override the driver.
- Phase 2: `TQI_COR_AMT` alone, unclamped, bounded only by `N_MAX_AMT_CAN + C_N_MAX_AMT_ADD`.

Note the consequence for monitoring: the `TQI_AMT > MAX(TQI, TQ_LOSS) + C_TQI_AMT_DIAG` timeout only applies in phases **1 and 3**. During phase 2 — where the blip actually happens, clutch open — that ceiling is not applied; the rev limiter is the safeguard instead. Sensible, since with an open clutch an over-request can only over-speed the engine, not the driveline.

`SF_TQD = 1` is set for the whole time an SSG intervention is active, telling the rest of the vehicle that filling is under external control.

## 5. The intervention itself: filling path and ignition path

**Filling path** (9.11.4.2.3, p. 667). The request is a *crankshaft* torque at best ignition and reference conditions, so it must be corrected *upward* before the inverse map is entered:

```
TQI_MAF_COR_AMT = TQI_MAF_AMT × 1/IP_FAC_IGA_TQR__IGA_DIF × 1/FAC_TQI_COR
TQI_COR_AMT     = TQI_AMT     × 1/IP_FAC_IGA_TQR__IGA_DIF × 1/FAC_TQI_COR
PVS_TQI         = IP_PVS_TQI__N__TQI(N, selected torque)
```

`STATE_TQ_INTV` (0…5: none / ASR / MSR / LIM / GEAR / AMT) reports which requester currently owns filling. `PVS_TQI` then goes into MIN/MAX selection against `PVS_DRIV_MAX` in the ETC/EGAS filling control. Even for an SSG intervention the throttle closing ramp into the lower mechanical stop is retained (component protection, same as ASR).

**Ignition path** (9.11.4.2.4, p. 671). Two distinct jobs:

1. **Dynamic intervention — compensating MTC inertia.** The throttle can't follow `TPS_SP` instantly, so during the transient `TPS_AV > TPS_SP` and torque is too high. Start conditions: `TQI_MAF_AMT(n-1) − TQI_MAF_AMT(n) ≥ C_TQI_DIF_AMT` **or** `TPS_AV − TPS_SP_PVS ≥ C_TPS_IGA_DIF_AMT`. End condition: `TQI_TPS_AV_MMV − TQI_TPS_SP < C_TQI_IGA_DIF_AMT`. The reduction factor is the ratio of modelled setpoint to modelled actual torque:

```
FAC_TQD_TPS      = TQI_TPS_SP / TQI_TPS_AV_MMV
FAC_TQD_TPS_MIN  = MIN(FAC_TQD_TPS, 1)                       ! normal operation
                 = MIN(FAC_TQD_TPS, FAC_TQD_TPS_MAF, 1)      ! MTC limp home
FAC_TQD_TPS_COR  = 1 − IP_FAC_TQD_COR_AMT__TQI_AMT_CAN × (1 − FAC_TQD_TPS_MIN)
FAC_TQD_TPS_TQR  = FAC_TQD_TPS_COR × IP_FAC_IGA_TQR__IGA_DIF
```

`IP_FAC_TQD_COR_AMT__TQI_AMT_CAN` is the calibration handle for how aggressive the dynamic retard is (0 = none, 1 = full). Multiplying by `IP_FAC_IGA_TQR__IGA_DIF` accounts for retard already present for other reasons (cat heating, knock), which is why that factor must be evaluated *without* the SSG's own retard. Note the trap flagged in the doc: after MTC limp home this weighting must be replaced by `IP_FAC_TQD_CAT__PVS_AV`.

2. **Static intervention — the deliberate spark cut requested by the SSG:**

```
FAC_TQD_IGA     = TQI_AMT / TQI_MAF_AMT
FAC_TQD_TPS_TOT = FAC_TQD_TPS_TQR × FAC_TQD_IGA
TQD_FAC         = FAC_TQD_TPS_TOT
```

`TQD_FAC` enters `IP_IGA_DIF_TQR__TQD_FAC` and the resulting retard is executed. Three properties specific to SSG:
- **No rate limiting.** Neither entering nor leaving the SSG ignition intervention is rate-limited — the retard is applied and removed in one step. (Exception: if ending the intervention causes a transition to PU/PUC, the normal `T_IGA_LGRD_PUC` ramp applies.)
- **Its own late limit:** `IP_IGA_TQR_AMT__N__MAF` replaces the usual `IP_IGA_TQR__N__MAF` — the SSG is allowed to go later than other requesters, at the cost of exhaust temperature.
- **Time-bounded:** a timer starts when `TQI_AMT_CAN < TQI_MAF_AMT_CAN`. If the static intervention isn't finished within `C_T_IGA_AMT`, it's treated as a fault and `TQI_MAF_AMT_CAN` is ramped down to `TQI_AMT_CAN` via `C_TQI_CAN_LGRD_DIAG`.

While an SSG intervention is active, no *additional* torque-reducing ignition intervention from other requesters is possible — `TQD_FAC` is taken purely from the AMT chain.

## 6. Plausibility, fault handling, safety

`ERR_AMT` (2 bit, sent back in DME1) is derived from an internal 8-bit `STATE_ERR_AMT`:

| `STATE_ERR_AMT` | Meaning | `ERR_AMT` |
|---|---|---|
| 00H | no fault | 00H |
| 01H | no shift intervention, boundary conditions implausible | 02H |
| 10H | engine stop, conditions not met | 03H |
| 11H | shift intervention without shift phase | 02H |
| 20H | timeout of a shift phase | 03H |
| 21H | torque request too large | 03H |
| 22H | clutch does not open | 03H |
| 30H | torque transmission faulty | 02H |
| 40H | CAN timeout | 01H |
| 50H | message counter timeout | 02H |

Message integrity is checked arithmetically: `TQI_AMT_CAN + CTR_AMT − TQI_AMT_CPL_CAN = 0`, else `30H`. Plus counter-stall monitoring (`C_T_MAX_CTR_AMT_DIAG`) and CAN timeout.

**Limp-home** (9.11.4.1.2): while `ERR_AMT > 0`, no intervention is permitted and any active one is ramped back to driver demand `TQI_CAN` / `TQI_PVS_REF` using `C_TQI_AMT_CAN_LGRD_POS/NEG_DIAG`. Faults recorded under fault location 134 (86H). If the fault clears, interventions are re-enabled — except where Level 2 has latched (see below). With `LV_GP > 0` (SSG fault flag) the ECU additionally limits to `C_N_MAX_AMT_GP` rpm and `C_VS_MAX_AMT` km/h.

**Safety engine stop:** `LV_AMT_ES` from the SSG requests injection blanking. Only accepted if `STATE_AMT = 0`, `TQI_AMT_CAN ≤ C_TQI_AMT_CAN_ES` and `VS_FIL < C_VS_FIL_AMT`; otherwise `10H`. `LV_ENG_STOP` then latches **for the rest of the engine run** — injectors are only re-enabled once engine standstill (`LV_ES = 1`) *and* a terminal-15 OFF→ON transition are detected.

**Level 2 (ETC safety concept, Chapter E, p. 1328f)** duplicates the whole thing in the monitoring computer at 40 ms with its own `_MON` variables and `C_TQI_STND_MON`, and can set `LV_AMT_INH_MON = 1` — inhibiting AMT torque requests **non-reversibly** until the error-bit initialisation conditions are met.

## 7. Peripheral SSG adaptations worth knowing

- **Variant detection:** the "SSG gearbox with CAN11H" variant is learned after each ECU initialisation and only established once an SSG1 message has been received; relearning from manual to AT/SSG happens per ECU init.
- **Misfire detection blanking** (B.2.1.6.10): on `LV_GS` 0→1 a timer `C_AMT_GS_DLY_ER` starts and misfire monitoring is blanked for the whole shift plus that delay — necessary because of injection cut-off and driveline shock.
- **Idle/creep:** with `STATE_CLU_AMT = 1` (creep) the slipping clutch adds drag, compensated by ISA pre-control offsets `IP_ISAPWM_AMT__TCO` and `IP_ISAPWM_VS_FIL_AMT__VS_FIL__N_DIF`, backed off via `C_ISAPWM_LGRD_CRP`.
- **Launch support:** SSG-specific variants of the launch maps — idle speed raise `IP_N_SP_ADD_CS_AMT__TOIL` and retard `IP_IGA_CS_IS_AMT__TOIL` — build torque reserve before pull-away.
- **Anti-jerk (AJ)** uses SSG-specific thresholds (`C_VS_MIN_AJ_AMT`, `C_PVS_GRD_MIN_AJ_AMT`), derives `GEAR_AJ` from `GEAR_INFO`, and per-clutch-state enables via `C_STATE_CLU_AMT_OPEN/CRP/ST/CLOSE/DFT`. Fast deceleration (ramp-out) is generally not allowed on the SSG variant.

If you want to go a level deeper, the two places I'd read next in the PDF are the arbitration matrix on p. 668–669 (it's the clearest single statement of who wins what) and the `TQI_TQR_CAN` formula on p. 202–203, which shows exactly how the ECU reports back what it actually delivered, including the cylinder cut-off and limiter-pattern terms.
