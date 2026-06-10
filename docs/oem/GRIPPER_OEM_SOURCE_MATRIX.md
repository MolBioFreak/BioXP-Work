# BioXP OEM Gripper Source Matrix

## Scope
Source-line anchored comparison for the BioXP G / MotorGrip subsystem. This file is the phase-0 gate for gripper harmonization and must be kept current as implementation phases land.

## OEM source anchors

| OEM source behavior | OEM evidence | Current status before harmonization | Target |
| --- | --- | --- | --- |
| Manual gripper home raises action current before homing | `ClassControlInterface.btnGripperHome_Click`, captured in `OEM_HOMING_CODE.md` lines ~2046-2075: `setGripperCurrent(31)` then `goHome(...)` | Partially represented via `motor_oem_home_axis("g")`, but generic UI G jogs do not use this as the main contract | Dedicated `POST /motion/gripper/home` with ack/reason and scoped current |
| Manual gripper home is gripper-version dependent | v0 uses `goHome(..., 600, true)`; v1 uses `goHome(..., 200, true)` | Backend profile branches, but UI is v0-shaped and does not surface version/provenance | Status exposes `gripper_version`, selected profile, and provenance |
| OEM gripper home restores idle current for v1 | `btnGripperHome_Click` restores `setGripperCurrent(10)` for v1 | Some current restore helpers exist; generic routes can still leave/operator-display ambiguous current semantics | All gripper actions restore param6/param7 to idle safe in `finally` |
| OEM confirmation predicate is not generic limit sanity | `confirmAxis("g"/"gripper")`: home query OR `getG() < 50` | Current status shows raw switch/reference data but not a first-class OEM gripper predicate | `GET /motion/gripper/status` returns `oem_home_predicate` with `query_home`, `position_lt_50`, and source label |
| initialize-without-motion applies G profile | `initializeMotorsWithoutMotion`: v0 speed/acc/current/stall; v1 speed/acc/current/stall; `setRdivPdiv(..., 6, 2)` | `gripper-clear` mostly scopes current; it does not guarantee full profile/RDIV/PDIV immediately before action | Gripper clear/home call a profile application helper before motion |
| startup clear is `moveSteps(+10000)` before home | `initializeMotors`: `setGripperCurrent(31)`, `moveSteps(MotorGrip.axis, 10000, true)`, then `axisSearchHome` | Stepwise startup has `gripper-clear`, but sends it even when both G limits are active and only fails after no-motion | Preflight both-limit conflict before move; then clear action evidence packet |
| startup gripper home follows clear | `initializeMotors`: gripper home after clear, before X/Y/door | Current startup order is correct | Preserve order; route implementation through dedicated gripper contract |

## Current failure anchor
2026-06-10 repeat OEM home stopped at `gripper-clear`: controller ACKed board 4/motor 2 `+10000` but speed stayed 0 for ~12s. Passive G showed both effective limits active and unmasked. This is a hard blocker until explained.

## Harmonization status
- Phase 0 source matrix: in progress when created.
- Phases 1+: see commit history and tests.
