# BioXP OEM Initialization Controller Parity Spec

Date: 2026-06-13
Status: Draft spec for implementation
Source anchor appendix: `docs/references/2026-06-13-oem-initialization-source-anchors.md`

## Goal

Recreate the GenBotApp/OEM initialization mechanism as an explicit Linux-side controller/state machine, rather than treating initialization as a single homing primitive or a generic axis-home command.

The controller must preserve OEM command ordering, machine-calibrated coordinates, switch predicates, state transitions, fail-closed behavior, and operator evidence.

## Non-goals

- Do not use generic `/motion/axis/home` as an OEM-equivalent initializer.
- Do not infer success from controller ACKs alone.
- Do not silently substitute source defaults for machine calibration values.
- Do not collapse GenBotApp command queue/UI/runtime state into raw motion calls without labeling the missing semantics.
- Do not bundle unrelated gripper dirty work into initialization commits.

## Source references extracted from original SSD

Primary source anchors are copied verbatim in:

- `docs/references/2026-06-13-oem-initialization-source-anchors.md`

Critical anchors:

- GenBotApp main initialization branch: `BioXPMainWindow.cs:1128-1165`
- GenBotApp PrepareToRunJob path and park: `BioXPMainWindow.cs:1240-1288`
- GenBotApp command worker cases: `BioXPMainWindow.cs:2040-2120`
- GenBotApp `validateJob`: `BioXPMainWindow.cs:1370-1425`
- ControlLib `initializeMotion`: `ControlLib.cs:8790-8845`
- Control-interface `initializeMotors` and without-motion setup: `ClassControlInterface.cs:3180-3410`
- Control-interface `HomeXY`: `ClassControlInterface.cs:5050-5070`
- Thermal door home/open/close anchors: `ClassControlInterface.cs:1218-1240`, `1960-2018`
- Z home anchor: `ClassControlInterface.cs:4620-4665`
- CAN-board `goHome` and `doorSearchHome`: `ClassBaseBoard.cs:140-170`, `ClassHeadBoard.cs:50-80`, `ClassThermalBoard.cs:110-130`
- Machine settings and XML fields: `ClassBioXPSettings.cs:220-275`, `3138-3160`, `3828-3855`

## Current Linux truth after live testing

Verified live on 2026-06-13:

- Thermal door OEM parity is strong:
  - machine-calibrated `m_TCDoorOpen=18500` is bound from `config/oem/original_ssd_appdata_20260610/config.xml`.
  - `/motion/axes/status?axes=door` now reports resolved profile source `original_ssd_machine_config`.
  - open route moves to 18500 and succeeds only when raw right/open switch asserts.
  - close route moves to 0 and succeeds only when closed/home predicate asserts.
  - home route searches closed/home and does no-motion `setHome` when closed predicate is true.
- Core X/Y HomeXY primitive physically works but is incomplete as an OEM-ready route:
  - Y reached home and coordinate 0.
  - X reached physical home switch but required a manual no-motion direct `setHome` to rebase from 1253 to -1.
- Z is physically at home/top (`~ -1/-2`) and currently trips the false-home guard if asked to re-search from an already-active home switch.
- G/gripper remains a risk surface:
  - stale dirty files exist under `src/bioxp/oem_gripper.py`, `tests/test_oem_gripper_contract.py`, and related tests.
  - prior double-switch semantics and motor current history mean initialization must not assume G is fully retired as a risk without source-backed predicates.
- `/motion/oem/rehome` is no longer a 500 after `bb440c9`, but still returns `ok=false` when it aborts at `z_home` under already-home conditions.

## Discrete requirements

### R1 — Initialization is a controller

Linux must expose an initialization controller that models GenBotApp/ControlLib initialization phases explicitly:

1. runtime command accepted / queued,
2. initial checks,
3. interlock and board prep,
4. initialize-without-motion motor parameter setup,
5. source-shaped homing/rehome body,
6. door state save/restore or explicit unsupported status,
7. tip/pipette cleanup and gantry park status,
8. final readiness/status publication.

The controller must return structured state for every phase, including source anchors and whether physical motion was commanded.

### R2 — OEM source-mode and Linux implementation-mode must both be explicit

Every route/result must include:

- `source_command`: e.g. `GenBotApp.initializeSystem`, `ControlLib.initializeMotion`, `ControlLib.rehome`, `ClassControlInterface.HomeXY`.
- `linux_implementation`: e.g. `stepwise_controller`, `direct_route`, `manual_guarded_set_home`, `not_implemented`.
- `source_anchor`: file and line range.
- `not_equivalent_to`: at minimum `/motion/axis/home`, `/motion/axis/zero` when applicable.

### R3 — Machine calibration overrides source defaults

The init controller must consume machine config values before compiled defaults:

- `m_TCDoorOpen=18500` for this robot.
- gripper positions/offsets from config:
  - `m_originOffsetG=4450`
  - `m_GripperClosePOS=27350`
  - `m_GripperOpenPOS=31400`
  - `m_GripperOpenWide=32400`
- axis limits from config:
  - `X_limit maxSteps=90263`
  - `Y_limit maxSteps=102956`
  - `Z_limit maxSteps=160000`
  - `G_limit maxSteps=15000`

Compiled defaults may be fallback only, and every fallback must be reported with source.

### R4 — Door state save/restore must be modeled

OEM `ControlLib.rehome` saves/restores door state around `initializeMotors`.

Linux must implement one of:

- true source-equivalent door state save/restore, if source-backed; or
- explicit `unsupported_but_safe` state that records no restore was attempted, with post-init door status proof.

Silent omission is forbidden.

### R5 — Z already-home/top predicate is valid when proven

If Z is already at the active home/top predicate and position is near the known reference (`0/-1/-2`), the initializer must not abort solely because it did not observe an inactive→active transition.

Required behavior:

- if Z home/top predicate active and speed 0 and position within configured already-home tolerance, return `already_home=true`, `physical_motion_commanded=false`, and continue to clearance verification;
- if predicate active but position/switch state is ambiguous, fail closed;
- if predicate inactive, run source-shaped search with transition guard.

### R6 — HomeXY must own coordinate rebasing

The HomeXY implementation must not require manual maintenance follow-up.

Required behavior after each axis search:

- verify stopped,
- verify raw home predicate active,
- perform no-motion `setHome`,
- read back coordinate near zero,
- restore OEM speed/acc,
- return per-axis `home_rebased=true`.

A route that leaves X at a nonzero coordinate while physically at the switch is incomplete.

### R7 — G/gripper semantics remain gated until source-backed

Initialization must not hide gripper uncertainty.

Required behavior:

- use machine config gripper positions and offsets;
- preserve safe current invariants after G home/clear;
- include raw switch/mask truth and OEM predicate mapping in result;
- fail closed if G state is double-active/ambiguous in a way not explained by source.

### R8 — Final readiness is not just motion success

Final initialization success requires:

- all physical motion steps stopped,
- X/Y/Z/G/Door predicates and coordinates consistent with spec,
- latch/rail/door interlocks green,
- motion arm state updated,
- status/readiness endpoint updated,
- artifact bundle written.

## Acceptance criteria

A supervised live run is acceptable only when an artifact proves:

- source anchors used for each phase,
- machine config values consumed for door/gripper/limits,
- no generic axis-home route used,
- door closed at end unless explicit state restore requests open,
- X/Y home switches active and coordinates near zero,
- Z already-home or searched-home branch documented,
- G is homed or explicitly gated with source-backed reason,
- all speeds are 0,
- final `/motion/power/status` is armed with `strict_init_pass` or a new OEM-init pass reason,
- camera snapshot saved.

## Known implementation gaps to close

1. Init controller/state-machine route does not yet exist as a first-class surface.
2. `/motion/oem/rehome` still uses a monolithic mimic and does not model all GenBotApp/controller phases.
3. Door state save/restore is labeled unimplemented.
4. Z already-home/top branch needs implementation and tests.
5. HomeXY must perform its own no-motion `setHome` and verify coordinate rebasing.
6. G/gripper semantics and dirty files need source-aligned resolution.
7. Tip/pipette cleanup and gantry parking need explicit source comparison and either implementation or unsupported status.
8. Runtime status must distinguish startup, strict startup, OEM init in progress, OEM init failed, and OEM init ready.
