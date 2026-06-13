# BioXP OEM Initialization Controller Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Build a source-anchored Linux initialization controller that recreates the GenBotApp/OEM initialization mechanism instead of treating init as a generic homing primitive.

**Architecture:** Add a first-class `oem_init_controller` layer that orchestrates current API/driver primitives stepwise, records source anchors, and fails closed on predicate mismatches. Harden the lower-level primitives first where live testing exposed gaps: Z already-home, HomeXY rebasing, door state modeling, and G/gripper source semantics.

**Tech Stack:** Python FastAPI backend, `src/bioxp/api.py`, `src/bioxp/usb_driver.py`, existing pytest suite, original SSD decompiled C# anchors copied into `docs/references/2026-06-13-oem-initialization-source-anchors.md`.

---

## Ground rules

- Preserve unrelated dirty files unless a phase explicitly owns them.
- Commit after every phase.
- Every implementation phase must include a spec/OEM comparison section in either code result or docs.
- No live motion in tests unless the phase explicitly enters a supervised live gate.
- Never use generic `/motion/axis/home` to claim OEM initialization parity.
- Every route that can move hardware must require explicit operator ack.

## Existing dirty files warning

At plan creation, unrelated gripper-related dirty files were present:

- `src/bioxp/oem_gripper.py`
- `tests/test_bioxp_oem_initialize_motors_live_parity.py`
- `tests/test_oem_gripper_contract.py`

Do not stage them in phases that do not explicitly own G/gripper work.

## Phase 0 — Source oracle and spec lock

**Objective:** Make the OEM source anchors and current Linux gaps auditable before code changes.

**Files:**

- Keep: `docs/references/2026-06-13-oem-initialization-source-anchors.md`
- Keep: `docs/specs/2026-06-13-oem-initialization-controller-parity-spec.md`
- Keep: `docs/plans/2026-06-13-oem-initialization-controller-implementation-plan.md`

**Tasks:**

1. Verify source anchors cover:
   - GenBotApp init branch and command queue.
   - ControlLib `initializeMotion` and `rehome`.
   - ClassControlInterface `initializeMotors`, `initializeMotorsWithoutMotion`, `HomeXY`, door home/open/close, Z home.
   - CAN board `goHome` / `doorSearchHome`.
   - machine config fields.
2. Add missing line ranges before implementation.
3. Ensure every later phase references source anchors by file/line range.

**OEM comparison gate:**

- Confirm the plan does not invent an init order not present in the source.
- Confirm it distinguishes GenBotApp command queue from ControlLib motion primitives.

**Verification:**

```bash
test -s docs/references/2026-06-13-oem-initialization-source-anchors.md
test -s docs/specs/2026-06-13-oem-initialization-controller-parity-spec.md
test -s docs/plans/2026-06-13-oem-initialization-controller-implementation-plan.md
git diff --check -- docs/references docs/specs docs/plans
```

**Commit:**

```bash
git add docs/references/2026-06-13-oem-initialization-source-anchors.md \
        docs/specs/2026-06-13-oem-initialization-controller-parity-spec.md \
        docs/plans/2026-06-13-oem-initialization-controller-implementation-plan.md
git commit -m "docs: specify OEM initialization controller parity"
```

## Phase 1 — Initialization source model and machine-calibration manifest

**Objective:** Add a small source-model module that centralizes OEM init phases, source anchors, machine-calibration fields, and unsupported semantics.

**Files:**

- Create: `src/bioxp/oem_initialization.py`
- Test: `tests/test_oem_initialization_source_model.py`
- Maybe modify: `src/bioxp/oem_config.py`

**Tasks:**

1. Add dataclasses or dict builders for:
   - `OemInitPhase`
   - `OemSourceAnchor`
   - `OemMachineCalibrationManifest`
2. Load machine config via existing `find_oem_machine_config_bundle()` / binding helpers.
3. Expose manifest fields:
   - `m_TCDoorOpen`
   - gripper positions/offsets
   - axis max steps
   - source path and fallback flags.
4. Add tests asserting this robot’s manifest includes:
   - `TCDoorOpen=18500`
   - gripper config values from XML
   - axis limits from XML
   - no silent fallback when XML exists.

**OEM comparison gate:**

- Compare manifest fields to `ClassBioXPSettings.cs` and `config.xml` anchors.
- Phase is incomplete if a machine-config value is known in XML but not surfaced in the manifest.

**Verification:**

```bash
PYTHONPATH=$PWD/src:$PWD .venv/bin/python -m pytest tests/test_oem_initialization_source_model.py -q
python3 -m py_compile src/bioxp/oem_initialization.py
```

**Commit:**

```bash
git add src/bioxp/oem_initialization.py tests/test_oem_initialization_source_model.py src/bioxp/oem_config.py
git commit -m "feat: model OEM initialization source and calibration manifest"
```

## Phase 2 — Z already-home branch

**Objective:** Make the init homing body accept a proven Z already-home/top state instead of aborting before X/Y travel.

**Files:**

- Modify: `src/bioxp/usb_driver.py`
- Test: `tests/test_bioxp_oem_initialize_motors_live_parity.py` or new `tests/test_oem_init_z_already_home.py`

**Tasks:**

1. Extract helper:
   - `motor_oem_axis_already_home(axis_key, tolerance_steps=...)`
2. For Z only, treat raw home/top predicate active + speed 0 + coordinate near zero as:
   - `ok=true`
   - `already_home=true`
   - `physical_motion_commanded=false`
3. Preserve fail-closed behavior if coordinate or switch state is ambiguous.
4. Update `motor_startup_homing_mimic()` so Z already-home proceeds to `motor_oem_verify_z_clearance_for_xy()`.

**OEM comparison gate:**

- Source uses Z home as the first initializer step, but real machine can start already at the top/home predicate.
- Compare the accepted no-motion branch against the source intent: safe Z reference established before X/Y/G travel.
- Do not weaken transition guard for other axes unless source/live proof requires it.

**Verification:**

```bash
PYTHONPATH=$PWD/src:$PWD .venv/bin/python -m pytest tests/test_oem_init_z_already_home.py tests/test_bioxp_oem_initialize_motors_live_parity.py -q
python3 -m py_compile src/bioxp/usb_driver.py
```

**Commit:**

```bash
git add src/bioxp/usb_driver.py tests/test_oem_init_z_already_home.py tests/test_bioxp_oem_initialize_motors_live_parity.py
git commit -m "fix: accept proven OEM Z already-home state"
```

## Phase 3 — HomeXY owns coordinate rebasing

**Objective:** Make `/motion/oem/home_xy` complete: after physical switch hit, it performs no-motion `setHome` and verifies near-zero coordinates.

**Files:**

- Modify: `src/bioxp/usb_driver.py`
- Modify: `src/bioxp/api.py` only if response schema needs clearer fields
- Test: `tests/test_oem_homing_routes.py`
- Test: new `tests/test_oem_home_xy_rebase.py`

**Tasks:**

1. In `motor_oem_home_xy()`, after each axis home result:
   - verify speed 0,
   - verify raw home predicate active,
   - call `motor_set_home()` no-motion,
   - read back coordinate.
2. Return per-axis fields:
   - `home_switch_confirmed`
   - `set_home`
   - `position_after_set_home`
   - `home_rebased`
3. Mark route `ok=true` only if both X/Y are stopped, raw home active, and coordinate near zero.
4. Preserve speed/acc restoration in `finally`.

**OEM comparison gate:**

- Compare against `ClassControlInterface.HomeXY lines 5054-5067` and board `goHome` semantics.
- Document whether OEM `goHome` performs setHome internally or via board behavior; Linux must prove the resulting coordinate matches OEM reference either way.

**Verification:**

```bash
PYTHONPATH=$PWD/src:$PWD .venv/bin/python -m pytest tests/test_oem_home_xy_rebase.py tests/test_oem_homing_routes.py -q
python3 -m py_compile src/bioxp/usb_driver.py src/bioxp/api.py
```

**Commit:**

```bash
git add src/bioxp/usb_driver.py src/bioxp/api.py tests/test_oem_home_xy_rebase.py tests/test_oem_homing_routes.py
git commit -m "fix: complete OEM HomeXY coordinate rebasing"
```

## Phase 4 — Door state save/restore model

**Objective:** Replace the current vague `door_state_save/restore implemented=false` with an explicit source-mode model and safe behavior.

**Files:**

- Modify: `src/bioxp/oem_initialization.py`
- Modify: `src/bioxp/usb_driver.py`
- Test: `tests/test_oem_init_door_state_model.py`

**Tasks:**

1. Source-map `ControlLib.rehome` door save/restore line range into model.
2. Define door state enum:
   - `closed`
   - `open`
   - `ambiguous`
3. Implement `capture_thermal_door_state()` using predicates:
   - `tcDoorClosed`
   - `tcDoorOpened`
4. Decide and encode safe restore policy:
   - if source-equivalent writeback exists, restore previous state;
   - otherwise return `unsupported_but_safe` and leave door closed after init.
5. Final result must include before/after door state and source equivalence status.

**OEM comparison gate:**

- Compare to `ControlLib.rehome` save/restore lines in source anchor appendix.
- If Linux does not restore open state, the result must say exactly why and prove final door state.

**Verification:**

```bash
PYTHONPATH=$PWD/src:$PWD .venv/bin/python -m pytest tests/test_oem_init_door_state_model.py tests/test_oem_thermal_door_operations.py -q
```

**Commit:**

```bash
git add src/bioxp/oem_initialization.py src/bioxp/usb_driver.py tests/test_oem_init_door_state_model.py
git commit -m "feat: model OEM initialization door state handling"
```

## Phase 5 — G/gripper source semantics cleanup

**Objective:** Resolve G/gripper initialization semantics before the full init controller can claim OEM parity.

**Files:**

- Modify: `src/bioxp/oem_gripper.py`
- Modify: `src/bioxp/usb_driver.py`
- Test: `tests/test_oem_gripper_contract.py`
- Test: `tests/test_bioxp_oem_initialize_motors_live_parity.py`
- Docs: optional `docs/reviews/YYYY-MM-DD-oem-gripper-init-semantics.md`

**Tasks:**

1. Source-map gripper open/close/home/init functions from original SSD.
2. Bind machine config values:
   - `originOffsetG`
   - `GripperClosePOS`
   - `GripperOpenPOS`
   - `GripperOpenWide`
3. Define raw vs masked switch semantics for G.
4. Preserve current safety invariant: restore idle current after G home/clear.
5. Remove or commit existing gripper dirty work intentionally; do not leave ambiguous mixed state.

**OEM comparison gate:**

- Compare each gripper target and predicate to source and config.
- If both raw switches are active, distinguish source-valid masked state from electrical/semantic ambiguity.

**Verification:**

```bash
PYTHONPATH=$PWD/src:$PWD .venv/bin/python -m pytest tests/test_oem_gripper_contract.py tests/test_bioxp_oem_initialize_motors_live_parity.py -q
```

**Commit:**

```bash
git add src/bioxp/oem_gripper.py src/bioxp/usb_driver.py tests/test_oem_gripper_contract.py tests/test_bioxp_oem_initialize_motors_live_parity.py docs/reviews/*gripper*
git commit -m "fix: align OEM gripper initialization semantics"
```

## Phase 6 — First-class OEM initialization controller

**Objective:** Implement a stepwise controller that mirrors GenBotApp/ControlLib initialization as phases with artifacts.

**Files:**

- Create/modify: `src/bioxp/oem_initialization.py`
- Modify: `src/bioxp/api.py`
- Modify: `src/bioxp/usb_driver.py`
- Test: `tests/test_oem_initialization_controller.py`
- Test: `tests/test_oem_runtime_api.py`

**Tasks:**

1. Add controller function:
   - `run_oem_initialization(run_homing: bool, restore_door_state: bool, include_tip_pipette_cleanup: bool)`
2. Add route:
   - `POST /motion/oem/initialization/run`
   - ack: `OEM_INITIALIZATION_RUN`
3. Implement phases:
   - `accepted`
   - `initial_check`
   - `interlock_prepare`
   - `initialize_without_motion`
   - `door_state_capture`
   - `z_reference`
   - `g_reference`
   - `home_xy`
   - `door_home_or_restore`
   - `tip_pipette_cleanup_or_unsupported`
   - `park_or_ready_position`
   - `final_readiness`
4. Each phase result must include source anchor, movement flag, predicate result, and next action.
5. Abort on first unsafe/ambiguous phase, but still collect final passive status.

**OEM comparison gate:**

- Compare phase order to GenBotApp `initializeSystem`, command worker, `ControlLib.initializeMotion`, `ControlLib.rehome`, and `ClassControlInterface.initializeMotors` anchors.
- Any phase not implemented must be explicit `unsupported` or `deferred`, not hidden.

**Verification:**

```bash
PYTHONPATH=$PWD/src:$PWD .venv/bin/python -m pytest tests/test_oem_initialization_controller.py tests/test_oem_runtime_api.py -q
python3 -m py_compile src/bioxp/oem_initialization.py src/bioxp/api.py src/bioxp/usb_driver.py
```

**Commit:**

```bash
git add src/bioxp/oem_initialization.py src/bioxp/api.py src/bioxp/usb_driver.py tests/test_oem_initialization_controller.py tests/test_oem_runtime_api.py
git commit -m "feat: add OEM initialization controller"
```

## Phase 7 — Runtime status and command queue parity

**Objective:** Connect the new init controller to the existing OEM runtime command surfaces without pretending UI parity is complete.

**Files:**

- Modify: `src/bioxp/api.py`
- Modify: existing OEM runtime modules under `src/bioxp/` discovered by tests
- Test: `tests/test_oem_runtime_worker.py`
- Test: `tests/test_oem_runtime_status.py`
- Test: `tests/test_oem_startup_api.py`

**Tasks:**

1. Make `/oem/runtime/commands/initializeSystem` enqueue or invoke the new controller.
2. Status transitions:
   - `startup`
   - `initializing`
   - `init_ready`
   - `init_failed`
3. Preserve existing runtime endpoints and command history.
4. Ensure `PrepareToRunJob` remains separate from init but can require `init_ready`.

**OEM comparison gate:**

- Compare to GenBotApp command worker cases at `BioXPMainWindow.cs:2040-2120`.
- Do not implement job validation or network interactions as motion init unless source indicates it.

**Verification:**

```bash
PYTHONPATH=$PWD/src:$PWD .venv/bin/python -m pytest tests/test_oem_runtime_worker.py tests/test_oem_runtime_status.py tests/test_oem_startup_api.py -q
```

**Commit:**

```bash
git add src/bioxp/api.py src/bioxp/*runtime* tests/test_oem_runtime_worker.py tests/test_oem_runtime_status.py tests/test_oem_startup_api.py
git commit -m "feat: route OEM runtime initializeSystem through init controller"
```

## Phase 8 — Supervised live validation gate

**Objective:** Run the controller on the robot under supervision and produce proof artifacts.

**Files:**

- Add: `docs/reviews/YYYY-MM-DD-oem-initialization-live-validation.md`
- No code changes unless live validation reveals a bug.

**Pre-live requirements:**

- Clean or intentionally staged working tree for init-related files.
- Door closed.
- Interlocks green.
- API freshly loaded.
- Operator watching.
- Camera functional.

**Run order:**

1. Passive status snapshot.
2. `POST /motion/arm/strict_startup {"run_homing": false}`.
3. `POST /motion/oem/initialization/run` with `OEM_INITIALIZATION_RUN` ack.
4. Passive status snapshot.
5. Camera snapshot.
6. Artifact review.

**OEM comparison gate:**

- Live artifact must show source anchor and predicate proof per phase.
- Final state must satisfy the spec acceptance criteria.

**Verification:**

```bash
# Exact command to be filled in after route lands.
python3 scripts/live_oem_initialization_validation.py --robot-local --artifact /tmp/bioxp-live-runs/YYYYMMDD_oem_init.json
```

**Commit:**

```bash
git add docs/reviews/YYYY-MM-DD-oem-initialization-live-validation.md
git commit -m "docs: record OEM initialization live validation"
```

## Rollback strategy

- Phase 0 docs can be reverted independently.
- Phases 1-4 are mostly additive/hardening and should be safe to keep unless tests fail.
- Phase 5 owns gripper cleanup; revert separately if G assumptions are wrong.
- Phase 6 controller route is new; existing routes should keep working.
- Phase 7 runtime wiring is the riskiest integration; if necessary, revert only that commit to restore prior runtime behavior while preserving hardened primitives.

## Definition of done

The work is done only when:

- Source anchors are in-repo.
- Init controller exists as a first-class route/state machine.
- All known gaps in the spec are either implemented or explicitly unsupported with source-backed reason.
- Non-live tests pass.
- Live supervised artifact proves final home/reference/readiness state.
- No generic route is used to claim OEM init parity.
