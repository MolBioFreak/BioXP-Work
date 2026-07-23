# BioXP3200 Full OEM Startup and Homing — Source-Only Remaining Work Specification

> **For Hermes:** Execute only a task whose required OEM source anchor is present in the authority set below. If an anchor is missing, stop at evidence acquisition; do not substitute a Linux behavior, a generic TMCL routine, a remembered prior run, or an inferred switch polarity.

**Goal:** Complete the remaining OEM Windows startup path after the now-proven non-motion `initializeEnvironment` / `initialCheck` boundary, preserving the OEM call order, branches, predicates, and configuration dependencies.

**Architecture:** Model the OEM application-level queue and its `initializeSystem → initializeMotion → initializeMotors` calls as distinct states. The full-motion path must retain the original ordering and source conditionality; it must never be replaced with a generic home-all routine or coordinate-zero substitution.

**Scope rule:** This is a source contract, not a claim that the current Linux implementation already satisfies it. Every normative requirement below is derived from the hashed decompiled OEM sources named in §1. Runtime proof is a later acceptance layer and cannot retroactively create an OEM source requirement.

---

## 1. Authority set

| OEM source | SHA-256 | Required use |
|---|---|---|
| `BioXPControlLib/ClassControlInterface.cs` | `86093e5270c82ea2e45cb4de449076372ca79d9485ba6de9565d5eb255811e6e` | motor configuration, full initialization order, post-home settings |
| `BioXPControlLib/ControlLib.cs` | `f69b3529dcb9723c705ac55ecb3f035010cc294d3891de096c165bb20116f6c2` | CAN/door initial check, initializeMotion, tip-remediation branch |
| `GenBotApp/BioXPMainWindow.cs` | `b288a45e2de54cd2c8d30a4498a343cd6f423aff7e88a78847076bfbfb4e904c` | application startup, door/latch gate, motion worker, initializeSystem branches |

**Source locations**

```text
/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ClassControlInterface.cs
/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ControlLib.cs
/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs
```

## 2. Boundary already passed

The completed non-motion milestone ends before `initializeSystem`.

- `initializeEnvironment()` calls `initialCheck()` after `CAN_READY`, then queues `initializeSystem` only when both enclosure door and latch are closed: `BioXPMainWindow.cs:973-1003`.
- The motion worker consumes that named command and calls `initializeSystem()`: `BioXPMainWindow.cs:2039-2051`.
- The OEM `initialCheck()` contract waits for CAN, verifies door state outside BoardTestMode, and cycles boards: `ControlLib.cs:8728-8760`.

**No claim of full OEM startup is valid until the remaining tasks below are accepted.**

## 3. Binding OEM control flow

```text
initializeEnvironment
  └─ CAN_READY
     └─ initialCheck
        └─ enclosure door AND latch closed
           └─ enqueue "initializeSystem"
              └─ motion_thread_process consumes command
                 └─ initializeSystem
                    ├─ initialCheck
                    ├─ initializeMotion
                    │  ├─ initializeMotors
                    │  │  ├─ Z search-home
                    │  │  ├─ gripper current = 31; clear by +10000 steps
                    │  │  ├─ gripper search-home
                    │  │  ├─ X search-home; set-home; speed 1700; moveX(6000)
                    │  │  ├─ Y search-home
                    │  │  ├─ thermal-door search-home; closed check branch
                    │  │  └─ Y set-home; chiller rates; version-1 gripper current = 10
                    │  └─ query tips; conditionally run OEM tip-remediation sequence
                    ├─ optional self test
                    ├─ conditional camera check
                    ├─ cover inspection
                    ├─ park gantry
                    └─ StartMode-specific door/UI/job-preparation branch
```

The queue and worker boundary is mandatory; it is not an implementation convenience. The OEM app sets `GantryAvailable=false` while executing a queued command and restores it after dispatch: `BioXPMainWindow.cs:2041-2043, 2099`.

## 4. Source-only implementation phases

### Phase 1 — Represent OEM application gate and serialized dispatch

**Objective:** Implement the source-shaped transition from an already-completed `initializeEnvironment` gate to queued `initializeSystem` execution.

**Binding requirements:**

1. Require `CAN_READY` before this flow begins.
2. Execute `initialCheck()` before making the enclosure decision.
3. When both enclosure door and latch are closed, enqueue exactly the `initializeSystem` work item; do not invoke physical homing directly from the UI/request thread.
4. Serialize command consumption and mark gantry unavailable during processing.
5. Preserve the OEM wait/unlock branches when either enclosure condition is not satisfied.

**Anchors:** `BioXPMainWindow.cs:973-1027, 2039-2051, 2099`; `ControlLib.cs:8728-8760`.

**Acceptance artifact:** a deterministic state trace that names the source step, records CAN/door/latch observations, queue submission, worker claim, and terminal result. It must show no motion before worker execution.

**Prohibited substitutions:** direct UI-to-axis dispatch; an implicit "already ready" promotion; a Linux-only interlock rule presented as OEM behavior.

### Phase 2 — Bind OEM configuration inputs before physical startup

**Objective:** Obtain and bind the OEM settings values used by the source before any live implementation claims literal parity.

**Binding configuration dependencies from source:**

- `Z_MOTOR_MAX_CURRENT_UP`, `Z_MOTOR_STALL_GUARD_THRESHOLD`: `ClassControlInterface.cs:3209-3215`.
- `GripperVersion`: configuration and startup branch: `ClassControlInterface.cs:3217-3243, 3354-3365, 3417-3420`.
- `TC_DOOR_VELOCITY`, `TC_DOOR_ACCELERATION`, `TC_DOOR_MAX_CURRENT`, `TCDoorStallGuardThreshold`: `ClassControlInterface.cs:3244-3255, 3380-3387`.
- `SerialNumber` and `CameraCalibrated` for thermal-door failure behavior: `ClassControlInterface.cs:3384-3387`.
- `Calibrated`, `StartMode`, `SelfTest`, `CheckCamera`, and `CameraInstalled` for post-motion behavior: `ClassControlInterface.cs:3393-3412`; `BioXPMainWindow.cs:1102-1125, 1163-1180, 1203-1295`.

**Acceptance artifact:** an immutable, redacted configuration snapshot that identifies the OEM source of every input. A missing value is a **blocker**, not permission to select a default.

### Phase 3 — Complete the source contract for board-level homing primitives

**Objective:** Acquire the OEM implementation or an OEM trace for every called primitive whose behavior is not defined in the available authority set.

**Required evidence before implementation/live enablement:**

| Primitive | Called by | Available authority proves | Missing source evidence that must be acquired |
|---|---|---|---|
| `axisSearchHome(axis, speed)` | Z/G/X/Y startup calls | axis order and caller-supplied speeds | direction command, switch predicate/polarity, transition rule, timeout, stop behavior, reference-set semantics, failure return |
| `doorSearchHome(axis, velocity, stallGuard)` | thermal-door startup call | call parameters and following closed-door branch | wire/stop/predicate semantics and failure return |
| `moveSteps(gripper, 10000, true)` | gripper-clear step | exact signed value and boolean argument | motion completion/timeout/result contract |
| `setHome(axis)` | X and Y post-search | exact placement in sequence | controller/reference completion semantics |
| `moveX(6000)` | immediately after X set-home | exact position/order | completion and bounds semantics |

The current three-file authority set calls these board primitives but does not define their bodies. Do **not** infer them from method names, generic driver behavior, or a prior Linux route. Acquire the OEM board-control assembly/decompilation or a captured OEM transport trace with request, response/event, order, and timing.

**Acceptance artifact:** one source dossier per primitive: origin file/binary hash, method body or trace provenance, parameter semantics, success/failure observation, and the exact startup consumer.

### Phase 4 — Implement `initializeMotors()` in literal OEM order

**Objective:** Reproduce only the startup sequence expressed in `ClassControlInterface.initializeMotors()`.

**Required order and parameters:**

1. If Z board exists: `axisSearchHome(Z, 1791)`.
2. `setGripperCurrent(31)`.
3. `moveSteps(Grip, 10000, true)`.
4. If gripper board exists: `axisSearchHome(Grip, 600)` when `GripperVersion == 0`; otherwise `axisSearchHome(Grip, 200)`.
5. If X board exists: `axisSearchHome(X, 250)`; sleep 20 ms; `setHome(X)`; `setSpeed(X, 1700)`; sleep 40 ms; `moveX(6000)`.
6. If Y board exists: `axisSearchHome(Y, 250)`.
7. If thermal-door board exists: `doorSearchHome(Door, TC_DOOR_VELOCITY, TCDoorStallGuardThreshold)`.
8. If `SerialNumber > 9`, thermal door is not closed, and camera is calibrated: open thermal door and fail with `Cannot close thermal cycler door!`.
9. If Y board exists: `setHome(Y)`.
10. When calibrated: update the OEM coordinate display fields.
11. Set chiller cool rate for `OC`, then `RC`.
12. Set system status to initialized.
13. If `GripperVersion == 1`: `setGripperCurrent(10)`.

**Anchor:** `ClassControlInterface.cs:3348-3420`.

**Acceptance artifact:** a strict per-step trace with source label, parameters, transport frames/events once Phase 3 evidence exists, completion result, and an explicit stop at the first failed step. The trace must make it impossible to reorder or omit the gripper-clear and X-park steps silently.

**Non-goals:** Do not merge manual `HomeAxis()` or `rehome()` semantics into this program. They are separately callable OEM paths: `ClassControlInterface.cs:4997+`; `ControlLib.cs:8784-8795`.

### Phase 5 — Implement the OEM `initializeMotion()` post-homing branch

**Objective:** Follow the OEM post-home tip state handling rather than inventing a generic cleanup sequence.

**Binding requirements:**

1. Set stop-scripts/abort state as in the source, then call the full `initializeMotors()` sequence.
2. Mark thermal door closed in machine state.
3. Query pipette tip status; wait 500 ms.
4. If a tip exists, perform the OEM branch in source order: open thermal door; set `ThermalDoorOpen` and `TipLoaded`; perform the scripted move from location 28 to location 6; eject tips; move Z to 80000; move X to 79000; re-query tips after 100 ms.
5. If tips remain, raise `Eject tip failed`; do not continue to ready.
6. If removal succeeds, clear dirty/loaded state, initiate pipette group, and retry status once exactly as shown.
7. If no tip exists, record `TipLoaded=false`.

**Anchor:** `ControlLib.cs:8797-8856`.

**Configuration/evidence blocker:** the available source identifies location IDs and motion values but not their calibrated physical mapping. Bind their source configuration before live movement; do not translate them into guessed coordinates.

**Acceptance artifact:** source-labeled branch trace containing tip query results, branch selection, every movement call, retry outcome, and fail-closed result when removal fails.

### Phase 6 — Implement `initializeSystem()` downstream gates and terminal branches

**Objective:** Preserve the OEM behavior after `initializeMotion()` rather than reporting ready immediately after homing.

**Binding requirements:**

1. Prevent re-entry while system motion is active.
2. Re-run `initialCheck()` before `initializeMotion()`.
3. Preserve the unexpected-shutdown branch when saved status is 3 or 4: run `initializeMotion`, inspect cover, unlock door, and return through the warning branch.
4. Run `initializeMotion()` on the normal path.
5. Run self-test only under the OEM `SelfTest` and elapsed-date condition.
6. If all OEM camera predicates hold and camera initialization fails: unlock door and fail initialization.
7. Run cover inspection; on nonzero error, park gantry, unlock door, and take the corresponding error path.
8. On success, park gantry and execute the StartMode branch—door opening and the appropriate local/trade-show/network/job-preparation flow.

**Anchors:** `BioXPMainWindow.cs:1046-1295`.

**Acceptance artifact:** a terminal state record naming the exact source branch, including self-test, camera, cover inspection, park, door, and StartMode evidence. A motor-home trace alone is not a successful OEM startup result.

### Phase 7 — Source-to-runtime acceptance sequence

**Objective:** Prove each source phase before proceeding to the next physical phase.

1. **Static source contract test:** assert the program order/parameters in Phase 4 and required branches in Phases 1, 5, and 6 against a source-anchored fixture.
2. **No-USB dry run:** emit the full source step list and every branch predicate without opening hardware.
3. **Transport conformance:** compare implementation output to the Phase 3 OEM primitive dossier. Mismatches fail closed.
4. **Live, single-step supervised execution:** run Phase 4 one OEM step at a time in the exact source order; capture raw request/reply/event/timing and operator physical observation per step.
5. **Live initializeMotion branch testing:** test no-tip first; test tip-present only with an explicitly safe physical setup and full artifact capture.
6. **Live initializeSystem terminal path:** run the actual queued sequence through camera/cover/park/start-mode branches applicable to the bound OEM configuration.

No phase may call a later motion step merely because an earlier step returned an acknowledgement. Source parity requires the source step, source predicate, and its recorded completion contract.

## 5. Explicit exclusions

The following must not appear in an OEM-compliance implementation unless added to the authority set with a new source anchor:

- generic `home_all` behavior;
- controller-coordinate-zero as a substitute for a source homing predicate;
- a new head-clearance motion before the source Z step;
- reordering gripper before Z or omitting the +10000 gripper clear;
- inferred switch polarity/direction/timeout for `axisSearchHome` or `doorSearchHome`;
- a blanket "startup passed" outcome before the post-home tip, camera, cover, park, and StartMode gates;
- a successful result based only on command acknowledgement when OEM completion/predicate evidence is unavailable.

## 6. Definition of the full OEM startup/homing milestone

This milestone may be declared only when all conditions are true:

1. The source evidence deficit in Phase 3 is closed.
2. The full queued application path from `initializeEnvironment` through `initializeSystem` is implemented and source-order-tested.
3. `initializeMotors()` executes the exact 13-item source order in Phase 4, with source-derived primitive semantics.
4. The applicable `initializeMotion()` tip branch and `initializeSystem()` camera/cover/park/StartMode branches complete or fail exactly as source-directed.
5. Live artifacts prove each physical motion phase without a substituted generic control path.
6. Any unavailable OEM dependency ends in an explicit fail-closed state, never a ready state.

---

## Implementation handoff

This document deliberately does not name Linux implementation files or prescribe a new API/UI. Those are engineering decisions to be made only after Phase 3 closes the missing OEM primitive semantics. The next permissible work item is **evidence acquisition for the board-level homing primitive implementation/trace**, followed by a source-to-runtime mapping review against this contract.
