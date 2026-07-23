# BioXP OEM Runtime-Parity and BMS Supervised-Control Specification

> **Status:** Proposed implementation contract. This document specifies the runtime/BMS architecture and testing controls; it does not claim that full OEM startup/homing is implemented or live-proven.
>
> **Source authority:** The source-order and branch contract is `docs/specs/2026-07-23-oem-full-startup-homing-source-only.md`. This document may not relax, reorder, or fill gaps in that contract.

**Goal:** Turn the OEM Windows startup chain into a robot-local, serialized, evidence-producing runtime that BMS can operate as a thin, typed, supervised control surface.

**Architecture:** The robot owns all OEM semantics, USB/CAN access, source-order enforcement, artifacts, and terminal truth. BMS owns the saved target, connection generation, request admission, concise operator controls, and local command ledger. BMS must proxy only named runtime commands; it must never become a generic motion proxy or reinterpret a source stage.

**Primary source sequence:**

```text
initializeEnvironment
→ queue initializeSystem on motion worker
→ initialCheck
→ initializeMotion
→ initializeMotors
   → z-home
   → gripper-clear (+10000 at current 31)
   → gripper-home
   → x-home
   → x setHome / speed 1700 / moveX(6000)
   → y-home
   → thermal-door-home + closed predicate
   → y setHome
→ initializeMotion tip branch
→ initializeSystem self-test / camera / cover / park / StartMode terminal branch
```

---

## 1. Verified current baseline

### Robot-local runtime already present

| Surface | Current route | Current state |
|---|---|---|
| OEM runtime command worker | `POST /oem/runtime/commands/initializeSystem` | Exposed; queues/consumes command and records durable state. |
| OEM worker/runtime status | `GET /oem/runtime/status`, `GET /oem/runtime/worker/status`, `GET /oem/runtime/state` | Exposed. |
| Canonical non-motion startup | `POST /oem/startup/initialize_environment` | Live-proven through pipette stage, `initializeMotorsWithoutMotion`, and `initialCheck`. |
| Stepwise OEM startup sequence | Worker command `initializeSystem` with `run_stepwise_homing=true` and `homing_step` | Exposed as a staged/testing surface; current live request must name one step and use `operator_ack="HOME"`. |
| Legacy motion routes | `/motion/axis/home`, `/motion/axis/zero`, `/motion/oem/startup_step` | Not the proposed BMS runtime-parity control path. They must not be relabeled as full OEM startup. |

**Robot source files to extend or verify**

```text
src/bioxp/oem_runtime_commands.py       # source-shaped initializeSystem worker stages
src/bioxp/oem_initialization.py         # source-shaped controller/finalization model
src/bioxp/oem_homing_runtime.py          # if promoted after source primitive dossier closes
src/bioxp/oem_homing_routes.py           # source-program exposure and dry runs
src/bioxp/api.py                          # typed robot-local route composition
```

### BMS baseline

BMS currently exposes only three typed commissioning commands:

```text
activate_usb_for_service
collect_hardware_snapshot
initialize_oem_environment
```

The current BMS registry intentionally maps only those names in:

```text
platform/api/services/bioxp/command_models.py
platform/api/services/bioxp/robot_client.py
platform/api/services/bioxp/command_coordinator.py
platform/frontend/src/lib/bioxpClient.ts
platform/frontend/src/components/BioXpCockpit.tsx
```

The current cockpit is therefore correct to stop before motion. It must be extended deliberately rather than bypassed with an arbitrary URL forwarding mechanism.

---

## 2. Runtime ownership and truth contract

### Robot-local responsibilities — mandatory

1. Own the single serialized OEM motion worker.
2. Enforce OEM stage order and reject an out-of-order live step.
3. Hold physical USB/CAN ownership only while the service runtime is active.
4. Emit one durable JSON artifact per command and per physical stage.
5. Record pre/post state: CAN ownership, 24 V/interlock/door/latch, relevant axis telemetry, pipette state, active source anchor, request payload, transport responses/events, and terminal result.
6. Return `ready=true` only after the source-defined terminal `initializeSystem` branch succeeds for the bound configuration.
7. Return `failed_closed` with explicit blocker names whenever a source input, primitive semantic, configuration input, or hardware observation is unavailable.

### BMS responsibilities — mandatory

1. Keep profile/target policy, active connection, generation protection, idempotency, mutation policy, and local command history.
2. Proxy only named command definitions with fixed robot route and fixed payload schema.
3. Render server admission and robot result; do not recompute or weaken robot stage admission in React.
4. Show current stage, source label, artifact path/reference, robot response status, and the exact blocker list.
5. Require an explicit click-time confirmation for every mutating stage; no typed magic-word field in the normal operator flow.
6. Never expose arbitrary request paths, raw transport frames, generic `home_all`, `axis/zero`, or an unlabelled "Initialize" that hides which OEM boundary it runs.

### Truth labels

Every command/result card must distinguish these states:

| Label | Meaning |
|---|---|
| `SOURCE PLAN` | OEM source order is known; no hardware execution occurred. |
| `DRY RUN` | Runtime rendered source stages without opening USB or commanding motion. |
| `ROBOT ACKNOWLEDGED` | Robot handler accepted/completed the named stage; not physical proof alone. |
| `SUPERVISED STEP COMPLETE` | Named single source step completed and durable telemetry/artifact exists; operator observation is still pending. |
| `OPERATOR OBSERVED` | Operator recorded expected physical result for that named source step. |
| `FULL STARTUP COMPLETE` | Only valid after the source-defined terminal `initializeSystem` path returns ready. |
| `FAILED CLOSED` | A required source/configuration/predicate/proof condition was unavailable or failed. |

A green top-bar connectivity label must remain separate from commissioning state.

---

## 3. Source-shaped runtime state machine

### States

```text
SAVED / DISCONNECTED
→ RUNTIME ACTIVE
→ HARDWARE SNAPSHOT FRESH
→ NON-MOTION INITIALIZATION PASSED
→ OEM STARTUP PLAN READY
→ OEM INITIALCHECK PASSED
→ STEPWISE HOMING IN PROGRESS
→ INITIALIZEMOTION POST-HOME IN PROGRESS
→ INITIALIZESYSTEM FINAL GATES IN PROGRESS
→ FULL STARTUP COMPLETE
```

Every transition is one robot-local command. No route may skip from `NON-MOTION INITIALIZATION PASSED` to `FULL STARTUP COMPLETE`.

### Source stages available to the runtime worker

| Runtime stage key | OEM source action | Required predecessor | Live execution policy |
|---|---|---|---|
| `oem_startup_plan` | Render `initializeSystem → initializeMotion → initializeMotors` path | non-motion initialization | Dry run only; no USB/motion. |
| `oem_initial_check` | `ControlLib.initialCheck()` | non-motion initialization | Query/configuration effects only; live confirmation. |
| `z_home` | `axisSearchHome(Z, 1791)` | initial check | One supervised physical step. |
| `gripper_clear` | current 31; `moveSteps(Grip, 10000, true)` | `z_home` observed | One supervised physical step. |
| `gripper_home` | version-specific `axisSearchHome(Grip, 600/200)` | `gripper_clear` observed | One supervised physical step. |
| `x_home` | `axisSearchHome(X, 250)` | `gripper_home` observed | One supervised physical step. |
| `x_park_6000` | `setHome(X)`, speed 1700, `moveX(6000)` | `x_home` observed | One supervised physical step. |
| `y_home` | `axisSearchHome(Y, 250)` | `x_park_6000` observed | One supervised physical step. |
| `thermal_door_home` | `doorSearchHome(...)`, source closed-door predicate | `y_home` observed | One supervised physical step. |
| `y_set_home` | `setHome(Y)`, final chiller/gripper-current actions | `thermal_door_home` observed | One supervised physical step. |
| `initialize_motion_post_home` | OEM tip query/remediation branch | all startup homing stages observed | Disabled until source configuration/primitive proof is bound. |
| `initialize_system_final` | self-test/camera/cover/park/StartMode branches | post-home complete | Disabled until all dependent source gates are implemented. |

The order derives from `ClassControlInterface.initializeMotors()` and the downstream method nesting in `ControlLib.initializeMotion()` / `BioXPMainWindow.initializeSystem()`. See the source-only specification for immutable line anchors and open evidence gaps.

### Required runtime enforcement

- A live step accepts **one named stage only**. `plan`, `full`, `all`, or a client-invented sequence must be rejected for live execution.
- The runtime computes the expected next stage; BMS may display it but must not choose it itself.
- A repeated stage is allowed only via an explicit robot-local `repeat_current_stage` request with a recorded reason and a new artifact. It must not silently reset later successful stages.
- Before each physical stage, capture the source-defined preflight and stage-specific telemetry. If the primitive semantics are not in the source dossier, reject before USB/axis command.
- After each physical stage, persist raw transport evidence and require an operator-observation record before admitting the next source stage.
- `initializeMotion` cannot claim to be complete while homing is absent, and no stage can claim `ready=true` before terminal `initializeSystem` gates complete.

---

## 4. New BMS/BioXP operator controls

These are **proposed buttons**. They are grouped by source boundary and map only to named BMS commands. Buttons are not implementation approval to enable live motion before the underlying runtime stage is source-complete.

### A. Existing commissioning controls — keep

| Button | State effect |
|---|---|
| **Activate USB for BioXP Service** | Claims service USB ownership; no homing/motion. |
| **Collect Hardware Snapshot** | Query-only readiness evidence. |
| **Initialize BioXP OEM Environment** | Completed non-motion source boundary only. |

### B. New planning and worker controls

| Button | BMS command name | Robot route / fixed intent | Availability |
|---|---|---|---|
| **Preview OEM Full Startup** | `preview_oem_full_startup` | `POST /oem/runtime/commands/initializeSystem`; `mode="dry_run"`; source plan only | After non-motion initialization. |
| **Show OEM Runtime State** | Read-only query, not a normal command | `GET /oem/runtime/state`, `/oem/runtime/worker/status` | Always while connected. |
| **Run OEM Initial Check** | `run_oem_initial_check` | `POST /oem/runtime/commands/initializeSystem`; `mode="live"`; `operator_ack="INITIALIZE"`; `params.run_initial_check=true` | Fresh runtime/hardware and non-motion lifecycle passed. |
| **InitializeMotion Diagnostic — No Homing** | `run_initialize_motion_diagnostic` | Same worker route; `mode="live"`; `operator_ack="INITIALIZE"`; `params.run_initialize_motion=true`, `run_initial_check=true` | Diagnostic only; button detail must say `ready=false`. |

### C. New supervised homing-stage controls

Render this as an **OEM Startup Homing** stage rail, not as generic per-axis buttons. The rail shows source order, current expected stage, last artifact, and whether the prior stage has an operator-observation record.

| Visible button | BMS command name | Fixed runtime intent |
|---|---|---|
| **Test OEM Z Home** | `oem_home_z` | `initializeSystem` worker, `mode="live"`, `operator_ack="HOME"`, `params={run_stepwise_homing:true, homing_step:"z-home", require_operator_observed:true}` |
| **Test OEM Gripper Clear (+10000)** | `oem_gripper_clear` | Same worker, `homing_step:"gripper-clear"` |
| **Test OEM Gripper Home** | `oem_gripper_home` | Same worker, `homing_step:"gripper-home"` |
| **Test OEM X Home** | `oem_home_x` | Same worker, `homing_step:"x-home"` |
| **Test OEM X Park (+6000)** | `oem_x_park_6000` | Same worker, `homing_step:"x-park-6000"` |
| **Test OEM Y Home** | `oem_home_y` | Same worker, `homing_step:"y-home"` |
| **Test OEM Thermal-Door Home** | `oem_thermal_door_home` | Same worker, `homing_step:"door-home"` |
| **Test OEM Y Set-Home** | `oem_y_set_home` | Same worker, `homing_step:"y-set-home"` |
| **Record Observed Stage Outcome** | `record_oem_stage_observation` | New robot-local typed route; records stage ID, pass/fail, optional operator note, and artifact reference. It commands no hardware. |

**Button behavior:**

- The stage rail makes the **next expected** button primary.
- Earlier/later stage buttons are visible for test planning but disabled with the robot-provided source/order reason unless an explicit robot-local repeat/test policy admits them.
- Every physical-stage click uses a concise native confirmation that names the exact source operation, not a generic warning.
- Buttons remain disabled while a normal command is active.
- If a stage is rejected, the UI displays the robot blocker and last artifact; it does not offer a substitute manual `home` or `zero` action.

### D. Future end-to-end controls — present only when their runtime stages exist

| Button | Meaning | Enablement requirement |
|---|---|---|
| **Run OEM InitializeMotion Post-Home** | Execute the source tip-query/remediation branch after completed startup homing. | Source configuration mapped; all homing stages complete and observed; tip branch implementation verified. |
| **Run OEM initializeSystem Final Gates** | Execute source self-test/camera/cover/park/StartMode terminal branches. | Required source dependencies and the post-home stage complete. |
| **Run Complete OEM Startup** | Queue the full source worker path, not a monolith that bypasses the stage ledger. | Only after all individual stage contracts are source-complete and their test artifacts have passed review. |

Until their robot-side implementations exist, these controls must be absent or visibly labelled **SOURCE STAGE NOT IMPLEMENTED**, not enabled placeholders.

### E. Existing manual controls

Existing manual axis/gripper/door controls may remain available in their own **Manual Diagnostics** section. They must be visibly distinguished from OEM startup controls:

```text
Manual control ≠ OEM startup sequence
Manual Home ≠ ClassControlInterface.initializeMotors()
Zero ≠ source homing reference
```

They must not update the OEM startup-stage ledger or make the full startup button appear ready.

---

## 5. Required BMS changes

### API / typed command registry

Modify only typed command definitions and explicit robot route mappings:

```text
platform/api/services/bioxp/command_models.py
platform/api/services/bioxp/robot_client.py
platform/api/services/bioxp/command_registry.py
platform/api/services/bioxp/command_coordinator.py
platform/api/routers/bioxp/commands.py
platform/api/routers/bioxp/connection.py
```

Add discriminated BMS command models for the names in §4. Every model must include:

```text
command
expected_generation
idempotency_key
mode (where applicable)
fixed source-stage enum (where applicable)
```

Do **not** accept client-provided robot paths, arbitrary route parameters, arbitrary stage names, raw frame data, or unbounded timeouts.

The robot client receives one fixed named route for each BMS command. Startup worker commands map to:

```text
POST /oem/runtime/commands/initializeSystem
```

The BMS server constructs the worker payload from its typed command; the browser never supplies a free-form `params` object.

### Frontend

Modify:

```text
platform/frontend/src/lib/bioxpClient.ts
platform/frontend/src/components/BioXpCockpit.tsx
platform/frontend/src/components/bioxpInterlinkStatus.ts
platform/frontend/tests/bioxpOemCommissioningSurface.test.ts
```

Add typed React hooks for status, stage commands, observation records, and artifact/result reads. Replace the current three-stage lifecycle display with:

```text
Non-motion initialization
→ Worker plan / initialCheck
→ OEM homing stage rail
→ initializeMotion post-home
→ initializeSystem final gates
```

BMS must render the robot-provided stage state and blockers rather than locally encoding the source sequence as an admission authority.

---

## 6. Robot-local additions required before BMS enables live buttons

1. **Stage ledger:** Persist expected-next step, completed step, transport evidence, operator-observed state, and immutable artifact reference.
2. **Observation route:** Add a no-motion typed route to record the operator result of a completed physical stage. It cannot mutate a controller reference.
3. **Source primitive dossier binding:** Close the missing `axisSearchHome`, `doorSearchHome`, `moveSteps`, `setHome`, and `moveX` semantics from OEM source/trace before any corresponding stage is admitted live.
4. **Bounded primitive contracts:** Enforce source-derived direction, predicate, stop behavior, timeout, and travel bound; never infer them from a generic home helper.
5. **Full-stage implementations:** Implement post-home tip handling and terminal camera/cover/park/StartMode gates before showing end-to-end buttons.
6. **Artifact index/read API:** Make the exact stage artifact available through a safe read-only robot route so BMS can display/link the result without filesystem access.

---

## 7. Verification and acceptance

### Unit / route contracts

- Every BMS command model accepts only its declared source stage and rejects arbitrary route/params/stage values.
- BMS cannot dispatch a source stage when generation, runtime readiness, hardware freshness, lifecycle predecessor, or mutation policy fails.
- A live stage request with `plan`, `full`, or `all` is rejected before USB/runtime-provider acquisition.
- Out-of-order stage requests are rejected robot-side, even if a client tampers with the BMS request.
- A manual home/zero result cannot promote OEM startup state.
- Observation recording cannot command motion or rewrite a controller reference.

### Browser acceptance

1. BMS `/bioxp` displays the exact robot runtime state and source-stage rail.
2. Dry-run button produces a source-plan artifact without USB/motion.
3. Initial-check card has a click-time confirmation and records a typed BMS command result.
4. Each live stage confirmation names its source action and the current step.
5. While a stage is active, all normal stage controls are disabled and the command ID is visible.
6. After result, UI displays source stage, robot acknowledgment, artifact reference, blocker list, and operator-observation requirement.
7. Console has no errors; the visible BMS origin and serving worktree/API are proven before reporting UI delivery.

### Live acceptance

- One physical OEM step at a time, in source order.
- Operator observation is recorded before the next stage.
- No full-startup claim until the terminal `initializeSystem` branch completes its applicable source gates.
- A command acknowledgement alone is never physical proof.

---

## 8. Explicit non-goals

- No generic robot API proxy from BMS.
- No `Home All`, direct full-homing monolith, or coordinate-zero shortcut under the OEM-startup label.
- No use of manual axis controls as evidence that an OEM stage passed.
- No UI-only state transition to ready.
- No enabled end-to-end button before robot-local source semantics and stage implementations exist.

## Recommended implementation order

1. Robot source-primitive evidence acquisition and source dossier (§6.3).
2. Robot stage ledger + no-motion observation/artifact read endpoints.
3. BMS typed dry-run, initial-check, runtime-state, and artifact-read controls.
4. Robot bounded implementation of one source step at a time, beginning with Z only after its primitive semantics are proven.
5. BMS stage rail and the corresponding single-stage buttons.
6. Post-home `initializeMotion` source branch.
7. Terminal `initializeSystem` source branches.
8. Full queued OEM startup button only after individual-stage and terminal acceptance.
