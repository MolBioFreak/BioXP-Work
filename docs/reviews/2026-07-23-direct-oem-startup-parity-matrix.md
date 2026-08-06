# Direct OEM Startup/Homing Parity Matrix — 2026-07-23

**Authority standard:** Every OEM claim in this matrix is traced to the hashed direct C# source set in `docs/specs/2026-07-23-oem-full-startup-homing-source-only.md`. A runtime behavior not present in an OEM anchor is a deviation, even if it may be a useful diagnostic or safety adaptation.

**Review scope:** source-to-runtime implementation review only. No activation, initialization, homing, axis movement, camera operation, board cycle, solenoid action, or reference write was performed for this review.

## Direct OEM control chain

```text
BioXPMainWindow.initializeEnvironment
  → ControlLib.initialCheck
  → enqueue initializeSystem when door + latch are closed
  → motion_thread_process consumes initializeSystem
  → BioXPMainWindow.initializeSystem
  → ControlLib.initialCheck
  → ControlLib.initializeMotion
  → ClassControlInterface.initializeMotors
```

| OEM stage | Direct OEM evidence | Current runtime location | Disposition | Review finding / required action |
|---|---|---|---|---|
| App environment gate | `BioXPMainWindow.cs:973-1027`; `ControlLib.cs:8728-8760` | `api.py` canonical non-motion startup / lifecycle | **Passed, non-motion only** | Completed separately through constructor pipettes, `initializeMotorsWithoutMotion`, and `initialCheck`. It correctly stops before `initializeSystem`. |
| Worker naming and serial dispatch | `BioXPMainWindow.cs:2039-2051`; `initializeSystem` at `1050+` | `oem_runtime_commands.py:56-113` | **Partial / staged** | Named worker route and fail-closed staging exist. It is not full OEM `initializeSystem`; this must remain visible in status/UI. |
| `initializeSystem` re-check and `initializeMotion` call | `BioXPMainWindow.cs:1140-1162` | `oem_runtime_commands.py:178-332` | **Partial / diagnostic only** | Runtime supports explicit initial-check and non-homing initializeMotion diagnostics; it cannot claim source completion because `ControlLib.initializeMotion` includes physical startup and tip handling. |
| Monolithic full initializer / rehome | `ClassControlInterface.cs:3348-3420`; `ControlLib.rehome` | `usb_driver.py:motor_oem_initialize_motors_full_sequence`, `motor_oem_rehome` | **Rejected and fail-closed** | Review found non-OEM Z clearance, conditional omission of mandatory gripper actions, missing X timing, and missing door/finalization branches. Both public full/rehome wrappers now return `blocked_reason=literal_direct_oem_stage_rewrite_pending` before configuration or motion access. |
| Z startup home | `ClassControlInterface.cs:3348-3353`; `ClassHeadBoard.cs:368-386`; `ClassHeadBoard.cs:60-119`; `ClassHeadBoard.cs:389-400` | `api.py:_execute_oem_startup_step`; `oem_startup_program.py:355-407`; `usb_driver.py:4223+` | **Source-stage correction applied; live proof absent** | Correct direct source is `axisSearchHome(Z,1791)` → left-home path. The prior right-limit Z reference return and pre-home coordinate probe were removed from both staged executors because neither is an OEM `initializeMotors` action. Live enablement remains separate from source provenance. |
| Gripper current / clear | `ClassControlInterface.cs:3354-3355`; `ClassHeadBoard.cs:231-286` | `oem_startup_program.py:359`; helper family in `usb_driver.py` | **Plan only** | Source requires current 31 then `moveSteps(+10000,true)` with limit check and up-to-20-second completion behavior. No live stage is promoted by this review. |
| Gripper home | `ClassControlInterface.cs:3356-3365`; `ClassHeadBoard.cs:368-386` | `oem_startup_program.py:360`; `usb_driver.py:4880+` | **Partial** | Source branch is speed 600 for version 0, else 200. Runtime has a version-aware helper but must bind actual OEM `GripperVersion` before live parity claim. |
| X home and park | `ClassControlInterface.cs:3367-3375`; `ClassHeadBoard.cs:368-386`; `ClassControlInterface.cs:4206-4248` | `api.py:_execute_oem_startup_step`; `oem_startup_program.py:361-362`; helper family in `usb_driver.py` | **Staged raw-order correction; live proof absent** | The raw X-park executor now preserves `sleep(20 ms) → setHome → speed 1700 → sleep(40 ms) → moveX(6000)`. Stage-level execution remains blocked from parity acceptance until the predecessor X-home evidence and bound configuration are accepted. |
| Y home and Y reference | `ClassControlInterface.cs:3376-3379,3389-3392`; `ClassHeadBoard.cs:368-386` | `oem_startup_program.py:363,365` | **Plan only** | Direct source order is after X park and before/after thermal door as shown. No live stage is enabled by this review. |
| Thermal door home | `ClassControlInterface.cs:3380-3388`; `ClassThermalBoard.cs:364-410,433-459` | `usb_driver.py:4734+`; `oem_startup_program.py:364` | **Partial; active-home source correction applied** | Runtime now pre-clears `+2000` at stall guard `+2` when OEM home is active, matching source. Still open: serial-`<10` branch awaits typed OEM serial binding; OEM polling counter/timing and failure branches require literal regression coverage. |
| Chiller/status/current closeout | `ClassControlInterface.cs:3414-3420` | `usb_driver.py:5554+` / staging | **Not accepted** | Must run only after all preceding source stages. Version-1 current restore needs bound `GripperVersion`. |
| Tip remediation | `ControlLib.cs:8797-8856` | `oem_runtime_commands.py:249-332` | **Not implemented** | OEM includes conditional thermal-door open, scripted motion, pipette eject/query/retry, then errors. Diagnostic-only runtime must remain `ready=false`. |
| Self test, camera, cover, park, StartMode | `BioXPMainWindow.cs:1163-1259+` | `oem_runtime_commands.py:393+`; BMS proposal | **Not implemented** | These remain separate source gates. No final-ready/full-startup button may be enabled. |

## Board primitive contract

| Primitive | Direct behavior | Runtime parity status |
|---|---|---|
| `MoveLeft` | `ClassMotor.cs:74-115`: command `2`, axis payload, success response byte 1 = 100 | Source contract recorded; Linux helper uses MoveLeft for OEM home path. |
| `StopMotor` | `ClassMotor.cs:161-182`: command `3` transmitted twice | Must be preserved by transport helper and asserted in source-contract tests. |
| `setHome` | `ClassMotor.cs:492-516`: command `5/1`, local position zero and home flags | Runtime uses controller reference command; full source proof requires transport/readback parity. |
| left / right sensor | `ClassMotor.cs:641-688`; board wrappers at `ClassHeadBoard.cs:389-415` | **Critical:** OEM `queryHome` is left sensor active (`queryLeftSwitchStatus()==0`). Right sensor is a distinct predicate and cannot be called OEM home. |
| `axisSearchHome` | set home → conditional +10000 preclear if OEM home active → 500 ms → `goHome(false)` | Runtime has source-shaped helper plus extra Linux proof guards. These guards are a deviation/acceptance layer, not OEM behavior. |
| `doorSearchHome` | conditional guard+2/+2000 preclear → restore guard → MoveLeft → 50 ms polling counter → Stop → conditionally set home/fail | Runtime active-home preclear was corrected. Serial/config and literal polling semantics remain open. |

## Current source deviations and rules

1. **Removed from OEM startup stage:** right-limit Z reference return. It remains a separate non-OEM diagnostic helper and may not be inserted after Z source home under an OEM label.
2. **Not source behavior:** Linux-only requirements such as observed switch transition, extra travel bounds, exact readback-zero proof, and independent physical observation. They may remain as safety/evidence gates, but response payloads must label them as Linux acceptance conditions rather than OEM semantics.
3. **Configuration remains a source blocker:** `GripperVersion`, door velocity/stall guard, serial number, calibration, camera flags, and axis limits must come from the OEM configuration record; no fallback/default can be represented as machine parity.
4. **No full sequence control:** direct C# `initializeMotors()` is a monolithic source method, but the test harness may execute a single literal source step only for observability. It may not reorder, add an invented pre-step, or silently advance to a later stage.

## BMS effect

The committed BMS control proposal remains correct only as a **typed thin-proxy design**. Until each row above is accepted, BMS may show source plans, runtime state, and blockers; it must not enable a live stage merely because a button is implemented.

## Verification ledger

- TDD RED: source-contract test showed stepwise Z executed the right-limit return and described a non-OEM correction.
- TDD GREEN: both the staged runtime and raw API Z executors now invoke only `motor_oem_home_axis("z", startup=True, timeout_s=30.0)`; no plan/result contains a right-limit reference return.
- TDD RED/GREEN: raw X park lacked the OEM 20 ms delay before `setHome`; it now preserves `20 ms → setHome → speed 1700 → 40 ms → moveX(6000)`.
- TDD RED/GREEN: thermal-door startup omitted OEM active-home pre-clear; active OEM home now causes stall-guard `+2`, `moveSteps(+2000)`, stall restore, then `MoveLeft`.
- TDD RED/GREEN: monolithic full initialization and `rehome` could reach non-OEM motion. Both now fail closed before profile/motion access with `literal_direct_oem_stage_rewrite_pending`.
- Focused parity validation: `14 passed` (`tests/test_bioxp_oem_initialize_motors_live_parity.py`).
- Adjacent startup/API/thermal validation: `29 passed` (`test_oem_startup_program.py`, `test_oem_startup_api.py`, `test_oem_single_action_startup.py`, `test_oem_startup_hardware_initial_check_fallback.py`, `test_oem_thermal_door_api.py`).
- Wider legacy-caller sweep remains intentionally non-accepting: it contains obsolete expectations for the rejected monolithic/mimic path and configuration-fixture omissions. Those failures are recorded as test-debt for the literal rewrite, not masked by fallback values or a false full-startup claim.

## Independent direct-source audit reconciliation

Three independent read-only audits reviewed the direct C# sources and the runtime after the initial matrix. Their conclusions are incorporated as binding review findings below.

### Primitive semantics: source oracle versus guarded Linux runtime

The runtime must not call its guarded helpers “exact OEM parity.” For valid status-100 replies, the core command families and active-switch polarity align, but material behavior differs:

| Primitive / predicate | Direct OEM source | Guarded runtime disposition |
|---|---|---|
| `queryHome` / `queryRightSensor` | Board wrapper treats `ClassMotor` return `0` as active; raw status-100 byte 6=`1` maps to return `0`. A null reply and an uninitialized board also evaluate as source-true. `ClassHeadBoard.cs:389-415`; `ClassThermalBoard.cs:433-459`; `ClassMotor.cs:641-688`. | Valid raw mapping is aligned, but missing/invalid replies are non-proof. This is safer but **not literal OEM**; responses must report `oem_predicate` separately from raw/reply-valid/safety-valid evidence. |
| `MoveLeft` | One command-2 transmit and C# host-state mutation. `ClassMotor.cs:74-115`. | TMCL command family matches, with stricter matching/retry behavior; **partial parity**. |
| `StopMotor` | Command-3 is transmitted **twice**, and the second response controls C# result. `ClassMotor.cs:161-182`. | Current runtime sends one logical MST transaction. This is a **transaction-count deviation**, not exact parity. |
| `setHome` | Sends SAP `5/1`, with C# local home/position state updates even on failed transport. `ClassMotor.cs:492-516`. | Command target aligns, but runtime requires acknowledgement/readback evidence. This is a safety adaptation, not OEM acceptance semantics. |
| `goHome` / `axisSearchHome` | OEM uses its controller-stop/query sequence; `axisSearchHome` does `setHome`, conditional `+10000` preclear, 500 ms wait, clears `MotorHome`, then `goHome(false)`. `ClassHeadBoard.cs:60-119,368-386`. | Runtime adds preflight writes, active polling, transition/motion/readback proofs, and optional travel limits. It is a **guarded reconstruction**. It must never dispatch with an unbound `max_search_abs_delta`. |
| Gripper startup | OEM sequence is unconditional: current 31 → `moveSteps(+10000,true)` → home at 600/200 → restore current 10 only at the end when version 1. `ClassControlInterface.cs:3354-3365,3417-3420`. | One-axis helper restores current in its own `finally` and does not own the mandatory clear. It cannot be represented as the full OEM gripper startup stage. |

### Thermal-door residual blockers

`motor_oem_door_search_home` is **partial**, not accepted parity. It now has the active-home `+2000` preclear correction, but still lacks direct-OEM handling for: `SerialNumber < 10` eligibility/fallback, `m_24Vdropped`, uninitialized-board no-op, literal 300/80/50-ms polling, OEM’s two-transmit stop, and the `CameraCalibrated` failure branch. The OEM source has no observed-motion, ACK/readback, or bounded 8/20-second success predicate; those may exist only as separately named Linux safety intercepts, not as OEM truth.

### BMS / worker admission blockers

No proposed live BMS homing button may be enabled until robot-local code provides all of the following:

1. A persistent expected-next stage, completed-stage, immutable-artifact, and operator-observation ledger.
2. Robot-side predecessor enforcement—canonical `initial_check` alone cannot authorize a caller-selected later stage.
3. A no-motion observation route that cannot write controller reference state.
4. Typed fixed command payloads and bounded timeouts; the browser must never send arbitrary `params`, paths, frame data, or stage names.
5. Preservation of the handler’s terminal source stage/blockers in worker responses, separate from lifecycle operation state.
6. A real source-plan artifact before exposing a “Preview OEM Full Startup” control.

The only enabled BMS baseline remains ownership activation, verified hardware snapshot, and the already-proven non-motion OEM environment initialization. `InitializeMotion Diagnostic—No Homing` must stay explicitly diagnostic and `ready=false`.
