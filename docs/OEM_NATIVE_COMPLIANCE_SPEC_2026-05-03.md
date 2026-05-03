# BioXP3200 OEM Native Compliance Spec — 2026-05-03

Goal: define the additional work required to reach theoretical 100% native Linux compliance with the BioXP3200 OEM Windows 10 behavior. Testing and supervised physical validation are what would move each item from theory to fact.

## Definition of 100% native compliance

Native compliance means the Linux stack can run the same machine-level intents as the OEM Windows stack, without Windows, while preserving OEM semantics, safety gates, reply matching, state transitions, error handling, and physical behavior.

It does not mean pixel-for-pixel WPF UI parity. It means behavioral and protocol parity below the UI:

1. Transport parity: same USB/CAN/TMCL/Novo command semantics, reply matching, event demux, timeouts, retries, reset behavior, and logging.
2. Board/motor parity: same board IDs, axes, speed/current/stall/switch-mask parameters, startup order, homing order, door/gripper behavior, set-home semantics, and stop/error recovery.
3. Machine semantic parity: deck locations, plate/well/material models, PositionTable, locationID/wellID, calibration offsets, and script movement targets.
4. Script/job parity: OEM XML verbs, loop/dwell/timing semantics, job lifecycle, pause/resume/stop, recovery, artifacts, and operator intervention points.
5. Pipette parity: initialization, tip state, pressure/fluid sensing, aspirate/dispense/mix/load/eject behavior, CAN ACK/readback/error semantics, and calibration constants.
6. Vision/barcode/inspection parity: camera calibration, barcode reading, cover detection, pool-plate checks, biosecurity-cover checks, focus/lighting behavior, and explicit unavailable/fail states.
7. Thermal/chiller parity: setpoint profiles, acceptance tests, timing, telemetry, error states, and protocol integration.
8. BMS integration parity: BMS remains a thin operator/proxy/control-plane surface, not the robot brain; all robot truth remains robot-local and route coverage/capabilities are explicit.
9. Proof parity: every live command has trace artifacts and physical proof gates sufficient to distinguish controller counters from actual mechanism motion.

## Current baseline

Already present on workstation/robot:
- Additive `src/bioxp/oem_compat/` package with `api.py`, `boards.py`, `control_interface.py`, `control_lib.py`, `frames.py`, `modes.py`, `motor.py`, `pipette.py`, `scripts.py`, `state.py`, `transport.py`, and `vision.py`.
- Robot-local `/oem-compat/startup/dry-run` route validated through the live robot service.
- Dry-run invariants: `physical_motion=false`, `opened_usb=false`, `frame_count=70`.
- Script translation dry-run route plans actions with `executed=false`.
- Existing Linux runtime already has motion, latch, LED, thermal/chiller, camera, pipette, protocol, reference, and service modules.
- Decompiled OEM roots exist for core and supporting assemblies: `decompiled_src`, `decompiled_src_bioxpcommon`, `decompiled_src_can`, `decompiled_src_commonlib`, `decompiled_src_communication`, `decompiled_src_genbotapp`, `decompiled_src_novo`, `decompiled_src_novodevices`, `decompiled_src_novoutilities`, and `decompiled_src_vision`.

Current honest limitation:
- The OEM-compat layer is a dry-run/control-plane parity scaffold, not a proven full live OEM replacement.
- Complete live pipette, vision, job orchestration, deck semantics, and transport reply parity are not yet proven.

## Phase 0 — OEM oracle locking and trace corpus

Objective: make the OEM Windows stack the immutable oracle.

Work:
1. Preserve the OEM SSD image and document exact app/build versions.
2. Extract and index all ClickOnce `.deploy` assemblies, scripts, config XMLs, calibration XMLs, templates, and process-time files.
3. Build an OEM semantic map from decompiled C# and strings:
   - `ClassControlInterface`
   - `ControlLib`
   - `ClassBioXPScriptHandler`
   - `ClassPipette`
   - `ClassPipetteCollection`
   - `ClassFrameGrabber`
   - `PositionTable`, `locationID`, `wellID`, `plateName`
4. Capture OEM Windows behavior traces where possible:
   - startup without motion
   - startup with homing
   - manual home X/Y/Z/G/door
   - basic moveTo/scriptmoveTo
   - latch/door operations
   - pipette init/status/tip/pressure
   - barcode/camera calibration/inspection flows
5. Normalize traces into a stable JSON oracle format.

Files/modules:
- `docs/REVERSE_ENGINEERING_TRACEABILITY.md`
- `docs/TMCL_VENDOR_CORE_CONTROL_MAP.md`
- new `docs/OEM_ORACLE_TRACE_CORPUS.md`
- new `tools/oem_trace_indexer.py`
- new `tests/fixtures/oem_oracle/*.json`

Acceptance gate:
- Every claimed OEM behavior has at least one source anchor: decompiled code, config/script file, string-mined assembly evidence, or captured trace.

## Phase 1 — transport and reply compliance

Objective: replace dry-run-only confidence with a native transport that behaves like NovoCANUSBLib/Novo.Devices.

Work:
1. Implement typed OEM frame objects for all known Novo/TMCL/CAN envelopes.
2. Implement response matcher keyed by board, command, motor/channel, status, sequence/timing where available.
3. Implement asynchronous event demux so unsolicited/status frames cannot be mistaken for command replies.
4. Implement reset/reconnect policy matching OEM behavior, with provenance logging.
5. Define live/shadow/dry-run modes:
   - dry-run emits frames only
   - shadow opens transport only for status/query operations
   - live requires operator ack and artifact root
6. Make USB ownership explicit with acquire/release/reconnect endpoints and logs.

Files/modules:
- `src/bioxp/oem_compat/transport.py`
- `src/bioxp/oem_compat/frames.py`
- `src/bioxp/usb_driver.py`
- `src/bioxp/api.py`
- tests: `tests/test_oem_transport_reply_matching.py`, `tests/test_oem_transport_modes.py`

Acceptance gate:
- Recorded dry-run frames replay exactly.
- Shadow mode query traffic is correlated to expected replies.
- Live transport never reports success on unmatched or ambiguous replies.

## Phase 2 — motion/startup/homing OEM compliance

Objective: implement OEM startup, initialization, homing, door, gripper, switch, current, and stall semantics exactly enough to test physically.

Work:
1. Port `initializeMotorsWithoutMotion()` semantics into live-capable Linux methods.
2. Port `initializeMotors()` startup homing order:
   - Z home
   - gripper current high
   - gripper +10000 premove
   - gripper home
   - X home
   - setHome X
   - speed X 1700
   - move X 6000
   - Y home
   - door home
   - setHome Y
3. Port manual home button semantics and speeds:
   - X/Y 500
   - Z 1791
   - gripper 600 or 200 by version
   - door via `doorSearchHome`
4. Port switch-mask, StallGuard, run/hold current, acceleration/speed profiles per axis.
5. Remove or hard-gate risky prepared-jog reuse; normal moves must run full board activation and axis prep.
6. Add `motion_truth` metadata to every move/home response distinguishing controller-reported counters from physical proof.
7. Add pre/post artifact capture for every supervised move.

Files/modules:
- `src/bioxp/usb_driver.py`
- `src/bioxp/api.py`
- `src/bioxp/services/motion_service.py`
- `src/bioxp/services/reference_service.py`
- `src/bioxp/oem_compat/control_interface.py`
- tests: `tests/test_bioxp_oem_homing.py`, `tests/test_motion_phase1.py`, `tests/test_oem_motion_truth.py`

Acceptance gate:
- Unit tests prove the call order and parameters match OEM sources.
- Robot-local supervised physical tests prove at least one axis moves physically, not just controller counters.
- Homing is not considered compliant until operator-visible physical end positions and switch/reference states agree.

## Phase 3 — deck/location/material semantic compliance

Objective: port the OEM middle layer that turns protocol intent into physical targets.

Work:
1. Port/import the OEM `PositionTable` model and calibration offsets.
2. Implement `locationID`, `wellID`, `plateName`, labware, tray, reagent, and material state models.
3. Implement semantic `moveTo(...)` and `scriptmoveTo(...)` using deck state and calibration offsets.
4. Track material presence and state changes across protocol execution.
5. Implement plate press, cover, door, and gripper semantic actions as machine-level operations, not raw motors.

Files/modules:
- `src/bioxp/domain/deck.py`
- `src/bioxp/domain/locations.py`
- `src/bioxp/domain/labware.py`
- `src/bioxp/domain/capabilities.py`
- `src/bioxp/oem_compat/state.py`
- new `src/bioxp/oem_compat/position_table.py`
- tests: `tests/test_oem_position_table.py`, `tests/test_oem_semantic_moves.py`

Acceptance gate:
- OEM scripts using location/well/plate fields compile to the same semantic targets as OEM evidence.
- No raw axis move is required by a high-level protocol caller.

## Phase 4 — XML script interpreter and job engine compliance

Objective: run OEM scripts natively with equivalent control flow, timing, state, and operator behavior.

Work:
1. Complete XML `cmd` verb support, including at minimum:
   - `MT`, `FP`, `ET`, `PP`, `MC`, `TCD`, `SP`, `CC`, `LA`, `WAIT`, `LOOP`, `DWELL`, `DELAYPOINT`
2. Port `scriptInterpretor`, `executeScript`, `transferScript`, pause/resume/stop, and job state transitions.
3. Implement loop/dwell/timing semantics and process-time accounting.
4. Implement preflight checks: reference state, latch, 24V, deck manifest, liquid/pipette state, thermal readiness.
5. Implement protocol artifact bundle format containing inputs, compiled plan, frame trace, status snapshots, and operator acknowledgements.

Files/modules:
- `src/bioxp/oem_compat/scripts.py`
- `src/bioxp/protocols/oem_xml_import.py`
- `src/bioxp/protocols/compiler.py`
- `src/bioxp/protocols/executor.py`
- `src/bioxp/protocols/runtime_state.py`
- `src/bioxp/protocols/validators.py`
- `src/bioxp/services/protocol_service.py`
- tests: `tests/test_oem_script_verbs.py`, `tests/test_oem_protocol_execution.py`

Acceptance gate:
- `demo.xml`, `lifetest.xml`, and representative OEM scripts compile into audited native plans.
- Dry-run and replay are deterministic.
- Live execution is impossible without explicit operator acknowledgement and artifact root.

## Phase 5 — pipette/liquid subsystem compliance

Objective: replace partial CAN/TX-oriented liquid handling with OEM-equivalent pipette behavior.

Work:
1. Port `ClassPipette` and `ClassPipetteCollection` semantics.
2. Implement initialization, status, errors, reset, and version/channel detection.
3. Implement tip lifecycle:
   - load tip
   - eject tip/all tips
   - query tip status
4. Implement fluid operations:
   - aspirate
   - dispense
   - mix
   - detect fluid
   - pressure query
5. Implement ACK/readback/error matching instead of transmit-only success.
6. Tie liquid operations to deck/reference/material state preflights.
7. Add calibration constants and liquid-class parameters where OEM has them.

Files/modules:
- `src/bioxp/pipette/models.py`
- `src/bioxp/pipette/transport.py`
- `src/bioxp/services/pipette_service.py`
- `src/bioxp/oem_compat/pipette.py`
- `src/bioxp/api.py`
- tests: `tests/test_oem_pipette_protocol.py`, `tests/test_liquid_preflight.py`, `tests/test_pipette_ack_readback.py`

Acceptance gate:
- Status reads real pipette state, not software shadow only.
- Fluid operations fail closed without valid reference/deck/tip/pressure state.
- OEM-equivalent errors are represented instead of generic success/failure.

## Phase 6 — vision/barcode/inspection compliance

Objective: port OEM CVisionLib behavior or native equivalents with explicit calibration and failure semantics.

Work:
1. Port/replace camera frame grabber lifecycle.
2. Implement barcode scan using native Linux zbar/OpenCV equivalent.
3. Implement camera calibration and focus/lighting controls.
4. Implement cover detection, pool plate check, biosecurity-cover check, and template matching.
5. Separate camera transport health from inspection semantic success.
6. Add timeout/recovery provenance for UVC failures.

Files/modules:
- `src/bioxp/vision/barcode.py`
- `src/bioxp/vision/inspection.py`
- `src/bioxp/services/vision_service.py`
- `src/bioxp/oem_compat/vision.py`
- `src/bioxp/api.py`
- tests: `tests/test_oem_vision_semantics.py`, `tests/test_barcode_pipeline.py`

Acceptance gate:
- Camera can fail transport without falsely passing inspection.
- Barcode/template checks have artifact images and confidence values.
- OEM scripts depending on vision receive equivalent pass/fail semantics.

## Phase 7 — thermal/chiller/process compliance

Objective: make thermal/chiller behavior protocol-grade, not just setpoint-grade.

Work:
1. Port thermal controller acceptance-test semantics.
2. Port chiller acceptance/status semantics.
3. Integrate setpoints, dwell timing, ramp readiness, and process-time XML data.
4. Surface thermal/chiller errors in protocol state and preflight.

Files/modules:
- `src/bioxp/can_driver.py`
- `src/bioxp/api.py`
- `src/bioxp/services/protocol_service.py`
- `src/bioxp/protocols/executor.py`
- tests: `tests/test_oem_thermal_chiller.py`

Acceptance gate:
- Protocol engine treats thermal/chiller readiness as first-class gating state.

## Phase 8 — BMS thin-control integration compliance

Objective: expose native robot truth to BMS without making BMS the robot brain.

Work:
1. Add robot capability matrix to BMS showing robot-local route expected/present/proxied/UI-exposed.
2. Ensure BMS proxies all required first-class surfaces:
   - motion/reference
   - liquid
   - vision/camera stream state
   - protocol dry-run/live execution
   - OEM-compat traces/artifacts
3. Surface `prep_policy`, `motion_truth`, `reference_state`, `artifact_root`, and `operator_ack_required` in the cockpit/UI.
4. Keep BMS move controls forcing `reuse_prepared=false` unless debug mode is explicitly enabled.
5. Make BMS report controller telemetry as controller telemetry, not physical proof.

Files/modules:
- `/home/dalab/biomodstack/biomodstack/platform/api/routers/bioxp.py`
- `/home/dalab/biomodstack/biomodstack/platform/frontend/src/lib/bioxpClient.ts`
- `/home/dalab/biomodstack/biomodstack/platform/frontend/src/components/BioXpCockpit.tsx`
- `/home/dalab/biomodstack/biomodstack/platform/frontend/src/components/bioxpConnectionSemantics.ts`

Acceptance gate:
- BMS cannot initiate live motion/protocol without robot-local preflight and operator ack.
- Route coverage is explicit; absent routes are visible as capability gaps.

## Phase 9 — physical validation ladder

Objective: convert theory to fact through staged physical proof.

Required ladder:
1. Dry-run parity: emitted frames match oracle traces.
2. Replay parity: recorded traces replay deterministically.
3. Shadow parity: status/query traffic works without actuation.
4. Strict startup: interlocks/24V/latch/boards validate without homing.
5. Single-axis micro-move: one tiny motion with operator observation and artifacts.
6. Axis reversibility: move out/back where switch guards allow.
7. One-axis homing: supervised physical homing with switch/reference proof.
8. Full startup homing: OEM order validated physically.
9. Semantic move: move to known deck location with physical fiducial confirmation.
10. Pipette dry cycle: tip/pressure state validated without fluid.
11. Wet pipette cycle: aspirate/dispense/mix with controlled test liquid.
12. Vision/barcode check: artifact-backed semantic detection.
13. Minimal OEM script: dry-run then live supervised execution.
14. Full representative protocol: operator-supervised, artifacts complete.

Artifact root recommendation:
- `/mnt/BioModStack/bms_results/bioxp_validation/<timestamp>/`

Each stage must record:
- input request
- compiled semantic plan
- emitted frames
- matched replies
- pre/post status
- pre/post reference state
- pre/post camera snapshots if useful
- operator observation
- pass/fail and reason

## Phase 10 — compliance scorecard and signoff

Objective: prevent false claims.

Work:
1. Maintain a matrix with each OEM capability row marked:
   - source anchored
   - implemented native dry-run
   - transport-shadow validated
   - live validated
   - physical proof validated
   - BMS exposed
   - known gaps
2. Require red/yellow/green status for every subsystem.
3. Never label a capability 100% compliant until it has source anchor + native implementation + live validation + physical proof where applicable.

Files/modules:
- `docs/VENDOR_PARITY_SCORECARD.md`
- new `docs/OEM_NATIVE_COMPLIANCE_MATRIX.md`

## Practical ordering

Recommended execution order:
1. Lock oracle trace corpus.
2. Finish transport reply/event compliance.
3. Lock motion/startup/homing to OEM order and physical truth metadata.
4. Run supervised physical validation ladder through single-axis proof.
5. Implement deck/location semantics.
6. Complete XML interpreter/job engine.
7. Complete pipette subsystem.
8. Complete vision/barcode/inspection subsystem.
9. Complete thermal/chiller protocol integration.
10. Complete BMS thin-control exposure.
11. Run representative protocol validation.
12. Maintain compliance matrix.

## Bottom line

The path to theoretical 100% native compliance is not more random motion endpoints. It is an oracle-driven compatibility runtime:
- OEM source/trace anchored
- native Linux transport/motion/semantic/pipette/vision/job layers
- dry-run/shadow/live modes
- artifacted validation
- explicit physical truth gates

Current deployed OEM-compat work is the right foundation, but it is Phase 0/1 style dry-run/control-plane groundwork. Full compliance requires finishing the semantic middle layer, pipette, vision, job engine, live transport reply semantics, and staged physical validation.
