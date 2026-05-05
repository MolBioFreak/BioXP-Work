# BioXP3200 OEM Full-Init Pipette + Vision Conversion Plan

Date: 2026-05-03

Goal: fully decode and convert the OEM full-initialization portions that happen after/around motion initialization: ClassPipetteCollection/ClassPipette startup and cleanup, plus CVisionLib/ClassFrameGrabber camera/inspection/barcode/cover behaviors. This document is intentionally not a claim that pipette or camera parity already exists.

Current bottom line

The Linux stack has useful transport and endpoint scaffolding for /liquid/*, /camera/*, and /vision/*, but full OEM startup parity is not implemented for pipette or vision.

The correct full-init shape from OEM ControlLib.initializeMotion is:

1. m_ControlInterface.initializeMotors()
2. m_machineStatus.ThermalDoorOpen = false
3. m_PipetteControl.queryTipStatus(-1)
4. sleep 500 ms
5. if TipExist:
   - openThermalDoor()
   - set ThermalDoorOpen and TipLoaded true
   - scriptmoveTo(locationID 28 -> locationID 6)
   - update machine location to locationID 6
   - ejectAllTips(false, true)
   - moveZ(80000)
   - moveX(79000)
   - queryTipStatus(-1)
   - if still TipExist: errorEvent("Eject tip failed") and throw
   - clear TipDirty and TipLoaded
   - initiateGroup()
   - checkedPipetteStatus(); retry initiateGroup()+checkedPipetteStatus() once
6. else TipLoaded = false

Source anchor: /home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ControlLib.cs:8797-8856

Vision/inspection is also not a generic snapshot operation. OEM inspectCover performs deck/chiller cover inspection with motion, camera settings, template matching, plate-location state mutation, and recovery moves.

Source anchor: /home/dalab/Desktop/BioXP 3200 Development Work/BioXP_SSD_Backup/decompiled_src/BioXPControlLib/ControlLib.cs:3663-3768

Do not treat Linux camera snapshot success as CVisionLib parity.

OEM source anchors decoded in this pass

A. Full startup/app orchestration

- GenBotApp/BioXPMainWindow.cs: initializeEnvironment queues motion command name initializeSystem when enclosure door and latch are closed.
  Source: decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs:989-997

- GenBotApp/BioXPMainWindow.cs: initializeSystem calls initializeMotion and inspectCover in startup/recovery paths.
  Source: decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs:1148-1160

- ControlLib.initializeMotion owns post-motor pipette cleanup/init.
  Source: decompiled_src/BioXPControlLib/ControlLib.cs:8797-8856

B. Pipette stack

- ClassPipetteCollection is the OEM collection-level controller for four ADP pipettes.
  Source: decompiled_src/BioXPControlLib/ClassPipetteCollection.cs

- Manual diagnostic initialize button does the same basic sequence we need to bind into startup:
  initiateGroup(); if checkedPipetteStatus() fails, initiateGroup(); checkedPipetteStatus(); then reports each pipette initialized.
  Source: ClassPipetteCollection.cs:584-604

- initiateGroup() behavior:
  for each pipette with CommandCompleted true:
    CommandCompleted=false
    reset wait handle
    m_pipette[i].initiate(false)
  waitforcompletion("Reinitialize pipette", 10000)
  enablePressureStream(true)
  sleep 1000
  enablePressureStream(false)
  m_cancontrol.calculatePressureoffset()
  Source: ClassPipetteCollection.cs:677-693

- reinitializePipette() behavior:
  for all four pipettes:
    reset wait handle
    m_pipette[i].initiateGroup()
  waitforcompletion("Reinitialize pipette", 10000)
  verify ErrorCode == 32 for all four
  reset m_TipType to 201
  Source: ClassPipetteCollection.cs:695-724

- checkedPipetteStatus() behavior:
  QueryStatus() each pipette, 30 ms apart, then verify GetErrorCode()==0 for all four; otherwise log system failure.
  Source: ClassPipetteCollection.cs:726-748

- queryTipStatus(-1) counts tips across all four channels using m_pipette[i].QueryTipStatus()==1.
  Source: ClassPipetteCollection.cs:1336-1357

- queryIndividualTipStatus() returns four booleans from QueryTipStatus().
  Source: ClassPipetteCollection.cs:1360-1370

- ejectAllTips(checkMissingTip=true, wait=true) is not just one E1R frame. It selects channels based on TipLocation and current tip readback, resets per-channel completion events, calls m_pipette[j].ejectTip(), waits, sets m_TipType=201, optionally raises lost-tip errors, calls verifyEjectTip(), and clears TipLocation.
  Source: ClassPipetteCollection.cs:1176-1235

- verifyEjectTip() re-queries tip state and can perform a second all-channel ejection pass before failing.
  Source: ClassPipetteCollection.cs:1265-1323

- ClassPipette.QueryTipStatus sends ASCII bytes {63,51,49} == "?31" on report CAN ID, then sets m_tipLoaded based on response byte 49/'1'.
  Source: decompiled_src_can/BioXPControlLib/ClassPipette.cs:571-589

- ClassPipette.QueryPressure sends "?57" and parses numeric payload after first two characters.
  Source: ClassPipette.cs:622-628

- ClassPipette.startFluidDetection sends "BR" and processMessage records completion/error semantics.
  Source: ClassPipette.cs:630-638+

C. Vision/camera stack

- ClassFrameGrabber contains the real CVisionLib/OpenCV/zbar implementation.
  Source: decompiled_src_vision/CVisionLib/ClassFrameGrabber.cs

- ScanBarcode() uses zbar image scanner against a single grabbed frame.
  Source: ClassFrameGrabber.cs:125+

- CamCalibration(int thre, string fname) returns an int[] calibration result using OpenCV operations.
  Source: ClassFrameGrabber.cs:4292+

- locateCover(...) performs image processing for cover detection.
  Source: ClassFrameGrabber.cs:4578+

- checkPoolPlate(string fname, string templateImage) performs pool-plate image/template logic.
  Source: ClassFrameGrabber.cs:6485+

- checkBioSecurityCover(string fname) performs cover validation from a grabbed frame/file.
  Source: ClassFrameGrabber.cs:11918+

- adjustFocus(string fname) computes focus score/adjustment from image data.
  Source: ClassFrameGrabber.cs:12273+

- ControlLib.inspectCover() performs the startup cover inspection semantics: ForceToHighHome, optional DeckInspection skip, close door, inspect output/reagent/storage/chiller cover locations, update machine plate locations, possibly catch/release covers, open door, and return an ErrorStatus.
  Source: ControlLib.cs:3663-3768

- InspectOutputLocation moves gantry/camera to calibrated offsets, applies camera settings, performs matchPattern against cover.jpg/output.jpg/outputw_foil.jpg/output_empty.jpg templates, snapshots on mismatch, and returns semantic integer state.
  Source: ControlLib.cs:3778-3823

Current Linux surfaces decoded in this pass

A. Robot-local FastAPI routes

Source: src/bioxp/api.py

- Camera/device endpoints exist:
  - GET /camera/devices
  - GET /camera/controls
  - POST /camera/control
  - POST /camera/snapshot
  - POST /camera/stream_health
  - POST /camera/auto_recover
  - POST /camera/reset
  - POST /camera/stop
  - GET /camera/stream_state
  - GET /camera/mjpeg

- Vision endpoints exist:
  - POST /vision/inspect
  - POST /vision/barcode/read

- Liquid endpoints exist:
  - GET /liquid/status
  - POST /liquid/init
  - POST /liquid/tip
  - POST /liquid/aspirate
  - POST /liquid/dispense
  - POST /liquid/mix

B. Current pipette transport

Source: src/bioxp/pipette/transport.py and src/bioxp/can_driver.py

Good:
- CanPipetteTransport exists.
- It opens BioXpCanDriver on can0 at 1 Mbit/s.
- It tracks initialization, tip loaded, pressure profile, last transaction, last tip status, and last pressure.
- get_status() safely queries tip status and pressure when driver supports them.
- initialize() sends driver.pipette_initialize(), then optional prime aspirate/dispense if requested.
- tip load is treated as verification, not fake actuator mutation.
- aspirate/dispense/mix require initialized + hardware tip readback.
- BioXpCanDriver implements OEM-ish CAN IDs:
  control 0x100 | (pipette_id << 3)
  command 0x101 | (pipette_id << 3)
  first_part_command 0x103 | (pipette_id << 3)
  middle_part_command 0x104 | (pipette_id << 3)
  report 0x106 | (pipette_id << 3)
- It sends ASCII commands WR, ?31, ?57, E1R, P<vol>,<profile>, D<vol>,<profile>.
- It requires ACK for pipette commands.

Gaps:
- It is effectively single-pipette-id oriented by default, not a faithful four-channel ClassPipetteCollection group model.
- It does not implement collection-level wait handles, per-channel CommandCompleted, per-channel processMessage, or event demux.
- initiateGroup(), reinitializePipette(), checkedPipetteStatus(), verifyEjectTip(), ejectAllTips(), enablePressureStream(), calculatePressureoffset(), terminatecommands(), detectfluid(), queryIndividualTipStatus() are not converted as OEM semantics.
- Response matching is too generic versus OEM per-command/per-channel event semantics.
- It does not yet bind pipette cleanup into ControlLib.initializeMotion runtime path.
- Current /liquid/* endpoints are operator/device primitives, not OEM full-init proof.

C. Current vision/camera service

Source: src/bioxp/services/vision_service.py, src/bioxp/vision/inspection.py, src/bioxp/vision/barcode.py, src/bioxp/api.py

Good:
- Camera enumeration, controls, snapshot, MJPEG stream, health, reset, and auto-recover exist.
- Vision service can require capabilities, capture snapshot, return inspection result, and decode barcode candidates from snapshot.
- Artifacts can include image path, size, image_b64, pick metadata, and capture error.

Gaps:
- InspectionResult.from_snapshot() only records snapshot success/requested checks; it does not implement CVisionLib matchPattern/locateCover/checkPoolPlate/checkBioSecurityCover/adjustFocus semantics.
- /vision/inspect is a generic inspection wrapper, not ControlLib.inspectCover().
- Camera settings profiles from OEM InspectionSettings/CameraControlParameter are not bound.
- Template assets such as cover.jpg/output.jpg/outputw_foil.jpg/output_empty.jpg and calibration offsets are not imported into an OEM vision binding.
- Gantry motion to each camera inspection location is not encoded.
- Machine plate-location state updates and cover move/catch/release recovery semantics are absent.
- Startup vision gate is intentionally a blocker in src/bioxp/oem_startup_program.py.

D. Current OEM startup gates

Source: src/bioxp/oem_startup_program.py

- pipette_startup_check() currently returns unavailable/blocking.
- vision_startup_check() currently returns unavailable/blocking.
- post_home_pipette_cleanup.json is currently written as a blocker with reason: queryTipStatus/eject/checkPipetteStatus gate not implemented for live parity.
- vision_inspection.json is currently written as a blocker with reason: vision/inspection artifact parity gate not yet proven.

That is correct fail-closed behavior, but it is not full startup parity.

Gap map and implementation plan

Phase 0 — Freeze and test the source coverage matrix

Objective: make the pipette/camera gaps executable as tests before changing runtime behavior.

Files to create:
- tests/test_oem_full_init_pipette_vision_coverage.py
- src/bioxp/oem_full_init_plan.py or src/bioxp/oem_init/pipette_vision_plan.py

Files to modify:
- src/bioxp/oem_startup_program.py
- src/bioxp/oem_runtime_commands.py

Required tests:
1. A static coverage test asserts that OEM anchors above are represented in a Python source coverage matrix:
   - ControlLib.initializeMotion lines 8797-8856
   - ClassPipetteCollection.initiateGroup lines 677-693
   - ClassPipetteCollection.checkedPipetteStatus lines 726-748
   - ClassPipetteCollection.ejectAllTips lines 1176-1235
   - ClassPipetteCollection.verifyEjectTip lines 1265-1323
   - ClassPipette.QueryTipStatus lines 571-589
   - ClassPipette.QueryPressure lines 622-628
   - ControlLib.inspectCover lines 3663-3768
   - ClassFrameGrabber.ScanBarcode/CamCalibration/locateCover/checkPoolPlate/checkBioSecurityCover/adjustFocus

2. A dry-run initializeMotion plan test asserts the sequence:
   initializeMotors -> queryTipStatus -> if tips: openThermalDoor -> scriptmoveTo 28->6 -> ejectAllTips -> moveZ 80000 -> moveX 79000 -> queryTipStatus -> initiateGroup -> checkedPipetteStatus retry -> vision gate/inspectCover if requested.

Acceptance gate:
- Tests fail before implementation.
- No live hardware access.

Phase 1 — Implement ClassPipetteCollection-compatible dry-run/state model

Objective: model four pipette channels and collection-level OEM behavior without moving hardware.

Files to create:
- src/bioxp/oem_pipette_types.py
- src/bioxp/oem_pipette_collection.py

Files to modify:
- src/bioxp/oem_startup_program.py
- src/bioxp/oem_runtime_commands.py
- tests/test_oem_full_init_pipette_vision_coverage.py

Implement:
- PipetteChannelState:
  channel_index
  can_ids
  initialized
  command_completed
  error_code
  tip_loaded
  pressure
  last_command
  last_reply

- OemPipetteCollection:
  query_tip_status(pipette=-1)
  query_individual_tip_status()
  initiate_group()
  reinitialize_pipette()
  checked_pipette_status()
  eject_all_tips(check_missing_tip=True, wait=True)
  verify_eject_tip(tip=-1)
  query_pressure()
  enable_pressure_stream(on)
  calculate_pressure_offset()
  terminate_commands()
  detect_fluid(wait=True)

Dry-run behavior:
- Mutates state deterministically.
- Emits source anchors and expected command intents.
- Does not report hardware truth.
- Artifact format: bioxp-oem-pipette-init-v1.

Acceptance gate:
- Dry-run plan/state tests pass.
- Startup still not live-ready; blockers must remain until live transport proof exists.

Phase 2 — Implement pipette CAN reply/event demux and four-channel live shadow

Objective: make Linux pipette transport match OEM ClassPipette command/reply/channel semantics.

Files to modify:
- src/bioxp/can_driver.py
- src/bioxp/pipette/transport.py
- src/bioxp/oem_pipette_collection.py

Likely new file:
- src/bioxp/pipette/event_demux.py

Implement:
- per-channel CAN IDs for pipettes 0..3
- command/report/control/fragment IDs from pipette_can_ids()
- send command with expected reply predicate:
  - ?31 returns tip status
  - ?57 returns pressure
  - WR/init requires no-error completion code
  - E1R requires no-error completion then tip-status readback
  - BR fluid detection produces async processMessage completion
- processMessage equivalent:
  - parse dlc/msg
  - map pipette channel from report/control ID or msg prefix
  - store ErrorCode
  - set CommandCompleted
  - record fluid timestamp
  - surface pipetteError events

Shadow mode:
- Opens CAN if explicitly requested.
- Only query status/pressure/error logs.
- No aspirate/dispense/eject unless live + operator_ack.

Acceptance gate:
- Unit tests with fake CAN bus prove demux and reply matching.
- Robot shadow test can query all four channels and write artifact without motion/liquid action.

Phase 3 — Convert ControlLib.initializeMotion pipette cleanup exactly

Objective: wire the OEM post-motor pipette cleanup/init into the runtime worker.

Files to modify:
- src/bioxp/oem_startup_program.py
- src/bioxp/oem_runtime_commands.py
- src/bioxp/oem_runtime_types.py
- src/bioxp/oem_runtime_worker.py
- src/bioxp/api.py if a new route is needed for supervised stage execution

Implement runtime step:
- run_initialize_motion_pipette_cleanup(mode, artifact_root, operator_ack)

Live gate:
- requires prior initializeMotors/reference complete
- requires operator_ack="INITIALIZE" or stricter "PIPETTE_INIT" for first live run
- requires absolute writable artifact_root
- requires CAN shadow proof from Phase 2

Sequence:
1. queryTipStatus(-1)
2. if no tips:
   - set TipLoaded false
   - artifact ok but not ready until checkedPipetteStatus passes if OEM requires group init in this branch after further source audit
3. if tips exist:
   - openThermalDoor
   - scriptmoveTo(28, well 0, 6, 0, 0)
   - update machine location
   - ejectAllTips(false, true)
   - moveZ(80000)
   - moveX(79000)
   - queryTipStatus(-1)
   - if TipExist: fail closed Eject tip failed
   - clear TipDirty/TipLoaded
   - initiateGroup
   - checkedPipetteStatus; retry once; fail closed if still false

Artifacts:
- post_home_pipette_cleanup.json
- pipette_tip_status_before.json
- pipette_eject_all_tips.json
- pipette_tip_status_after.json
- pipette_initiate_group.json
- pipette_checked_status.json

Acceptance gate:
- Dry-run and fake-hardware tests prove exact sequence and failure branches.
- First robot run should be shadow/query-only.
- First live run must not aspirate/dispense; only OEM startup cleanup/init behavior.

Phase 4 — Import OEM vision/calibration assets into a binding layer

Objective: stop treating camera as generic snapshot; bind OEM camera settings, templates, offsets, and semantic inspection names.

Files to create:
- src/bioxp/oem_vision_bindings.py
- src/bioxp/oem_vision_inspection.py
- tests/test_oem_vision_bindings.py
- tests/test_oem_inspect_cover_plan.py

Data/assets to locate and bind from SSD/config:
- config.xml camera offsets:
  CameraXOffset
  CameraYOffset
  CameraZOffset
  CameraInstalled
  CameraCalibrated
  DeckInspection
  ScreenResolutionHigh
  InspectionLogOnly
- InspectionSettings camera profiles by InspectionItems
- template images:
  cover.jpg
  output.jpg
  outputw_foil.jpg
  output_empty.jpg
  reagentTray.jpg
  reagentEmpty.jpg
  pool-plate templates
  biosecurity cover templates if present

Implement:
- OemVisionBindings.load_from_ssd_config(...)
- OemInspectionTarget dataclass:
  target_name
  locationID
  camera_offsets
  z_position
  settings_profile
  templates
  expected_result
  source_anchor

Acceptance gate:
- Unit test loads real bindings or fails explicitly with missing fields.
- No fake defaults for live mode.

Phase 5 — Convert CVisionLib algorithms to Linux OpenCV/zbar facades

Objective: implement the semantic CVisionLib operations using Linux libraries while preserving OEM result semantics.

Files to modify/create:
- src/bioxp/oem_vision_inspection.py
- src/bioxp/vision/barcode.py
- src/bioxp/vision/inspection.py
- src/bioxp/services/vision_service.py

Operations:
- scan_barcode(): zbar/pyzbar or OpenCV barcode fallback with artifacted raw image and candidates
- match_pattern(): OpenCV template matching with confidence scores
- locate_cover(): Linux equivalent of ClassFrameGrabber.locateCover result contract
- check_pool_plate(): template/image contract and integer/status result
- check_bio_security_cover(): boolean result contract
- adjust_focus(): focus score using Laplacian/variance or OEM-derived method after deeper decode
- snapshot_image(): save raw frame, metadata, camera controls, timestamp

Rules:
- If a camera/template/calibration binding is absent, fail closed.
- If only generic snapshot works, return camera_transport_ok=true but oem_semantics_ok=false.

Acceptance gate:
- Tests run against saved fixture images or generated fixtures, not live camera.
- Result payloads include source anchors, image artifact path, template path, threshold, confidence, and pass/fail.

Phase 6 — Convert ControlLib.inspectCover startup inspection

Objective: reproduce OEM inspectCover semantics, including motion plan and machine-state updates.

Files to modify:
- src/bioxp/oem_startup_program.py
- src/bioxp/oem_runtime_commands.py
- src/bioxp/oem_vision_inspection.py
- src/bioxp/oem_compat/state.py or machine-status equivalent

Implement dry-run plan:
- ForceToHighHome intent
- DeckInspection skip path
- doorOpen(false)
- inspect output chiller location 17
- inspect reagent chiller location 19
- inspect cover storage locations 20 and 18
- count covers
- short/over cover ErrorStatus branches
- catchPlate/releasePlate recovery intents
- setPlateLocation OUTPUT/REAGENT to 18/20
- doorOpen(true)

Live gate:
- requires referenced motion
- requires camera device healthy
- requires loaded OEM vision bindings/templates
- requires artifact_root
- requires operator_ack="INSPECT" for first live pass

Artifacts:
- vision_inspection.json
- inspect_cover_plan.json
- per-target image files
- per-target match results
- machine_state_before_after.json

Acceptance gate:
- Dry-run plan matches OEM branch structure.
- First live vision pass can be camera-only at parked/reference positions, then supervised full inspectCover with motion.

Phase 7 — Integrate full initializeSystem/initializeMotion readiness state

Objective: full-init ready means motion + pipette + vision gates all pass; no green state from partial success.

Files to modify:
- src/bioxp/oem_startup_types.py
- src/bioxp/oem_startup_program.py
- src/bioxp/oem_runtime_commands.py
- src/bioxp/oem_runtime_status.py
- src/bioxp/oem_runtime_api.py

State requirements:
- initializeMotors_complete
- pipette_tip_cleanup_checked
- pipette_group_initialized
- pipette_status_checked
- vision_inspection_checked
- gantry_park_checked
- door_ready_checked
- ready=true only if all required gates pass

Acceptance gate:
- Runtime initializeSystem dry-run returns diagnostic_complete, ready=false until all gate implementations are enabled.
- Runtime initializeSystem shadow can prove status/camera/pipette queries without physical actuation.
- Runtime initializeSystem live cannot skip pipette or vision if settings say required.

Phase 8 — Robot validation ladder

No live pipetting/liquid movement until motion reference and shadow pipette proof are complete.

Robot validation order:

1. Safe GET baseline:
   - /status
   - /motion/power/status
   - /latch/status
   - /liquid/status
   - /camera/devices
   - /camera/stream_state

2. Pipette shadow:
   - query all four tip statuses
   - query all four pressures
   - query error/status for all four
   - no WR/E1R/P/D commands yet

3. Camera shadow:
   - enumerate camera
   - capture raw snapshot
   - record controls
   - record kernel/UVC errors if any

4. Motion already referenced:
   - verify initializeMotors/ref status from current motion work

5. Pipette startup live without liquid transfer:
   - only if shadow proof passes
   - run queryTipStatus/ejectAllTips/initiateGroup/checkedPipetteStatus sequence with operator present
   - no aspirate/dispense

6. Vision inspectCover live:
   - only after camera bindings/templates are loaded and dry-run plan matches source
   - first camera-only target images
   - then supervised full inspectCover motion plan

7. Full initializeSystem live:
   - initializeMotors
   - initializeMotion pipette cleanup/init
   - inspectCover/vision
   - park/door-ready

Definition of done for this portion

This portion is done only when:

- Every OEM source anchor above is represented in a coverage matrix.
- The Linux runtime has source-shaped pipette and vision modules, not generic endpoint wrappers.
- Dry-run plans emit exact OEM order and branch conditions.
- Shadow mode proves CAN pipette readbacks and camera capture/template availability without actuation.
- Live pipette startup cleanup runs and artifacts query/eject/init/status behavior.
- Live vision inspectCover runs and artifacts images, templates, confidence, machine-state changes, and ErrorStatus mapping.
- initializeSystem cannot report ready unless motion, pipette, vision, park, and door-ready gates all pass.

Immediate next execution tranche

Start with Phase 0 and Phase 1 only:

1. Add tests/test_oem_full_init_pipette_vision_coverage.py.
2. Add src/bioxp/oem_pipette_types.py.
3. Add src/bioxp/oem_pipette_collection.py with dry-run/source-anchored four-channel state.
4. Patch src/bioxp/oem_startup_program.py to replace the current one-line post_home_pipette blocker with a dry-run pipette cleanup plan that still fails closed for live.
5. Validate with targeted pytest, then full pytest.
6. Do not sync to robot until dry-run tests pass locally.

Expected first tranche output:
- post_home_pipette_cleanup.json becomes a real OEM-shaped plan/artifact in dry-run.
- live remains blocked until Phase 2 shadow CAN proof.
- No camera/vision claims yet except decoded plan references.
