# BioXP OEM homing source matrix and API topology (no-motion artifact)

Truth level: `source_model_only_no_motion_no_usb`.

This artifact separates OEM homing modes and route surfaces before any further live Linux route repair. It is not hardware proof and does not claim physical motion succeeded.

## OEM mode separation

- `initializeMotorsWithoutMotion` (`ClassControlInterface.cs:3181-3265`): setup only; speed/acc/current/stallguard/switch-mask configuration for X/Y/Z/gripper/door.
- `initializeMotors` (`ClassControlInterface.cs:3348-3421`): full startup mechanical/reference sequence:
  - Z `axisSearchHome(...,1791)`
  - gripper current high + `moveSteps(...,10000,true)`
  - gripper `axisSearchHome(...,600/200)`
  - X `axisSearchHome(...,250)`, `setHome`, restore speed `1700`, `moveX(6000)`
  - Y `axisSearchHome(...,250)`
  - door `doorSearchHome(...)`
  - final Y `setHome`
  - gripper-version-1 current restore to `10`
- Manual buttons: X/Y/Z/G use `goHome(true, ..., speed, true)` with X/Y `500`, Z `1791`, gripper `600/200`; door uses `doorSearchHome(...)`.
- `HomeAxis` (`ClassControlInterface.cs:4997-5048`): generic source utility with X/Y `axisSearchHome(250)`, Z `axisSearchHome(597)` after current high, gripper `axisSearchHome(200/150)`, door preclear + `doorSearchHome`.
- `HomeXY` (`ClassControlInterface.cs:5054-5067`): temporary X/Y speedacc `200/200`, parallel `goHome(false,...,200,true)`, restore X/Y speedacc.
- `ControlLib.rehome` (`ControlLib.cs:8784-8795`): save door state, call `initializeMotors`, sleep/restore door/thermal state.
- `ControlLib.initializeMotion` (`ControlLib.cs:8797-8848`): script flags, call `initializeMotors`, then tip/pipette cleanup and park-related motion.

## Board primitive anchors

- Deck/head/thermal `goHome`: `ClassDeckBoard.cs:73-132`, `ClassHeadBoard.cs:60-119`, `ClassThermalBoard.cs:118-177`.
- Deck/head/thermal `axisSearchHome`: `ClassDeckBoard.cs:362-380`, `ClassHeadBoard.cs:368-386`, `ClassThermalBoard.cs:412-430`.
- Thermal door `doorSearchHome`: `ClassThermalBoard.cs:364-410`.
- Board `queryHome`: left switch status `0` means home true (`ClassDeckBoard.cs:383-395`, `ClassHeadBoard.cs:389-401`, `ClassThermalBoard.cs:433-445`).
- Motor switch query payloads: left `{6,9,axis,...}` at `ClassMotor.cs:641-664`; right `{6,10,axis,...}` at `ClassMotor.cs:666-689`.
- Motor `setHome`: `{5,1,axis,...}` at `ClassMotor.cs:492-517`.

## Raw FastAPI vs BMS/proxy topology

- Raw FastAPI route `/motion/oem/home_xy` (POST): direct `HomeXY` mode surface with X/Y speedacc envelope and concurrent X/Y `goHome(false, axis, 200, true)` execution matching OEM `Task.Run`/`Task.WaitAll`; not equivalent to single-axis Home or Zero.
- Raw FastAPI route `/motion/oem/initialization/initialize_motors` (POST): canonical serial-206 provider entry point for the next approved `initializeMotors` stage.
- Raw FastAPI route `/motion/oem/initialization/initialize_motion` (POST): canonical serial-206 provider entry point for the next approved `initializeMotion` stage.
- Raw FastAPI route `/motion/oem/initialization/provider-status` (GET): canonical atomic initialization-state projection.
- Raw FastAPI route `/motion/axis/home` (POST): provider-owned source-shaped leaf; direct external mutation is retired.
- Raw FastAPI route `/motion/axis/zero` (POST): Linux absolute controller-zero route, not OEM switch/reference homing.
- BMS `/api/bioxp/operator-controls/actions/{action_id}/invoke`: thin proxy for robot-owned catalog actions and receipts; BMS does not own a second mutation policy.

## Live Linux acceptance note

Source-shaped software contracts are present. Controller and physical acceptance remain pending. Focused tests prove software/source consistency and do not establish physical parity.

Machine-readable matrix source: `src/bioxp/oem_homing_model.py::source_matrix()`.


## Source-to-live-target status

Current software binding is source-shaped and provider-owned. Controller and physical acceptance remain separate gates:

- `initializeMotors` and `initializeMotion` -> `Serial206OemInitializationProvider` (`oem_serial206_initialization.py:3529`): one atomic authority owns admission, expected-stage execution, observation, persistence, and receipts.
- startup `axisSearchHome` -> `motor_oem_axis_search_home` (`usb_driver.py:5576`): the canonical serial-206 provider owns the source-shaped leaf; switch transitions are telemetry rather than an OEM success predicate.
- manual button `goHome(true)` -> `motor_oem_go_home` / `motor_oem_home_axis(startup=False)` (`usb_driver.py:5664`, `7056`): direct external mutation is retired; canonical operator/provider dispatch owns execution.
- `doorSearchHome` -> `motor_oem_door_search_home` (`usb_driver.py:6558`): source-shaped software contract present; physical predicate confirmation remains pending.
- `HomeXY` -> `BioXpTester.motor_oem_home_xy` (`usb_driver.py:7147`): provider-owned leaf preserves concurrent X/Y `goHome(false, axis, 200, true)` calls and OEM `Task.Run`/`Task.WaitAll` profile restoration.
- Retired compatibility wrappers (`startup_step`, `rehome`, the no-homing `initialize_motion` diagnostic, and `initialization/run`) are absent from executable source and route topology.

Raw FastAPI route table was enumerated by importing `src.bioxp.api:app` only; no USB or motion endpoint was called.
