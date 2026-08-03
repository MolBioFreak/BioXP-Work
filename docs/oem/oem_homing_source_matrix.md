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
- Raw FastAPI route `/motion/axis/home` (POST): manual-button/goHome-style route (`startup=False` historically), not equivalent to startup `axisSearchHome`/full re-reference.
- Raw FastAPI route `/motion/axis/zero` (POST): Linux absolute controller-zero route, not OEM switch/reference homing.
- BMS `/api/bioxp/operator-controls/actions/{action_id}/invoke`: thin proxy for robot-owned catalog actions and receipts; BMS does not own a second mutation policy.

## Live Linux deviation note

The current live robot implementation is a guarded Linux reconstruction with safety/workaround paths, not a clean one-for-one OEM port. In particular, Z startup live handling may use a GAP10/controller-zero workaround while the source model keeps OEM `ClassHeadBoard.queryHome`/left-switch provenance separate. Focused tests prove source-model consistency only, not physical/OEM parity.

Machine-readable matrix source: `src/bioxp/oem_homing_model.py::source_matrix()`.


## Source-to-live-target status

This is the part that was missing from the first pass. Current Linux target status is explicitly not clean parity:

- `initializeMotorsWithoutMotion` -> `BioXpTester.motor_oem_initialize_without_motion` (`usb_driver.py:3397`): source-shaped setup, constants partly reconstructed/defaulted without recovered `config.xml`.
- `initializeMotors` and `initializeMotion` -> `Serial206OemInitializationProvider`: one atomic authority owns admission, expected-stage execution, observation, persistence, and receipts.
- startup `axisSearchHome` -> `motor_oem_axis_search_home` (`usb_driver.py:3418`): partial guarded reconstruction; Z may use GAP10/controller-zero workaround instead of source GAP9 search.
- manual button `goHome(true)` -> `motor_oem_go_home` / `motor_oem_home_axis(startup=False)` (`usb_driver.py:3510`, `3833`; `/motion/axis/home` at `api.py:2894`): unsafe until predicate matrix and deassert->active proof are fixed.
- `doorSearchHome` -> `motor_oem_door_search_home` (`usb_driver.py:3725`): separate partial reconstruction, needs physical predicate proof.
- `HomeXY` -> `BioXpTester.motor_oem_home_xy` / `/motion/oem/home_xy`: direct mode label/setup/restore surface now exists; X/Y `goHome(false, axis, 200, true)` calls are launched concurrently to match OEM `Task.Run`/`Task.WaitAll`; this is separate from manual single-axis Home and Zero.
- Retired compatibility wrappers (`startup_step`, `rehome`, the no-homing `initialize_motion` diagnostic, and `initialization/run`) are absent from executable source and route topology.

Raw FastAPI route table was enumerated by importing `src.bioxp.api:app` only; no USB or motion endpoint was called.
