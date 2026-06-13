# BioXP Thermal Door OEM-Parity Implementation Status

Date: 2026-06-13

## Summary

Thermal-door OEM parity implementation is complete through the non-live code/test phases and is ready for supervised runtime reload/live validation.

No live robot motion was executed during this implementation tranche. No API/service restart was performed.

## Commits

- `bb38e68` — `feat: model OEM thermal door settings`
- `2022269` — `fix: align thermal door profile with OEM settings`
- `dbb544f` — `feat: add OEM thermal door driver operations`
- `6c53753` — `feat: expose OEM thermal door API routes`

Planning/review commits:

- `1852329` — `docs: add thermal door OEM parity gap review`
- `bd5a316` — `docs: plan thermal door OEM parity phases`

## OEM spec comparison

### Settings

Implemented:

- serial `<10`: `TCDoorOpen=93000`, `TC_DOOR_VELOCITY=900`
- serial `>=10`: `TCDoorOpen=16000`, `TC_DOOR_VELOCITY=50`
- all: `TC_DOOR_ACCELERATION=20`, `TC_DOOR_MAX_CURRENT=31`, `TCDoorStallGuardThreshold=6`

Source: `ClassBioXPSettings.cs` lines 266, 308, 310, 312, 314, 779-785.

### Door profile/setup

Implemented in `BioXpTester.MOTOR_FUNCTION_PRESETS["door"]`:

- speed/home speed 50
- acceleration 20
- run current 31
- standby current 10
- stallguard 6
- open position 16000
- close position 0
- both switch masks true

Diagnostic menu now derives open/close from `open_position`/`close_position`; old ad-hoc `10750/-7000` targets are removed from the target assignment.

### Driver operations

Implemented:

- `motor_thermal_door_status()`
- `motor_oem_door_search_home()` enhanced with before/after closed/open predicates
- `motor_oem_open_thermal_door()`
- `motor_oem_close_thermal_door()`

Success is predicate-backed:

- open success requires `tcDoorOpened`
- close/home success requires `tcDoorClosed`
- coordinate-only movement is not success

### API routes

Implemented:

- `POST /motion/thermal_door/home` with ack `HOME_THERMAL_DOOR`
- `POST /motion/thermal_door/open` with ack `OPEN_THERMAL_DOOR`
- `POST /motion/thermal_door/close` with ack `CLOSE_THERMAL_DOOR`

Failures return HTTP 409 with JSON detail instead of hiding partial movement.

## Verification

Command:

```bash
python3 -m py_compile src/bioxp/oem_config.py src/bioxp/usb_driver.py src/bioxp/api.py
PYTHONPATH=$PWD .venv/bin/python -m pytest \
  tests/test_oem_thermal_door_config.py \
  tests/test_oem_thermal_door_profile.py \
  tests/test_oem_thermal_door_operations.py \
  tests/test_oem_thermal_door_api.py \
  tests/test_bioxp_oem_initialize_motors_live_parity.py \
  tests/test_oem_switch_predicate_interpretation.py \
  tests/test_oem_gripper_contract.py -q
```

Result:

```text
31 passed in 7.47s
```

## Ready-for-testing gate

Next step is **not automatic live execution**. To test on hardware:

1. Explicitly approve API reload/service restart.
2. Passive status readback after reload.
3. Run thermal-door home route.
4. Run thermal-door open route and confirm both API predicate and camera/operator observation.
5. Run thermal-door close route and confirm closed predicate.

Stop and write RCA if physical observation conflicts with API predicate.
