# BioXP3200 live head-clearance and XY test spec — 2026-05-03

## Physical observation now proven

The first Linux command sequence in this session that produced real observed head movement was not the generic relative-move API path. It was the OEM-shaped head-clearance primitive:

1. release service USB ownership for direct maintenance control
2. instantiate `BioXpTester`
3. `activate_boards(expect_reply=True)`
4. `motor_prepare_motion_interlock(force_lock=True)`
5. `motor_ensure_head_clearance(force_rehome=True, preclear_abs=N, ensure_interlock=...)`
6. configured clearance axis: `z`
7. configured physical clearance direction: negative Z
8. reconnect service USB ownership

Observed live increments:

- `20260503_171450_head_lock_clearance_z_only`: Z `-2200 -> -4700`, operator observed head moved up slightly.
- `20260503_171605_head_lock_clearance_z_only_second_increment`: Z `-4700 -> -7200`, operator observed same correct direction.
- `20260503_171724_head_lock_clearance_z_to_minus10000`: Z `-7200 -> -10000`.
- `20260503_171838_head_lock_clearance_z_safety_margin_2500`: Z `-10000 -> -12500`.

Operator assessment: Z negative/up is the right direction and is intended to free the head from the locking mechanism; clearance was not complete at `-10000`, so `-12500` is the current commissioning safety-margin height.

## Runtime policy

- Generic `/motion/axis/relative` controller deltas are not accepted as physical proof during commissioning.
- Physical operator observation outranks controller position/speed telemetry.
- The previously observed `head-clearance-z-up` primitive remains historical physical evidence for negative-Z/up direction, not the first canonical OEM startup step.
- The current OEM-shaped live startup gate starts with supervised `z-home`; Z reference return is allowed only after the home result is confirmed OK.
- Monolithic all-axis homing remains blocked for operator-facing live startup; only explicit supervised stepwise commissioning gates should run.
- X/Y testing is only allowed after head-clearance is physically observed.
- For X/Y test commissioning, run one axis at a time with explicit artifacts and pauses.

## X/Y test plan

Given current switch state commonly shows right-limit active for X and Y, initial commissioning moves should use negative direction unless the operator explicitly requests otherwise.

1. passive snapshot: rail, latch, all axes.
2. X test: prepare X with OEM current/speed envelope, move X `-10000`, wait stopped, snapshot.
3. pause for operator observation.
4. Y test: prepare Y with OEM current/speed envelope, move Y `-10000`, wait stopped, snapshot.
5. pause for operator observation.
6. no return move until physical direction and clearance are confirmed.

## Next gates after X/Y

- gripper-clear: OEM source shape indicates `setGripperCurrent(31); MotorGrip.moveSteps(10000,true)` before gripper homing.
- pipette system: require ACK/readback-only diagnostics first; no liquid/deck interaction until gantry and gripper steps are source/physically verified.
