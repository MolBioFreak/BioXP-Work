# Subsystems and Operations

This document describes each active subsystem control surface in `src/bioxp/usb_driver.py`.

## 1. Latch and Deck IO
### What It Controls
Deck-side latch/solenoid and IO sensor channels.

### Primary Methods
- `latch_oneshot`, `latch_oem`, `latch_compat`
- `deck_io_query_type`, `deck_io_set_type`
- `deck_io_query_matrix`, `deck_io_sio_probe`
- `head_lock_actuator_probe`

### Command Semantics
- `cmd14` set IO type/value
- `cmd15` query IO type/value
- Known mappings used in code/comments:
  - type0: 24V sensor
  - type1: door sensor
  - type2: solenoid/latch control state path
  - type3: latch sensor

### Use Cases
1. Lock/unlock and pulse latch for interlock testing.
2. Verify sensor states before motion workflows.
3. Probe lock-state effects on head Y/Z micro motion.

### Why Multiple Paths Exist
- `latch_oem`: ack-first aligned path.
- `latch_compat`: broad and tolerant path for unstable sessions.
- `latch_oneshot`: write-only forced reconnect path when replies are unreliable.

## 2. LED (Main Strip)
### What It Controls
Main strip RGB channels via deck board LED command path.

### Primary Methods
- `led_write`, `led_mask_scaled`, `led_rgb_scaled`
- `strip_set_rgb`, `strip_set_pct`, `strip_on`, `strip_off`
- `strip_rainbow_cycle`, `led_firstpath_restart`
- `save_led_state`, `apply_saved_led_state`

### Command Semantics
- `cmd50`, type `0`, mask-based RGB write
- intensity scaling: `tmcl_value = (1024 * intensity) / 255`

### Current Menu Capabilities
- on/off/intensity presets/custom RGB
- rainbow cycle
- preset color submenu (20 named colors)
- persisted default save/reapply

### Use Cases
1. Static operator illumination states (persisted).
2. Quick visual diagnostics (re-prime/restart path).
3. Temporized visual cycle tests (rainbow).

### Why Persistence Was Added
LED menu previously favored transient toggles/effects. Persistence provides deterministic default visual state across restarts/sessions for repeatable operation.

## 3. Motor System
### What It Controls
Axis control on head/deck/thermal motor channels.

Function presets:
- X: `0x05:0`
- Y: `0x04:0`
- Z: `0x04:1`
- GRIPPER: `0x04:2`
- THERMAL_DOOR: `0x06:0`

### Primary Methods
- Axis IO: `motor_get_axis_param`, `motor_set_axis_param`, status helpers
- Motion: `motor_move_relative`, `motor_move_absolute`, `motor_stop`
- Homing/seek: `motor_axis_search_home`, `motor_startup_homing_mimic`
- Prep/guard: `motor_prepare_axis`, `motor_prepare_motion_interlock`
- Diagnostics: `motor_step_test`, `motor_visible_oneway_probe`, driver/profile diagnostics
- Recovery: `motor_hard_reset`, `motor_quick_reliability_smoke`

### Control Patterns
1. Preflight/interlock prep for gantry motion.
2. Parameterized axis prep (speed, accel, current, stall, switch masks).
3. Motion command + wait-for-stopped verification.
4. Optional visibility probes and event/stall inspection.

### Use Cases
1. Routine positioning and homing.
2. Controlled bring-up of idle hardware.
3. Driver profile comparison/copy experiments.
4. Recovery from no-response via panic-off/hard reset.

### Why This Tooling Exists
Motor path has highest command traffic and highest lockup risk. The runtime includes extensive guard/prep/diagnostic and reset tooling to keep failures observable and recoverable without rebooting the entire stack.

## 4. Thermal Door (Focused Motor Subsystem)
### What It Controls
Thermal door axis (`0x06:0`) with sensor-guided open/close semantics.

### Primary Methods
- `run_thermal_door_menu` helper flow
- `_door_seek` logic using switch activity states

### Use Cases
1. Sensor-guided open/close with explicit state verification.
2. Fine-tuning seek step/max move limits for instrument-specific behavior.

### Why Separate Menu Exists
Door behavior is safety-sensitive and semantically different from generic axis moves. Dedicated controls provide clearer operator intent and diagnostics.

## 5. Chiller System
### What It Controls
Chiller board telemetry and setpoint/rates/fan/PWM controls.

### Primary Methods
- `chiller_activate`, `chiller_query_firmware`, `chiller_read_temps`
- `chiller_gp_read`, `chiller_gp_write`
- `chiller_set_target_temp`, `chiller_set_fan`, `chiller_set_rates`, `chiller_set_pwm`
- `chiller_apply_vendor_baseline`, `chiller_hard_reset`

### Safety and Guardrails
- safe write parameter allowlist: `CHILLER_SAFE_WRITE_PARAMS`
- pacing and settle delays
- retry/recovery wrapper `_send_chiller`

### Use Cases
1. Reachability and health checks.
2. Applying known-good baseline profile.
3. Controlled thermal setpoint and tuning operations.
4. Panic-off/hard-reset recovery on comm failure.

## 6. Thermal Cycler
### What It Controls
Thermal board nest/lid temperatures, fan, rates, PWM, and GP paths.

### Primary Methods
- `thermal_activate`, `thermal_query_firmware`, `thermal_read_temps`
- `thermal_gp_read`, `thermal_gp_write`
- `thermal_set_target_temp`, `thermal_set_ped_temp`, `thermal_set_fan`
- `thermal_set_rates`, `thermal_set_pwm`
- `thermal_apply_vendor_baseline`, `thermal_apply_fast_profile`, `thermal_hard_reset`

### Safety and Guardrails
- safe write parameter allowlist: `THERMAL_SAFE_WRITE_PARAMS`
- pacing and settle delays
- retry/recovery wrapper `_send_thermal`

### Use Cases
1. Standard baseline operation.
2. Faster profile experiments.
3. Controlled setpoint/rate tuning with readback verification.
4. Recovery from no-response states.

## 7. Camera System
### What It Controls
Linux V4L2 controls and vendor UVC extension-unit (XU) probing/recovery.

### Primary Methods
- Device/control: `camera_devices`, `camera_enumerate_controls`, `v4l2_set_ctrl`
- XU tooling: `uvc_xu_query`, `camera_xu_sweep`, selector profile helpers
- Media checks: snapshot and stream health
- Recovery: owner release, USB soft reset, one-click auto flow

### Use Cases
1. Snapshot/stream verification.
2. V4L2 control tuning (gain/exposure/backlight).
3. XU control characterization and profiling.
4. Automated recovery when camera path is busy or drifting.

### Why Advanced Camera Tooling Exists
Camera path is less deterministic across devices and ownership states. The script includes diagnostics, auto-sweep, and reset/re-enumeration logic to make failures actionable in-session.

## 8. Cross-Subsystem Patterns
Common design traits across subsystems:

1. explicit ack/readback printing
2. safe ranges and input clamping
3. conservative defaults for potentially dangerous motion
4. one-command recovery hooks for degraded bus states
5. operator-visible telemetry before and after control actions
