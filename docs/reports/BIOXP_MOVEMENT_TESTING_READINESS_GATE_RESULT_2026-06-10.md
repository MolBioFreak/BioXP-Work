# BioXP Movement Testing Readiness Gate Result

Date: 2026-06-10

Artifact directory:

```text
/tmp/bioxp_movement_gate_20260610T051948Z
```

## Action taken

Ran the non-homing readiness activation gate only:

```text
POST /motion/power/enable
POST /motion/arm/strict_startup {"run_homing": false}
```

No homing command was issued. No axis movement command was issued.

## Result

```text
strict_ok: true
strict_reason: strict_init_pass
motion_arm.armed: true
motion_arm.reason: strict_init_pass
rail_24v.no24v: false
```

Raw latch/interlock truth:

```text
door_sensor: 1
solenoid_state: 1
latch_sensor: 1
rail_24v: 0
```

Axis stopped state:

```text
X speed: 0, position: 0
Y speed: 0, position: 0
Z speed: 0, position: 0
G speed: 0, position: 0
door speed: 0, position: 0
```

Reference state remains explicitly untrusted:

```text
X: desynced
Y: desynced
Z: desynced
G: desynced
door: desynced
```

Gripper idle-current safety:

```text
classification: G_CURRENT_IDLE_SAFE
G speed: 0
run_current_param6: 10
standby_current_param7: 10
safe_idle_max: 10
```

## Movement-test implication

The robot is now logically/controller-side armed for a **supervised relative proof move only**.

Still not authorized/ready for:

- blind absolute moves
- full OEM path execution
- switch-search homing
- path moves from PositionTable as if reference were trusted

First actual movement test should be:

1. operator/camera physically watching
2. before image or operator-confirmed starting pose
3. one axis only
4. small relative step
5. immediate stop/status capture
6. after image/operator confirmation
7. result classified as physical only if observation/camera confirms it

Controller counters are accepted as the controller-side result; commissioning still records observed instrument behavior.
