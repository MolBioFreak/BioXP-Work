# BioXP OEM Machine Calibration Binding Gap Review

Date: 2026-06-13

## Summary

The live thermal-door test exposed a broader OEM-parity issue: compiled OEM defaults are **not** sufficient for motion parity on a field-calibrated BioXP3200. OEM persists machine-specific calibration values in `config.xml`, and the Linux runtime must consume those values before declaring parity.

## Direct evidence

### Original SSD AppData config

Repo source:

- `config/oem/original_ssd_appdata_20260610/config.xml`

Relevant values:

```xml
<Offsets
  m_originOffsetG="4450"
  m_GripperClosePOS="27350"
  m_GripperOpenPOS="31400"
  m_GripperOpenWide="32400"
  m_TCDoorOpen="18500"
  m_TCDoorStallGuardThreshold="6"
  m_TC_DOOR_VELOCITY="50"
  m_TC_DOOR_ACCELERATION="20"
  m_TC_DOOR_MAX_CURRENT="31"
  m_Z_MOTOR_MAX_CURRENT_DOWN="25"
  m_Z_MOTOR_MAX_CURRENT_UP="31"
  m_Z_MOTOR_STALL_GUARD_THRESHOLD="3" />
```

Axis limits are also machine-specific:

```xml
<X_limit minSteps="0" maxSteps="90263" />
<Y_limit minSteps="0" maxSteps="102956" />
<Z_limit minSteps="0" maxSteps="160000" />
<G_limit minSteps="0" maxSteps="15000" />
```

### Decompiled OEM settings loader

`ClassBioXPSettings.cs` parses these persisted values:

- `m_GripperClosePOS`
- `m_GripperOpenPOS`
- `m_GripperOpenWide`
- `m_TCDoorOpen`
- `m_TCDoorStallGuardThreshold`
- `m_TC_DOOR_VELOCITY`
- `m_TC_DOOR_ACCELERATION`
- `m_TC_DOOR_MAX_CURRENT`
- Z current/stall values

The OEM writer stores the same fields back under `CalibrationFactors/Offsets`.

## Live test implication

The Linux runtime used source default `TCDoorOpen=16000`. The door moved to `16000` but did not assert `tcDoorOpened` / right switch. The original machine config says the calibrated open endpoint is `18500`, which matches Christian's physical observation that the door was not quite fully open.

## Correct parity rule

Runtime priority must be:

1. explicit machine config from original SSD/AppData `config.xml`,
2. environment-selected machine config bundle,
3. only then decompiled source defaults.

Compiled defaults are source-backed fallback values, not machine calibration truth.

## Immediate implemented fix

`BioXpTester._motion_oem_axis_profile("door")` now overlays the bound machine config:

- `open_position=18500`
- `open_position_source=original_ssd_machine_config`
- stallguard/velocity/acc/current also sourced from `CalibrationFactors/Offsets`

## Broader parity work required

This same rule applies beyond the thermal door:

- gripper close/open/open-wide positions must come from machine config, not hard-coded defaults;
- X/Y/Z/G limits must come from `AxisLimits`, not compiled fallbacks;
- any semantic deck move must use the machine-bound position table/config layer;
- status endpoints should expose source/provenance for every calibrated endpoint.

## Review gate before more live testing

Before declaring thermal-door open fixed, reload the API and verify the runtime profile reports/open uses `18500`, then perform a supervised open test. Success requires both:

- `Door` reaches `18500` or calibrated target;
- `tcDoorOpened=true` / right switch asserts.

If `18500` still does not assert right/open, treat it as either stale config or mechanical/switch adjustment, not as permission to silently raise the endpoint.
