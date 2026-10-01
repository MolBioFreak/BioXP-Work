# BioXP3200 OEM Phase2 Motion Progress — 2026-05-03

## Scope

Phase2 objective is OEM startup/homing/motion constants plus a truth-gated physical validation harness. This document records the current workstation-safe progress and claim boundaries.

## Existing verified runtime coverage

Targeted tests already verify:

- `motor_oem_home_axis("x")` uses OEM/manual-home style speed and axis prep expectations.
- Gripper version-one homing uses the slower OEM speed and restores current afterward.
- `motor_startup_homing_mimic()` follows the OEM-style startup sequence:
  - reconnect
  - initialize without motion
  - gripper +10000 pre-move
  - home Z
  - home gripper
  - home X
  - set home X
  - set X speed
  - move X to 6000
  - home Y
  - home door
- API `_execute_home_axis(...)` routes through `tester.motor_oem_home_axis(...)`.
- Home speed overrides above the OEM profile are rejected before dispatch.
- Nested OEM home payloads are handled by strict startup validation.
- Relative move responses expose controller-only motion truth metadata.
- Prepared-motion reuse is disabled by default and still forces fresh interlock wake even in debug reuse mode.

## Historical helper workflow

The supervised shell helpers and their shell-only checks are retired; see
[the surviving manual/recovery interfaces](TOOLING_RETIREMENT.md).

## Claim boundary

This is still not live physical proof. Phase2 is partially implemented and workstation-verified. Full phase2 compliance still requires supervised robot-local validation with independent physical observation/camera/fiducial confirmation, because controller counters are not sufficient proof of motion on this BioXP.
