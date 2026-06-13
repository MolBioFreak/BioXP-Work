# OEM Gripper Initialization Semantics Review

Date: 2026-06-13

## Source anchors

See `docs/references/2026-06-13-oem-gripper-source-anchors.md`.

Key OEM facts:

- `MotorGrip` is the head board gripper axis (`ClassControlInterface.cs:56-60`).
- Initialization performs `setGripperCurrent(31)`, `moveSteps(MotorGrip,+10000,true)`, then version-specific gripper home (`600` for version 0, `200` for version 1) (`ClassControlInterface.cs:3354-3365`).
- Gripper confirmation is not generic left/right double-limit logic. OEM `confirmAxis("g"/"gripper")` is `queryHome(MotorGrip)` OR `getG()<50` (`ClassControlInterface.cs:2736-2745`).
- This robot's original SSD machine config provides gripper calibration fields under `CalibrationFactors/Offsets`:
  - `m_originOffsetG=4450`
  - `m_GripperClosePOS=27350`
  - `m_GripperOpenPOS=31400`
  - `m_GripperOpenWide=32400`

## Linux parity decision

The previous generic blocker `both_effective_limits_active` is not an OEM gripper predicate. It is retained as raw diagnostic evidence but is not a hard blocker by itself for gripper clear/home.

Linux gripper status now reports:

- raw GAP9/GAP10 switch state and masks,
- OEM home predicate (`queryHome OR getG()<50`),
- machine-config gripper positions,
- idle-current safety blocker (`g_current_hot_while_idle`).

Motion remains gated by operator ack/reason, current restore, wait-stopped evidence, and OEM predicate/status readback.
