# Reverse Engineering Traceability

This document links active Linux control behavior to known Windows/decompiled origins.

## 1. Purpose
The project intentionally reuses proven OEM semantics while replacing the Windows runtime with a direct Linux control plane.

Traceability provides:

1. confidence that command behavior is not guessed
2. auditability when changing control logic
3. a clear rationale for added hardening wrappers

## 2. Key Source References
Windows backup/decompiled roots used as reference:

- `BioXP_Original_Backup/BioXP_SSD_Backup/decompiled_src/`
- `BioXP_Original_Backup/BioXP_SSD_Backup/decompiled_src_can/`

Examples used during runtime alignment:

- `ClassCanLib/ClassIOControl.cs`
- `BioXPControlLib/ClassControlInterface.cs`
- `BioXPControlLib/ControlLib.cs`

## 3. Mapping: Windows to Linux Runtime
### 3.1 LED Command Path
Windows reference:
- `ClassIOControl.setLED(byte RGBmask, int intensity)`
- scaling formula `1024 * intensity / 255`
- command structure around `cmd 50`

Linux implementation:
- `_led_scale_to_tmcl`
- `led_write`
- `strip_set_rgb`

Rationale:
- maintain verified visible LED behavior while adding retry/recovery and operator diagnostics.

### 3.2 Color Abstraction
Windows reference:
- `ClassControlInterface.setColor(R,G,B)` with 0..255 clamping

Linux implementation:
- `_clamp_u8`
- `strip_set_rgb` and higher-level menu actions

Rationale:
- preserve user-facing RGB semantics and clamp behavior.

### 3.3 Axis/Board Mapping
Windows reference:
- `m_AxisIODesignater` mappings (X/Y/Z/Gripper/ThermalDoor)

Linux implementation:
- `MOTOR_AXIS_PRESETS`
- `MOTOR_FUNCTION_PRESETS`

Rationale:
- avoid remapping risk and keep function-to-board alignment consistent.

### 3.4 Deck IO Semantics
Windows reference:
- `ClassIOControl` query/set methods for cmd14/cmd15 channels

Linux implementation:
- `deck_io_query_type`
- `deck_io_set_type`
- latch helpers and probe tooling

Rationale:
- preserve known sensor/solenoid channel semantics and make them testable in-session.

### 3.5 Thermal/Chiller Scaling and GP Patterns
Windows references and extracted behavior informed:
- board firmware/readout usage
- GP scaling conventions
- baseline parameter profiles

Linux implementation:
- `CHILLER_GP_SCALE`, `THERMAL_GP_SCALE`
- safe write allowlists
- vendor baseline helpers

Rationale:
- retain interpretable engineering units and avoid unsafe arbitrary writes.

## 4. Linux-Specific Hardening (Intentional Additions)
These are not strict OEM clones; they were added to stabilize Linux operations:

1. pacing wrappers (`_send_motor/_send_chiller/_send_thermal`)
2. no-response streak detection and reconnect policy
3. non-blocking burst panic/reset flows
4. explicit operator telemetry and readback checks
5. subsystem hard-reset menu actions

Why added:
- mixed-command sessions can degrade into reply starvation or intermittent no-response; hardening was required for practical control continuity.

## 5. Camera Path Note
`camera_smi_status` acknowledges Windows SMI LED-related artifacts but notes they are not directly callable from the Linux Python path.

Rationale:
- maintain clarity on what has been ported versus what remains Windows-only.

## 6. Practical Rule
When docs or historical notes conflict:

1. trust active behavior in `src/bioxp/usb_driver.py`
2. use decompiled references to explain intent and mapping
3. update this traceability file whenever runtime behavior diverges from old assumptions
