# BioXP serial-206 Z-axis OEM closure specification

**Status:** active correction; source contract frozen; deployment and functional commissioning not performed by this change.

## Authority and physical identity

- Machine: BioXP serial 206.
- Z transport identity: head board/CAN `4`, motor `1`.
- OEM C# authority:
  - `ClassControlInterface.cs:2350-2368` — diagnostic-panel Z Home: `goHome(true, Z, 1791, true)`.
  - `ClassControlInterface.cs:4254-4265` — `moveZ(z, current=31)`: clamp to dynamic `PSUDO_Z_HOME`, set Z max current, then `moveToAbs`.
  - `ClassControlInterface.cs:4623-4632` — `MoveZHome(rehome=true)`: set/read max current 31, then `goHome(rehome, Z, 1791, true)`.
  - `ClassControlInterface.cs:4657-4687` — `homeGZ`: move Z to `PSUDO_Z_HOME`, home gripper, restore Z to `PSUDO_Z_HOME`; not a generic Z-home substitute.
  - `ClassControlInterface.cs:4997-5052` — `HomeAxis("z")`: Z-current prelude then `axisSearchHome(Z, 597)`.
  - `ClassControlInterface.cs:3348-3353` — `initializeMotors` M01: `axisSearchHome(Z, 1791)`.
  - `ClassHeadBoard.cs:60-124` — `goHome`: optional `moveToAbs(10000)` on rehome, `moveLeft(speed)`, wait/timeout, `queryHome`, stop, then `setHome`.
  - `ClassHeadBoard.cs:368-400` — `axisSearchHome`: `setHome`, optional `moveToAbs(10000)` when home is active, 500-ms wait, then `goHome(false, speed)`; `queryHome` is the left-switch predicate.
  - `ClassMotor.cs:492-516` — `setHome`: host coordinate reset plus the sole controller write `SAP param 1 = 0`; no profile preparation, interlock check, movement, or search.
  - `ClassMotor.cs:641-663` — raw GAP9 value `1` is converted by OEM `queryLeftSwitchStatus()` to return `0`, then `queryHome()` treats that return code as true. Linux raw GAP9 active value is therefore `1`.

## Required separate Z modes

| OEM mode | Required Linux semantic | Current source surface |
|---|---|---|
| `initializeMotors` M01 | `axisSearchHome(Z,1791)`; first physical startup stage | `z_startup_home` / initialization ledger `z-home` |
| panel Z Home | `goHome(true,Z,1791,true)` | `z_manual_home` / `oem.z.manual_home` |
| board-test `HomeAxis("z")` | current=31 then `axisSearchHome(Z,597)` | `z_diagnostic_home_axis` |
| `MoveZHome` | current=31 then `goHome(rehome,Z,1791,true)` | `motor_oem_move_z_home` |
| no-motion `setHome` | SAP1=0 at the operator-confirmed physical position | `z_set_home` / `oem.z.set_home` |
| `moveZ` | dynamic `max(PSUDO_Z_HOME, request)`, current=31, absolute move | `z_move_absolute` |
| `homeGZ` | coupled Z/gripper transaction with its own branches | `motor_oem_home_gz`; not admitted as a Z-home alias |

## Concrete missing condition and correction

### Defect

The live reported Z coordinate was stale/desynced while the gantry was physically at zero. The only OEM primitive that resets that controller coordinate without movement is `ClassMotor.setHome`.

Linux incorrectly routed `z_set_home()` through `_z_profile()`. That helper requires a Linux no-motion profile and motion interlock, despite neither being part of OEM `ClassMotor.setHome`. The provider also allowed `set_home` only after `prepared_unreferenced` or `referenced_ready`, creating a circular recovery gate: the state-establishing operation required the state it establishes.

### Implemented correction

`src/bioxp/oem_serial206_initialization.py` now:

1. binds no-motion Z `setHome` only to the immutable serial-206 board/motor identity (`4/1`);
2. performs the exact OEM SAP1=0 write and position-zero readback without a profile, interlock, homing, or movement prelude;
3. admits this no-motion state-establishing operation from `unprepared`, `failed_latched`, `prepared_unreferenced`, or `referenced_ready` lifecycle states;
4. permits it across stale preparation generation because it establishes fresh controller reference rather than consuming preparation; and
5. preserves operator-plane ownership generation, idempotency, durable receipt, controller readback, and robot-local reference publication.

This is a narrow parity correction. It does not alter `axisSearchHome`, `goHome`, `MoveZHome`, `HomeAxis`, `moveZ`, `homeGZ`, direction, switch polarity, current, bounds, or any physical motion sequence.

## Outstanding acceptance boundary

After the exact no-motion `setHome` write returns acknowledged and Z position reads zero, the controller/reference state can be called established. It is not proof of physical motion or switch home.

Physical Z functional testing remains separate and must use one selected source mode, not a generic Z action:

1. startup M01 `axisSearchHome(Z,1791)`; or
2. panel/manual `MoveZHome(true)` at 1791; or
3. board-test `HomeAxis("z")` at 597.

Each physical run requires its own bounded operator authorization and must record controller ACK, speed-zero/stop, GAP9 home predicate, SAP1 zero readback, and supervised physical observation. No such test or deployment is included here.
