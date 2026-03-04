# Architecture and Control Plane

## 1. Scope
This document describes the active Linux control architecture implemented in `src/bioxp/usb_driver.py`.

## 2. System Overview
The current control path is a Python runtime that talks directly to the Atmel Novo USB-to-CAN adapter and sends TMCL-style framed commands to instrument boards.

Primary layers:

1. USB transport and framing
2. TMCL request/reply handling
3. reliability wrappers and pacing
4. subsystem control primitives (latch/LED/motor/chiller/thermal/camera)
5. interactive operator menus

## 3. Hardware and Board Topology
Board IDs used by the runtime:

- `0x04`: HEAD
- `0x05`: DECK
- `0x06`: THERMAL
- `0x07`: CHILLER

These are defined in `BioXpTester` constants and reused across all subsystem methods.

## 4. Core Transport Model
### 4.1 Frame Construction
`_build_frame(board_id, command, cmd_type, motor, value)` builds a framed payload:

- start sentinel `0x7E`
- board/command/type/motor
- 32-bit signed big-endian value
- checksum
- end sentinel `0x7E`

### 4.2 Send/Reply Path
`send_tmcl(...)`:

1. drains stale USB RX frames
2. writes frame to EP OUT
3. optionally waits for a matching reply

`_wait_for_reply(...)` filters:

- known async heartbeat traffic
- short/non-command frames
- non-matching board/cmd when strict mode is enabled

### 4.3 Retry Behavior
`send_tmcl_retry(...)` retries identical sends with bounded attempts.

Rationale: USB/CAN bridge traffic can be bursty, and single-shot sends are not consistently reliable under mixed subsystem activity.

## 5. Reliability and Recovery Layer
### 5.1 Global Recovery
`reconnect()` fully releases/reset/rebinds USB interface and resets streak/pacing state.

`status_with_recovery()` retries board status after reconnect if all boards initially fail.

### 5.2 Subsystem Pacing Wrappers
Dedicated wrappers enforce minimum TX gaps and controlled recovery:

- `_send_motor(...)`
- `_send_chiller(...)`
- `_send_thermal(...)`

Each wrapper provides:

1. pacing guard (`*_pace`)
2. retry-based send
3. no-response streak tracking
4. one-shot reconnect/retry policy after repeated no-response

Rationale: This reduces bus flooding, response starvation, and cascading failures under sustained command traffic.

### 5.3 Write-Only Burst Paths
Some recovery/reset actions use non-blocking burst sends, for example:

- `_motor_send_noreply_burst(...)`
- `_chiller_send_noreply_burst(...)`
- `_thermal_send_noreply_burst(...)`
- `latch_oneshot(...)`

Rationale: In degraded states, ack paths can fail while transmit still succeeds. Burst no-reply sequences provide a deterministic panic-off/recovery primitive.

## 6. Canonical Command Surface in Use
Major command IDs currently exercised by runtime code:

- `64`: board activate/deactivate
- `173`: firmware query
- `14`: SIO set
- `15`: GIO query
- `50`: LED set
- `5`: SAP (set axis param)
- `6`: GAP (get axis param)
- motor motion commands (relative/absolute/rotate/stop/home-related paths)

TMCL status decoding uses `TMCL_STATUS` (success, busy, invalid value, stall, etc.).

## 7. Menu and Operator Layer
Top-level control menu in `usb_driver.py` dispatches to subsystem menus:

1. LATCH CONTROL
2. LED CONTROL
3. CAMERA SYSTEM
4. MOTOR CONTROL
5. CHILLER SYSTEM
6. THERMAL CYCLER
7. RECONNECT USB + STATUS

All menu actions call the same subsystem primitives used elsewhere in script code, so menu behavior is the practical control contract.

## 8. Why This Architecture
This architecture exists to preserve OEM behavior where useful while improving Linux-side operability:

1. Keep board IDs, command encodings, and scaling rules aligned to decompiled Windows behavior.
2. Remove dependence on Windows runtime by talking directly to device transport.
3. Add explicit pacing/recovery controls to survive long mixed-control sessions.
4. Keep tooling interactive and debuggable for hardware bring-up and fault recovery.

## 9. Boundaries and Non-Goals
- Camera SMI LED path from Windows vision DLLs is not directly callable in this Linux runtime.
- `src/bioxp/can_driver.py` and `src/bioxp/api.py` are not the active hardware control runtime.
- Primary operator/runtime truth remains `src/bioxp/usb_driver.py`.
