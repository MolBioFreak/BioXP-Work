# BioXP 3200 Project Task Tracker

## Historical Work (Completed)
- [x] Extract and analyze BioXP Windows SSD software stack.
- [x] Reverse engineer core DLL architecture and protocol surface.
- [x] Establish Linux USB/CAN control path and interactive driver shell.
- [x] Bring up latch, LED strip, camera, motor, chiller, and thermal menus.
- [x] Implement thermal and chiller hard reset / panic-off recovery actions.

## Current Checkpoint (2026-03-02)
- [x] Near full subsystem control achieved in `src/bioxp/usb_driver.py`.
- [x] Chiller and thermal telemetry/setpoint/fan/rate/PWM controls functioning.
- [x] Motor function/board/channel workflows and homing diagnostics functioning.

## Remaining Critical Minimums
- [ ] Add motor transport pacing + no-response auto-recovery wrapper.
- [ ] Add motor hard reset / panic-off menu action.
- [ ] Add standardized motion preflight guard (alive + 24V + interlock).
- [ ] Tighten thermal door homing path to vendor parity.
- [ ] Add 10-minute mixed-operation soak test and use as stability gate.

## Documentation Sync
- [x] Refresh stale RCA and fix-plan docs to current state.
- [x] Add dated progress checkpoint doc for near-full-control milestone.
