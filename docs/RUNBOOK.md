# Operator Runbook

This runbook is for day-to-day control via `src/bioxp/usb_driver.py`.

## 1. Startup
1. Connect instrument and USB adapter.
2. Run:
   - `sudo .venv/bin/python src/bioxp/usb_driver.py`
3. Confirm startup status print shows board responses.
4. If LED persisted state exists, startup auto-apply should report the applied RGB.

## 2. Top-Level Menu Strategy
Recommended control order for stable sessions:

1. `s` reconnect/status if session is stale.
2. Latch/interlock confirmation before major motion.
3. Motor operations.
4. Thermal/chiller adjustments.
5. Camera diagnostics as needed.

## 3. Latch Workflow
Typical sequence:

1. Enter `LATCH CONTROL`.
2. Use lock/unlock and verify with `DECK IO QUERY SWEEP`.
3. If behavior is unclear, run `DECK SIO PROBE` and `SOLENOID/HEAD LOCK PROBE`.

Success signals:
- expected write sends
- expected IO query value changes

If unstable:
- return to main menu and run `s` reconnect/status, then retry.

## 4. LED Workflow
### Static State
1. Enter `LED CONTROL`.
2. Use On/Off/level/custom RGB or preset list (`p`).
3. Confirm persisted default message after static selection.

### Persisted Apply
- Use `r` to re-apply persisted default at any time.

### Dynamic Effect
- Rainbow cycle (`a`) is duration-based and transient.

## 5. Motor Workflow
For gantry axis work:

1. `MOTOR CONTROL` -> `MOTION INTERLOCK PREP`.
2. Enter channel menu by function or board.
3. Run `Status` and `Prepare axis`.
4. Perform `Nudge` or controlled move.
5. Confirm wait-stopped and position deltas.

For reliability checks:
- use one-click diagnostics and smoke/hard-reset options in motor menu.

If comm drops occur:
- run `HARD RESET / PANIC OFF (HEAD+DECK)`.
- if still failing, run top-level `s` and then power-cycle hardware.

## 6. Thermal Door Workflow
1. Use `THERMAL DOOR CONTROL` menu.
2. Prefer sensor-guided `OPEN`/`CLOSE` over blind absolute moves.
3. Verify switch states after motion.

## 7. Chiller Workflow
1. Run `Reachability check` first.
2. Use `Telemetry snapshot` to establish baseline.
3. Apply vendor baseline if needed.
4. Perform setpoint/fan/rate/PWM changes with readback.
5. Use safe GP write only for allowlisted params.

If no-response:
- run `HARD RESET / PANIC OFF` in chiller menu.

## 8. Thermal Cycler Workflow
1. Run `Reachability check`.
2. Use `Telemetry snapshot`.
3. Apply vendor baseline or fast profile.
4. Set nest/lid/pedestal/fan/rates/PWM as required.
5. Use safe GP write only for allowlisted params.

If no-response:
- run `HARD RESET / PANIC OFF` in thermal menu.

## 9. Camera Workflow
Quick checks:

1. `AUTO CAMERA CHECK` for one-click recover + profile.
2. `CAPTURE SNAPSHOT` and `STREAM HEALTH TEST`.

Advanced tasks:
- open advanced menu for V4L2 raw controls, XU sweep/profile, USB reset, and auto one-click recovery.

If busy/device drift:
- use camera reset and re-enumeration tools in advanced menu.

## 10. Failure Handling Ladder
Use this escalation path:

1. Retry the same action once.
2. Top-level `s` reconnect/status.
3. Use subsystem hard reset/panic-off action.
4. Reconnect/status again.
5. Power-cycle instrument/adapter if still unrecovered.

## 11. Validation Checklist After Recovery
1. Board status replies for expected boards.
2. Subsystem reachability check passes.
3. One low-risk control action succeeds with expected readback.
4. No immediate no-response recurrence under light command load.
