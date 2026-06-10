# BioXP live homing runbook — 2026-04-13

This is the shortest practical operator runbook for supervised homing / re-reference testing tonight.

## Operator role for the live session

This document assumes Hermes runs the commands and the human operator only supervises physical behavior.

Operator responsibilities during live motion:

- watch the machine while I issue the API calls
- confirm whether each axis physically moved as expected
- stop me immediately if anything sounds wrong, binds, stalls, or moves in an unexpected direction
- do not spend time typing routine curls unless local robot-host access is strictly needed

## Prepared command wrappers

These helper scripts are ready so I can drive the session with one command per step and capture logs automatically:

```bash
/home/dalab/Desktop/bioxp_re/scripts/bioxp_motion_readiness_snapshot.sh
/home/dalab/Desktop/bioxp_re/scripts/bioxp_supervised_strict_startup.sh
/home/dalab/Desktop/bioxp_re/scripts/bioxp_supervised_home_axis.sh
```

They save timestamped payload logs under `/tmp/bioxp-live-runs` by default.

## Which API base to use

Best current choice from this workstation, verified live:

```bash
export BIOXP_BASE_URL=http://robot:8123
```

If you are actually on the robot host itself, this is equivalent:

```bash
export BIOXP_BASE_URL=http://127.0.0.1:8123
```

Fallback through the BMS proxy:

```bash
export BIOXP_BASE_URL=http://127.0.0.1:8000/api/bioxp
```

## Current pre-live state already checked

Read-only checks completed before live testing:

- BMS proxy `http://127.0.0.1:8000/api/bioxp/status` -> `status=ok`, `hardware_connected=true`
- direct robot API `http://robot:8123/status` -> reachable and healthy
- `/motion/power/status` -> `rail_24v.no24v=false`, `motion_arm.armed=true`, `motion_arm.reason=strict_init_pass`
- `/latch/status` -> `rail_24v=0`, `door_sensor=1`, `solenoid_state=1`, `latch_sensor=1`
- `/motion/axes/status?axes=x,y,z,g,door` returned healthy passive snapshots
- direct `/motion/reference/status?axes=x,y,z,g,door` is reachable on `http://robot:8123` and currently reports all axes `unknown`

Observed passive positions before tonight's homing test:

- X: `1540`
- Y: `160`
- Z: `-6000`
- G: `0`
- door: `0`

Interpretation: the stack is alive and armed, but these coordinates should not be treated as trustworthy machine-zero until supervised homing / re-reference is run again.

## Known proxy limitations right now

Through `http://127.0.0.1:8000/api/bioxp` on this setup:

- `GET /motion/reference/status` currently returns `404`
- dry-run artifact-bundle requests to `/motion/axis/home` and `/motion/axis/relative` currently return `500`

So for tonight:

- use the proxy for normal status and motion calls if needed
- use the robot-local API for reference metadata if you want `/motion/reference/*`
- do not depend on dry-run bundle generation tonight

## One-command readiness snapshot

A helper script is ready here:

```bash
/home/dalab/Desktop/bioxp_re/scripts/bioxp_motion_readiness_snapshot.sh
```

Example:

```bash
BIOXP_BASE_URL=http://robot:8123 /home/dalab/Desktop/bioxp_re/scripts/bioxp_motion_readiness_snapshot.sh
```

## If motion is no longer armed when you get home

Preferred assistant-operated wrapper:

```bash
BIOXP_BASE_URL="$BIOXP_BASE_URL" /home/dalab/Desktop/bioxp_re/scripts/bioxp_supervised_strict_startup.sh
```

Equivalent raw API call:

```bash
curl -sS -X POST "$BIOXP_BASE_URL/motion/arm/strict_startup" \
  -H 'Content-Type: application/json' \
  -d '{"run_homing":false}'
```

Then re-check:

```bash
curl -sS "$BIOXP_BASE_URL/motion/power/status"
curl -sS "$BIOXP_BASE_URL/latch/status"
```

Do not start homing unless all of the following are still true:

- `motion_arm.armed=true`
- `motion_arm.reason=strict_init_pass`
- `rail_24v.no24v=false`
- `door_sensor=1`
- `solenoid_state=1`
- `latch_sensor=1`

## OEM-style home commands

Preferred assistant-operated wrapper shape:

```bash
BIOXP_BASE_URL="$BIOXP_BASE_URL" /home/dalab/Desktop/bioxp_re/scripts/bioxp_supervised_home_axis.sh x
BIOXP_BASE_URL="$BIOXP_BASE_URL" /home/dalab/Desktop/bioxp_re/scripts/bioxp_supervised_home_axis.sh y
BIOXP_BASE_URL="$BIOXP_BASE_URL" /home/dalab/Desktop/bioxp_re/scripts/bioxp_supervised_home_axis.sh z
BIOXP_BASE_URL="$BIOXP_BASE_URL" /home/dalab/Desktop/bioxp_re/scripts/bioxp_supervised_home_axis.sh g
BIOXP_BASE_URL="$BIOXP_BASE_URL" /home/dalab/Desktop/bioxp_re/scripts/bioxp_supervised_home_axis.sh door
```

The API now routes `/motion/axis/home` through `motor_oem_home_axis(...)`.

If `speed` is omitted, current OEM defaults are:

- X: `500`
- Y: `500`
- Z: `1791`
- G: `200` for gripper v1, else `600`
- door: door OEM home path with door-specific search-home behavior

### Home X

```bash
curl -sS -X POST "$BIOXP_BASE_URL/motion/axis/home" \
  -H 'Content-Type: application/json' \
  -d '{"axis":"x","timeout_s":20.0}'
```

### Home Y

```bash
curl -sS -X POST "$BIOXP_BASE_URL/motion/axis/home" \
  -H 'Content-Type: application/json' \
  -d '{"axis":"y","timeout_s":20.0}'
```

### Home Z

```bash
curl -sS -X POST "$BIOXP_BASE_URL/motion/axis/home" \
  -H 'Content-Type: application/json' \
  -d '{"axis":"z","timeout_s":20.0}'
```

### Home gripper

```bash
curl -sS -X POST "$BIOXP_BASE_URL/motion/axis/home" \
  -H 'Content-Type: application/json' \
  -d '{"axis":"g","timeout_s":20.0}'
```

### Home thermal door

```bash
curl -sS -X POST "$BIOXP_BASE_URL/motion/axis/home" \
  -H 'Content-Type: application/json' \
  -d '{"axis":"door","timeout_s":20.0}'
```

## Recommended supervised order tonight

For individual manual-home validation:

1. X
2. Y
3. Z
4. G
5. door

The wrapper already captures the before/after snapshots and stores them under `/tmp/bioxp-live-runs`.

If you need the equivalent raw post-home checks:

```bash
curl -sS "$BIOXP_BASE_URL/motion/power/status"
curl -sS "$BIOXP_BASE_URL/latch/status"
curl -sS "$BIOXP_BASE_URL/motion/axis/x/status"
```

Swap the last line for the axis you just homed.

What you want to see:

- no new interlock failure
- speed returns to `0`
- axis status still responds cleanly
- physical end position matches the command you expected

Telemetry is necessary but not sufficient. Physical supervision still matters.

## Full strict startup homing, if you want the OEM startup sequence

Preferred assistant-operated wrapper:

```bash
BIOXP_BASE_URL="$BIOXP_BASE_URL" /home/dalab/Desktop/bioxp_re/scripts/bioxp_supervised_strict_startup.sh --homing
```

Equivalent raw API call:

```bash
curl -sS -X POST "$BIOXP_BASE_URL/motion/arm/strict_startup" \
  -H 'Content-Type: application/json' \
  -d '{"run_homing":true}'
```

Current startup sequence in code is OEM-style:

1. reconnect / activate boards
2. initialize without motion
3. gripper `+10000` pre-move
4. Z home
5. gripper home
6. X home
7. `setHome(X)`
8. restore X speed to `1700`
9. move X to `6000`
10. Y home
11. `setHome(Y)`
12. door home

## Re-reference / calibration metadata

If you want the API metadata to explicitly show an axis as referenced after a manual operation and you are on the robot-local API:

```bash
curl -sS -X POST "$BIOXP_BASE_URL/motion/reference/mark_referenced" \
  -H 'Content-Type: application/json' \
  -d '{"axis":"x","position_steps":0,"source":"manual_live_test","note":"Supervised home confirmed."}'
```

Check it with:

```bash
curl -sS "$BIOXP_BASE_URL/motion/reference/status?axes=x,y,z,g,door"
```

Reminder: this metadata is advisory continuity state. Real truth is still controller `setHome` plus fresh homing.

## Tests already passing locally before live test

```bash
PYTHONPATH=/home/dalab/Desktop/bioxp_re /usr/bin/python3 -m pytest tests/test_motion_service.py tests/test_bioxp_oem_homing.py tests/test_reference_service.py -q
```

Result at prep time:

- `26 passed`

And full local suite previously passed:

- `66 passed`
