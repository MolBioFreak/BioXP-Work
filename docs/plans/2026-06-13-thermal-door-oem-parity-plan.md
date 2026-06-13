# BioXP Thermal Door OEM-Parity Implementation Plan

> **For Hermes:** Implement only after Christian approves this plan. Use phase-by-phase commits and stop after every phase for spec comparison + Christian review. Do not execute live door motion or restart `bioxp-api.service` without explicit approval.

**Goal:** Replace ad-hoc thermal-door movement with OEM-spec homing/open/close semantics derived from the OEM SSD/decompiled source.

**Architecture:** Add source-backed thermal-door settings, first-class OEM door home/open/close operations, explicit API routes, predicate-backed success criteria, and live validation gates. Generic axis moves may remain, but operator door actions must use OEM door semantics.

**Source of truth:**
- `decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPSettings.cs`
- `decompiled_src/BioXPControlLib/ClassControlInterface.cs`
- `decompiled_src_can/ClassCanLib/ClassThermalBoard.cs`

**Current review:** `docs/reviews/2026-06-13-thermal-door-oem-parity-gap-review.md`

---

## OEM spec checklist

Use this checklist after every phase.

- **Settings**
  - serial `<10`: `TCDoorOpen=93000`, `TC_DOOR_VELOCITY=900`
  - serial `>=10`: `TCDoorOpen=16000`, `TC_DOOR_VELOCITY=50`
  - all: `TC_DOOR_ACCELERATION=20`, `TC_DOOR_MAX_CURRENT=31`, `TCDoorStallGuardThreshold=6`
- **Setup**
  - `setSpeedAcc(TC_DOOR_VELOCITY, TC_DOOR_ACCELERATION)`
  - `setMaxCurrent(TC_DOOR_MAX_CURRENT)`
  - `setStallGuardThreshold(TCDoorStallGuardThreshold)`
  - `disableRightSwitch` and `disableLeftSwitch`
- **Home / closed reference**
  - use `doorSearchHome(axis, TC_DOOR_VELOCITY, TCDoorStallGuardThreshold)`
  - if already home/closed, preclear by `moveSteps(axis, 2000)` under threshold+2
  - then `moveLeft(axis, speed)` until stopped
  - `setHome(axis)` only if `queryHome(axis)` confirms closed/home
- **Open**
  - set stallguard to `TCDoorStallGuardThreshold + 2`
  - set current to `TC_DOOR_MAX_CURRENT`
  - move absolute to `TCDoorOpen`
  - success requires `tcDoorOpened`, not coordinate-only movement
- **Close**
  - only close via absolute `0` when `tcDoorOpened` is true, matching OEM guard
  - success requires `tcDoorClosed`
- **Predicates**
  - `tcDoorClosed` = `queryHome(ThermalDoor)`
  - `tcDoorOpened` = `queryRightSensor(ThermalDoor)`
- **Forbidden**
  - do not use diagnostic `10750` as OEM open
  - do not use diagnostic `-7000` as OEM close
  - do not infer full open/closed from coordinate alone

---

## Commit / review discipline

For each phase:

1. Implement only that phase.
2. Run phase tests.
3. Write or update a short phase spec-comparison section in the PR/commit notes or docs.
4. Commit that phase only.
5. Stop for review before the next phase.

Do **not** bundle unrelated existing dirty files. Current repo already has unrelated dirty source/test changes; use explicit `git add <paths>`.

---

## Phase 0: Planning and non-motion review gate

**Objective:** Establish source-backed gap review and implementation plan before code changes.

**Files:**
- `docs/reviews/2026-06-13-thermal-door-oem-parity-gap-review.md`
- `docs/plans/2026-06-13-thermal-door-oem-parity-plan.md`

**Plan-vs-OEM comparison:**
- PASS: plan identifies `TCDoorOpen`, velocity/current/stallguard, setup, home/open/close, predicates.
- PASS: plan rejects `10750/-7000` as OEM truth.
- PASS: plan requires predicate-backed success.
- REMAINING: implementation not yet performed.

**Validation:**

```bash
python3 -m py_compile src/bioxp/usb_driver.py src/bioxp/api.py src/bioxp/oem_config.py
PYTHONPATH=$PWD .venv/bin/python -m pytest \
  tests/test_bioxp_oem_initialize_motors_live_parity.py \
  tests/test_oem_switch_predicate_interpretation.py -q
```

**Commit:**

```bash
git add docs/reviews/2026-06-13-thermal-door-oem-parity-gap-review.md
git commit -m "docs: add thermal door OEM parity gap review"

git add docs/plans/2026-06-13-thermal-door-oem-parity-plan.md
git commit -m "docs: plan thermal door OEM parity phases"
```

**Review gate:** Christian approves or edits the plan before Phase 1.

---

## Phase 1: Source-backed settings model

**Objective:** Encode OEM thermal-door defaults and serial-class branch in `oem_config.py`.

**Files:**
- Modify: `src/bioxp/oem_config.py`
- Create: `tests/test_oem_thermal_door_config.py`

**Implementation tasks:**

1. Add:

```python
OEM_THERMAL_DOOR_DEFAULTS_BY_SERIAL_CLASS = {
    "serial_lt_10": {
        "TCDoorOpen": 93000,
        "TC_DOOR_VELOCITY": 900,
        "TC_DOOR_ACCELERATION": 20,
        "TC_DOOR_MAX_CURRENT": 31,
        "TCDoorStallGuardThreshold": 6,
    },
    "serial_ge_10": {
        "TCDoorOpen": 16000,
        "TC_DOOR_VELOCITY": 50,
        "TC_DOOR_ACCELERATION": 20,
        "TC_DOOR_MAX_CURRENT": 31,
        "TCDoorStallGuardThreshold": 6,
    },
}
```

2. Add helper:

```python
def oem_thermal_door_defaults(serial_number: int | str | None) -> dict[str, int]:
    try:
        serial = int(serial_number) if serial_number is not None else 10
    except (TypeError, ValueError):
        serial = 10
    return dict(
        OEM_THERMAL_DOOR_DEFAULTS_BY_SERIAL_CLASS[
            "serial_lt_10" if serial < 10 else "serial_ge_10"
        ]
    )
```

3. Add `m_TCDoorOpen -> TCDoorOpen` to config diffing.

**Tests:**

- serial `<10` returns `TCDoorOpen=93000`, velocity `900`.
- serial `>=10` and invalid/unknown serial return `TCDoorOpen=16000`, velocity `50`.
- acceleration/current/stallguard match source defaults.
- config diff detects machine override for `m_TCDoorOpen`.

**Phase 1 spec comparison:**

- Compare every constant against `ClassBioXPSettings.cs` lines 266, 308, 310, 312, 314, 779-785.
- Confirm no motion code changed.

**Validation:**

```bash
PYTHONPATH=$PWD .venv/bin/python -m pytest tests/test_oem_thermal_door_config.py -q
python3 -m py_compile src/bioxp/oem_config.py
```

**Commit:**

```bash
git add src/bioxp/oem_config.py tests/test_oem_thermal_door_config.py
git commit -m "feat: model OEM thermal door settings"
```

**Review gate:** Stop. Review Phase 1 diff and spec comparison before Phase 2.

---

## Phase 2: OEM door profile values / remove diagnostic targets from operator path

**Objective:** Make door preparation use OEM values and eliminate diagnostic `10750/-7000` from operator-facing behavior.

**Files:**
- Modify: `src/bioxp/usb_driver.py`
- Create/modify: `tests/test_oem_thermal_door_profile.py`

**Implementation tasks:**

1. Change `MOTOR_FUNCTION_PRESETS["door"]` to source-backed values for serial `>=10` default:

```python
"speed": 50,
"home_speed": 50,
"acc": 20,
"run_current": 31,
"standby_current": 10,
"stall_guard": 6,
"open_position": 16000,
"close_position": 0,
"disable_right": True,
"disable_left": True,
```

2. Replace diagnostic target assignment:

```python
open_pos = int(p.get("open_position", 16000))
close_pos = int(p.get("close_position", 0))
```

3. Add a comment that `run_thermal_door_menu()` is diagnostic, but its targets are now OEM settings.

**Tests:**

- door preset speed/current/acc/open/close match source.
- door masks remain both true.
- test source or behavior to prove `10750` and `-7000` are not production targets.

**Phase 2 spec comparison:**

- Compare profile values against `ClassControlInterface.initializeMotorsWithoutMotion` and `ClassBioXPSettings`.
- Confirm `10750/-7000` cannot be reached via operator/open-close path.
- Confirm no API route yet claims full door open/close.

**Validation:**

```bash
PYTHONPATH=$PWD .venv/bin/python -m pytest tests/test_oem_thermal_door_profile.py -q
python3 -m py_compile src/bioxp/usb_driver.py
```

**Commit:**

```bash
git add src/bioxp/usb_driver.py tests/test_oem_thermal_door_profile.py
git commit -m "fix: align thermal door profile with OEM settings"
```

**Review gate:** Stop. Review Phase 2 diff and spec comparison before Phase 3.

---

## Phase 3: Driver-level OEM door status/home/open/close

**Objective:** Add driver-level operations that match OEM semantics and return predicate-backed telemetry.

**Files:**
- Modify: `src/bioxp/usb_driver.py`
- Create/modify: `tests/test_oem_thermal_door_operations.py`

**Implementation tasks:**

1. Add `motor_thermal_door_status()` returning:
   - position
   - speed
   - raw switch/mask values
   - `closed` = `queryHome(ThermalDoor)`
   - `opened` = `queryRightSensor(ThermalDoor)`
2. Harden `motor_oem_door_search_home()`:
   - startup mode vs manual mode preserved
   - manual preclear if already closed/home
   - set home only when closed/home predicate confirms
   - return `closed_before/opened_before/closed_after/opened_after`
3. Add `motor_oem_open_thermal_door()`:
   - prepare with threshold+2/current 31/OEM speed+acc/both masks
   - move abs to `TCDoorOpen`
   - wait stopped
   - success only if opened predicate confirms
4. Add `motor_oem_close_thermal_door()`:
   - if not opened, skip/report according to closed predicate
   - prepare with threshold+2/current 31
   - move abs to 0
   - success only if closed predicate confirms

**Tests:**

- fake driver verifies exact move target `16000` for serial >=10.
- open success requires `opened=true`.
- close success requires `closed=true`.
- home setHome requires closed/home predicate.
- partial coordinate-only movement is not success.

**Phase 3 spec comparison:**

- Compare each driver operation against OEM source:
  - `ClassThermalBoard.doorSearchHome` lines 364-410
  - `ClassControlInterface.openThermalDoor` lines 2651-2678
  - `ClassControlInterface.closeThermalDoor` lines 2678-2692
  - `confirmAxis` predicates lines 2748-2757
- Confirm no API route exposes these yet unless Phase 4 is approved.

**Validation:**

```bash
PYTHONPATH=$PWD .venv/bin/python -m pytest tests/test_oem_thermal_door_operations.py -q
python3 -m py_compile src/bioxp/usb_driver.py
```

**Commit:**

```bash
git add src/bioxp/usb_driver.py tests/test_oem_thermal_door_operations.py
git commit -m "feat: add OEM thermal door driver operations"
```

**Review gate:** Stop. Review Phase 3 diff and spec comparison before Phase 4.

---

## Phase 4: API routes with operator ack and failure semantics

**Objective:** Expose explicit OEM thermal-door routes; prevent generic axis movement from being used as proof of door open/closed.

**Files:**
- Modify: `src/bioxp/api.py`
- Create/modify: `tests/test_oem_thermal_door_api.py`

**Implementation tasks:**

1. Add request model:

```python
class ThermalDoorActionRequest(BaseModel):
    operator_ack: str
    reason: str = Field(..., min_length=1, max_length=2000)
    timeout_s: float = Field(20.0, gt=0.1, le=60.0)
    capture_bundle: bool = False
```

2. Add routes:
   - `POST /motion/thermal_door/home`, ack `HOME_THERMAL_DOOR`
   - `POST /motion/thermal_door/open`, ack `OPEN_THERMAL_DOOR`
   - `POST /motion/thermal_door/close`, ack `CLOSE_THERMAL_DOOR`
3. Return HTTP 409 with full JSON detail when predicates do not confirm.
4. Add OpenAPI descriptions stating coordinate-only movement is not success.

**Tests:**

- ack strings enforced.
- OpenAPI includes routes.
- failing driver response becomes HTTP 409 with detail.
- success body includes before/after predicates and target.

**Phase 4 spec comparison:**

- Compare route behavior to OEM functions, not generic axis semantics.
- Confirm API cannot label a door open unless `tcDoorOpened` confirms.
- Confirm no runtime reload or live motion was performed during tests.

**Validation:**

```bash
PYTHONPATH=$PWD .venv/bin/python -m pytest tests/test_oem_thermal_door_api.py -q
python3 -m py_compile src/bioxp/api.py
```

**Commit:**

```bash
git add src/bioxp/api.py tests/test_oem_thermal_door_api.py
git commit -m "feat: expose OEM thermal door API routes"
```

**Review gate:** Stop. Review Phase 4 diff and spec comparison before Phase 5.

---

## Phase 5: Integration tests and docs refresh

**Objective:** Prove the full non-live code surface is coherent and update operator docs/skills.

**Files:**
- Modify/create: relevant tests under `tests/`
- Modify: `docs/reviews/2026-06-13-thermal-door-oem-parity-gap-review.md` if implementation changes conclusions
- Modify: BioXP skill only after verified implementation

**Tests:**

```bash
PYTHONPATH=$PWD .venv/bin/python -m pytest \
  tests/test_oem_thermal_door_config.py \
  tests/test_oem_thermal_door_profile.py \
  tests/test_oem_thermal_door_operations.py \
  tests/test_oem_thermal_door_api.py \
  tests/test_bioxp_oem_initialize_motors_live_parity.py \
  tests/test_oem_switch_predicate_interpretation.py -q
```

**Phase 5 spec comparison:**

- Produce a concise checklist result: settings, setup, home, open, close, predicates, forbidden diagnostics.
- Identify any remaining non-live gaps plainly.

**Commit:**

```bash
git add tests/ docs/ src/bioxp/oem_config.py src/bioxp/usb_driver.py src/bioxp/api.py
git commit -m "test: verify OEM thermal door parity surface"
```

Use explicit file paths; do not stage unrelated dirty changes.

**Review gate:** Stop. Christian reviews all non-live work before runtime reload.

---

## Phase 6: Live validation plan after explicit approval

**Objective:** Validate the new OEM door routes on hardware after Christian approves runtime reload and live motion.

**Precondition:** Explicit approval for API reload. Previous restart wedged/crashed the robot PC; do not restart by implication.

**Live sequence:**

1. Passive state only:

```bash
curl -fsS http://127.0.0.1:8123/motion/axes/status?axes=x,y,z,g,door
curl -fsS http://127.0.0.1:8123/motion/power/status
curl -fsS http://127.0.0.1:8123/latch/status
```

2. Runtime reload under operator supervision.
3. Passive state again; all speeds must be 0.
4. Door home:

```json
{"operator_ack":"HOME_THERMAL_DOOR","reason":"OEM parity validation"}
```

Expected: `closed=true`, position set home/0, speed 0.

5. Door open:

```json
{"operator_ack":"OPEN_THERMAL_DOOR","reason":"OEM TCDoorOpen validation"}
```

Expected: target `16000` unless machine config overrides; `opened=true`; operator/camera confirms full-open behavior.

6. Door close:

```json
{"operator_ack":"CLOSE_THERMAL_DOOR","reason":"OEM close validation"}
```

Expected: `closed=true`, speed 0.

**Phase 6 spec comparison:**

- Compare live artifact against OEM checklist.
- If any predicate disagrees with physical/operator observation, stop and write RCA before more motion.

**Commit after live validation:**

Only commit logs/docs/tests if useful; do not commit ephemeral artifacts unless requested.

**Review gate:** Christian decides whether to promote routes to normal workflow.

---

## Non-goals

- Do not alter X/Y/Z/G behavior in this thermal-door PR.
- Do not auto-open/close the door around deck moves until explicit door routes are validated.
- Do not treat coordinate-only door movement as success.
- Do not use `10750` or `-7000` as OEM truth.
