# BioXP OEM X/Y/Z/G Semantics Audit — source-first report before further fixes

Date: 2026-06-14
Repo: `/home/molbiofreak/bioxp_re`
Scope: X/Y/Z/G initialization/homing semantics only. No live motion was commanded for this report.

## Executive conclusion

Christian is correct: the current live/pushed full-init implementation is **not OEM-spec for X/Y**.

The current `test` tip / live code includes commit:

```text
7dcd859 fix: use live XY reference recovery in full init
```

That commit made the monolithic full init pass by replacing the OEM X/Y physical switch-home step with:

```text
move axis to controller coordinate 0 -> call setHome -> continue
```

That is **not** equivalent to the OEM SSD behavior. If the head physically remains mid-deck, then the route's `ready=true` result is a false-ready controller-coordinate artifact, not OEM initialization success.

The next code change should not be another controller workaround. The next code change should be:

1. Remove/revert the X/Y controller-zero substitution from monolithic full init.
2. Make monolithic full init fail closed at X/Y until live X/Y OEM switch-search semantics are repaired.
3. Repair X/Y by matching the OEM SSD `axisSearchHome/goHome/queryHome` contract, especially predicate polarity and switch transition behavior.
4. Keep the live-proven Z and G adaptations only where they are explicitly documented as live hardware reconciliations of OEM intent, not byte-identical OEM behavior.

## OEM SSD source truth

### Full `initializeMotors()` order

Source anchors in `docs/references/2026-06-13-oem-initialization-source-anchors.md` and `docs/references/2026-06-13-oem-gripper-source-anchors.md` show:

```text
ClassControlInterface.initializeMotors(), lines 3348-3391
```

Order:

```text
1. MotorZ.axisSearchHome(axis, 1791)
2. setGripperCurrent(31)
3. MotorGrip.moveSteps(axis, +10000, true)
4. MotorGrip.axisSearchHome(axis, 600 or 200 depending GripperVersion)
5. MotorX.axisSearchHome(axis, 250)
6. MotorX.setHome(axis)
7. MotorX.setSpeed(axis, 1700)
8. moveX(6000)
9. MotorY.axisSearchHome(axis, 250)
10. ThermalDoor.doorSearchHome(...)
11. MotorY.setHome(axis)
```

Important: X/Y are not controller-zero recovery. OEM calls `axisSearchHome(250)`.

### OEM lower-level `goHome` / `axisSearchHome` contract

Source anchors in `docs/references/2026-06-13-oem-initialization-source-anchors.md` show board-level `goHome`:

```text
ClassHeadBoard.goHome lines 60-80+
ClassThermalBoard.goHome lines 118-130+
```

Observed source semantics:

```text
if 24V dropped -> exception
if board not initialized -> return 1
if MotorHome && CurrentPosition == 0 -> return 0
store search speed
if rehome -> moveToAbs(axis, 10000)
moveLeft(axis, speed)
poll queryHome(axis)
when queryHome true -> stop, result = -queryActualPosition(), setHome(axis)
```

The OEM behavior is switch/predicate driven. `setHome(axis)` is earned by `queryHome(axis)`, not by arbitrary controller coordinate zero.

### OEM setup/masks

From `initializeMotorsWithoutMotion()` anchors:

- X:
  - board 5 motor 0
  - speed/acc 1700/350
  - max current 31
  - stall guard 16
  - **no SAP12/SAP13 switch disable write shown**
- Y:
  - board 4 motor 0
  - speed/acc 1800/400
  - max current 31
  - stall guard 16
  - **disableRightSwitch** shown
- Z:
  - board 4 motor 1
  - speed/acc 1791/576
  - max current from machine setting
  - stall guard from machine setting
  - **no SAP12/SAP13 switch disable write shown**
- G:
  - board 4 motor 2
  - GripperVersion-dependent speed/home speed
  - action current 31, idle/hold 10
  - no generic X/Y/Z-style switch mask in the inspected init setup

## Current Linux implementation truth

### Correct/near-correct parts

Current `MOTOR_AXIS_PRESETS` is mostly aligned for board/motor mapping and base setup:

```text
X = board 5 motor 0
Y = board 4 motor 0
Z = board 4 motor 1
G = board 4 motor 2
```

Base switch mask model is also mostly aligned:

```text
X: no disable_right/disable_left key -> no write
Y: disable_right=True
Z: no disable_right/disable_left key -> no write
G: no disable mask in base preset
```

G idle current is now correctly represented as 10/10, with action current raised in gripper actions.

### Current full-init X/Y discrepancy

`src/bioxp/usb_driver.py::motor_oem_initialize_motors_full_sequence()` currently does this for X/Y after commit `7dcd859`:

```text
x_axisSearchHome_250_live_reference_0
  move X absolute to controller 0
  verify controller position near 0
  x_setHome
  x_setSpeed_1700
  x_moveX_6000

y_axisSearchHome_250_live_reference_0
  move Y absolute to controller 0
  verify controller position near 0
  y_setHome
```

That is not OEM `axisSearchHome(250)`.

It is labeled as an adaptation:

```json
{
  "adaptation": "live_controller_reference_zero_recovery_no_switch_search",
  "source_step_adapted_from": "MotorX.axisSearchHome(250)"
}
```

This is the exact false-ready mechanism Christian observed.

### Predicate-model discrepancy inside the codebase

There are two competing predicate descriptions:

1. `src/bioxp/oem_switch_audit.py` says:

```text
ClassMotor.queryLeftSwitchStatus/queryRightSwitchStatus read GAP9/GAP10.
Raw reply value 1 -> C# helper returns 0.
Board-level queryHome/queryRightSensor treats return code 0 as True.
Linux raw-active value therefore maps raw GAP9=1 to OEM home true.
```

2. `src/bioxp/oem_parity_predicates.py` says:

```text
x/y/z/g home_query=queryHome, home_active_value=0
```

Both can be true at different abstraction layers only if clearly separated:

```text
raw GAP9 value 1  -> ClassMotor queryLeftSwitchStatus returns 0 -> board queryHome boolean true
```

But the current runtime often mixes raw GAP values, helper return values, and boolean predicates. This is a prime suspect for why Linux switch-search does not behave like OEM even while source labels look correct.

## Live evidence from this session

### First correct-ACK full init attempt

Artifact:

```text
/tmp/bioxp-live-runs/20260614T015022Z_full_init_after_gripper_home.json
```

Result:

```text
ok=false
failed_at=initialize_motors_full_sequence / x_axisSearchHome_250
```

Sequence before failure:

```text
z_axisSearchHome_1791_probe
z_return_to_live_reference_0
z_reference_verify_after_return
z_clearance_for_xy
g_confirmAxis_before_clear
g_moveSteps_plus10000_then_axisSearchHome
x_axisSearchHome_250
```

X failure evidence:

```text
X position_before = -94991
X position_after  = -189798
home_before       = 0
home_after        = 0
switch_transition = false
false_home_guard  = max_search_abs_delta_exceeded_before_home_switch_transition
```

Interpretation:

- Linux's current source-shaped X `moveLeft/GAP9` search moved farther negative.
- It did not see the OEM home predicate transition.
- The route correctly failed closed at that point.
- This failure should have triggered source/predicate investigation, not controller-zero substitution.

### False-pass full init attempt after `7dcd859`

Artifact:

```text
/tmp/bioxp-live-runs/20260614T054637Z_full_init_after_xy_reference_fix.json
```

Result:

```text
ok=true
ready=true
```

But the X/Y steps were:

```text
x_axisSearchHome_250_live_reference_0
y_axisSearchHome_250_live_reference_0
```

Those steps used controller-zero recovery, not OEM switch home. Christian observed the head physically remained mid-deck. Therefore this artifact is **not** physical/OEM init proof.

### Z evidence

In the successful/false-pass artifact, Z did run first:

```text
z_axisSearchHome_1791_probe
z_return_to_live_reference_0
z_reference_verify_after_return
z_clearance_for_xy
```

The live Z adaptation is not byte-identical OEM `axisSearchHome(1791)`, but it is grounded in repeated live evidence that generic Z `MoveLeft/GAP9` search drove negative away from the live reference while GAP10/right was the observed reference condition at controller zero.

Current report position: keep Z adaptation only as an explicit, documented live-hardware reconciliation:

```text
OEM intent: establish Z reference first, then clear Z before XY.
Live implementation: return to controller 0 with Z-specific right/GAP10 mask recovery, verify, then move to -15000 clearance.
```

Do not call that byte-identical OEM. Do call it the currently verified live-safe Z behavior.

### G evidence

Artifacts:

```text
/tmp/bioxp-live-runs/20260614T013057Z_gripper_home_requested.json
/tmp/bioxp-live-runs/20260614T011926Z_gripper_down_oem_clear_fixed.json
```

Verified:

- `/motion/gripper/home` reached controller `G=0`, `queryHome(MotorGrip)=true`, physical operator-confirmed home.
- G idle current restored to `10/10`.
- `/motion/gripper/clear` required temporary G SAP12/SAP13 mask to leave the both-switch-active home state and produced real motion.

Current report position:

```text
G home acceptance via queryHome(MotorGrip) is aligned with OEM confirmAxis(g): queryHome OR getG()<50.
G clear movement with temporary masks is a live implementation workaround for Linux/TMCL behavior after home; OEM source does not show generic G switch-disable writes in initializeMotorsWithoutMotion.
```

## Axis-by-axis discrepancy table

### X

OEM SSD:

```text
initializeMotors: MotorX.axisSearchHome(250), then setHome, setSpeed(1700), moveX(6000)
queryHome comes from board/motor switch semantics, not controller coordinate zero
```

Current Linux:

```text
After 7dcd859: move absolute to controller 0, setHome, moveX(6000)
```

Discrepancy:

```text
Major. Controller-zero substitution is not OEM homing and can pass while physically mid-deck.
```

Required fix:

```text
Remove false-pass substitution. Repair X queryHome/GAP9 predicate and direction against SSD source + live observation, with required physical/operator proof before ready=true.
```

### Y

OEM SSD:

```text
initializeMotors: MotorY.axisSearchHome(250), later setHome(Y)
initializeMotorsWithoutMotion disables Y right switch only
```

Current Linux:

```text
After 7dcd859: move absolute to controller 0, setHome
Y preset keeps disable_right=True
```

Discrepancy:

```text
Major for homing method. Mask setup is likely aligned, but homing proof is not.
```

Required fix:

```text
Same as X: repair actual switch-search predicate/direction; do not treat controller 0 as physical Y home.
```

### Z

OEM SSD:

```text
MotorZ.axisSearchHome(1791)
```

Current Linux:

```text
Live Z reference recovery: controller 0 + GAP10/right raw active, using Z-specific right-mask move-to-reference, then Z clearance to -15000.
```

Discrepancy:

```text
Known intentional adaptation. Not byte-identical OEM, but source-shaped MoveLeft/GAP9 has repeatedly been live-dangerous/wrong on this robot.
```

Required fix/reporting:

```text
Keep as explicit live adaptation unless/until SSD predicate replay proves a better equivalent. Do not market as exact OEM axisSearchHome bytes.
```

### G / gripper

OEM SSD:

```text
setGripperCurrent(31)
moveSteps(MotorGrip,+10000,true)
axisSearchHome(MotorGrip, 600 or 200)
confirmAxis(g): queryHome(MotorGrip) OR getG()<50
restore/idle current 10 for GripperVersion 1 behavior
```

Current Linux:

```text
First-class /motion/gripper/clear and /motion/gripper/home
queryHome accepted for home even with both raw switch lines active
idle current restored to 10/10
clear uses temporary G SAP12/SAP13 masks to get off home and verify position delta
```

Discrepancy:

```text
Home acceptance is aligned with OEM confirmAxis(g). Clear mask is a live workaround; OEM source does not show generic G disable writes, but without it Linux ACKed and did not move.
```

Required fix/reporting:

```text
Keep first-class gripper routes. Continue reporting raw switch diagnostics separately from OEM home truth. Do not use generic G relative/absolute routes for gripper movement.
```

## Immediate remediation proposal, before live motion

### Phase A — undo false-ready behavior

Code should be changed so full init does **not** report ready on X/Y controller-zero recovery.

Expected behavior until X/Y are repaired:

```text
full init: Z first + G proof may pass, then fail closed at X/Y with:
xy_oem_switch_home_unverified_fail_closed
```

This prevents another false `ready=true` while the head is physically mid-deck.

### Phase B — repair X/Y by OEM source contract

Need to implement/fix the actual equivalent of:

```text
axisSearchHome(250)
  set/source search speed
  moveLeft(axis, speed)
  poll queryHome(axis)
  stop on queryHome true
  setHome only after valid transition/home predicate
```

But before live motion, we need a no-motion predicate matrix from current hardware state:

```text
for X/Y:
- raw GAP9
- raw GAP10
- SAP12/SAP13 mask states
- board-level queryHome-equivalent interpretation
- controller position/speed
- whether current physical operator truth says mid-deck/home/not-home
```

Then a bounded supervised diagnostic, not monolithic init:

```text
X only: small, reversible, watched command in OEM moveLeft direction while polling GAP9/GAP10/position/speed
Y only: same
stop immediately on wrong direction, no transition, or operator concern
```

Goal is not to invent a new route. Goal is to find why Linux's current `moveLeft/GAP9` interpretation differs from OEM working behavior.

### Phase C — only then rerun monolithic init

Full init can report `ready=true` only when:

```text
Z reference/clear verified
G home verified by queryHome/operator-compatible status
X/Y true home semantics verified by switch predicate/transition or exact OEM-equivalent proof
Door home verified
all speeds 0
```

## What not to do

- Do not claim `controller 0` means physical home for X/Y.
- Do not call `setHome` at an operator-observed mid-deck pose.
- Do not report `ready=true` from a monolithic route if any OEM physical home step is substituted with a non-OEM controller coordinate move.
- Do not use generic G-axis routes for gripper movement.
- Do not re-run monolithic init again until the X/Y semantics are corrected or the route is fail-closed.

## Trust status of latest commits

```text
70347b2 fix: move gripper clear with OEM limit mask semantics
394307b fix: accept OEM gripper query-home after homing
64bede1 fix: remove stale full init homing kwarg
4e3fb21 fix: accept live Z reference predicate after recovery
6c9be90 fix: route full init Z through live reference recovery
```

These are broadly defensible with caveats above.

```text
7dcd859 fix: use live XY reference recovery in full init
```

This is **not** defensible as OEM full init. It should be reverted or superseded by a fail-closed X/Y patch before further work.


## Implementation follow-up: 2026-06-14 Phase 2/3 fixes

After the fail-closed guard commit, the full-init X/Y block was restored to the SSD method names/order:

```text
MotorX.axisSearchHome(250)
MotorX.setHome()
MotorX.setSpeed(1700)
moveX(6000)
MotorY.axisSearchHome(250)
ThermalDoor.doorSearchHome(...)
MotorY.setHome()
```

Critical comparison against the bad `7dcd859` behavior:

- Bad: `move_absolute(X,0)` / `move_absolute(Y,0)` as a substitute for home.
- Fixed: no controller-zero substitution; X/Y must pass `motor_oem_axis_search_home(... speed=250 ...)`.
- Bad: controller coordinate success could report full init ready while head was physically mid-deck.
- Fixed: full init success depends on the guarded switch-search primitive, which only returns `ok=true` after queryHome/GAP9 proof and setHome acceptance.

The Linux `max_search_abs_delta` guard is disabled only inside this OEM full-init X/Y call (`None`) because OEM SSD `axisSearchHome/goHome` does not stop based on stale/desynced controller coordinates. QueryHome transition is the proof, not controller distance.

Predicate-layer clarification was added in `src/bioxp/oem_parity_predicates.py`: `home_active_value=0` is the OEM helper return-code layer; raw Linux GAP reads still use raw active value `1` via `usb_driver.MOTOR_SWITCH_ACTIVE_VALUE`.


## Live verification follow-up: 2026-06-14 Phase 4

Commits applied before live test:

```text
00e5021 fix: fail closed on unverified XY OEM homing
aa779f9 fix: restore OEM XY axis search in full init
019ef99 fix: normalize X switch masks for OEM homing
```

Live full-init artifact after restoring SSD X/Y sequence and normalizing X SAP12/SAP13:

```text
/tmp/bioxp-live-runs/20260614T062011Z_full_init_oem_xy_xmask_fixed.json
```

Result:

```text
Z reference/recovery: ok
Z clearance: ok, Z=-15000
G confirm/home: ok, G=0, current 10/10
X axisSearchHome(250): failed closed
```

X failure details:

```text
source step: MotorX.axisSearchHome(250)
Linux command: MoveLeft/cmd=2, speed=250
X masks before search: SAP12=0, SAP13=0
X before search: GAP9/left=0, GAP10/right=1
X after search: GAP9/left=0, GAP10/right=1
X final position after failed search: -281869
failure: home_switch_timeout_without_transition
```

A bounded reverse-direction diagnostic was also run before the mask-normalized full init:

```text
/tmp/bioxp-live-runs/20260614T061533Z_x_positive_gap9_search.json
```

It moved X in +50,000-step chunks from `-338017` to `+61987`; GAP9/left remained `0` throughout. Therefore GAP9 did not assert in either direction across the tested controller span.

Interpretation:

- The false-ready software discrepancy has been fixed: full init now fails closed instead of claiming ready.
- The SSD command bytes and query polarity are implemented: `MoveLeft` is cmd=2; `queryHome` uses GAP9 raw `1` via helper return-code `0`.
- X SAP12/SAP13 are now normalized to enabled for source parity before X homing.
- The remaining blocker is live X switch truth: GAP9 does not assert while X is being driven by source-equivalent commands.

Do not run more blind X motion until operator/camera truth confirms where the head is relative to the physical X home switch and whether the switch/harness is expected to be engaged. The next diagnostic should correlate live visual position with GAP9/GAP10 while the head is physically placed/observed near the OEM X home side.
