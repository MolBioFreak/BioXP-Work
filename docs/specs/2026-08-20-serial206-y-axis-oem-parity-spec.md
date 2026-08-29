# BioXP Serial-206 Y-Axis OEM Parity Execution Specification

> **For Hermes:** Convert each approved work package in this specification into a separate PR-sized implementation plan before changing source. Christian must authorize source work, code tests, deployment, service restart, controller writes, and physical motion as separate gates. A command to implement software does not authorize motion.

**Contract status:** Execution-ready specification. Implementation has not started.

**Goal:** Add a robot-owned Y provider that preserves the recovered Serial-206 OEM Y contract, keeps Y and Z independent on shared board 4, records controller discrepancies without a drift-only movement latch, admits commands quickly, and gives BioModStack a thin typed relay.

**Claim boundary:** This specification can close `Y_PROVIDER_SOFTWARE_PARITY`, `Y_CONTROLLER_PARITY`, and `Y_PHYSICAL_PARITY` in separate stages. It does not claim full `ControlLib`, full startup, gripper, Z, or complete robot OEM parity.

---

## 1. Review disposition and frozen identities

### 1.1 Rejected candidate

The earlier candidate is rejected as a final specification.

```text
rejected_sha256=fee4ced908f44d1317ecd77678b5c62e1b3f133c4718774d244b7bdaf823bc66
rejected_lines=916
rejected_bytes=37169
```

Verified recovery artifacts:

```text
/home/dalab/.hermes/plan-snapshots/
  2026-08-20-serial206-y-axis-oem-parity-spec.md.20260820T185020Z.fee4ced908f4.backup
  2026-08-20-serial206-y-axis-oem-parity-spec.md.20260820T185020Z.fee4ced908f4.json
```

The rejected candidate mixed OEM facts with Linux replacement architecture. It also misstated public returns, timeout behavior, range enforcement, aggregate abort, current modes, and operator exposure.

### 1.2 Current implementation targets

| Target | Frozen identity |
|---|---|
| Robot worktree | `/home/dalab/worktrees/bioxp-wpb-queue-20260819` |
| Robot commit | `3d317b25037caa96faaea627563d6b68ac2ab302` |
| Robot tree | `97a02766cf02ce9ffd55bc55378327bc0350b277` |
| Robot branch | `work/bioxp-wpb-queue-20260819` |
| BMS Development checkout | `/home/dalab/biomodstack/dev-test-canonical` |
| BMS commit | `efd24d9c8378d8a67ce096dd79ed5603bb5d583f` |
| BMS tree | `eac30f36fa98abd2080cf1993876242b5012ff49` |
| BMS branch | `test` |

The robot tree is clean except for this untracked specification. The BMS Development tree is clean at the identity above.

No live service, deployed SHA, controller state, or physical position was audited for this specification. Any earlier serving-release observation is historical context only.

### 1.3 OEM authority set

| Evidence | SHA-256 |
|---|---|
| `decompiled_src/BioXPControlLib/ClassControlInterface.cs` | `86093e5270c82ea2e45cb4de449076372ca79d9485ba6de9565d5eb255811e6e` |
| `decompiled_src_can/ClassCanLib/ClassHeadBoard.cs` | `342a9b2f09731002194b67e37f1d4e866ecbfb3c25effd85b3cd609e8cbdd1ea` |
| `decompiled_src_can/ClassCanLib/ClassMotor.cs` | `9fb1b4bec771165053a82b4fe95510615d6ed9beda1a041280584ceb4ab7fe99` |
| `decompiled_src/BioXPControlLib/ControlLib.cs` | `f69b3529dcb9723c705ac55ecb3f035010cc294d3891de096c165bb20116f6c2` |
| `decompiled_src/BioXPControlLib/WindowBoardTest.cs` | `aa4b8f8c2dcfdf231889084dde3de28f19a068f0ec8e9f1233651ed0c2a013c2` |
| `decompiled_src_genbotapp/GenBotApp/BioXPMainWindow.cs` | `b288a45e2de54cd2c8d30a4498a343cd6f423aff7e88a78847076bfbfb4e904c` |
| `decompiled_src_can/ClassCanLib/ClassNovoCommandQueue.cs` | `70bed5af6c244d63f6506e1ed9003d5c3e9f07dae5aac1262520ec56506f35dd` |
| `decompiled_src_can/ClassCanLib/ClassBaseBoard.cs` | `2622aee1810b9a8a52bed54a49196d50e50751465608c8420dd1e5077d95dd5e` |
| `decompiled_src_can/ClassCanLib/StatusCode.cs` | `76313556be8027f5e447561781880427cee7229ad0aaebf1d08c46f5ee241ef6` |
| Selected Serial-206 `appdata/config.xml` | `33aadf87f631cf33f2e0b4c86948c92be3b21412ca5477ea8fa8bc7848cbf475` |

Selected configuration path:

```text
/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/reports/
  bioxp_historical_run_forensics_20260717/derived/
  oem_runtime_parity_spec_20260719/oem_machine_runtime_bundle_serial206/
  appdata/config.xml
```

### 1.4 Evidence classes

Every implementation requirement uses one class:

| Class | Meaning |
|---|---|
| `OEM_LITERAL` | Directly required by recovered source and selected configuration. |
| `OEM_CALLER_POLICY` | A recovered caller applies policy around an OEM primitive. |
| `LINUX_REPLACEMENT` | Linux architecture that preserves the OEM effect or order. It is not recovered OEM code. |
| `LINUX_EVIDENCE_HARDENING` | Extra validity or evidence checks that do not change the source-visible outcome. |
| `OPERATOR_REQUIRED_REPLACEMENT` | Christian's binding rule resolves a source defect or an unsafe cross-axis effect. |
| `EVIDENCE_DEFICIT` | The available source cannot prove the claimed behavior. |

A replacement or hardening rule must never be reported as literal OEM parity.

---

## 2. Scope

### 2.1 Included

- Y board/motor identity and selected-machine limits.
- Y no-motion profile preparation.
- Relative and absolute Y movement.
- Blocking and nonblocking source modes.
- Manual-panel, diagnostic, startup, composite, and workflow-owned Y home modes.
- Y STOP.
- Y status, controller position, switch state, and profile evidence.
- `moveXY` and `HomeXY` Y ownership.
- Board-4 Y/Z/gripper independence.
- Shared board transition authority.
- Durable normal command admission and source-ordered methods.
- Robot-side command, receipt, discrepancy, and authority persistence in the existing runtime root and SQLite database.
- Typed robot API, action catalog, dashboard, BMS relay, and cockpit closure.
- Software, controller, and physical acceptance gates.

### 2.2 Excluded

- Reimplementation of every recovered `ControlLib` workflow that has no current Linux counterpart.
- Full gripper provider parity.
- New Z movement semantics beyond the shared board and queue changes required for independence.
- New arbitrary raw-frame, current, speed, acceleration, stall, switch-mask, or set-home buttons.
- Automatic hardware retry, automatic home, or automatic redispatch of an ambiguous physical command.
- Deployment, restart, controller mutation, homing, movement, or physical commissioning during specification work.

### 2.3 Claim limit for recovered callers

The Y provider must supply exact leaves to current Linux callers and new typed Y routes. A recovered OEM workflow that is absent from Linux remains a separate workflow gap. Its presence in the caller inventory does not make that workflow part of this Y delivery.

The preliminary mechanical caller scan found 136 material call sites across 28 enclosing methods. This count is assessment evidence, not an acceptance denominator. WP0 must freeze the exact rows before any source implementation begins. The direct caller groups are:

| Source | Y-bearing responsibility |
|---|---|
| `ClassControlInterface` | no-motion setup, startup, manual controls, relative/absolute movement, `moveXY`, `HomeAxis`, `HomeXY`, location movement, current modes, board tests, limits, and STOP/abort wrappers |
| `ControlLib` | park, rehome, self-test, and caller-local lost-step handling |
| `WindowBoardTest` | `my`, `mxy`, and test-cycle diagnostic modes |
| `BioXPMainWindow` | application worker admission and major-operation sequencing |
| `ClassHeadBoard` / `ClassMotor` | board/motor commands, waits, source fallbacks, limits, cached state, and event handling |

WP0 must materialize the exact call-site list with file, line, enclosing method, source mode, and target owner. This specification does not silently omit an unmatched current Linux caller.

---

## 3. Selected-machine Y contract

### 3.1 Identity

```text
logical_axis=Y
physical_board=4
motor=0
selected_min_steps=0
selected_max_steps=102956
normal_speed=1800
normal_acceleration=400
max_current=31
stall_guard=16
home_direction=left
home_switch=GAP9
raw_active_value=1
right_switch=disabled during no-motion preparation
normal_target_event=128
stall_event=130
```

Board 4 also owns:

```text
Z=motor 1
Gripper=motor 2
```

Board sharing does not merge motor authority.

### 3.2 No-motion profile

`ClassControlInterface.initializeMotorsWithoutMotion`, lines 3181-3265, is `OEM_LITERAL`; the Y block is at lines 3196-3205. Lines 3278-3292 belong to `initializeforBoardTest` and are not the no-motion Y authority:

1. write Y speed `1800` and acceleration `400`;
2. sleep 2 ms;
3. write max or run current `31`;
4. sleep 2 ms;
5. write stall guard `16`;
6. sleep 2 ms;
7. disable the right switch.

The source does not write a Y standby-current parameter in this stage. Linux must not invent one and call it OEM parity.

Readback verification is `LINUX_EVIDENCE_HARDENING`. It records whether the written controller values match. A write ACK and a readback value remain separate fields.

### 3.3 Setter semantics

| Source method | Exact Y rule |
|---|---|
| `setMaxSpeed("y", speed)` | `0` becomes `1800`; other values pass through; absent board is a no-op; wrapper returns `void`. |
| `setMaxAcc("y", acc)` | `0` becomes `400`; other values pass through; absent board is a no-op; wrapper returns `void`. |
| `restoreOriginalSpeed("y")` | Writes speed `1800`; it does not restore acceleration. |
| `setStallGuard("y", value)` | `0` becomes `16`; other values pass through; low-level encoding casts to one byte. |
| `setMaxCurrent` | Low-level value clamps to `0..31`. |
| `disableRightSwitch` | Uses the motor-specific right-switch disable write. |

These are internal provider stages. Their source presence is not evidence for normal operator buttons.

### 3.4 Current modes

There is no standalone recovered `enableY()` method.

| Recovered aggregate | Y behavior |
|---|---|
| `enableXY(true)` | X/Y current `10`, wait 500 ms, then X/Y current `31`. |
| `enableXY(false)` | Y current `1` with X. |
| `enableXYZ(true)` | X/Y current `10`, wait 500 ms, X/Y current `31`, wait 500 ms, then Z configured-up current. |
| `enableXYZ(false)` | Y current `1` with X and Z. |
| `enableYZ(true)` | Y and Z current `31` directly. No Y `10 -> 31` ramp appears in this method. |
| `enableYZ(false)` | Y and Z current `1`. |

These recovered operations are sequential `void` best-effort write sequences. They ignore subordinate returns and have no rollback. Preserve each write, wait, raw child result, and absent-board branch.

Linux admits each aggregate as one parent method so unrelated operator commands cannot splice into its sequence. Parent admission atomicity is `LINUX_REPLACEMENT`; it does not make the controller writes OEM-atomic. The operations are not routine BMS actions.

`enableYZ` checks only that the Z board exists before dereferencing the Y board. Preserve that raw branch in fixtures. Production preflight rejects either mandatory-board absence as a classified replacement.

---

## 4. Controller reply and telemetry validity

### 4.1 Value envelope

Every controller-derived value is represented as:

```text
value
reply_valid
status_code
status_accepted
sampled_at
board
motor
command_id or observation_id
```

Source fallbacks remain visible:

- `queryActualPosition()` can return the cached motor position after a null reply.
- `queryHome()` returns true while the head board is uninitialized.
- `queryLeftSwitchStatus()` returns wrapper code `0` after a null reply, and high-level `queryHome()` interprets that code as active.
- `queryMotorSpeed()` returns cached speed after a null reply. `checkMotorStopped()` can therefore accept cached zero.
- `checkMotorStopped()` can return true after 24-V loss or while uninitialized.
- `queryMotorStop()` returns wrapper code `0` on a null reply and returns before resetting the wait handle.
- `setHome()` sets cached position to zero before transmission. A null reply returns raw `0` before setting the host home/clean flags. A non-null negative reply returns raw `1` but still sets those host flags.
- `StopMotor()` maps a null second reply and a status-100 second reply to the same raw return code.

A cached or short-circuit value is valid OEM source evidence. It cannot satisfy Linux reference publication, terminal-position authority, profile proof, or reconciliation.

### 4.2 Normal movement event

`ClassHeadBoard.handleReturnMessageNovo` routes event `128` to the motor selected by the message motor index. Event identity must include board, motor, event-window ID, and command ID.

Event `130` identifies a motor stall and raises the recovered board-error callback. The OEM application callback invokes aggregate `forceAbortMotion(false)`. `ControlLib.forceAbortMotion` also changes application script/event and thermal-state surfaces before conditionally reaching controller-interface abort. The controller-interface abort sets global `No24V` and asks every board to release its motor waiters. It sends no addressed MST.

For this Y slice, preserve the recovered controller-interface aggregate software abort and parent/child receipts when event 130 reaches the application error path. Broader thermal/script effects remain owned by their existing application orchestrator. Aggregate software abort can interrupt concurrent work, but it does not erase a sibling coordinate or physically prove a sibling motor stopped.

`queryMotorStop()` resets the recovered auto-reset wait handle only after a non-null reply path. `forceAbortMotion()` also sets the same motor wait handles. Linux must use a fresh command-scoped event generation and a decoded event record. A waiter wake is not proof that event 128 arrived.

Controller codes `13` and `14` are safety-relevant error evidence in the Linux stack. The inspected head-board handler does not prove literal Y handling for these codes. Linux treatment of them is `LINUX_EVIDENCE_HARDENING`, with board or motor scope determined from verified decoded addressing.

### 4.3 Completion classes

A blocking absolute move may return normally through either recovered source class:

1. `event_128`: the motor-specific wait event arrives.
2. `oem_timeout_target_equal`: the source wait expires and a valid actual-position query equals the effective controller target.

The second class is a literal source branch. It must be reported distinctly. It does not prove physical displacement. Set `source_returned_normally=true` and `controller_completion_verified=false` for that class. A cached/null position can reproduce the source branch but cannot establish Linux controller-coordinate truth.

If the timeout readback differs from the effective target, the recovered absolute source throws. It does not issue STOP in that branch. A Linux post-timeout STOP is permitted as `LINUX_EVIDENCE_HARDENING`; record it separately from the source outcome.

A blocking relative timeout logs and returns primitive `-1`. It has no target-equality fallback. The high-level relative wrapper still performs its subsequent position query and returns that position.

GAP1 position, GAP3 speed, and GAP8 helper returns remain controller telemetry. They do not establish independent physical movement.

---

## 5. Relative Y movement

### 5.1 High-level contract

`ClassControlInterface.moveSteps("y", steps)` calls the board relative primitive with its default blocking behavior. The high-level method then returns a fresh Y position.

Preserve the source prebranches: 24-V-dropped throws, an uninitialized board primitive returns raw `1`, and an absent high-level Y board returns public `0` without a command. Provider admission may fail before transport, but primitive fixtures must constrain these source outcomes.

The receipt must retain all three source layers:

```text
requested_steps
position_before
requested_target
motor_command_delivery_count
motor_command_reply_valid
motor_command_status_code
motor_command_raw_return
board_wrapper_return
public_wrapper_return
position_after
completion_class
```

The motor command return is consumed inside the board method. The board wrapper independently returns `1`, `-1`, or a queried position. `ClassControlInterface.moveSteps` discards that board return and performs another position query for its public return. Linux must not collapse these values.

### 5.2 Relative bounds

`ClassMotor.beyondLimit` defines the recovered relative guard:

```text
low: target < 20 is rejected
high: target > 102936 is rejected
```

The relative primitive rejects both 20-step end bands. It does not clamp the requested relative target.

### 5.3 Replacement result

The typed provider returns a structured receipt. It preserves the source raw and public results, then adds:

```text
machine_outcome
controller_completion_verified
terminal_speed_zero
coordinate_truth_available
physical_effect_verified=false
```

A valid final discrepancy does not create a Y drift latch.

`board_wrapper_return=-1` produces a failed board outcome even though the high-level wrapper returns a position. Preserve all layers and keep later movement available when controller truth remains coherent.

---

## 6. Absolute Y movement

### 6.1 High-level wrappers

Recovered wrappers return `void`.

```text
moveY(y, waitforstop=true, appAdjustment=true)
moveY(y, acc, waitforstop=true, appAdjustment=true)
```

`appAdjustment` is unused in the inspected bodies. The wrapper default is blocking. The manual-panel absolute handler and the `my` board-test command explicitly pass `waitforstop=false`.

Preserve the source prebranches: 24-V-dropped throws, an uninitialized board primitive returns raw `1`, and an absent high-level Y board is a `void` no-op. A typed provider may reject before transport, but its receipt must name the equivalent source branch.

The acceleration overload writes the requested acceleration, calls the absolute move, then restores default acceleration on the normal source path. The recovered code is not exception-safe. Linux may use `finally` to restore the profile, but that is `LINUX_EVIDENCE_HARDENING` and must be logged as a replacement guarantee.

The low-level absolute motor command retries once after a null first reply. A null second reply maps to raw `0`. The relative motor command has one delivery and also maps a null reply to raw `0`. Preserve delivery count, reply validity, and raw return separately.

### 6.2 Absolute range behavior

The configured motor range is `0..102956`. It is not a high-level caller clamp.

- `ClassControlInterface.moveY` performs no range check.
- `ClassHeadBoard.moveToAbs` clamps a negative request to zero.
- `ClassMotor.moveToAbs` clamps to its configured low and high limits.
- The board near-high branch returns the current position without a move when `abs(high_limit - current) < 10` and the request is farther positive.

Receipts must preserve:

```text
caller_requested_target
board_effective_target
motor_effective_target
near_high_noop
motor_command_delivery_count
motor_command_reply_valid
motor_command_status_code
motor_command_raw_return
board_wrapper_return
public_wrapper_return_kind=void
```

The dead `fallback=95247` argument in the current Python Y profile loader is not active authority. Remove it or label it non-authoritative. The immutable selected-machine snapshot remains fail-closed at `102956`.

### 6.3 Explicit nonblocking mode

An explicit `waitforstop=false` source call produces an issued command, not an immediate terminal success.

The Linux dispatcher owns the later event/readback correlation. Initial state:

```text
accepted -> queued -> dispatched -> issued_pending
```

A later status transition records completion or failure. The HTTP request must not invent event 128, terminal speed, or final position.

---

## 7. Homing and coordinate authority

### 7.1 Source modes

| Mode | Recovered call |
|---|---|
| Startup | `axisSearchHome(Y,250)`, then door work, then final `setHome(Y)` |
| Diagnostic `HomeAxis("y")` | `axisSearchHome(Y,250)` |
| Manual panel | `goHome(true,Y,500,true)` |
| `HomeXY` | concurrent `goHome(false,Y,200,true)` |
| Location workflow recovery | direct `goHome(true,Y,1800,true)` in `moveTo` branches |
| Board-test XY cycle | workflow-specific Y profile and rehome at speed `200` |

A receipt must name `source_mode`. One route cannot label diagnostic speed 250 as manual-panel speed 500.

### 7.2 `goHome`

Preserve the recovered branches:

- 24-V-dropped throws.
- uninitialized board returns raw `1`.
- already-home and cached position zero returns raw `0`.
- `rehome=true` first commands the source preposition move.
- leftward home search uses the requested mode speed.
- failure to stop before the source timeout issues source STOP and throws.
- a motor that stops without a confirmed home predicate can return without `setHome`.
- on confirmed home, source STOP precedes signed pre-zero readback and `setHome`.

`raw_return=0` does not prove home by itself.

### 7.3 `axisSearchHome`

Preserve:

1. initial `setHome`;
2. if source `queryHome` is true, preclear with `moveToAbs(10000)` and wait 500 ms;
3. clear the host `MotorHome` flag;
4. call `goHome(false, axis, speed)`.

Do not add a mandatory Linux deassert-then-reassert switch transition. Record a transition when seen, but accept the recovered final home predicate and valid `setHome` evidence.

### 7.4 Reference publication

Publish Y reference only when all are valid:

```text
final home predicate active
source STOP stage completed as required
setHome delivery and accepted reply are valid
zero readback is valid
board epoch is current
```

The controller-complete home can publish software reference immediately. Independent physical observation is not a software movement blocker. It remains mandatory for the physical-parity claim in section 18.3.

Standalone `setHome(Y)` is not a routine operator action. The recovered direct Y caller is the startup sequence. Any supervised recovery use requires a separate recovery action, explicit acknowledgement, valid current board authority, and a receipt that states no home travel was proved.

Keep the existing Linux home-search containment as `LINUX_EVIDENCE_HARDENING`:

```text
home_search_max_abs_delta=102956
source=selected Serial-206 Y maximum
```

This bound does not require a deassert/reassert switch transition and does not create a later drift latch.

---

## 8. Position discrepancy and lost-step policy

### 8.1 Normal movement

After a valid terminal observation, store:

```text
requested_position_steps
controller_target_steps
observed_position_steps
discrepancy_steps = observed - controller_target
completion_class
```

Then make the valid observed controller position the next controller-coordinate starting point.

A nonzero discrepancy alone:

- does not fail the command;
- does not mark Y desynchronized;
- does not require rehome;
- does not block a later Y command;
- does not invalidate Z or gripper.

Recovered normal move primitives clear the host `m_bHome`/at-home flag. That flag is not persistent coordinate-frame authority. Linux must not convert an ordinary source move into `desynchronized` merely because the motor is no longer at home.

### 8.2 Failed command with coherent terminal truth

If a command fails but valid evidence proves STOP, speed zero, actual position, and unchanged board authority, retain or restore usable controller-coordinate truth at that observed position. Record the failure. Do not create a drift-only latch.

Use `reconciliation_required` only when controller coordinate continuity or board state is unknown.

### 8.3 Caller-local lost-step policy

Recovered callers can apply local thresholds:

- `ControlLib.parkGantry(rehome=true)` handles `abs(HomeXY displacement) > 100` in that workflow.
- self-test evaluates `abs(HomeXY displacement) > 100` for its own result.
- board tests log displacement.

These are `OEM_CALLER_POLICY`. They do not create a global axis admission blocker.

---

## 9. `moveXY` and `HomeXY`

### 9.1 Composite ownership

The parent command owns method order only. X and Y child commands own their motor evidence, reference updates, interruptions, discrepancies, and terminal results.

```text
parent method receipt
  x child receipt
  y child receipt
```

A Y failure cannot rewrite X reference authority. An X failure cannot rewrite Y authority.

### 9.2 `moveXY`

Preserve these source dimensions:

- one public `moveXY(int x, int y)` wrapper returns `void`;
- current X/Y position determines source branch selection;
- if either delta is `<=20`, nonzero X then nonzero Y execute sequentially through their default blocking wrappers;
- otherwise X acceleration is `400` when X delta is over `10000`, else `350`;
- otherwise Y acceleration is `750` when Y delta is over `10000`, else `400`;
- the larger-delta axis is issued first with its low-level nonblocking primitive;
- when the shorter delta is over `4000`, preserve the source `50 * larger / shorter` ms stagger before issuing the other axis;
- after a 5 ms sleep, STA waits each motor handle for up to 5 seconds, while MTA waits both handles for 5 seconds total;
- a composite wait timeout is logged and the source wrapper returns normally;
- event waits are motor-specific;
- profile restoration occurs only on the recovered normal path;
- the all-zero no-op applies only in the source board-present branch.

The typed parent receipt must expose source log-only timeout separately from Linux child completion evidence. It must not invent child success from the wrapper's `void` return.

The recovered missing-board branches are anomalous, including the X-missing branch that routes the Y value through an X call. Production Serial-206 requires both mandatory boards for an XY method. Linux rejects a missing mandatory board before motion and records `source_anomalous_missing_board_branch_rejected`. It must not silently repair the source branch and call that literal parity.

Exception-safe profile restoration is a logged Linux hardening.

### 9.3 `HomeXY`

Preserve:

- X speed/acceleration `200/200` and Y speed/acceleration `200/200`;
- parallel `goHome(false, ..., 200, true)` leaves;
- signed X and Y returns;
- unbounded source `Task.WaitAll` behavior;
- normal-path restoration to X `1700/350` and Y `1800/400`;
- source return `null` when either required board is absent.

Linux may guarantee restoration in `finally`. Record the deviation.

### 9.4 Composite STOP

Y STOP affects Y only.

- queued Y-only rows become `cleared`;
- a queued XY parent marks the Y child `cleared` and the parent `cleared` without dispatching the X child;
- an active XY parent marks the Y child `interrupting`, lets an independent active X child finish, and then records a partial/interrupted parent outcome;
- unrelated X, Z, and gripper commands remain intact.

---

## 10. Shared board 4 authority

### 10.1 OEM facts

`ClassHeadBoard.activateBoard` resets host motor-home flags before sending command 64. The recovered source does not prove that command 64 erases controller positions.

`ClassControlInterface.forceAbortMotion` and `ClassHeadBoard.forceAbortMotion` set the software abort or no-24-V condition and release motor waiters. They do not prove addressed MST delivery to each motor.

### 10.2 Linux board epoch

`board_epoch` is `LINUX_REPLACEMENT`. It is a logical control-plane epoch, not a controller register.

One canonical board row contains:

```text
board_id=4
state=inactive | transitioning | active | faulted
prior_board_epoch
active_board_epoch
transition_id
deactivation_attempt_id
deactivation_delivery
deactivation_reply_valid
deactivation_status_code
activation_attempt_id
activation_delivery
activation_reply_valid
activation_status_code
member_motors={y:0,z:1,gripper:2}
state_version
updated_at
```

Rules:

1. in one pre-frame SQLite transaction, reserve `transition_id`, copy the current active epoch to `prior_board_epoch`, set `active_board_epoch=NULL`, set state `transitioning`, and mark every board-4 member profile/reference `generation_stale`;
2. block normal board-4 dispatch before the first command-64 frame;
3. deliver and record command 64 deactivation `0`;
4. only after an accepted deactivation reply, deliver and record command 64 activation `1`;
5. partial, negative, timeout, or ambiguous evidence in either phase sets board state `faulted`; the prior epoch cannot become active again;
6. only the ordered, accepted `0 -> 1` cycle mints `active_board_epoch=prior_board_epoch+1` and state `active` in one transaction;
7. each member profile remains stale until prepared against that new epoch;
8. valid controller truth after a board change can restore `prepared_unreferenced` controller-coordinate usability. It cannot publish `reference_state=referenced`;
9. only the complete source home plus accepted set-home plus valid zero-readback predicate publishes a referenced state;
10. requalifying Y neither proves nor erases Z;
11. a transport-owner replacement or process restart cannot reuse a prepared board epoch when controller continuity is not proved;
12. `reuse_prepared` cannot bypass board or axis epoch checks.

The generation fence is an operator-required authority policy. It is not a drift threshold.

### 10.3 Axis authority

Each axis row contains:

```text
axis
board_id
motor_id
ownership_generation
prepared_board_epoch
profile_fingerprint
lifecycle_state
reference_state
origin_position_steps
observed_position_steps
last_discrepancy_steps
last_command_id
last_receipt_id
state_version
updated_at
```

Y and Z share `board_epoch`. They do not share `lifecycle_state`, `reference_state`, active command, interrupt epoch, or terminal position.

Normal Y relative and absolute moves may run in `prepared_unreferenced` or `referenced_ready`. They require valid current board/profile authority and method-specific controller evidence. They do not require independent physical observation, a fresh unrelated Z lifecycle, broad full-machine readiness, or a target-equality history.

---

## 11. Durable Linux command journal and scheduler

### 11.1 Classification

The recovered source has:

- an application `BlockingCollection<motionCommands>` for major worker operations;
- a low-level CAN command queue whose entries carry axis identity.

This evidence does not prove that every OEM Y API call used one durable movement ledger.

Recovered `ClassNovoCommandQueue.ClearAll(axis)` drops the final queued entry and all `axis=-1` entries while rebuilding the queue. Do not port that defect. Axis containment and sibling preservation in sections 9 and 12 are `OPERATOR_REQUIRED_REPLACEMENT`.

The design below is `LINUX_REPLACEMENT`. It preserves method order and fast admission. It is not a literal port of one recovered queue.

### 11.2 Existing database and schema migration

Use the existing runtime root and database:

```text
<runtime_state_root>/bioxp_runtime.db
PRAGMA journal_mode=WAL
PRAGMA synchronous=FULL
```

Do not create another database or logging framework. Preserve `synchronous=FULL`. Measure admission latency before proposing a durability change. Any downgrade requires a separate operator decision.

#### One migration owner

`src/bioxp/oem_runtime_store.py` owns `migrate_runtime_database_v2(connection)`. API lifespan runs it before constructing `OEMRuntimeStore`, `OperatorReceiptStore`, the scheduler, or any route owner. `OperatorReceiptStore` stops setting `PRAGMA user_version`; both stores only verify the migrated schema after startup.

The version contract is exact:

```text
user_version=0 -> create the complete v2 schema
user_version=1 -> migrate the existing runtime schema to v2
user_version=2 -> verify all tables, columns, indexes, and constraints; make no schema write
user_version>2 -> refuse startup without modifying the database
```

Migration runs under one `BEGIN IMMEDIATE` transaction after a verified SQLite backup and JSON-file hash capture. Failure rolls back all SQL changes. The migration record stores version `2`, backup digest, source JSON digests, start/end time, and result.

#### v2 table ownership and exact DDL

The migration preserves `runtime_metadata`, `operator_commands`, `operator_transitions`, and `serial206_receipts` with their frozen v1 columns and indexes. It creates these tables exactly, with foreign keys enabled:

```sql
CREATE TABLE runtime_schema_migrations (
  version INTEGER PRIMARY KEY CHECK(version=2),
  backup_sha256 TEXT NOT NULL CHECK(length(backup_sha256)=64),
  source_json_digests_json TEXT NOT NULL CHECK(json_valid(source_json_digests_json)),
  started_at REAL NOT NULL,
  finished_at REAL NOT NULL,
  result TEXT NOT NULL CHECK(result='committed')
) WITHOUT ROWID;

CREATE TABLE serial206_board_authority (
  board_id INTEGER PRIMARY KEY CHECK(board_id=4),
  state TEXT NOT NULL CHECK(state IN ('inactive','transitioning','active','faulted')),
  prior_board_epoch INTEGER CHECK(prior_board_epoch IS NULL OR prior_board_epoch>=0),
  active_board_epoch INTEGER CHECK(active_board_epoch IS NULL OR active_board_epoch>=0),
  transition_id TEXT,
  deactivation_attempt_id TEXT,
  deactivation_delivery INTEGER CHECK(deactivation_delivery IS NULL OR deactivation_delivery IN (0,1)),
  deactivation_reply_valid INTEGER CHECK(deactivation_reply_valid IS NULL OR deactivation_reply_valid IN (0,1)),
  deactivation_status_code INTEGER,
  activation_attempt_id TEXT,
  activation_delivery INTEGER CHECK(activation_delivery IS NULL OR activation_delivery IN (0,1)),
  activation_reply_valid INTEGER CHECK(activation_reply_valid IS NULL OR activation_reply_valid IN (0,1)),
  activation_status_code INTEGER,
  member_motors_json TEXT NOT NULL CHECK(json_valid(member_motors_json)),
  state_version INTEGER NOT NULL CHECK(state_version>=1),
  updated_at REAL NOT NULL
);

CREATE TABLE serial206_axis_authority (
  axis TEXT PRIMARY KEY CHECK(axis IN ('y','z','gripper')),
  board_id INTEGER NOT NULL REFERENCES serial206_board_authority(board_id),
  motor_id INTEGER NOT NULL CHECK(motor_id BETWEEN 0 AND 2),
  ownership_generation INTEGER NOT NULL CHECK(ownership_generation>=0),
  prepared_board_epoch INTEGER CHECK(prepared_board_epoch IS NULL OR prepared_board_epoch>=0),
  profile_fingerprint TEXT,
  lifecycle_state TEXT NOT NULL CHECK(lifecycle_state IN ('unprepared','prepared_unreferenced','referenced_ready','generation_stale','reconciliation_required','faulted')),
  reference_state TEXT NOT NULL CHECK(reference_state IN ('unreferenced','referenced','generation_stale','reconciliation_required')),
  origin_position_steps INTEGER,
  observed_position_steps INTEGER,
  last_discrepancy_steps INTEGER,
  last_command_id TEXT,
  last_receipt_id TEXT,
  interrupt_epoch INTEGER NOT NULL DEFAULT 0 CHECK(interrupt_epoch>=0),
  state_version INTEGER NOT NULL CHECK(state_version>=1),
  updated_at REAL NOT NULL
) WITHOUT ROWID;

CREATE TABLE serial206_movement_methods (
  method_id TEXT PRIMARY KEY,
  idempotency_key TEXT NOT NULL UNIQUE,
  action_id TEXT NOT NULL,
  canonical_inputs_sha256 TEXT NOT NULL CHECK(length(canonical_inputs_sha256)=64),
  state TEXT NOT NULL CHECK(state IN ('queued','active','completed','completed_partial','failed','cleared','interrupted','ambiguous')),
  state_version INTEGER NOT NULL CHECK(state_version>=1),
  failure_policy TEXT NOT NULL CHECK(failure_policy='require_completed'),
  child_count INTEGER NOT NULL CHECK(child_count>=1),
  accepted_at REAL NOT NULL,
  started_at REAL,
  finished_at REAL
) WITHOUT ROWID;

CREATE TABLE serial206_movement_commands (
  sequence INTEGER PRIMARY KEY AUTOINCREMENT,
  command_id TEXT NOT NULL UNIQUE,
  idempotency_key TEXT NOT NULL,
  action_id TEXT NOT NULL,
  method_id TEXT REFERENCES serial206_movement_methods(method_id) ON DELETE CASCADE,
  method_order INTEGER NOT NULL DEFAULT 0 CHECK(method_order>=0),
  parallel_group INTEGER NOT NULL DEFAULT 0 CHECK(parallel_group>=0),
  axis_scope TEXT,
  board_scope_json TEXT NOT NULL CHECK(json_valid(board_scope_json)),
  ownership_generation INTEGER NOT NULL CHECK(ownership_generation>=0),
  expected_board_epochs_json TEXT NOT NULL CHECK(json_valid(expected_board_epochs_json)),
  canonical_inputs_sha256 TEXT NOT NULL CHECK(length(canonical_inputs_sha256)=64),
  state TEXT NOT NULL CHECK(state IN ('queued','dispatched','issued_pending','interrupting','completed','failed','cleared','interrupted','ambiguous','rejected')),
  state_version INTEGER NOT NULL CHECK(state_version>=1),
  admitted_interrupt_epochs_json TEXT NOT NULL CHECK(json_valid(admitted_interrupt_epochs_json)),
  accepted_at REAL NOT NULL,
  queued_at REAL NOT NULL,
  dispatched_at REAL,
  finished_at REAL,
  terminal_receipt_id TEXT
);

CREATE TABLE serial206_command_resources (
  command_id TEXT NOT NULL REFERENCES serial206_movement_commands(command_id) ON DELETE CASCADE,
  resource_key TEXT NOT NULL,
  PRIMARY KEY(command_id,resource_key)
) WITHOUT ROWID;

CREATE TABLE serial206_command_dependencies (
  command_id TEXT NOT NULL REFERENCES serial206_movement_commands(command_id) ON DELETE CASCADE,
  depends_on_command_id TEXT NOT NULL REFERENCES serial206_movement_commands(command_id) ON DELETE CASCADE,
  required_terminal TEXT NOT NULL CHECK(required_terminal='completed'),
  PRIMARY KEY(command_id,depends_on_command_id),
  CHECK(command_id<>depends_on_command_id)
) WITHOUT ROWID;

CREATE TABLE serial206_interrupt_imports (
  record_sha256 TEXT PRIMARY KEY CHECK(length(record_sha256)=64),
  interrupt_attempt_id TEXT NOT NULL UNIQUE,
  axis TEXT NOT NULL,
  interrupt_epoch INTEGER NOT NULL CHECK(interrupt_epoch>=0),
  imported_at REAL NOT NULL,
  receipt_id TEXT NOT NULL
) WITHOUT ROWID;
```

Required new indexes are:

```sql
CREATE UNIQUE INDEX serial206_movement_commands_idempotency_idx ON serial206_movement_commands(idempotency_key);
CREATE INDEX serial206_movement_commands_ready_idx ON serial206_movement_commands(state,sequence);
CREATE INDEX serial206_movement_commands_method_idx ON serial206_movement_commands(method_id,method_order,parallel_group,sequence);
CREATE INDEX serial206_command_resources_lookup_idx ON serial206_command_resources(resource_key,command_id);
CREATE INDEX serial206_command_dependencies_reverse_idx ON serial206_command_dependencies(depends_on_command_id,command_id);
```

The frozen v1 `serial206_receipts` table has no stream CHECK and already supports a `y` stream. Preserve its bytes, row count, primary keys, and three existing indexes. Migration does not rebuild it.

Keep existing `operator_commands` and `operator_transitions` v1 rows as immutable history. Do not reinterpret them as dispatchable queue rows. v2 queue rows use `serial206_movement_commands`. Each top-level v2 admission also writes one ordinary operator-history row in the same transaction; method child state remains in movement tables and detailed v2 evidence.

Backfill is exact: old receipt, operator-command, transition, and metadata rows remain byte-for-byte unchanged; no old command becomes dispatchable; all new movement/method/resource/dependency/import tables start empty; board and axis authority come only from the JSON cutover rules below. Set `PRAGMA user_version=2` only after row counts, primary-key digests, foreign-key checks, and required indexes pass inside the migration transaction.

#### JSON authority cutover

For `user_version` 0 or 1, parse the configured reference-state file and `<runtime_state_root>/serial206_oem_initialization_state.json` before the SQL transaction. Validate their current schemas with the frozen v1 readers and hash the exact bytes. Invalid JSON aborts startup.

The import mapping is exact and conservative:

```text
board 4:
  prior_board_epoch = valid z_lifecycle.board_lifecycle_generation, else null
  active_board_epoch = null
  state = faulted
  transition_id = migration-v2-continuity-unproved
  member_motors_json = {"y":0,"z":1,"gripper":2}

Y axis:
  lifecycle_state = generation_stale when a Y reference row exists, else unprepared
  reference_state = generation_stale for legacy referenced, reconciliation_required for legacy desynced, else unreferenced
  origin_position_steps = legacy Y origin when integer, else null
  prepared_board_epoch/profile_fingerprint/observed position = null

Z axis:
  lifecycle_state = unprepared only for legacy unprepared; otherwise generation_stale
  reference_state = generation_stale for legacy referenced, reconciliation_required for legacy desynced or failed_latched, else unreferenced
  origin_position_steps = legacy Z origin when integer, else null
  prepared_board_epoch = valid legacy board generation as historical provenance
  profile_fingerprint/observed position = null

gripper axis:
  lifecycle_state = unprepared
  reference_state = unreferenced
  all position/profile fields = null
```

An existing legacy `executing` state aborts migration until mutation admission is quiesced and that command is resolved or explicitly recorded ambiguous. The imported ownership generation is the new process generation. State versions start at 1; interrupt epochs start at 0.

This mapping preserves historical references without authorizing them. JSON alone cannot prove transport continuity after restart, so no imported axis is dispatchable. A later ordered command-64 `0 -> 1` cycle and per-axis preparation must create new active authority under section 10.

The migration commits SQL authority and its cutover marker together. After commit, regenerate compatibility JSON projections from SQL. Route and scheduler readers use SQLite immediately; a stale projection cannot overwrite SQLite. Projection failure is an operator warning and rollback concern, not a second authority path.

SQLite is canonical for board authority, axis authority used by the scheduler, queue state, and compact receipts. Use one transaction owner for movement-journal, Serial-206 authority, operator-command, transition, and terminal-receipt updates. Independent store connections must not perform authority-bearing dual writes.

### 11.3 Command and method state machines

Each normal command row contains:

```text
sequence
command_id
idempotency_key
action_id
method_id
method_order
parallel_group
axis_scope
board_scope
ownership_generation
expected_board_epoch_by_board
canonical_inputs_sha256
state
state_version
interrupt_epoch_by_axis_at_dispatch
accepted_at
queued_at
dispatched_at
finished_at
terminal_receipt_id
```

`resource_keys` and dependencies live in their normalized child tables.

Persisted command states are closed:

```text
NONTERMINAL: queued | dispatched | issued_pending | interrupting
TERMINAL: completed | failed | cleared | interrupted | ambiguous | rejected
```

Legal command transitions:

```text
new admission -> queued
queued -> dispatched | cleared | rejected
dispatched -> issued_pending | completed | failed | interrupting | ambiguous
issued_pending -> completed | failed | interrupting | ambiguous
interrupting -> interrupted | failed | ambiguous
terminal -> no later state
```

`active` is a derived predicate over `dispatched`, `issued_pending`, and `interrupting`. `restart-dispatched` is not a state. On startup, every persisted active state becomes `ambiguous` in one recovery transaction before dispatch resumes.

Persisted method states are:

```text
queued | active | completed | completed_partial | failed | cleared | interrupted | ambiguous
```

One method-admission transaction inserts the parent method, all child commands, every resource row, every dependency edge, and compact operator history. Children in the same `parallel_group` have no edge between them. Every child in group N depends on every child in the prior sequential group. The fixed dependency policy is `require_completed`; failed, cleared, interrupted, ambiguous, or rejected prerequisites cause dependent queued children to become `cleared` with `reason=dependency_not_completed`.

Parent state becomes `active` when its first child dispatches. After every child terminal transition, compute parent terminal state in the same transaction:

```text
all children completed -> completed
any child ambiguous -> ambiguous
at least one completed plus any failed/cleared/interrupted/rejected -> completed_partial
no completed child and any interrupted -> interrupted
all children cleared before issue -> cleared
otherwise, when all children terminal -> failed
```

This algorithm makes active XY plus Y STOP finish `completed_partial` after X ends. A queued XY cleared on Y finishes `cleared` without dispatching X.

### 11.4 Resource scheduler

Keep one global admission sequence and method identity. Dispatch is resource-scoped.

- commands for the same motor serialize;
- board-4 command 64 conflicts with every board-4 motor;
- profile mutations conflict with their motor;
- transport request/reply exchange remains single-writer;
- waiting for a motor event does not hold the transport writer;
- source-defined XY parallel leaves can overlap;
- independent motors can proceed when resource and method barriers permit;
- a submitted method preserves its explicit sequential and parallel groups.

The scheduler uses one robot process owner. Before claim, it resolves failed dependencies to `cleared` and stale generation/epoch rows to `rejected`. A row is eligible only when:

```text
state=queued
all dependency rows require and observe command state=completed
current ownership generation equals the row
all expected board epochs equal current active epochs
no active command holds a conflicting resource
no earlier global barrier remains nonterminal
```

Resource keys are acquired in lexical order and released only in the child terminal transaction. The dispatch transaction rechecks eligibility, updates `queued -> dispatched` with expected `state_version`, and records active resource ownership. The single-writer transport lock is acquired after this SQL claim and released after request/reply exchange.

Dispatch scans global sequence order and chooses the earliest eligible row for each disjoint resource set. A later row may pass an earlier active or queued row only when resources are disjoint and no dependency, method, or global barrier connects them. Same-motor order is strict. Board activation uses `board:4:*` as a global board barrier.

A global sequence is durable truth. It is not universal physical serialization.

### 11.5 Atomic admission

For normal actions:

1. validate the typed request and current authority;
2. reserve the idempotency key, sequence, method membership, and compact command row in one bounded SQLite transaction;
3. return the compact admission receipt;
4. dispatch asynchronously on the robot.

The handler does not wait for physical completion.

No normal command is sent if admission persistence fails.

Once admission commits, client disconnect or HTTP cancellation does not remove the command. A retry with the same idempotency key recovers the committed command ID.

### 11.6 Idempotency

- same key plus identical canonical request returns the existing command ID;
- same key plus different request returns HTTP 409;
- nonterminal idempotency claims are never removed by retention;
- terminal history retention cannot orphan method rows or live claims.

Safety interrupts are physical attempts. They are never suppressed by a repeated idempotency key. Each attempt gets a unique `interrupt_attempt_id` and may link to the same operator request.

### 11.7 Dispatch and terminal CAS

Dispatcher claim requires a compare-and-swap on:

```text
command_id
state=queued
state_version
current ownership generation
current board epoch
```

Terminal write requires the dispatch state version and unchanged axis interrupt epoch. A stale completion cannot overwrite `interrupting`, `cleared`, `interrupted`, or `ambiguous`.

Terminal receipt rows, axis authority updates, method-child updates, and command transition are committed in one SQLite transaction.

Large transport evidence stays in existing bounded artifact files and is referenced by digest/path from the compact receipt.

### 11.8 Restart

- queued rows survive restart;
- they dispatch only after current-generation and current-board-epoch re-admission;
- dispatched nonterminal rows become `ambiguous`;
- ambiguous rows never auto-dispatch again;
- terminal SQLite state regenerates stale JSON projections;
- no watchdog or polling thread issues recovery motion.

A process restart increments runtime ownership generation. If transport/board continuity is not proved, board state becomes `faulted`, and member axes become `generation_stale`, before queued re-admission.

If terminal SQLite persistence fails after a physical side effect, retain the command as nonterminal/ambiguous and write bounded evidence through the existing durable fallback or artifact path. Never redispatch it automatically.

### 11.9 Latency acceptance protocol

Use `time.monotonic_ns()` at these boundaries:

```text
request_enter
admission_sql_begin
admission_sql_commit
http_response_complete
first_stop_transport_write_call
stop_delivery_complete
stop_durable_record_complete
```

Run on the target robot filesystem and deployed Python process with physical transport replaced by the approved deterministic Serial-206 transport fixture. Use 30 unmeasured warm-up calls followed by 200 measured calls per case. Keep all samples. Compute nearest-rank p95. Record raw samples, process affinity, database path, WAL settings, database size, and fixture digest.

Required cases and thresholds:

```text
single-client normal admission: SQL p95 <= 50 ms and request-to-response p95 <= 150 ms
32 concurrent normal clients, 200 total admissions: request-to-response p95 <= 150 ms
active simulated Y command plus normal SQLite traffic: STOP entry-to-first-write p95 <= 50 ms
SQLite BEGIN IMMEDIATE held by another connection for 250 ms: STOP entry-to-first-write p95 <= 50 ms and post-delivery fsynced fallback p95 <= 100 ms
SQLite available: STOP delivery-to-SQLite-commit p95 <= 50 ms
```

The interrupt SQLite busy budget is 5 ms after STOP delivery. At expiry, write the fallback instead of waiting on the normal writer. A threshold miss fails the software speed gate; it does not authorize weaker durability or skipped logs.

These are acceptance targets, not assumed facts.

---

## 12. STOP, aggregate abort, and emergency stop

### 12.1 Y STOP

`oem.y.stop` preserves the recovered motor-specific double MST sequence.

Receipt fields:

```text
first_delivery
first_reply
second_delivery
second_reply
motor_command_raw_return_from_second_reply
board_wrapper_return_kind=void
public_wrapper_return_kind=void
terminal_speed
terminal_position
queue_rows_cleared
active_command_transition
physical_effect_verified=false
```

The motor method ignores the first raw result and evaluates the second. Both the board wrapper and `ClassControlInterface.stopMotor` discard that result and return `void`. Linux exposes all layers without rewriting them.

Y STOP uses an independent interrupt lane. It must not wait behind the normal scheduler or a normal SQLite writer lock.

Before the first STOP frame, take the bounded motor-local interrupt gate, increment the in-process Y interrupt epoch, set an in-memory dispatch pause, and shadow any active Y child as `interrupting`. Release the gate before transport. Normal admission snapshots the current interrupt epoch; final pre-dispatch checks require an exact match. Terminal handlers use the same gate and epoch, so a completion after reservation cannot commit over the interrupt. A terminal handler committed before reservation remains terminal.

The first STOP frame attempt has priority. After both attempts, clear every queued Y child whose admitted interrupt epoch is older than the reserved epoch. Persist the reserved epoch, active transition, cleared rows, and STOP receipt in one SQLite transaction. Normal Y dispatch remains paused until that transaction or fallback completes.

The interrupt persistence path uses a dedicated connection to the same database with `busy_timeout=5`; it never borrows the normal writer connection or lock. At the 5 ms SQLite busy budget, append one canonical JSON record to `<runtime_state_root>/operator_interrupt_fallback.jsonl` using the existing operator fallback lock and `O_APPEND`, flush, and `fsync` the file and parent directory. The record contains format version, interrupt attempt ID, axis, reserved epoch, affected command IDs and pre-states, both transport attempts/replies, process generation, wall and monotonic times, and a SHA-256 over canonical content.

API startup imports and verifies every fallback record before starting the scheduler or mutation routes. In one transaction, it advances the durable axis interrupt epoch to at least the record epoch, changes matching persisted active commands to `ambiguous`, clears matching queued pre-epoch commands, writes the STOP receipt/transitions, and records the imported hash. Duplicate hashes are idempotent. Invalid or unimportable records keep mutation routes unavailable. Only after SQL commit may the importer rotate the imported file. A process crash in the delivery-to-persistence window therefore cannot authorize completion or redispatch.

### 12.2 OEM aggregate abort

`oem.abort_all` preserves the recovered software abort and waiter-release behavior. Its receipt must not claim per-motor STOP delivery unless those frames were sent by a separate hardening action.

### 12.3 Cross-motor physical emergency stop

Changes to `meta.emergency_stop` are outside this Y specification. This slice neither upgrades it nor claims that it sends physical STOP frames. Required interrupt actions are `oem.y.stop` and source-shaped `oem.abort_all` only.

---

## 13. Robot provider and route contract

### 13.1 Provider ownership

Extend `Serial206OemInitializationProvider` or extract a focused provider-owned axis module without creating a second authority.

Required Y functions:

```text
prepare_y_without_motion
execute_y_move_steps
execute_y_move_absolute
execute_y_manual_panel_home
execute_y_diagnostic_home
execute_y_stop
reconcile_y_controller_truth
y_status_projection
execute_xy_move
execute_home_xy
```

Low-level profile, current-mode, and set-home stages remain internal methods.

### 13.2 Routine semantic actions

```text
oem.y.move_steps
oem.y.move_absolute
oem.y.manual_panel_home
oem.y.stop
oem.xy.move_absolute
oem.xy.home
```

Routine inputs are closed and source-shaped:

| Action | Allowed inputs | Fixed source mode |
|---|---|---|
| `oem.y.move_steps` | signed integer `steps` | blocking high-level relative wrapper |
| `oem.y.move_absolute` | integer `target_steps` | manual-panel absolute mode, `waitforstop=false`, no free profile input |
| `oem.y.manual_panel_home` | no speed/current override | `goHome(true,Y,500,true)` |
| `oem.y.stop` | operator reason and observed generations | addressed Y double MST interrupt |
| `oem.xy.move_absolute` | integer `x_steps`, integer `y_steps` | recovered `moveXY(int,int)` branch logic |
| `oem.xy.home` | no speed/current override | recovered `HomeXY()` |

Every normal request uses this strict common envelope:

```text
schema_version: literal bioxp.operator_action_request.v2
idempotency_key: string, 1..128 bytes
expected_ownership_generation: integer >= 0
expected_board_epoch_by_board: object<string, integer >= 0>
inputs: one strict action-specific object
```

Action-specific `inputs` are:

```text
oem.y.move_steps: {steps: signed int32}
oem.y.move_absolute: {target_steps: signed int32}
oem.y.manual_panel_home: {}
oem.xy.move_absolute: {x_steps: signed int32, y_steps: signed int32}
oem.xy.home: {}
```

Relative admission requires valid current Y position and an effective target in `20..102936`; otherwise it returns 409 without transport. Absolute requests keep the signed int32 caller value, record board and motor effective targets, and preserve source low/high clamp and near-high no-op behavior.

STOP uses a separate strict body:

```text
schema_version: literal bioxp.operator_interrupt_request.v1
reason: string, 1..500 bytes
observed_ownership_generation: integer | null
observed_board_epoch_by_board: object<string, integer >= 0>
```

Generation fields on STOP are evidence only. A mismatch does not suppress an addressed STOP attempt. The robot creates a unique `interrupt_attempt_id`; clients cannot supply an idempotency key for STOP.

`POST /operator/methods` accepts a closed method template, not an arbitrary graph:

```text
schema_version: literal bioxp.operator_method_request.v1
idempotency_key: string, 1..128 bytes
method_action_id: oem.xy.move_absolute | oem.xy.home
expected_ownership_generation: integer >= 0
expected_board_epoch_by_board: object<string, integer >= 0>
inputs: strict action-specific object above
```

The robot expands the template into the fixed parent, child, group, resource, and dependency rows from sections 8 and 11.

Existing internal pathing callers may select the recovered blocking Y absolute wrapper or acceleration overload through a typed provider method. They do not expand the routine action inputs.

Internal absolute intent is a closed discriminated union:

```text
manual_panel: target_steps:int, waitforstop=false, acceleration_override=null
wrapper_default: target_steps:int, waitforstop=true, acceleration_override=null
acceleration_overload: target_steps:int, waitforstop:bool=true, acceleration_override:int required
board_test_my: target_steps:int, waitforstop=false, acceleration_override=400
```

Absolute target integers preserve source clamp behavior. Relative requests accept signed integer steps only when the readback-derived effective target is within `20..102936`.

Advanced diagnostic action:

```text
oem.y.diagnostic_home
```

Service/recovery actions:

```text
oem.y.prepare_without_motion
oem.y.reconcile_controller_truth
```

Do not publish normal actions named:

```text
oem.y.profile_set
oem.y.set_speed
oem.y.set_acc
oem.y.set_current
oem.y.set_stall_guard
oem.y.disable_right_switch
oem.y.enable
oem.y.disable
oem.y.set_home
```

### 13.3 Generic route retirement

For one explicit BMS cutover window, these Y paths may call the canonical provider and return its canonical v2 receipt:

```text
/motion/oem/manual/relative
/motion/oem/manual/absolute
/motion/oem/manual/home
/motion/axis/relative
/motion/axis/absolute
/motion/axis/home
/motion/diagnostics/stop
```

After the cutover window, these paths return HTTP 410 for Y with the canonical replacement action ID. Stable reads can remain.

These Y authority-forcing paths return HTTP 410 immediately when robot v2 is enabled:

```text
/motion/oem/manual/sethome
/motion/axis/zero
/motion/reference/mark_referenced
```

No compatibility path can keep a second reference or receipt owner.

Current `/motion/oem/manual/home` incorrectly routes Y to diagnostic `HomeAxis` while using a manual label. Replace it with the correct semantic action split.

### 13.4 Existing operator routes

Keep and extend:

```text
POST /operator/actions/{action_id}/admission
POST /operator/actions/{action_id}
GET  /operator/actions/receipts/{command_id}?detail=false
GET  /operator/actions/receipts/{command_id}?detail=true
GET  /operator/actions/history
GET  /operator/control-catalog
GET  /operator/dashboard
```

Add atomic method submission in WP5:

```text
POST /operator/methods
GET  /operator/methods/{method_id}
```

`POST /operator/actions/{action_id}/admission` remains a no-side-effect readiness query. It never reserves a command ID and cannot substitute for the durable invocation transaction.

Normal motion invocation returns HTTP 202 with a compact receipt:

```json
{
  "schema_version": "bioxp.operator_action_receipt.v2",
  "command_id": "cmd-...",
  "action_id": "oem.y.move_steps",
  "status": "queued",
  "terminal": false,
  "sequence": 1,
  "method_id": null,
  "ownership_generation": 1,
  "expected_board_epoch_by_board": {"4": 2},
  "state_version": 1,
  "status_path": "/operator/actions/receipts/cmd-...",
  "accepted_at": 0.0,
  "queued_at": 0.0,
  "dispatched_at": null,
  "finished_at": null,
  "terminal_receipt_id": null,
  "completion_class": null,
  "physical_effect_verified": false,
  "error": null
}
```

STOP returns after its bounded interrupt attempt and persistence path. It does not use the normal asynchronous admission semantics.

### 13.5 Dashboard

Publish strict `y_axis` with:

```text
axis="y"
board_id=4
motor_id=0
ownership_generation
prior_board_epoch: integer | null
active_board_epoch: integer | null
prepared_board_epoch: integer | null
lifecycle_state
reference_state
position_steps: integer | null
position_reply_valid: boolean
position_status_code: integer | null
speed_steps_s: integer | null
speed_reply_valid: boolean
speed_status_code: integer | null
left_switch_raw: integer | null
left_switch_reply_valid: boolean
left_switch_status_code: integer | null
home_effective: boolean | null
profile_fingerprint: string | null
profile_readback_valid: boolean
profile_mismatches: array<string>
active_command: compact receipt | null
interrupt_epoch: integer >= 0
latest_compact_receipt: compact receipt | null
last_discrepancy_steps: integer | null
state_version: integer >= 1
updated_at: finite number
physical_position_verified: boolean
```

`physical_position_verified` defaults false and changes only from independently recorded observation.

Publish one global command-journal projection plus per-resource cards. Do not preserve the old per-axis depth-eight queue as canonical truth.

---

## 14. Wire versions and BioModStack cutover

### 14.1 Strict producer and consumer schemas

The producer versions are mandatory:

```text
bioxp.operator_dashboard.v2
bioxp.operator_control_catalog.v2
bioxp.operator_action_receipt.v2
bioxp.operator_action_history.v2
bioxp.oem_command_queue.v1
bioxp.operator_method.v1
```

All models use `extra="forbid"`. Every listed field is required unless marked nullable. Times are finite Unix seconds. IDs are nonempty strings. Integers reject booleans. Generation and epoch integers are nonnegative. Optional data is represented by required nullable fields, never by omitted keys.

The command receipt status enum is closed:

```text
queued, dispatched, issued_pending, interrupting,
completed, failed, cleared, interrupted, ambiguous, rejected
```

The first four states are nonterminal. The remaining states are terminal. Method status adds terminal `completed_partial` and otherwise uses the applicable command names. Producer and BMS frontend use the same enum and terminality table.

Compact action receipt fields are:

```text
schema_version: literal bioxp.operator_action_receipt.v2
command_id: string
action_id: string
status: CommandStatus
terminal: boolean derived from the fixed table
sequence: integer >= 1
method_id: string | null
ownership_generation: integer >= 0
expected_board_epoch_by_board: object<string, integer >= 0>
state_version: integer >= 1
status_path: string
accepted_at: finite number
queued_at: finite number
dispatched_at: finite number | null
finished_at: finite number | null
terminal_receipt_id: string | null
completion_class: string | null
physical_effect_verified: boolean
error: {code:string,message:string,retryable:boolean} | null
```

`detail=false` returns exactly the compact shape. `detail=true` returns the compact fields plus these required fields:

```text
canonical_inputs: object
requested_values: object<string, integer | number | string | boolean | null>
effective_values: object<string, integer | number | string | boolean | null>
observed_values: object<string, integer | number | string | boolean | null>
raw_return_layers: object
controller_evidence: object
transport_artifacts: array<{sha256:string,path:string,bytes:integer>=0}>
child_receipts: array<compact receipt>
transitions: array<{transition_id:string,from_status:string|null,to_status:CommandStatus,at:number,reason:string|null}>
```

History v2 is:

```text
schema_version: literal bioxp.operator_action_history.v2
items: array<compact receipt>
next_cursor: string | null
limit: integer 1..200
```

Queue v1 is:

```text
schema_version: literal bioxp.oem_command_queue.v1
generated_at: finite number
items: array<{command_id:string,sequence:int>=1,status:nonterminal CommandStatus,method_id:string|null,resource_keys:array<string>,accepted_at:number}>
```

Dashboard v2 requires:

```text
schema_version: literal bioxp.operator_dashboard.v2
generated_at: finite number
ownership_generation: integer >= 0
board4: strict board-authority object
y_axis: strict axis-authority object
active_commands: array<compact receipt>
command_queue: queue-v1 object
latest_receipts: array<compact receipt>
```

The strict `y_axis` object contains every field in section 13.5. Reply-derived scalar fields have an adjacent required validity boolean and nullable status code. `board4` exposes state, prior and active epochs, transition phase evidence, member motors, state version, and update time.

Catalog v2 is mandatory because the frozen catalog embeds dashboard data. It contains:

```text
schema_version: literal bioxp.operator_control_catalog.v2
dashboard: dashboard-v2 object
actions: array<{action_id:string,request_schema_version:string,response_schema_version:string,interrupt:boolean,enabled:boolean,disabled_reason:string|null}>
```

Method v1 contains `method_id`, action ID, fixed method status, state version, child compact receipts, accepted time, and nullable finished time.

BMS uses a discriminated `schema_version` union for complete v1 and v2 models. It does not widen strict v1 models or translate a v2 state into a v1 state.

Robot and BMS relay HTTP mapping is fixed:

```text
200: readiness query, receipt/history/dashboard/catalog/method GET, or completed STOP attempt response
202: durable normal command or method admission
400: malformed or out-of-range request
404: unknown command, method, or action
409: idempotency conflict, stale generation/epoch, or illegal state transition
410: retired Y mutation route
422: strict schema validation failure
503: authority unavailable, migration unavailable, transport unavailable before normal admission, or STOP persistence/fallback failure
```

BMS preserves the robot status code and strict error payload. A network failure before any robot response is BMS 502 with `robot_response_received=false`; a BMS timeout is 504 with the same flag. BMS never converts a robot 409/410/422/503 into 200.

Do not add mandatory `y_axis` fields to strict v1 models.

### 14.2 BMS ownership

BMS remains a thin relay and viewer. It does not:

- own the physical queue;
- infer reference state;
- reconcile discrepancy;
- choose home speed;
- synthesize child receipts;
- hold a connection lock for physical execution;
- resend an ambiguous command.

### 14.3 BMS files

Primary API seams:

```text
platform/api/services/bioxp/operator_models.py
platform/api/services/bioxp/robot_client.py
platform/api/services/bioxp/connection.py
platform/api/routers/bioxp/operator_controls.py
```

Primary frontend seams:

```text
platform/frontend/src/lib/bioxpClient.ts
platform/frontend/src/components/BioXpCockpit.tsx
platform/frontend/src/components/BioXpQuickDashboard.tsx
platform/frontend/src/components/BioXpOperatorControlTabs.tsx
```

### 14.4 Connection lease, lock order, and timeouts

Keep `_transition_lock` for connect, disconnect, client replacement, generation transitions, and lease-count mutation. Each client generation has:

```text
generation
state=OPEN | DRAINING | CLOSED
lease_count
client_reference
zero_lease_event
```

Lease acquisition takes `_transition_lock`, requires the current generation to be `OPEN`, increments its lease count, copies the client reference, and releases the lock before network I/O. Lease release takes `_transition_lock`, decrements the matching generation, signals `zero_lease_event` at zero, and releases the lock.

Disconnect or rebind takes `_transition_lock`, changes the old generation to `DRAINING`, removes it from new-lease selection, and installs the new open generation when applicable. It then releases `_transition_lock` before waiting for the old generation's zero-lease event. At zero, it closes the old client and marks it `CLOSED`. A draining generation accepts no new call. An existing leased enqueue, query, or interrupt may finish against its retained client.

Lock order is fixed:

```text
_transition_lock only for lease bookkeeping
then release it
then acquire one request owner: v1_workflow | v2_enqueue | v2_query | interrupt
then perform robot I/O under the retained lease
release request owner
release lease
```

No request owner can acquire `_transition_lock` while held. Disconnect never waits for lease zero while holding `_transition_lock`. `interrupt` serializes only interrupt requests and does not wait on `v1_workflow` or `v2_enqueue`.

Request lane and timeout selection uses the catalog action schema before invocation:

```text
legacy v1 normal/workflow action: existing workflow lane and existing 900 s timeout
v2 normal action or method admission: v2_enqueue lane, 5 s timeout
v2 receipt/history/dashboard/catalog/method query: v2_query lane, 5 s timeout
v2 oem.y.stop or oem.abort_all: interrupt lane, 10 s timeout
```

Do not change the common v1 timeout or route unrelated X/Z actions through a v2 lane. Do not retain a lease across robot physical execution after a v2 202 response. Polling acquires a fresh generation-bound query lease per request.

Extend the safety-interrupt allowlist with `oem.y.stop` and source-shaped `oem.abort_all`. Do not add an unnamed aggregate hardening action in this slice.

### 14.5 Cockpit

Primary Y controls invoke `oem.y.*` directly. Generic path lookup remains only for unrelated legacy actions.

The UI must:

- render Y authority from `dashboard.y_axis`;
- use the selected range `0..102956`;
- distinguish manual-panel and diagnostic home;
- submit normal commands without waiting for prior physical completion;
- render immediate local pending state, then bind the robot command ID;
- poll `GET /operator/actions/receipts/{command_id}` every 500 ms while that command is nonterminal, then stop polling it;
- keep STOP on a separate mutation owner;
- leave unrelated controls responsive;
- show requested, effective target, observed, discrepancy, and completion class;
- keep `physical_effect_verified=false` until independent evidence exists;
- remove the current “waiting for terminal receipt” behavior from the enqueue mutation.

### 14.6 Consumer-first rollout

1. Deploy BMS expand-only consumers that accept old v1 and new v2 responses.
2. While connected to v1, keep canonical Y controls unavailable. Do not infer missing Y authority.
3. Deploy the robot v2 producer after its software gates pass.
4. Confirm BMS parses v2 and shows command-ID status.
5. Enable canonical Y controls only from robot-published v2 action and dashboard authority.
6. Remove the temporary v1 compatibility branch after the approved rollback window.

This order prevents strict-model HTTP 502 responses during cutover.

---

## 15. Current gap ledger

Status at the frozen robot and BMS commits:

| ID | Required capability | Current status | Required closure |
|---|---|---|---|
| Y01 | Board 4, motor 0 identity | Present | Keep selected-machine binding and exact `resetXYLimits` reload behavior. |
| Y02 | Exact relative range and layered returns | Partial | Enforce both 20-step end bands and retain primitive/public results. |
| Y03 | Blocking absolute mode | Wrong owner | Route through Y provider and v2 receipt. |
| Y04 | Nonblocking absolute mode | Wrong owner | Add issued-pending lifecycle and later status. |
| Y05 | Manual-panel home speed 500 | Wrong owner | Add distinct provider action. |
| Y06 | Diagnostic `HomeAxis` speed 250 | Partial | Keep separate from manual action. |
| Y07 | Startup Y order | Present | Bind its result to Y authority. |
| Y08 | Internal startup `setHome` | Wrong owner | Keep internal and generation-bound. |
| Y09 | Y STOP lifecycle | Partial | Existing double MST needs Y interrupt/queue/receipt ownership. |
| Y10 | Validity-tagged Y status | Partial | Add reply validity and provider projection. |
| Y11 | Exact Y profile setters | Partial | Remove dead metadata, remove no-motion standby-current writes, and preserve source defaults/returns. |
| Y12 | `enableXY` mode | Partial | Preserve aggregate source sequence internally. |
| Y13 | `enableXYZ` mode | Partial | Preserve aggregate source sequence internally. |
| Y14 | `enableYZ` mode | Absent | Add the exact internal aggregate mode. |
| Y15 | `moveXY` parent/children | Wrong owner | Split X/Y authority and child receipts. |
| Y16 | `HomeXY` parent/children | Wrong owner | Update each reference independently. |
| Y17 | Existing pathing callers | Wrong owner | Route Y leaves through provider without claiming absent workflows. |
| Y18 | Park local 100-step policy | Wrong owner | Keep local to park workflow. |
| Y19 | Self-test local 100-step policy | Wrong owner | Keep local to self-test workflow. |
| Y20 | Board-test caller modes | Partial | Preserve diagnostic source modes without routine exposure. |
| Y21 | Durable normal queue | Absent | Add SQLite journal and resource scheduler. |
| Y22 | Y and XY receipt streams | Absent | Extend SQLite stream/schema. |
| Y23 | Y axis authority row | Absent | Add canonical board/motor authority. |
| Y24 | Board-4 logical epoch | Partial | Replace Z-only observation with shared board authority. |
| Y25 | Y/Z independent invalidation | Wrong owner | Axis-local updates and interrupts. |
| Y26 | Discrepancy reconciliation | Absent | Store requested/effective/observed and advance to observed. |
| Y27 | Restart/idempotency/CAS | Absent | Add no-replay ambiguity and stale-completion rejection. |
| Y28 | Typed Y routes/catalog/dashboard | Absent | Add v2 robot surface and retire duplicate mutations. |
| Y29 | BMS typed Y relay and UI | Absent | Add v2 models, polling, safety lane, cockpit projection. |
| Y30 | Controller and physical acceptance | Unproven | Separate authorized gates. |

Frozen denominator totals:

```text
present=2
partial=9
wrong_owner=10
absent=8
unproven=1
```

No current `Y_SINGLE_AXIS_OEM_PARITY` or `XY_COMPOSITE_OEM_PARITY` claim is supportable.

---

## 16. Work packages

Each work package needs separate execution approval. Each source package uses test-first implementation only after Christian authorizes tests.

### WP0. Freeze the source contract

**Modify:**

```text
docs/specs/2026-08-20-serial206-y-axis-oem-parity-spec.md
```

**Create after approval:**

```text
docs/specs/2026-08-20-serial206-y-axis-oem-parity-source-lock.json
```

Record exact source files, hashes, inclusive method lines, selected config path/hash, caller classification, and intentional deviations.

**Gate:** deterministic verifier passes without controller access.

### WP1. Add canonical SQLite authority and migration

**Modify:**

```text
src/bioxp/oem_runtime_store.py
src/bioxp/operator_receipt_store.py
src/bioxp/services/reference_service.py
```

**Create or modify tests after approval:**

```text
tests/test_oem_runtime_store.py
tests/test_y_axis_sqlite_authority.py
```

Add board, axis, method, and command tables. Import existing compact state once. Make JSON a derived projection. Preserve the existing database and runtime root.

**Gate:** migration, restart, projection rebuild, CAS, retention, and rollback fixtures pass.

### WP2. Add board-4 authority and independent motor states

**Modify:**

```text
src/bioxp/oem_serial206_initialization.py
src/bioxp/usb_driver.py
```

Add board state, ACK-qualified epoch transitions, Y/Z/gripper membership, independent axis rows, and current authority projections.

**Gate:** ordinary Y and Z actions never alter sibling authority; command-64 and ambiguous transition cases match section 10.

### WP3. Close Y primitives and source modes

**Modify:**

```text
src/bioxp/usb_driver.py
src/bioxp/oem_serial206_initialization.py
```

**Create after approval:**

```text
tests/test_y_axis_oem_parity_contract.py
tests/test_y_axis_provider.py
```

Implement exact profile, relative, absolute, completion, manual home, diagnostic home, startup binding, STOP, validity tagging, and discrepancy reconciliation.

**Gate:** every raw/public return and source branch is fixture-constrained. Low-level controls remain internal.

### WP4. Repair XY authority

**Modify:**

```text
src/bioxp/oem_serial206_initialization.py
src/bioxp/usb_driver.py
```

**Create after approval:**

```text
tests/test_xy_composite_authority.py
```

Add parent/child receipts, independent reference updates, source-defined overlap, local interruption, and restoration hardening labels.

**Gate:** Y STOP preserves active or queued X/Z/gripper work and stale child completion cannot overwrite STOP.

### WP5. Add durable command admission and resource scheduling

**Modify:**

```text
src/bioxp/operator_controls.py
src/bioxp/operator_receipt_store.py
src/bioxp/oem_runtime_store.py
```

**Create after approval:**

```text
tests/test_y_axis_command_journal.py
tests/test_oem_resource_scheduler.py
tests/test_oem_interrupt_races.py
```

Implement compact admission, method groups, resource conflicts, dispatch CAS, restart rules, normal idempotency, nonreplayable STOP attempts, and latency instrumentation.

**Gate:** queue and interrupt race matrix passes. No depth-eight axis rejection remains.

### WP6. Add robot v2 API and retire duplicate Y mutations

**Modify:**

```text
src/bioxp/api.py
src/bioxp/operator_controls.py
src/bioxp/oem_runtime_store.py
```

**Create or modify after approval:**

```text
tests/test_y_axis_routes.py
tests/test_operator_dashboard.py
tests/test_operator_controls.py
```

Publish Y actions, dashboard v2, receipts v2, command polling, method endpoints, and canonical compatibility redirects.

**Gate:** every Y mutation reaches one provider and one authority store.

### WP7. Add BMS expand-only v2 consumers

**Modify:**

```text
platform/api/services/bioxp/operator_models.py
platform/api/services/bioxp/robot_client.py
platform/api/services/bioxp/connection.py
platform/api/routers/bioxp/operator_controls.py
```

**Modify tests after approval:**

```text
platform/api/tests/test_bioxp_operator_controls.py
platform/api/tests/test_bioxp_connection.py
```

Accept v1 and v2 during cutover. Add command-ID query relay, short enqueue timeout, client leasing, and Y safety interrupt routing.

**Gate:** old producer stays readable; Y controls remain unavailable until v2 authority exists.

### WP8. Add BMS Y cockpit

**Modify:**

```text
platform/frontend/src/lib/bioxpClient.ts
platform/frontend/src/components/BioXpCockpit.tsx
platform/frontend/src/components/BioXpQuickDashboard.tsx
platform/frontend/src/components/BioXpOperatorControlTabs.tsx
```

**Modify or create tests after approval:**

```text
platform/frontend/tests/vitest/bioxpCockpitAdmissionFanoutMounted.test.tsx
platform/frontend/tests/vitest/bioxpReceiptTerminality.test.ts
platform/frontend/tests/bioxpControlSurfaceCompliance.test.ts
```

Render Y authority and independent command rows. Remove Y generic-path execution. Keep STOP responsive.

**Gate:** request-to-command-ID-to-terminal-receipt path is visible with no global mutation blocker.

### WP9. Software release

Requires separate approval.

1. freeze exact robot and BMS candidates;
2. run approved test and static gates;
3. deploy BMS expand-only consumers;
4. deploy robot v2 software;
5. activate BMS v2 Y rendering;
6. prove source SHA, process owner, database path, route schema, and browser origin;
7. remove temporary v1 compatibility after its approved window.

No controller write or motion is part of software release.

### WP10. Controller non-motion acceptance

Requires separate approval.

Allowed scope can include:

- transport ownership and selected-machine identity;
- read-only board/motor identity;
- no-motion Y profile writes and readbacks;
- event-router observation without movement;
- database and receipt proof.

No home or travel is implied.

### WP11. Physical commissioning

Requires a new, explicit physical-motion command from Christian. Commission only the claim lane authorized in that command.

The acceptance denominator is exact:

| Mode ID | Source mode | Software fixture | Y controller/physical claim | XY controller/physical claim | Classification |
|---|---|---:|---:|---:|---|
| M01 | blocking `moveSteps(Y)` | Required | Required | No | standalone Y |
| M02 | manual-panel nonblocking `moveY` | Required | Required | No | standalone Y |
| M03 | default blocking `moveY` wrapper | Required | Required | No | current internal caller leaf |
| M04 | acceleration-overload `moveY` | Required | Required for one blocking and one nonblocking call | No | current internal caller leaf |
| H01 | startup `axisSearchHome(Y,250)` plus final `setHome(Y)` | Required | Required | No | startup Y |
| H02 | diagnostic `HomeAxis("y")` at 250 | Required | Required | No | diagnostic Y |
| H03 | manual `goHome(true,Y,500,true)` | Required | Required | No | routine Y |
| H04 | `HomeXY` Y leaf at 200 | Required | No | Required | composite XY |
| H05 | location-recovery `goHome(true,Y,1800,true)` | Required | No | No | workflow excluded by section 2.3; fixture-only |
| H06 | board-test Y rehome at 200 | Required | No | No | workflow excluded by section 2.3; fixture-only |
| X01 | `moveXY(int,int)` Y child | Required | No | Required | composite XY |
| S01 | addressed Y double MST STOP during a controlled Y command | Required | Required | No | standalone Y interrupt |
| C01 | `enableXY` current sequence | Required | controller non-motion gate | No | internal aggregate |
| C02 | `enableXYZ` current sequence | Required | controller non-motion gate | No | internal aggregate |
| C03 | `enableYZ` current sequence | Required | controller non-motion gate | No | internal aggregate |

`Y_CONTROLLER_PARITY` and `Y_PHYSICAL_PARITY` require M01-M04, H01-H03, and S01. C01-C03 must already pass the separately authorized controller non-motion gate. `XY_COMPOSITE_CONTROLLER_PARITY` and `XY_COMPOSITE_PHYSICAL_PARITY` separately require H04 and X01. Those XY claims never borrow evidence from standalone Y.

H05 and H06 remain explicit workflow gaps. Their absence cannot be hidden by a broad Y claim and does not block the scoped standalone Y or XY claims.

Every commissioned mode requires:

```text
fresh robot-local authority
valid controller receipt
position/status change where applicable
matching event or exact source fallback class
independent physical observation
bounded work envelope
no automatic retry
```

A software deployment, API 200/202, counter delta, or event alone does not prove physical Y movement.

---

## 17. Approved test and review matrix

These commands are specifications only. Do not run them without Christian's test approval.

### 17.1 Robot focused tests

```bash
pytest -q \
  tests/test_y_axis_oem_parity_contract.py \
  tests/test_y_axis_provider.py \
  tests/test_xy_composite_authority.py \
  tests/test_y_axis_sqlite_authority.py \
  tests/test_y_axis_command_journal.py \
  tests/test_oem_resource_scheduler.py \
  tests/test_oem_interrupt_races.py \
  tests/test_y_axis_routes.py
```

### 17.2 Required negative cases

```text
wrong board or motor
wrong selected config hash
relative target below 20
relative target above 102936
absolute low and high clamp
near-high no-op
null/cached position reply
null left-switch reply that source interprets as active
null `setHome` reply and negative non-null `setHome` reply
foreign event 128
missing event plus target-equal source fallback
missing event plus target mismatch
relative event timeout and layered wrapper return
stall 130 for Y and foreign motor
manual versus diagnostic home speed
home raw return without home proof
home search exceeding selected maximum containment
profile restoration exception
moveXY STA/MTA timeout with source log-only return
HomeXY missing-board null return
Y discrepancy without latch
Y failure with coherent stopped actual position
Y failure with unknown position
ordinary Y action preserving Z/gripper
command-64 negative and ambiguous reply
transport-owner replacement and process restart
normal idempotency race
STOP repeated key produces two physical attempts
STOP during active XY
stale completion after STOP
crash before dispatch
crash after dispatch
crash after terminal SQLite commit
queued restart re-admission
SQLite contention and STOP fallback
corrupt or duplicate STOP fallback import
STOP fallback import before scheduler startup
migration v1-to-v2 row-count and primary-key digest preservation
migration failure atomicity
future user_version refusal without schema write
legacy executing-state migration refusal
v1 JSON projection cannot overwrite SQL authority
strict v1/v2 discriminated models and extra-field rejection
complete receipt status terminality mapping
BMS lease release during disconnect and rebind
v1 900-second timeout remains unchanged while v2 uses selected short lanes
post-boundary rollback retains v2 evidence and disables mutation
more than eight normal commands accepted without a command-count cap
```

### 17.3 BMS tests

```bash
cd platform/api
pytest -q tests/test_bioxp_operator_controls.py tests/test_bioxp_connection.py

cd ../frontend
pnpm exec vitest run --config vitest.config.ts \
  tests/vitest/bioxpCockpitAdmissionFanoutMounted.test.tsx \
  tests/vitest/bioxpReceiptTerminality.test.ts
pnpm exec tsc -b --pretty false
```

### 17.4 Independent review

A final candidate requires:

- source-contract reviewer;
- robot authority and persistence reviewer;
- queue/interrupt race reviewer;
- BMS producer-consumer reviewer;
- exact final bytes and target-scoped Git status.

A reviewer verdict against an obsolete SHA cannot approve a later candidate.

---

## 18. Deployment, rollback, and truth claims

### 18.1 Software deployment

Software deployment proves only:

```text
reviewed source selected
managed process uses that source
schema migrated
routes and models load
BMS parses the producer
```

It does not prove controller preparation, home, movement, or physical position.

### 18.2 Rollback

Before migration or deployment:

- quiesce mutation admission;
- create and verify the accepted SQLite backup;
- snapshot lifecycle/reference projections and their hashes;
- record source SHA, tree, unit, process, database identity, schema version, and v1 row counts;
- stage a BMS expand-only build that reads v1 and v2 while hiding v2 Y controls;
- stage the v2 robot build with `SERIAL206_Y_V2_MUTATIONS=0`, which preserves v2 reads, fallback import, and recovery while disabling new v2 mutations.

The irreversible boundary is the first committed v2 command, method, board transition, axis-authority mutation, or imported interrupt fallback.

#### Rollback A: no v2 authority or command row committed

1. stop BMS Y-control publication and robot mutation admission;
2. prove the v2 tables contain no authority mutation, command, method, or fallback-import row;
3. stop the robot API under the separately approved service procedure;
4. archive the attempted v2 database and logs;
5. restore the verified `user_version=1` SQLite backup and exact JSON snapshots;
6. deploy the frozen v1 robot source and the BMS expand-only consumer;
7. start the service and verify schema version 1, v1 row counts, projection hashes, process/source/database identity, and unavailable v2 Y actions.

#### Rollback B: at or after the irreversible boundary

1. disable all new v2 normal admissions and hide BMS Y controls;
2. pause scheduler dispatch without sending STOP or other controller writes;
3. allow already active handlers to record a terminal result only within their existing source timeout; mark every remaining active child `ambiguous` in one transaction and clear undispatched dependent children;
4. import and verify every interrupt fallback before any restart;
5. create a verified post-boundary SQLite backup plus immutable receipt/artifact manifest;
6. deploy the staged v2 robot build with `SERIAL206_Y_V2_MUTATIONS=0`; retain schema v2, fallback import, dashboard, history, and receipt reads;
7. keep the BMS expand-only v2-capable consumer and confirm all mutation controls remain unavailable;
8. verify `user_version=2`, migration row, command/method/receipt counts, ambiguous rows, fallback hashes, authority rows, source/process/database identity, and zero dispatchable Y rows.

Do not restore the pre-v2 database after the irreversible boundary. Do not start the frozen v1 binary against schema v2. A later return to v1 mutation code requires a separately reviewed lossless evidence export, isolated database migration, and controller-authority reconciliation. Restoring software never restores physical coordinates.

### 18.3 Claim ladder

| Claim | Minimum evidence |
|---|---|
| `Y_PROVIDER_SOFTWARE_PARITY` | Frozen source lock; all M01-M04, H01-H06, S01, and C01-C03 fixtures; reviewed implementation; approved deterministic tests; one canonical Y authority. |
| `XY_COMPOSITE_SOFTWARE_PARITY` | H04 and X01 parent/child order, independent references, interrupt races, and restart recovery. |
| `Y_CONTROLLER_PARITY` | M01-M04, H01-H03, and S01 real-controller delivery, accepted replies, event/source fallback class, valid readback, durable receipt; C01-C03 controller non-motion gate. |
| `XY_COMPOSITE_CONTROLLER_PARITY` | H04 and X01 real-controller parent/child evidence without borrowing standalone Y results. |
| `Y_PHYSICAL_PARITY` | Y controller parity plus independent physical observation for M01-M04, H01-H03, and S01. |
| `XY_COMPOSITE_PHYSICAL_PARITY` | XY controller parity plus independent physical observation for H04 and X01. |
| `LIVE_Y_PARITY` | Deployed source/process/database/BMS identity plus Y controller and physical gates. |

Do not collapse these labels into “100%” before every required layer passes.

---

## 19. Definition of done

### 19.1 Specification done

This document is done when:

- all reviewer findings are reconciled;
- OEM facts and Linux replacements are classified;
- the 30-row gap denominator is complete;
- exact source, robot, and BMS identities are frozen;
- work packages, files, gates, and claim boundaries are explicit;
- the rejected candidate backup remains verified;
- only this specification changed.

### 19.2 Software done

Software is done only after separately approved implementation and tests prove:

- one provider owns all Y mutations;
- one canonical board/axis authority governs queue admission;
- ordinary Y and Z work stay independent;
- no drift-only latch exists;
- raw source returns remain visible;
- timeout-target-equality is reported as its own source completion class;
- low-level profile and set-home controls remain internal;
- normal HTTP returns after durable admission;
- STOP bypasses normal scheduling and stale completion;
- BMS consumes v2 without owning physical behavior;
- duplicate Y mutation paths are retired or canonical redirects;
- no unrelated regression is introduced.

### 19.3 Controller done

Controller acceptance requires separately authorized real-board proof for non-motion preparation and each required event/reply path.

### 19.4 Physical done

Physical acceptance requires Christian's explicit authorization, controller evidence, and independent observation. No unobserved or ambiguous move counts as physical parity.

---

## 20. Critical review decision ledger

| Review | Verdict on rejected SHA | Final disposition |
|---|---|---|
| OEM source reviewer | Reject | Added exact wrappers, returns, null behavior, ranges, current modes, caller policies, and source fallback completion. |
| Robot implementation reviewer | Not implemented | Preserved 30-row denominator and exact present/partial/wrong-owner/absent counts. |
| Architecture reviewer | Changes required | Reclassified queue and board epoch, added resource scheduler, SQL authority, CAS, STOP priority, and aggregate-abort truth. |
| BMS reviewer | Changes required | Added v2 wire contract, command-ID polling, client leasing, exact files, and consumer-first rollout. |

Reviewers assessed the rejected hash only. Their verdicts reject that artifact and inform this revision. They do not independently approve this revision.

---

## 21. Current execution status

```text
specification=complete_and_byte_verified
robot_implementation=not_started
bms_implementation=not_started
code_tests=not_run
builds=not_run
deployment=not_performed
service_restart=not_performed
controller_mutation=not_performed
physical_motion=not_performed
```
