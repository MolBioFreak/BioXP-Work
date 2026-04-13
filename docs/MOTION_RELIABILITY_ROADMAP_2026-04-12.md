# BioXP3200 Motion Reliability Roadmap (2026-04-12)

Goal
- Turn the current BioXP3200 bring-up state into a reliable, operator-safe motion stack that proves real movement before we expand into higher-level automation.

Working assumptions from current state
- Non-homing strict startup can pass.
- 24V, door, latch, and solenoid conditions can all be healthy at the same time.
- Very small supervised moves have produced controller-side telemetry deltas.
- BMS/operator telemetry is not independent proof of mechanical displacement.
- The machine/UI can still overload, so stability and proof collection come before feature growth.

Critical code surfaces
- Robot runtime API: `/home/dalab/Desktop/bioxp_re/src/bioxp/api.py`
- Robot USB driver: `/home/dalab/Desktop/bioxp_re/src/bioxp/usb_driver.py`
- BMS proxy router: `/home/dalab/biomodstack/biomodstack/platform/api/routers/bioxp.py`
- BMS client hooks: `/home/dalab/biomodstack/biomodstack/platform/frontend/src/lib/bioxpClient.ts`
- BMS operator cockpit: `/home/dalab/biomodstack/biomodstack/platform/frontend/src/components/BioXpCockpit.tsx`
- OEM/reverse-engineering reference: `/home/dalab/Desktop/BioXP_SSD_Backup`

Important current findings that shape the plan
1. The operator-facing BMS path is already using fresh prep for hold-jog style moves.
   - `BioXpCockpit.tsx` currently sends `reuse_prepared: false`.
   - `bioxpClient.ts` defaults `reuse_prepared = false` for relative moves.
2. The robot runtime still contains a risky prepared fast path.
   - In `src/bioxp/api.py`, `_prepare_motion_axis(...)` can skip interlock wake, board activation, and axis prep when `reuse_prepared=True` and arm/live state is already good.
   - This is exactly the kind of path that can create false confidence when telemetry moves but physical motion remains uncertain.
3. The runtime is explicitly controller-centric.
   - `motor_get_position()` / `motor_get_speed()` are controller reads.
   - Motion prep disables encoder deviation checks, so controller deltas are useful but not independent proof of real displacement.
4. There is no obvious automated test suite in the standalone BioXP repo today.
   - That means each phase needs explicit manual verification gates and artifact capture.

Non-goals until the roadmap gates are cleared
- No protocol-level liquid handling workflows
- No broad multi-axis automation
- No aggressive homing/referencing sequences without supervised single-axis evidence
- No reliance on BMS polling alone as proof of motion

Phase 0 — Freeze the safe operating baseline
Objective
- Make sure the known-safe path is the only normal path while we debug reality versus telemetry.

Concrete actions
- Treat `reuse_prepared=false` as the only approved operator path for supervised moves.
- Audit all BMS callers of `/api/bioxp/motion/axis/relative` and keep the default fresh-prep behavior.
- Document the current robot-side prepared fast path as experimental / unsafe-for-proof.
- Define a single artifact location for motion validation bundles.

Recommended artifact path
- `/mnt/BioModStack/bms_results/bioxp_validation/<timestamp>/`

Acceptance gate
- There is one clearly documented safe path for supervised motion.
- Every live probe is tied to a saved artifact bundle path.

Phase 1 — Build a motion proof harness
Objective
- Convert one-off supervised probes into repeatable proof bundles.

Why this matters
- Right now we can show controller deltas.
- What we need is repeatable evidence that separates:
  - controller-reported motion
  - camera-visible change
  - human-observed physical movement

Concrete actions
- Add a simple supervised-probe workflow that always captures:
  - pre-move arm/latch/24V state
  - pre-move axis status
  - before snapshot image
  - exact motion request JSON
  - raw response JSON
  - after snapshot image
  - post-move arm/latch/24V state
  - operator note: saw motion / heard motion / no independent confirmation
- Start with canned probes only:
  - Z: `-500`
  - Z: `-2000`
  - X: `+1000`
- Save every run under the artifact path above.

Implementation surfaces
- Robot side can stay unchanged for the first pass.
- The quickest place to orchestrate/save proof bundles is probably the BMS proxy/router layer plus operator cockpit.
- Likely touchpoints:
  - `/home/dalab/biomodstack/biomodstack/platform/api/routers/bioxp.py`
  - `/home/dalab/biomodstack/biomodstack/platform/frontend/src/components/BioXpCockpit.tsx`

Acceptance gate
- For each approved probe, collect at least 3 supervised runs with:
  - raw telemetry
  - paired images
  - explicit operator note
- If the camera is too poor to confirm motion, say that plainly in the artifact and treat the camera as secondary evidence only.

Phase 2 — Harden the robot-side motion prep path
Objective
- Eliminate or severely constrain the prepared fast path that can bypass important bring-up work.

Concrete actions
- Patch `/home/dalab/Desktop/bioxp_re/src/bioxp/api.py` in `_prepare_motion_axis(...)` so that:
  - strict-arm/live reuse may skip only the interlock wake if desired,
  - but board activation is never skipped,
  - and `tester.motor_prepare_axis(...)` is never skipped for supervised operator moves.
- Strongest near-term option:
  - keep the request field for compatibility,
  - but ignore `reuse_prepared=True` unless an explicit debug-only env flag is set.
- Preserve response metadata so every motion response states whether any reuse happened.

Concrete current risk location
- `src/bioxp/api.py:610-665`
  - this block currently allows `prep = {"reused": True, ...}` and a synthetic reused `board_status` when `reuse_prepared=True`.

Acceptance gate
- On supervised motion runs, responses no longer report reused prep/board activation.
- Fresh-prep supervised moves remain stable across repeated daemon restarts and re-arms.

Phase 3 — Clean up truth semantics in the API and UI
Objective
- Prevent the operator from confusing controller truth with physical truth.

Concrete actions
- Rename or explicitly label motion readouts as controller-reported state where appropriate.
- Surface when motion proof is only telemetry-backed.
- Add explicit “desynced after manual reposition” state to the operator workflow.
- Add a warning banner in the cockpit when the system has not been re-referenced since any manual disturbance.

Implementation surfaces
- `/home/dalab/Desktop/bioxp_re/src/bioxp/api.py`
- `/home/dalab/Desktop/bioxp_re/src/bioxp/usb_driver.py`
- `/home/dalab/biomodstack/biomodstack/platform/frontend/src/lib/bioxpClient.ts`
- `/home/dalab/biomodstack/biomodstack/platform/frontend/src/components/BioXpCockpit.tsx`

Acceptance gate
- A human operator can tell, from the UI alone, whether a “successful” move means:
  - command accepted,
  - controller delta observed,
  - or independently confirmed physical displacement.

Phase 4 — Establish a controlled re-reference / homing sequence
Objective
- Move from “single supervised probes” to a repeatable start-of-day recovery/reference procedure.

Concrete actions
- Define the axis order explicitly; likely start with Z clearance behavior first, then a cautious horizontal axis.
- Record actual switch semantics and permitted directions at each step.
- Add abort conditions for every homing/reference stage.
- Keep the process single-axis and supervised until the reference procedure is repeatable.

Do not do yet
- No automatic full-machine homing macro
- No multi-axis choreography
- No unattended recovery

Acceptance gate
- The machine can be brought from cold/restarted state into a known reference state using a documented single-axis supervised procedure.

Phase 5 — Harden the BMS operator surface
Objective
- Make the safe path easy and the misleading path hard.

Concrete actions
- Add a dedicated “Supervised Probe” section in `BioXpCockpit.tsx` with canned safe moves only.
- Show preconditions inline:
  - strict init pass
  - 24V OK
  - latch/door permissive
  - speed zero
- Save artifact bundles automatically.
- Add explicit status badges such as:
  - `controller-only`
  - `paired-images-captured`
  - `human-confirmed`
  - `manual-disturbance-desynced`
- Reduce UI overload by keeping polling conservative outside active commands.

Acceptance gate
- An operator can validate the machine from BMS without touching raw endpoints and without mistaking polling data for proof.

Phase 6 — Only then evaluate OEM-parity / repurposing extensions
Objective
- Decide how far to push beyond motion bring-up once motion/reference are truly reliable.

Possible longer-term branches
A. Continue strengthening the Linux runtime
- Add better state machines, proof artifacts, and axis/reference logic directly in the Python stack.

B. Mine OEM stack for higher-value missing capabilities
- Vision-assisted calibration
- barcode/traceability hooks
- volumetric/meniscus logic
- calibration compensation ideas

C. Native-wrapper path
- Use the managed OEM assemblies as reference or wrapper targets only after the motion/reference layer is stable enough to compare behavior meaningfully.

Acceptance gate
- Choose a longer-term architecture only after the bring-up stack has repeatable real-world motion proof and a safe reference procedure.

Immediate next sprint (what to do next, in order)
1. Robot-side lockdown
- Patch `_prepare_motion_axis(...)` so supervised moves never skip board activation or axis prep.

2. Restart and supervised repro
- Restart the robot daemon.
- Re-run the current minimal probes with fresh prep only.

3. Proof-bundle capture
- Save before/after images plus raw JSON for each probe under `/mnt/BioModStack/bms_results/bioxp_validation/...`.

4. Operator labeling
- Add UI labels that distinguish controller-reported movement from independently confirmed physical movement.

5. Re-reference design
- After repeated stable single-axis probes, draft the first guarded re-reference/homing procedure.

Decision rule for the whole roadmap
- If a step improves telemetry but not proof, it is not enough.
- If a step increases automation while reducing confidence in real motion, it is the wrong next step.
- Reliability, traceability, and physical verification beat feature breadth until motion truth is solved.
