# Native checkTips binding (offline)

## Recovered chain

SSD `decompiled_src/BioXPControlLib/ControlLib.cs`:

- `newloadTips` 9473–9592 queries/picks/removes the selected group, sets
  TipLoaded, then calls `checkTips(tiptray, tiplocation)` only without hotel
  and with CameraInstalled + CameraCalibrated. Its local `tiptray` remains 0
  even when pickup selected a different tray. This port preserves that call.
- `checkTips` 9918–10222 returns true early if CheckForStaticTipLoss is false,
  tray type is 200, or tipLocation ordinal >=23 (B12). It writes LED2 off on
  those branches and AllLEDOff in finally.
- Otherwise AdjustCamera uses InspectionItems 15 = **ClungTips**, not
  TipInspection/missingTps. AdjustCamera 1883–1920 writes all three camera LEDs;
  Gain is not consumed. It only changes exposure if not sentinel 1000.
- Relative CI.moveSteps Y=(3198+CameraYOffset−2132 for B), then
  X=(-1066+CameraXOffset), sleep 200ms, capture; relative Y=8528, sleep 200ms,
  second capture. No Z lift, position publication, return move, or retry.
- ClassControlInterface.cs 4165–4203 uses board.moveSteps(wait=true) followed
  by getCurrentPosition. Reused driver methods motor_[xy]_move_relative_strict
  implement this public wrapper; the board-only helper would omit that read.
- CVisionLib/ClassFrameGrabber.cs `clungTps` 3792–4197 uses gray live conversion,
  ROI (20,120,width−40,height/2), or height/4 for column12; threshold 0..thre;
  existing GetBlobs; area 3001..9499; nearest centers x=111,237,363,489,
  y=57,183 (60 for reduced). Encoding is mask*256 + vector size, including
  one initialized vector element. There is no minimum count or -1 rack gate.
- checkTips shifts >>8 and preserves source masks 0xFFFFF0/0xFFFFF5 and the
  peculiar 0x28 C/G mapping. Existing inventory entries make flag false;
  every mapped well is removed even when already absent. Second image is
  retained whenever the accumulated flag is false, even if its own mask is
  empty. Source exceptions are swallowed retaining the current flag;
  AllLEDOff exceptions propagate. False returns invoke the existing caller's
  waste/eject(false), TipLoaded=false, Z80000/X79000 recovery.

## Integration

`OemPipetteSourceBindings.check_tips` now submits the finite
`pipette_check_tips` / `sourceCheckTips` operation. The existing provider owns
motion, shared inspection callbacks, canonical tray reads/publications, and
source-return evidence. The callback synchronizes only committed removals
into the protocol source model. No hardware connection or scheduler is added.

**Composition addition for parent:** call `_bind_deck_cover_inspection(provider)`
before creating the pipette source bindings, as is already done for cover
inspection. That supplies the existing CameraProvider capture and illumination
owner. Ensure captured CheckForStaticTipLoss is preserved in source settings;
if absent there, the shared captured inspection settings are used, then the
proven OEM constructor false (ClassBioXPSettings.cs:1650). This branch does not
modify api.py/operator_controls.py. No independent camera discovery is added.

## Deliberately unchanged behavior / limits

- `ok` means the source child returned, **not** optical verification. Read
  inspection_completed, source_exception, skip_reason and capture evidence.
  A camera exception can leave source_return=true exactly as OEM; making it
  false or refusing continuation requires Christian's approval.
- Likewise, fixing the caller's constant tray0, interpreting 0x28 differently,
  adding Z/return moves, retries, or treating ignored controller return values
  as a new workflow failure requires explicit approval. None is implemented.
- Reuses the current shared inspection profile support: non-1000 exposure is
  unsupported there, recorded through OEM's catch without claiming inspection.
  Real camera exposure/illumination and physical recognition are unqualified.
- Successful frames are not retained, matching OEM's delete-on-true result.
  Failed decision frames use the existing shared artifact saver rather than
  OEM file naming. Save receipt is retained separately; no new save-proof gate.
- Offline tests use synthetic PNG frames at the camera boundary and replaced
  native motion/LED boundaries. Real recognizer, finite compiler/executor,
  provider callback and retained SQLite tray publication remain connected.
  This is not a complete live ldtip or physical pipetting demonstration.

## Verification

Interpreter: `/home/dalab/robot/BioXP-latency-review/.venv/bin/python` with
`PYTHONPATH=src`, `BMS_HOME` set to the worktree, live-test flags unset, and
`DECK_RETAINED_BASELINE=/home/dalab/.hermes/profiles/fresh/cache/scratch/reagent-noop-fixture/state`.
The older reference-guide retained-baseline path no longer exists here.

- `test_pipette_check_tips_connected.py`: **20 passed**, no skips.
- Combined with `test_protocol_oem_pipette_composites.py`,
  `test_protocol_tip_transition_publication.py`, and
  `test_protocol_oem_provider_bindings.py`: **95 passed, 1 failed**.
- The same three existing modules on a pristine git archive of installed
  `a65a1760e81d45dfe8b74811e9b5c51954af2303`: **75 passed, 1 failed**, same
  `test_current_stall_and_rgb_source_contract` failure (expects one RGB write,
  source implementation writes twice). No unrelated expectation was changed.
- `test_oem_prepare_cv.py` could not collect without its external
  `OEM_CV_EVIDENCE` manifest. It is not claimed as qualified by this run.
- `git diff --check` passed. Nothing deployed, no robot/device contact.
