# BioXP 3200 #206: OEM pipette completion specification

## 1. Answer and scope

For the gaps identified in this tranche, the OEM source is already recovered. We are missing parts of its **application behavior and operator integration**, not a new pipette driver or an undiscovered CAN protocol.

The target is an operator-usable mechanism for source-supported channel/tip selection, specific-well pipetting, tip handling, air/liquid operations, mixing, diagnostics, fluid detection and pipette calibration. An available primitive does not establish a complete OEM caller. Source-composed diagnostic functions are required, not optional.

This document replaces the earlier two-item remaining-work assessment. It updates current candidate disposition, not the pinned OEM source facts in `2026-08-02-pipette-oem-gap-rectification-spec.md`. Historical release gates in that document and `2026-08-16-pipette-oem-parity-execution-plan.md` do not authorize new refusals. Existing OEM/controller interlocks remain intact. Missing observations, records or physical proof must not become new admission or success conditions without Christian's exact approval; default NO.

This is a specification-only change. It does not authorize implementation, deployment, initialization, motion, liquid operations, active geometry replacement or a new concurrent-write policy. The completion evidence below is for evaluating the delivered work, not for adding runtime gates.

## 2. Reviewed candidates and source notation

- Robot: branch `feat/oem-well-pipetting`, reviewed implementation `329c7a8f735767d3d2e79e0ecdccaf7aa86b552a`.
- BMS: branch `integrate/pipette-bms-dev-20260925`, reviewed implementation `8c46d8de2a3f889e74e7d467080de3675880c289`.
- These are local candidates. Neither deployment nor physical qualification is established by this specification.
- `R:` means a path in this robot repository.
- `B:` means a path in the BMS candidate repository.
- `OEM:` is `/home/dalab/Desktop/ROBOT/BioXP 3200 Development Work/BioXP_SSD_Backup`.
- `ControlLib` and `Collection` below mean `OEM:decompiled_src/BioXPControlLib/ControlLib.cs` and `ClassPipetteCollection.cs`.
- `Settings` and `MachineStatus` mean `OEM:decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPSettings.cs` and `ClassMachineStatus.cs`.

Source line anchors describe these reviewed snapshots. Follow the named methods if subsequent edits move port lines. The companion `2026-09-25-pipette-diagnostic-control-map.md` inventories the original pipette diagnostic UI.

## 3. Keep and reuse

The candidate already supplies:

- Native named-location/well Move and calibrated in-place Lower/Lift, explicit-channel Aspirate/Dispense, repeated-stroke manual Mix and ordered programs through the existing BMS protocol relay and robot workflow owner.
- Physical manual tray/well `load_tip`, with overpress and optional lift; one-well `measure_fluid_height` at the current pose.
- Scripted `ldtip`, source `newloadTips`/`pressload` behavior, `KeepTip`, inventory publication, `mmix`, source air/purge/liquid procedures, ejection and shared-camera `checkTips` mechanisms. They are not all interchangeable with the narrower manual buttons.
- Source `loadTips(T50, forcenewtip:true)`, repeated `zOffset`, separate diagnostic Detect Fluid and calibration `calwithFluid` compositions under existing finite ownership.
- Software-only T50/T200/UNKNOWN tip-type selection, consumed operation flags, pressure/status/data controls and the advanced typed catalog.
- PositionTable edits, paired current-Z tray Set, per-station calibration persistence, active/saved readback and an atomic saved-revision restore primitive.
- Existing four-channel transport, shared USB owner, canonical worker, source logical model, OperatorCommandStore, PipetteReceiptStore and BMS relay.

No second scheduler, replacement motion controller, generic firmware console or parallel inventory authority is needed. Do not reproduce obsolete Windows logging or retain noncritical diagnostic bulk merely to imitate the OEM UI. Unrelated thermal/chiller/gripper calibration is outside this tranche.

## 4. Remaining work packages

### PIP-C1 — Connect source channel/tip selection to actual well alignment

**Missing:** a complete operator path from selecting a pipette to establishing its source `TipLocation`, then moving that pipette to the addressed source/destination well. Current liquid-channel selection only chooses plungers. Current Move consumes the pre-existing alignment.

**OEM behavior to reuse:**

- `ControlLib.newloadTips`, lines 9473–9592, accepts `pipette=-1` or a selected pipette, uses `approachCorrectTip` and `pressload`, publishes `MachineStatus.TipLocation` after successful loading, and updates tip occupancy.
- `ControlLib.pressload`, lines 9325–9471, includes the selected-tip `KeepTip` behavior, source retry branches and source Home behavior.
- The selected source `TipLocation` subsequently participates in `scriptmoveTo` geometry; an arbitrary UI plunger checkbox does not change it.

**Existing port:** `R:src/bioxp/services/pipette_service.py:1249–1388` (`pressload`, `newload`, `ldtip`), `R:src/bioxp/oem_serial206_initialization.py:11812–11820` and `R:src/bioxp/manual_pipetting.py:38–54,169–178`.

**Required integration:**

1. Expose the existing source-supported channel-selecting load/retain operation through the typed manual authoring path and mounted BMS control, using the same source model, transport and semantic publisher as scripted operation.
2. Preserve the distinction between source `newloadTips`, the calibration helper `loadTips`, manual tray/well Load Tip and host-only tip-type selection. Do not replace all four with one approximate action.
3. Preserve source channel numbering and the all-channel sentinel; label operator channels clearly without silently normalizing indices. A four-head fixed geometry is not four independently positioned XY tools. Express source-supported single/group targeting, not invented independent-well placement for arbitrary subsets.
4. Preserve existing early returns. In particular, `newloadTips` may return early when tip type matches and forced loading is false. Do not relabel `TipLocation` merely because a different channel was requested; any source-required loading/retention must actually execute.
5. Let the operator author an ordered source-to-destination transfer using that established alignment. Keep loading, movement, strokes and cleanup explicit; adding this capability must not silently initialize, eject, Park or run full-job cleanup for existing basic strokes.

**Completion evidence:** mounted typed requests for each source-supported selected channel and group mode; the real native handler, finite executor and both SQLite owners with only hardware transport replaced; actual tip-location publication followed by source/destination motion and strokes. Do not seed matching `tip_location` in the fixture as a substitute for the operator path, as `tests/test_manual_pipetting.py:170–171` currently does. Include existing-tip early return, successful selection, source retry/failure and retained tip state between consecutive requests.

### PIP-C2 — Preserve the OEM distinction between returns, propagated exceptions and suppressed exceptions

**Missing:** faithful exception transport between already-ported source bodies and their composing adapters.

**OEM behavior:** `ControlLib.loadTips:9684–9900` contains genuine throws; `zOffset:3387–3627` ignores certain Boolean/integer child returns but does not catch those exceptions. `catchPlate` logs/suppresses its source exception (`ControlLib:8394–8402`); the diagnostic caller at `1440–1469` continues after that method returns. These are different cases, not one generic `ok` rule.

**Port defects:**

- `R:src/bioxp/oem_serial206_initialization.py:12664–12670` turns every `loadTips` exception into an ordinary unsuccessful envelope. `R:src/bioxp/pipette/oem_fluid_owner.py:120–124` passes it to `oem_fluid_workflows.py:46–49,57,78`, where it is ignored like a legitimate source Boolean.
- `R:src/bioxp/oem_serial206_initialization.py:12706–12724` turns source-suppressed finite catch/release results back into diagnostic exceptions. Their suppression is explicit at `R:src/bioxp/oem_deck_movement.py:1732–1736,1768–1772`.

**Required correction:** retain source exception identity/evidence across the existing adapter and let the appropriate OEM caller catch or propagate it. Continue to ignore the exact returns the source ignores. Keep source-suppressed errors observable without rethrowing them merely because a receipt has `ok:false`. Do not add a blanket false-result gate, retry, cleanup or exception suppression rule. Preserve original OEM/controller Stop and Abort behavior independently.

**Completion evidence:** genuine thrown load failure unwinds before the next scan-body move/stroke; legitimate false `loadTips` return follows source continuation; source-suppressed catch/release follows the original diagnostic continuation; true outer exceptions end the diagnostic as source specifies. Verify the calibration caller's distinct finally behavior separately, including failure during finalization. Run these through the real provider/finite adapter, not only callback-shaped result dictionaries.

### PIP-C3 — Finish the OEM diagnostic callers and their typed controls

**Missing:** source application handlers above available collection/transport primitives. Reuse the current owners and catalog; do not add a separate diagnostics controller or require operator-authored JSON.

Required source compositions:

1. **Aspirate and Dispense.** `Collection:418–531` reconciles cached tipped channels with tip queries before liquid commands. Dispense also removes lost-tip channels from the selected set. Preserve this outer caller behavior for OEM diagnostic actions. Explicit-speed port overloads already set speed (`R:src/bioxp/pipette/transport.py:2830–2889`); do not duplicate that work or silently change the deliberately narrower explicit-stroke API. The OEM Dispense textbox cross-reference at line 523 is a UI bug, not a reason to feed aspirate input into a correctly typed dispense request; record that distinction rather than claiming literal widget parity.
2. **Dispense All.** `Collection:533–555` targets cached `m_PipetteHasTip` channels and performs the group wait. The present `transport.dispense_all` uses supplied/TipLocation-based selection with fresh verification (`transport.py:2030–2058`). Bind the correct caller instead of treating those selections as interchangeable.
3. **Diagnoses.** `Collection:571–582` runs tests 0, 1 and 2 in order and assembles plunger-force, step-loss and pressure-sensor results. `/liquid/diagnoses` currently executes one `number` (`R:src/bioxp/api.py:10795–10805`). Its collection method also uses cached tipped-channel eligibility (`Collection:261–278`); review the port's additional freshness/provenance/generation restrictions (`transport.py:1465–1511,1679–1686`) as a source discrepancy, not as a reason to add more requirements.
4. **Diagnostic Initialize.** `Collection:584–604` calls `initiateGroup`, checks status and conditionally performs the second group/status call. The retry helper already exists (`transport.py:1408–1418`). `/liquid/init` uses constructor lifecycle semantics (`api.py:10541–10603`), which are not this caller. Expose the diagnostic composition without importing the constructor's firmware check or another caller's retry count.
5. **Selected Eject.** `Collection:606–624` queries each channel in source order, then ejects it only when that query reports 1 and the operator selected it. Preserve this exact query/selection behavior. Do not substitute all-tip ejection, kept-tip selection or travel-to-waste without a source call for that action.
6. **Get Data.** `Collection:640–659` obtains `retriveADPInformation(i)` and `getData()` for every channel and assembles part number, revision, firmware and data. Deliver that usable typed result to the operator. Existing `/liquid/data` and `/liquid/firmware` are building blocks. No recreation of `c:\logfile\pipettor-data.txt` is required.
7. **Plunger Up/Down.** `ControlLib:1417–1433` sets Z maximum current to 31, then performs signed relative Z movement. Reuse the existing current and relative-motion primitives in the source order. A bare relative-Z button is not the whole caller; do not invent an extra current-restoration finally.
8. **Last Error.** `Collection:557–569` queries byte 1 on all four channels and displays the four results. The backend already does four-channel `query_error_log` (`transport.py:1674–1677`). Reuse it with the fixed source argument and an appropriate typed operator presentation; this is not a missing four-channel transport implementation.

**Already present, retain:** four-channel pressure (`Collection:626–638`), host-only tip type (`1418–1423`), physical manual Load Tip (`ControlLib:1324–1368`), paired Z Set (`1370–1415`), Move to Waste (`1435–1438`), consumed pressure logging setting (`1471–1477`), and the separate five-station Detect Fluid binding. Existing primitive controls may remain available under accurate names; they must not masquerade as these compositions.

**Completion evidence:** source-ordered child execution with real collection/provider and canonical owners, selected/unselected and tipped/tipless branches, actual source partial failures and retry counts, mounted request/response compatibility, and useful results for each composed action. Source eligibility changes must be explicit; no generic freshness gate should be silently copied into a source-cached branch. Keep OEM/controller interlocks intact and settle any execution-policy change with Christian before implementation.

### PIP-C4 — Complete calibration reset, decision, restoration and runtime semantics

**Missing:** the end of the source calibration interaction, faithful reset/finalization semantics and agreement on the deliberately different activation behavior.

**OEM chain:**

- `ControlLib.calwithFluid:2782–2849` resets status, scans TC/MS/OC/RC/STRIP, calls `adjustZ` and `saveConfig` after each station, then executes its source finally: cached-tip conditional waste/eject, Park, comparison, UI completion and Z acceleration restoration.
- `MachineStatus.resetStatus:655–689` resets occupancy AND tip types/locations: T50/T50/T50/T200/T50 at locations 7/8/9/10/15, plus tray/strip/movable defaults.
- `Settings.adjustZ:6039–6138` updates the live position table and related source settings; the strip adjustment affects the related strip rows. `resultComparison` presents before/after values; its accept/reject section at `6518–6539` saves history or restores calibration and internally catches dialog/history/restore errors.
- Comparison still occurs after a calibration-body failure. Acceptance is not limited to a complete five-station body. When the OEM has no previous calibrated backup, it follows the source no-backup result rather than inventing an operator acceptance.

**Required work:**

1. Correct the reset binding so prepared/altered tip-tray types and locations are reset as well as occupancy. `R:src/bioxp/pipette/oem_fluid_owner.py:224–242` currently reuses `oem_job_preparation.py:521–531`, which retains those metadata. Keep constructor/reset logical declarations distinct from observed physical contents.
2. Retain the exact pre-run calibration and each saved station outcome through the existing durable owners. The operator's before/after comparison must remain recoverable from the run identity rather than only a large transient callback result. Keep critical calibration history separate from noncritical diagnostic logs.
3. Add typed robot/API and mounted BMS acceptance/restoration of that run's values. The existing `CalibrationSettingsService.restore` is the owner to reuse. Cover full run, partial run, prior saved revision and no prior saved revision. An unrelated newer settings write must not be silently overwritten as if it belonged to the compared run; the exact concurrent-write behavior requires Christian's approval, not an invented rejection policy.
4. Match source comparison exception boundaries so its internally handled errors do not skip the caller's subsequent completion/Z acceleration restoration. Preserve real source-finally exceptions where the OEM would propagate them. Do not add a blanket always-clean-up path to earlier failures.
5. Preserve the source adjustment arithmetic, fluid-reference identity and liquid-calibration metadata. Trace derived settings such as `m_PLLow`/`m_current_tool` to their actual consumers rather than assuming a recorded `settings_updates` proposal is an applied setting. Do not silently change scientific calculations or source mixing/calibration quirks.
6. Resolve activation semantics explicitly. OEM `adjustZ` and manual paired-tray Set update the in-process table. The candidate saves values for the next ordinary startup and does not replace the bound snapshot. Full literal runtime equivalence cannot be claimed while this difference remains. Either implement approved owner-scoped source-equivalent application/restoration, or retain and name the approved next-startup divergence. Do not implement live rebinding, restarting, homing or new locks under this document alone.

The current production binding hardwires comparison to `None` at `oem_fluid_owner.py:267–271`. `oem_fluid_callers.py:157–174` additionally conditions acceptance history on body success and lets comparison errors skip later finalization. These are concrete missing/incorrect bindings, not missing OEM source.

**Important current behavior:** a run marked `incomplete` with no comparison choice still saves offsets. The parent review's real settings/SQLite probe confirmed that ordinary startup projection activates them. Neither that label nor `pending_restart` makes them inert. Report the truth; do not add an unapproved acceptance-based startup or motion gate.

**Completion evidence:** real settings/SQLite save, compare, accept and restore through the typed operator path; failure after each station; no-backup case; comparison/history/restore exceptions; source reset from nondefault prepared metadata; ordinary startup readback of whichever revision is actually saved. Separate body completion, operator decision, durable settings effect and active geometry. These are reporting dimensions, not extra physical admission conditions.

### PIP-C5 — Carry usable results through the real dispatcher into BMS

**Missing:** lossless critical result propagation. This is our integration defect, not an OEM method to port.

The parent review passed real connected five-station executor results through the production boundary. Responses of 12,087,972 and 12,087,851 bytes exceeded `_bounded_json(...,131072)` and became only bounding metadata/preview. Required children, measurements and saved revisions disappeared.

**Affected path:**

- `R:src/bioxp/operator_command_plane.py:8459–8463` and `operator_controls.py:710–718,3204–3208`.
- `R:src/bioxp/manual_pipetting.py:271–275` consequently computes `calibration_persisted:false` when child data is absent.
- Failures instead preserve child results under `response.provider_results` (`operator_command_plane.py:8424–8453`).
- `B:platform/frontend/src/components/BioXpWellPipettingPanel.tsx:185–202` only reads completed/source children and does not surface the partial-failure shape or terminal error details.

**Required correction:** keep compact typed operational outcomes independent of diagnostic bounding. Preserve run/action identity, original source outcome, completed stations/samples, measured values, saved revision, actual active/saved distinction, decision state and source/finalization failures. Use the existing canonical record for critical child detail. Delete redundant noncritical nested copies; do not solve this by retaining/hashing all bulk, increasing caps or inventing a new result service. A save flag must reflect the settings write, not the survival of a diagnostic child list.

Both complete and partial-save results must render in BMS. Distinguish source-body completion from operator acceptance and verified physical effect; none is interchangeable with the others.

**Completion evidence:** real handler, queue/dispatcher, finite executor, terminal persistence, workflow action result, BMS relay and mounted consumer, with only hardware transport replaced. Include success, failure after two saves, finalization failure and large repeated scans. `tests/test_oem_calwith_fluid_connected.py:167–175` currently bypasses production bounding, and its failure branch at `150–154` directly invokes the provider; close those boundaries rather than replacing results with hand-authored fixtures.

### PIP-C6 — Close full operator/API coverage without duplicating existing OEM functions

**Missing:** a consolidated, source-correct operator surface across manual workflows, prepared scripts and the advanced catalog. A raw primitive catalog is not the whole application, but its already implemented functions must receive credit.

Retain reachable typed controls and agent/API parity for the existing families:

- Liquid aspiration/dispensing, all-dispense, air aspiration/dispensing, source purge and speed selection.
- Source scientific mixing (`mmix` and collection `mixAll`), separately named from repeated manual strokes.
- Physical loading, source-selected loading/retention, kept-tip behavior, ejection/verification, tip queries, software tip type and source inventory/inspection effects.
- Initialization and reinitialization as distinct source call sites; status, condition, pressure, timestamps, firmware/data, errors and diagnostics.
- Fluid-height detection, repeated offsets, five-station diagnostic, calibration, settings and paired-tray Set.
- Source command termination and heartbeat controls. HTTP task cancellation is not addressed device Stop; do not introduce No24V inhibition, latch bypass or non-OEM cancellation behavior.

Existing routes include `/liquid/keep-tip`, `/liquid/aspirate-air`, `/liquid/dispense-air`, `/liquid/mix-all`, `/liquid/terminate`, `/liquid/heartbeat`, `/liquid/fluid-detection/{channel}/timestamp`, `/liquid/condition`, `/liquid/status/readback`, `/liquid/firmware` and `/liquid/reinitialize` (`R:src/bioxp/api.py:10645–11178`). Existing scripted bodies are in `R:src/bioxp/services/pipette_service.py`. Do not list these as missing backends merely because they are not on the Well pipetting panel.

For each existing source function, the operator needs an appropriate typed input and usable result through the existing supported surface. Reuse the native script adapters for source liquid/mix/purge semantics; do not invoke a dummy full job or make manual `Mix` pretend to be `mmix`. Preserve the source `mixAll` quirk that uses tip type as volume rather than silently correcting it; any scientific behavior change requires Christian's decision.

**Completion evidence:** an updated control map identifies each family, exact source caller, existing implementation, mounted typed access and result consumer. Add only the missing composition/exposure. Exercise exported real UI requests against robot models and pass real producer replies to mounted consumers. No absent observation should disable otherwise valid controls merely to make the map look complete.

## 5. Implementation sequence and decision points

1. Correct PIP-C2 exception semantics and PIP-C5 critical-result propagation in the existing components; both are shared foundations for reliable composition.
2. Connect PIP-C1 selected-tip well targeting and establish the complete operator transfer path.
3. Complete PIP-C3 diagnostic callers and PIP-C4 calibration interaction/reset/finalization.
4. Reconcile PIP-C6 operator/API coverage against the original source families; delete stale optional/missing-backend descriptions rather than accumulating another control layer.
5. Separately deploy the reviewed robot/BMS revisions when authorized and verify the actual served paths. Then perform the authorized bounded physical checks of pickup, selected-well alignment, transfer, height/inspection and calibration. Offline source execution is not physical accuracy.

Before changing activation or concurrent-write behavior, obtain Christian's exact decision. Keep source bugs and approved product differences explicit; a broad desire for OEM equivalence does not authorize silently changing scientific behavior or inventing new host gates. Recording a discrepancy is not an instruction to stop normal commands.

## 6. Evidence carried forward; no new execution in this write-up

The preceding review at the candidate revisions in §2 established:

- Parent robot executions: 145 passed, 11 skipped. The skips require `DECK_RETAINED_BASELINE`; they are not passes.
- Mounted BMS execution: 27 passed. These relay tests use synthetic robot replies and therefore do not establish compatibility with the real oversized response.
- Real transport-replaced connected calibration success/partial-failure execution, plus production bounding and settings-startup probes, exposed the gaps described above.
- Candidate worktrees were clean before this documentation update; no installed-release or physical result was inferred.

This write-up changes only the current completion specification and companion diagnostic control map. Verification for this turn is document/source-reference consistency and diff checks, not another build, test campaign or robot run.
