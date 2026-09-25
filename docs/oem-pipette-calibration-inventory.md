# OEM pipette calibration: implementation and remaining decisions

Scope: offline source reconstruction against the retained BioXP 3200 SSD. No robot contact, live calibration, API/UI exposure, deployment, or configuration write is performed by this change.

## Executable provider bodies

`src/bioxp/pipette/oem_calibration.py` exports:

- `ManualTipLoadRequest(tray, well, overpress=False, lift_z=False)` and `manual_load_tip(request, bindings)`: physically addresses source tray 1..5 and A/B 1..12 tip-group identity. Tray index is resolved through the existing source-model reader, NOT assumed to be a fixed deck location. `moveTo` uses column/row, Tip10=false, highPos=true, runInParallel=false; publishes location/well; lowerPipette optionally adds 4030 steps to PositionTable zLow; queries all tip states; optional MoveZHome(rehome=false) retains lost steps and >100 warning; clears TipDirty. **Source does not remove tip inventory, set TipLoaded, set tip type, or select a channel.** Do not add those as purported parity. Source UI accepts syntactically nonsensical Enum.TryParse numeric/OVER inputs and has odd invalid-text/lift behavior; the typed application request exposes only physically meaningful tip identities, not those text-parser accidents.
- `FluidHeightRequest(speed=300)` and `measure_fluid_height(request, bindings)`: literal scrFluidDetection body at the current source location/well. The parent named-well workflow must position first, not expose a user-facing raw Z target. liftTo(zHigh), AspirateAir(15,false), 500 ms, get XYZ, set Z speed, four BRs, 1 ms, nonblocking Z to zLow+2015 with captured Z_MOTOR_MAX_CURRENT_DOWN, shared 15000 ms completion wait, stop Z, measured XYZ; four-channel time-outlier calculation; read XYZ; restore Z speed 1791; liftTo; DispenseAll. Returns actual source position, per-channel timestamp/delay evidence, threshold and failed pipettes, and nested real child results. Router receive timestamps are monotonic, so elapsed calculations use the same clock, not wall clock.
- `detect_fluid_with_motion`: uses the existing shared FourPipetteTransport and its completion tokens/interrupt epoch. Existing `detect_fluid()` waits before returning, which is unsuitable for moving Z between BR and wait. The new optional internal transport `after_sends` hook runs after **all owner tokens** are captured and outside the send lock. No new transport, scheduler, worker or public raw-command dispatcher.
- `aspirate_calibration_air`: exact source 15 rear-air bookkeeping, 1 ms/channel, 20 ms after sends, source volume/speed timeout, and ignored false wait return preserved separately from completion evidence. Existing transport initialization and actual exception/interruption behavior remain. The existing generic methods keep their defaults; new source paths do not change AllowToStop merely by using the common helper.
- `evaluate_fluid_timing`: sorted four delays; middle-pair median; upper-pair minus lower-pair mean spread; max(median+1.5*spread, median+1); source failure if any delay exceeds that threshold OR 10 seconds. This is an OEM post-measurement failure, not a new admission gate.
- `calibration_samples(plate, skip_steps=4)` and `calibration_adjustment(...)`: pure sample ordering and exact adjustZ proposal arithmetic. Neither measures, moves, writes, nor claims calibrated state.

Timeout preserves source TR then exception; source timing rejection preserves its exception before the success tail. No fabricated finally lifts, restores, or dispensations. Missing timestamps are reported as timing=None, not converted into a new refusal. Existing controller/native failures still propagate. Exception `.evidence` contains completed steps; successful results always say physical_effect_verified=false and calibration_persisted=false.

## Parent integration contract

`bind_calibration_provider(provider, finite=..., native=..., pipette=..., tray_location=..., z_current_down=...)` builds `CalibrationBindings` over the existing provider and source Z primitives.

- Run the entire composite within the **existing canonical deck/workflow owner**. It is not safe to dispatch these Python bodies from an unowned HTTP thread.
- `finite(operation, inputs)` compiles/executes the existing finite deck operations. Preserve native child source fields (`z`, `source_return`) at the returned mapping level as well as the aggregate evidence. Used operations: pipette_hotel (existing well moveTo), pipette_location, pipette_home, pipette_tip_state, pipette_lift, pipette_position.
- `native(name, callback, inputs)` executes the supplied provider primitive as that same owner's durable child. It is internal trusted composition, NOT a request-selected callable/method. Used primitives: lowerPipette, Z speed, nonblocking OEM Z, source Z stop.
- `pipette(name, collection_callback)` runs against the existing shared collection/receipt service. Fluid detection is a compound pipette + Z operation: its callback starts the owned Z child between the sends and waits, without releasing the parent deck ownership. Do not use an independent nested submission that deadlocks waiting for this parent.
- `tray_location(zero_based_tray)` comes from canonical source `get_tip_tray_location`, allowing the source's tray-location assignments including hotel. No added occupancy/proof admission is requested.
- `z_current_down` must be the captured setting. No guessed default. Bound PositionTable supplies high/low Z; normal source pseudo-home clamping stays in the real Z primitive (the C# appAdjustment parameter is unused in moveZ).
- Parent API/UI is intentionally untouched. These functions are not yet a claim that installed controls can invoke them.

## Original caller inventory and remaining unsupported composition

Exact anchors below are in `decompiled_src/BioXPControlLib/ControlLib.cs` unless otherwise stated.

### 1. Diagnostic Detect Fluid button (1440-1469)

This is NOT a one-well detector: sets fluid-level log; catch pool plate and release to location 23 with press; initiateGroup; zOffset(300,"TC",false,1); pool catch/release to 25 with press; zOffset MS; press output; zOffset OC; press reagent; zOffset RC; set strip[1].StripColor="X"; zOffset STRIP; park. Catch reports timeout dialog/log, no unconditional park finally.

**Remaining:** assemble this multi-plate workflow using existing catch/release/press/park and source initialization owner; add strip-color mutation with its real source model, not a synthetic occupancy publication; preserve partial failures. No route/button should claim this full button is implemented by the one-well body.

### 2. zOffset (3387-3627)

Mappings `(location, plate, columns, nominal fluid uL)`:
TC=(2,0,12,25); MS=(0,0,12,25); OC=(1,1,12,25); RC=(3,2,12,40); STRIP=(12,8,3,40); OCMS=(0,1,12,40).

Publishes selected plate location first. Defaults transferFluid=true and skipSteps=4; diagnostic caller passes false and 1. For transferFluid and location !=0: load fresh 50 uL tips; remember tray/current well; split volume using `(volume+49)/50`, integer divide volume; for each sampled column A then B: scriptmoveTo trough16 flag0, update trough well0, masp(speed100,volume), scriptmoveTo target flag1, publish target/well, mdsa(speed100). Return to remembered tray/well at flag1; **source incorrectly publishes target location with remembered tray well**; eject true,true and clear TipLoaded.

Measurement loop samples A then B for each selected column: fresh 50 uL tips; remember tray/well; scriptmoveTo sample flag1; publish sample; scrFluidDetection(speed); append height and delays; return tray/well flag1; **same mismatched location publication**; query tips; while source TipExist: eject true,true and query; clear TipLoaded. Return `(int)(sum/heights.Count + .5)`, not Python banker's round.

**Remaining:** executable repeated-scan body and transferFluid prefill composition with existing real loadTips/masp/mdsa; literal unusual publication must be surfaced as source behavior, not silently repaired. Repeated tip-ejection loop is unbounded in source; no arbitrary retry cap/new gate is approved. `calibration_samples` only inventories order, not an executable scan. No unsupported route exists.

### 3. Persisting fluid calibration button (2751-2850)

Dialog entry sets Z acceleration176 even before confirmation; on confirmation settings.backup(), background calwithFluid. calwithFluid resets machine status, presses pool, measures TC via zOffset default transfer=true/skip4, adjustZ and saveConfig; moves pool to MS25 with press, measures/saves MS; pool returns TC23; press output then OC measure/save; press reagent then RC measure/save; strip[1] color X; STRIP measure/save. Finally if TipExist: moveTo waste6 sequential,20ms,eject,20ms; park; resultComparison; restore acc576 and GUI/status. These finally effects are source caller behavior, not part of one-well scrFluidDetection.

**Remaining:** full caller composition, exact resetStatus/strip mutations, confirmed calibration UI interaction, durable apply ownership, comparison/rollback/history. Do not reuse plain measurement success as 'calibrated'.

## Persistence traced to the actual OEM settings owner

`decompiled_src_bioxpcommon/BioXPCommonLib/ClassBioXPSettings.cs`:

- adjustZ 6039-6138 sets current tool FluidReference and reference revision. TC/MS/OC new zLow = measured + FLUID_TC_OFFSET; MS also sets PLLow. RC uses FLUID_RC_OFFSET. OCMS updates OPBufferLow only using RC offset. STRIP delta = measured+FLUID_STRIP_OFFSET-zLow(strip2)-1500, applied to all four strips without changing their existing relative differences.
- backup 6258-6271 copies the PositionTable only when m_calibrated==1.
- saveConfig 3718-3962 marks m_Liquid_Cal/date/revision, serializes all PositionTable x/y/zLow/zDelta/inc_factor entries, writes current config.xml if present else appDir/config.xml, and writes InspectionSettings.xml. It is not an Operation_parameters.xml write. zHigh is not serialized directly.
- resultComparison 6273-6542 compares backup/current and presents the OEM dialog. Accept saves configuration history; reject calls restoreCalibration. It is not a validation gate on collecting a measurement.

### Explicit integration decision required for usable Apply

The port binds config.xml/PositionTable to an immutable sealed OemMachineSnapshot, and set_active refuses in-process replacement. The existing Operation_parameters copy-on-write path does not update PositionTable. **This patch does not modify sealed evidence, introduce a fake Save action, or silently rebind axis owners.**

A real Apply operation requires an explicit architecture decision: add a canonical writable calibration layer/versioned config owner whose atomic activation updates every PositionTable consumer, or create/activate a new machine configuration snapshot through the supported lifecycle. The contract must define revision/provenance, backup and rollback, zHigh/derived geometry recomputation, current-tool/liquid-cal/date/revision fields, MS PLLow and OCMS OPBufferLow, strip-relative updates, retained immutable source evidence, and owner/axis reference handling during activation. Obtain approval for that concrete integration; do not infer new physical refusal policy from this missing owner. Until implemented, expose **Measure / proposed adjustment**, not **Apply calibration**.

## Verification boundary

Hardware-isolated tests exercise these new bodies connected to the real finite compiler/executor, provider, production motion adapter, FourPipetteTransport/CanPipetteTransport and SQLite semantic publisher. Only native motor/pipette IO and captured state/PositionTable inputs are fixtures; sockets are blocked. No calibration implementation is mocked. Tests cover addressed pickup/overpress/home, native failure without false cleanup, BR→Z→wait ordering, timeout, source timing rejection, missing timestamps without an added gate, Stop ownership, source false air wait, and adjustment arithmetic. API admission/UI integration and installed hardware remain unverified.

Qualification run: 86 passed, zero skipped across test_oem_pipette_calibration.py, test_protocol_oem_air_native.py, test_protocol_oem_stream_native.py, and test_protocol_oem_pipette_composites.py. The additional provider-bindings suite has one unrelated RGB expectation failure (expects one write, implementation intentionally writes twice); reproduced identically using pristine `a65a176` source (19 pass/1 fail). No expectation was weakened. Original retained-baseline fixture paths from the skill were absent, so these tests prepare a fresh real canonical SQLite database rather than silently skipping retained-fixture cases.
