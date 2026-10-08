# Final method native execution matrix

Pinned native product: `960caec98518fb84e96dcd339bb514128f52d188`. No native production behavior changed by this qualification.

BMS actual immutable snapshot producer: `33b40edd5a945f8b2be602ff2416551cd8e2d125`; final self-contained producer regression: `a613d12531d41da0810a689cc8b523c8a677024d`. Native schema identity is committed in BMS, not a mutable sibling interface note. The Methods producer recompiles every supplied raw method/binding/dependency snapshot and verifies the original compiler document before native execution.

## Complete corpus

`../final_method_documents.json.gz` contains all **87** final model documents with actual `metadata.bms_method_run` snapshots. Every document is parsed, submitted once through real `/protocol/execute`, dispatched through real owners, persisted to scratch SQLite and reread through `/protocol/jobs/{id}`. Exact document, action IDs and occurrence metadata are asserted. Physical controller/camera leaves and installation setup are fixture-owned. Ordinary authored checkpoints are reviewed through the real review route.

**78 completed; 9 expected invalid-context documents** retain their actual failed/ambiguous outcomes, not rewritten inputs:

- Index 22: plate source is GANTRY ordinal 32 without a positioned predecessor (`machine_target_absent_from_serial206_position_table:32`); suffix never executes.
- Indices 29, 40: unstarted standalone timer (`timer_not_started`). Index 29 reaches its authored error hold and is explicitly aborted.
- Indices 50, 61: standalone release has no carried-plate authority (`source_authority_missing:updatePlateLocation`). Actual partial movement is preserved; final status is ambiguous, not “nothing happened.”
- Indices 53, 54, 66, 76: plate/cover move toward the thermal cycler with the existing door-closed condition (`thermal_door_must_be_open`). Error-hold variants retain the reached hold before explicit Abort.

These negative cases are acceptance of native error/partial-effect behavior, not successful execution of their requested operation. No new gate is introduced.

## Recovery and controls

`../final_recovery_documents.json.gz` contains four separately authored, genuine BMS run inputs. Eight connected scenarios export:

- Successful pickup followed by first recipe-command failure.
- Mid-aspiration and mid-dispense channel failure.
- Post-pickup `pause_for_operator` error hold and later explicit Abort.
- Cooperative Abort of an active wait.
- Reached ordinary pause, same-job Continue and completion without replay.
- Safe-state stop of the ordinary native document.
- Exact rejection of deferred pause and wake for this non-OEM document. Deferred pause requires OEM lifecycle; this matrix does not fabricate a reached deferred gate or claim ordinary-document wake support.

The `recovery-aborted` export is the actual terminal **ambiguous** state after Abort of a partial liquid error hold. Its filename identifies the requested control, not an invented native terminal enum.

## Evidence

`manifest.json` lists each document/digest/action/parameter set, native outcome and compressed/uncompressed capture hashes. **101 exports** contain the exact producer snapshots and canonical wire/job IDs. It excludes stale held captures from superseded failing runs by matching the final execution's actual held-state snapshot.

Selected final JUnit: **87 matrix + 8 recovery/control + 1 native export + 207 BMS = 303 passed, zero failures/errors/skips**. These are offline software tests, not physical liquid/thermal/scientific qualification. Full-method fixtures remain explicit software science.

The initial matrix had fixture/context expectation failures; the next run narrowed to inspection fixtures. Missing tray/constructor state, synchronous CAN completion/query replies, motion home/wait leaves and camera controls/topology were corrected in the harness, not by relaxing product assertions. Initial white-frame inspection also exercised over-cover receipt rejection; positive topology now uses the existing station-specific image fixtures. The original failing JUnit remains under `prior/` and the durable audit directory.

The pipette has no clot/air classifier, so none is modelled. No automatic Z Stop, cleanup, retry, live hardware action, database migration, push or deployment occurred.

Run using `../native_method_close/run-final-matrix.py`, then `../native_method_close/run-final-contract.py tests/test_final_recovery_producers.py tests/test_final_contract_export.py`. The runner reuses the established network-disabled, read-only container and offline hardware guard. `package-final-contract.py` verifies counts, snapshots, statuses and JUnit before packaging.
