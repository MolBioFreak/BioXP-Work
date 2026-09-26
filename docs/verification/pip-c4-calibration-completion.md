# PIP-C4 calibration completion

## Approved semantics

Calibration and paired tray-Z Set save/apply through CalibrationSettingsService in-process. Reject restores the complete pre-run calibration, replacing later calibration edits, including fields unrelated to the measured stations. No stale-write refusal, acceptance admission gate, new scheduler, reconnect, home or restart was added. Existing finite command ownership/exclusion remains unchanged. Original captured XML, provenance records and axis authority are not modified.

The active projection is published through the process snapshot getter used by PositionTable, legacy settings/config projection, provider movement and fluid-scan readers. It retains its sealed baseline separately: restoring an absent prior user revision projects that baseline rather than accidentally retaining later edited rows. Ordinary startup projects whichever revision is actually saved; pending operator choice is not a startup gate.

## Source anchors (retained SSD decompile)

- `decompiled_src/BioXPControlLib/ControlLib.cs:2782–2849`: reset, five station scans/adjustZ/saveConfig, caught body exception, cached TipExist, waste/eject, Park, comparison, completion and Z acceleration. Earlier finally exceptions skip subsequent source work; they remain distinct `finalization_error` evidence.
- `decompiled_src_bioxpcommon/BioXPCommonLib/ClassMachineStatus.cs:655–689`: reset occupancy and tip metadata T50/T50/T50/T200/T50 at locations 7/8/9/10/15, tray/strip/movable defaults. These are logical declarations, not observed physical stock.
- `ClassBioXPSettings.cs:6039–6138`: unchanged source adjustment arithmetic and strip-row coupling.
- `ClassBioXPSettings.cs:3740–3758,6071–6072,6348–6355`: `m_current_tool=FluidReference` drives liquid-calibrated metadata/date and comparison label. The owner applies persisted liquid metadata to the active configuration, not only a proposed `settings_updates` dictionary.
- `ClassBioXPSettings.cs:6083–6089,6872–6892`: `m_PLLow` preserves the MS low for `resetMSZ(false)`. Supported port movement/fluid consumers resolve `LOC_MS.zLow` through the shared table and now immediately consume its updated low. The port does not have an independent `resetMSZ`/output-buffer switch; this change does not invent that separate workflow or claim coverage for it.
- `ClassBioXPSettings.cs:6518–6539`: accept saves history regardless of body completion; reject restores; comparison/dialog/history/restore errors are internally caught, allowing subsequent UI and acceleration restoration. No-prior-calibrated-backup follows the source true result without fabricating operator consent.

## Frozen REST contract

- `GET /motion/oem/calibration_settings/runs/{run_id}`
- `POST /motion/oem/calibration_settings/runs/{run_id}/decision`, JSON `{ "decision": "accept" | "restore" }`.

Both return the run object, not an envelope:

- `schema_version: "bioxp.calibration_run.v1"`, `run_id: string`
- `before`, `after`: `{positions:[{name,x,y,zLow,zDelta,inc_factor}], liquid_calibration:{...}, revision_id:string|null}`
- `decision: null|"accept"|"restore"`, `decision_status: "running"|"pending"|"accepted"|"restored"|"no_previous_values"`
- `saved_revision_id`, `active_revision_id`: current owner revisions, string or null; `before/after` stay the compared run rather than subsequent edits.
- `body_completed: boolean`, `measurements:[{plate,measured_raw_z,saved_revision_id,calculated_z_lows,settings_updates,pending_restart}]`
- `error`, `comparison_error`, `finalization_error`: nullable strings; body failure does not prohibit a decision. Source no-backup may expose `comparison_choice:true`, `comparison_source:"no_previous_values"`, with `decision:null`.
- Acceptance retains `accepted_history` containing the compared `after` values. Restore retains the comparison for readback and restores the saved revision (or its absence) plus active projection. A comparison failure is returned as evidence without falsely recording a successful choice.
- Unknown run: HTTP 404. Invalid decision/extra request fields: HTTP 422. Existing unconfigured-service HTTP 503 remains.

The existing catalog discovers the typed routes; decision is classified with existing no-motion settings operations. The finite calibration result carries run_id, saved/active revision IDs, decision/status, body completion and errors, without embedding before/after snapshots. PIP-C5 owns their terminal `pipette_result` propagation; BMS owns mounted presentation.

## Persistence and qualification

The existing OEMRuntimeStore SQLite owner stores full pre-run revision and comparison values, each station result and acceptance history. Station revision and compact measurement/checkpoint share one transaction. A run can be recovered after owner reopening by its run_id. No diagnostic scan bulk is stored in comparison history.

Offline tests cover actual HTTP schema/catalog, SQLite owner reopen, full and each partial station failure, prior/no prior user revision, source no-calibrated-backup, accepted partial history, restore after unrelated newer edits, shared geometry readback, paired tray Set, sealed bytes, ordinary startup, comparison/history/restore errors, source-finally failure short-circuit and atomic station/checkpoint rollback. The connected calibration test executes the actual native handler/provider, source model, real settings and position-table consumers, OperatorCommandStore and PipetteReceiptStore, with hardware transport replaced, including prepared nondefault tip metadata.

Run with `PYTHONPATH=src:tests:. python -m pytest` (the installed `pytest` wrapper has a missing interpreter). No hardware operation, deployment, physical accuracy qualification or independent physical-stock verification was performed.
