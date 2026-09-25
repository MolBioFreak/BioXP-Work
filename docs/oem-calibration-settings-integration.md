# Editable calibration integration

## Contract and ownership

`bioxp.oem_calibration_settings.CalibrationSettingsPatch` is the typed PATCH body.
It accepts `positions: [{name, x?, y?, zLow?, zDelta?, inc_factor?}]` only.
Names are OEM enums and must exist in this machine's captured PositionTable.
Values are strict signed Int32 (CommonLib/positionStruct.cs), not coerced strings,
bools, floats, or nulls. No travel envelope, measurement/observation requirement,
operator attestation, or new motion admission policy is added. Existing motion
limits/interlocks and PositionTable normalization still apply.

The contract edits final saved PositionTable values, not raw calibration probe
measurements. Save a batch of all related calculated rows where an OEM calibration
changes multiple rows. `zHigh` is derived, not independently editable.

Storage is the existing OEMRuntimeStore SQLite runtime_metadata row
`machine_calibration_v1:<captured-lock-sha256>`. A revision contains schema_version,
revision_id, baseline_lock_sha256, saved_at, and merged typed position overrides.
It is user-authored configuration, NOT a new captured evidence lock, controller
receipt, or hardware authority fact. No schema migration or second filesystem
configuration authority is added. Save is transactional read/merge/write/readback;
a failed transaction rolls back. The revision is the latest saved configuration,
not an append-only audit/history service. Concurrent partial saves merge under
the canonical store's writer lock and SQLite BEGIN IMMEDIATE.

## Parent API changes (not made in this child)

1. At API lifespan startup, after canonical runtime schema preparation and before
   `configure_oem_machine_snapshot_from_env`, obtain an `OEMRuntimeStore` at the
   existing resolved `runtime_root`. Keep that owner on `app.state` for the
   settings service and close it during lifespan shutdown. Do not make settings
   depend on a connected hardware provider: configuration editing is offline.
2. Call:
   ```python
   machine_snapshot = configure_oem_machine_snapshot_from_env(
       require_operator_label=True, runtime_store=app.state.calibration_runtime_store,
   )
   configure_oem_runtime_state_from_env(machine_snapshot)
   app.state.calibration_settings = CalibrationSettingsService(
       app.state.calibration_runtime_store, machine_snapshot,
   )
   ```
   This is before provider construction and the one existing snapshot bind. Do
   not call this again after a save; existing in-process replacement refusal is
   unchanged. Do not introduce a restart/home/rebind endpoint.
3. GET `/motion/oem/calibration_settings`: return `service.read()`.
4. PATCH `/motion/oem/calibration_settings`: typed body
   `CalibrationSettingsPatch`, return `service.save(body)`; map input ValueError
   to a normal validation response. Storage errors must be reported as errors,
   never reported as applied. These methods are synchronous SQLite work: use a
   synchronous FastAPI route or the existing thread offload convention.
5. Generate frontend fields from the typed schema/OpenAPI. For each existing row
   expose the five integer fields with units: x/y/zLow/zDelta controller steps;
   inc_factor dimensionless. Use OEM names as stable identifiers.

`read`/`save` report `active_positions` and `saved_positions` (configuration),
`active_motion_positions` and `saved_motion_positions` (actual shared loader
projection), and each loader adjustment ledger. Show pending_restart and the
active/saved revision IDs distinctly. Save response includes committed_revision_id
so a concurrent later writer cannot be mistaken for this transaction. A save
never mutates the service's active snapshot; only next ordinary startup consumes
it. `bound_configuration` means the process's bound configuration, not a physical
accuracy or hardware calibration verification. Without the startup call above,
this child does not claim the API automatically consumes saved settings.

## Source and consumer chain

- SSD `decompiled_src_commonlib/CommonLib/positionStruct.cs`: all five fields are
  Int32. `ClassBioXPSettings.cs:3932-3945` serializes these exact five attributes
  in saveConfig. PositionTable is the mutable OEM dictionary (line 1389).
- `ClassBioXPSettings.calXYZoffset` updates related rows; this service does not
  replace or guess its calibration math. Child/provider calibration routines
  must pass their final derived row changes explicitly to the save UI/service.
- `setCameraOffset` (6219-6235) subtracts location 7's XY from measured XY and
  stores CAMERA_OFFSET XY; `setCameraZOffset` (6238-6246) stores its zLow. Do not
  treat a raw camera measurement as an already-computed offset.
- At normal startup a pure snapshot projection keeps records, fields, limits,
  flags and XML evidence untouched, overlays only PositionTable, and identifies
  edited rows with user-authored revision provenance. `_snapshot_legacy_bundle`
  reports the configuration source honestly.
- `load_bound_oem_position_table` consumes that snapshot through the existing
  PositionTable resolver, then existing well move/scriptmove formulas consume
  x/y/zLow/zDelta/inc_factor. Camera preparation also reads CAMERA_OFFSET from
  the same snapshot. No fresh hardware reads are needed to save or read settings.
- Important existing OEM normalization: TECAN rows always use zDelta=53000;
  zHigh below 5000 becomes zero; LOC_P_OC x follows LOC_P_MS x; certain press-row
  zero XY values inherit their source station. These rules are preserved and
  visible in the saved-motion projection. Do not label raw saved zDelta as the
  effective TECAN travel value.

## Deliberate boundary

This commit provides functional tray/well, tip rack, camera offset, park and
other captured PositionTable calibration fields. It does not invent editable
pipette liquid calibration coefficients or expose the entire config XML. Existing
Operation_parameters flags (LogPressure, CheckSnapTips, etc.) remain with
OemRuntimeStateStore.write_operation_parameters and its cached projection; they
are not silently copied into a competing SQLite overlay. General diagnostic
settings exposure remains parent work. Mechanical accuracy is not physically
verified by these offline tests.

Qualification:
`PYTHONPATH=src:tests /home/dalab/.cache/bioxp-oem-cv-venv/bin/python -m pytest -q tests/test_oem_calibration_settings.py`
