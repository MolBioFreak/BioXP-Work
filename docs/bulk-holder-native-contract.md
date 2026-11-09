# Bulk reagent holder native backend contract

Candidate base: installed `064f8d3d75d9cab2f7e467057a65b301e170afb1`. No deployment or live apply authorized by this artifact.

## Existing owners, extended narrowly
- `CalibrationSettingsService` / `OEMRuntimeStore` canonical SQLite machine-calibration revision: persistence, merge, in-process publication and cold startup read.
- `PositionTable.from_bound_machine_config`: carries the separate XY definition into the existing content revision without changing OEM station rows.
- `DeckCatalog`: one persisted definition generates finite listed destinations and XY metadata.
- `compile_named_location` -> existing provider `moveTo` offset overload; provider `_resolve_xy_target` uses the same definition for movement-mode sequences. Existing named-target request models accept these keys for future workflow consumers. No parallel registry, motion scheduler or depth gate.

## Save/read API
Existing `PATCH /motion/oem/calibration_settings`, payload (positions may be omitted):
```json
{"bulk_reagent_holder":{"micro_row_1":{"x":85555,"y":63278},"micro_row_2":{"x":90262,"y":63278},"pcr_row_1":{"x":80162,"y":61978},"pcr_row_2":{"x":82262,"y":61978},"pcr_y_pitch":2132}}
```
Complete holder replacement; unrelated PositionTable patches and liquid calibration retained. Omit holder to leave unchanged; null rejected. `GET` returns `active_bulk_reagent_holder`, `saved_bulk_reagent_holder`, revision IDs and `motion_commanded:false`. Z fields forbidden in this holder definition; no historical lock/XML edits.

## Addressable destinations / receiving
`LOC_TROUGH` remains station 16 and its legacy XY/height/inc_factor semantics remain unchanged. Its display label becomes **Bulk reagent holder**.

When saved, `GET /operator/v2/control-catalog?view=full` lists these destinations under `actions[action_id="oem.deck.move_to_location"].destination_options` (not the static `view=metadata` response):
- `LOC_TROUGH_MICRO_ROW_1`, `LOC_TROUGH_MICRO_ROW_2`: row anchors only; no inferred individual micro-tube pitch.
- `LOC_TROUGH_PCR_ROW_1`, `LOC_TROUGH_PCR_ROW_2`: first-well anchors.
- `LOC_TROUGH_PCR_ROW_{1|2}_WELL_{1..8}`: 1-based physical well numbers, Y = row anchor Y + (well-1)*2132. Both well 2 targets have Y=64110.

Holder options include `xy:{x,y}`, `holder_row`, `well_number` (null for row anchors), `channel_wells`, `calibration_axes:["x","y"]`, `station_target:"LOC_TROUGH"`; existing target/label/location_id/branch/availability fields remain. PCR well 1 advertises channels over [1,3,5,7], well 2 over [2,4,6,8]. Other individually addressable PCR wells do not claim validated four-channel coverage.

Existing action `oem.deck.move_to_location` inputs: `{"target":"LOC_TROUGH_PCR_ROW_2_WELL_2","camera_offset":false}`. Movement-mode named target union uses the same object inside the existing target field. Compiler passes `named_target` to provider for extensions only; not client-provided raw coordinates. Existing clearance and camera-offset behavior retained; no holder pipetting depth/height assigned. Legacy `move_to_well` station-16 integer addressing remains OEM-compatible, not silently reinterpreted as a holder row selector. Future consumers select explicit named row/well keys.

No UI geometry/layout/click-handler implementation, mobile APK changes, robot POSTs, collection, reconnect, homing, motion or service restarts are part of this candidate. Parent coordinates any later live calibration save and deployment.
