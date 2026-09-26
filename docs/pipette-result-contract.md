# Pipette terminal result contract

`terminal_evidence.pipette_result`, `terminal_evidence.response.pipette_result`,
and the native workflow action result's top-level `pipette_result` carry the same
operational result. Read this field rather than searching diagnostic children.
The native action's `command_id` and `action_id` retain command/action identity.
`run_id` is copied unchanged when the source supplies it; it is never synthesized.

The discriminator is one of:

- `source_calwith_fluid`: `measurements` in source station/save order.
- `diagnostic_detect_fluid`: `scans` in source station order.
- `source_fluid_offset`: `samples` in source sampling order.
- `measure_fluid_height`: source `position_steps` / `source_return`.

All fields below except `kind` are conditional on the actual producer. Missing is
not false, zero, null, or an invented completion. Explicit null/false remain exact.

```typescript
type Sample = { well: string; position_steps: number };
type Scan = {
  kind: "source_fluid_offset";
  samples?: Sample[];
  plate?: string;
  source_return?: number | boolean | null;
  ok?: boolean;
  error?: string;
  // Other supplied source fields, including speed/skip_steps/transfer_fluid,
  // source_anchor and physical_effect_verified, are retained unchanged.
};
type Measurement = {
  plate: string;
  measured_raw_z: number;
  calculated_z_lows: Record<string, number>;
  settings_updates: Record<string, unknown>;
  saved_revision_id: string;
  pending_restart: boolean;
  scan: Scan;
};
type PipetteResult = {
  kind: "source_calwith_fluid" | "diagnostic_detect_fluid" |
        "source_fluid_offset" | "measure_fluid_height";
  run_id?: string;
  ok?: boolean;
  body_completed?: boolean;
  completed?: boolean; // diagnostic producer's own completion field
  source_return?: number | boolean | null;
  saved_revision_id?: string | null;
  active_revision_id?: string | null;
  previous_saved_revision_id?: string | null;
  comparison_choice?: boolean | null;
  comparison_gap?: string | null;
  pending_restart?: boolean;
  outcome?: string;
  error?: string;
  finalization_error?: string;
  measurements?: Measurement[];
  scans?: { plate: string; measured_raw_z: number; scan: Scan }[];
  samples?: Sample[];
  failed_scans?: Scan[];
  source_errors?: unknown[]; // literal nested source failures, not success policy
  // Additional original source fields remain available.
};
```

`measurements[].scan.samples` is retained in full, including repeated values.
Partial calibration saves remain in `measurements`, with the actual final saved
revision and supplied active revision kept distinct. `failed_scans` retains any
partial sample result available only in a failed calibration source event.
Diagnostic `events` retain operation/occurrence identity, scalar source outcomes,
and literal nested errors, but discard repeated transport/result trees.

`ok`, body completion, comparison acceptance, durable saving and physical proof
are independent. Native action `ok` continues to follow canonical command status;
`pipette_result.ok` remains the raw source result. A body-complete calibrated run
can still have `comparison_choice: null` and source `outcome: "incomplete"`.
Nested suppressed/ignored source errors are evidence, never new execution gates.
`calibration_persisted` on the manual action is true only when this invocation has
at least one saved station revision; an older saved revision alone is not a write.

Critical data is extracted before diagnostic bounding in canonical child,
operation, terminal, outbox and native result paths. Non-pipette bounding is
unchanged. No transport log digest/preview substitutes for measurements, and the
diagnostic byte cap is unchanged. Critical samples are not truncated to that cap.

## Executed examples and qualification

[pipette-result-examples.json](pipette-result-examples.json) contains complete
literal `pipette_result` objects read from the real SQLite terminal records after
transport-replaced connected tests on base `7a4a58d` plus this change:

- success: five saved stations, body complete, comparison still pending;
- partial_failure: TC/MS saved before OC detector failure;
- finalization_failure: five saved stations, body complete, final acceleration
  transport exception retained independently.

These fixtures intentionally contain no `run_id`: the base producer does not
supply one. Future supplied run IDs and active-revision fields pass through.
The separate calibration implementation owns adding that identity and changing
actual in-process activation, not this result projection.

Connected tests execute ProtocolExecutor -> real manual handler -> canonical
admission/claim -> OperatorCommandPlane._dispatch_one -> real finite executor and
provider -> terminal SQLite -> shared native terminal reader -> workflow action
publication/readback. The hardware transports are replaced and network/CAN sockets
blocked. They cover five-station diagnostic repeated scans, calibration success,
failure after two saves, finalization failure, and source ignored Boolean return.
Contract tests additionally exercise every child-container shape and oversized
critical sample lists. BMS relay/mounted rendering and live hardware are outside
this robot-worktree qualification; nothing was deployed.

## Selected-source and diagnostic additions (frozen consumer contract)

These source-owned discriminators extend the union above, at the **same**
`action_result.pipette_result` location. Do not infer a kind from a command name,
revision, generic source anchor, or diagnostic success. Optional means absent if
not supplied; explicit null and false remain distinct. In particular, a thrown
load has no Boolean `source_return` and may retain a changed `tip_location`.

```typescript
type AdditionalCommon = {
  ok?: boolean;
  completed?: boolean;
  source_return_completed?: boolean;
  physical_effect_verified?: boolean;
  delivery_attempted?: boolean;
  controller_outcome_ok?: boolean;
  interrupted_by_terminate?: boolean;
  source_noop?: boolean;
  source_anchor?: string;
  source_occurrence_id?: string;
  error?: string;
  exception_type?: string;
  failure?: string;
  source_errors?: unknown[];
};
type Occurrence = {
  operation: string;
  step_id?: string;             // manual native_results occurrence
  source_identity?: string;     // diagnostic events occurrence
  result?: Observation;
  error?: string;
};
type SourceLoadResult = AdditionalCommon & {
  kind: "source_load_tips";
  source_return?: boolean;
  requested_pipette?: number;   // -1 means all; otherwise 0..3
  tip_location?: number;        // actual canonical source state, not request
  already_matching_tip_type?: boolean;
  alignment_published?: boolean;
  native_results?: Occurrence[];
};
type SourceLiquidResult = AdditionalCommon & {
  kind: "source_mix" | "source_aspirate_air" | "source_dispense_air" | "source_purge";
  source_return?: string[] | null; // original source return, not a Boolean coercion
  native_results?: Occurrence[];
};
type DiagnosticResult = AdditionalCommon & {
  kind: "diagnostic_pipette";
  action: "aspirate" | "dispense" | "dispense_all" | "diagnoses" |
    "initialize" | "eject" | "get_data" | "last_error" | "plunger_up" | "plunger_down";
  selected_channels?: number[];
  lost_tip_channels?: number[];
  cached_tip_channels?: number[];
  ejected_channels?: number[];
  events?: Occurrence[];
  stroke?: Observation;
  dispense_all?: Observation;
  tests?: {
    number: number; label: string; result: Observation;
    channels: {channel: number; diagnosis: string | null; display: string}[];
  }[];
  attempts?: {attempt: "initial" | "retry"; group: Observation; status: Observation | null}[];
  channels?: ({channel: number; part_number: string; revision: string;
    firmware: string | null; data: Observation} |
    {channel: number; error: number | null; display: string | null; result: Observation})[];
};
```

`Observation` is a recursive **projection of the actual source result**, not a
fresh interpretation of success. Its retained keys are closed to:

```
ok completed source_return source_return_completed controller_outcome_ok
interrupted_by_terminate channel channels result completion status group attempt
number label diagnosis display part_number revision firmware data query queries
results value value_ascii value_bytes unit error errors oem_error_code oem_error_free
semantic_ok reply_received timeout_ms operation step_id source_identity
source_occurrence_id source_noop outcome delivery_attempted exception_type failure
steps current current_max requested_current signed_steps requested_steps position_steps
command_issued physical_motion_commanded physical_effect_verified command_id
parent_command_id plan_digest child_order tip_loaded source_tip_loaded tip_type
tip_location volume_ul speed
```

Values retain their source types (including null, arrays and nested observations).
Diagnostic `events[].result` is the scalar-only subset of this projection; detailed
channel observations live once under `tests`, `attempts`, `channels`, `stroke` or
`dispense_all`. `native_results` retains projected nested channel execution
outcomes. Wire ACK/frame logs, `driver_result`, repeated `aggregate` and firmware
`information` copies, receipt trees and arbitrary bulk fields are not retained.
Errors hidden inside those removed trees are kept literally in `source_errors`.
Repeated compaction is idempotent, including those errors.

### Exact renderer paths

- Load: `requested_pipette`, `tip_location`, `source_return`,
  `already_matching_tip_type`, `alignment_published`. Matching-type early return
  does **not** realign to the newly requested channel. Failed reload can expose
  `tip_location: -1` after source ejection without claiming a new pickup.
- Aspirate/dispense: `selected_channels`, `lost_tip_channels`, `cached_tip_channels`,
  `stroke.ok` and `stroke.channels`. Source aspirate retains a lost-tip selection;
  dispense removes it. Do not replace the source choice with renderer policy.
- Dispense-all: `cached_tip_channels`, `dispense_all.ok/channels/timeout_ms`.
- Diagnoses: `tests[].number` (0,1,2), `label`, and
  `channels[].channel/diagnosis/display`. Partial failure retains only completed
  tests. `No data returned` is the actual source display, not an invented pass.
- Initialize: `attempts[].attempt`, `group.ok/outcome`, `status.ok/failure`,
  `status.channels[].result.oem_error_code`. Source-return success can coexist
  with `controller_outcome_ok: false`; no extra retry or gate is introduced.
- Eject: `ejected_channels` (only actual completed source ejections).
- Get-data: four `channels[].part_number/revision/firmware`; query values are at
  `channels[].data.channels[].results[]` (`query`, `value`, and supplied
  `label/unit/value_ascii/value_bytes/semantic_ok`). Partial arrays remain partial.
- Last-error: four `channels[].channel/error/display`; no fabricated zero codes.
- Plunger: event `operation: "setZaxisCurrentmax31"` has `result.value` (31);
  `operation: "moveStepsZ"` has `result.requested_steps` (signed),
  `result.command_issued`, `result.physical_motion_commanded`, and
  `result.physical_effect_verified`. These are commanded values, not measured travel.

### Reproducible additional exports

`testdata/pipette_completion/additional-results.json` has exactly:

```typescript
{ cases: {name: string; action_result: object; request: object}[] }
```

Each `action_result` is the complete actual workflow-published SQLite action row,
not a manufactured renderer fixture or an edited compact-result substitute.
`request` is the actual manual-authoring request used for that action execution.
There are 21 cases: selected pickup, matching-type no-realignment, failed reload,
four source liquid operations, all ten diagnostics, partial diagnosis failure,
initialize's ignored failed retry, and lost-tip aspirate/dispense. Fixture hardware
responses are simulated and explicitly **not live observations**. Dynamic command
IDs and timestamps intentionally remain literal; regenerations change them.

```sh
PYTHONPATH=src PIPETTE_ADDITIONAL_EXPORT=1 python -m pytest -q \
  tests/test_pipette_additional_result_connected.py
PYTHONPATH=src python -m pytest -q tests/test_pipette_result_contract.py \
  tests/test_pipette_additional_result_connected.py \
  tests/test_manual_source_pipetting_connected.py tests/test_oem_diagnostics_connected.py
```

The additional suite uses real manual handlers, queue admission/claim,
`OperatorCommandPlane._dispatch_one`, the finite executor, source provider, both
SQLite stores, outbox readback and workflow publication/readback. It reuses only
the connected hardware/configuration fixtures, not their direct execution helpers.
No cap increase, scheduler, service, admission policy, hardware run or deployment.
