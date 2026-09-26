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
