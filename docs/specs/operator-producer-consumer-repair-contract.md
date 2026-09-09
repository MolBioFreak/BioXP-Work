# Operator producer/consumer repair contract

## V1 catalog and durable history (C02 coordination)

`GET /operator/control-catalog` remains V1 and omits canonical methods requiring
the V2 request/receipt/authority contract. `oem.deck.move_to_location` remains in
`GET /operator/v2/control-catalog`; exclusion does not enable it or establish
missing deck authority.

`GET /operator/actions/history` returns the existing
`bioxp.operator_action_history.v1` envelope and its heterogeneous `receipts`.
Only the private `__projection_source` merge annotation is removed. In particular,
the durable `bioxp.operator_command_receipt.v1` variant with
`source: legacy_operator_plane` is **not** converted to a direct action receipt,
converted to a V2 receipt, or removed. BMS needs an explicit closed variant for
this producer contract, not a global extras allowance.

The retained durable command IDs are:

- `882ffcef-25f7-4ad9-ab35-c53accd5c89e`
- `867f8a2e-c940-4f0e-9679-3bdb4269e49a`

Their public fields are: `schema_version`, `source`, `command_id`, `action_id`,
`method_id`, `method_sequence`, `sequence`, `stream_sequence`, `transition_sequence`,
`status`, `stored_status`, `state_version`, `ownership_generation`,
`expected_board_epoch_by_board`, `requested_inputs`, `effective_inputs`,
`accepted_at`, `queued_at`, `dispatched_at`, `finished_at`, `source_noop`,
`source_noop_reason`, `remote_acknowledged`, `controller_acknowledged`,
`physical_effect_verified`, `completion_class`, `physical_outcome`,
`recovery_required`, `automatic_retry`, `terminal_receipt_id`, `terminal_evidence`.
The source of field types and legal values remains
`OperatorHistoryReader._command_projection`, not inference from only two samples.
The robot patch does not change this producer shape or fabricate evidence.

## Public diagnostics and retained evidence

Deck disabled reasons use an exact finite allowlist of producer diagnostics and
safe contexts. Unexpected exception arguments, subclasses' string formatting,
paths, credentials, and appended text are never interpolated into deck metadata.

Route failure classification recognizes only root, `detail`, `result`, and
`detail.result` envelopes. An explicit `ok:false` in any of those prevents an HTTP
2xx response being described as successful. Machine error codes and UI messages
come from a finite vocabulary. The exact OEM GZ-position timeout grammar maps to
`controller_position_wait_timeout`; arbitrary failure prose does not become a
machine code. Unknown errors get a stable generic classification. Ambiguous
completion takes priority and remains non-retryable, with reconciliation required.

This is a safe **summary** policy, not a redaction rewrite of historical evidence.
Existing retained response bodies and bounded structured failure details remain
available under the existing receipt/detail contract; their content is not replaced
with the safe summary. Tests verify the original nested body survives detailed
receipt retrieval, including message text omitted by the existing compact history
projection.

## Mixed-source history ordering

Select one complete projection per command identity **before** the cursor boundary.
Direct receipts win against durable receipts for the same command, regardless of
conflicting timestamp or status, consistent with existing direct-first lookup.
Do not merge evidence fields from different projections. Sort winners descending
by `(accepted/queued/started timestamp, direct source rank, sequence, command_id)`.
Opaque and legacy sequence cursors filter these winners, not individual stores.
Distinct identities are not discarded. Missing identity or a non-advancing full
store batch is an explicit failure rather than a silently incomplete result.

Both stores are scanned in 200-row batches before ordering/deduplication because
their readers paginate by sequence, which does not guarantee timestamp order.
This intentionally trades read cost for correctness; a future optimization needs
a shared indexed identity/order query, not premature per-store cursor filtering.
The cursor is not a transaction snapshot across concurrent changes or retention.

## Offline verification

The regression suite reads the actual retained corpus without copying live data
into the repository. Set `BIOXP_AUDIT_CORPUS` to the directory holding
`robot-catalog-v1.json` and `robot-history-v1.json`, and
`BIOXP_BMS_OPERATOR_MODELS` to the actual BMS `operator_models.py` under review.
No BMS runtime services are imported. The entire retained catalog validates after
the production action filter (237 actions); the actual history endpoint preserves
all 25 identities and every public V1 field, and both V1 and V2 histories validate
against the real consumer models. These checks require the separate BMS lane's
durable receipt variant. The original private annotation remains rejected.

Without those two environment variables only the retained-corpus checks skip;
normal isolated endpoint, error, authority, pagination and evidence regressions run.
This makes absence of the external corpus explicit rather than claiming the
incomplete generated X-provider fixture proves V1 consumer compatibility.
