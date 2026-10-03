# Robot method retirement

The former robot batch/XY method endpoints and their controls are removed. Ordinary
`/operator/v2/actions/{action_id}`, native `/protocol/execute`, command reads,
Stop/Abort, providers and shared workers retain their existing owners.

## Store upgrade

Migration **15**, `robot_method_retirement_v15`, follows the frozen v1–v14 registry.
It takes the existing exclusive lifecycle lock, verifies the registered prefix,
makes a verified SQLite backup, then rebuilds the four affected command/feed/
idempotency tables without method association or batch-group columns. Method
parent and pagination tables, their indexes, FKs and exclusive triggers disappear.
Surviving command identity, authority, immutable history and resource triggers
are reinstalled, with only removed-column comparisons deleted. The exact union
schema manifest and migration source digest attest the result. Historical
source-wrapper interrupt rows keep their already accepted, exact schema variant.

Run the upgrade only with the service quiesced. This is database maintenance,
not an additional runtime motion prerequisite. No live database was changed by
implementation or offline rehearsal.

## Nonempty restored history

Original parent rows, associations, method idempotency receipts and pagination
snapshots are retained verbatim as JSON in immutable `runtime_retired_records`.
It has no runtime writer, scheduler or replay path. Surviving command rows and
version snapshots remain; historical JSON is not rewritten. Explicit command
detail reads include `retired_method_history`, and existing method association
fields remain available for historical commands. Ordinary new command receipts
keep their existing null association fields.

Restored queued batch children are cancelled/cleared with a no-delivery retirement
reason; the original queued rows remain in the archive. They must never become
new standalone actions merely because their batch owner was retired. Existing
ordinary queued commands are unchanged. Active execution requires quiescing
before migration, using the existing maintenance boundary. No physical replay,
automatic homing or cleanup is performed.

Receipt retention excludes archived method children; it cannot discard their
source evidence. Full history/report version snapshots remain unchanged.

## Qualification

`tests/test_method_retirement_v15.py` exercises fresh creation, nonempty completed
and queued restored history, reopen, immutable archives and a private copy of a
real retained v14 store (`METHOD_RETIREMENT_RETAINED_DB`). It compares every
surviving table's rows and all frozen ledger rows. `test_method_retirement_connected.py`
exercises mounted route/schema removal and ordinary XY through the real provider,
production primitive adapter and SQLite receipts with only controller leaves
replaced. Run with `-p tests.z_stop_offline_guard`; retained provider tests also
require `DECK_RETAINED_BASELINE`.

Scoped naming changes use `XY_ACTIONS` and `queued_action` internally and replace
recovery-label adjectives. Pydantic strict typing, source attribution and existing
public recovery paths are not renamed or relaxed.
