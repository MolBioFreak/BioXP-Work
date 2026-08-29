# BioXP Runtime Audit Storage and Reporting Specification

**Status:** Controlling implementation and acceptance specification; implementation incomplete
**Date:** 2026-08-20; reassessed 2026-08-21
**Scope owner:** Christian
**Systems:** BioXP serial 206 robot runtime and BioModStack
**Primary authority:** Robot-local SQLite runtime audit store

## 1. Purpose

This specification defines durable command and pipette audit storage, evidence retention, migration, reporting APIs, the BioModStack read-only relay, and the cockpit report surface.

It closes the storage and reporting part of G-022 in `docs/specs/2026-08-02-pipette-oem-gap-rectification-spec.md`. The OEM pipette specification remains authoritative for command semantics, transport truth, controller evidence, physical admission, and commissioning. This document is authoritative for runtime audit ownership, database structure, migration, evidence lifecycle, correlation, report behavior, and retention.

Implementation must follow this document before any source candidate can be committed, deployed, or used for a physical pipette command.

## 2. Scope

### 2.1 Included

- all robot operator commands;
- direct and operator-mediated pipette operations;
- every public, protocol, lifecycle, diagnostic, callback, and application-level pipette entrypoint;
- readbacks, admission failures, timeouts, controller failures, callbacks, pressure evidence, and lifecycle events;
- command, pipette, channel, and CAN/Novo correlation;
- indefinite compact metadata retention;
- five-year full-evidence retention;
- migration from the existing operator store and pipette JSONL journal;
- versioned robot-local SQLite schema;
- crash-safe evidence-file handling;
- backup, restore, integrity, and capacity gates;
- bounded robot report APIs;
- strict BioModStack read-only relay models;
- cockpit filters, summaries, trends, detail, pagination, and export;
- archival and retirement of the unused workstation BioXP command database.

### 2.2 Excluded

- Y-axis implementation, remediation, and acceptance;
- new pipette command semantics;
- physical pipette execution or wet commissioning;
- changes to OEM TX, RX, ACK, or completion semantics;
- redesign of host CPU, RAM, or GPU telemetry;
- moving audit authority to BioModStack;
- using `/mnt/BioModStack/bioxp/commands.sqlite3` as an authority;
- merging protocol-job lifecycle data into the runtime audit store.

### 2.3 Authority limits

This specification authorizes documentation only. It does not authorize code changes, tests, database migration, service restart, deployment, CAN traffic, robot motion, or pipette action.

## 3. Store ownership

### 3.1 Canonical robot store

The sole runtime audit database is:

`/var/lib/bioxp-oem-runtime/bioxp_runtime.db`

The governed full-evidence roots are:

- `/var/lib/bioxp-oem-runtime/operator_evidence/`
- `/var/lib/bioxp-oem-runtime/pipette/evidence/`

Existing evidence remains in its current governed root. A later migration may consolidate roots only through a separate hash-verified migration receipt.

Operator commands, pipette operations, channel observations, transport exchanges, transitions, assessments, migration receipts, evidence objects, evidence links, export receipts, and retention receipts belong to this store.

### 3.2 Path invariant

All runtime audit writers and readers must resolve one canonical state root before opening SQLite. They must use one shared path resolver.

- `BIOXP_OEM_RUNTIME_STATE_ROOT` is the supported override for isolated development and recovery fixtures.
- `BIOXP_OEM_RUNTIME_ROOT` is a legacy alias. It is accepted only when it resolves to the same canonical path as `BIOXP_OEM_RUNTIME_STATE_ROOT` or when the primary variable is absent.
- `BIOXP_PIPETTE_RECEIPT_ROOT` may identify the legacy JSONL source. It must never select a second runtime database.
- A conflicting root configuration must stop API readiness before any command or report route is mounted.
- `/tmp` and user-home fallback paths are prohibited in a managed runtime.
- Status must expose the canonical database path, schema version, store identity, and evidence root without exposing credentials.

### 3.3 Other databases

| Store | Writer and schema owner | Readers | Purpose | Retention and backup | Reviewed row state | Authority and disposition |
|---|---|---|---|---|---|---|
| `/var/lib/bioxp-oem-runtime/bioxp_runtime.db` | Robot runtime audit coordinator | Robot API; BMS through robot API | Command, transition, pipette, channel, transport, evidence, assessment, migration, health, and export audit | Sections 8, 11, and 12 | Must be inventoried before migration | Sole runtime audit authority |
| `/mnt/BioModStack/bioxp/jobs.sqlite3` | BMS BioXP protocol job store | BMS API and project/run surfaces | Compiled protocol jobs and job events | Existing BMS job policy | Last review found zero jobs and zero events | Separate operational store; retained |
| `/home/dalab/.biomodstack-dev/bioxp/jobs.sqlite3` | BMS development protocol job store | Development BMS | Development job and event state | Development policy | Last review found zero jobs and zero events | Separate development store; retained |
| `/mnt/BioModStack/telemetry/telemetry.sqlite3` | BMS telemetry collector | Telemetry API and dashboards | CPU, RAM, GPU, and generic time-series samples | Existing telemetry policy | Live count must be inventoried before release | Separate telemetry authority; no audit role |
| `/mnt/BioModStack/bioxp/commands.sqlite3` | No current writer found | None found | Abandoned command-receipt prototype | Preserve read-only pending Section 17 | Last review found zero receipts | Non-authoritative; archive then retire with explicit authorization |

BioModStack must not maintain a writable replica of the robot audit database. Every pre-release database inventory records writer, reader, schema owner, purpose, retention, backup, row count, integrity result, authority class, and disposition.

### 3.4 Physical-execution boundary

- Pipette controls remain in the cockpit beneath the door card.
- Physical pipette actions and wet commissioning remain excluded until Christian authorizes a named campaign.
- The assistant must not activate `Activate 24 V / Prepare Motion`, home the machine, press physical controls, or execute a physical pipette mutation for storage or reporting acceptance.
- Christian is the physical tester.
- `Load Tip` remains `mode:"plan_only"`, `execution_admitted:false`, with blocker `physical_pipette_execution_not_authorized`.
- Snapshot collection and report acceptance remain passive and query-only.
- BMS active probes remain status-only.
- Snapshot refresh uses the same exclusion discipline as the shared runtime owner.

### 3.5 Closed pipette control denominator

HTTP method does not define hardware effect. `GET /liquid/data`, `GET /liquid/pressure`, `GET /liquid/condition`, and `GET /liquid/status/readback` can issue CAN queries. `POST /liquid/fluid-detection` can be plan-only or physical according to its typed request and admission. Every control is classified from its final transport behavior.

The final API must align method with effect. Projection-only status and report reads remain GET and perform no database write, CAN query, snapshot refresh, or other hidden collection. A hardware query, claim creation, assessment, export generation, or other receipt-producing operation uses POST. Existing hardware-query GET routes are replaced by typed POST routes and retired with an explicit compatibility response. They are not retained as unlogged aliases.

Required classes are:

- `projection_only`: reads cached robot state and sends no CAN or motion command;
- `hardware_query`: sends a bounded query and does not request a physical liquid-handling change;
- `pipette_state_command`: changes pipette initialization, speed, heartbeat, stream, tip, or controller state;
- `physical_liquid_command`: aspirate, dispense, mix, air, eject, or fluid-detection behavior with possible physical effect;
- `machine_composite`: combines pipette work with gantry, X, Z, deck, door, waste, plate, strip, or location behavior;
- `interrupt`: terminate, abort, or emergency behavior;
- `callback_event`: unsolicited pressure, completion, error, heartbeat, or lifecycle evidence.

A generated source inventory must cover every row in these current source families:

| Family | Required denominator |
|---|---|
| OEM `PagePipetteControl` | Aspirate; Dispense; Dispense All; Query Last Error; Diagnoses; Initialize; Eject; Load Tip; Move to Waste; Detect Fluid; plunger-low Set; Plunger Up; Plunger Down; Read Pressure; pressure logging; Get Data; channel, tip-type, lift-Z, and overpress inputs. Source: `PagePipetteControl.cs:12-108,226-368`. |
| OEM `ClassPipette` | command send; initiate and group initiate; data sweep; dispense all; mix; eject; terminate; speed; dispense; aspirate and air; status, error, diagnostic, tip, firmware, and pressure queries; fluid detection; `processMessage`; pressure stream; heartbeat; error logging. Source: `ClassPipette.cs:165-795`. |
| OEM `ClassPipetteCollection` | pressure stream/read/offset; diagnoses and data; group initiation and reinitialization; status and condition; speed; aspirate, dispense, mix, and air variants; eject, keep, verify, and load tip; terminate; tip query; fluid detection and timestamps; completion waits; heartbeat and error callbacks. Source: `ClassPipetteCollection.cs:240-1480`. |
| Robot direct routes | Every `/liquid/*` route in the final mounted route table, including projection, readback, initialization, tip, eject, keep, liquid, air, mix-all, fluid, terminate, heartbeat, diagnostic, error, data, pressure, firmware, reinitialize, condition, and status-readback routes. |
| Operator controls | Every generated action or semantic alias whose final path, provider, or application plan can reach pipette, liquid, waste, plunger, tip, fluid, pressure, initialization, or termination behavior. |
| Protocol execution | Every `ProtocolActionKind` handler that reaches pipette initialization, tip, eject, aspirate, dispense, mix, liquid-adjust, or imported OEM XML macro behavior. |
| Application lifecycle | Constructor pipette stage, BoardTest and diagnostic paths, `ControlLib.initializeMotion`, `initPipette`, startup tip remediation, calibration, fluid-level workflows, shutdown, reconnect, and recovery. |
| Async transport | Novo transaction, semantic ACK, delayed completion, completion-before-ACK taint, duplicate or late completion, Q1 error state, pressure stream, heartbeat, fluid timestamp, and pipette error callback. |
| BMS and cockpit | Every fixed BMS relay, typed client call, mounted button, planner action, readback action, and report or export action that reaches one of the robot rows above. |

Each inventory row records source file and line, public method or path, final callable, OEM anchor, control class, transport effect, physical-effect possibility, durable-claim requirement, mutation gate, correlation fields, report projection, BMS exposure, cockpit control, and verification ID.

The inventory is generated from the final mounted route table, protocol handler registry, operator catalog, transport public methods, lifecycle callers, and frontend action lists. A manually maintained allowlist is supporting evidence only. Any discovered command-producing row without a classified inventory record blocks RA-2, RA-4, and RA-6.

## 4. Durability before side effects

### 4.1 Universal command claim

Every transport-producing pipette path, including a hardware query, and every mutation-capable robot operator command must create a durable claim before transport, controller, filesystem, or other external side effects.

The claim transaction must persist:

- robot-issued `command_id`;
- caller idempotency key;
- canonical request fingerprint;
- action or operation ID;
- entrypoint ID, caller class, and control class;
- parent protocol job and action, lifecycle stage, or operator command identity when applicable;
- expected ownership generation;
- connection generation when supplied by BioModStack;
- safety class;
- admission state;
- request time;
- source and deployment identity;
- initial `reserved` transition.

SQLite unavailability, schema mismatch, state-root ambiguity, disk-pressure refusal, or claim conflict must reject before the first CAN, Novo, USB, motion, filesystem, or other external side effect.

### 4.2 Idempotency

- Claim reservation is atomic.
- Same key and same canonical request joins or replays the current durable command.
- Same key and different request returns conflict.
- Public POST callers supply an idempotency key. Robot-owned lifecycle callers derive one from the durable lifecycle attempt identity. A server-generated key is returned only for a single explicit query attempt and must not be treated as caller replay protection.
- A replay must verify the current robot and ownership generation before returning an old terminal result.
- An interrupted `reserved`, `admitted`, or `dispatched` command becomes `outcome_unknown` or `reconciliation_required` after restart. It is never automatically retried.
- Safety interrupts use the existing non-replayable policy, but they still require a durable attempt receipt or the existing bounded emergency fallback when SQLite is unavailable.

### 4.3 Terminal publication

Transport execution updates the existing claim. It does not create a separate uncorrelated receipt.

Terminal publication must preserve:

- delivery attempted and delivery verified as separate facts;
- controller ACK;
- completion;
- hardware precondition and postcondition;
- physical effect truth;
- failure, timeout, or ambiguity reason;
- requested and effective inputs;
- final transport and provider evidence bindings;
- final transition time.

If terminal publication fails after dispatch, the durable claim remains nonterminal. Restart reconciliation must publish `outcome_unknown`. The API must never return a retry-safe failure shape for that condition.

## 5. Correlation contract

### 5.1 Canonical identity chain

One command identity must span:

1. BioModStack invocation;
2. robot durable command claim;
3. operator dispatch context;
4. direct route, protocol action, lifecycle stage, or callback session;
5. pipette operation;
6. per-channel observation or pressure-stream session;
7. CAN/Novo exchange;
8. controller ACK and completion;
9. operator assessment;
10. report and export rows.

The robot-issued `command_id` is canonical. BioModStack carries it after the robot returns the durable claim. BioModStack does not invent a second command identity.

### 5.2 Required relationships

- Each operator-mediated pipette operation has a non-null `command_id` foreign key to the command projection.
- Direct `/liquid/*` operations create the command claim themselves and therefore also have a non-null `command_id`.
- Each pipette operation has one immutable `pipette_operation_id`.
- Every channel observation and transport exchange references `pipette_operation_id` and `command_id`.
- An action ID is copied from the trusted operator dispatch context. It is never inferred from a transport result.
- BMS connection generation and robot ownership generation remain separate fields.
- Protocol job ID, protocol action ID, lifecycle stage ID, entrypoint ID, and caller class remain separate typed fields.
- Cross-layer identifiers are returned in report details and governed exports.

## 6. SQLite schema contract

### 6.1 Schema control

The database uses an ordered migration ledger and a monotonic `PRAGMA user_version`. `CREATE TABLE IF NOT EXISTS` without exact attestation is not a migration.

Required schema-control tables:

- `runtime_metadata`;
- `runtime_schema_migrations`;
- `runtime_store_identity`;
- `runtime_migration_receipts`.

Each migration row contains its ordered version, name, source revision, applied time, canonical DDL digest, pre-migration backup receipt, and post-migration integrity receipt.

Readiness must attest exact tables, columns, indexes, foreign keys, CHECK constraints, triggers, migration order, and DDL digests.

Every table declared append-only in this specification must have database-enforced `BEFORE UPDATE` and `BEFORE DELETE` rejection triggers. Its parent foreign keys use `ON DELETE RESTRICT`. This rule applies to command transitions, channel observations, transport exchanges, evidence links, evidence events, runtime events, pressure chunks, assessments, migration receipts, and terminal export receipts. Application convention is not enforcement.

### 6.2 Command projection

`operator_commands` remains the current command projection. It must include:

- sequence;
- command ID and idempotency key;
- canonical request SHA-256;
- action or operation ID;
- entrypoint ID, caller class, control class, and parent caller identities;
- command kind;
- status;
- safety class;
- connection and ownership generations;
- created, admitted, dispatched, finished, and last-updated times;
- delivery, ACK, completion, postcondition, and physical-effect truth;
- compact requested and effective inputs;
- response summary;
- source and deployment identity;
- current evidence state.

The projection may update through guarded state transitions. Direct deletion is prohibited.

### 6.3 Immutable command transitions

`operator_transitions` is append-only. Each row includes command ID, ordered transition ID, source state, target state, observed time, reason code, and bounded detail JSON.

SQLite must reject UPDATE and DELETE with `BEFORE UPDATE` and `BEFORE DELETE` triggers. The command foreign key uses `ON DELETE RESTRICT`.

### 6.4 Pipette operations

`pipette_operations` contains one row per attempted pipette action or query. Required fields include:

- pipette operation ID;
- command ID;
- action ID when operator-mediated;
- entrypoint ID, caller class, control class, protocol job or action ID, and lifecycle stage ID when applicable;
- operation name and operation class;
- request and effective inputs;
- created, dispatched, and finished times;
- connection and ownership generations;
- delivery, ACK, completion, precondition, postcondition, and physical-effect truth;
- outcome and failure code;
- source and deployment identities;
- evidence state.

The attempt row exists before transport. Terminal facts are published with guarded state transitions.

### 6.5 Per-channel observations

`pipette_channel_observations` supports multiple observations per receipt and channel. Its primary identity is `observation_id`, not `(receipt_id, channel)`.

Required fields:

- observation ID;
- command and pipette operation IDs;
- channel `0..3`;
- phase: `precondition`, `query`, `ack`, `completion`, `postcondition`, `error`, or `callback`;
- observed time;
- semantic validity;
- truth source;
- tip loaded;
- pressure and units;
- status and error code;
- firmware or diagnostic class where applicable;
- bounded typed detail JSON.

Normalization must follow the actual producer payloads. Nested `tip`, `pressure`, `status`, `firmware`, `data`, `hardware_tip_status`, and `hardware_pressure` objects must be parsed explicitly. Missing fields remain null. They must not be invented.

### 6.6 CAN/Novo exchanges

`pipette_transport_exchanges` contains one coherent row per transaction or transaction phase. Required fields include:

- exchange ID;
- command and pipette operation IDs;
- channel;
- transaction phase;
- command family and matcher name;
- TX ID, DLC, and bounded bytes;
- expected RX ID;
- observed RX ID, DLC, and bounded bytes;
- router generation;
- sent, received, ACK, and completion times;
- delivery, semantic match, ACK, and completion truth;
- multipart sequence identity and frame order;
- bounded raw exchange JSON.

The normalizer must map the current producer field `observed_rx_id`. The store must enforce `expected_rx_id = tx_id | 0x400` when both values are present. A mismatch is an integrity failure, not a nullable success.

Parent provenance and child completion data must be joined by their transaction identity. They must not become unrelated rows merely because they occur in separate nested objects.

Completion before a semantic ACK remains tainted. It must not be reported as controller-confirmed execution.

### 6.7 Evidence objects

`runtime_evidence_objects` is the durable metadata authority for full evidence files. It contains:

- evidence ID;
- evidence-root identity;
- evidence kind and schema;
- root-relative path while retained;
- SHA-256 and byte count;
- created time;
- immutable `retention_until`;
- state: `publishing`, `retained`, `expiry_pending`, `expired`, `missing`, or `integrity_failed`;
- legal-hold state, reason, actor, and time;
- publication and expiry receipt IDs.

The digest, size, kind, schema, creation time, retention deadline, and expiry receipt remain indefinitely. Expiry clears the active path after verified deletion. It does not erase the historical digest or size.

`runtime_evidence_links` is an append-only many-to-many mapping from evidence objects to commands, pipette operations, channel observations, exchanges, runtime events, pressure streams, pressure chunks, transitions, assessments, migrations, and exports. Shared evidence is stored once. It is not deleted while any active legal hold applies.

`runtime_evidence_events` is append-only. It records ordered publication, binding, verification, legal-hold, expiry-pending, deletion, expiry, missing, integrity-failure, and reconciliation events. The mutable evidence-object projection must be reproducible from these events.

### 6.8 Runtime and pipette events

`runtime_events` is append-only. It stores unsolicited and application-level events that are not command projections. Required fields include event ID, source class, event kind, command or pipette-operation link when known, channel, transaction or stream-session identity, reader and ownership generation, observed time, ordered source sequence, semantic validity, bounded typed payload, and evidence links.

`ControlLib.errorEvent`, `pipetteError`, Novo router errors, Q1 error-state events, completion-before-ACK taints, duplicate or late completions, callback delivery failures, lifecycle errors, and audit-writer overflow remain distinct event kinds. One event type must never be rewritten as another.

### 6.9 Pressure-stream sessions and chunks

`pipette_pressure_streams` stores one start-to-stop or start-to-loss session. It includes stream session ID, parent command ID, selected channels, sample period, start and stop times, source and reader generation, offset-calculation identity, terminal state, loss count, and evidence links.

`pipette_pressure_chunks` is append-only compact metadata. Each row includes stream session ID, channel, chunk sequence, first and last sample sequence, first and last time, sample count, lost-sample count, units, raw and corrected minimum, maximum, mean, offset identity, chunk schema, byte count, SHA-256, and evidence-object link. `(stream_session_id, channel, chunk_sequence)` is unique.

Raw pressure samples use a typed chunk-evidence schema. Each sample contains channel, sample sequence, controller timestamp when available, robot receive time, raw pressure, offset, corrected pressure, units, validity, and transport exchange identity when available. Full sample chunks follow the five-year evidence policy. Stream and chunk summaries remain indefinitely.

Discrete pressure queries remain typed channel observations and are retained as compact metadata. Report trends read retained sample chunks when available and return the exact `expired`, `missing`, or `integrity_failed` evidence state after full sample expiry. Silent downsampling is prohibited.

### 6.10 Export receipts

`runtime_export_receipts` is append-only. It records export ID, requester class, format, schema version, canonical filter digest, snapshot boundary, row counts, byte count, SHA-256, created time, expiry or evidence binding, and terminal outcome.

### 6.11 Assessments

Operator assessments are append-only rows. They do not rewrite the command creation time, evidence creation time, or evidence retention deadline.

## 7. Failure and event coverage

The audit store must represent every attempted in-scope operation, including:

- successful execution;
- admission rejection after a durable claim;
- preflight failure;
- transport unavailable;
- TX failure;
- timeout;
- malformed response;
- ACK without completion;
- completion before ACK;
- controller error;
- hardware postcondition failure;
- callback and pressure event;
- protocol and lifecycle caller identity;
- shutdown or restart ambiguity;
- persistence failure after dispatch.

Validation rejected before durable claim must be limited to malformed requests that cannot be assigned a safe canonical fingerprint. BioModStack may record its own HTTP access evidence, but that evidence does not become robot audit authority.

Receive-thread callbacks must not perform SQLite or filesystem I/O. They publish bounded in-memory event objects to the single runtime audit writer. Queue overflow, writer failure, or restart loss must latch an explicit audit-health failure that blocks new physical pipette commands until reconciled.

Direct routes, protocol handlers, operator relays, constructor stages, initialization programs, BoardTest and diagnostic callers, and recovery paths must enter the same durable command coordinator before their first transport or machine side effect. A receipt written after a direct transport call does not satisfy this requirement.

## 8. Full-evidence lifecycle

### 8.1 Retention

- Compact command, transition, pipette, channel, transport, event, pressure-stream, pressure-summary, assessment, migration, export, and evidence metadata is retained indefinitely.
- Full evidence bytes are retained for five calendar years from immutable `evidence_created_at`.
- Five calendar years means the same UTC month, day, and time in year `N+5`. February 29 expires on February 28 in a non-leap expiry year.
- Later assessments, report reads, replays, exports, or metadata updates do not move `retention_until`.
- A legal hold suspends expiry for the linked evidence object. Creating and releasing a hold requires an append-only assessment and a named operator identity. Hold release restores the original `retention_until`; it does not start a new five-year period.
- Count-based deletion is prohibited.
- Disk pressure must not shorten retention.

### 8.2 Publication

1. Create the evidence file in the canonical evidence directory.
2. Write and fsync the bytes.
3. Atomically publish the immutable filename.
4. Fsync the containing directory.
5. Bind path, digest, size, and schema in SQLite.
6. Commit the evidence transition to `retained`.

A crash between filesystem publication and database binding leaves a discoverable orphan. Startup reconciliation must hash and classify it before any deletion.

### 8.3 Expiry

Expiry is a two-phase operation:

1. SQLite marks the evidence `expiry_pending` and commits.
2. The file is unlinked and the directory is fsynced.
3. SQLite verifies absence, records an expiry receipt, clears the active path, and marks the object `expired`.

Restart reconciliation resumes `publishing` and `expiry_pending` objects. It never silently deletes an unclassified file.

### 8.4 Missing or corrupt evidence

A missing file, digest mismatch, size mismatch, symlink, non-regular file, path escape, or unreadable object returns a typed integrity state. Report APIs must not replace it with a generic unavailable value.

## 9. SQLite operation and concurrency

- All writers use one process-wide runtime audit coordinator.
- SQLite uses WAL, `synchronous=FULL`, foreign keys, a bounded busy timeout, and deliberate checkpoints.
- A passive WAL checkpoint runs at least every 60 seconds or 1,000 pages. A successful backup, migration, or clean service stop performs a truncate checkpoint after command admission is quiesced.
- A failed checkpoint raises audit health. New physical commands are blocked when the last successful checkpoint is older than 24 hours or WAL size exceeds the lower of 1 GiB and 10 percent of the audit volume.
- Full `VACUUM` is prohibited while command admission is active. `PRAGMA optimize` runs after migration and at least weekly. Any vacuum policy requires a separately measured capacity need and a backup-first maintenance window.
- Migration, claim, transition, pipette normalization, evidence binding, retention, and recovery use the same writer authority.
- A physical command is blocked when audit health is unavailable.
- Report queries execute in one read transaction so totals, summaries, and rows share one SQLite snapshot.
- Report responses include store identity, schema version, generated time, snapshot boundary, and filter scope.
- Startup must reconcile incomplete commands, incomplete evidence publication, incomplete expiry, and legacy migration before normal physical admission becomes ready.

## 10. Migration

### 10.1 Inputs

Migration covers:

- existing `operator_commands`, `operator_transitions`, `runtime_metadata`, and related robot runtime tables;
- `/var/lib/bioxp-oem-runtime/pipette/receipts.jsonl` or the explicitly resolved legacy JSONL source;
- existing operator and pipette evidence files.

### 10.2 Backup-first gate

Before DDL or import:

1. acquire the runtime migration lock;
2. stop new command admission;
3. checkpoint WAL;
4. create a SQLite online backup in the canonical backup area;
5. hash the backup;
6. inventory and hash the legacy JSONL and evidence roots;
7. write a pre-migration manifest outside the source database;
8. verify the backup opens read-only and passes `quick_check` and `foreign_key_check`.

Migration stops if any input cannot be inventoried or backed up.

### 10.3 Import contract

- Parse and validate the complete JSONL before database mutation.
- Validate receipt schema, required identity, timestamps, truth types, and source identity.
- Invalid records go to a governed quarantine artifact. They are not silently defaulted to false or unknown.
- Recheck the migration marker inside the same `BEGIN IMMEDIATE` transaction that claims the import.
- Use exact receipt IDs for idempotent replay.
- Import command links, observations, exchanges, and evidence bindings transactionally.
- Keep the source JSONL byte-identical and read-only until migration acceptance and restore proof complete.
- New runtime writes must target SQLite only after the migration transaction commits.

### 10.4 Post-migration gate

The migration is accepted only when:

- ordered migration ledger and `user_version` match;
- exact schema attestation passes;
- `quick_check` returns `ok`;
- `foreign_key_check` returns no rows;
- source and imported receipt counts match by class;
- every source receipt ID is imported or listed in quarantine;
- evidence hashes and sizes match;
- restart produces zero duplicate rows and zero new migration actions;
- the pre-migration backup restores successfully into an isolated root;
- the migrated store restores successfully into an isolated root;
- command admission remains blocked until these checks pass.

## 11. Backup and recovery

The backup unit contains:

- an online SQLite backup;
- an evidence-object manifest;
- retained evidence files or an incremental content-addressed backup reference;
- store identity and schema ledger;
- SHA-256 for every manifest and database object;
- backup time and source deployment identity.

The runtime backup policy must preserve the five-year evidence contract. Backup pruning cannot remove the only retained copy of an evidence object before `retention_until`.

Backup schedule:

- one database and evidence-manifest backup in every 24-hour period;
- one full or content-addressed evidence backup in every seven-day period;
- one additional backup immediately before migration, deployment, or schema change;
- backup failure raises audit health and blocks schema migration;
- a restore drill runs before first release and at least every 90 days after release.

Integrity schedule:

- `quick_check` and `foreign_key_check` run after migration, restore, and clean startup;
- `quick_check` runs at least daily;
- full `integrity_check` runs at least weekly in a bounded maintenance task;
- retained evidence receives a complete digest reconciliation before release and at least every 90 days;
- every failure creates a durable health or integrity receipt and blocks new physical pipette commands until resolved.

A release candidate must prove:

- database-only restore;
- database plus evidence restore;
- WAL recovery;
- interrupted command recovery;
- interrupted publication recovery;
- interrupted expiry recovery;
- legacy migration rollback;
- historical report and detail integrity after restore.

## 12. Capacity and health

SQLite row count is not a retention control. Millions of compact indexed rows are an expected operating condition.

Before deployment, capacity planning must use measured values from representative runs:

- commands per run;
- pipette operations per run;
- channel observations per operation;
- CAN exchanges per operation;
- runtime event rate by event kind;
- pressure-stream sample rate, active duty cycle, chunk size, compression ratio, and per-channel loss overhead;
- average and maximum compact row size;
- average and maximum full-evidence size;
- daily and annual run counts;
- five-year evidence projection;
- WAL and backup overhead.

The projected five-year retained footprint must fit below 60 percent of the allocated audit volume. The remaining capacity covers WAL, backup, migration, and abnormal growth. If this gate fails, storage is expanded. Retention is not shortened.

Status and reports expose:

- database, WAL, evidence, and backup byte counts;
- oldest and newest records;
- retained and pending-expiry evidence counts;
- integrity failures;
- last checkpoint, backup, restore drill, migration, and retention sweep;
- audit writer health and queue depth;
- free-space state.

New physical pipette commands fail closed when audit storage cannot guarantee a durable claim and terminal receipt.

## 13. Robot reporting APIs

### 13.1 General rules

- list, summary, detail, health, and completed-export download routes use GET and perform zero database or filesystem writes;
- governed export creation uses POST and may append only its export receipt and evidence binding;
- robot-authoritative;
- GET report queries use one read-only SQLite transaction;
- export generation uses one read snapshot, then a separate narrow transaction for its receipt and evidence binding;
- fixed route registry;
- bounded limits;
- deterministic keyset pagination;
- strict typed response schemas;
- no filesystem paths in public list responses;
- exact evidence state and truth fields;
- typed integrity failures.

Interactive list and summary requests default to a 24-hour window and permit at most a 31-day window. Historical access uses bounded adjacent windows. The default page size is 100 and the maximum is 1,000. Detail requests return one identity and its bounded child pages. Export permits at most a 31-day window and 100,000 rows per request.

List, summary, and detail queries have a five-second robot-side deadline. Export has a 30-second generation deadline. Overload, timeout, or bound violations return a typed 422, 429, or 503 result with no partial success body. Acceptance must prove indexed query plans against the projected five-year row count. A maximum-bound interactive query must not depend on an unbounded full-table scan.

### 13.2 Required routes

- `GET /operator/reports/summary`
- `GET /operator/reports/commands`
- `GET /operator/reports/commands/{command_id}`
- `GET /operator/reports/commands/{command_id}/transitions`
- `GET /operator/reports/pipette`
- `GET /operator/reports/pipette/{pipette_operation_id}`
- `GET /operator/reports/pipette/{pipette_operation_id}/channels`
- `GET /operator/reports/pipette/{pipette_operation_id}/exchanges`
- `GET /operator/reports/events`
- `GET /operator/reports/events/{event_id}`
- `GET /operator/reports/pressure-streams`
- `GET /operator/reports/pressure-streams/{stream_session_id}`
- `GET /operator/reports/pressure-streams/{stream_session_id}/samples`
- `POST /operator/reports/exports`
- `GET /operator/reports/exports/{export_id}`
- `GET /operator/reports/exports/{export_id}/download`
- `GET /operator/audit-health`

A combined dashboard projection may exist. It does not replace these independently pageable resources.

### 13.3 Filters

The command and pipette resources support:

- inclusive start and exclusive end time;
- action or operation;
- entrypoint, caller class, control class, protocol job or action, and lifecycle stage;
- status and outcome;
- channel;
- event source and event kind;
- pressure-stream session;
- delivery verified;
- controller acknowledged;
- completion verified;
- hardware postcondition verified;
- physical effect verified;
- evidence state;
- command ID;
- pipette operation ID;
- connection and ownership generation.

Each response echoes canonical filters. Summaries are computed from those exact filters. A window-wide summary must be labeled `scope=window`, and a filtered summary must be labeled `scope=filtered`.

### 13.4 Pagination

- Each resource has its own opaque cursor.
- Responses include `next_cursor`, `has_more`, `returned_count`, and `filtered_total` when the total can be computed within the bound.
- The first page captures high-water sequences for commands, command transitions, pipette operations, channel observations, transport exchanges, runtime events, pressure streams, pressure chunks, evidence events, and assessments.
- Cursors bind immutable sort values, canonical filter digest, store identity, schema version, and every high-water sequence.
- Later pages evaluate truth as of those immutable event boundaries. They do not use current mutable projection values to reclassify an earlier snapshot.
- Cursors expire after 15 minutes. An expired, changed-filter, changed-store, or changed-schema cursor returns typed conflict and never restarts silently at page one.
- Mutable projection updates do not cause omissions or duplication within one paginated snapshot.

### 13.5 Detail and export

Detail responses expose requested and effective inputs, entrypoint and parent-caller identities, transitions, channel observations, transport exchanges, runtime events, pressure streams and samples, source and deployment identity, and evidence descriptors.

Export creation supports governed JSON and CSV. It records an export receipt with filter digest, snapshot boundary, row counts, schema, and SHA-256. The metadata and download GET routes read that completed receipt and perform no mutation. Export does not expose credentials, absolute paths, or unrestricted raw evidence.

## 14. BioModStack relay

BioModStack is a strict read-only consumer.

- It uses fixed robot route names.
- It forwards only typed, bounded filters.
- It does not write or cache authoritative command rows in a BioModStack SQLite database.
- It may request robot-owned export generation through the fixed POST route. It does not alter command, pipette, event, pressure, assessment, or evidence truth and stores no local authoritative export receipt.
- It validates every robot response with closed models.
- `summary`, `retention`, `window`, page metadata, command rows, pipette rows, channel rows, exchange rows, event rows, pressure-stream and sample rows, evidence descriptors, and integrity errors use named strict models. Generic `dict[str, JsonValue]` contracts are prohibited for authority-bearing report fields.
- Robot 4xx and 5xx states retain safe structured reason codes.
- A post-dispatch robot response validation error is reported as `robot_may_have_executed`; BioModStack does not invite blind retry.
- Generation and store identity remain visible to the cockpit.

Required BioModStack routes mirror the robot report resources under `/api/bioxp/operator-controls/reports/`.

## 15. Cockpit reports UI

The pipette cockpit includes a read-only Runtime and Pipette Reports surface.

Required controls and views:

- time range;
- action and operation filters;
- status and outcome filters;
- channel filter;
- delivery, ACK, completion, postcondition, and physical-truth filters;
- evidence-state filter;
- manual refresh;
- deterministic next and previous page controls;
- command and pipette summary counts;
- ACK, completion, postcondition, and failure rates;
- latency and pressure trends with explicit units;
- error-code frequency;
- command detail;
- transition timeline;
- per-channel observation detail;
- CAN/Novo TX, expected RX, observed RX, ACK, and completion detail;
- source-distinct event timeline for `ControlLib.errorEvent`, `pipetteError`, Q1, completion taints, callbacks, and lifecycle errors;
- pressure-stream session, sample-loss, offset, units, and per-channel trend detail;
- evidence retained, expired, missing, and integrity-failed state;
- governed JSON and CSV export.

Freshness rules:

- successful commands, assessments, readbacks, migrations, and retention state changes invalidate report queries;
- mounted reports refetch on explicit refresh and selected-window change;
- stale cached data remains visibly labeled with its generated time and store snapshot;
- a failed refetch cannot leave an unlabeled current-looking report;
- old robots that lack the report contract render an explicit version incompatibility, not an empty report.

Boolean evidence must use truthful labels such as `Delivered`, `Controller ACK`, `Completion`, and `Physical effect verified`. Raw `true/false` triplets are not sufficient operator presentation.

## 16. Security and redaction

- Credentials, tokens, connection strings, private keys, and secret-bearing headers never enter audit rows, evidence files, reports, exports, or migration artifacts.
- Redaction uses a closed allowlist for public projections and a deny policy for full evidence.
- Absolute filesystem paths remain robot-local.
- Evidence retrieval validates confinement, regular-file status, symlink exclusion, size, and SHA-256.
- Report filters are parameterized. Dynamic SQL identifiers are prohibited.
- Export sizes and date windows are bounded.

## 17. Unused workstation command database retirement

Target:

`/mnt/BioModStack/bioxp/commands.sqlite3`

Retirement steps:

1. open read-only;
2. record schema, row counts, file size, modification time, SHA-256, and `quick_check`;
3. search current `test` and `main` source, managed service configuration, process maps, and backups for ownership references;
4. prove no active process opens or writes it;
5. create a hash-bound archival copy and receipt;
6. remove runtime references and obsolete source, tests, or documentation;
7. remove the active file only after Christian authorizes destructive retirement;
8. verify robot reports remain authoritative and BMS has no writable command-store fallback.

An empty database is not proof that deletion is safe. It remains preserved until this gate completes.

## 18. Original requirements and candidate ledger

This ledger records the pre-implementation baseline. Section 24 is the binding current-candidate reassessment and supersedes the candidate-state and defect columns below. The approved requirements and closing gates remain controlling.

Allowed candidate states are `absent`, `partial`, `implemented_unverified`, and `verified`. A source change without the required test, migration, deployment, and acceptance evidence cannot be `verified`.

| ID | Approved requirement | Candidate implementation | Confirmed defect or omission | Required remediation | Closing gate |
|---|---|---|---|---|---|
| RAQ-001 | Robot `bioxp_runtime.db` is the sole audit authority. | `partial` | Operator and pipette defaults can resolve different databases. A legacy pipette override can select another database. | Use one resolver and reject conflicting or managed-runtime fallback roots. | RA-1 |
| RAQ-002 | Compact command and pipette metadata remains indefinitely. | `implemented_unverified` | The 512-row deletion was removed, but capacity, integrity, backup, and disk-full behavior were not closed. | Keep count deletion removed and implement Sections 9, 11, and 12. | RA-2, RA-7, RA-11, RA-12 |
| RAQ-003 | Every operator, pipette, channel, transport, event, and pressure record has typed correlation. | `partial` | Pipette persistence does not consume the trusted operator dispatch context. Several typed fields are inferred from incompatible result payloads. Protocol/lifecycle identity and typed async event/pressure records are absent. | Implement the identity chain and foreign keys in Sections 5 and 6. | RA-6 |
| RAQ-004 | A durable idempotent claim exists before every pipette side effect. | `absent` | Current pipette transport runs before SQLite persistence. A persistence failure can leave physical execution unrecorded and retryable. | Implement Section 4 before enabling any physical pipette command. | RA-4 |
| RAQ-005 | Success, rejection, timeout, malformed response, controller error, and ambiguity are durable. | `partial` | Several service failures occur before pipette receipt creation. Terminal persistence failure has no durable ambiguity path. | Implement universal attempt rows, guarded transitions, and restart reconciliation. | RA-5 |
| RAQ-006 | Legacy pipette JSONL migrates once, safely, and with full reconciliation. | `partial` | Prototype import has no backup-first gate, schema attestation, full validation, quarantine, source digest ledger, rollback, or cross-process-safe claim. | Implement Section 10 as a versioned migration. | RA-3 |
| RAQ-007 | Full evidence remains for five calendar years while compact bindings remain indefinitely. | `partial` | Operator expiry uses mutable command `updated_at`. Pipette expiry and publication are not crash-safe. No legal-hold behavior exists. | Use immutable evidence creation and retention times, legal holds, two-phase expiry, and reconciliation. | RA-7 |
| RAQ-008 | Evidence integrity is verifiable across publication, retention, report, backup, and restore. | `partial` | Path, SHA-256, and byte count exist in parts of the prototype. Directory fsync, orphan reconciliation, periodic verification, and append-only link metadata are missing. | Implement Sections 6.7, 8, 11, and 16. | RA-7, RA-11 |
| RAQ-009 | Database and evidence backup and restore are operational requirements. | `absent` | No complete backup unit, schedule, restore drill, or database-to-evidence reconciliation exists. | Implement Section 11 and prove both restore paths. | RA-11 |
| RAQ-010 | WAL, checkpoint, integrity, storage growth, and disk-full behavior are governed. | `partial` | WAL mode exists. Checkpoint ownership, capacity gate, periodic integrity work, health thresholds, and fail-closed disk behavior are absent. | Implement Sections 9 and 12. | RA-2, RA-12 |
| RAQ-011 | Robot reports are bounded, snapshot-consistent, typed, filterable, and independently pageable. | `partial` | Prototype queries use multiple autocommit snapshots. Summaries ignore row filters. Dual cursors lack continuation metadata. Pipette/channel/exchange detail and export are absent. | Implement Section 13. | RA-8 |
| RAQ-012 | BioModStack is a strict read-only relay. | `partial` | Fixed proxy routing exists. Important nested response fields remain generic dictionaries and have no contract acceptance. | Implement closed models and safe upstream error semantics in Section 14. | RA-9 |
| RAQ-013 | Cockpit reports provide filters, drill-down, trends, freshness, pagination, and governed exports. | `partial` | Prototype has a time selector and compact rows. Required filters, detail, trends, pagination, exports, and mutation-driven invalidation are absent. | Implement Section 15 beneath the existing door card. | RA-10 |
| RAQ-014 | The unused workstation command database has a governed disposition. | `absent` | `/mnt/BioModStack/bioxp/commands.sqlite3` is empty and has no current writer, but remains unexplained. | Complete the read-only owner audit, archive it, then obtain explicit deletion authorization. | RA-13 |
| RAQ-015 | Existing physical-execution boundaries remain unchanged. | `implemented_unverified` | The reporting prototype did not establish physical acceptance. Storage work cannot authorize `Load Tip` or wet mutation. | Preserve Section 3.4 and perform no physical action for RA acceptance. | RA-14 plus separate physical authorization |
| RAQ-016 | Completion requires source review, automated tests, migration tests, deployment identity, API acceptance, restore proof, and browser acceptance. | `absent` | Candidate tests, builds, migrations, commits, pushes, deployments, restarts, and browser acceptance are all zero. Existing tests directly contradict parts of the prototype. | Complete RA-WP0 through RA-WP6 in order against one exact candidate. | RA-0 through RA-14 |
| RAQ-017 | Every OEM, robot, protocol, lifecycle, operator, callback, BMS, and cockpit pipette entrypoint is classified and governed. | `absent` | Current source has direct `/liquid/*`, protocol, constructor, initialization, diagnostic, recovery, and callback paths. The prototype did not establish a generated closed-world inventory or prove that each path enters the durable coordinator before transport. | Implement the generated denominator and bypass-denial contract in Section 3.5. | RA-2, RA-4, RA-6 |

## 19. Acceptance matrix

| Gate | Required result |
|---|---|
| RA-0 | This specification is tracked, hashed, independently reviewed, and approved. |
| RA-1 | One canonical state-root resolver and fail-closed path invariant. |
| RA-2 | Versioned schema, exact attestation, append-only triggers, shared writer authority, and a zero-missing-row pipette entrypoint inventory. |
| RA-3 | Backup-first migration passes counts, hashes, quarantine, restart, and rollback. |
| RA-4 | Every transport-producing pipette path, including queries and alternate callers, creates an idempotent durable claim before transport. |
| RA-5 | Success, rejection, timeout, failure, callback, pressure-stream loss, and ambiguity all persist. |
| RA-6 | Entrypoint, operator, protocol, lifecycle, pipette, channel, event, pressure, and CAN/Novo identities correlate exactly. |
| RA-7 | Full evidence publication, five-year retention, expiry, orphan recovery, and integrity checks pass. |
| RA-8 | Robot summary, list, detail, transition, channel, exchange, event, pressure, health, and export APIs pass. |
| RA-9 | BMS strict relay models and error semantics pass. |
| RA-10 | Mounted cockpit filters, trends, detail, pagination, freshness, and export pass. |
| RA-11 | Backup and restore drills pass against the exact candidate schema. |
| RA-12 | Capacity gate passes for projected five-year data volume. |
| RA-13 | Unused workstation command database is archived and retired with explicit authorization. |
| RA-14 | Exact source, remote integration, immutable deployment, live read-only contract, and browser acceptance pass. |

A later gate cannot compensate for an earlier failure. Physical execution is outside this specification and cannot be used to close a storage or reporting gate.

## 20. Required verification

Implementation work must add focused tests for:

- state-root equality and conflicting override rejection;
- generated closed-world inventory over mounted routes, operator actions, protocol handlers, transport methods, lifecycle callers, callbacks, BMS relays, and cockpit actions;
- zero transport from every `projection_only` row;
- zero database or filesystem writes from every report/list/detail/health GET and every projection-only status GET;
- retirement of hardware-query GET routes in favor of typed claim-producing POST routes;
- preclaim-before-transport from every `hardware_query`, state command, physical command, machine composite, protocol, and lifecycle row;
- claim-before-transport with a transport fake that fails if reached early;
- same-key replay and changed-request conflict;
- DB busy, disk-full, serialization, filesystem, and terminal-publication failure;
- restart conversion to `outcome_unknown` with no retry;
- every success and failure receipt class;
- exact four-channel nested readback normalization;
- single-channel operation normalization;
- multiple precondition and postcondition observations per channel;
- `observed_rx_id` mapping and `expected_rx_id = tx_id | 0x400` enforcement;
- multipart transaction joining;
- source-distinct `ControlLib.errorEvent`, `pipetteError`, Q1, completion-taint, and callback-failure events;
- ordered four-channel pressure sessions, offsets, units, loss accounting, chunk digests, and restart termination;
- migration backup, exact source inventory, invalid-row quarantine, concurrent startup, restart idempotency, and rollback;
- evidence file and directory fsync ordering;
- crash points during publication and expiry;
- evidence corruption, missing file, symlink, and path escape;
- append-only trigger enforcement;
- one-snapshot report consistency during concurrent pipette writes;
- filtered summary accuracy;
- cursor stability and dual-resource pagination;
- strict BMS model rejection of malformed nested report fields;
- report invalidation after commands, assessments, and readbacks;
- mounted UI filters, stale/error transition, detail, pagination, and export;
- dead-store owner audit and non-use by current runtime;
- backup and restore of database plus evidence.

Repository tests that assert the old 512-record cap or old exact route count must be changed only after the new specification tests exist and fail for the intended reason.

## 21. Original ordered implementation packages

These packages record the initial implementation sequence. Section 25 defines the binding remaining work after the 2026-08-21 exact-tree reassessment.

### RA-WP0: Specification and migration design

- seal this document;
- freeze schema DDL and migration ledger;
- define canonical state-root resolution;
- freeze the generated closed pipette entrypoint denominator;
- define backup, capacity, and rollback artifacts.

Exit: RA-0 design review passes. No runtime database changes.

### RA-WP1: Durable claim and schema foundation

- implement migrations and attestation;
- add pre-dispatch claim and idempotency across direct, protocol, lifecycle, operator, and query callers;
- add append-only transitions and assessments;
- enforce one writer authority.

Exit: RA-1, RA-2, and RA-4 pass in isolated tests.

### RA-WP2: Pipette and transport normalization

- persist success and failure attempts;
- bind operator context;
- bind protocol, lifecycle, entrypoint, and caller context;
- normalize nested channel readbacks;
- join CAN/Novo exchanges;
- persist source-distinct runtime events and ordered pressure sessions;
- preserve exact truth phases.

Exit: RA-5 and RA-6 pass without hardware use.

### RA-WP3: Evidence lifecycle and migration

- implement crash-safe publication and expiry;
- add orphan and incomplete-state recovery;
- implement backup-first JSONL migration;
- prove rollback and restart behavior.

Exit: RA-3, RA-7, and RA-11 pass in isolated roots.

### RA-WP4: Robot reports

- implement independently pageable summary, command, transition, pipette, channel, exchange, event, pressure, health, detail, and export APIs;
- bind each response to one read snapshot.

Exit: RA-8 passes.

### RA-WP5: BioModStack relay and cockpit

- implement strict models and fixed routes;
- add complete report controls, trends, details, pagination, freshness, and exports;
- preserve robot authority.

Exit: RA-9 and RA-10 pass in development.

### RA-WP6: Capacity, dead-store retirement, and release acceptance

- measure representative storage;
- pass the five-year capacity gate;
- archive and retire the unused workstation command database after explicit authorization;
- complete exact-tree review, integration, deployment, read-only live acceptance, and browser acceptance.

Exit: RA-12 through RA-14 pass. This package authorizes no physical pipette action.

## 22. Prototype disposition

The uncommitted prototype in these worktrees is rejected as an implementation candidate and retained only as design evidence:

- `/home/dalab/worktrees/bioxp-audit-retention-20260820`
- `/home/dalab/worktrees/bms-bioxp-audit-reports-20260820`

Known blocking prototype defects include:

- transport execution before durable pipette claim;
- no durable receipt for several failure paths;
- split default database roots;
- unversioned live DDL and no backup-first migration;
- missing operator-to-pipette correlation;
- no generated closed-world pipette entrypoint inventory or alternate-caller preclaim proof;
- channel normalization that misses nested producer fields;
- `rx_id` extraction that misses `observed_rx_id`;
- no typed source-distinct runtime-event or ordered pressure-stream schema;
- non-crash-safe evidence publication and expiry;
- report queries without one read snapshot;
- mutable operator evidence retention clock;
- incomplete filters, cursor metadata, detail, export, freshness, and strict BMS models;
- direct contradictions with current route, retention, and mounted frontend tests.

No prototype code may be committed, rebased into implementation, deployed, or used as acceptance evidence. A later implementation may reuse reviewed ideas only after RA-WP0 freezes the exact contract.

## 23. Initial pre-implementation verdict

The 2026-08-20 document review closed the original RA-0 documentation gate. Later implementation and deployment work did not close every acceptance gate. Sections 24 through 26 supersede any later claim that the complete storage and reporting assignment passed.

## 24. Binding 2026-08-21 reassessment

### 24.1 Reviewed identities and evidence boundary

The reassessment used these immutable identities:

- governing specification baseline commit `bed03aaeb76399ce5d994256d5abfef93fdc804b`;
- governing specification baseline SHA-256 `00d6c6030708c7a2742f907452623ddb30a66c9af47ea67ce14db860eb0d42f8`;
- robot candidate commit `23e717aa3809776341fc4339af9c79cbca27fea9`;
- robot candidate tree `fbb12fd8a6669b829622b9e9248118946b9c37ca`;
- focused robot acceptance result: 39 tests passed;
- broad robot suite: not proven green because one run exited during collection and the isolated-state retry exceeded its time limit;
- live BMS snapshot: API and frontend shared immutable release `5c519c34022f2725626c4f8580623ecb9dd7d34a`, whose BioXP bytes matched parent `61a04f1908b4a534ca39412f700f0339004298ca`;
- BMS integration state at that snapshot: `origin/test` was `61a04f1908b4a534ca39412f700f0339004298ca`, the live child commit was not on the remote branch, and the canonical checkout had eight unrelated dirty BioXP files that were not present in the live immutable release.

The robot source review and direct live read-only inspection support the findings below. No physical pipette command, CAN mutation, motion, 24 V activation, homing, or wet commissioning was part of this reassessment.

### 24.2 Current gate ledger

| Requirement | Current state | Decisive current evidence | Work required to close |
|---|---|---|---|
| RAQ-001 sole robot SQLite authority | `partial` | The canonical SQLite store is active, but `/var/lib/bioxp-oem-runtime/pipette/receipts.jsonl` remains an active receipt sink. | Remove every production JSONL write path after a backup-first, digest-bound, idempotent migration. Fail readiness if a runtime caller can select or append a second audit authority. |
| RAQ-002 indefinite compact retention | `implemented_unverified` | Count-based truncation is absent and current command rows remain available. | Pass capacity, backup, restore, WAL, checkpoint, integrity, and disk-full gates. |
| RAQ-003 typed correlation | `partial` | Typed columns exist, but the `pipette_operations` claim insert omits connection generation, protocol job/action, and lifecycle stage fields. Protocol and lifecycle writers do not populate the canonical typed chain. | Carry trusted identities through the claim API, database insert, terminal publication, reports, BMS models, and exports. Add foreign-key and exact-chain tests. |
| RAQ-004 durable pre-side-effect claim | `partial` | The shared service claims before dispatch. `/liquid/readback`, `/liquid/init`, protocol handlers, constructor initialization, and Serial-206 lifecycle methods can reach transport outside that coordinator. | Route every generated denominator row through one coordinator before transport. Add denial fixtures that fail on the first early driver call. |
| RAQ-005 durable success and failure | `partial` | Central service failures persist. Bypass paths can fail before a claim. Typed-normalization failure can leave `reserved`; startup reconciliation closes only the operator projection. | Persist all failure classes and reconcile both operator and pipette rows to `outcome_unknown` without retry. |
| RAQ-006 JSONL migration | `partial` | A migration helper and unit fixture exist. Production startup does not invoke it. The live source contained nine legacy receipts, while `runtime_migration_receipts` contained zero rows at reassessment. | Implement one owner-elected migration with backup, source digest, count reconciliation, quarantine, rollback, restart idempotency, and a terminal migration receipt. Disable JSONL append after success. |
| RAQ-007 five-year evidence lifecycle | `partial` | Evidence tables and report projections exist. Full retention, legal hold, two-phase expiry, and recovery remain unaccepted. | Complete Section 8 and pass RA-7. |
| RAQ-008 evidence integrity | `partial` | Existing export bytes matched recorded digest and byte count. Broader evidence publication and periodic integrity closure remain open. | Add exact evidence-link receipts, confinement, corruption handling, orphan recovery, and periodic verification. |
| RAQ-009 backup and restore | `absent` | No accepted database-plus-evidence restore drill exists for this candidate. | Run both restore paths against the exact migration/schema candidate after separate test authorization. |
| RAQ-010 WAL, capacity, and disk behavior | `partial` | WAL and quick-check evidence exist. Five-year capacity and fail-closed disk behavior remain open. | Pass RA-12 with measured representative data and explicit thresholds. |
| RAQ-011 robot reports | `partial` | Live summary, lists, details, pagination, events, pressure, health, and exports render. Pipette/event/pressure summary totals are global while command metrics honor filters. | Apply one filter contract to every summary numerator and denominator. Add concurrent-write snapshot tests and exact filtered-summary fixtures. |
| RAQ-012 BMS read-only relay | `partial` | The live relay is read-only and mutation access was disabled. Exact integration and current-branch provenance did not pass. | Seal strict models against the final robot contract, integrate the exact candidate into `test`, and prove live source identity. |
| RAQ-013 cockpit reports | `partial` | The mounted report panel settled and displayed command rows, zero typed pipette rows, details, filters, and export controls. It did not reveal the nine unmigrated legacy receipts as typed pipette operations. | Render reconciled robot-authoritative data after migration and filtered-summary repair. Re-run mounted browser acceptance against the integrated release. |
| RAQ-014 workstation command database retirement | `verified` | Active `commands.sqlite3`, WAL, and SHM paths were absent. A hash-bound archived database and retirement receipts remained. No active writer, handle, source fallback, or BMS fallback was found. | Preserve receipts and recheck non-use during final release acceptance. |
| RAQ-015 physical boundary | `verified` for software acceptance | Effective BMS mutations were disabled. No reassessment action produced physical effects. | Keep this gate unchanged. Physical and wet acceptance remain separate and require Christian's named authorization. |
| RAQ-016 complete release evidence | `partial` | Focused tests and live read-only behavior passed. Broad robot tests, restore, capacity, exact BMS integration, and all earlier acceptance gates did not pass. | Complete Section 25 in order against one exact candidate and one immutable deployed release. |
| RAQ-017 closed entrypoint denominator | `partial` | The direct route snapshot contained 29 `/liquid/*` routes: 24 used the typed coordinator, two dispatched before claim, one plan route used legacy receipt recording, and two were projections. Five hardware/query routes remained effect-producing GETs. Protocol and lifecycle bypasses also remained. | Generate the final route, protocol, lifecycle, operator, callback, transport, BMS, and cockpit inventory from mounted source. Require zero missing and zero bypass rows. |

### 24.3 Binding defect list

The remaining implementation must close all defects below. One passing example does not close a caller family.

1. **Direct route bypasses:** `/liquid/readback` and `/liquid/init` dispatch before a typed claim.
2. **Protocol bypass:** the protocol pipette handler dispatches direct transport and records afterward.
3. **Lifecycle bypasses:** constructor initialization and Serial-206 query, eject, and initialize methods reach transport outside the durable coordinator.
4. **Effect-producing GET routes:** data, fluid timestamp, pressure, condition, and status readback require typed POST replacements and explicit retired-route responses.
5. **Legacy authority:** no-ID receipt calls append active JSONL. Application-plan, direct bypass, and protocol paths must stop using that fallback.
6. **Migration not wired:** production startup has no owner-elected call to the existing JSONL migration.
7. **Incomplete typed identities:** connection generation, protocol job/action, lifecycle stage, and callback session must be inserted as typed columns rather than buried in untyped source JSON.
8. **Idempotent replay hole:** replay of an existing operator command must reject or repair a missing pipette child. It must never synthesize a nonexistent `pipette_operation_id`.
9. **Misleading terminal status:** `acknowledged` requires `controller_acknowledged=true`. A non-completed, non-ACK result uses an exact state such as `dispatched`, `outcome_unknown`, or `failed` according to evidence.
10. **Partial reconciliation:** normalization or terminal-publication failure must not leave a pipette row indefinitely `reserved`; restart reconciliation must update both linked projections.
11. **Missing live event writers:** receive-thread callbacks, source-distinct errors, completion taints, and pressure sessions must enter typed runtime tables with generation and transaction identity.
12. **Filtered summary mismatch:** every summary count and rate must use the same normalized filter set and read snapshot as the displayed resources.
13. **Export hardening:** download must validate stored byte count, regular-file and no-symlink status, canonical root confinement, and digest over the exact bytes returned. It must read once or use a race-safe file handle. Terminal export receipts must link to governed evidence objects.
14. **BMS provenance:** the accepted release must be reachable from the approved `test` branch, use one API/frontend release identity, and contain no unreviewed dirty overlay.
15. **Broad acceptance gaps:** the exact candidate still needs the specified broad regression, restore, capacity, corruption, migration, and mounted-browser gates.

### 24.4 Historical pipette truth

The live evidence at reassessment contained nine legacy pipette receipts: seven `live_readback` rows and two `tip` rows whose requested action was `load`. Four corresponding operator attempts were visible in the command projection. The two tip rows had operator-layer success but no verified delivery, controller ACK, completion, hardware postcondition, or physical effect.

The only accepted historical claim is:

> Zero typed pipette operations and zero verified physical effects were present at the reassessment snapshot.

The evidence does not prove zero historical attempts, zero transport activity, or zero physical effects. Migration must preserve these rows as historical, tainted evidence without upgrading their truth level.

## 25. Binding remaining work packages

No package below authorizes tests, migration, deployment, service restart, hardware queries, CAN traffic, physical pipette action, or wet commissioning. Those actions require separate authority. Source implementation, verification, integration, deployment, live read-only acceptance, controller evidence, and physical acceptance remain separate gates.

### RA-RW0: Freeze the final denominator and RED contract

**Objective:** Produce the machine-generated current-runtime denominator before further implementation.

**Primary files:**

- create `docs/specs/evidence/bioxp-runtime-audit-entrypoint-denominator.json`;
- create or update a deterministic inventory generator under `scripts/`;
- update `tests/test_pipette_wp0_wp4_contracts.py`;
- add focused route/protocol/lifecycle denominator tests under `tests/`.

**Required result:**

- inventory mounted `/liquid/*` routes, protocol handlers, operator aliases, application planners, constructor and Serial-206 lifecycle callers, callbacks, transport public methods, BMS relay routes, and cockpit actions;
- classify control effect and durable-claim requirement for every row;
- bind each row to its coordinator and verification ID;
- fail on an unclassified, removed, duplicate, stale, or direct-transport row;
- record the exact denominator SHA-256.

**Exit gate:** RA-2 denominator portion passes with zero missing and zero bypass rows in the intended final design. RED tests demonstrate every currently known bypass.

### RA-RW1: Close universal claim and failure persistence

**Objective:** Ensure every transport-producing path owns a durable claim before its first external side effect.

**Primary files:**

- `src/bioxp/api.py`;
- `src/bioxp/services/pipette_service.py`;
- `src/bioxp/oem_serial206_initialization.py`;
- `src/bioxp/pipette/receipts.py`;
- `src/bioxp/runtime_audit_store.py`;
- `src/bioxp/operator_receipt_store.py`;
- affected protocol and lifecycle tests under `tests/`.

**Required result:**

- route readback, initialization, protocol, constructor, diagnostic, startup, reconnect, recovery, query, eject, and initialize paths through one durable coordinator;
- replace hardware-query GET routes with typed idempotent POST operations and fail-closed compatibility responses;
- prevent transport acquisition and driver calls when claim persistence fails;
- define exact states for admitted, dispatched, ACK, completion, failure, timeout, and ambiguity;
- update operator and pipette projections together during restart reconciliation;
- require controller ACK before publishing `acknowledged`;
- prove zero driver calls for every denied or preclaim-failure row.

**Exit gate:** RA-4 and RA-5 pass against the complete RA-RW0 denominator without hardware use.

### RA-RW2: Establish sole-SQLite authority and migrate legacy receipts

**Objective:** Preserve all legacy evidence while removing JSONL as an active authority.

**Primary files:**

- `src/bioxp/pipette/receipts.py`;
- `src/bioxp/runtime_audit_store.py`;
- managed startup/readiness owner;
- migration tests in `tests/test_ra_wp3_evidence_migration.py` and new production-wiring tests.

**Required result:**

- elect one migration owner across processes;
- create and verify a backup before import;
- bind source path, source SHA-256, source byte count, row count, imported count, duplicate count, and quarantine count;
- import all valid rows idempotently while preserving false and unknown evidence phases;
- quarantine invalid rows with source line and digest evidence;
- publish a terminal migration receipt and exact reconciliation result;
- make repeat startup a no-op after the same source digest;
- disable every production JSONL append after migration;
- fail readiness if a no-ID receipt would otherwise fall back to JSONL.

**Exit gate:** RA-1 and RA-3 pass. The canonical robot store contains the migrated nine-row historical set at minimum, subject to a fresh pre-migration inventory. No active JSONL writer remains.

### RA-RW3: Complete typed correlation and asynchronous evidence

**Objective:** Close the canonical identity chain from caller through report and export.

**Primary files:**

- `src/bioxp/runtime_audit_store.py`;
- `src/bioxp/pipette/receipts.py`;
- `src/bioxp/pipette/audit.py`;
- `src/bioxp/operator_controls.py`;
- protocol and lifecycle dispatch owners;
- Novo receive, callback, completion, and pressure owners;
- `tests/test_ra_wp2_pipette_audit.py` plus end-to-end correlation fixtures.

**Required result:**

- write connection generation, robot ownership generation, protocol job ID, protocol action ID, lifecycle stage ID, callback session ID, action ID, command ID, and pipette operation ID as typed fields;
- reject identity conflicts and missing required parent rows;
- repair or reject idempotent replay when the pipette child is absent;
- bind channel observations, CAN/Novo exchanges, ACK, delayed completion, taints, callbacks, errors, pressure streams, and evidence links;
- preserve the evidence ladder without promoting TX, generic `ok`, cached state, or controller facts to physical truth.

**Exit gate:** RA-6 passes with direct, operator, protocol, lifecycle, callback, and pressure fixtures.

### RA-RW4: Repair reports and harden governed exports

**Objective:** Make every report and export truthful, filtered, snapshot-consistent, and byte-bound.

**Primary files:**

- `src/bioxp/operator_reports.py`;
- `tests/test_ra_wp4_reports.py`;
- BMS strict models and relay tests;
- `platform/frontend/src/components/BioXpOperatorReports.tsx` and mounted frontend tests.

**Required result:**

- apply one normalized filter set to commands, pipette rows, events, pressure, rates, and every summary denominator;
- keep all related resources on one declared read snapshot or one stable high-water contract;
- include migrated legacy rows with explicit historical/tainted labels;
- validate export confinement, regular-file status, no symlink traversal, stored byte count, and digest;
- hash and return the same immutable byte buffer or a race-safe opened file;
- link terminal export receipts to governed evidence objects;
- add corruption, false-byte-count, symlink, path escape, and replacement-race tests;
- preserve exact robot bytes through the BMS relay and browser download.

**Exit gate:** RA-8 through RA-10 pass against one exact robot and BMS candidate.

### RA-RW5: Close storage operations, integration, and release acceptance

**Objective:** Seal the complete software assignment without physical pipette execution.

**Primary surfaces:**

- schema migration and readiness attestation;
- database-plus-evidence backup and restore artifacts;
- capacity and disk-pressure evidence;
- robot and BMS `test` branches;
- immutable robot and BMS releases;
- live robot report API and mounted Development cockpit.

**Required result:**

- pass exact schema, append-only trigger, WAL, checkpoint, disk-full, corruption, and five-year capacity gates;
- pass backup and restore drills with database-to-evidence reconciliation;
- run the focused suites and the complete required regression suite with isolated managed state;
- obtain independent exact-tree review after the final byte change;
- commit and push robot and BMS candidates to their approved `test` branches;
- deploy one immutable API/frontend BMS release reachable from `origin/test`;
- prove source SHA, tree, process owner, listener, database identity, connection generation, mutation-disabled state, and report authority;
- perform live read-only API and mounted-browser acceptance;
- recheck retired workstation database non-use and archive receipts.

**Exit gate:** RA-0 through RA-14 pass on one final candidate. Physical commissioning remains outside this exit gate.

## 26. Current verdict and completion rule

The software storage and reporting assignment is **INCOMPLETE**.

The current implementation provides a functional robot SQLite foundation, read-only report API, BMS relay, cockpit report panel, and a happy-path exact-byte export. Those parts do not compensate for the universal-claim bypasses, active legacy JSONL authority, incomplete migration, missing typed correlations, partial reconciliation, filtered-summary defects, export race and confinement gaps, unsealed BMS integration, and unpassed restore and capacity gates.

Completion may be claimed only when RA-RW0 through RA-RW5 have passed against one exact candidate and every RA-0 through RA-14 gate is green. The completion statement must remain limited to software audit, storage, migration, reporting, relay, and read-only cockpit acceptance.

Physical pipette commissioning, controller postcondition campaigns, wet verification, and OEM physical parity remain unperformed and separately gated. No historical row, HTTP success, typed receipt, controller ACK, report rendering, or export can substitute for Christian's separately authorized physical acceptance.
