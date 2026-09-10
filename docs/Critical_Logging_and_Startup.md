# Critical logging and service startup

User-approved scope (2026-09-10): minimal command start/final outcome and critical
faults only. Routine OEM logs, raw transport/frame logs, polling traces and sampled
telemetry logs are retired. This supersedes earlier lossless receiver-audit scope.

## Runtime behavior

- Command identity, requested operation, dispatch/recovery state, final outcome,
  derived semantic truth and critical errors remain available. Command/Stop/
  recovery ownership is operational state, not a diagnostic log to discard.
- The receiver offers only critical faults to one bounded process writer. Routine
  frames do not enter the queue or undergo logging JSON serialization. Repeated
  identical faults from the same receive owner are coalesced.
- The receiver does not wait for SQLite locks or fsync. A queued fault is volatile
  until committed; logging failure cannot authorize motion or retry a command.
- Ordinary command history does not create full-response evidence files. Retired
  raw-exchange observers and normalized pressure/sample/event writers are not on
  the active command logging path. Existing archived evidence stays readable.
- Healthy idle receiver logging writes no opened/closed/session trace records.
  HTTP access logging and routine Uvicorn messages are disabled.

## Startup boundary

- A fresh managed service invocation must make status and operator HTTP reads
  available within 60 seconds. This is service readiness, not an authorization to
  run OEM initialization, home axes, or perform motion.
- Launch does not recursively hash/traverse the runtime/source trees or re-inspect
  the entire image store. Deployment identity/process binding remains, and a
  runtime-tree digest that was not measured is null, not a fabricated hash.
- Prepared databases use bounded schema/ledger metadata checks. Ordinary startup
  and store construction do not run database-wide foreign-key/integrity audits,
  re-run completed migrations under the writer lock, or audit all receipt history.
  SQLite foreign-key enforcement on writes and operational authority fences remain.
- Startup does not purge, truncate, vacuum, or re-copy retained database history.
  Existing bulk history is retained; this change stops new routine log production,
  rather than claiming to erase past evidence gaps or reclaim existing disk usage.

## Acceptance

Offline checks cover suppressed raw logging, critical fault persistence and writer
contention, prepared/retained schema compatibility, command outcome retention,
archived evidence reads, and Stop/recovery semantics. Live acceptance additionally
requires timing a fresh managed service invocation and observing idle log-table
and file growth without issuing motion commands. Offline fixtures alone do not
establish the live 60-second requirement.
