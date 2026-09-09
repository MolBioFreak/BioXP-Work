"""Version 7: OEM loaded-tip group (-1) without changing frozen v5/v6.

The OEM ClassPipetteCollection group selector is -1; individual channels are
0..3. The loaded-tip predicate requires this integer domain; NULL and other coherence,
authorization and append-only clause remain identical.
"""
from __future__ import annotations

import hashlib
import sqlite3
from .oem_deck_schema_v6 import (
    DECK_SCHEMA_V6_TRIGGER_SQL, DECK_SCHEMA_V6_TABLES, DECK_SCHEMA_V6_SUPPORT_TABLES,
    DECK_SCHEMA_V6_INDEXES, DECK_SCHEMA_V6_SUPPORT_INDEXES,
    DECK_SCHEMA_V6_TRIGGERS, DECK_SCHEMA_V6_SUPPORT_TRIGGERS,
    DECK_SCHEMA_V6_TABLE_COLUMNS, DECK_SCHEMA_V5_SEMANTIC_COLUMNS,
    _statements, _canonical_v6_attestation, _normalized_sql,
    _deck_domain_object, _schema_constraint_tuple,
)

GROUP_TRIGGER_NAME = "operator_plane_deck_semantic_state_coherence_v1"
_FROZEN_LOADED_BOUND = "NEW.tip_loaded=1 AND (NEW.tip_location IS NULL OR NEW.tip_location NOT BETWEEN 0 AND 3)"
_GROUP_LOADED_BOUND = "NEW.tip_loaded=1 AND (NEW.tip_location IS NULL OR typeof(NEW.tip_location)<>'integer' OR NEW.tip_location NOT BETWEEN -1 AND 3)"
_GROUP_TRIGGER_SOURCES = tuple(
    statement for statement in _statements(DECK_SCHEMA_V6_TRIGGER_SQL)
    if statement.startswith("CREATE TRIGGER IF NOT EXISTS " + GROUP_TRIGGER_NAME + "\n")
)
if len(_GROUP_TRIGGER_SOURCES) != 1 or _GROUP_TRIGGER_SOURCES[0].count(_FROZEN_LOADED_BOUND) != 1:
    raise RuntimeError("frozen v6 loaded-tip trigger source is not exact")
DECK_SCHEMA_V7_TRIGGER_SQL = _GROUP_TRIGGER_SOURCES[0].replace(
    _FROZEN_LOADED_BOUND, _GROUP_LOADED_BOUND
)


def apply_deck_schema_v7(connection: sqlite3.Connection) -> None:
    """Replace one trigger inside the caller's authorized migration transaction."""
    connection.execute(f'DROP TRIGGER "{GROUP_TRIGGER_NAME}"')
    connection.execute(DECK_SCHEMA_V7_TRIGGER_SQL)


def _canonical_v7_attestation() -> tuple[dict[tuple[str, str], str], dict[str, tuple]]:
    objects, constraints = _canonical_v6_attestation()
    identity = ("trigger", GROUP_TRIGGER_NAME)
    objects[identity] = objects[identity].replace(
        _normalized_sql(_FROZEN_LOADED_BOUND), _normalized_sql(_GROUP_LOADED_BOUND)
    )
    return objects, constraints


def verify_deck_schema_v7(connection: sqlite3.Connection) -> None:
    required_tables = DECK_SCHEMA_V6_TABLES | DECK_SCHEMA_V6_SUPPORT_TABLES
    required_indexes = DECK_SCHEMA_V6_INDEXES | DECK_SCHEMA_V6_SUPPORT_INDEXES
    required_triggers = DECK_SCHEMA_V6_TRIGGERS | DECK_SCHEMA_V6_SUPPORT_TRIGGERS
    expected_sql, expected_constraints = _canonical_v7_attestation()
    expected_names = {name for _object_type, name in expected_sql}
    domain_rows = [row for row in connection.execute(
        "SELECT type,name,tbl_name,sql FROM sqlite_master WHERE name NOT LIKE 'sqlite_%'"
    ) if _deck_domain_object(str(row[1]), str(row[2]), expected_names)]
    actual_sql = {
        (str(row[0]), str(row[1])): _normalized_sql(row[3]) for row in domain_rows
    }
    if set(actual_sql) != set(expected_sql):
        extra = sorted(set(actual_sql) - set(expected_sql))
        missing = sorted(set(expected_sql) - set(actual_sql))
        raise RuntimeError(
            f"canonical deck schema v7 object manifest is not exact: extra={extra},missing={missing}"
        )
    for identity, sql in expected_sql.items():
        accepted_sql_hashes = {
            hashlib.sha256(sql.encode()).hexdigest(),
        }
        if identity == ("table", "operator_plane_deck_semantic_state"):
            accepted_sql_hashes.update({
                "216bd31d911a5808693213ba7243e33fd62928af383d17a7694dd2fac78d48ee",
                "bf7e7c749b21399d3076970b3263649176583c59cf155e66db0cb1572092729b",
            })
        actual_sql_hash = hashlib.sha256(actual_sql[identity].encode()).hexdigest()
        if actual_sql_hash not in accepted_sql_hashes:
            raise RuntimeError(
                f"canonical deck schema v7 normalized SQL is not exact: {identity[1]}:{actual_sql_hash}"
            )
    for table_name, expected in expected_constraints.items():
        actual_constraint_hash = hashlib.sha256(
            repr(_schema_constraint_tuple(connection, table_name)).encode()
        ).hexdigest()
        accepted_constraint_hashes = {hashlib.sha256(repr(expected).encode()).hexdigest()}
        if table_name == "operator_plane_deck_semantic_state":
            accepted_constraint_hashes.update({
                "9a1a4b13ee27b90e7bd5b875ebe8ac413b3bea68684016cf83345a7278522d01",
                "11d548e0319816a7e0181ab4d1be58e42fc3b755060a638f8937cfb6a2b99e16",
            })
        if actual_constraint_hash not in accepted_constraint_hashes:
            raise RuntimeError(
                f"canonical deck schema v7 constraint tuples are not exact: {table_name}:{actual_constraint_hash}"
            )
    objects = set(actual_sql)
    missing_tables = sorted(name for name in required_tables if ("table", name) not in objects)
    missing_indexes = sorted(name for name in required_indexes if ("index", name) not in objects)
    missing_triggers = sorted(name for name in required_triggers if ("trigger", name) not in objects)
    if missing_tables or missing_indexes or missing_triggers:
        raise RuntimeError(
            f"canonical deck schema v7 objects missing: tables={missing_tables},indexes={missing_indexes},triggers={missing_triggers}"
        )
    expected_support_columns = {
        "operator_plane_evidence": (
            "evidence_id", "command_id", "evidence_kind", "content_sha256",
            "payload_json", "payload_bytes", "created_at",
        ),
        "operator_plane_interrupt_evidence": (
            "evidence_id", "interrupt_attempt_id", "action_id", "evidence_kind",
            "content_sha256", "payload_json", "payload_bytes", "created_at",
        ),
        "operator_plane_interrupt_attempts": (
            "attempt_sequence", "interrupt_attempt_id", "idempotency_key", "fingerprint",
            "action_id", "phase", "receipt_json", "created_at",
        ),
        "operator_plane_recovery_acknowledgements": (
            "acknowledgement_id", "command_id", "recovery_epoch", "operation",
            "receipt_json", "created_at",
        ),
    }
    for table_name, expected_columns in expected_support_columns.items():
        actual_columns = tuple(str(row[1]) for row in connection.execute(
            f'PRAGMA table_xinfo("{table_name}")'
        ))
        if actual_columns != expected_columns:
            raise RuntimeError(f"canonical deck schema v7 support columns are not exact: {table_name}")
    for table_name, expected_columns in DECK_SCHEMA_V6_TABLE_COLUMNS.items():
        actual_columns = tuple(str(row[1]) for row in connection.execute(
            f'PRAGMA table_xinfo("{table_name}")'
        ))
        accepted_columns = {expected_columns}
        if table_name == "operator_plane_deck_semantic_state":
            accepted_columns.add(DECK_SCHEMA_V5_SEMANTIC_COLUMNS + (
                "save_tip", "old_well", "old_well_text", "old_location",
                "plate_pierced_json", "well_pierced_json",
            ))
        if actual_columns not in accepted_columns:
            raise RuntimeError(f"canonical deck schema v7 columns are not exact: {table_name}")
    child_fk = {(str(row[3]), str(row[2]), str(row[4]), str(row[6]).upper()) for row in connection.execute(
        "PRAGMA foreign_key_list(operator_plane_mov_execution_children)"
    )}
    if ("command_id", "operator_plane_deck_stages", "command_id", "RESTRICT") not in child_fk:
        raise RuntimeError("canonical movExecution child foreign key is incomplete")
    if connection.execute("PRAGMA foreign_key_check").fetchone() is not None:
        raise RuntimeError("canonical deck schema v7 foreign-key check failed")
