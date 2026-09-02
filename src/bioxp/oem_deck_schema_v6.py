"""Canonical additive runtime schema v6 for OEM deck workflows."""
from __future__ import annotations

import hashlib
import sqlite3

DECK_SCHEMA_V6_TABLE_SQL = "\n                BEGIN IMMEDIATE;\n                CREATE TABLE IF NOT EXISTS operator_plane_deck_commands (\n                    command_id TEXT PRIMARY KEY REFERENCES operator_plane_commands(command_id),\n                    target TEXT NOT NULL,\n                    target_label TEXT NOT NULL,\n                    resolved_location_id INTEGER NOT NULL,\n                    destination_catalog_revision TEXT NOT NULL CHECK(length(destination_catalog_revision)=64),\n                    position_table_revision TEXT NOT NULL CHECK(length(position_table_revision)=64),\n                    authority_snapshot_digest TEXT NOT NULL CHECK(length(authority_snapshot_digest)=64),\n                    complete_authority_digest TEXT NOT NULL CHECK(length(complete_authority_digest)=64),\n                    plan_digest TEXT NOT NULL CHECK(length(plan_digest)=64),\n                    source_branch TEXT NOT NULL,\n                    source_anchors_json TEXT NOT NULL CHECK(json_valid(source_anchors_json)),\n                    source_hazards_json TEXT NOT NULL CHECK(json_valid(source_hazards_json)),\n                    delivery_attempted INTEGER NOT NULL DEFAULT 0 CHECK(delivery_attempted IN (0,1)),\n                    controller_command_acknowledged INTEGER NOT NULL DEFAULT 0 CHECK(controller_command_acknowledged IN (0,1)),\n                    controller_completion_verified INTEGER NOT NULL DEFAULT 0 CHECK(controller_completion_verified IN (0,1)),\n                    hardware_postcondition_verified INTEGER NOT NULL DEFAULT 0 CHECK(hardware_postcondition_verified IN (0,1)),\n                    semantic_state_committed INTEGER NOT NULL DEFAULT 0 CHECK(semantic_state_committed IN (0,1)),\n                    physical_observation_verified INTEGER NOT NULL DEFAULT 0 CHECK(physical_observation_verified IN (0,1)),\n                    transition_revision INTEGER,\n                    ambiguity_state TEXT NOT NULL DEFAULT 'none' CHECK(ambiguity_state IN ('none','blocked','ambiguous','recovery_required')),\n                    provider_evidence_json TEXT CHECK(provider_evidence_json IS NULL OR json_valid(provider_evidence_json)),\n                    planned_at REAL NOT NULL,\n                    committed_at REAL,\n                    CHECK(controller_command_acknowledged<=delivery_attempted),\n                    CHECK(hardware_postcondition_verified<=controller_completion_verified),\n                    CHECK((semantic_state_committed=0 AND transition_revision IS NULL) OR (semantic_state_committed=1 AND transition_revision IS NOT NULL)),\n                    CHECK((semantic_state_committed=0 AND committed_at IS NULL) OR (semantic_state_committed=1 AND committed_at IS NOT NULL))\n                ) WITHOUT ROWID;\n                CREATE INDEX IF NOT EXISTS operator_plane_deck_commands_plan_idx\n                    ON operator_plane_deck_commands(plan_digest);\n                CREATE TABLE IF NOT EXISTS operator_plane_deck_stages (\n                    command_id TEXT NOT NULL REFERENCES operator_plane_deck_commands(command_id) ON DELETE RESTRICT,\n                    stage_order INTEGER NOT NULL CHECK(stage_order>=0),\n                    operation TEXT NOT NULL CHECK(length(operation)>0),\n                    source_anchor TEXT NOT NULL CHECK(length(source_anchor)>0),\n                    resources_json TEXT NOT NULL CHECK(json_valid(resources_json) AND json_type(resources_json)='array'),\n                    arguments_json TEXT NOT NULL CHECK(json_valid(arguments_json) AND json_type(arguments_json)='object'),\n                    dependency_order_json TEXT NOT NULL CHECK(json_valid(dependency_order_json) AND json_type(dependency_order_json)='array'),\n                    terminal_evidence_json TEXT CHECK(terminal_evidence_json IS NULL OR json_valid(terminal_evidence_json)),\n                    terminal_state TEXT NOT NULL DEFAULT 'planned' CHECK(terminal_state IN ('planned','completed','failed','ambiguous','stopped','aborted')),\n                    PRIMARY KEY(command_id,stage_order)\n                ) WITHOUT ROWID;\n                CREATE INDEX IF NOT EXISTS operator_plane_deck_stages_terminal_idx\n                    ON operator_plane_deck_stages(command_id,terminal_state,stage_order);\n                CREATE TABLE IF NOT EXISTS operator_plane_wp8_operations (\n                    command_id TEXT PRIMARY KEY,\n                    operation TEXT NOT NULL,\n                    plan_digest TEXT NOT NULL CHECK(length(plan_digest)=64),\n                    authority_digest TEXT NOT NULL CHECK(length(authority_digest)=64),\n                    authority_stamps_json TEXT NOT NULL CHECK(json_valid(authority_stamps_json)),\n                    plan_json TEXT NOT NULL CHECK(json_valid(plan_json)),\n                    terminal_result_json TEXT CHECK(terminal_result_json IS NULL OR json_valid(terminal_result_json)),\n                    created_at REAL NOT NULL,\n                    finished_at REAL\n                ) WITHOUT ROWID;\n                CREATE TABLE IF NOT EXISTS operator_plane_wp8_children (\n                    command_id TEXT NOT NULL REFERENCES operator_plane_wp8_operations(command_id) ON DELETE RESTRICT,\n                    child_order INTEGER NOT NULL CHECK(child_order>=0), operation TEXT NOT NULL,\n                    dependency_order_json TEXT NOT NULL CHECK(json_valid(dependency_order_json)),\n                    arguments_json TEXT NOT NULL CHECK(json_valid(arguments_json)),\n                    ignored_return INTEGER NOT NULL CHECK(ignored_return IN (0,1)),\n                    awaited INTEGER NOT NULL CHECK(awaited IN (0,1)), exception_policy TEXT NOT NULL,\n                    state_mutation_json TEXT NOT NULL CHECK(json_valid(state_mutation_json)),\n                    terminal_state TEXT NOT NULL DEFAULT 'planned' CHECK(terminal_state IN ('planned','completed','failed','ambiguous')),\n                    terminal_evidence_json TEXT CHECK(terminal_evidence_json IS NULL OR json_valid(terminal_evidence_json)),\n                    PRIMARY KEY(command_id,child_order)\n                ) WITHOUT ROWID;\n                CREATE TABLE IF NOT EXISTS operator_plane_wp8_state_transitions (\n                    command_id TEXT NOT NULL, child_order INTEGER NOT NULL,\n                    transition_json TEXT NOT NULL CHECK(json_valid(transition_json)),\n                    authority_stamps_json TEXT NOT NULL CHECK(json_valid(authority_stamps_json)), created_at REAL NOT NULL,\n                    PRIMARY KEY(command_id,child_order),\n                    FOREIGN KEY(command_id,child_order) REFERENCES operator_plane_wp8_children(command_id,child_order) ON DELETE RESTRICT\n                ) WITHOUT ROWID;\n                CREATE TABLE IF NOT EXISTS operator_plane_deck_semantic_state (\n                    singleton INTEGER PRIMARY KEY CHECK(singleton=1),\n                    current_location TEXT,\n                    current_well INTEGER CHECK(current_well IS NULL OR current_well BETWEEN 0 AND 95),\n                    current_tray TEXT,\n                    tip_loaded INTEGER CHECK(tip_loaded IS NULL OR tip_loaded IN (0,1)),\n                    tip_dirty INTEGER CHECK(tip_dirty IS NULL OR tip_dirty IN (0,1)),\n                    tip_location INTEGER,\n                    clean_path INTEGER CHECK(clean_path IS NULL OR clean_path IN (0,1)),\n                    plate_on_gantry TEXT,\n                    movable_plate_locations_json TEXT NOT NULL CHECK(json_valid(movable_plate_locations_json)),\n                    pseudo_z_home INTEGER CHECK(pseudo_z_home IN (500,65000)),\n                    save_tip INTEGER NOT NULL DEFAULT 0 CHECK(save_tip IN (0,1)),\n                    old_well INTEGER NOT NULL DEFAULT 0 CHECK(old_well IN (0,1)),\n                    old_well_text TEXT NOT NULL DEFAULT '',\n                    old_location INTEGER,\n                    plate_pierced_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(plate_pierced_json) AND json_type(plate_pierced_json)='object'),\n                    well_pierced_json TEXT NOT NULL DEFAULT '{}' CHECK(json_valid(well_pierced_json) AND json_type(well_pierced_json)='object'),\n                    semantic_state_revision INTEGER NOT NULL CHECK(semantic_state_revision>=0),\n                    producer_operation TEXT,\n                    producer_command_id TEXT,\n                    ownership_generation INTEGER,\n                    board_epoch_4 INTEGER,\n                    board_epoch_5 INTEGER,\n                    transition_provenance_json TEXT NOT NULL CHECK(json_valid(transition_provenance_json) AND json_type(transition_provenance_json)='object'),\n                    ambiguity_state TEXT NOT NULL DEFAULT 'none' CHECK(ambiguity_state IN ('none','ambiguous','recovery_required')),\n                    updated_at REAL NOT NULL,\n                    FOREIGN KEY(producer_command_id) REFERENCES operator_plane_commands(command_id) ON DELETE RESTRICT\n                );\n                CREATE TABLE IF NOT EXISTS operator_plane_deck_semantic_transitions (\n                    transition_revision INTEGER PRIMARY KEY AUTOINCREMENT,\n                    command_id TEXT NOT NULL REFERENCES operator_plane_commands(command_id) ON DELETE RESTRICT,\n                    source_operation TEXT NOT NULL CHECK(length(source_operation)>0),\n                    before_revision INTEGER NOT NULL CHECK(before_revision>=0),\n                    after_revision INTEGER NOT NULL CHECK(after_revision=before_revision+1),\n                    transition_json TEXT NOT NULL CHECK(json_valid(transition_json) AND json_type(transition_json)='object'),\n                    created_at REAL NOT NULL,\n                    UNIQUE(command_id,after_revision)\n                );\n                CREATE INDEX IF NOT EXISTS operator_plane_deck_transitions_command_idx\n                    ON operator_plane_deck_semantic_transitions(command_id,transition_revision);\n                CREATE TABLE IF NOT EXISTS operator_plane_tip_tray_state (\n                    tray_id INTEGER PRIMARY KEY CHECK(tray_id BETWEEN 0 AND 4),\n                    occupancy_json TEXT CHECK(occupancy_json IS NULL OR (json_valid(occupancy_json) AND json_type(occupancy_json)='array' AND json_array_length(occupancy_json)=96)),\n                    tip_available INTEGER CHECK(tip_available IS NULL OR tip_available IN (0,1)),\n                    available_count INTEGER CHECK(available_count IS NULL OR available_count BETWEEN 0 AND 24),\n                    revision INTEGER NOT NULL DEFAULT 0 CHECK(revision>=0),\n                    operation_id TEXT,\n                    command_id TEXT,\n                    ownership_generation INTEGER,\n                    board_epoch_4 INTEGER,\n                    board_epoch_5 INTEGER,\n                    produced_at REAL,\n                    provenance_json TEXT CHECK(provenance_json IS NULL OR (json_valid(provenance_json) AND json_type(provenance_json)='object')),\n                    provenance_sha256 TEXT CHECK(provenance_sha256 IS NULL OR length(provenance_sha256)=64),\n                    CHECK((revision=0 AND occupancy_json IS NULL AND tip_available IS NULL AND available_count IS NULL AND operation_id IS NULL AND command_id IS NULL AND ownership_generation IS NULL AND board_epoch_4 IS NULL AND board_epoch_5 IS NULL AND produced_at IS NULL AND provenance_json IS NULL AND provenance_sha256 IS NULL) OR (revision>0 AND occupancy_json IS NOT NULL AND tip_available IS NOT NULL AND operation_id IS NOT NULL AND command_id IS NOT NULL AND ownership_generation IS NOT NULL AND board_epoch_4 IS NOT NULL AND board_epoch_5 IS NOT NULL AND produced_at IS NOT NULL AND provenance_json IS NOT NULL AND provenance_sha256 IS NOT NULL))\n                ) WITHOUT ROWID;\n                CREATE TABLE IF NOT EXISTS operator_plane_tip_tray_transitions (\n                    transition_sequence INTEGER PRIMARY KEY AUTOINCREMENT,\n                    tray_id INTEGER NOT NULL CHECK(tray_id BETWEEN 0 AND 4),\n                    revision INTEGER NOT NULL CHECK(revision>0),\n                    transition TEXT NOT NULL CHECK(transition IN ('construct','reset','load_all','remove_well','remove_group','remove_all','camera_missing','add_tip','retip')),\n                    occupancy_json TEXT NOT NULL CHECK(json_valid(occupancy_json) AND json_type(occupancy_json)='array' AND json_array_length(occupancy_json)=96),\n                    tip_available INTEGER NOT NULL CHECK(tip_available IN (0,1)),\n                    available_count INTEGER CHECK(available_count IS NULL OR available_count BETWEEN 0 AND 24),\n                    operation_id TEXT NOT NULL CHECK(length(operation_id)>0),\n                    command_id TEXT NOT NULL CHECK(length(command_id)>0),\n                    ownership_generation INTEGER NOT NULL CHECK(ownership_generation>=0),\n                    board_epoch_4 INTEGER NOT NULL CHECK(board_epoch_4>=0),\n                    board_epoch_5 INTEGER NOT NULL CHECK(board_epoch_5>=0),\n                    produced_at REAL NOT NULL,\n                    provenance_json TEXT NOT NULL CHECK(json_valid(provenance_json) AND json_type(provenance_json)='object'),\n                    provenance_sha256 TEXT NOT NULL CHECK(length(provenance_sha256)=64),\n                    UNIQUE(tray_id,revision)\n                );\n                CREATE INDEX IF NOT EXISTS operator_plane_tip_tray_transitions_revision_idx\n                    ON operator_plane_tip_tray_transitions(tray_id,revision);\n                INSERT OR IGNORE INTO operator_plane_tip_tray_state(tray_id)\n                    VALUES(0),(1),(2),(3),(4);\n                INSERT OR IGNORE INTO operator_plane_deck_semantic_state(\n                    singleton,movable_plate_locations_json,pseudo_z_home,semantic_state_revision,\n                    transition_provenance_json,updated_at\n                ) VALUES(1,'{}',65000,0,'{}',strftime('%s','now'));\n                CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_operations_guard_insert\n                BEFORE INSERT ON operator_plane_wp8_operations WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT,'wp8 writer is not authoritative'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_operations_no_delete\n                BEFORE DELETE ON operator_plane_wp8_operations BEGIN SELECT RAISE(ABORT,'wp8 plan is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_operations_identity_immutable\n                BEFORE UPDATE ON operator_plane_wp8_operations WHEN authority_write_allowed()<>1 OR NEW.command_id IS NOT OLD.command_id OR NEW.operation IS NOT OLD.operation OR NEW.plan_digest IS NOT OLD.plan_digest OR NEW.authority_digest IS NOT OLD.authority_digest OR NEW.authority_stamps_json IS NOT OLD.authority_stamps_json OR NEW.plan_json IS NOT OLD.plan_json\n                BEGIN SELECT RAISE(ABORT,'wp8 plan identity is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_children_guard_insert\n                BEFORE INSERT ON operator_plane_wp8_children WHEN authority_write_allowed()<>1 OR NEW.child_order<>(SELECT COUNT(*) FROM operator_plane_wp8_children WHERE command_id=NEW.command_id) OR NEW.dependency_order_json<>CASE WHEN NEW.child_order=0 THEN '[]' ELSE '['||(NEW.child_order-1)||']' END\n                BEGIN SELECT RAISE(ABORT,'wp8 child order is not authoritative'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_children_terminal_immutable\n                BEFORE UPDATE ON operator_plane_wp8_children WHEN authority_write_allowed()<>1 OR OLD.terminal_state<>'planned' OR NEW.command_id IS NOT OLD.command_id OR NEW.child_order IS NOT OLD.child_order OR NEW.operation IS NOT OLD.operation OR NEW.dependency_order_json IS NOT OLD.dependency_order_json OR NEW.arguments_json IS NOT OLD.arguments_json OR NEW.ignored_return IS NOT OLD.ignored_return OR NEW.awaited IS NOT OLD.awaited OR NEW.exception_policy IS NOT OLD.exception_policy OR NEW.state_mutation_json IS NOT OLD.state_mutation_json\n                BEGIN SELECT RAISE(ABORT,'wp8 child evidence is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_children_no_delete\n                BEFORE DELETE ON operator_plane_wp8_children BEGIN SELECT RAISE(ABORT,'wp8 children are immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_state_guard_insert\n                BEFORE INSERT ON operator_plane_wp8_state_transitions WHEN authority_write_allowed()<>1 OR NOT EXISTS(SELECT 1 FROM operator_plane_wp8_children c WHERE c.command_id=NEW.command_id AND c.child_order=NEW.child_order AND c.terminal_state='completed' AND c.state_mutation_json=NEW.transition_json)\n                BEGIN SELECT RAISE(ABORT,'wp8 state transition is not terminal-child-owned'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_state_no_update\n                BEFORE UPDATE ON operator_plane_wp8_state_transitions BEGIN SELECT RAISE(ABORT,'wp8 state transitions are immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_state_no_delete\n                BEFORE DELETE ON operator_plane_wp8_state_transitions BEGIN SELECT RAISE(ABORT,'wp8 state transitions are immutable'); END;\n                COMMIT;\n                "
DECK_SCHEMA_V6_TRIGGER_SQL = "\n                CREATE TRIGGER IF NOT EXISTS operator_plane_transitions_no_delete\n                BEFORE DELETE ON operator_plane_transitions\n                BEGIN SELECT RAISE(ABORT, 'operator transitions are append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_transitions_no_update\n                BEFORE UPDATE ON operator_plane_transitions\n                BEGIN SELECT RAISE(ABORT, 'operator transitions are append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_evidence_no_delete\n                BEFORE DELETE ON operator_plane_evidence\n                BEGIN SELECT RAISE(ABORT, 'operator evidence is append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_evidence_no_update\n                BEFORE UPDATE ON operator_plane_evidence\n                BEGIN SELECT RAISE(ABORT, 'operator evidence is append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_evidence_no_delete\n                BEFORE DELETE ON operator_plane_interrupt_evidence\n                BEGIN SELECT RAISE(ABORT, 'operator interrupt evidence is append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_evidence_no_update\n                BEFORE UPDATE ON operator_plane_interrupt_evidence\n                BEGIN SELECT RAISE(ABORT, 'operator interrupt evidence is append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_no_terminal_delete\n                BEFORE DELETE ON operator_plane_commands\n                WHEN OLD.status IN ('completed','failed','ambiguous','stopped','aborted','cancelled','cleared','interrupted')\n                BEGIN SELECT RAISE(ABORT, 'terminal operator commands are immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_methods_no_terminal_delete\n                BEFORE DELETE ON operator_plane_methods\n                WHEN OLD.status IN ('completed','failed','cancelled','stopped','aborted','interrupted')\n                BEGIN SELECT RAISE(ABORT, 'terminal operator methods are immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_idempotency_no_update_v2\n                BEFORE UPDATE ON operator_plane_idempotency\n                BEGIN SELECT RAISE(ABORT, 'operator idempotency receipts are append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_idempotency_no_delete_v2\n                BEFORE DELETE ON operator_plane_idempotency\n                BEGIN SELECT RAISE(ABORT, 'operator idempotency receipts are append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_identity_immutable_v2\n                BEFORE UPDATE ON operator_plane_commands\n                WHEN NEW.command_id IS NOT OLD.command_id\n                  OR NEW.stream_sequence IS NOT OLD.stream_sequence\n                  OR NEW.method_id IS NOT OLD.method_id\n                  OR NEW.method_sequence IS NOT OLD.method_sequence\n                  OR NEW.action_id IS NOT OLD.action_id\n                  OR NEW.requested_json IS NOT OLD.requested_json\n                  OR NEW.effective_json IS NOT OLD.effective_json\n                  OR NEW.ownership_generation IS NOT OLD.ownership_generation\n                  OR NEW.queued_at IS NOT OLD.queued_at\n                BEGIN SELECT RAISE(ABORT, 'operator command identity is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_terminal_no_update_v2\n                BEFORE UPDATE ON operator_plane_commands\n                WHEN OLD.status IN ('completed','failed','stopped','aborted','cancelled','cleared')\n                BEGIN SELECT RAISE(ABORT, 'terminal operator commands are immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_methods_identity_immutable_v2\n                BEFORE UPDATE ON operator_plane_methods\n                WHEN NEW.method_id IS NOT OLD.method_id\n                  OR NEW.name IS NOT OLD.name\n                  OR NEW.source_json IS NOT OLD.source_json\n                  OR NEW.digest IS NOT OLD.digest\n                  OR NEW.failure_policy IS NOT OLD.failure_policy\n                  OR NEW.ownership_generation IS NOT OLD.ownership_generation\n                  OR NEW.expanded_count IS NOT OLD.expanded_count\n                  OR NEW.first_stream_sequence IS NOT OLD.first_stream_sequence\n                  OR NEW.last_stream_sequence IS NOT OLD.last_stream_sequence\n                  OR NEW.queued_at IS NOT OLD.queued_at\n                BEGIN SELECT RAISE(ABORT, 'operator method identity is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_methods_terminal_no_update_v2\n                BEFORE UPDATE ON operator_plane_methods\n                WHEN OLD.status IN ('completed','failed','cancelled','stopped','aborted','interrupted')\n                BEGIN SELECT RAISE(ABORT, 'terminal operator methods are immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_movement_commands_identity_immutable_v2\n                BEFORE UPDATE ON serial206_movement_commands\n                WHEN NEW.sequence IS NOT OLD.sequence\n                  OR NEW.command_id IS NOT OLD.command_id\n                  OR NEW.idempotency_key IS NOT OLD.idempotency_key\n                  OR NEW.action_id IS NOT OLD.action_id\n                  OR NEW.method_id IS NOT OLD.method_id\n                  OR NEW.method_order IS NOT OLD.method_order\n                  OR NEW.parallel_group IS NOT OLD.parallel_group\n                  OR NEW.axis_scope IS NOT OLD.axis_scope\n                  OR NEW.board_scope_json IS NOT OLD.board_scope_json\n                  OR NEW.ownership_generation IS NOT OLD.ownership_generation\n                  OR NEW.expected_board_epochs_json IS NOT OLD.expected_board_epochs_json\n                  OR NEW.canonical_inputs_sha256 IS NOT OLD.canonical_inputs_sha256\n                  OR NEW.admitted_interrupt_epochs_json IS NOT OLD.admitted_interrupt_epochs_json\n                  OR NEW.accepted_at IS NOT OLD.accepted_at\n                  OR NEW.queued_at IS NOT OLD.queued_at\n                BEGIN SELECT RAISE(ABORT, 'canonical movement command identity is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_movement_commands_terminal_no_update_v2\n                BEFORE UPDATE ON serial206_movement_commands\n                WHEN OLD.state IN ('completed','failed','cleared','interrupted','rejected')\n                BEGIN SELECT RAISE(ABORT, 'terminal canonical movement commands are immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_movement_commands_no_terminal_delete_v2\n                BEFORE DELETE ON serial206_movement_commands\n                WHEN OLD.state IN ('completed','failed','cleared','interrupted','ambiguous','rejected')\n                BEGIN SELECT RAISE(ABORT, 'terminal canonical movement commands are immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_movement_methods_identity_immutable_v2\n                BEFORE UPDATE ON serial206_movement_methods\n                WHEN NEW.method_id IS NOT OLD.method_id\n                  OR NEW.idempotency_key IS NOT OLD.idempotency_key\n                  OR NEW.action_id IS NOT OLD.action_id\n                  OR NEW.canonical_inputs_sha256 IS NOT OLD.canonical_inputs_sha256\n                  OR NEW.failure_policy IS NOT OLD.failure_policy\n                  OR NEW.child_count IS NOT OLD.child_count\n                  OR NEW.accepted_at IS NOT OLD.accepted_at\n                BEGIN SELECT RAISE(ABORT, 'canonical movement method identity is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_movement_methods_terminal_no_update_v2\n                BEFORE UPDATE ON serial206_movement_methods\n                WHEN OLD.state IN ('completed','completed_partial','failed','cleared','interrupted')\n                BEGIN SELECT RAISE(ABORT, 'terminal canonical movement methods are immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_movement_methods_no_terminal_delete_v2\n                BEFORE DELETE ON serial206_movement_methods\n                WHEN OLD.state IN ('completed','completed_partial','failed','cleared','interrupted','ambiguous')\n                BEGIN SELECT RAISE(ABORT, 'terminal canonical movement methods are immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_command_resources_no_update_v2\n                BEFORE UPDATE ON serial206_command_resources\n                BEGIN SELECT RAISE(ABORT, 'command resource authority is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_command_resources_no_delete_v2\n                BEFORE DELETE ON serial206_command_resources\n                BEGIN SELECT RAISE(ABORT, 'command resource authority is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_command_dependencies_no_update_v2\n                BEFORE UPDATE ON serial206_command_dependencies\n                BEGIN SELECT RAISE(ABORT, 'command dependency authority is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_command_dependencies_no_delete_v2\n                BEFORE DELETE ON serial206_command_dependencies\n                BEGIN SELECT RAISE(ABORT, 'command dependency authority is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_attempts_no_update_v2\n                BEFORE UPDATE ON operator_plane_interrupt_attempts\n                BEGIN SELECT RAISE(ABORT, 'interrupt attempt history is append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_attempts_no_delete_v2\n                BEFORE DELETE ON operator_plane_interrupt_attempts\n                BEGIN SELECT RAISE(ABORT, 'interrupt attempt history is append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_command_resources_sealed_insert_v3\n                BEFORE INSERT ON serial206_command_resources\n                WHEN EXISTS(SELECT 1 FROM operator_plane_transitions WHERE command_id=NEW.command_id)\n                BEGIN SELECT RAISE(ABORT, 'command resource authority is sealed at admission'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_command_dependencies_sealed_insert_v3\n                BEFORE INSERT ON serial206_command_dependencies\n                WHEN EXISTS(SELECT 1 FROM operator_plane_transitions WHERE command_id=NEW.command_id)\n                  OR EXISTS(\n                    WITH RECURSIVE ancestors(command_id) AS (\n                      SELECT NEW.depends_on_command_id\n                      UNION\n                      SELECT dependency.depends_on_command_id\n                      FROM serial206_command_dependencies AS dependency\n                      JOIN ancestors ON dependency.command_id=ancestors.command_id\n                    )\n                    SELECT 1 FROM ancestors WHERE command_id=NEW.command_id\n                  )\n                BEGIN SELECT RAISE(ABORT, 'command dependency authority is sealed or cyclic'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_evidence_coherence_insert_v1\n                BEFORE INSERT ON operator_plane_evidence\n                WHEN authority_write_allowed()<>1\n                  OR NEW.content_sha256 <> sha256_utf8(NEW.payload_json)\n                  OR NEW.payload_bytes <> length(CAST(NEW.payload_json AS BLOB))\n                  OR NEW.payload_json <> canonical_json(NEW.payload_json)\n                  OR NEW.evidence_id <> NEW.command_id || ':' || NEW.evidence_kind || ':' || NEW.content_sha256\n                BEGIN SELECT RAISE(ABORT, 'operator evidence bytes and identity are incoherent'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_evidence_coherence_insert_v1\n                BEFORE INSERT ON operator_plane_interrupt_evidence\n                WHEN authority_write_allowed()<>1\n                  OR NEW.content_sha256 <> sha256_utf8(NEW.payload_json)\n                  OR NEW.payload_bytes <> length(CAST(NEW.payload_json AS BLOB))\n                  OR NEW.payload_json <> canonical_json(NEW.payload_json)\n                  OR NEW.evidence_id <> NEW.interrupt_attempt_id || ':' || NEW.evidence_kind || ':' || NEW.content_sha256\n                BEGIN SELECT RAISE(ABORT, 'interrupt evidence bytes and identity are incoherent'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_attempts_lineage_insert_v1\n                BEFORE INSERT ON operator_plane_interrupt_attempts\n                WHEN authority_write_allowed()<>1\n                  OR NEW.receipt_json <> canonical_json(NEW.receipt_json)\n                  OR json_extract(NEW.receipt_json,'$.interrupt_attempt_id') IS NOT NEW.interrupt_attempt_id\n                  OR json_extract(NEW.receipt_json,'$.action_id') IS NOT NEW.action_id\n                  OR (NEW.phase='admitted' AND EXISTS(SELECT 1 FROM operator_plane_interrupt_attempts WHERE interrupt_attempt_id=NEW.interrupt_attempt_id))\n                  OR (NEW.phase<>'admitted' AND NOT EXISTS(SELECT 1 FROM operator_plane_interrupt_attempts WHERE interrupt_attempt_id=NEW.interrupt_attempt_id AND phase='admitted'))\n                  OR (NEW.phase<>'admitted' AND EXISTS(SELECT 1 FROM operator_plane_interrupt_attempts WHERE interrupt_attempt_id=NEW.interrupt_attempt_id AND phase='terminal'))\n                  OR (NEW.phase<>'admitted' AND EXISTS(\n                      SELECT 1 FROM operator_plane_interrupt_attempts\n                      WHERE interrupt_attempt_id=NEW.interrupt_attempt_id AND phase='admitted'\n                        AND (idempotency_key IS NOT NEW.idempotency_key OR fingerprint IS NOT NEW.fingerprint OR action_id IS NOT NEW.action_id)\n                  ))\n                BEGIN SELECT RAISE(ABORT, 'interrupt attempt lineage or receipt identity is incoherent'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_recoverable_terminal_no_update_v3\n                BEFORE UPDATE ON operator_plane_commands\n                WHEN OLD.status IN ('ambiguous','interrupted')\n                BEGIN SELECT RAISE(ABORT, 'recoverable terminal operator commands are immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_movement_commands_ambiguous_no_update_v3\n                BEFORE UPDATE ON serial206_movement_commands\n                WHEN OLD.state='ambiguous'\n                BEGIN SELECT RAISE(ABORT, 'ambiguous canonical movement commands are immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_recovery_acknowledgements_no_update_v1\n                BEFORE UPDATE ON operator_plane_recovery_acknowledgements\n                BEGIN SELECT RAISE(ABORT, 'recovery acknowledgements are append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_recovery_acknowledgements_no_delete_v1\n                BEFORE DELETE ON operator_plane_recovery_acknowledgements\n                BEGIN SELECT RAISE(ABORT, 'recovery acknowledgements are append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_recovery_acknowledgements_authorized_insert_v2\n                BEFORE INSERT ON operator_plane_recovery_acknowledgements\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'recovery acknowledgement writer is not authoritative'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_authorized_insert_v4\n                BEFORE INSERT ON operator_plane_commands\n                WHEN authority_write_allowed()<>1\n                  OR NEW.status NOT IN ('queued','dispatched','issued_pending','stop_requested','abort_requested','completed','failed','ambiguous','stopped','aborted','cancelled','cleared','interrupted','rejected')\n                BEGIN SELECT RAISE(ABORT, 'operator command writer or state is not authoritative'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_authorized_update_v4\n                BEFORE UPDATE ON operator_plane_commands\n                WHEN authority_write_allowed()<>1\n                  OR NEW.status NOT IN ('queued','dispatched','issued_pending','stop_requested','abort_requested','completed','failed','ambiguous','stopped','aborted','cancelled','cleared','interrupted','rejected')\n                BEGIN SELECT RAISE(ABORT, 'operator command writer or state is not authoritative'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_movement_commands_authorized_insert_v4\n                BEFORE INSERT ON serial206_movement_commands\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'canonical command writer is not authoritative'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_movement_commands_authorized_update_v4\n                BEFORE UPDATE ON serial206_movement_commands\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'canonical command writer is not authoritative'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_movement_commands_authorized_delete_v4\n                BEFORE DELETE ON serial206_movement_commands\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'canonical command delete requires authoritative writer'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_command_resources_authorized_insert_v4\n                BEFORE INSERT ON serial206_command_resources\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'command resource writer is not authoritative'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_command_dependencies_authorized_insert_v4\n                BEFORE INSERT ON serial206_command_dependencies\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'dependency insert requires authoritative writer'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_methods_authorized_insert_v4\n                BEFORE INSERT ON operator_plane_methods\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'method insert requires authoritative writer'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_methods_authorized_update_v4\n                BEFORE UPDATE ON operator_plane_methods\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'method update requires authoritative writer'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_methods_authorized_delete_v4\n                BEFORE DELETE ON operator_plane_methods\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'method delete requires authoritative writer'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_movement_methods_authorized_insert_v4\n                BEFORE INSERT ON serial206_movement_methods\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'movement method insert requires authoritative writer'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_movement_methods_authorized_update_v4\n                BEFORE UPDATE ON serial206_movement_methods\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'movement method update requires authoritative writer'); END;\n                CREATE TRIGGER IF NOT EXISTS serial206_movement_methods_authorized_delete_v4\n                BEFORE DELETE ON serial206_movement_methods\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'movement method delete requires authoritative writer'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_idempotency_authorized_insert_v4\n                BEFORE INSERT ON operator_plane_idempotency\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'idempotency insert requires authoritative writer'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_commands_authorized_delete_v4\n                BEFORE DELETE ON operator_plane_commands\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'command delete requires authoritative writer'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_history_authorized_insert_v4\n                BEFORE INSERT ON operator_plane_interrupt_history\n                WHEN authority_write_allowed()<>1\n                BEGIN SELECT RAISE(ABORT, 'interrupt history insert requires authoritative writer'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_history_no_update_v4\n                BEFORE UPDATE ON operator_plane_interrupt_history\n                BEGIN SELECT RAISE(ABORT, 'interrupt history is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_history_no_delete_v4\n                BEFORE DELETE ON operator_plane_interrupt_history\n                BEGIN SELECT RAISE(ABORT, 'interrupt history is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_interrupt_history_coherence_insert_v4\n                BEFORE INSERT ON operator_plane_interrupt_history\n                WHEN NEW.source_wrapper_json IS NULL\n                  OR NEW.source_wrapper_json<>canonical_json(NEW.source_wrapper_json)\n                  OR NEW.record_sha256<>sha256_utf8(NEW.source_wrapper_json)\n                  OR json_extract(NEW.source_wrapper_json,'$.stream') IS NOT NEW.stream\n                  OR json_extract(NEW.source_wrapper_json,'$.receipt.interrupt_attempt_id') IS NOT NEW.interrupt_attempt_id\n                  OR canonical_json(json_extract(NEW.source_wrapper_json,'$.receipt'))<>NEW.receipt_json\n                BEGIN SELECT RAISE(ABORT, 'interrupt history wrapper is incoherent'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_transitions_authorized_coherent_insert_v4\n                BEFORE INSERT ON operator_plane_transitions\n                WHEN authority_write_allowed()<>1\n                  OR (NEW.command_id IS NOT NULL AND NOT EXISTS(\n                    SELECT 1\n                    FROM operator_plane_commands AS operator\n                    JOIN serial206_movement_commands AS canonical ON canonical.command_id=operator.command_id\n                    WHERE operator.command_id=NEW.command_id AND (\n                      operator.status=canonical.state\n                      OR (operator.status='cancelled' AND canonical.state='cleared')\n                      OR (operator.status='interrupted' AND canonical.state='ambiguous')\n                      OR (operator.status IN ('stop_requested','abort_requested') AND canonical.state='interrupting')\n                      OR (operator.status IN ('stopped','aborted') AND canonical.state='interrupted')\n                    )\n                  ))\n                BEGIN SELECT RAISE(ABORT, 'operator and canonical command authority is incoherent'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_deck_commands_authorized_insert_v1\n                BEFORE INSERT ON operator_plane_deck_commands\n                WHEN authority_write_allowed()<>1\n                  OR NOT EXISTS(SELECT 1 FROM operator_plane_commands WHERE command_id=NEW.command_id AND action_id='oem.deck.move_to_location')\n                  OR NEW.source_anchors_json<>canonical_json(NEW.source_anchors_json)\n                  OR json_type(NEW.source_anchors_json)<>'array'\n                  OR NEW.source_hazards_json<>canonical_json(NEW.source_hazards_json)\n                  OR json_type(NEW.source_hazards_json)<>'array'\n                  OR NEW.delivery_attempted<>0\n                  OR NEW.controller_command_acknowledged<>0\n                  OR NEW.controller_completion_verified<>0\n                  OR NEW.hardware_postcondition_verified<>0\n                  OR NEW.semantic_state_committed<>0\n                  OR NEW.physical_observation_verified<>0\n                  OR NEW.transition_revision IS NOT NULL\n                  OR NEW.provider_evidence_json IS NOT NULL\n                  OR NEW.committed_at IS NOT NULL\n                BEGIN SELECT RAISE(ABORT, 'deck plan insert is unauthorized or incoherent'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_deck_commands_identity_immutable_v1\n                BEFORE UPDATE ON operator_plane_deck_commands\n                WHEN NEW.command_id IS NOT OLD.command_id\n                  OR NEW.target IS NOT OLD.target\n                  OR NEW.target_label IS NOT OLD.target_label\n                  OR NEW.resolved_location_id IS NOT OLD.resolved_location_id\n                  OR NEW.destination_catalog_revision IS NOT OLD.destination_catalog_revision\n                  OR NEW.position_table_revision IS NOT OLD.position_table_revision\n                  OR NEW.authority_snapshot_digest IS NOT OLD.authority_snapshot_digest\n                  OR NEW.complete_authority_digest IS NOT OLD.complete_authority_digest\n                  OR NEW.plan_digest IS NOT OLD.plan_digest\n                  OR NEW.source_branch IS NOT OLD.source_branch\n                  OR NEW.source_anchors_json IS NOT OLD.source_anchors_json\n                  OR NEW.source_hazards_json IS NOT OLD.source_hazards_json\n                  OR NEW.planned_at IS NOT OLD.planned_at\n                BEGIN SELECT RAISE(ABORT, 'deck plan identity is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_deck_commands_evidence_coherence_v1\n                BEFORE UPDATE ON operator_plane_deck_commands\n                WHEN authority_write_allowed()<>1\n                  OR NEW.delivery_attempted<OLD.delivery_attempted\n                  OR NEW.controller_command_acknowledged<OLD.controller_command_acknowledged\n                  OR NEW.controller_completion_verified<OLD.controller_completion_verified\n                  OR NEW.hardware_postcondition_verified<OLD.hardware_postcondition_verified\n                  OR NEW.semantic_state_committed<OLD.semantic_state_committed\n                  OR NEW.physical_observation_verified<OLD.physical_observation_verified\n                  OR (OLD.semantic_state_committed=1 AND (\n                       NEW.transition_revision IS NOT OLD.transition_revision\n                       OR NEW.provider_evidence_json IS NOT OLD.provider_evidence_json\n                       OR NEW.committed_at IS NOT OLD.committed_at))\n                  OR (NEW.provider_evidence_json IS NOT NULL AND NEW.provider_evidence_json<>canonical_json(NEW.provider_evidence_json))\n                BEGIN SELECT RAISE(ABORT, 'deck evidence is unauthorized or incoherent'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_deck_commands_no_delete_v1\n                BEFORE DELETE ON operator_plane_deck_commands\n                BEGIN SELECT RAISE(ABORT, 'deck plans are immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_deck_stages_authorized_insert_v1\n                BEFORE INSERT ON operator_plane_deck_stages\n                WHEN authority_write_allowed()<>1\n                  OR NEW.resources_json<>canonical_json(NEW.resources_json)\n                  OR NEW.arguments_json<>canonical_json(NEW.arguments_json)\n                  OR NEW.dependency_order_json<>canonical_json(NEW.dependency_order_json)\n                  OR NEW.terminal_state<>'planned'\n                  OR NEW.terminal_evidence_json IS NOT NULL\n                  OR NEW.stage_order<>COALESCE((SELECT MAX(stage_order)+1 FROM operator_plane_deck_stages WHERE command_id=NEW.command_id),0)\n                  OR NEW.dependency_order_json<>CASE WHEN NEW.stage_order=0 THEN '[]' ELSE '[' || (NEW.stage_order-1) || ']' END\n                BEGIN SELECT RAISE(ABORT, 'deck stage insert is unauthorized or incoherent'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_deck_stages_identity_immutable_v1\n                BEFORE UPDATE ON operator_plane_deck_stages\n                WHEN authority_write_allowed()<>1\n                  OR NEW.command_id IS NOT OLD.command_id\n                  OR NEW.stage_order IS NOT OLD.stage_order\n                  OR NEW.operation IS NOT OLD.operation\n                  OR NEW.source_anchor IS NOT OLD.source_anchor\n                  OR NEW.resources_json IS NOT OLD.resources_json\n                  OR NEW.arguments_json IS NOT OLD.arguments_json\n                  OR NEW.dependency_order_json IS NOT OLD.dependency_order_json\n                BEGIN SELECT RAISE(ABORT, 'deck stage identity is immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_deck_stages_terminal_immutable_v1\n                BEFORE UPDATE ON operator_plane_deck_stages\n                WHEN OLD.terminal_state<>'planned'\n                  OR NEW.terminal_state='planned'\n                  OR NEW.terminal_evidence_json IS NULL\n                  OR NEW.terminal_evidence_json<>canonical_json(NEW.terminal_evidence_json)\n                BEGIN SELECT RAISE(ABORT, 'deck stage terminal evidence is immutable or incoherent'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_deck_stages_no_delete_v1\n                BEFORE DELETE ON operator_plane_deck_stages\n                BEGIN SELECT RAISE(ABORT, 'deck stages are immutable'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_deck_semantic_state_coherence_v1\n                BEFORE UPDATE ON operator_plane_deck_semantic_state\n                WHEN authority_write_allowed()<>1\n                  OR NEW.singleton<>1\n                  OR NEW.semantic_state_revision<>OLD.semantic_state_revision+1\n                  OR NEW.transition_provenance_json<>canonical_json(NEW.transition_provenance_json)\n                  OR NEW.producer_command_id IS NULL\n                  OR NOT EXISTS(SELECT 1 FROM operator_plane_commands WHERE command_id=NEW.producer_command_id)\n                  OR (NEW.tip_loaded=1 AND (NEW.tip_location IS NULL OR NEW.tip_location NOT BETWEEN 0 AND 3))\n                BEGIN SELECT RAISE(ABORT, 'deck semantic state update is unauthorized or incoherent'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_deck_semantic_state_no_delete_v1\n                BEFORE DELETE ON operator_plane_deck_semantic_state\n                BEGIN SELECT RAISE(ABORT, 'deck semantic state cannot be deleted'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_deck_semantic_transitions_coherence_v1\n                BEFORE INSERT ON operator_plane_deck_semantic_transitions\n                WHEN authority_write_allowed()<>1\n                  OR NEW.after_revision<>NEW.before_revision+1\n                  OR NEW.transition_json<>canonical_json(NEW.transition_json)\n                  OR json_extract(NEW.transition_json,'$.command_id') IS NOT NEW.command_id\n                  OR json_extract(NEW.transition_json,'$.source_operation') IS NOT NEW.source_operation\n                  OR json_extract(NEW.transition_json,'$.before_revision') IS NOT NEW.before_revision\n                  OR json_extract(NEW.transition_json,'$.after_revision') IS NOT NEW.after_revision\n                  OR NOT EXISTS(\n                      SELECT 1 FROM operator_plane_deck_semantic_state\n                      WHERE singleton=1 AND semantic_state_revision=NEW.after_revision\n                        AND producer_command_id=NEW.command_id AND producer_operation=NEW.source_operation\n                  )\n                BEGIN SELECT RAISE(ABORT, 'deck semantic transition is unauthorized or incoherent'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_deck_semantic_transitions_no_update_v1\n                BEFORE UPDATE ON operator_plane_deck_semantic_transitions\n                BEGIN SELECT RAISE(ABORT, 'deck semantic transitions are append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_deck_semantic_transitions_no_delete_v1\n                BEFORE DELETE ON operator_plane_deck_semantic_transitions\n                BEGIN SELECT RAISE(ABORT, 'deck semantic transitions are append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_tip_tray_state_coherence_v1\n                BEFORE UPDATE ON operator_plane_tip_tray_state\n                WHEN authority_write_allowed()<>1\n                  OR NEW.tray_id IS NOT OLD.tray_id\n                  OR NEW.revision<>OLD.revision+1\n                  OR NEW.occupancy_json IS NULL\n                  OR NEW.occupancy_json<>canonical_json(NEW.occupancy_json)\n                  OR EXISTS(SELECT 1 FROM json_each(NEW.occupancy_json) WHERE type NOT IN ('true','false'))\n                  OR NEW.tip_available IS NULL\n                  OR NEW.operation_id IS NULL OR length(NEW.operation_id)=0\n                  OR NEW.command_id IS NULL OR length(NEW.command_id)=0\n                  OR NEW.ownership_generation IS NULL OR NEW.ownership_generation<0\n                  OR NEW.board_epoch_4 IS NULL OR NEW.board_epoch_4<0\n                  OR NEW.board_epoch_5 IS NULL OR NEW.board_epoch_5<0\n                  OR NEW.produced_at IS NULL\n                  OR NEW.provenance_json IS NULL\n                  OR NEW.provenance_json<>canonical_json(NEW.provenance_json)\n                  OR NEW.provenance_sha256<>sha256_utf8(NEW.provenance_json)\n                BEGIN SELECT RAISE(ABORT, 'tip tray projection update is unauthorized or incoherent'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_tip_tray_state_no_delete_v1\n                BEFORE DELETE ON operator_plane_tip_tray_state\n                BEGIN SELECT RAISE(ABORT, 'tip tray projection cannot be deleted'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_tip_tray_transitions_coherence_v1\n                BEFORE INSERT ON operator_plane_tip_tray_transitions\n                WHEN authority_write_allowed()<>1\n                  OR NEW.occupancy_json<>canonical_json(NEW.occupancy_json)\n                  OR EXISTS(SELECT 1 FROM json_each(NEW.occupancy_json) WHERE type NOT IN ('true','false'))\n                  OR NEW.provenance_json<>canonical_json(NEW.provenance_json)\n                  OR NEW.provenance_sha256<>sha256_utf8(NEW.provenance_json)\n                  OR NOT EXISTS(\n                      SELECT 1 FROM operator_plane_tip_tray_state\n                      WHERE tray_id=NEW.tray_id AND revision=NEW.revision\n                        AND occupancy_json=NEW.occupancy_json\n                        AND tip_available=NEW.tip_available\n                        AND available_count IS NEW.available_count\n                        AND operation_id=NEW.operation_id\n                        AND command_id=NEW.command_id\n                        AND ownership_generation=NEW.ownership_generation\n                        AND board_epoch_4=NEW.board_epoch_4\n                        AND board_epoch_5=NEW.board_epoch_5\n                        AND produced_at=NEW.produced_at\n                        AND provenance_json=NEW.provenance_json\n                        AND provenance_sha256=NEW.provenance_sha256\n                  )\n                BEGIN SELECT RAISE(ABORT, 'tip tray transition insert is unauthorized or incoherent'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_tip_tray_transitions_no_update_v1\n                BEFORE UPDATE ON operator_plane_tip_tray_transitions\n                BEGIN SELECT RAISE(ABORT, 'tip tray transitions are append-only'); END;\n                CREATE TRIGGER IF NOT EXISTS operator_plane_tip_tray_transitions_no_delete_v1\n                BEFORE DELETE ON operator_plane_tip_tray_transitions\n                BEGIN SELECT RAISE(ABORT, 'tip tray transitions are append-only'); END;\n                "
DECK_SCHEMA_V6_TRIGGER_SQL = DECK_SCHEMA_V6_TRIGGER_SQL.replace(
    "action_id='oem.deck.move_to_location'",
    "action_id IN ('oem.deck.move_to_location','oem.deck._mov_execution')",
)

DECK_SCHEMA_V6_SUPPORT_SQL = """
CREATE TABLE IF NOT EXISTS operator_plane_evidence (
    evidence_id TEXT PRIMARY KEY,
    command_id TEXT NOT NULL REFERENCES operator_plane_commands(command_id),
    evidence_kind TEXT NOT NULL,
    content_sha256 TEXT NOT NULL CHECK(length(content_sha256)=64),
    payload_json TEXT NOT NULL CHECK(json_valid(payload_json)),
    payload_bytes INTEGER NOT NULL CHECK(payload_bytes>=0),
    created_at REAL NOT NULL,
    UNIQUE(command_id,evidence_kind,content_sha256)
);
CREATE TABLE IF NOT EXISTS operator_plane_interrupt_evidence (
    evidence_id TEXT PRIMARY KEY,
    interrupt_attempt_id TEXT NOT NULL,
    action_id TEXT NOT NULL,
    evidence_kind TEXT NOT NULL,
    content_sha256 TEXT NOT NULL CHECK(length(content_sha256)=64),
    payload_json TEXT NOT NULL CHECK(json_valid(payload_json)),
    payload_bytes INTEGER NOT NULL CHECK(payload_bytes>=0),
    created_at REAL NOT NULL,
    UNIQUE(interrupt_attempt_id,evidence_kind,content_sha256)
) WITHOUT ROWID;
CREATE TABLE IF NOT EXISTS operator_plane_interrupt_attempts (
    attempt_sequence INTEGER PRIMARY KEY AUTOINCREMENT,
    interrupt_attempt_id TEXT NOT NULL,
    idempotency_key TEXT NOT NULL,
    fingerprint TEXT NOT NULL CHECK(length(fingerprint)=64),
    action_id TEXT NOT NULL,
    phase TEXT NOT NULL CHECK(phase IN ('admitted','attempted','terminal')),
    receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
    created_at REAL NOT NULL,
    UNIQUE(interrupt_attempt_id,phase)
);
CREATE INDEX IF NOT EXISTS operator_plane_interrupt_attempts_key_idx
    ON operator_plane_interrupt_attempts(idempotency_key,attempt_sequence DESC);
CREATE TABLE IF NOT EXISTS operator_plane_recovery_acknowledgements (
    acknowledgement_id TEXT PRIMARY KEY,
    command_id TEXT NOT NULL REFERENCES operator_plane_commands(command_id),
    recovery_epoch INTEGER NOT NULL CHECK(recovery_epoch>=0),
    operation TEXT NOT NULL CHECK(operation IN ('resume_undispatched','cancel_pending')),
    receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),
    created_at REAL NOT NULL,
    UNIQUE(command_id,recovery_epoch)
) WITHOUT ROWID;
"""

DECK_SCHEMA_V6_SUPPORT_TABLES = frozenset({
    "operator_plane_evidence",
    "operator_plane_interrupt_evidence",
    "operator_plane_interrupt_attempts",
    "operator_plane_recovery_acknowledgements",
})
DECK_SCHEMA_V6_SUPPORT_INDEXES = frozenset({
    "operator_plane_interrupt_attempts_key_idx",
})
DECK_SCHEMA_V6_SUPPORT_TRIGGERS = frozenset({
    "operator_plane_evidence_no_delete",
    "operator_plane_evidence_no_update",
    "operator_plane_evidence_coherence_insert_v1",
    "operator_plane_interrupt_evidence_no_delete",
    "operator_plane_interrupt_evidence_no_update",
    "operator_plane_interrupt_evidence_coherence_insert_v1",
    "operator_plane_interrupt_attempts_no_update_v2",
    "operator_plane_interrupt_attempts_no_delete_v2",
    "operator_plane_interrupt_attempts_lineage_insert_v1",
    "operator_plane_recovery_acknowledgements_no_update_v1",
    "operator_plane_recovery_acknowledgements_no_delete_v1",
    "operator_plane_recovery_acknowledgements_authorized_insert_v2",
})

DECK_SCHEMA_V6_EXTRA_SQL = "\nCREATE TABLE IF NOT EXISTS operator_plane_deck_recovery_decisions (\n    decision_id TEXT PRIMARY KEY,\n    command_id TEXT NOT NULL REFERENCES operator_plane_commands(command_id) ON DELETE RESTRICT,\n    stream_sequence INTEGER NOT NULL,\n    dispatch_attempt_id TEXT NOT NULL,\n    recovery_epoch INTEGER NOT NULL CHECK(recovery_epoch>=0),\n    plan_digest TEXT NOT NULL CHECK(length(plan_digest)=64),\n    authority_snapshot_digest TEXT NOT NULL CHECK(length(authority_snapshot_digest)=64),\n    position_table_revision TEXT NOT NULL CHECK(length(position_table_revision)=64),\n    destination_catalog_revision TEXT NOT NULL CHECK(length(destination_catalog_revision)=64),\n    controller_observation_id TEXT NOT NULL,\n    decision_json TEXT NOT NULL CHECK(json_valid(decision_json)),\n    receipt_json TEXT NOT NULL CHECK(json_valid(receipt_json)),\n    created_at REAL NOT NULL,\n    UNIQUE(command_id,recovery_epoch)\n) WITHOUT ROWID;\nCREATE INDEX IF NOT EXISTS operator_plane_deck_recovery_command_idx\n    ON operator_plane_deck_recovery_decisions(command_id,recovery_epoch);\nCREATE TABLE IF NOT EXISTS operator_plane_mov_execution_children (\n    command_id TEXT NOT NULL REFERENCES operator_plane_deck_commands(command_id) ON DELETE RESTRICT,\n    stage_order INTEGER NOT NULL,\n    child_order INTEGER NOT NULL,\n    operation TEXT NOT NULL CHECK(length(operation)>0),\n    arguments_json TEXT NOT NULL CHECK(json_valid(arguments_json) AND json_type(arguments_json)='object'),\n    join_kind TEXT NOT NULL CHECK(join_kind IN ('Task.WaitAll')),\n    terminal_state TEXT NOT NULL DEFAULT 'planned' CHECK(terminal_state IN ('planned','completed','failed','ambiguous','stopped','aborted')),\n    terminal_evidence_json TEXT CHECK(terminal_evidence_json IS NULL OR json_valid(terminal_evidence_json)),\n    PRIMARY KEY(command_id,stage_order,child_order),\n    FOREIGN KEY(command_id,stage_order) REFERENCES operator_plane_deck_stages(command_id,stage_order) ON DELETE RESTRICT\n) WITHOUT ROWID;\nCREATE INDEX IF NOT EXISTS operator_plane_mov_execution_children_terminal_idx\n    ON operator_plane_mov_execution_children(command_id,terminal_state,stage_order,child_order);\nCREATE TRIGGER IF NOT EXISTS operator_plane_deck_recovery_decisions_authorized_insert_v1\nBEFORE INSERT ON operator_plane_deck_recovery_decisions\nWHEN authority_write_allowed()<>1\n  OR NEW.decision_json<>canonical_json(NEW.decision_json)\n  OR NEW.receipt_json<>canonical_json(NEW.receipt_json)\n  OR json_extract(NEW.decision_json,'$.decision_id') IS NOT NEW.decision_id\n  OR json_extract(NEW.receipt_json,'$.command_id') IS NOT NEW.command_id\n  OR NOT EXISTS(\n    SELECT 1 FROM operator_plane_commands c\n    JOIN operator_plane_deck_commands d USING(command_id)\n    WHERE c.command_id=NEW.command_id\n      AND c.stream_sequence=NEW.stream_sequence\n      AND c.dispatch_attempt_id=NEW.dispatch_attempt_id\n      AND d.plan_digest=NEW.plan_digest\n      AND d.authority_snapshot_digest=NEW.authority_snapshot_digest\n      AND d.position_table_revision=NEW.position_table_revision\n      AND d.destination_catalog_revision=NEW.destination_catalog_revision\n  )\nBEGIN SELECT RAISE(ABORT,'deck recovery decision is unauthorized or rebound'); END;\nCREATE TRIGGER IF NOT EXISTS operator_plane_deck_recovery_decisions_no_update_v1\nBEFORE UPDATE ON operator_plane_deck_recovery_decisions\nBEGIN SELECT RAISE(ABORT,'deck recovery decisions are immutable'); END;\nCREATE TRIGGER IF NOT EXISTS operator_plane_deck_recovery_decisions_no_delete_v1\nBEFORE DELETE ON operator_plane_deck_recovery_decisions\nBEGIN SELECT RAISE(ABORT,'deck recovery decisions are immutable'); END;\nCREATE TRIGGER IF NOT EXISTS operator_plane_mov_execution_children_authorized_insert_v1\nBEFORE INSERT ON operator_plane_mov_execution_children\nWHEN authority_write_allowed()<>1\n  OR NEW.arguments_json<>canonical_json(NEW.arguments_json)\n  OR NEW.terminal_state<>'planned'\n  OR NEW.terminal_evidence_json IS NOT NULL\n  OR NEW.child_order<>COALESCE((SELECT MAX(child_order)+1 FROM operator_plane_mov_execution_children WHERE command_id=NEW.command_id AND stage_order=NEW.stage_order),0)\n  OR NOT EXISTS(SELECT 1 FROM operator_plane_deck_stages s WHERE s.command_id=NEW.command_id AND s.stage_order=NEW.stage_order AND s.terminal_state='planned')\nBEGIN SELECT RAISE(ABORT,'movExecution child insert is unauthorized or incoherent'); END;\nCREATE TRIGGER IF NOT EXISTS operator_plane_mov_execution_children_terminal_immutable_v1\nBEFORE UPDATE ON operator_plane_mov_execution_children\nWHEN authority_write_allowed()<>1\n  OR OLD.terminal_state<>'planned'\n  OR NEW.command_id IS NOT OLD.command_id\n  OR NEW.stage_order IS NOT OLD.stage_order\n  OR NEW.child_order IS NOT OLD.child_order\n  OR NEW.operation IS NOT OLD.operation\n  OR NEW.arguments_json IS NOT OLD.arguments_json\n  OR NEW.join_kind IS NOT OLD.join_kind\nBEGIN SELECT RAISE(ABORT,'movExecution child evidence is immutable'); END;\nCREATE TRIGGER IF NOT EXISTS operator_plane_mov_execution_children_no_delete_v1\nBEFORE DELETE ON operator_plane_mov_execution_children\nBEGIN SELECT RAISE(ABORT,'movExecution children are immutable'); END;\nCREATE TRIGGER IF NOT EXISTS operator_plane_deck_semantic_state_wp7_coherence_v1\nBEFORE UPDATE ON operator_plane_deck_semantic_state\nWHEN authority_write_allowed()<>1\n  OR NEW.save_tip NOT IN (0,1)\n  OR NEW.old_well NOT IN (0,1)\n  OR NEW.plate_pierced_json<>canonical_json(NEW.plate_pierced_json)\n  OR NEW.well_pierced_json<>canonical_json(NEW.well_pierced_json)\nBEGIN SELECT RAISE(ABORT,'WP7 semantic state is unauthorized or incoherent'); END;\n"

DECK_SCHEMA_V6_EXTRA_SQL += """
CREATE TABLE IF NOT EXISTS operator_plane_delivery_attempts (
 attempt_sequence INTEGER PRIMARY KEY AUTOINCREMENT,
 command_id TEXT NOT NULL REFERENCES operator_plane_commands(command_id) ON DELETE RESTRICT,
 work_kind TEXT NOT NULL, work_identity TEXT NOT NULL,
 dispatch_attempt_id TEXT NOT NULL, plan_digest TEXT NOT NULL CHECK(length(plan_digest)=64), owner_id TEXT NOT NULL,
 ownership_generation INTEGER NOT NULL CHECK(ownership_generation>=0), board_epoch_4 INTEGER NOT NULL CHECK(board_epoch_4>=0), board_epoch_5 INTEGER NOT NULL CHECK(board_epoch_5>=0), created_at REAL NOT NULL);
CREATE INDEX IF NOT EXISTS operator_plane_delivery_attempts_command_idx ON operator_plane_delivery_attempts(command_id,attempt_sequence);
CREATE TABLE IF NOT EXISTS operator_plane_wp8_background_tasks (
 command_id TEXT NOT NULL, child_order INTEGER NOT NULL CHECK(child_order>=0), task_id TEXT NOT NULL UNIQUE, task_kind TEXT NOT NULL,
 plan_digest TEXT NOT NULL CHECK(length(plan_digest)=64), dispatch_attempt_id TEXT NOT NULL,
 delivery_attempt_sequence INTEGER NOT NULL UNIQUE REFERENCES operator_plane_delivery_attempts(attempt_sequence) ON DELETE RESTRICT,
 authority_stamps_json TEXT NOT NULL CHECK(json_valid(authority_stamps_json)),
 state TEXT NOT NULL CHECK(state IN ('created','running','completed','failed','ambiguous','interrupted')), evidence_json TEXT NOT NULL CHECK(json_valid(evidence_json)),
 created_at REAL NOT NULL, updated_at REAL NOT NULL, PRIMARY KEY(command_id,child_order),
 FOREIGN KEY(command_id,child_order) REFERENCES operator_plane_wp8_children(command_id,child_order) ON DELETE RESTRICT) WITHOUT ROWID;
CREATE TRIGGER IF NOT EXISTS operator_plane_delivery_attempts_authorized_insert BEFORE INSERT ON operator_plane_delivery_attempts
 WHEN authority_write_allowed()<>1 BEGIN SELECT RAISE(ABORT,'delivery attempt writer is not authoritative'); END;
CREATE TRIGGER IF NOT EXISTS operator_plane_delivery_attempts_lineage_insert BEFORE INSERT ON operator_plane_delivery_attempts
 WHEN NOT EXISTS (
   SELECT 1 FROM operator_plane_commands c
   JOIN serial206_movement_commands m ON m.command_id=c.command_id
   JOIN operator_plane_lane lane ON lane.singleton=1
   JOIN operator_plane_deck_semantic_state semantic ON semantic.singleton=1
   WHERE c.command_id=NEW.command_id AND c.status='dispatched'
     AND c.dispatch_attempt_id=NEW.dispatch_attempt_id
     AND c.ownership_generation=NEW.ownership_generation
     AND lane.owner_id=NEW.owner_id
     AND semantic.ownership_generation=NEW.ownership_generation
     AND semantic.board_epoch_4=NEW.board_epoch_4 AND semantic.board_epoch_5=NEW.board_epoch_5
     AND CAST(json_extract(m.expected_board_epochs_json,'$.4') AS INTEGER)=NEW.board_epoch_4
     AND CAST(json_extract(m.expected_board_epochs_json,'$.5') AS INTEGER)=NEW.board_epoch_5
 ) OR NOT (
   (
     NEW.work_kind='named_stage' AND EXISTS (
       SELECT 1 FROM operator_plane_deck_commands d
       JOIN operator_plane_deck_stages s ON s.command_id=d.command_id
       WHERE d.command_id=NEW.command_id AND d.plan_digest=NEW.plan_digest
         AND NEW.work_identity='stage:'||s.stage_order||':'||s.operation
     )
   ) OR (
     NEW.work_kind='wp7_stage' AND EXISTS (
       SELECT 1 FROM operator_plane_deck_commands d
       JOIN operator_plane_deck_stages s ON s.command_id=d.command_id
       WHERE d.command_id=NEW.command_id AND d.plan_digest=NEW.plan_digest
         AND (
           NEW.work_identity='stage:'||s.stage_order||':'||s.operation
           OR (s.operation='pierceCurrentWell' AND NEW.work_identity='stage:'||s.stage_order||':pierceCurrentWell:scriptmoveTo')
           OR EXISTS (
             SELECT 1 FROM operator_plane_mov_execution_children child
             WHERE child.command_id=s.command_id AND child.stage_order=s.stage_order
               AND NEW.work_identity='stage:'||child.stage_order||':child:'||child.child_order||':'||child.operation
           )
         )
     )
   ) OR (
     NEW.work_kind IN ('wp8_child','wp8_background_task') AND EXISTS (
       SELECT 1 FROM operator_plane_wp8_operations op
       JOIN operator_plane_wp8_children child ON child.command_id=op.command_id
       WHERE op.command_id=NEW.command_id AND op.plan_digest=NEW.plan_digest
         AND NEW.work_identity='child:'||child.child_order||':'||child.operation
         AND (
           (NEW.work_kind='wp8_background_task' AND child.awaited=0)
           OR (NEW.work_kind='wp8_child' AND child.awaited=1)
         )
     )
   )
 ) BEGIN SELECT RAISE(ABORT,'delivery attempt lineage is invalid'); END;
CREATE TRIGGER IF NOT EXISTS operator_plane_delivery_attempts_no_update BEFORE UPDATE ON operator_plane_delivery_attempts BEGIN SELECT RAISE(ABORT,'delivery attempts are append-only'); END;
CREATE TRIGGER IF NOT EXISTS operator_plane_delivery_attempts_no_delete BEFORE DELETE ON operator_plane_delivery_attempts BEGIN SELECT RAISE(ABORT,'delivery attempts are append-only'); END;
CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_background_tasks_authorized_insert BEFORE INSERT ON operator_plane_wp8_background_tasks
 WHEN authority_write_allowed()<>1 BEGIN SELECT RAISE(ABORT,'wp8 background task writer is not authoritative'); END;
CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_background_tasks_lineage_insert BEFORE INSERT ON operator_plane_wp8_background_tasks
 WHEN NOT EXISTS (
   SELECT 1 FROM operator_plane_delivery_attempts attempt
   JOIN operator_plane_wp8_operations op ON op.command_id=attempt.command_id
   JOIN operator_plane_wp8_children child
     ON child.command_id=op.command_id AND child.child_order=NEW.child_order
   WHERE attempt.attempt_sequence=NEW.delivery_attempt_sequence
     AND attempt.command_id=NEW.command_id
     AND attempt.work_kind='wp8_background_task'
     AND attempt.work_identity='child:'||NEW.child_order||':'||NEW.task_kind
     AND attempt.plan_digest=NEW.plan_digest
     AND attempt.dispatch_attempt_id=NEW.dispatch_attempt_id
     AND op.plan_digest=NEW.plan_digest
     AND child.operation=NEW.task_kind AND child.awaited=0
     AND json_type(NEW.authority_stamps_json)='object'
     AND (SELECT COUNT(*) FROM json_each(NEW.authority_stamps_json))=3
     AND json_type(NEW.authority_stamps_json,'$.ownership_generation')='integer'
     AND json_type(NEW.authority_stamps_json,'$.board_epoch_4')='integer'
     AND json_type(NEW.authority_stamps_json,'$.board_epoch_5')='integer'
     AND CAST(json_extract(NEW.authority_stamps_json,'$.ownership_generation') AS INTEGER)=attempt.ownership_generation
     AND CAST(json_extract(NEW.authority_stamps_json,'$.board_epoch_4') AS INTEGER)=attempt.board_epoch_4
     AND CAST(json_extract(NEW.authority_stamps_json,'$.board_epoch_5') AS INTEGER)=attempt.board_epoch_5
 ) BEGIN SELECT RAISE(ABORT,'wp8 background task lineage is invalid'); END;
CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_background_tasks_authorized_update BEFORE UPDATE ON operator_plane_wp8_background_tasks
 WHEN authority_write_allowed()<>1 BEGIN SELECT RAISE(ABORT,'wp8 background task writer is not authoritative'); END;
CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_background_tasks_terminal_authority BEFORE UPDATE ON operator_plane_wp8_background_tasks
 WHEN OLD.state IN ('created','running') AND NEW.state IN ('completed','failed','ambiguous')
  AND NOT EXISTS (
   SELECT 1 FROM operator_plane_delivery_attempts attempt
   JOIN operator_plane_commands command ON command.command_id=attempt.command_id
   JOIN serial206_movement_commands movement ON movement.command_id=command.command_id
   JOIN operator_plane_lane lane ON lane.singleton=1
   JOIN operator_plane_deck_semantic_state semantic ON semantic.singleton=1
   WHERE attempt.attempt_sequence=OLD.delivery_attempt_sequence
     AND attempt.command_id=OLD.command_id
     AND attempt.dispatch_attempt_id=OLD.dispatch_attempt_id
     AND attempt.plan_digest=OLD.plan_digest
     AND attempt.owner_id=lane.owner_id
     AND lane.owner_lease_until>((julianday('now')-2440587.5)*86400.0)
     AND command.status IN ('dispatched','issued_pending')
     AND movement.state IN ('dispatched','issued_pending')
     AND command.dispatch_attempt_id=attempt.dispatch_attempt_id
     AND command.ownership_generation=attempt.ownership_generation
     AND semantic.ownership_generation=attempt.ownership_generation
     AND semantic.board_epoch_4=attempt.board_epoch_4
     AND semantic.board_epoch_5=attempt.board_epoch_5
     AND CAST(json_extract(movement.expected_board_epochs_json,'$.4') AS INTEGER)=attempt.board_epoch_4
     AND CAST(json_extract(movement.expected_board_epochs_json,'$.5') AS INTEGER)=attempt.board_epoch_5
  ) BEGIN SELECT RAISE(ABORT,'wp8 background terminal authority is stale'); END;
CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_background_tasks_identity_immutable BEFORE UPDATE ON operator_plane_wp8_background_tasks
 WHEN NEW.command_id<>OLD.command_id OR NEW.child_order<>OLD.child_order OR NEW.task_id<>OLD.task_id OR NEW.task_kind<>OLD.task_kind
   OR NEW.plan_digest<>OLD.plan_digest OR NEW.dispatch_attempt_id<>OLD.dispatch_attempt_id
   OR NEW.delivery_attempt_sequence<>OLD.delivery_attempt_sequence
   OR NEW.authority_stamps_json<>OLD.authority_stamps_json OR NEW.created_at<>OLD.created_at
 BEGIN SELECT RAISE(ABORT,'wp8 background task identity is immutable'); END;
CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_background_tasks_terminal_immutable BEFORE UPDATE ON operator_plane_wp8_background_tasks
 WHEN OLD.state IN ('completed','failed','ambiguous','interrupted')
 BEGIN SELECT RAISE(ABORT,'terminal wp8 background task is immutable'); END;
CREATE TRIGGER IF NOT EXISTS operator_plane_wp8_background_tasks_no_delete BEFORE DELETE ON operator_plane_wp8_background_tasks BEGIN SELECT RAISE(ABORT,'wp8 background tasks are durable'); END;
"""

DECK_SCHEMA_V6_TABLES = frozenset({
    "operator_plane_deck_commands",
    "operator_plane_deck_stages",
    "operator_plane_deck_semantic_state",
    "operator_plane_deck_semantic_transitions",
    "operator_plane_tip_tray_state",
    "operator_plane_tip_tray_transitions",
    "operator_plane_wp8_operations",
    "operator_plane_wp8_children",
    "operator_plane_wp8_state_transitions",
    "operator_plane_deck_recovery_decisions",
    "operator_plane_mov_execution_children",
    "operator_plane_delivery_attempts",
    "operator_plane_wp8_background_tasks",
})
DECK_SCHEMA_V6_INDEXES = frozenset({
    "operator_plane_deck_commands_plan_idx",
    "operator_plane_deck_stages_terminal_idx",
    "operator_plane_deck_transitions_command_idx",
    "operator_plane_tip_tray_transitions_revision_idx",
    "operator_plane_deck_recovery_command_idx",
    "operator_plane_mov_execution_children_terminal_idx",
    "operator_plane_delivery_attempts_command_idx",
})
DECK_SCHEMA_V6_TRIGGERS = frozenset({
    "operator_plane_deck_commands_authorized_insert_v1",
    "operator_plane_deck_commands_identity_immutable_v1",
    "operator_plane_deck_commands_evidence_coherence_v1",
    "operator_plane_deck_commands_no_delete_v1",
    "operator_plane_deck_stages_authorized_insert_v1",
    "operator_plane_deck_stages_identity_immutable_v1",
    "operator_plane_deck_stages_terminal_immutable_v1",
    "operator_plane_deck_stages_no_delete_v1",
    "operator_plane_deck_semantic_state_coherence_v1",
    "operator_plane_deck_semantic_state_wp7_coherence_v1",
    "operator_plane_deck_semantic_state_no_delete_v1",
    "operator_plane_deck_semantic_transitions_coherence_v1",
    "operator_plane_deck_semantic_transitions_no_update_v1",
    "operator_plane_deck_semantic_transitions_no_delete_v1",
    "operator_plane_tip_tray_state_coherence_v1",
    "operator_plane_tip_tray_state_no_delete_v1",
    "operator_plane_tip_tray_transitions_coherence_v1",
    "operator_plane_tip_tray_transitions_no_update_v1",
    "operator_plane_tip_tray_transitions_no_delete_v1",
    "operator_plane_wp8_operations_guard_insert",
    "operator_plane_wp8_operations_no_delete",
    "operator_plane_wp8_operations_identity_immutable",
    "operator_plane_wp8_children_guard_insert",
    "operator_plane_wp8_children_terminal_immutable",
    "operator_plane_wp8_children_no_delete",
    "operator_plane_wp8_state_guard_insert",
    "operator_plane_wp8_state_no_update",
    "operator_plane_wp8_state_no_delete",
    "operator_plane_deck_recovery_decisions_authorized_insert_v1",
    "operator_plane_deck_recovery_decisions_no_update_v1",
    "operator_plane_deck_recovery_decisions_no_delete_v1",
    "operator_plane_mov_execution_children_authorized_insert_v1",
    "operator_plane_mov_execution_children_terminal_immutable_v1",
    "operator_plane_mov_execution_children_no_delete_v1",
    "operator_plane_delivery_attempts_authorized_insert",
    "operator_plane_delivery_attempts_lineage_insert",
    "operator_plane_delivery_attempts_no_update",
    "operator_plane_delivery_attempts_no_delete",
    "operator_plane_wp8_background_tasks_authorized_insert",
    "operator_plane_wp8_background_tasks_lineage_insert",
    "operator_plane_wp8_background_tasks_authorized_update",
    "operator_plane_wp8_background_tasks_terminal_authority",
    "operator_plane_wp8_background_tasks_identity_immutable",
    "operator_plane_wp8_background_tasks_terminal_immutable",
    "operator_plane_wp8_background_tasks_no_delete",
})

DECK_SCHEMA_V5_TABLES = DECK_SCHEMA_V6_TABLES - {
    "operator_plane_deck_recovery_decisions",
    "operator_plane_mov_execution_children",
    "operator_plane_delivery_attempts",
    "operator_plane_wp8_background_tasks",
}
DECK_SCHEMA_V5_INDEXES = DECK_SCHEMA_V6_INDEXES - {
    "operator_plane_deck_recovery_command_idx",
    "operator_plane_mov_execution_children_terminal_idx",
    "operator_plane_delivery_attempts_command_idx",
}
DECK_SCHEMA_V5_TRIGGERS = DECK_SCHEMA_V6_TRIGGERS - {
    "operator_plane_deck_semantic_state_wp7_coherence_v1",
    "operator_plane_deck_recovery_decisions_authorized_insert_v1",
    "operator_plane_deck_recovery_decisions_no_update_v1",
    "operator_plane_deck_recovery_decisions_no_delete_v1",
    "operator_plane_mov_execution_children_authorized_insert_v1",
    "operator_plane_mov_execution_children_terminal_immutable_v1",
    "operator_plane_mov_execution_children_no_delete_v1",
    "operator_plane_delivery_attempts_authorized_insert",
    "operator_plane_delivery_attempts_lineage_insert",
    "operator_plane_delivery_attempts_no_update",
    "operator_plane_delivery_attempts_no_delete",
    "operator_plane_wp8_background_tasks_authorized_insert",
    "operator_plane_wp8_background_tasks_lineage_insert",
    "operator_plane_wp8_background_tasks_authorized_update",
    "operator_plane_wp8_background_tasks_terminal_authority",
    "operator_plane_wp8_background_tasks_identity_immutable",
    "operator_plane_wp8_background_tasks_terminal_immutable",
    "operator_plane_wp8_background_tasks_no_delete",
}
DECK_SCHEMA_V5_LEGACY_GLOBAL_TRIGGERS = frozenset({
    "operator_plane_commands_authorized_delete_v4",
    "operator_plane_commands_authorized_insert_v4",
    "operator_plane_commands_authorized_update_v4",
    "operator_plane_commands_identity_immutable_v2",
    "operator_plane_commands_recoverable_terminal_no_update_v3",
    "operator_plane_commands_terminal_no_update_v2",
    "operator_plane_idempotency_authorized_insert_v4",
    "operator_plane_idempotency_no_delete_v2",
    "operator_plane_idempotency_no_update_v2",
    "operator_plane_methods_authorized_delete_v4",
    "operator_plane_methods_authorized_insert_v4",
    "operator_plane_methods_authorized_update_v4",
    "operator_plane_methods_identity_immutable_v2",
    "operator_plane_methods_terminal_no_update_v2",
    "operator_plane_transitions_authorized_coherent_insert_v4",
    "serial206_command_dependencies_authorized_insert_v4",
    "serial206_command_dependencies_no_delete_v2",
    "serial206_command_dependencies_no_update_v2",
    "serial206_command_dependencies_sealed_insert_v3",
    "serial206_command_resources_authorized_insert_v4",
    "serial206_command_resources_no_delete_v2",
    "serial206_command_resources_no_update_v2",
    "serial206_command_resources_sealed_insert_v3",
    "serial206_movement_commands_ambiguous_no_update_v3",
    "serial206_movement_commands_authorized_delete_v4",
    "serial206_movement_commands_authorized_insert_v4",
    "serial206_movement_commands_authorized_update_v4",
    "serial206_movement_commands_identity_immutable_v2",
    "serial206_movement_commands_no_terminal_delete_v2",
    "serial206_movement_commands_terminal_no_update_v2",
    "serial206_movement_methods_authorized_delete_v4",
    "serial206_movement_methods_authorized_insert_v4",
    "serial206_movement_methods_authorized_update_v4",
    "serial206_movement_methods_identity_immutable_v2",
    "serial206_movement_methods_no_terminal_delete_v2",
    "serial206_movement_methods_terminal_no_update_v2",
})
DECK_SCHEMA_V5_TRIGGER_SQL_SHA256 = {
    "operator_plane_wp8_operations_guard_insert": "8902bb0f33dd33f869e05c5e1c5ee813c5513034896dada46c9191b8fbe6d3a4",
    "operator_plane_wp8_operations_no_delete": "383394de6b1878bc0c7d900d31c7fcdb8232ffc5f9e2b407b0aac42fa0f99778",
    "operator_plane_wp8_operations_identity_immutable": "76e3954e034969e73c95b9e817460bb9d4465cac00ca7f3049bbe16a7ff66286",
    "operator_plane_wp8_children_guard_insert": "8a38bd71eccb92c3ac8671a97dedac29191c2aaa6fb4e7124b8fc48ea19a28aa",
    "operator_plane_wp8_children_terminal_immutable": "ed60c0d9f56e53e23673e9950a4b4722336dc28c16292f45a48d8b6212afa08d",
    "operator_plane_wp8_children_no_delete": "851ea300a4c4cb70ecf0cfed0621a50b51650aca5b271a2f02828cf2b6e44162",
    "operator_plane_wp8_state_guard_insert": "b16e542582f0db531ee7f2ef9bd5f647b9803ac00582640f450d295108a51263",
    "operator_plane_wp8_state_no_update": "ae5adbb1c28dc2d276ba25ff7e2ab5163c8a7659806de8886b0dfcb3386155cb",
    "operator_plane_wp8_state_no_delete": "3a6aa2b66a859c87ca1a0c95439233de0c0d47d47bcb34789401b380e7c109ca",
    "operator_plane_deck_commands_authorized_insert_v1": "f04999d82edebd8a2aa928f11c020d2c20863b1e87b5a297a78791fc3d2f1a1e",
    "operator_plane_deck_commands_identity_immutable_v1": "44953e8f0332c1061fe25c5eafaeb3fec22f188e895f7f66dc6d5b03e88834f2",
    "operator_plane_deck_commands_evidence_coherence_v1": "263f67587f4d9db7c3f47483b827c73907ff32bce8a35c1c30db418a0f92f75d",
    "operator_plane_deck_commands_no_delete_v1": "935c5e8019b6e468eac43fb7be5d7c4d4f94cee3575cc89cd8ffb832e713a376",
    "operator_plane_deck_stages_authorized_insert_v1": "8f4478d03634df315d7b2ccc5d0e98e5e732a2b0a1ec9e9ad4ae32992266917b",
    "operator_plane_deck_stages_identity_immutable_v1": "fe2cecf0b8dd7b3243185b404c6fb8bc03873db50259acca15ed76a6396e07d8",
    "operator_plane_deck_stages_terminal_immutable_v1": "7984898d77cbd718e4738ec8a9df04171751d37f58cafa5afeb8a03aebbc77a8",
    "operator_plane_deck_stages_no_delete_v1": "23db09e63d8ec1954296cf93d1d4d60e7faf11b879b6d73a93f4b810c8049f0d",
    "operator_plane_deck_semantic_state_coherence_v1": "37afafda2abface0e9f9db605fa1a8046359af6ff7086a48a627e480ca1b6346",
    "operator_plane_deck_semantic_state_no_delete_v1": "ca15e4813381368e8198d5280f4196f956c24ef019f0e36d90cd547ef0fd1b6f",
    "operator_plane_deck_semantic_transitions_coherence_v1": "f0c2f57dce85b5c1aca19a4fdcbb79e22fb1721ed8e883ce4d776f5b725a8796",
    "operator_plane_deck_semantic_transitions_no_update_v1": "ce04164cafa7e27e950f2886b16a5f922188d55adae0740a59763fe2c09227e8",
    "operator_plane_deck_semantic_transitions_no_delete_v1": "3df1c6681265633fb69a05a004e26ce5efa90a0edfc1a59c83a011b9b63b2eed",
    "operator_plane_tip_tray_state_coherence_v1": "430e406da5fe26ec9e59a196fcb4fb83e9bca2e9450a6f329c955127af93acf7",
    "operator_plane_tip_tray_state_no_delete_v1": "675c2842cd73439208c4efd4956495a368cc3d54a010f08c8643ce36a65d292b",
    "operator_plane_tip_tray_transitions_coherence_v1": "3d33a8bdbdf0ba31466a70df86c4abd168ba22b6f8e178c791a099c47fa29f7d",
    "operator_plane_tip_tray_transitions_no_update_v1": "ba6106709981e9a085566fdfe642b1c33c44bef499fff0f7a1b3b740d96d8033",
    "operator_plane_tip_tray_transitions_no_delete_v1": "dc6efc3572718a018fae12f72d0b94e9ab6105bc16c3034aba3aafdec6377d32",
}
DECK_SCHEMA_V6_TABLE_COLUMNS = {
    "operator_plane_deck_commands": (
        "command_id", "target", "target_label", "resolved_location_id",
        "destination_catalog_revision", "position_table_revision", "authority_snapshot_digest",
        "complete_authority_digest", "plan_digest", "source_branch", "source_anchors_json",
        "source_hazards_json", "delivery_attempted", "controller_command_acknowledged",
        "controller_completion_verified", "hardware_postcondition_verified",
        "semantic_state_committed", "physical_observation_verified", "transition_revision",
        "ambiguity_state", "provider_evidence_json", "planned_at", "committed_at",
    ),
    "operator_plane_deck_stages": (
        "command_id", "stage_order", "operation", "source_anchor", "resources_json",
        "arguments_json", "dependency_order_json", "terminal_evidence_json", "terminal_state",
    ),
    "operator_plane_deck_semantic_state": (
        "singleton", "current_location", "current_well", "current_tray", "tip_loaded",
        "tip_dirty", "tip_location", "clean_path", "plate_on_gantry",
        "movable_plate_locations_json", "pseudo_z_home", "save_tip", "old_well",
        "old_well_text", "old_location", "plate_pierced_json", "well_pierced_json",
        "semantic_state_revision", "producer_operation", "producer_command_id",
        "ownership_generation", "board_epoch_4", "board_epoch_5",
        "transition_provenance_json", "ambiguity_state", "updated_at",
    ),
    "operator_plane_deck_semantic_transitions": (
        "transition_revision", "command_id", "source_operation", "before_revision",
        "after_revision", "transition_json", "created_at",
    ),
    "operator_plane_tip_tray_state": (
        "tray_id", "occupancy_json", "tip_available", "available_count", "revision",
        "operation_id", "command_id", "ownership_generation", "board_epoch_4",
        "board_epoch_5", "produced_at", "provenance_json", "provenance_sha256",
    ),
    "operator_plane_tip_tray_transitions": (
        "transition_sequence", "tray_id", "revision", "transition", "occupancy_json",
        "tip_available", "available_count", "operation_id", "command_id",
        "ownership_generation", "board_epoch_4", "board_epoch_5", "produced_at",
        "provenance_json", "provenance_sha256",
    ),
    "operator_plane_wp8_operations": (
        "command_id", "operation", "plan_digest", "authority_digest", "authority_stamps_json",
        "plan_json", "terminal_result_json", "created_at", "finished_at",
    ),
    "operator_plane_wp8_children": (
        "command_id", "child_order", "operation", "dependency_order_json", "arguments_json",
        "ignored_return", "awaited", "exception_policy", "state_mutation_json",
        "terminal_state", "terminal_evidence_json",
    ),
    "operator_plane_wp8_state_transitions": (
        "command_id", "child_order", "transition_json", "authority_stamps_json", "created_at",
    ),
    "operator_plane_delivery_attempts": (
        "attempt_sequence", "command_id", "work_kind", "work_identity", "dispatch_attempt_id",
        "plan_digest", "owner_id", "ownership_generation", "board_epoch_4", "board_epoch_5", "created_at",
    ),
    "operator_plane_wp8_background_tasks": (
        "command_id", "child_order", "task_id", "task_kind", "plan_digest", "dispatch_attempt_id",
        "delivery_attempt_sequence", "authority_stamps_json", "state", "evidence_json", "created_at", "updated_at",
    ),
    "operator_plane_deck_recovery_decisions": (
        "decision_id", "command_id", "stream_sequence", "dispatch_attempt_id", "recovery_epoch",
        "plan_digest", "authority_snapshot_digest", "position_table_revision",
        "destination_catalog_revision", "controller_observation_id", "decision_json",
        "receipt_json", "created_at",
    ),
    "operator_plane_mov_execution_children": (
        "command_id", "stage_order", "child_order", "operation", "arguments_json",
        "join_kind", "terminal_state", "terminal_evidence_json",
    ),
}
DECK_SCHEMA_V5_SEMANTIC_COLUMNS = tuple(
    name for name in DECK_SCHEMA_V6_TABLE_COLUMNS["operator_plane_deck_semantic_state"]
    if name not in {
        "save_tip", "old_well", "old_well_text", "old_location",
        "plate_pierced_json", "well_pierced_json",
    }
)


def _statements(script: str):
    buffer = ""
    for line in script.splitlines():
        buffer += line + "\n"
        if sqlite3.complete_statement(buffer):
            statement = buffer.strip()
            buffer = ""
            normalized = "".join(statement.upper().split())
            if normalized in {"BEGINIMMEDIATE;", "COMMIT;"}:
                continue
            if statement:
                yield statement
    if buffer.strip():
        raise RuntimeError("incomplete canonical deck schema statement")


def _is_v6_trigger(statement: str) -> bool:
    normalized = " ".join(statement.upper().split())
    if not normalized.startswith("CREATE TRIGGER IF NOT EXISTS "):
        return False
    return any(
        f" ON {table.upper()}" in normalized
        for table in DECK_SCHEMA_V6_TABLES | DECK_SCHEMA_V6_SUPPORT_TABLES
    )


def verify_deck_schema_v5(connection: sqlite3.Connection) -> None:
    rows = connection.execute(
        "SELECT type,name FROM sqlite_master WHERE name NOT LIKE 'sqlite_%' AND ("
        "name LIKE 'operator_plane_deck_%' OR name LIKE 'operator_plane_tip_tray_%' "
        "OR name LIKE 'operator_plane_wp8_%')"
    ).fetchall()
    actual = {(str(row[0]), str(row[1])) for row in rows}
    expected = (
        {("table", name) for name in DECK_SCHEMA_V5_TABLES}
        | {("index", name) for name in DECK_SCHEMA_V5_INDEXES}
        | {("trigger", name) for name in DECK_SCHEMA_V5_TRIGGERS}
    )
    if actual != expected:
        raise RuntimeError("canonical deck schema v5 object manifest is not exact")
    canonical_sql, canonical_constraints = _canonical_v6_attestation()
    actual_sql = {
        (str(row[0]), str(row[1])): _normalized_sql(row[2])
        for row in connection.execute(
            "SELECT type,name,sql FROM sqlite_master WHERE name NOT LIKE 'sqlite_%'"
        )
    }
    expected_identities = (
        {("table", name) for name in DECK_SCHEMA_V5_TABLES}
        | {("index", name) for name in DECK_SCHEMA_V5_INDEXES}
        | {("trigger", name) for name in DECK_SCHEMA_V5_TRIGGERS}
    )
    for identity in expected_identities:
        if identity == ("table", "operator_plane_deck_semantic_state"):
            accepted = {
                "216bd31d911a5808693213ba7243e33fd62928af383d17a7694dd2fac78d48ee",
                "bf7e7c749b21399d3076970b3263649176583c59cf155e66db0cb1572092729b",
            }
            if hashlib.sha256(actual_sql[identity].encode()).hexdigest() not in accepted:
                raise RuntimeError("canonical v5 normalized SQL is not exact: operator_plane_deck_semantic_state")
        elif identity[0] == "trigger":
            actual_hash = hashlib.sha256(actual_sql[identity].encode()).hexdigest()
            if actual_hash != DECK_SCHEMA_V5_TRIGGER_SQL_SHA256[identity[1]]:
                raise RuntimeError(f"canonical v5 normalized SQL is not exact: {identity[1]}")
        elif actual_sql.get(identity) != canonical_sql.get(identity):
            raise RuntimeError(f"canonical v5 normalized SQL is not exact: {identity[1]}")
    for table_name in DECK_SCHEMA_V5_TABLES:
        expected_columns = (
            DECK_SCHEMA_V5_SEMANTIC_COLUMNS
            if table_name == "operator_plane_deck_semantic_state"
            else DECK_SCHEMA_V6_TABLE_COLUMNS[table_name]
        )
        actual_columns = tuple(str(row[1]) for row in connection.execute(
            f'PRAGMA table_xinfo("{table_name}")'
        ))
        if actual_columns != expected_columns:
            raise RuntimeError(f"canonical deck schema v5 columns are not exact: {table_name}")
        if table_name != "operator_plane_deck_semantic_state":
            if _schema_constraint_tuple(connection, table_name) != canonical_constraints[table_name]:
                raise RuntimeError(f"canonical v5 constraint tuples are not exact: {table_name}")
    if connection.execute("PRAGMA foreign_key_check").fetchone() is not None:
        raise RuntimeError("canonical deck schema v5 foreign-key check failed")


def migrate_deck_schema_v5_to_v6(connection: sqlite3.Connection) -> None:
    verify_deck_schema_v5(connection)
    for trigger_name in sorted(
        DECK_SCHEMA_V5_TRIGGERS | DECK_SCHEMA_V5_LEGACY_GLOBAL_TRIGGERS
    ):
        connection.execute(f'DROP TRIGGER IF EXISTS "{trigger_name}"')
    additions = (
        "save_tip INTEGER NOT NULL DEFAULT 0 CHECK(save_tip IN (0,1))",
        "old_well INTEGER NOT NULL DEFAULT 0 CHECK(old_well IN (0,1))",
        "old_well_text TEXT NOT NULL DEFAULT ''",
        "old_location INTEGER",
        "plate_pierced_json TEXT NOT NULL DEFAULT '{}' "
        "CHECK(json_valid(plate_pierced_json) AND json_type(plate_pierced_json)='object')",
        "well_pierced_json TEXT NOT NULL DEFAULT '{}' "
        "CHECK(json_valid(well_pierced_json) AND json_type(well_pierced_json)='object')",
    )
    for definition in additions:
        connection.execute(
            "ALTER TABLE operator_plane_deck_semantic_state ADD COLUMN " + definition
        )
    apply_deck_schema_v6(connection)


def apply_deck_schema_v6(connection: sqlite3.Connection) -> None:
    for statement in _statements(DECK_SCHEMA_V6_SUPPORT_SQL):
        connection.execute(statement)
    for statement in _statements(DECK_SCHEMA_V6_TABLE_SQL):
        connection.execute(statement)
    for statement in _statements(DECK_SCHEMA_V6_TRIGGER_SQL):
        if _is_v6_trigger(statement):
            connection.execute(statement)
    for statement in _statements(DECK_SCHEMA_V6_EXTRA_SQL):
        connection.execute(statement)


def _normalized_sql(sql: str | None) -> str:
    if sql is None:
        return ""
    return " ".join(str(sql).replace('"', "").split()).lower()


def _deck_domain_object(name: str, table_name: str, expected_names: set[str]) -> bool:
    prefixes = (
        "operator_plane_deck_", "operator_plane_tip_tray_", "operator_plane_wp8_",
        "operator_plane_mov_execution_", "operator_plane_delivery_attempt",
    )
    return (
        name in expected_names or table_name in expected_names
        or name.startswith(prefixes) or table_name.startswith(prefixes)
    )


def _schema_constraint_tuple(connection: sqlite3.Connection, table_name: str) -> tuple:
    columns = tuple(tuple(row) for row in connection.execute(f'PRAGMA table_xinfo("{table_name}")'))
    foreign_keys = tuple(sorted(tuple(row) for row in connection.execute(
        f'PRAGMA foreign_key_list("{table_name}")'
    )))
    indexes = []
    for row in connection.execute(f'PRAGMA index_list("{table_name}")'):
        index_name = str(row[1])
        indexes.append((tuple(row), tuple(tuple(item) for item in connection.execute(
            f'PRAGMA index_xinfo("{index_name}")'
        ))))
    return columns, foreign_keys, tuple(sorted(indexes, key=lambda item: str(item[0][1])))


def _canonical_v6_attestation() -> tuple[dict[tuple[str, str], str], dict[str, tuple]]:
    reference = sqlite3.connect(":memory:", isolation_level=None)
    try:
        reference.create_function("authority_write_allowed", 0, lambda: 1)
        apply_deck_schema_v6(reference)
        rows = reference.execute(
            "SELECT type,name,tbl_name,sql FROM sqlite_master WHERE name NOT LIKE 'sqlite_%'"
        ).fetchall()
        objects = {(str(row[0]), str(row[1])): _normalized_sql(row[3]) for row in rows}
        tables = {
            name: _schema_constraint_tuple(reference, name)
            for object_type, name in objects if object_type == "table"
        }
        return objects, tables
    finally:
        reference.close()


def verify_deck_schema_v6(connection: sqlite3.Connection) -> None:
    required_tables = DECK_SCHEMA_V6_TABLES | DECK_SCHEMA_V6_SUPPORT_TABLES
    required_indexes = DECK_SCHEMA_V6_INDEXES | DECK_SCHEMA_V6_SUPPORT_INDEXES
    required_triggers = DECK_SCHEMA_V6_TRIGGERS | DECK_SCHEMA_V6_SUPPORT_TRIGGERS
    expected_sql, expected_constraints = _canonical_v6_attestation()
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
            f"canonical deck schema v6 object manifest is not exact: extra={extra},missing={missing}"
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
                f"canonical deck schema v6 normalized SQL is not exact: {identity[1]}:{actual_sql_hash}"
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
                f"canonical deck schema v6 constraint tuples are not exact: {table_name}:{actual_constraint_hash}"
            )
    objects = set(actual_sql)
    missing_tables = sorted(name for name in required_tables if ("table", name) not in objects)
    missing_indexes = sorted(name for name in required_indexes if ("index", name) not in objects)
    missing_triggers = sorted(name for name in required_triggers if ("trigger", name) not in objects)
    if missing_tables or missing_indexes or missing_triggers:
        raise RuntimeError(
            f"canonical deck schema v6 objects missing: tables={missing_tables},indexes={missing_indexes},triggers={missing_triggers}"
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
            raise RuntimeError(f"canonical deck schema v6 support columns are not exact: {table_name}")
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
            raise RuntimeError(f"canonical deck schema v6 columns are not exact: {table_name}")
    child_fk = {(str(row[3]), str(row[2]), str(row[4]), str(row[6]).upper()) for row in connection.execute(
        "PRAGMA foreign_key_list(operator_plane_mov_execution_children)"
    )}
    if ("command_id", "operator_plane_deck_stages", "command_id", "RESTRICT") not in child_fk:
        raise RuntimeError("canonical movExecution child foreign key is incomplete")
    if connection.execute("PRAGMA foreign_key_check").fetchone() is not None:
        raise RuntimeError("canonical deck schema v6 foreign-key check failed")
