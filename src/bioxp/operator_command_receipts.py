"""Canonical command-plane receipt projection, without authority or writer owners.

The writer and legacy receipt reader share this exact renderer. Borrow the
caller's SQLite connection and lock: never initialize a store or read an
uncommitted command through a second connection.
"""
from __future__ import annotations

import json
import os
import sqlite3
from collections.abc import Mapping
from typing import Any

RECEIPT_SCHEMA = "bioxp.operator_command_receipt.v1"
ROBOT_IDENTITY = os.getenv("BIOXP_ROBOT_IDENTITY", "serial206").strip() or "serial206"


def _json_load(value: str | None, default: Any = None) -> Any:
    if value is None:
        return default
    try:
        return json.loads(value)
    except (TypeError, ValueError):
        return default


class CommandReceiptReader:
    """Read-only renderer over an already-open, caller-owned connection."""

    def __init__(self, connection: sqlite3.Connection, lock: Any) -> None:
        self.connection = connection
        self._lock = lock

    def _deck_command_detail(self, command_id: str) -> dict[str, Any] | None:
        row = self.connection.execute(
            "SELECT * FROM operator_plane_deck_commands WHERE command_id=?", (str(command_id),)
        ).fetchone()
        if row is None:
            return None
        stages = self.connection.execute(
            "SELECT * FROM operator_plane_deck_stages WHERE command_id=? ORDER BY stage_order",
            (str(command_id),),
        ).fetchall()
        # A process can stop after durable native stage completion but before
        # final semantic publication. Project those motor facts without editing
        # failed history or treating source-only ForceToHighHome as motor I/O.
        parsed_stages = [(stage, _json_load(stage["terminal_evidence_json"], None)) for stage in stages]
        motor_stages = [
            (str(stage["terminal_state"]), evidence or {})
            for stage, evidence in parsed_stages
            if str(stage["operation"]) not in {
                "ForceToHighHome", "check_latch_status", "check_machine_latch_closed"
            }
        ]
        stage_delivery = any(evidence.get("delivery_attempted") is True
                             for _, evidence in motor_stages)
        motor_facts = {"delivery_attempted": bool(row["delivery_attempted"]) or stage_delivery}
        for fact in ("controller_command_acknowledged", "controller_completion_verified"):
            # One completed child cannot attest a later planned/failed motor
            # stage. A genuine source no-op adds no controller claim itself.
            stage_proven = stage_delivery and all(
                (fact == "controller_command_acknowledged" or state == "completed") and (
                    evidence.get(fact) is True
                    or (isinstance(evidence.get("provider_evidence"), Mapping)
                        and evidence["provider_evidence"].get("source_noop") is True)
                )
                for state, evidence in motor_stages
            )
            motor_facts[fact] = bool(row[fact]) or stage_proven
        recovery_resolution = None
        resolution = self.connection.execute(
            "SELECT decision_id,command_id,decision_json,receipt_json FROM operator_plane_deck_recovery_decisions WHERE command_id=?",
            (str(command_id),),
        ).fetchone()
        if resolution is not None:
            decision = _json_load(resolution["decision_json"], {})
            receipt = _json_load(resolution["receipt_json"], {})
            if (isinstance(decision, dict) and isinstance(receipt, dict)
                and decision.get("command_id") == receipt.get("command_id") == str(command_id)
                and decision.get("decision_id") == resolution["decision_id"]
                and isinstance(receipt.get("reconciliation_decision"), dict)
                and receipt["reconciliation_decision"].get("decision_id") == resolution["decision_id"]
                and all(type(receipt.get(k)) is int and receipt[k] >= 1 for k in ("semantic_state_revision", "transition_sequence"))):
                recovery_resolution = {"command_id": str(command_id), "decision_id": str(resolution["decision_id"]),
                    "semantic_state_revision": receipt["semantic_state_revision"],
                    "transition_sequence": receipt["transition_sequence"]}
        return {
            "recovery_resolution": recovery_resolution,
            "target": str(row["target"]), "target_label": str(row["target_label"]),
            "source_branch": str(row["source_branch"]), "resolved_location_id": int(row["resolved_location_id"]),
            "destination_catalog_revision": str(row["destination_catalog_revision"]),
            "position_table_revision": str(row["position_table_revision"]),
            "authority_snapshot_digest": str(row["authority_snapshot_digest"]),
            "complete_authority_digest": str(row["complete_authority_digest"]),
            "plan_digest": str(row["plan_digest"]), "source_anchors": _json_load(row["source_anchors_json"], []),
            **motor_facts,
            "hardware_postcondition_verified": bool(row["hardware_postcondition_verified"]),
            "semantic_state_committed": bool(row["semantic_state_committed"]),
            "physical_observation_verified": bool(row["physical_observation_verified"]),
            "transition_revision": row["transition_revision"], "ambiguity_state": str(row["ambiguity_state"]),
            "stages": [
                {
                    "order": int(stage["stage_order"]), "operation": str(stage["operation"]),
                    "source_anchor": str(stage["source_anchor"]),
                    "resources": _json_load(stage["resources_json"], []),
                    "arguments": _json_load(stage["arguments_json"], {}),
                    "dependencies": _json_load(stage["dependency_order_json"], []),
                    "terminal_state": str(stage["terminal_state"]),
                    "terminal_evidence": evidence,
                }
                for stage, evidence in parsed_stages
            ],
        }

    def _wp8_finite_recovery_detail(self, command_id: str) -> dict[str, Any] | None:
        """Surface the governed recovery resolution for WP8 finite receipts.

        WP8 first-Park / critical-image children never capture a named-deck
        command row, so the named-deck detail returns None for them even after
        a reconcile wrote a decision.  Expose the same validated resolution
        shape the deck detail would carry so consumers (the cockpit
        reconciliation gate) can see the retained ambiguity disposed.  Read
        only; the terminal receipt and its outcome are never rewritten.
        """
        decision = self.connection.execute(
            "SELECT decision_id,command_id,decision_json,receipt_json FROM operator_plane_deck_recovery_decisions WHERE command_id=?",
            (str(command_id),),
        ).fetchone()
        if decision is None:
            return None
        wp8 = self.connection.execute(
            "SELECT 1 FROM operator_plane_wp8_operations WHERE command_id=?", (str(command_id),)
        ).fetchone()
        if wp8 is None:
            return None
        resolution_decision = _json_load(decision["decision_json"], {})
        resolution_receipt = _json_load(decision["receipt_json"], {})
        if (isinstance(resolution_decision, dict) and isinstance(resolution_receipt, dict)
            and resolution_decision.get("command_id") == resolution_receipt.get("command_id") == str(command_id)
            and resolution_decision.get("decision_id") == decision["decision_id"]
            and isinstance(resolution_receipt.get("reconciliation_decision"), dict)
            and resolution_receipt["reconciliation_decision"].get("decision_id") == decision["decision_id"]
            and all(type(resolution_receipt.get(k)) is int and resolution_receipt[k] >= 1
                    for k in ("semantic_state_revision", "transition_sequence"))):
            return {"recovery_resolution": {
                "command_id": str(command_id),
                "decision_id": str(decision["decision_id"]),
                "semantic_state_revision": resolution_receipt["semantic_state_revision"],
                "transition_sequence": resolution_receipt["transition_sequence"],
            }}
        return None

    def _command_response(self, row: sqlite3.Row, *, transition_sequence: int | None = None, compact: bool = False) -> dict[str, Any]:
        if transition_sequence is None:
            transition_row = self.connection.execute("SELECT MAX(transition_sequence) FROM operator_plane_transitions WHERE command_id=?", (str(row["command_id"]),)).fetchone()
            transition_sequence = transition_row[0] if transition_row and transition_row[0] is not None else None
        canonical = self.connection.execute(
            "SELECT sequence,state,state_version,expected_board_epochs_json,terminal_receipt_id FROM serial206_movement_commands WHERE command_id=?",
            (str(row["command_id"]),),
        ).fetchone()
        terminal = _json_load(row["terminal_json"], None)
        response = {
            "schema_version": RECEIPT_SCHEMA,
            "robot_identity": ROBOT_IDENTITY,
            "command_id": str(row["command_id"]),
            "method_id": row["method_id"],
            "method_sequence": row["method_sequence"],
            "stream_sequence": int(row["stream_sequence"]),
            "action_id": str(row["action_id"]),
            "status": str(canonical["state"]) if canonical is not None else str(row["status"]),
            "ownership_generation": int(row["ownership_generation"]),
            "requested_inputs": _json_load(row["requested_json"], {}),
            "effective_inputs": _json_load(row["effective_json"], {}),
            "accepted_at": float(row["queued_at"]),
            "queued_at": float(row["queued_at"]),
            "dispatched_at": row["dispatched_at"],
            "finished_at": row["finished_at"],
            "source_noop": bool(row["source_noop"]),
            "source_noop_reason": row["source_noop_reason"],
            "remote_acknowledged": bool(row["remote_acknowledged"]),
            "controller_acknowledged": bool(row["controller_acknowledged"]),
            "physical_effect_verified": bool(row["physical_effect_verified"]),
            "terminal_evidence": terminal,
            "sequence": int(canonical["sequence"]) if canonical is not None else int(row["stream_sequence"]),
            "state_version": int(canonical["state_version"]) if canonical is not None else int(row["version"]),
            "expected_board_epoch_by_board": _json_load(canonical["expected_board_epochs_json"], {}) if canonical is not None else {},
            "terminal_receipt_id": canonical["terminal_receipt_id"] if canonical is not None else None,
            "completion_class": terminal.get("completion_class") if isinstance(terminal, Mapping) else None,
            "transition_sequence": transition_sequence,
        }
        if compact:
            # Compact V2 receipts consume only the completion-class correction,
            # never stage bodies. Keep the same historical recovery semantics.
            deck_row = self.connection.execute(
                "SELECT ambiguity_state FROM operator_plane_deck_commands WHERE command_id=?",
                (str(row["command_id"]),),
            ).fetchone()
            deck_detail = None if deck_row is None else {"ambiguity_state": deck_row[0]}
        else:
            deck_detail = self._deck_command_detail(str(row["command_id"]))
            if deck_detail is None:
                deck_detail = self._wp8_finite_recovery_detail(str(row["command_id"]))
        if deck_detail is not None:
            response["deck_movement"] = deck_detail
            # A retained post-delivery exception can lack the outer class even
            # though the canonical deck row records ambiguous recovery. Expose
            # that existing disposition; never turn it into completion or alter
            # the saved terminal evidence / recovery hold.
            if (response["completion_class"] is None
                    and response["status"] == "ambiguous"
                    and response["finished_at"] is not None
                    and deck_detail.get("ambiguity_state") == "recovery_required"):
                response["completion_class"] = "recovery_required"
        return response

    def get_command(self, command_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self.connection.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            return self._command_response(row) if row else None

    def get_command_summary(self, command_id: str) -> dict[str, Any] | None:
        """Internal input to the unchanged V2 compact serializer; not authority.

        Avoid reading/decode-copying requested plans and large terminal/stage
        evidence on every poll. Explicit detail and recovery keep full readers.
        """
        with self._lock:
            row = self.connection.execute(
                "SELECT command_id,method_id,method_sequence,stream_sequence,action_id,status,"
                "ownership_generation,queued_at,dispatched_at,finished_at,source_noop,"
                "source_noop_reason,remote_acknowledged,controller_acknowledged,physical_effect_verified,version,"
                "'{}' AS requested_json,'{}' AS effective_json,"
                "json_object('completion_class',json_extract(terminal_json,'$.completion_class')) AS terminal_json "
                "FROM operator_plane_commands WHERE command_id=?", (command_id,),
            ).fetchone()
            return self._command_response(row, compact=True) if row else None

    def command_detail_v2(self, command_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self.connection.execute("SELECT * FROM operator_plane_commands WHERE command_id=?", (command_id,)).fetchone()
            if row is None:
                return None
            projection = self._command_response(row)
            raw_terminal = projection.get("terminal_evidence")
            terminal: dict[str, Any] = dict(raw_terminal) if isinstance(raw_terminal, Mapping) else {}
            nested_response = terminal.get("response")
            response: dict[str, Any] = dict(nested_response) if isinstance(nested_response, Mapping) else dict(terminal)
            raw_keys = ("motor_command_raw_return", "board_wrapper_return", "public_wrapper_return", "public_wrapper_return_kind", "motor_command_delivery_count")
            controller_keys = ("completion_class", "controller_completion_verified", "terminal_speed_zero", "source_returned_normally", "event", "event_window", "wait")
            observed_keys = ("position_after", "terminal_position", "terminal_speed", "discrepancy", "observed_position_steps")
            transitions = self.connection.execute(
                "SELECT transition_sequence,state,payload_json,created_at FROM operator_plane_transitions WHERE command_id=? ORDER BY transition_sequence LIMIT 200",
                (command_id,),
            ).fetchall()
            resources = [str(item[0]) for item in self.connection.execute(
                "SELECT resource_key FROM serial206_command_resources WHERE command_id=? ORDER BY resource_key",
                (command_id,),
            ).fetchall()]
            effective_values = dict(projection.get("effective_inputs") or {})
            missing = object()

            def scalar_observation(key: str, value: Any) -> Any:
                if value is None or isinstance(value, (bool, int, float, str)):
                    return value
                if not isinstance(value, Mapping):
                    return missing
                preferred = {
                    "position_after": ("position", "position_steps", "value"),
                    "terminal_position": ("position", "position_steps", "value"),
                    "terminal_speed": ("speed", "speed_steps", "value"),
                    "discrepancy": ("discrepancy", "steps", "value"),
                    "observed_position_steps": ("position", "position_steps", "value"),
                }.get(key, ("value",))
                for nested_key in preferred:
                    nested = value.get(nested_key)
                    if nested is None or isinstance(nested, (bool, int, float, str)):
                        if nested_key in value:
                            return nested
                return missing

            raw_observed = terminal.get("observed_values")
            if not isinstance(raw_observed, Mapping):
                raw_observed = {key: response[key] for key in observed_keys if key in response}
            observed_values = {}
            for key, value in raw_observed.items():
                scalar = scalar_observation(str(key), value)
                if scalar is not missing:
                    observed_values[str(key)] = scalar

            def transition_status(value: Any) -> str:
                return {
                    "stop_requested": "interrupting",
                    "abort_requested": "interrupting",
                    "stopped": "interrupted",
                    "aborted": "interrupted",
                    "cancelled": "cleared",
                }.get(str(value), str(value))
            for key in ("board_effective_target", "motor_effective_target", "near_high_noop"):
                if key in response:
                    effective_values[key] = response[key]
            projection.update({
                "canonical_inputs": dict(projection.get("requested_inputs") or {}),
                "requested_values": dict(projection.get("requested_inputs") or {}),
                "effective_values": effective_values,
                "observed_values": observed_values,
                "raw_return_layers": dict(response.get("raw_return_layers") or terminal.get("raw_return_layers") or {key: response[key] for key in raw_keys if key in response}),
                "controller_evidence": dict(response.get("controller_evidence") or terminal.get("controller_evidence") or {key: response[key] for key in controller_keys if key in response}),
                "transport_artifacts": list(terminal.get("transport_artifacts") or []),
                "child_receipts": list(terminal.get("child_receipts") or []),
                "resource_keys": resources,
                "transitions": [
                    {
                        "transition_id": str(item["transition_sequence"]),
                        "from_status": None if index == 0 else transition_status(transitions[index - 1]["state"]),
                        "to_status": transition_status(item["state"]),
                        "at": float(item["created_at"]),
                        "reason": (_json_load(item["payload_json"], {}) or {}).get("reason"),
                    }
                    for index, item in enumerate(transitions)
                ],
            })
            return projection
