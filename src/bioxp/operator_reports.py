from __future__ import annotations

import base64
import csv
import hashlib
import hmac
import io
import json
import math
import os
import shutil
import sqlite3
import stat

import time
import uuid
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Callable, Iterator, Mapping

from fastapi import APIRouter, Body, Depends, HTTPException, Query
from fastapi.responses import Response
from pydantic import (
    BaseModel,
    ConfigDict,
    Field,
    JsonValue,
    RootModel,
    StrictBool,
    StrictInt,
    field_validator,
)

from .operator_receipt_store import OperatorReceiptStore, _fsync_directory
from .release_identity import (
    current_release_identity,
    public_release_identity as _public_release_identity,
)
from .runtime_audit_store import runtime_lifecycle_lock
from .storage_operations import (
    StorageEvidenceError,
    _contains_absolute_filesystem_path,
    _validate_report_export_row,
    audit_health_report,
    reconcile_report_exports,
    report_export_evidence_id,
    report_export_retention_deadline,
)


_SCHEMA_VERSION = 2
_MAX_EXPORT_ROWS = 100_000
_CURSOR_TTL_S = 15 * 60
_CURSOR_VERSION = 1
_DEFAULT_WINDOW_S = 24 * 60 * 60
_MAX_WINDOW_S = 31 * 24 * 60 * 60
_DETAIL_CHILD_LIMIT = 100
_EXPORT_PUBLISHER_IDENTITY = "bioxp.operator_reports"
_REPORT_DEADLINE_S = 5.0
_EXPORT_DEADLINE_S = 30.0

_FILTER_FIELDS = (
    "start",
    "end",
    "status",
    "operation",
    "action",
    "channel",
    "entrypoint",
    "caller_class",
    "control_class",
    "protocol_job_id",
    "protocol_action_id",
    "lifecycle_stage_id",
    "lifecycle_attempt_id",
    "outcome",
    "event_source",
    "event_kind",
    "pressure_stream_id",
    "delivery_verified",
    "controller_acknowledged",
    "completion_verified",
    "hardware_postcondition_verified",
    "physical_effect_verified",
    "evidence_state",
    "command_id",
    "pipette_operation_id",
    "connection_generation",
    "ownership_generation",
)

_SOURCE_HIGH_WATERS = {
    "operator_commands": "sequence",
    "operator_transitions": "rowid",
    "pipette_operations": "rowid",
    "pipette_channel_observations": "rowid",
    "pipette_transport_exchanges": "rowid",
    "runtime_events": "event_id",
    "pipette_pressure_streams": "rowid",
    "pipette_pressure_chunks": "rowid",
    "runtime_evidence_objects": "rowid",
    "runtime_evidence_links": "evidence_link_id",
    "runtime_evidence_events": "event_id",
    "operator_plane_command_versions": "version_sequence",
    "operator_plane_pipette_versions": "version_sequence",
    "operator_plane_pressure_stream_versions": "version_sequence",
    "operator_plane_evidence_versions": "version_sequence",
}
_LEGACY_SOURCE_HIGH_WATER_KEYS = frozenset(_SOURCE_HIGH_WATERS) - {
    "runtime_evidence_links"
}

_COMMAND_VERSION_FIELDS = (
    "sequence", "command_id", "idempotency_key", "operation", "command_kind", "entrypoint_id",
    "caller_class", "control_class", "action_id", "status", "outcome", "failure_code",
    "ownership_generation", "connection_generation", "started_at", "admitted_at", "dispatched_at",
    "finished_at", "duration_ms", "delivery_verified", "controller_acknowledged", "completion_verified",
    "hardware_precondition_verified", "hardware_postcondition_verified", "physical_effect_verified",
    "evidence_state", "requested_inputs_json", "effective_inputs_json", "source_identity_json", "updated_at",
    "semantic_query_response_verified",
)
_PIPETTE_VERSION_FIELDS = (
    "pipette_operation_id", "command_id", "operation", "entrypoint_id", "caller_class", "control_class",
    "action_id", "status", "outcome", "failure_code", "ownership_generation", "connection_generation",
    "protocol_job_id", "protocol_action_id", "lifecycle_stage_id", "lifecycle_attempt_id", "callback_session_id",
    "delivery_verified", "controller_acknowledged", "completion_verified", "hardware_precondition_verified",
    "hardware_postcondition_verified", "physical_effect_verified", "evidence_state", "dispatched_at",
    "finished_at", "requested_inputs_json", "effective_inputs_json", "source_identity_json", "updated_at",
    "semantic_query_response_verified",
)


_PRESSURE_STREAM_VERSION_FIELDS = (
    "stream_session_id", "command_id", "pipette_operation_id", "channels_json", "sample_period_ms",
    "started_at", "stopped_at", "source_generation", "reader_generation", "offset_identity",
    "terminal_state", "loss_count",
)
_EVIDENCE_VERSION_FIELDS = (
    "evidence_artifact_id", "command_id", "pipette_operation_id", "original_relpath", "active_relpath",
    "sha256", "byte_count", "created_at", "retention_deadline", "legal_hold", "expiry_state",
    "expiry_receipt_id", "updated_at",
)


def _versioned_source(table: str, identity: str, watermark: int, fields: tuple[str, ...], rowid_field: str) -> str:
    projection = ",".join(
        f"json_extract(v.row_json,'$.{field}') AS {field}" for field in fields
    )
    return (
        f"(SELECT v.{rowid_field} AS rowid,{projection} FROM {table} v "
        f"JOIN (SELECT {identity},MAX(version_sequence) AS selected_version FROM {table} "
        f"WHERE version_sequence<={int(watermark)} GROUP BY {identity}) latest "
        f"ON latest.{identity}=v.{identity} AND latest.selected_version=v.version_sequence "
        "WHERE v.deleted=0)"
    )


def _command_source(high_waters: Mapping[str, Any]) -> str:
    return _versioned_source(
        "operator_plane_command_versions", "command_id",
        int(high_waters["operator_plane_command_versions"]), _COMMAND_VERSION_FIELDS, "source_sequence",
    )


def _pipette_source(high_waters: Mapping[str, Any]) -> str:
    return _versioned_source(
        "operator_plane_pipette_versions", "pipette_operation_id",
        int(high_waters["operator_plane_pipette_versions"]), _PIPETTE_VERSION_FIELDS, "source_rowid",
    )


def _pressure_stream_source(high_waters: Mapping[str, Any]) -> str:
    return _versioned_source(
        "operator_plane_pressure_stream_versions", "stream_session_id",
        int(high_waters["operator_plane_pressure_stream_versions"]),
        _PRESSURE_STREAM_VERSION_FIELDS, "source_rowid",
    )


def _evidence_source(high_waters: Mapping[str, Any]) -> str:
    return _versioned_source(
        "operator_plane_evidence_versions", "evidence_artifact_id",
        int(high_waters["operator_plane_evidence_versions"]), _EVIDENCE_VERSION_FIELDS, "source_rowid",
    )


class _ExportRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)

    format: str = "json"
    limit: StrictInt = Field(default=1000, ge=1, le=_MAX_EXPORT_ROWS)
    start: float | StrictInt | None = None
    end: float | StrictInt | None = None
    status: str | None = None
    operation: str | None = None
    action: str | None = None
    channel: StrictInt | None = Field(default=None, ge=0, le=3)
    entrypoint: str | None = None
    caller_class: str | None = None
    control_class: str | None = None
    protocol_job_id: str | None = None
    protocol_action_id: str | None = None
    lifecycle_stage_id: str | None = None
    lifecycle_attempt_id: str | None = None
    outcome: str | None = None
    event_source: str | None = None
    event_kind: str | None = None
    pressure_stream_id: str | None = None
    delivery_verified: StrictBool | None = None
    controller_acknowledged: StrictBool | None = None
    completion_verified: StrictBool | None = None
    hardware_postcondition_verified: StrictBool | None = None
    physical_effect_verified: StrictBool | None = None
    evidence_state: str | None = None
    command_id: str | None = None
    pipette_operation_id: str | None = None
    connection_generation: StrictInt | None = Field(default=None, ge=0)
    ownership_generation: StrictInt | None = Field(default=None, ge=0)

    @field_validator("format", mode="before")
    @classmethod
    def _normalize_format(cls, value: Any) -> str:
        if not isinstance(value, str):
            raise ValueError("format must be a string")
        normalized = value.lower()
        if normalized not in {"json", "csv"}:
            raise ValueError("unsupported export format")
        return normalized

    @field_validator("start", "end")
    @classmethod
    def _finite_timestamp(cls, value: float | int | None) -> float | int | None:
        if value is not None and not math.isfinite(float(value)):
            raise ValueError("timestamp must be finite")
        return value


class _ClosedModel(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, populate_by_name=True)


class _JsonObject(RootModel[dict[str, JsonValue]]):
    model_config = ConfigDict(strict=True)


class _JsonArray(RootModel[list[JsonValue]]):
    model_config = ConfigDict(strict=True)


class _PublicListener(_ClosedModel):
    host: str | None
    port: int | None


class _PublicReleaseSource(_ClosedModel):
    commit: str | None
    tree: str | None
    mode: str | None
    manifest_sha256: str | None
    aggregate_sha256: str | None


class _PublicReleaseImage(_ClosedModel):
    id: str | None
    inspection_receipt_sha256: str | None


class _PublicReleaseDeployment(_ClosedModel):
    receipt_id: str | None
    installed_at: float | int | str | None
    receipt_sha256: str | None


class _PublicReleaseBinding(_ClosedModel):
    service_unit: str | None
    unit_sha256: str | None
    launcher_sha256: str | None
    configuration_sha256: str | None
    oem_lock_sha256: str | None
    udocker_sha256: str | None
    udocker_tree_sha256: str | None
    declared_listener: _PublicListener | None
    observed_listener: _PublicListener | None


class _PublicReleaseIdentity(_ClosedModel):
    schema_value: str | None = Field(alias="schema", serialization_alias="schema")
    status: str | None
    verified: bool
    reason_code: str | None
    release_id: str | None
    source: _PublicReleaseSource
    image: _PublicReleaseImage
    deployment: _PublicReleaseDeployment
    binding: _PublicReleaseBinding


class _UnavailableReleaseIdentity(_ClosedModel):
    status: str
    verified: bool
    reason_code: str


class _SourceHighWaters(_ClosedModel):
    operator_commands: int
    operator_transitions: int
    pipette_operations: int
    pipette_channel_observations: int
    pipette_transport_exchanges: int
    runtime_events: int
    pipette_pressure_streams: int
    pipette_pressure_chunks: int
    runtime_evidence_objects: int
    runtime_evidence_links: int | None = None
    runtime_evidence_events: int
    operator_plane_command_versions: int
    operator_plane_pipette_versions: int
    operator_plane_pressure_stream_versions: int
    operator_plane_evidence_versions: int


class _SchemaIdentity(_ClosedModel):
    database_identity: str
    schema_version: int
    identity_version: int | None
    release_identity: _PublicReleaseIdentity


class _Snapshot(_ClosedModel):
    database_incarnation_id: str
    schema_identity: _SchemaIdentity
    release_identity: _PublicReleaseIdentity
    source_high_waters: _SourceHighWaters


class _RowSnapshot(_Snapshot):
    high_water_rowid: int


class _SequenceSnapshot(_Snapshot):
    high_water_sequence: int


class _EventSnapshot(_Snapshot):
    high_water_event_id: int


class _ReportFilters(_ClosedModel):
    start: float | int
    end: float | int
    status: str | None
    operation: str | None
    action: str | None
    channel: int | None
    entrypoint: str | None
    caller_class: str | None
    control_class: str | None
    protocol_job_id: str | None
    protocol_action_id: str | None
    lifecycle_stage_id: str | None
    lifecycle_attempt_id: str | None
    outcome: str | None
    event_source: str | None
    event_kind: str | None
    pressure_stream_id: str | None
    delivery_verified: bool | None
    controller_acknowledged: bool | None
    completion_verified: bool | None
    hardware_postcondition_verified: bool | None
    physical_effect_verified: bool | None
    evidence_state: str | None
    command_id: str | None
    pipette_operation_id: str | None
    connection_generation: int | None
    ownership_generation: int | None
    limit: int


class _CommandChildFilters(_ClosedModel):
    command_id: str
    limit: int


class _PipetteChildFilters(_ClosedModel):
    pipette_operation_id: str
    limit: int


class _CommandRow(_ClosedModel):
    sequence: int
    command_id: str
    idempotency_key: str
    operation: str | None
    command_kind: str | None
    entrypoint_id: str | None
    caller_class: str | None
    control_class: str | None
    action_id: str | None
    status: str
    outcome: str | None
    failure_code: str | None
    ownership_generation: int | None
    connection_generation: int | None
    started_at: float | int | str | None
    admitted_at: float | int | str | None
    dispatched_at: float | int | str | None
    finished_at: float | int | str | None
    duration_ms: float | int | None
    delivery_verified: bool
    controller_acknowledged: bool
    completion_verified: bool
    semantic_query_response_verified: bool
    hardware_precondition_verified: bool
    hardware_postcondition_verified: bool
    physical_effect_verified: bool
    evidence_state: str | None


class _PipetteRow(_ClosedModel):
    pipette_operation_id: str
    command_id: str
    operation: str | None
    entrypoint_id: str | None
    caller_class: str | None
    control_class: str | None
    action_id: str | None
    protocol_job_id: str | None
    protocol_action_id: str | None
    lifecycle_stage_id: str | None
    lifecycle_attempt_id: str | None
    callback_session_id: str | None
    status: str
    outcome: str | None
    failure_code: str | None
    delivery_verified: bool
    controller_acknowledged: bool
    completion_verified: bool
    semantic_query_response_verified: bool
    hardware_postcondition_verified: bool
    physical_effect_verified: bool
    evidence_state: str | None


class _TransitionRow(_ClosedModel):
    transition_id: int
    state: str
    observed_at: float | int
    detail: _JsonObject


class _LegalHoldAssessment(_ClosedModel):
    event_id: str
    observed_at: float | int
    legal_hold_requested: bool
    assessment: JsonValue
    actor: str | None
    retained_deadline: float | int | None
    legal_hold_projection_updated: bool


class _EvidenceRow(_ClosedModel):
    evidence_artifact_id: str
    sha256: str
    byte_count: int
    created_at: float | int
    retention_deadline: float | int | None
    legal_hold: bool
    latest_legal_hold_assessment: _LegalHoldAssessment | None
    expiry_state: str
    expiry_receipt_id: str | None


class _ChannelRow(_ClosedModel):
    observation_id: str
    command_id: str
    pipette_operation_id: str
    channel: int
    phase: str | None
    observed_at: float | int
    semantic_validity: str | None
    truth_source: str | None
    tip_loaded: bool | None
    pressure: float | int | None
    pressure_units: str | None
    status: str | int | None
    error_code: str | int | None
    firmware_class: str | None
    detail: _JsonObject


class _ExchangeRow(_ClosedModel):
    exchange_id: str
    transaction_id: str | int | None
    channel: int | None
    transaction_phase: str | None
    command_family: str | None
    matcher_name: str | None
    tx_id: int | None
    expected_rx_id: int | None
    observed_rx_id: int | None
    tx_bytes: list[int]
    rx_bytes: list[int]
    delivery_verified: bool
    semantic_match: bool
    controller_acknowledged: bool
    completion_verified: bool
    completion_before_ack: bool
    sent_at: float | int | None
    received_at: float | int | None
    ack_at: float | int | None
    completion_at: float | int | None


class _EventRow(_ClosedModel):
    event_id: int
    command_id: str | None
    pipette_operation_id: str | None
    event_source: str
    event_kind: str
    channel: int | None
    observed_at: float | int
    event: _JsonObject


class _PipetteEventRow(_ClosedModel):
    event_id: int
    event_source: str
    event_kind: str
    observed_at: float | int
    event: _JsonObject


class _PressureStreamRow(_ClosedModel):
    stream_session_id: str
    pipette_operation_id: str | None
    channels: list[int]
    sample_period_ms: float | int | None
    started_at: float | int
    stopped_at: float | int | None
    source_generation: int | None
    reader_generation: int | None
    offset_identity: str | None
    terminal_state: str | None
    loss_count: int | None


class _PressureSampleRow(_ClosedModel):
    chunk_id: str
    channel: int
    chunk_sequence: int
    sample_count: int
    lost_sample_count: int
    units: str
    sha256: str
    byte_count: int
    evidence_artifact_id: str | None
    summary: _JsonObject


class _PressureChunkRow(_ClosedModel):
    chunk_id: str
    channel: int
    chunk_sequence: int
    sample_count: int
    lost_sample_count: int
    units: str
    sha256: str
    byte_count: int
    evidence_artifact_id: str | None


class _PageContinuation(_ClosedModel):
    returned_count: int
    filtered_total: int
    has_more: bool
    next_cursor: str | None


class _PipetteDetailProjection(_PipetteRow):
    channels: list[_ChannelRow]
    exchanges: list[_ExchangeRow]
    events: list[_PipetteEventRow]
    pressure_streams: list[_PressureStreamRow]


class _CommandDetailResponse(_CommandRow):
    requested_inputs: _JsonObject
    effective_inputs: _JsonObject
    source_identity: _JsonObject
    transitions: list[_TransitionRow]
    evidence: list[_EvidenceRow]
    evidence_preview: list[_EvidenceRow]
    evidence_continuation: _PageContinuation
    pipette: _PipetteDetailProjection | None
    snapshot: _Snapshot
    child_page_limit: int


class _PipetteDetailResponse(_PipetteDetailProjection):
    snapshot: _Snapshot
    child_page_limit: int


class _CommandPageResponse(_ClosedModel):
    filters: _ReportFilters
    snapshot: _SequenceSnapshot
    returned_count: int
    filtered_total: int
    has_more: bool
    next_cursor: str | None
    commands: list[_CommandRow]


class _PipettePageResponse(_ClosedModel):
    filters: _ReportFilters
    snapshot: _RowSnapshot
    returned_count: int
    filtered_total: int
    has_more: bool
    next_cursor: str | None
    pipette: list[_PipetteRow]


class _CommandTransitionPageResponse(_ClosedModel):
    command_id: str
    filters: _CommandChildFilters
    snapshot: _RowSnapshot
    returned_count: int
    filtered_total: int
    has_more: bool
    next_cursor: str | None
    transitions: list[_TransitionRow]


class _CommandEvidencePageResponse(_ClosedModel):
    command_id: str
    filters: _CommandChildFilters
    snapshot: _RowSnapshot
    returned_count: int
    filtered_total: int
    has_more: bool
    next_cursor: str | None
    evidence: list[_EvidenceRow]


class _PipetteChannelPageResponse(_ClosedModel):
    pipette_operation_id: str
    filters: _PipetteChildFilters
    snapshot: _RowSnapshot
    returned_count: int
    filtered_total: int
    has_more: bool
    next_cursor: str | None
    channels: list[_ChannelRow]


class _PipetteExchangePageResponse(_ClosedModel):
    pipette_operation_id: str
    filters: _PipetteChildFilters
    snapshot: _RowSnapshot
    returned_count: int
    filtered_total: int
    has_more: bool
    next_cursor: str | None
    exchanges: list[_ExchangeRow]


class _EventPageResponse(_ClosedModel):
    event_kind: str | None
    filters: _ReportFilters
    snapshot: _EventSnapshot
    returned_count: int
    filtered_total: int
    has_more: bool
    next_cursor: str | None
    events: list[_EventRow]


class _PressureStreamPageResponse(_ClosedModel):
    filters: _ReportFilters
    snapshot: _RowSnapshot
    returned_count: int
    filtered_total: int
    has_more: bool
    next_cursor: str | None
    pressure_streams: list[_PressureStreamRow]


class _PressureSamplePageResponse(_ClosedModel):
    stream_session_id: str
    filters: _ReportFilters
    snapshot: _RowSnapshot
    returned_count: int
    filtered_total: int
    has_more: bool
    next_cursor: str | None
    samples: list[_PressureSampleRow]


class _EventDetailResponse(_EventRow):
    snapshot: _Snapshot


class _PressureStreamDetailResponse(_ClosedModel):
    stream_session_id: str
    pipette_operation_id: str | None
    channels: list[int]
    sample_period_ms: float | int | None
    started_at: float | int
    stopped_at: float | int | None
    terminal_state: str | None
    loss_count: int | None
    chunks: list[_PressureChunkRow]
    child_page_limit: int
    snapshot: _Snapshot


class _StatusCounts(RootModel[dict[str, int]]):
    model_config = ConfigDict(strict=True)


class _ErrorCounts(RootModel[dict[str, int]]):
    model_config = ConfigDict(strict=True)


class _CommandSummary(_ClosedModel):
    total: int
    by_status: _StatusCounts


class _TotalSummary(_ClosedModel):
    total: int


class _PressureSummary(_ClosedModel):
    streams: int
    chunks: int


class _RateSummary(_ClosedModel):
    delivery_rate: float
    ack_rate: float
    completion_rate: float
    postcondition_rate: float
    physical_effect_rate: float
    failure_rate: float


class _LatencySummary(_ClosedModel):
    average_ms: float
    maximum_ms: float


class _ErrorSummary(_ClosedModel):
    by_code: _ErrorCounts


class _ReportSummaryResponse(_ClosedModel):
    scope: str
    filters: _ReportFilters
    snapshot: _SequenceSnapshot
    commands: _CommandSummary
    pipette_operations: _TotalSummary
    runtime_events: _TotalSummary
    pressure: _PressureSummary
    rates: _RateSummary
    latency: _LatencySummary
    errors: _ErrorSummary


class _ExportArtifact(_ClosedModel):
    format: str
    sha256: str
    byte_count: int
    relpath: str | None = None


class _ExportReceiptV1(_ClosedModel):
    receipt_schema: str
    publisher_identity: str
    export_id: str
    evidence_artifact_id: str
    created_at: float | int
    retention_deadline: float | int
    normalized_filters: _ReportFilters
    filter_sha256: str
    source_high_waters: _SourceHighWaters
    schema_identity: _SchemaIdentity
    release_identity: _PublicReleaseIdentity
    database_incarnation_id: str
    row_count: int
    artifact: _ExportArtifact
    public_download_available: bool = True
    evidence_state: str
    legal_hold: bool
    evidence_available: bool


class _LegacyExportReceipt(_ClosedModel):
    receipt_schema: str
    publisher_identity: str
    export_id: str
    evidence_artifact_id: str
    created_at: float | int
    retention_deadline: float | int
    filter_sha256: str
    row_count: int
    artifact: _ExportArtifact
    legacy_snapshot: _JsonObject
    release_identity: _UnavailableReleaseIdentity
    public_download_available: bool
    evidence_state: str
    legal_hold: bool
    evidence_available: bool


_ExportReceipt = _ExportReceiptV1 | _LegacyExportReceipt


class _ExportCreatedResponse(_ClosedModel):
    export_id: str
    evidence_artifact_id: str
    status: str
    format: str
    row_count: int
    sha256: str
    byte_count: int
    release_identity: _PublicReleaseIdentity
    download: str


class _ExportListItem(_ClosedModel):
    export_id: str
    format: str
    row_count: int
    sha256: str
    byte_count: int
    status: str
    created_at: float | int
    release_identity: _PublicReleaseIdentity | _UnavailableReleaseIdentity
    publication_state: str
    evidence_state: str
    legal_hold: bool
    evidence_available: bool
    download: str | None


class _ExportListResponse(_ClosedModel):
    items: list[_ExportListItem]
    returned_count: int
    limit: int


class _ExportMetadataResponse(_ClosedModel):
    export_id: str
    format: str
    filter: _ReportFilters | None
    filter_sha256: str
    snapshot: _ExportReceipt
    receipt: _ExportReceipt
    release_identity: _PublicReleaseIdentity | _UnavailableReleaseIdentity
    row_count: int
    sha256: str
    byte_count: int
    status: str
    created_at: float | int
    completed_at: float | int | None
    publication_state: str
    evidence_state: str
    legal_hold: bool
    evidence_available: bool
    download: str | None


class _HealthSchemaCheck(_ClosedModel):
    status: str | None


class _HealthStoreIdentityCheck(_ClosedModel):
    status: str | None
    recorded_schema_version: int | None
    current_schema_version: int | None


class _HealthDurabilityCheck(_ClosedModel):
    status: str | None
    journal_mode: str | None
    synchronous: int | None


class _HealthCheckpointCheck(_ClosedModel):
    status: str | None
    age_seconds: float | int | None


class _HealthBackupCheck(_ClosedModel):
    status: str | None
    age_seconds: float | int | None
    backup_id: str | None


class _HealthWriterCheck(_ClosedModel):
    status: str | None
    writer_status: str | None
    queue_depth: int | None
    telemetry_available: bool | None
    mode: str | None


class _HealthStorageCheck(_ClosedModel):
    status: str | None
    database_bytes: int | None
    wal_bytes: int | None
    wal_threshold_bytes: int | None
    free_bytes: int | None
    minimum_free_bytes: int | None
    evidence_bytes: int | None
    backup_bytes: int | None


class _AuditHealthChecks(_ClosedModel):
    schema_check: _HealthSchemaCheck = Field(alias="schema", serialization_alias="schema")
    store_identity: _HealthStoreIdentityCheck
    integrity: _HealthSchemaCheck
    foreign_keys: _HealthSchemaCheck
    durability: _HealthDurabilityCheck
    checkpoint: _HealthCheckpointCheck
    backup: _HealthBackupCheck
    writer: _HealthWriterCheck
    storage: _HealthStorageCheck


class _AuditHealthCounts(_ClosedModel):
    commands: int | None
    pipette_operations: int | None
    retained_evidence: int | None
    pending_expiry_evidence: int | None
    integrity_failures: int | None


class _AuditHealthResponse(_ClosedModel):
    schema_value: str | None = Field(alias="schema", serialization_alias="schema")
    status: str | None
    generated_at: float | int | str | None
    degraded_reasons: list[str]
    checks: _AuditHealthChecks
    counts: _AuditHealthCounts
    release_identity: _PublicReleaseIdentity
    physical_admission_gate_added: bool


class _ReportErrorDetail(_ClosedModel):
    error: str
    reason: str | None = None
    command_id: str | None = None
    pipette_operation_id: str | None = None
    evidence_state: str | None = None


class _ReportErrorResponse(_ClosedModel):
    detail: _ReportErrorDetail


def _canonical(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), default=str)


def _decode_json(value: Any, default: Any) -> Any:
    if value is None:
        return default
    try:
        return json.loads(value)
    except (TypeError, ValueError, json.JSONDecodeError):
        return default


def _report_unavailable(reason: str) -> HTTPException:
    return HTTPException(status_code=503, detail={"error": "report_unavailable", "reason": reason})


def _runtime_metadata_value(connection: Any, key: str) -> str:
    columns = {str(row[1]) for row in connection.execute("PRAGMA table_info(runtime_metadata)").fetchall()}
    key_column = next((name for name in ("key", "metadata_key", "name") if name in columns), None)
    value_column = next((name for name in ("value", "metadata_value") if name in columns), None)
    if key_column is None or value_column is None:
        raise _report_unavailable("runtime_metadata_schema_unavailable")
    row = connection.execute(
        f'SELECT "{value_column}" FROM runtime_metadata WHERE "{key_column}"=?',
        (key,),
    ).fetchone()
    if row is None or row[0] is None or not str(row[0]):
        raise _report_unavailable(f"missing_runtime_metadata:{key}")
    return str(row[0])


def _source_high_waters(connection: Any) -> dict[str, int]:
    try:
        return {
            table: int(connection.execute(f'SELECT COALESCE(MAX("{column}"),0) FROM "{table}"').fetchone()[0])
            for table, column in _SOURCE_HIGH_WATERS.items()
        }
    except Exception as exc:
        raise _report_unavailable("source_high_water_unavailable") from exc


def _schema_identity(connection: Any) -> dict[str, Any]:
    identity = _store_identity(connection)
    return {
        "database_identity": identity["database_identity"],
        "schema_version": identity["schema_version"],
        "identity_version": identity["identity_version"],
        "release_identity": identity["release_identity"],
    }


def _report_context(connection: Any) -> dict[str, Any]:
    return {
        "hmac_key": _runtime_metadata_value(connection, "report_cursor_hmac_key").encode("utf-8"),
        "database_incarnation_id": _runtime_metadata_value(connection, "database_incarnation_id"),
        "schema_identity": _schema_identity(connection),
        "source_high_waters": _source_high_waters(connection),
    }


def _public_snapshot(context: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "database_incarnation_id": context["database_incarnation_id"],
        "schema_identity": context["schema_identity"],
        "release_identity": context["schema_identity"]["release_identity"],
        "source_high_waters": context["source_high_waters"],
    }


def _encode_cursor(
    *,
    context: Mapping[str, Any],
    report_kind: str,
    filters: Mapping[str, Any],
    position: Mapping[str, Any],
) -> str:
    issued_at = time.time()
    payload = {
        "version": _CURSOR_VERSION,
        "report_kind": report_kind,
        "filters": dict(filters),
        "database_incarnation_id": context["database_incarnation_id"],
        "schema_identity": context["schema_identity"],
        "source_high_waters": context["source_high_waters"],
        "position": dict(position),
        "issued_at": issued_at,
        "expires_at": issued_at + _CURSOR_TTL_S,
    }
    signature = hmac.new(context["hmac_key"], _canonical(payload).encode(), hashlib.sha256).hexdigest()
    envelope = {"version": _CURSOR_VERSION, "payload": payload, "hmac_sha256": signature}
    return base64.urlsafe_b64encode(_canonical(envelope).encode()).decode().rstrip("=")


def _decode_cursor(
    value: str,
    *,
    context: Mapping[str, Any],
    report_kind: str,
    filters: Mapping[str, Any],
) -> dict[str, Any]:
    try:
        padding = "=" * (-len(value) % 4)
        decoded = base64.urlsafe_b64decode((value + padding).encode()).decode()
        envelope = json.loads(decoded)
    except (ValueError, TypeError, json.JSONDecodeError, UnicodeError) as exc:
        raise HTTPException(status_code=409, detail={"error": "invalid_report_cursor"}) from exc
    if not isinstance(envelope, dict) or envelope.get("version") != _CURSOR_VERSION:
        raise HTTPException(status_code=409, detail={"error": "unsupported_report_cursor_version"})
    payload = envelope.get("payload")
    supplied_signature = envelope.get("hmac_sha256")
    if not isinstance(payload, dict) or not isinstance(supplied_signature, str):
        raise HTTPException(status_code=409, detail={"error": "invalid_report_cursor"})
    expected_signature = hmac.new(context["hmac_key"], _canonical(payload).encode(), hashlib.sha256).hexdigest()
    if not hmac.compare_digest(supplied_signature, expected_signature):
        raise HTTPException(status_code=409, detail={"error": "invalid_report_cursor_signature"})
    if payload.get("version") != _CURSOR_VERSION:
        raise HTTPException(status_code=409, detail={"error": "unsupported_report_cursor_version"})
    now = time.time()
    if float(payload.get("issued_at", now + 1)) > now + 1:
        raise HTTPException(status_code=409, detail={"error": "invalid_report_cursor_time"})
    if float(payload.get("expires_at", 0)) < now:
        raise HTTPException(status_code=409, detail={"error": "expired_report_cursor"})
    if payload.get("report_kind") != report_kind:
        raise HTTPException(status_code=409, detail={"error": "cursor_report_kind_mismatch"})
    if payload.get("filters") != dict(filters):
        raise HTTPException(status_code=409, detail={"error": "cursor_filter_mismatch"})
    if payload.get("database_incarnation_id") != context["database_incarnation_id"]:
        raise HTTPException(status_code=409, detail={"error": "cursor_database_incarnation_mismatch"})
    if payload.get("schema_identity") != context["schema_identity"]:
        raise HTTPException(status_code=409, detail={"error": "cursor_schema_identity_mismatch"})
    high_waters = payload.get("source_high_waters")
    if not isinstance(high_waters, dict) or set(high_waters) != set(_SOURCE_HIGH_WATERS):
        raise HTTPException(status_code=409, detail={"error": "invalid_report_cursor_high_waters"})
    return payload


def _require_report_deadline(deadline: float, *, reason: str) -> None:
    if time.monotonic() >= float(deadline):
        raise HTTPException(
            status_code=503,
            detail={"error": "report_unavailable", "reason": reason},
        )


@contextmanager
def _report_store_lock(
    store: OperatorReceiptStore,
    *,
    deadline: float,
    reason: str,
) -> Iterator[None]:
    remaining = float(deadline) - time.monotonic()
    if remaining <= 0 or not store.lock.acquire(timeout=remaining):
        raise HTTPException(
            status_code=503,
            detail={"error": "report_unavailable", "reason": reason},
        )
    try:
        _require_report_deadline(deadline, reason=reason)
        yield
    finally:
        store.lock.release()


@contextmanager
def _read_snapshot(
    store: OperatorReceiptStore,
    *,
    deadline: float | None = None,
) -> Iterator[Any]:
    selected_deadline = (
        time.monotonic() + _REPORT_DEADLINE_S
        if deadline is None
        else float(deadline)
    )
    with _report_store_lock(
        store,
        deadline=selected_deadline,
        reason="report_lock_deadline_exceeded",
    ):
        def interrupt_when_expired() -> int:
            return int(time.monotonic() >= selected_deadline)

        store.connection.set_progress_handler(interrupt_when_expired, 1000)
        try:
            store.connection.execute("BEGIN")
            yield store.connection
            _require_report_deadline(
                selected_deadline,
                reason="report_query_deadline_exceeded",
            )
        except sqlite3.OperationalError as exc:
            if time.monotonic() >= selected_deadline or "interrupted" in str(exc).lower():
                raise HTTPException(
                    status_code=503,
                    detail={
                        "error": "report_unavailable",
                        "reason": "report_query_deadline_exceeded",
                    },
                ) from exc
            raise
        finally:
            store.connection.set_progress_handler(None, 0)
            if store.connection.in_transaction:
                store.connection.execute("ROLLBACK")


def _store_identity(connection: Any) -> dict[str, Any]:
    version = int(connection.execute("PRAGMA user_version").fetchone()[0])
    identity = connection.execute(
        "SELECT database_path,schema_version FROM runtime_store_identity WHERE identity_id=1"
    ).fetchone()
    return {
        "database_identity": "robot_authoritative_sqlite",
        "schema_version": version,
        "database_path_exposed": False,
        "identity_version": None if identity is None else int(identity["schema_version"]),
        "release_identity": _public_release_identity(current_release_identity()),
    }


def _tree_bytes(root: Path) -> int:
    total = 0
    if not root.exists():
        return 0
    for path in root.rglob("*"):
        if path.is_file() and not path.is_symlink():
            total += path.stat().st_size
    return total


def _base_filters(
    *,
    start: float | None,
    end: float | None,
    status: str | None,
    operation: str | None,
    action: str | None,
    channel: int | None,
    entrypoint: str | None = None,
    caller_class: str | None = None,
    control_class: str | None = None,
    protocol_job_id: str | None = None,
    protocol_action_id: str | None = None,
    lifecycle_stage_id: str | None = None,
    lifecycle_attempt_id: str | None = None,
    outcome: str | None = None,
    event_source: str | None = None,
    event_kind: str | None = None,
    pressure_stream_id: str | None = None,
    delivery_verified: bool | None = None,
    controller_acknowledged: bool | None = None,
    completion_verified: bool | None = None,
    hardware_postcondition_verified: bool | None = None,
    physical_effect_verified: bool | None = None,
    evidence_state: str | None = None,
    command_id: str | None = None,
    pipette_operation_id: str | None = None,
    connection_generation: int | None = None,
    ownership_generation: int | None = None,
    limit: int = 100,
) -> dict[str, Any]:
    normalized_end = time.time() if end is None else float(end)
    normalized_start = normalized_end - _DEFAULT_WINDOW_S if start is None else float(start)
    if normalized_start >= normalized_end:
        raise HTTPException(status_code=422, detail={"error": "invalid_time_window"})
    if normalized_end - normalized_start > _MAX_WINDOW_S:
        raise HTTPException(status_code=422, detail={"error": "time_window_exceeds_31_days"})
    if channel is not None and channel not in range(4):
        raise HTTPException(status_code=422, detail={"error": "invalid_channel"})
    return {
        "start": normalized_start,
        "end": normalized_end,
        "status": status,
        "operation": operation,
        "action": action,
        "channel": channel,
        "entrypoint": entrypoint,
        "caller_class": caller_class,
        "control_class": control_class,
        "protocol_job_id": protocol_job_id,
        "protocol_action_id": protocol_action_id,
        "lifecycle_stage_id": lifecycle_stage_id,
        "lifecycle_attempt_id": lifecycle_attempt_id,
        "outcome": outcome,
        "event_source": event_source,
        "event_kind": event_kind,
        "pressure_stream_id": pressure_stream_id,
        "delivery_verified": delivery_verified,
        "controller_acknowledged": controller_acknowledged,
        "completion_verified": completion_verified,
        "hardware_postcondition_verified": hardware_postcondition_verified,
        "physical_effect_verified": physical_effect_verified,
        "evidence_state": evidence_state,
        "command_id": command_id,
        "pipette_operation_id": pipette_operation_id,
        "connection_generation": connection_generation,
        "ownership_generation": ownership_generation,
        "limit": limit,
    }


def _command_scope_where(
    filters: Mapping[str, Any],
    *,
    high_waters: Mapping[str, Any],
    alias: str = "c",
    last_sequence: int | None = None,
) -> tuple[str, list[Any]]:
    clauses = [f"{alias}.sequence <= ?"]
    params: list[Any] = [int(high_waters["operator_commands"])]
    if last_sequence is not None:
        clauses.append(f"{alias}.sequence < ?")
        params.append(last_sequence)
    if filters.get("start") is not None:
        clauses.append(f"{alias}.updated_at >= ?")
        params.append(float(filters["start"]))
    if filters.get("end") is not None:
        clauses.append(f"{alias}.updated_at < ?")
        params.append(float(filters["end"]))
    for key, column in (
        ("status", "status"),
        ("operation", "operation"),
        ("action", "action_id"),
        ("entrypoint", "entrypoint_id"),
        ("caller_class", "caller_class"),
        ("control_class", "control_class"),
        ("outcome", "outcome"),
        ("evidence_state", "evidence_state"),
        ("command_id", "command_id"),
        ("connection_generation", "connection_generation"),
        ("ownership_generation", "ownership_generation"),
    ):
        if filters.get(key) is not None:
            clauses.append(f"{alias}.{column} = ?")
            params.append(filters[key] if key in {"connection_generation", "ownership_generation"} else str(filters[key]))
    for key, column in (
        ("delivery_verified", "delivery_verified"),
        ("controller_acknowledged", "controller_acknowledged"),
        ("completion_verified", "completion_verified"),
        ("hardware_postcondition_verified", "hardware_postcondition_verified"),
        ("physical_effect_verified", "physical_effect_verified"),
    ):
        if filters.get(key) is not None:
            clauses.append(f"{alias}.{column} = ?")
            params.append(int(bool(filters[key])))
    return " AND ".join(clauses), params


def _command_where(
    filters: Mapping[str, Any],
    *,
    high_waters: Mapping[str, Any],
    last_sequence: int | None = None,
) -> tuple[str, list[Any]]:
    where, params = _command_scope_where(
        filters, high_waters=high_waters, alias="c", last_sequence=last_sequence
    )
    clauses = [where]

    pipette_filters = (
        ("protocol_job_id", "p.protocol_job_id"),
        ("protocol_action_id", "p.protocol_action_id"),
        ("lifecycle_stage_id", "p.lifecycle_stage_id"),
        ("lifecycle_attempt_id", "p.lifecycle_attempt_id"),
        ("pipette_operation_id", "p.pipette_operation_id"),
    )
    needs_pipette = any(filters.get(key) is not None for key, _ in pipette_filters)
    needs_pipette = needs_pipette or filters.get("pressure_stream_id") is not None
    if needs_pipette:
        pipette_clauses = ["p.command_id=c.command_id", "p.rowid <= ?"]
        pipette_params: list[Any] = [int(high_waters["pipette_operations"])]
        for key, column in pipette_filters:
            if filters.get(key) is not None:
                pipette_clauses.append(f"{column} = ?")
                pipette_params.append(str(filters[key]))
        if filters.get("pressure_stream_id") is not None:
            stream_clauses = [
                "s.pipette_operation_id=p.pipette_operation_id",
                "s.rowid <= ?",
                "s.stream_session_id = ?",
            ]
            pipette_params.extend(
                [int(high_waters["pipette_pressure_streams"]), str(filters["pressure_stream_id"])]
            )
            if filters.get("channel") is not None:
                stream_clauses.append(
                    "EXISTS (SELECT 1 FROM json_each(s.channels_json) WHERE CAST(json_each.value AS INTEGER)=?)"
                )
                pipette_params.append(int(filters["channel"]))
            pipette_clauses.append(
                f"EXISTS (SELECT 1 FROM {_pressure_stream_source(high_waters)} s WHERE {' AND '.join(stream_clauses)})"
            )
        clauses.append(
            f"EXISTS (SELECT 1 FROM {_pipette_source(high_waters)} p WHERE {' AND '.join(pipette_clauses)})"
        )
        params.extend(pipette_params)

    event_filters = (("event_source", "e.event_source"), ("event_kind", "e.event_kind"))
    if any(filters.get(key) is not None for key, _ in event_filters):
        event_clauses = ["e.command_id=c.command_id", "e.event_id <= ?"]
        event_params: list[Any] = [int(high_waters["runtime_events"])]
        for key, column in event_filters:
            if filters.get(key) is not None:
                event_clauses.append(f"{column} = ?")
                event_params.append(str(filters[key]))
        if filters.get("channel") is not None:
            event_clauses.append("e.channel = ?")
            event_params.append(int(filters["channel"]))
        clauses.append(f"EXISTS (SELECT 1 FROM runtime_events e WHERE {' AND '.join(event_clauses)})")
        params.extend(event_params)
    elif filters.get("channel") is not None and filters.get("pressure_stream_id") is None:
        clauses.append(
            "EXISTS (SELECT 1 FROM pipette_channel_observations o "
            "WHERE o.command_id=c.command_id AND o.rowid <= ? AND o.channel=?)"
        )
        params.extend(
            [int(high_waters["pipette_channel_observations"]), int(filters["channel"])]
        )
    return " AND ".join(clauses), params


def _pipette_where(
    filters: Mapping[str, Any],
    *,
    high_waters: Mapping[str, Any],
    last_rowid: int | None = None,
) -> tuple[str, list[Any]]:
    clauses = ["p.rowid <= ?", "p.updated_at >= ?", "p.updated_at < ?"]
    params: list[Any] = [
        int(high_waters["pipette_operations"]),
        float(filters["start"]),
        float(filters["end"]),
    ]
    for key, column in (
        ("status", "p.status"),
        ("operation", "p.operation"),
        ("action", "p.action_id"),
        ("entrypoint", "p.entrypoint_id"),
        ("caller_class", "p.caller_class"),
        ("control_class", "p.control_class"),
        ("protocol_job_id", "p.protocol_job_id"),
        ("protocol_action_id", "p.protocol_action_id"),
        ("lifecycle_stage_id", "p.lifecycle_stage_id"),
        ("lifecycle_attempt_id", "p.lifecycle_attempt_id"),
        ("outcome", "p.outcome"),
        ("evidence_state", "p.evidence_state"),
        ("command_id", "p.command_id"),
        ("pipette_operation_id", "p.pipette_operation_id"),
        ("connection_generation", "p.connection_generation"),
        ("ownership_generation", "p.ownership_generation"),
    ):
        if filters.get(key) is not None:
            clauses.append(f"{column} = ?")
            params.append(filters[key] if key in {"connection_generation", "ownership_generation"} else str(filters[key]))
    for key, column in (
        ("delivery_verified", "p.delivery_verified"),
        ("controller_acknowledged", "p.controller_acknowledged"),
        ("completion_verified", "p.completion_verified"),
        ("hardware_postcondition_verified", "p.hardware_postcondition_verified"),
        ("physical_effect_verified", "p.physical_effect_verified"),
    ):
        if filters.get(key) is not None:
            clauses.append(f"{column} = ?")
            params.append(int(bool(filters[key])))
    event_filters = (("event_source", "e.event_source"), ("event_kind", "e.event_kind"))
    if any(filters.get(key) is not None for key, _ in event_filters):
        event_clauses = ["e.pipette_operation_id=p.pipette_operation_id", "e.event_id <= ?"]
        params.append(int(high_waters["runtime_events"]))
        for key, column in event_filters:
            if filters.get(key) is not None:
                event_clauses.append(f"{column} = ?")
                params.append(str(filters[key]))
        if filters.get("channel") is not None:
            event_clauses.append("e.channel = ?")
            params.append(int(filters["channel"]))
        clauses.append(f"EXISTS (SELECT 1 FROM runtime_events e WHERE {' AND '.join(event_clauses)})")
    elif filters.get("channel") is not None:
        clauses.append(
            "EXISTS (SELECT 1 FROM pipette_channel_observations o "
            "WHERE o.pipette_operation_id=p.pipette_operation_id AND o.rowid <= ? AND o.channel=?)"
        )
        params.extend(
            [int(high_waters["pipette_channel_observations"]), int(filters["channel"])]
        )
    if filters.get("pressure_stream_id") is not None:
        clauses.append(
            f"EXISTS (SELECT 1 FROM {_pressure_stream_source(high_waters)} s "
            "WHERE s.pipette_operation_id=p.pipette_operation_id AND s.rowid <= ? AND s.stream_session_id=?)"
        )
        params.extend(
            [int(high_waters["pipette_pressure_streams"]), str(filters["pressure_stream_id"])]
        )
    if last_rowid is not None:
        clauses.append("p.rowid < ?")
        params.append(last_rowid)
    return " AND ".join(clauses), params


def _event_where(
    filters: Mapping[str, Any],
    *,
    high_waters: Mapping[str, Any],
    last_event_id: int | None = None,
) -> tuple[str, list[Any]]:
    clauses = ["e.event_id <= ?", "e.observed_at >= ?", "e.observed_at < ?"]
    params: list[Any] = [
        int(high_waters["runtime_events"]),
        float(filters["start"]),
        float(filters["end"]),
    ]
    for key, column in (
        ("event_kind", "e.event_kind"),
        ("event_source", "e.event_source"),
        ("command_id", "e.command_id"),
        ("pipette_operation_id", "e.pipette_operation_id"),
    ):
        if filters.get(key) is not None:
            clauses.append(f"{column} = ?")
            params.append(str(filters[key]))
    if filters.get("channel") is not None:
        clauses.append("e.channel = ?")
        params.append(int(filters["channel"]))

    command_filter_fields = (
        "status",
        "operation",
        "action",
        "entrypoint",
        "caller_class",
        "control_class",
        "outcome",
        "evidence_state",
        "connection_generation",
        "ownership_generation",
        "delivery_verified",
        "controller_acknowledged",
        "completion_verified",
        "hardware_postcondition_verified",
        "physical_effect_verified",
    )
    if any(filters.get(key) is not None for key in command_filter_fields):
        command_filters = {key: filters.get(key) for key in command_filter_fields}
        command_filters.update({"start": None, "end": None})
        command_where, command_params = _command_scope_where(
            command_filters, high_waters=high_waters, alias="c"
        )
        clauses.append(
            f"EXISTS (SELECT 1 FROM {_command_source(high_waters)} c WHERE c.command_id=e.command_id AND {command_where})"
        )
        params.extend(command_params)

    pipette_filters = (
        ("protocol_job_id", "p.protocol_job_id"),
        ("protocol_action_id", "p.protocol_action_id"),
        ("lifecycle_stage_id", "p.lifecycle_stage_id"),
        ("lifecycle_attempt_id", "p.lifecycle_attempt_id"),
    )
    if any(filters.get(key) is not None for key, _ in pipette_filters) or filters.get("pressure_stream_id") is not None:
        pipette_clauses = ["p.pipette_operation_id=e.pipette_operation_id", "p.rowid <= ?"]
        pipette_params: list[Any] = [int(high_waters["pipette_operations"])]
        for key, column in pipette_filters:
            if filters.get(key) is not None:
                pipette_clauses.append(f"{column} = ?")
                pipette_params.append(str(filters[key]))
        if filters.get("pressure_stream_id") is not None:
            pipette_clauses.append(
                f"EXISTS (SELECT 1 FROM {_pressure_stream_source(high_waters)} s "
                "WHERE s.pipette_operation_id=p.pipette_operation_id AND s.rowid <= ? AND s.stream_session_id=?)"
            )
            pipette_params.extend(
                [int(high_waters["pipette_pressure_streams"]), str(filters["pressure_stream_id"])]
            )
        clauses.append(
            f"EXISTS (SELECT 1 FROM {_pipette_source(high_waters)} p WHERE {' AND '.join(pipette_clauses)})"
        )
        params.extend(pipette_params)
    if last_event_id is not None:
        clauses.append("e.event_id < ?")
        params.append(last_event_id)
    return " AND ".join(clauses), params


def _pressure_stream_where(
    filters: Mapping[str, Any],
    *,
    high_waters: Mapping[str, Any],
    stream_alias: str = "s",
    pipette_alias: str = "p",
    event_alias: str = "e",
    include_channel: bool = True,
    last_rowid: int | None = None,
) -> tuple[str, list[Any]]:
    clauses = [
        f"{stream_alias}.rowid <= ?",
        f"{stream_alias}.started_at >= ?",
        f"{stream_alias}.started_at < ?",
    ]
    params: list[Any] = [
        int(high_waters["pipette_pressure_streams"]),
        float(filters["start"]),
        float(filters["end"]),
    ]
    if filters.get("pressure_stream_id") is not None:
        clauses.append(f"{stream_alias}.stream_session_id = ?")
        params.append(str(filters["pressure_stream_id"]))
    if include_channel and filters.get("channel") is not None:
        clauses.append(
            f"EXISTS (SELECT 1 FROM json_each({stream_alias}.channels_json) "
            "WHERE CAST(json_each.value AS INTEGER)=?)"
        )
        params.append(int(filters["channel"]))

    pipette_clauses = [
        f"{pipette_alias}.pipette_operation_id={stream_alias}.pipette_operation_id",
        f"{pipette_alias}.rowid <= ?",
    ]
    pipette_params: list[Any] = [int(high_waters["pipette_operations"])]
    for key, column in (
        ("status", "status"),
        ("operation", "operation"),
        ("action", "action_id"),
        ("entrypoint", "entrypoint_id"),
        ("caller_class", "caller_class"),
        ("control_class", "control_class"),
        ("protocol_job_id", "protocol_job_id"),
        ("protocol_action_id", "protocol_action_id"),
        ("lifecycle_stage_id", "lifecycle_stage_id"),
        ("lifecycle_attempt_id", "lifecycle_attempt_id"),
        ("outcome", "outcome"),
        ("evidence_state", "evidence_state"),
        ("command_id", "command_id"),
        ("pipette_operation_id", "pipette_operation_id"),
        ("connection_generation", "connection_generation"),
        ("ownership_generation", "ownership_generation"),
    ):
        if filters.get(key) is not None:
            pipette_clauses.append(f"{pipette_alias}.{column} = ?")
            pipette_params.append(
                filters[key]
                if key in {"connection_generation", "ownership_generation"}
                else str(filters[key])
            )
    for key, column in (
        ("delivery_verified", "delivery_verified"),
        ("controller_acknowledged", "controller_acknowledged"),
        ("completion_verified", "completion_verified"),
        ("hardware_postcondition_verified", "hardware_postcondition_verified"),
        ("physical_effect_verified", "physical_effect_verified"),
    ):
        if filters.get(key) is not None:
            pipette_clauses.append(f"{pipette_alias}.{column} = ?")
            pipette_params.append(int(bool(filters[key])))

    event_filters = (("event_source", "event_source"), ("event_kind", "event_kind"))
    if any(filters.get(key) is not None for key, _ in event_filters):
        event_clauses = [
            f"{event_alias}.pipette_operation_id={pipette_alias}.pipette_operation_id",
            f"{event_alias}.event_id <= ?",
        ]
        pipette_params.append(int(high_waters["runtime_events"]))
        for key, column in event_filters:
            if filters.get(key) is not None:
                event_clauses.append(f"{event_alias}.{column} = ?")
                pipette_params.append(str(filters[key]))
        pipette_clauses.append(
            f"EXISTS (SELECT 1 FROM runtime_events {event_alias} WHERE {' AND '.join(event_clauses)})"
        )

    clauses.append(
        f"EXISTS (SELECT 1 FROM {_pipette_source(high_waters)} {pipette_alias} WHERE {' AND '.join(pipette_clauses)})"
    )
    params.extend(pipette_params)
    if last_rowid is not None:
        clauses.append(f"{stream_alias}.rowid < ?")
        params.append(last_rowid)
    return " AND ".join(clauses), params


def _pressure_chunk_where(
    filters: Mapping[str, Any],
    *,
    high_waters: Mapping[str, Any],
    stream_session_id: str | None = None,
    last_rowid: int | None = None,
) -> tuple[str, list[Any]]:
    clauses = ["pc.rowid <= ?"]
    params: list[Any] = [int(high_waters["pipette_pressure_chunks"])]
    if stream_session_id is not None:
        clauses.append("pc.stream_session_id = ?")
        params.append(stream_session_id)
    if filters.get("channel") is not None:
        clauses.append("pc.channel = ?")
        params.append(int(filters["channel"]))
    stream_where, stream_params = _pressure_stream_where(
        filters,
        high_waters=high_waters,
        stream_alias="s",
        pipette_alias="p",
        event_alias="re",
        include_channel=False,
    )
    clauses.append(
        f"EXISTS (SELECT 1 FROM {_pressure_stream_source(high_waters)} s "
        f"WHERE s.stream_session_id=pc.stream_session_id AND {stream_where})"
    )
    params.extend(stream_params)
    if last_rowid is not None:
        clauses.append("pc.rowid < ?")
        params.append(last_rowid)
    return " AND ".join(clauses), params


def _evidence_projection(
    connection: Any,
    row: Any,
    *,
    high_waters: Mapping[str, Any],
) -> dict[str, Any]:
    assessment_row = connection.execute(
        """
        SELECT event_id,observed_at,detail_json
        FROM runtime_evidence_events
        WHERE evidence_artifact_id=? AND event_kind='legal_hold_assessment' AND event_id<=?
        ORDER BY event_id DESC LIMIT 1
        """,
        (str(row["evidence_artifact_id"]), int(high_waters["runtime_evidence_events"])),
    ).fetchone()
    assessment = None
    effective_legal_hold = bool(row["legal_hold"])
    if assessment_row is not None:
        try:
            detail = json.loads(str(assessment_row["detail_json"]))
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            raise RuntimeError("stored evidence assessment detail is invalid") from exc
        if not isinstance(detail, Mapping) or type(detail.get("legal_hold")) is not bool:
            raise RuntimeError("stored evidence assessment detail is incomplete")
        if bool(row["legal_hold"]) != bool(detail["legal_hold"]):
            raise RuntimeError("stored evidence legal-hold projection contradicts history")
        assessment = {
            "event_id": str(assessment_row["event_id"]),
            "observed_at": float(assessment_row["observed_at"]),
            "legal_hold_requested": detail["legal_hold"],
            "assessment": detail.get("assessment"),
            "actor": detail.get("actor"),
            "retained_deadline": row["retention_deadline"],
            "legal_hold_projection_updated": True,
        }
        effective_legal_hold = bool(detail["legal_hold"])
    return {
        "evidence_artifact_id": str(row["evidence_artifact_id"]),
        "sha256": str(row["sha256"]),
        "byte_count": int(row["byte_count"]),
        "created_at": row["created_at"],
        "retention_deadline": row["retention_deadline"],
        "legal_hold": effective_legal_hold,
        "latest_legal_hold_assessment": assessment,
        "expiry_state": row["expiry_state"],
        "expiry_receipt_id": row["expiry_receipt_id"],
    }


def _semantic_query_response_verified(connection: Any, row: Any) -> bool:
    keys = set(row.keys()) if hasattr(row, "keys") else set()
    if "semantic_query_response_verified" in keys:
        return bool(row["semantic_query_response_verified"])
    pipette_operation_id = row["pipette_operation_id"] if "pipette_operation_id" in keys else None
    if pipette_operation_id is not None:
        attestation = connection.execute(
            "SELECT MAX(semantic_query_response_verified) FROM operator_plane_pipette_query_attestations WHERE pipette_operation_id=? AND command_id=?",
            (pipette_operation_id, row["command_id"]),
        ).fetchone()
    else:
        attestation = connection.execute(
            "SELECT MAX(semantic_query_response_verified) FROM operator_plane_pipette_query_attestations WHERE command_id=?",
            (row["command_id"],),
        ).fetchone()
    return bool(attestation is not None and attestation[0])


def _command_projection(
    connection: Any,
    row: Any,
    *,
    detail: bool = False,
    context: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    result = {
        "sequence": int(row["sequence"]),
        "command_id": str(row["command_id"]),
        "idempotency_key": str(row["idempotency_key"]),
        "operation": row["operation"],
        "command_kind": row["command_kind"],
        "entrypoint_id": row["entrypoint_id"],
        "caller_class": row["caller_class"],
        "control_class": row["control_class"],
        "action_id": row["action_id"],
        "status": row["status"],
        "outcome": row["outcome"],
        "failure_code": row["failure_code"],
        "ownership_generation": row["ownership_generation"],
        "connection_generation": row["connection_generation"],
        "started_at": row["started_at"],
        "admitted_at": row["admitted_at"],
        "dispatched_at": row["dispatched_at"],
        "finished_at": row["finished_at"],
        "duration_ms": row["duration_ms"],
        "delivery_verified": bool(row["delivery_verified"]),
        "controller_acknowledged": bool(row["controller_acknowledged"]),
        "completion_verified": bool(row["completion_verified"]),
        "semantic_query_response_verified": _semantic_query_response_verified(connection, row),
        "hardware_precondition_verified": bool(row["hardware_precondition_verified"]),
        "hardware_postcondition_verified": bool(row["hardware_postcondition_verified"]),
        "physical_effect_verified": bool(row["physical_effect_verified"]),
        "evidence_state": row["evidence_state"],
    }
    if detail:
        result["requested_inputs"] = _decode_json(row["requested_inputs_json"], {})
        result["effective_inputs"] = _decode_json(row["effective_inputs_json"], {})
        result["source_identity"] = _decode_json(row["source_identity_json"], {})
        result["transitions"] = [
            {
                "transition_id": int(item["transition_id"]),
                "state": item["state"],
                "observed_at": item["observed_at"],
                "detail": _decode_json(item["detail_json"], {}),
            }
            for item in connection.execute(
                "SELECT transition_id,state,observed_at,detail_json FROM operator_transitions WHERE command_id=? ORDER BY transition_id LIMIT ?",
                (row["command_id"], _DETAIL_CHILD_LIMIT),
            ).fetchall()
        ]
        if context is not None:
            evidence_page = _evidence_page(
                connection,
                str(row["command_id"]),
                _DETAIL_CHILD_LIMIT,
                None,
                context,
            )
            result["evidence"] = evidence_page["evidence"]
            result["evidence_preview"] = evidence_page["evidence"]
            result["evidence_continuation"] = {
                "returned_count": evidence_page["returned_count"],
                "filtered_total": evidence_page["filtered_total"],
                "has_more": evidence_page["has_more"],
                "next_cursor": evidence_page["next_cursor"],
            }
        pipette = connection.execute(
            "SELECT * FROM pipette_operations WHERE command_id=?",
            (row["command_id"],),
        ).fetchone()
        result["pipette"] = None if pipette is None else _pipette_projection(connection, pipette, detail=True)
    return result


def _pipette_projection(connection: Any, row: Any, *, detail: bool = False) -> dict[str, Any]:
    result = {
        "pipette_operation_id": str(row["pipette_operation_id"]),
        "command_id": str(row["command_id"]),
        "operation": row["operation"],
        "entrypoint_id": row["entrypoint_id"],
        "caller_class": row["caller_class"],
        "control_class": row["control_class"],
        "action_id": row["action_id"],
        "protocol_job_id": row["protocol_job_id"],
        "protocol_action_id": row["protocol_action_id"],
        "lifecycle_stage_id": row["lifecycle_stage_id"],
        "lifecycle_attempt_id": row["lifecycle_attempt_id"],
        "callback_session_id": row["callback_session_id"],
        "status": row["status"],
        "outcome": row["outcome"],
        "failure_code": row["failure_code"],
        "delivery_verified": bool(row["delivery_verified"]),
        "controller_acknowledged": bool(row["controller_acknowledged"]),
        "completion_verified": bool(row["completion_verified"]),
        "semantic_query_response_verified": _semantic_query_response_verified(connection, row),
        "hardware_postcondition_verified": bool(row["hardware_postcondition_verified"]),
        "physical_effect_verified": bool(row["physical_effect_verified"]),
        "evidence_state": row["evidence_state"],
    }
    if detail:
        op_id = str(row["pipette_operation_id"])
        result["channels"] = [
            {
                "observation_id": str(item["observation_id"]),
                "command_id": str(item["command_id"]),
                "pipette_operation_id": op_id,
                "channel": int(item["channel"]),
                "phase": item["phase"],
                "observed_at": item["observed_at"],
                "semantic_validity": item["semantic_validity"],
                "truth_source": item["truth_source"],
                "tip_loaded": None if item["tip_loaded"] is None else bool(item["tip_loaded"]),
                "pressure": item["pressure"],
                "pressure_units": item["pressure_units"],
                "status": item["status"],
                "error_code": item["error_code"],
                "firmware_class": item["firmware_class"],
                "detail": _decode_json(item["detail_json"], {}),
            }
            for item in connection.execute(
                "SELECT * FROM pipette_channel_observations WHERE pipette_operation_id=? ORDER BY observed_at,observation_id LIMIT ?",
                (op_id, _DETAIL_CHILD_LIMIT),
            ).fetchall()
        ]
        result["exchanges"] = [
            {
                "exchange_id": str(item["exchange_id"]),
                "transaction_id": item["transaction_id"],
                "channel": item["channel"],
                "transaction_phase": item["transaction_phase"],
                "command_family": item["command_family"],
                "matcher_name": item["matcher_name"],
                "tx_id": item["tx_id"],
                "expected_rx_id": item["expected_rx_id"],
                "observed_rx_id": item["observed_rx_id"],
                "tx_bytes": _decode_json(item["tx_bytes_json"], []),
                "rx_bytes": _decode_json(item["rx_bytes_json"], []),
                "delivery_verified": bool(item["delivery_verified"]),
                "semantic_match": bool(item["semantic_match"]),
                "controller_acknowledged": bool(item["controller_acknowledged"]),
                "completion_verified": bool(item["completion_verified"]),
                "completion_before_ack": bool(item["completion_before_ack"]),
                "sent_at": item["sent_at"],
                "received_at": item["received_at"],
                "ack_at": item["ack_at"],
                "completion_at": item["completion_at"],
            }
            for item in connection.execute(
                "SELECT * FROM pipette_transport_exchanges WHERE pipette_operation_id=? ORDER BY sent_at,exchange_id LIMIT ?",
                (op_id, _DETAIL_CHILD_LIMIT),
            ).fetchall()
        ]
        result["events"] = [
            {
                "event_id": int(item["event_id"]),
                "event_source": item["event_source"],
                "event_kind": item["event_kind"],
                "observed_at": item["observed_at"],
                "event": _decode_json(item["event_json"], {}),
            }
            for item in connection.execute(
                "SELECT * FROM runtime_events WHERE pipette_operation_id=? ORDER BY event_id LIMIT ?",
                (op_id, _DETAIL_CHILD_LIMIT),
            ).fetchall()
        ]
        result["pressure_streams"] = [
            {
                "stream_session_id": item["stream_session_id"],
                "pipette_operation_id": op_id,
                "channels": _decode_json(item["channels_json"], []),
                "sample_period_ms": item["sample_period_ms"],
                "started_at": item["started_at"],
                "stopped_at": item["stopped_at"],
                "source_generation": item["source_generation"],
                "reader_generation": item["reader_generation"],
                "offset_identity": item["offset_identity"],
                "terminal_state": item["terminal_state"],
                "loss_count": item["loss_count"],
            }
            for item in connection.execute(
                "SELECT * FROM pipette_pressure_streams WHERE pipette_operation_id=? ORDER BY started_at,stream_session_id LIMIT ?",
                (op_id, _DETAIL_CHILD_LIMIT),
            ).fetchall()
        ]
    return result


def _child_page_state(
    *, context: Mapping[str, Any], cursor: str | None, report_kind: str,
    filters: Mapping[str, Any], table: str,
) -> tuple[dict[str, Any], int, int | None]:
    page_context = dict(context)
    high_water = int(context["source_high_waters"][table])
    last_rowid = None
    if cursor:
        state = _decode_cursor(cursor, context=context, report_kind=report_kind, filters=filters)
        page_context["source_high_waters"] = state["source_high_waters"]
        high_water = int(state["source_high_waters"][table])
        last_rowid = int(state["position"]["last_rowid"])
    return page_context, high_water, last_rowid


def _child_page_body(
    *, context: Mapping[str, Any], report_kind: str, filters: Mapping[str, Any],
    high_water: int, rows: list[Any], total: int, collection_name: str,
    collection: list[dict[str, Any]],
) -> dict[str, Any]:
    limit = int(filters["limit"])
    has_more = len(rows) > limit
    visible_rows = rows[:limit]
    body = {
        "filters": dict(filters),
        "snapshot": {"high_water_rowid": high_water, **_public_snapshot(context)},
        "returned_count": len(visible_rows), "filtered_total": total,
        "has_more": has_more, "next_cursor": None,
        collection_name: collection[:limit],
    }
    if has_more and visible_rows:
        body["next_cursor"] = _encode_cursor(
            context=context, report_kind=report_kind, filters=filters,
            position={"last_rowid": int(visible_rows[-1]["rowid"])},
        )
    return body


def _transition_page(connection: Any, command_id: str, limit: int, cursor: str | None, context: Mapping[str, Any]) -> dict[str, Any]:
    filters = {"command_id": command_id, "limit": limit}
    page_context, high_water, last_rowid = _child_page_state(
        context=context, cursor=cursor, report_kind="command_transitions", filters=filters, table="operator_transitions"
    )
    clauses = ["command_id=?", "rowid<=?"]
    params: list[Any] = [command_id, high_water]
    total = int(connection.execute(f"SELECT COUNT(*) FROM operator_transitions WHERE {' AND '.join(clauses)}", params).fetchone()[0])
    if last_rowid is not None:
        clauses.append("rowid<?")
        params.append(last_rowid)
    rows = connection.execute(
        f"SELECT rowid AS rowid,transition_id,state,observed_at,detail_json FROM operator_transitions WHERE {' AND '.join(clauses)} ORDER BY rowid DESC LIMIT ?",
        [*params, limit + 1],
    ).fetchall()
    collection = [
        {"transition_id": int(row["transition_id"]), "state": row["state"], "observed_at": row["observed_at"], "detail": _decode_json(row["detail_json"], {})}
        for row in rows
    ]
    return {"command_id": command_id, **_child_page_body(context=page_context, report_kind="command_transitions", filters=filters, high_water=high_water, rows=rows, total=total, collection_name="transitions", collection=collection)}


def _evidence_page(
    connection: Any,
    command_id: str,
    limit: int,
    cursor: str | None,
    context: Mapping[str, Any],
) -> dict[str, Any]:
    filters = {"command_id": command_id, "limit": limit}
    page_context, high_water, last_rowid = _child_page_state(
        context=context,
        cursor=cursor,
        report_kind="command_evidence",
        filters=filters,
        table="runtime_evidence_objects",
    )
    link_high_water = int(page_context["source_high_waters"]["runtime_evidence_links"])
    total_clauses = [
        "o.rowid<=?",
        "EXISTS (SELECT 1 FROM runtime_evidence_links l "
        "WHERE l.evidence_artifact_id=o.evidence_artifact_id "
        "AND l.target_kind='command' AND l.target_identity=? "
        "AND l.evidence_link_id<=?)",
    ]
    total_params: list[Any] = [high_water, command_id, link_high_water]
    evidence_source = _evidence_source(page_context["source_high_waters"])
    total = int(
        connection.execute(
            f"SELECT COUNT(*) FROM {evidence_source} o WHERE {' AND '.join(total_clauses)}",
            total_params,
        ).fetchone()[0]
    )
    page_clauses = list(total_clauses)
    page_params = list(total_params)
    if last_rowid is not None:
        page_clauses.append("o.rowid<?")
        page_params.append(last_rowid)
    rows = connection.execute(
        f"SELECT o.* FROM {evidence_source} o "
        f"WHERE {' AND '.join(page_clauses)} ORDER BY o.rowid DESC LIMIT ?",
        [*page_params, limit + 1],
    ).fetchall()
    collection = [
        _evidence_projection(connection, row, high_waters=page_context["source_high_waters"])
        for row in rows
    ]
    return {
        "command_id": command_id,
        **_child_page_body(
            context=page_context,
            report_kind="command_evidence",
            filters=filters,
            high_water=high_water,
            rows=rows,
            total=total,
            collection_name="evidence",
            collection=collection,
        ),
    }


def _channel_page(connection: Any, operation_id: str, limit: int, cursor: str | None, context: Mapping[str, Any]) -> dict[str, Any]:
    filters = {"pipette_operation_id": operation_id, "limit": limit}
    page_context, high_water, last_rowid = _child_page_state(
        context=context, cursor=cursor, report_kind="pipette_channels", filters=filters, table="pipette_channel_observations"
    )
    clauses = ["pipette_operation_id=?", "rowid<=?"]
    params: list[Any] = [operation_id, high_water]
    total = int(connection.execute(f"SELECT COUNT(*) FROM pipette_channel_observations WHERE {' AND '.join(clauses)}", params).fetchone()[0])
    if last_rowid is not None:
        clauses.append("rowid<?")
        params.append(last_rowid)
    rows = connection.execute(
        f"SELECT rowid AS rowid,* FROM pipette_channel_observations WHERE {' AND '.join(clauses)} ORDER BY rowid DESC LIMIT ?",
        [*params, limit + 1],
    ).fetchall()
    collection = [
        {
            "observation_id": str(row["observation_id"]), "command_id": str(row["command_id"]),
            "pipette_operation_id": operation_id, "channel": int(row["channel"]), "phase": row["phase"],
            "observed_at": row["observed_at"], "semantic_validity": row["semantic_validity"],
            "truth_source": row["truth_source"], "tip_loaded": None if row["tip_loaded"] is None else bool(row["tip_loaded"]),
            "pressure": row["pressure"], "pressure_units": row["pressure_units"], "status": row["status"],
            "error_code": row["error_code"], "firmware_class": row["firmware_class"], "detail": _decode_json(row["detail_json"], {}),
        }
        for row in rows
    ]
    return {"pipette_operation_id": operation_id, **_child_page_body(context=page_context, report_kind="pipette_channels", filters=filters, high_water=high_water, rows=rows, total=total, collection_name="channels", collection=collection)}


def _exchange_page(connection: Any, operation_id: str, limit: int, cursor: str | None, context: Mapping[str, Any]) -> dict[str, Any]:
    filters = {"pipette_operation_id": operation_id, "limit": limit}
    page_context, high_water, last_rowid = _child_page_state(
        context=context, cursor=cursor, report_kind="pipette_exchanges", filters=filters, table="pipette_transport_exchanges"
    )
    clauses = ["pipette_operation_id=?", "rowid<=?"]
    params: list[Any] = [operation_id, high_water]
    total = int(connection.execute(f"SELECT COUNT(*) FROM pipette_transport_exchanges WHERE {' AND '.join(clauses)}", params).fetchone()[0])
    if last_rowid is not None:
        clauses.append("rowid<?")
        params.append(last_rowid)
    rows = connection.execute(
        f"SELECT rowid AS rowid,* FROM pipette_transport_exchanges WHERE {' AND '.join(clauses)} ORDER BY rowid DESC LIMIT ?",
        [*params, limit + 1],
    ).fetchall()
    collection = [
        {
            "exchange_id": str(row["exchange_id"]), "transaction_id": row["transaction_id"], "channel": row["channel"],
            "transaction_phase": row["transaction_phase"], "command_family": row["command_family"], "matcher_name": row["matcher_name"],
            "tx_id": row["tx_id"], "expected_rx_id": row["expected_rx_id"], "observed_rx_id": row["observed_rx_id"],
            "tx_bytes": _decode_json(row["tx_bytes_json"], []), "rx_bytes": _decode_json(row["rx_bytes_json"], []),
            "delivery_verified": bool(row["delivery_verified"]), "semantic_match": bool(row["semantic_match"]),
            "controller_acknowledged": bool(row["controller_acknowledged"]), "completion_verified": bool(row["completion_verified"]),
            "completion_before_ack": bool(row["completion_before_ack"]), "sent_at": row["sent_at"], "received_at": row["received_at"],
            "ack_at": row["ack_at"], "completion_at": row["completion_at"],
        }
        for row in rows
    ]
    return {"pipette_operation_id": operation_id, **_child_page_body(context=page_context, report_kind="pipette_exchanges", filters=filters, high_water=high_water, rows=rows, total=total, collection_name="exchanges", collection=collection)}


def _command_page(
    connection: Any,
    filters: dict[str, Any],
    cursor: str | None,
    context: Mapping[str, Any],
) -> dict[str, Any]:
    page_context = dict(context)
    high_water = int(context["source_high_waters"]["operator_commands"])
    last_sequence = None
    if cursor:
        state = _decode_cursor(cursor, context=context, report_kind="commands", filters=filters)
        page_context["source_high_waters"] = state["source_high_waters"]
        high_water = int(state["source_high_waters"]["operator_commands"])
        last_sequence = int(state["position"]["last_sequence"])
    high_waters = page_context["source_high_waters"]
    command_source = _command_source(high_waters)
    total_where, total_params = _command_where(filters, high_waters=high_waters)
    where, params = _command_where(
        filters, high_waters=high_waters, last_sequence=last_sequence
    )
    total = int(connection.execute(f"SELECT COUNT(*) FROM {command_source} c WHERE {total_where}", total_params).fetchone()[0])
    rows = connection.execute(
        f"SELECT c.* FROM {command_source} c WHERE {where} ORDER BY c.sequence DESC LIMIT ?",
        [*params, int(filters["limit"]) + 1],
    ).fetchall()
    has_more = len(rows) > int(filters["limit"])
    rows = rows[: int(filters["limit"])]
    body = {
        "filters": filters,
        "snapshot": {"high_water_sequence": high_water, **_public_snapshot(page_context)},
        "returned_count": len(rows),
        "filtered_total": total,
        "has_more": has_more,
        "next_cursor": None,
        "commands": [_command_projection(connection, row) for row in rows],
    }
    if has_more and rows:
        body["next_cursor"] = _encode_cursor(
            context=page_context,
            report_kind="commands",
            filters=filters,
            position={"last_sequence": int(rows[-1]["sequence"])},
        )
    return body


def _pipette_page(
    connection: Any,
    filters: dict[str, Any],
    cursor: str | None,
    context: Mapping[str, Any],
) -> dict[str, Any]:
    page_context = dict(context)
    high_water = int(context["source_high_waters"]["pipette_operations"])
    last_rowid = None
    if cursor:
        state = _decode_cursor(cursor, context=context, report_kind="pipette", filters=filters)
        page_context["source_high_waters"] = state["source_high_waters"]
        high_water = int(state["source_high_waters"]["pipette_operations"])
        last_rowid = int(state["position"]["last_rowid"])

    high_waters = page_context["source_high_waters"]
    pipette_source = _pipette_source(high_waters)
    total_where, total_params = _pipette_where(filters, high_waters=high_waters)
    where, params = _pipette_where(
        filters, high_waters=high_waters, last_rowid=last_rowid
    )
    total = int(connection.execute(f"SELECT COUNT(*) FROM {pipette_source} p WHERE {total_where}", total_params).fetchone()[0])
    rows = connection.execute(
        f"SELECT p.* FROM {pipette_source} p WHERE {where} ORDER BY p.rowid DESC LIMIT ?",
        [*params, int(filters["limit"]) + 1],
    ).fetchall()
    has_more = len(rows) > int(filters["limit"])
    rows = rows[: int(filters["limit"])]
    body = {
        "filters": filters,
        "snapshot": {"high_water_rowid": high_water, **_public_snapshot(page_context)},
        "returned_count": len(rows),
        "filtered_total": total,
        "has_more": has_more,
        "next_cursor": None,
        "pipette": [_pipette_projection(connection, row) for row in rows],
    }
    if has_more and rows:
        body["next_cursor"] = _encode_cursor(
            context=page_context,
            report_kind="pipette",
            filters=filters,
            position={"last_rowid": int(rows[-1]["rowid"])},
        )
    return body


def _summary(connection: Any, filters: dict[str, Any], context: Mapping[str, Any]) -> dict[str, Any]:
    high_water = int(context["source_high_waters"]["operator_commands"])
    high_waters = context["source_high_waters"]
    command_source = _command_source(high_waters)
    where, params = _command_where(filters, high_waters=high_waters)
    total = int(connection.execute(f"SELECT COUNT(*) FROM {command_source} c WHERE {where}", params).fetchone()[0])
    statuses = {
        str(row["status"]): int(row["count"])
        for row in connection.execute(f"SELECT c.status,COUNT(*) AS count FROM {command_source} c WHERE {where} GROUP BY c.status", params).fetchall()
    }
    aggregate = connection.execute(
        f"""
        SELECT
            COUNT(*) AS total,
            COALESCE(SUM(c.delivery_verified), 0) AS delivered,
            COALESCE(SUM(c.controller_acknowledged), 0) AS acknowledged,
            COALESCE(SUM(c.completion_verified), 0) AS completed,
            COALESCE(SUM(c.hardware_postcondition_verified), 0) AS postcondition,
            COALESCE(SUM(c.physical_effect_verified), 0) AS physical_effect,
            COALESCE(SUM(CASE WHEN c.failure_code IS NOT NULL OR c.status IN ('failed','blocked','rejected') THEN 1 ELSE 0 END), 0) AS failures,
            COALESCE(AVG(CASE WHEN c.duration_ms IS NOT NULL THEN c.duration_ms END), 0.0) AS average_ms,
            COALESCE(MAX(CASE WHEN c.duration_ms IS NOT NULL THEN c.duration_ms ELSE 0 END), 0.0) AS maximum_ms
        FROM {command_source} c WHERE {where}
        """,
        params,
    ).fetchone()
    total_for_rates = max(1, int(aggregate["total"]))
    error_rows = connection.execute(
        f"SELECT COALESCE(c.failure_code, c.outcome, 'unknown') AS code, COUNT(*) AS count FROM {command_source} c WHERE {where} AND (c.failure_code IS NOT NULL OR c.status IN ('failed','blocked','rejected')) GROUP BY code ORDER BY code",
        params,
    ).fetchall()
    errors = {str(row["code"]): int(row["count"]) for row in error_rows}
    scope_keys = tuple(key for key in filters if key not in {"start", "end", "limit"})
    scope = "filtered" if any(filters.get(key) is not None for key in scope_keys) else "window"
    pipette_where, pipette_params = _pipette_where(filters, high_waters=high_waters)
    pipette_total = int(
        connection.execute(
            f"SELECT COUNT(*) FROM {_pipette_source(high_waters)} p WHERE {pipette_where}", pipette_params
        ).fetchone()[0]
    )
    event_where, event_params = _event_where(filters, high_waters=high_waters)
    event_total = int(
        connection.execute(
            f"SELECT COUNT(*) FROM runtime_events e WHERE {event_where}", event_params
        ).fetchone()[0]
    )
    pressure_where, pressure_params = _pressure_stream_where(
        filters, high_waters=high_waters
    )
    pressure_stream_total = int(
        connection.execute(
            f"SELECT COUNT(*) FROM {_pressure_stream_source(high_waters)} s WHERE {pressure_where}",
            pressure_params,
        ).fetchone()[0]
    )
    chunk_where, chunk_params = _pressure_chunk_where(filters, high_waters=high_waters)
    pressure_chunk_total = int(
        connection.execute(
            f"SELECT COUNT(*) FROM pipette_pressure_chunks pc WHERE {chunk_where}",
            chunk_params,
        ).fetchone()[0]
    )
    return {
        "scope": scope,
        "filters": filters,
        "snapshot": {"high_water_sequence": high_water, **_public_snapshot(context)},
        "commands": {"total": total, "by_status": statuses},
        "pipette_operations": {"total": pipette_total},
        "runtime_events": {"total": event_total},
        "pressure": {"streams": pressure_stream_total, "chunks": pressure_chunk_total},
        "rates": {
            "delivery_rate": float(aggregate["delivered"]) / total_for_rates,
            "ack_rate": float(aggregate["acknowledged"]) / total_for_rates,
            "completion_rate": float(aggregate["completed"]) / total_for_rates,
            "postcondition_rate": float(aggregate["postcondition"]) / total_for_rates,
            "physical_effect_rate": float(aggregate["physical_effect"]) / total_for_rates,
            "failure_rate": float(aggregate["failures"]) / total_for_rates,
        },
        "latency": {"average_ms": float(aggregate["average_ms"]), "maximum_ms": float(aggregate["maximum_ms"])},
        "errors": {"by_code": errors},
    }


def _pressure_stream_page(
    connection: Any,
    filters: dict[str, Any],
    cursor: str | None,
    context: Mapping[str, Any],
) -> dict[str, Any]:
    page_context = dict(context)
    high_water = int(context["source_high_waters"]["pipette_pressure_streams"])
    last_rowid = None
    if cursor:
        state = _decode_cursor(cursor, context=context, report_kind="pressure_streams", filters=filters)
        page_context["source_high_waters"] = state["source_high_waters"]
        high_water = int(state["source_high_waters"]["pipette_pressure_streams"])
        last_rowid = int(state["position"]["last_rowid"])
    high_waters = page_context["source_high_waters"]
    total_where, total_params = _pressure_stream_where(
        filters, high_waters=high_waters
    )
    where, params = _pressure_stream_where(
        filters, high_waters=high_waters, last_rowid=last_rowid
    )
    pressure_source = _pressure_stream_source(high_waters)
    total = int(connection.execute(f"SELECT COUNT(*) FROM {pressure_source} s WHERE {total_where}", total_params).fetchone()[0])
    rows = connection.execute(
        f"SELECT s.* FROM {pressure_source} s WHERE {where} ORDER BY s.rowid DESC LIMIT ?",
        [*params, int(filters["limit"]) + 1],
    ).fetchall()
    has_more = len(rows) > int(filters["limit"])
    rows = rows[: int(filters["limit"])]
    body = {
        "filters": filters,
        "snapshot": {"high_water_rowid": high_water, **_public_snapshot(page_context)},
        "returned_count": len(rows),
        "filtered_total": total,
        "has_more": has_more,
        "next_cursor": None,
        "pressure_streams": [
            {
                "stream_session_id": row["stream_session_id"],
                "pipette_operation_id": row["pipette_operation_id"],
                "channels": _decode_json(row["channels_json"], []),
                "sample_period_ms": row["sample_period_ms"],
                "started_at": row["started_at"],
                "stopped_at": row["stopped_at"],
                "source_generation": row["source_generation"],
                "reader_generation": row["reader_generation"],
                "offset_identity": row["offset_identity"],
                "terminal_state": row["terminal_state"],
                "loss_count": row["loss_count"],
            }
            for row in rows
        ],
    }
    if has_more and rows:
        body["next_cursor"] = _encode_cursor(
            context=page_context,
            report_kind="pressure_streams",
            filters=filters,
            position={"last_rowid": int(rows[-1]["rowid"])},
        )
    return body


def _pressure_samples_page(
    connection: Any,
    stream_session_id: str,
    filters: dict[str, Any],
    cursor: str | None,
    context: Mapping[str, Any],
) -> dict[str, Any]:
    cursor_filters = {"stream_session_id": stream_session_id, **filters}
    page_context = dict(context)
    high_water = int(context["source_high_waters"]["pipette_pressure_chunks"])
    last_rowid = None
    if cursor:
        state = _decode_cursor(cursor, context=context, report_kind="pressure_samples", filters=cursor_filters)
        page_context["source_high_waters"] = state["source_high_waters"]
        high_water = int(state["source_high_waters"]["pipette_pressure_chunks"])
        last_rowid = int(state["position"]["last_rowid"])
    high_waters = page_context["source_high_waters"]
    total_where, total_params = _pressure_chunk_where(
        filters, high_waters=high_waters, stream_session_id=stream_session_id
    )
    where, params = _pressure_chunk_where(
        filters,
        high_waters=high_waters,
        stream_session_id=stream_session_id,
        last_rowid=last_rowid,
    )
    total = int(connection.execute(f"SELECT COUNT(*) FROM pipette_pressure_chunks pc WHERE {total_where}", total_params).fetchone()[0])
    rows = connection.execute(
        f"SELECT pc.rowid AS rowid,pc.* FROM pipette_pressure_chunks pc WHERE {where} ORDER BY pc.rowid DESC LIMIT ?",
        [*params, int(filters["limit"]) + 1],
    ).fetchall()
    has_more = len(rows) > int(filters["limit"])
    rows = rows[: int(filters["limit"])]
    body = {
        "stream_session_id": stream_session_id,
        "filters": filters,
        "snapshot": {"high_water_rowid": high_water, **_public_snapshot(page_context)},
        "returned_count": len(rows),
        "filtered_total": total,
        "has_more": has_more,
        "next_cursor": None,
        "samples": [
            {
                "chunk_id": row["chunk_id"],
                "channel": row["channel"],
                "chunk_sequence": row["chunk_sequence"],
                "sample_count": row["sample_count"],
                "lost_sample_count": row["lost_sample_count"],
                "units": row["units"],
                "sha256": row["sha256"],
                "byte_count": row["byte_count"],
                "evidence_artifact_id": row["evidence_artifact_id"],
                "summary": _decode_json(row["sample_summary_json"], {}),
            }
            for row in rows
        ],
    }
    if has_more and rows:
        body["next_cursor"] = _encode_cursor(
            context=page_context,
            report_kind="pressure_samples",
            filters=cursor_filters,
            position={"last_rowid": int(rows[-1]["rowid"])},
        )
    return body


def _event_page(
    connection: Any,
    filters: dict[str, Any],
    cursor: str | None,
    context: Mapping[str, Any],
) -> dict[str, Any]:
    page_context = dict(context)
    high_water = int(context["source_high_waters"]["runtime_events"])
    last_event_id = None
    if cursor:
        state = _decode_cursor(cursor, context=context, report_kind="events", filters=filters)
        page_context["source_high_waters"] = state["source_high_waters"]
        high_water = int(state["source_high_waters"]["runtime_events"])
        last_event_id = int(state["position"]["last_event_id"])
    high_waters = page_context["source_high_waters"]
    total_where, total_params = _event_where(filters, high_waters=high_waters)
    where, params = _event_where(
        filters, high_waters=high_waters, last_event_id=last_event_id
    )
    total = int(connection.execute(f"SELECT COUNT(*) FROM runtime_events e WHERE {total_where}", total_params).fetchone()[0])
    rows = connection.execute(
        f"SELECT * FROM runtime_events e WHERE {where} ORDER BY e.event_id DESC LIMIT ?",
        [*params, int(filters["limit"]) + 1],
    ).fetchall()
    has_more = len(rows) > int(filters["limit"])
    rows = rows[: int(filters["limit"])]
    body = {
        "event_kind": filters.get("event_kind"),
        "filters": filters,
        "snapshot": {"high_water_event_id": high_water, **_public_snapshot(page_context)},
        "returned_count": len(rows),
        "filtered_total": total,
        "has_more": has_more,
        "next_cursor": None,
        "events": [
            {
                "event_id": int(row["event_id"]),
                "command_id": row["command_id"],
                "pipette_operation_id": row["pipette_operation_id"],
                "event_source": row["event_source"],
                "event_kind": row["event_kind"],
                "channel": row["channel"],
                "observed_at": row["observed_at"],
                "event": _decode_json(row["event_json"], {}),
            }
            for row in rows
        ],
    }
    if has_more and rows:
        body["next_cursor"] = _encode_cursor(
            context=page_context,
            report_kind="events",
            filters=filters,
            position={"last_event_id": int(rows[-1]["event_id"])},
        )
    return body


def _write_export(
    store: OperatorReceiptStore,
    *,
    export_id: str,
    fmt: str,
    payload: Mapping[str, Any],
    deadline: float,
) -> tuple[str, int, str]:
    _require_report_deadline(deadline, reason="export_generation_deadline_exceeded")
    relpath = Path("report_exports") / f"{export_id}.{fmt}"
    if fmt == "json":
        encoder = json.JSONEncoder(
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            default=str,
        )
        pieces: list[str] = []
        for piece in encoder.iterencode(payload):
            _require_report_deadline(
                deadline,
                reason="export_generation_deadline_exceeded",
            )
            pieces.append(piece)
        raw = ("".join(pieces) + "\n").encode()
    else:
        output = io.StringIO()
        identity_value = payload.get("release_identity")
        identity = dict(identity_value) if isinstance(identity_value, Mapping) else {}
        source_value = identity.get("source")
        source = dict(source_value) if isinstance(source_value, Mapping) else {}
        image_value = identity.get("image")
        image = dict(image_value) if isinstance(image_value, Mapping) else {}
        deployment_value = identity.get("deployment")
        deployment = dict(deployment_value) if isinstance(deployment_value, Mapping) else {}
        writer = csv.DictWriter(
            output,
            fieldnames=[
                "command_id", "operation", "status", "outcome", "failure_code",
                "release_identity_status", "release_id", "source_commit", "source_tree",
                "image_id", "deployment_receipt_id",
            ],
        )
        writer.writeheader()
        for row in payload.get("commands", []):
            _require_report_deadline(deadline, reason="export_generation_deadline_exceeded")
            writer.writerow(
                {
                    **{key: row.get(key) for key in ("command_id", "operation", "status", "outcome", "failure_code")},
                    "release_identity_status": identity.get("status"),
                    "release_id": identity.get("release_id"),
                    "source_commit": source.get("commit"),
                    "source_tree": source.get("tree"),
                    "image_id": image.get("id"),
                    "deployment_receipt_id": deployment.get("receipt_id"),
                }
            )
        raw = output.getvalue().encode()
    _require_report_deadline(deadline, reason="export_generation_deadline_exceeded")
    digest = hashlib.sha256(raw).hexdigest()
    _require_report_deadline(deadline, reason="export_generation_deadline_exceeded")
    root_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    directory_flags = root_flags
    root_fd = os.open(str(store.root), root_flags)
    export_fd = None
    temporary_name = f".{export_id}.{uuid.uuid4().hex}.tmp"
    try:
        try:
            os.mkdir("report_exports", mode=0o700, dir_fd=root_fd)
        except FileExistsError:
            pass
        export_fd = os.open("report_exports", directory_flags, dir_fd=root_fd)
        artifact_fd = os.open(
            temporary_name,
            os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0),
            0o600,
            dir_fd=export_fd,
        )
        try:
            view = memoryview(raw)
            while view:
                _require_report_deadline(deadline, reason="export_generation_deadline_exceeded")
                written = os.write(artifact_fd, view)
                view = view[written:]
            _require_report_deadline(deadline, reason="export_generation_deadline_exceeded")
            os.fchmod(artifact_fd, 0o400)
            os.fsync(artifact_fd)
            _require_report_deadline(deadline, reason="export_generation_deadline_exceeded")
        finally:
            os.close(artifact_fd)
        _require_report_deadline(deadline, reason="export_generation_deadline_exceeded")
        os.link(
            temporary_name,
            relpath.name,
            src_dir_fd=export_fd,
            dst_dir_fd=export_fd,
            follow_symlinks=False,
        )
        os.unlink(temporary_name, dir_fd=export_fd)
        os.fsync(export_fd)
        _require_report_deadline(deadline, reason="export_generation_deadline_exceeded")
        os.fsync(root_fd)
        _require_report_deadline(deadline, reason="export_generation_deadline_exceeded")
    finally:
        try:
            if export_fd is not None:
                os.unlink(temporary_name, dir_fd=export_fd)
        except (FileNotFoundError, OSError):
            pass
        if export_fd is not None:
            os.close(export_fd)
        os.close(root_fd)
    return relpath.as_posix(), len(raw), digest


def _read_export_artifact(store: OperatorReceiptStore, row: Any) -> bytes:
    relative = Path(str(row["artifact_relpath"]))
    if (
        relative.is_absolute()
        or len(relative.parts) != 2
        or relative.parts[0] != "report_exports"
        or any(part in {"", ".", ".."} for part in relative.parts)
    ):
        raise HTTPException(status_code=409, detail={"error": "export_integrity_failure"})
    root_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_CLOEXEC", 0)
    nofollow = getattr(os, "O_NOFOLLOW", 0)
    root_fd = None
    current_fd = None
    try:
        root_fd = os.open(str(store.root), root_flags | nofollow)
        current_fd = root_fd
        for index, component in enumerate(relative.parts):
            flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | nofollow
            if index < len(relative.parts) - 1:
                flags |= getattr(os, "O_DIRECTORY", 0)
            next_fd = os.open(component, flags, dir_fd=current_fd)
            if current_fd != root_fd:
                os.close(current_fd)
            current_fd = next_fd
        before = os.fstat(current_fd)
        if not stat.S_ISREG(before.st_mode):
            raise HTTPException(status_code=409, detail={"error": "export_integrity_failure"})
        chunks: list[bytes] = []
        while True:
            chunk = os.read(current_fd, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        content = b"".join(chunks)
        after = os.fstat(current_fd)
        if before.st_size != after.st_size or len(content) != int(row["byte_count"]):
            raise HTTPException(status_code=409, detail={"error": "export_integrity_failure"})
        if hashlib.sha256(content).hexdigest() != str(row["sha256"]):
            raise HTTPException(status_code=409, detail={"error": "export_integrity_failure"})
        return content
    except HTTPException:
        raise
    except FileNotFoundError as exc:
        raise HTTPException(status_code=409, detail={"error": "export_artifact_missing"}) from exc
    except OSError as exc:
        raise HTTPException(status_code=409, detail={"error": "export_integrity_failure"}) from exc
    finally:
        if current_fd is not None and current_fd != root_fd:
            os.close(current_fd)
        if root_fd is not None:
            os.close(root_fd)


def _unlink_export_artifact(
    store: OperatorReceiptStore,
    relpath: str,
    *,
    expected_sha256: str,
    expected_bytes: int,
) -> None:
    relative = Path(relpath)
    if relative.is_absolute() or len(relative.parts) != 2 or relative.parts[0] != "report_exports":
        raise RuntimeError("export cleanup path escaped report_exports")
    from .storage_operations import _rename_noreplace_at

    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    root_fd = os.open(str(store.root), flags)
    export_fd = None
    tombstone = f".cleanup-{uuid.uuid4().hex}-{relative.name}"
    try:
        export_fd = os.open("report_exports", flags, dir_fd=root_fd)
        _rename_noreplace_at(export_fd, relative.name, tombstone)
        artifact_fd = os.open(
            tombstone,
            os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0),
            dir_fd=export_fd,
        )
        try:
            selected = os.fstat(artifact_fd)
            if not stat.S_ISREG(selected.st_mode) or int(selected.st_size) != int(expected_bytes):
                raise RuntimeError("export cleanup artifact identity changed")
            chunks: list[bytes] = []
            while True:
                chunk = os.read(artifact_fd, 1024 * 1024)
                if not chunk:
                    break
                chunks.append(chunk)
            if hashlib.sha256(b"".join(chunks)).hexdigest() != str(expected_sha256):
                raise RuntimeError("export cleanup artifact digest changed")
        finally:
            os.close(artifact_fd)
        os.unlink(tombstone, dir_fd=export_fd)
        os.fsync(export_fd)
        os.fsync(root_fd)
    finally:
        if export_fd is not None:
            os.close(export_fd)
        os.close(root_fd)


def _validated_export_receipt(connection: Any, row: Any) -> dict[str, Any]:
    try:
        receipt_mode = _validate_report_export_row(row)
    except StorageEvidenceError as exc:
        raise HTTPException(
            status_code=409,
            detail={"error": "export_receipt_integrity_failure"},
        ) from exc
    receipt = _decode_json(row["snapshot_json"], {})
    filters = _decode_json(row["filter_json"], None)
    artifact = receipt.get("artifact") if isinstance(receipt, dict) else None
    high_waters = receipt.get("source_high_waters") if isinstance(receipt, dict) else None
    filter_sha256 = (
        hashlib.sha256(_canonical(filters).encode()).hexdigest()
        if isinstance(filters, dict)
        else None
    )
    valid_numbers = False
    if isinstance(receipt, dict) and isinstance(artifact, dict) and isinstance(high_waters, dict):
        try:
            valid_numbers = (
                int(receipt.get("row_count", -1)) == int(row["row_count"])
                and int(artifact.get("byte_count", -1)) == int(row["byte_count"])
                and all(int(high_waters[table]) >= 0 for table in _SOURCE_HIGH_WATERS)
            )
        except (KeyError, TypeError, ValueError, RuntimeError):
            valid_numbers = False

    evidence_id = report_export_evidence_id(str(row["export_id"]))
    deadline = report_export_retention_deadline(float(row["created_at"]))
    evidence_objects = connection.execute(
        "SELECT * FROM runtime_evidence_objects WHERE evidence_artifact_id=?", (evidence_id,)
    ).fetchall()
    evidence_events = connection.execute(
        "SELECT * FROM runtime_evidence_events WHERE evidence_artifact_id=?", (evidence_id,)
    ).fetchall()
    evidence_links = connection.execute(
        "SELECT * FROM runtime_evidence_links WHERE evidence_artifact_id=?", (evidence_id,)
    ).fetchall()
    evidence_valid = False
    evidence_state = "integrity_failed"
    evidence_legal_hold = False
    evidence_available = False
    if len(evidence_objects) == 1:
        obj = evidence_objects[0]
        expected_detail = {
            "export_id": str(row["export_id"]),
            "evidence_artifact_id": evidence_id,
            "relpath": str(row["artifact_relpath"]),
            "sha256": str(row["sha256"]),
            "byte_count": int(row["byte_count"]),
            "format": str(row["format"]),
            "filter_sha256": str(row["filter_sha256"]),
            "filter_json": str(row["filter_json"]),
            "created_at": float(row["created_at"]),
            "retention_deadline": deadline,
        }
        published_events = [
            event
            for event in evidence_events
            if str(event["event_kind"]) == "published"
            and float(event["observed_at"]) == float(row["created_at"])
            and _decode_json(event["detail_json"], None) == expected_detail
        ]
        export_links = [
            link
            for link in evidence_links
            if str(link["target_kind"]) == "export"
            and str(link["target_identity"]) == str(row["export_id"])
            and link["command_id"] is None
            and link["pipette_operation_id"] is None
            and str(link["link_kind"]) == f"report_export:{row['export_id']}"
            and float(link["created_at"]) == float(row["created_at"])
        ]
        try:
            evidence_state = str(obj["expiry_state"])
            evidence_legal_hold = bool(obj["legal_hold"])
            active_relpath = obj["active_relpath"]
            retained_state = evidence_state in {"active", "retained", "expiry_pending"}
            expired_state = evidence_state == "expired"
            state_valid = evidence_state in {
                "active",
                "retained",
                "expiry_pending",
                "expired",
                "missing",
                "integrity_failed",
            }
            path_valid = (
                retained_state
                and active_relpath == row["artifact_relpath"]
            ) or (
                expired_state
                and active_relpath is None
                and obj["expiry_receipt_id"] is not None
            ) or evidence_state in {"missing", "integrity_failed"}
            evidence_available = bool(
                retained_state and active_relpath == row["artifact_relpath"]
            )
            evidence_valid = (
                len(published_events) == 1
                and len(export_links) == 1
                and obj["command_id"] is None
                and obj["pipette_operation_id"] is None
                and obj["original_relpath"] == row["artifact_relpath"]
                and obj["sha256"] == row["sha256"]
                and int(obj["byte_count"]) == int(row["byte_count"])
                and float(obj["created_at"]) == float(row["created_at"])
                and float(obj["retention_deadline"]) == deadline
                and state_valid
                and path_valid
            )
        except (TypeError, ValueError, RuntimeError):
            evidence_valid = False
    if not evidence_valid:
        raise HTTPException(status_code=409, detail={"error": "export_receipt_integrity_failure"})
    if receipt_mode == "legacy_snapshot_v0":
        return {
            "receipt_schema": "bioxp.operator_report_export_receipt.legacy_snapshot_v0",
            "publisher_identity": _EXPORT_PUBLISHER_IDENTITY,
            "export_id": str(row["export_id"]),
            "evidence_artifact_id": evidence_id,
            "created_at": float(row["created_at"]),
            "retention_deadline": deadline,
            "filter_sha256": str(row["filter_sha256"]),
            "row_count": int(row["row_count"]),
            "artifact": {
                "format": str(row["format"]),
                "sha256": str(row["sha256"]),
                "byte_count": int(row["byte_count"]),
            },
            "legacy_snapshot": dict(receipt),
            "release_identity": {
                "status": "unavailable",
                "verified": False,
                "reason_code": "legacy_export_without_release_identity",
            },
            "public_download_available": False,
            "evidence_state": evidence_state,
            "legal_hold": evidence_legal_hold,
            "evidence_available": evidence_available,
        }
    if (
        not isinstance(receipt, dict)
        or receipt.get("receipt_schema") != "bioxp.operator_report_export_receipt.v1"
        or receipt.get("publisher_identity") != _EXPORT_PUBLISHER_IDENTITY
        or receipt.get("export_id") != str(row["export_id"])
        or receipt.get("evidence_artifact_id") not in {None, evidence_id}
        or receipt.get("created_at") not in {None, float(row["created_at"])}
        or receipt.get("retention_deadline") not in {None, deadline}
        or not isinstance(filters, dict)
        or receipt.get("normalized_filters") != filters
        or filter_sha256 != str(row["filter_sha256"])
        or receipt.get("filter_sha256") != filter_sha256
        or not isinstance(high_waters, dict)
        or frozenset(high_waters) not in {
            frozenset(_SOURCE_HIGH_WATERS),
            _LEGACY_SOURCE_HIGH_WATER_KEYS,
        }
        or not isinstance(receipt.get("schema_identity"), dict)
        or not isinstance(receipt.get("release_identity"), dict)
        or receipt.get("release_identity") != receipt["schema_identity"].get("release_identity")
        or _contains_absolute_filesystem_path(receipt)
        or not isinstance(receipt.get("database_incarnation_id"), str)
        or not receipt["database_incarnation_id"]
        or not valid_numbers
        or not isinstance(artifact, dict)
        or artifact.get("format") != str(row["format"])
        or artifact.get("sha256") != str(row["sha256"])
        or artifact.get("relpath") != str(row["artifact_relpath"])
        or str(row["status"]) != "completed"
    ):
        raise HTTPException(status_code=409, detail={"error": "export_receipt_integrity_failure"})
    projected_receipt = dict(receipt)
    projected_receipt["evidence_state"] = evidence_state
    projected_receipt["legal_hold"] = evidence_legal_hold
    projected_receipt["evidence_available"] = evidence_available
    if not evidence_available:
        projected_receipt["public_download_available"] = False
    return projected_receipt


def _export_publication_state(receipt: Mapping[str, Any]) -> str:
    if receipt.get("evidence_available") is not True:
        return str(receipt.get("evidence_state") or "integrity_failed")
    if receipt.get("public_download_available") is False:
        return "legacy_confined"
    return "public"


def reconcile_operator_report_exports(store: Any) -> dict[str, int]:
    with store.lock:
        with runtime_lifecycle_lock(store.root, exclusive=True):
            return reconcile_report_exports(store.root)


_PUBLIC_CHECK_FIELDS = {
    "schema": ("status",),
    "store_identity": ("status", "recorded_schema_version", "current_schema_version"),
    "integrity": ("status",),
    "foreign_keys": ("status",),
    "durability": ("status", "journal_mode", "synchronous"),
    "checkpoint": ("status", "age_seconds"),
    "backup": ("status", "age_seconds", "backup_id"),
    "writer": ("status", "writer_status", "queue_depth", "telemetry_available", "mode"),
    "storage": (
        "status",
        "database_bytes",
        "wal_bytes",
        "wal_threshold_bytes",
        "free_bytes",
        "minimum_free_bytes",
        "evidence_bytes",
        "backup_bytes",
    ),
}
_PUBLIC_DEGRADED_REASON_CODES = frozenset(
    {
        "schema_or_migration_identity",
        "runtime_store_identity",
        "integrity_check",
        "foreign_key_check",
        "sqlite_durability",
        "checkpoint_freshness_or_identity",
        "backup_freshness_or_integrity",
        "audit_writer_evidence",
        "storage_threshold",
        "unclassified_health_reason",
    }
)


def _public_health_projection(value: Mapping[str, Any]) -> dict[str, Any]:
    """Publish an explicit path-free audit-health allowlist."""
    checks_value = value.get("checks")
    checks = checks_value if isinstance(checks_value, Mapping) else {}
    projected_checks: dict[str, Any] = {}
    for check_name, fields in _PUBLIC_CHECK_FIELDS.items():
        row_value = checks.get(check_name)
        row = row_value if isinstance(row_value, Mapping) else {}
        projected_checks[check_name] = {field: row.get(field) for field in fields}

    release_value = value.get("release_identity")
    release = release_value if isinstance(release_value, Mapping) else {}
    public_release = _public_release_identity(release)
    counts_value = value.get("counts")
    counts = counts_value if isinstance(counts_value, Mapping) else {}
    private_reasons = [
        reason
        for reason in value.get("degraded_reasons", [])
        if isinstance(reason, str)
    ]
    degraded_reasons = [
        reason for reason in private_reasons if reason in _PUBLIC_DEGRADED_REASON_CODES
    ]
    if any(reason not in _PUBLIC_DEGRADED_REASON_CODES for reason in private_reasons):
        degraded_reasons.append("unclassified_health_reason")
    return {
        "schema": value.get("schema"),
        "status": value.get("status"),
        "generated_at": value.get("generated_at"),
        "degraded_reasons": degraded_reasons,
        "checks": projected_checks,
        "counts": {
            key: counts.get(key)
            for key in (
                "commands",
                "pipette_operations",
                "retained_evidence",
                "pending_expiry_evidence",
                "integrity_failures",
            )
        },
        "release_identity": public_release,
        "physical_admission_gate_added": False,
    }


def create_operator_reports_router(
    store: Any,
    *,
    writer_health_provider: Callable[[], Mapping[str, Any]] | None = None,
) -> APIRouter:
    router = APIRouter(
        prefix="/operator",
        tags=["operator-reports"],
        responses={
            404: {"model": _ReportErrorResponse},
            409: {"model": _ReportErrorResponse},
            503: {"model": _ReportErrorResponse},
        },
    )

    def filters(
        start: float | None = None,
        end: float | None = None,
        status: str | None = None,
        operation: str | None = None,
        action: str | None = None,
        channel: int | None = Query(default=None, ge=0, le=3),
        entrypoint: str | None = None,
        caller_class: str | None = None,
        control_class: str | None = None,
        protocol_job_id: str | None = None,
        protocol_action_id: str | None = None,
        lifecycle_stage_id: str | None = None,
        outcome: str | None = None,
        event_source: str | None = None,
        event_kind: str | None = None,
        pressure_stream_id: str | None = None,
        delivery_verified: bool | None = None,
        controller_acknowledged: bool | None = None,
        completion_verified: bool | None = None,
        hardware_postcondition_verified: bool | None = None,
        physical_effect_verified: bool | None = None,
        evidence_state: str | None = None,
        command_id: str | None = None,
        pipette_operation_id: str | None = None,
        connection_generation: int | None = Query(default=None, ge=0),
        ownership_generation: int | None = Query(default=None, ge=0),
        limit: int = Query(100, ge=1, le=1000),
    ) -> dict[str, Any]:
        return _base_filters(
            start=start, end=end, status=status, operation=operation, action=action, channel=channel,
            entrypoint=entrypoint, caller_class=caller_class, control_class=control_class,
            protocol_job_id=protocol_job_id, protocol_action_id=protocol_action_id,
            lifecycle_stage_id=lifecycle_stage_id, outcome=outcome, event_source=event_source,
            event_kind=event_kind, pressure_stream_id=pressure_stream_id, delivery_verified=delivery_verified,
            controller_acknowledged=controller_acknowledged, completion_verified=completion_verified,
            hardware_postcondition_verified=hardware_postcondition_verified,
            physical_effect_verified=physical_effect_verified, evidence_state=evidence_state,
            command_id=command_id, pipette_operation_id=pipette_operation_id,
            connection_generation=connection_generation, ownership_generation=ownership_generation,
            limit=limit,
        )

    @router.get("/reports/summary", response_model=_ReportSummaryResponse)
    def report_summary(
        selected: dict[str, Any] = Depends(filters),
    ) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            context = _report_context(connection)
            return _summary(connection, selected, context)

    @router.get("/reports/commands", response_model=_CommandPageResponse)
    def report_commands(
        selected: dict[str, Any] = Depends(filters),
        cursor: str | None = None,
    ) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            context = _report_context(connection)
            return _command_page(connection, selected, cursor, context)

    @router.get("/reports/commands/{command_id}", response_model=_CommandDetailResponse)
    def report_command_detail(command_id: str) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            context = _report_context(connection)
            row = connection.execute("SELECT * FROM operator_commands WHERE command_id=?", (command_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "command_not_found", "command_id": command_id})
            return {**_command_projection(connection, row, detail=True, context=context), "snapshot": _public_snapshot(context), "child_page_limit": _DETAIL_CHILD_LIMIT}

    @router.get(
        "/reports/commands/{command_id}/transitions",
        response_model=_CommandTransitionPageResponse,
    )
    def report_command_transitions(command_id: str, limit: int = Query(100, ge=1, le=1000), cursor: str | None = None) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            context = _report_context(connection)
            if connection.execute("SELECT 1 FROM operator_commands WHERE command_id=?", (command_id,)).fetchone() is None:
                raise HTTPException(status_code=404, detail={"error": "command_not_found", "command_id": command_id})
            return _transition_page(connection, command_id, limit, cursor, context)

    @router.get(
        "/reports/commands/{command_id}/evidence",
        response_model=_CommandEvidencePageResponse,
    )
    def report_command_evidence(command_id: str, limit: int = Query(100, ge=1, le=1000), cursor: str | None = None) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            context = _report_context(connection)
            if connection.execute("SELECT 1 FROM operator_commands WHERE command_id=?", (command_id,)).fetchone() is None:
                raise HTTPException(status_code=404, detail={"error": "command_not_found", "command_id": command_id})
            return _evidence_page(connection, command_id, limit, cursor, context)

    @router.get("/reports/pipette", response_model=_PipettePageResponse)
    def report_pipette(
        selected: dict[str, Any] = Depends(filters),
        cursor: str | None = None,
    ) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            context = _report_context(connection)
            return _pipette_page(connection, selected, cursor, context)

    @router.get(
        "/reports/pipette/{pipette_operation_id}",
        response_model=_PipetteDetailResponse,
    )
    def report_pipette_detail(pipette_operation_id: str) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            context = _report_context(connection)
            row = connection.execute("SELECT * FROM pipette_operations WHERE pipette_operation_id=?", (pipette_operation_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "pipette_operation_not_found", "pipette_operation_id": pipette_operation_id})
            return {**_pipette_projection(connection, row, detail=True), "snapshot": _public_snapshot(context), "child_page_limit": _DETAIL_CHILD_LIMIT}

    @router.get(
        "/reports/pipette/{pipette_operation_id}/channels",
        response_model=_PipetteChannelPageResponse,
    )
    def report_pipette_channels(pipette_operation_id: str, limit: int = Query(100, ge=1, le=1000), cursor: str | None = None) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            context = _report_context(connection)
            row = connection.execute("SELECT * FROM pipette_operations WHERE pipette_operation_id=?", (pipette_operation_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "pipette_operation_not_found"})
            return _channel_page(connection, pipette_operation_id, limit, cursor, context)

    @router.get(
        "/reports/pipette/{pipette_operation_id}/exchanges",
        response_model=_PipetteExchangePageResponse,
    )
    def report_pipette_exchanges(pipette_operation_id: str, limit: int = Query(100, ge=1, le=1000), cursor: str | None = None) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            context = _report_context(connection)
            row = connection.execute("SELECT * FROM pipette_operations WHERE pipette_operation_id=?", (pipette_operation_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "pipette_operation_not_found"})
            return _exchange_page(connection, pipette_operation_id, limit, cursor, context)

    @router.get("/reports/events", response_model=_EventPageResponse)
    def report_events(selected: dict[str, Any] = Depends(filters), cursor: str | None = None) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            context = _report_context(connection)
            return _event_page(connection, selected, cursor, context)

    @router.get("/reports/events/{event_id}", response_model=_EventDetailResponse)
    def report_event_detail(event_id: int) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            context = _report_context(connection)
            row = connection.execute("SELECT * FROM runtime_events WHERE event_id=?", (event_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "event_not_found"})
            return {"event_id": int(row["event_id"]), "command_id": row["command_id"], "pipette_operation_id": row["pipette_operation_id"], "event_source": row["event_source"], "event_kind": row["event_kind"], "channel": row["channel"], "observed_at": row["observed_at"], "event": _decode_json(row["event_json"], {}), "snapshot": _public_snapshot(context)}

    @router.get(
        "/reports/pressure-streams",
        response_model=_PressureStreamPageResponse,
    )
    def report_pressure_streams(selected: dict[str, Any] = Depends(filters), cursor: str | None = None) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            context = _report_context(connection)
            return _pressure_stream_page(connection, selected, cursor, context)

    @router.get(
        "/reports/pressure-streams/{stream_session_id}",
        response_model=_PressureStreamDetailResponse,
    )
    def report_pressure_stream_detail(stream_session_id: str) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            context = _report_context(connection)
            row = connection.execute(
                f"SELECT * FROM {_pressure_stream_source(context['source_high_waters'])} WHERE stream_session_id=?",
                (stream_session_id,),
            ).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "pressure_stream_not_found"})
            chunks = connection.execute("SELECT * FROM pipette_pressure_chunks WHERE stream_session_id=? ORDER BY channel,chunk_sequence LIMIT ?", (stream_session_id, _DETAIL_CHILD_LIMIT)).fetchall()
            return {"stream_session_id": row["stream_session_id"], "pipette_operation_id": row["pipette_operation_id"], "channels": _decode_json(row["channels_json"], []), "sample_period_ms": row["sample_period_ms"], "started_at": row["started_at"], "stopped_at": row["stopped_at"], "terminal_state": row["terminal_state"], "loss_count": row["loss_count"], "chunks": [{"chunk_id": c["chunk_id"], "channel": c["channel"], "chunk_sequence": c["chunk_sequence"], "sample_count": c["sample_count"], "lost_sample_count": c["lost_sample_count"], "units": c["units"], "sha256": c["sha256"], "byte_count": c["byte_count"], "evidence_artifact_id": c["evidence_artifact_id"]} for c in chunks], "child_page_limit": _DETAIL_CHILD_LIMIT, "snapshot": _public_snapshot(context)}

    @router.get(
        "/reports/pressure-streams/{stream_session_id}/samples",
        response_model=_PressureSamplePageResponse,
    )
    def report_pressure_stream_samples(stream_session_id: str, selected: dict[str, Any] = Depends(filters), cursor: str | None = None) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            context = _report_context(connection)
            return _pressure_samples_page(connection, stream_session_id, selected, cursor, context)

    @router.post("/reports/exports", response_model=_ExportCreatedResponse)
    def create_export(payload: _ExportRequest = Body(default_factory=_ExportRequest)) -> dict[str, Any]:
        export_deadline = time.monotonic() + _EXPORT_DEADLINE_S
        fmt = payload.format
        limit = int(payload.limit)
        selected = _base_filters(
            **{field: getattr(payload, field) for field in _FILTER_FIELDS},
            limit=limit,
        )
        if _contains_absolute_filesystem_path(selected):
            raise HTTPException(
                status_code=422,
                detail={"error": "absolute_filter_value_not_allowed"},
            )
        export_id = uuid.uuid4().hex
        evidence_id = report_export_evidence_id(export_id)
        filter_json = _canonical(selected)
        filter_sha256 = hashlib.sha256(filter_json.encode()).hexdigest()
        published_relpath: str | None = None
        published_sha256: str | None = None
        published_bytes: int | None = None
        try:
            with _report_store_lock(
                store,
                deadline=export_deadline,
                reason="export_lock_deadline_exceeded",
            ):
                with runtime_lifecycle_lock(
                    store.root,
                    exclusive=False,
                    deadline=export_deadline,
                ):
                    with _read_snapshot(store, deadline=export_deadline) as connection:
                        context = _report_context(connection)
                        page = _command_page(connection, selected, None, context)
                        snapshot = page["snapshot"]
                        export_payload = {
                            "schema_version": "bioxp.operator_report_export.v1",
                            "release_identity": snapshot["release_identity"],
                            "filters": selected,
                            "snapshot": snapshot,
                            "commands": page["commands"],
                        }
                    row_count = len(page["commands"])
                    relpath, byte_count, digest = _write_export(
                        store,
                        export_id=export_id,
                        fmt=fmt,
                        payload=export_payload,
                        deadline=export_deadline,
                    )
                    _require_report_deadline(
                        export_deadline,
                        reason="export_generation_deadline_exceeded",
                    )
                    published_relpath = relpath
                    published_sha256 = digest
                    published_bytes = byte_count
                    created_at = time.time()
                    retention_deadline = report_export_retention_deadline(created_at)
                    receipt = {
                        "receipt_schema": "bioxp.operator_report_export_receipt.v1",
                        "publisher_identity": _EXPORT_PUBLISHER_IDENTITY,
                        "export_id": export_id,
                        "evidence_artifact_id": evidence_id,
                        "created_at": created_at,
                        "retention_deadline": retention_deadline,
                        "normalized_filters": selected,
                        "filter_sha256": filter_sha256,
                        "source_high_waters": snapshot["source_high_waters"],
                        "schema_identity": snapshot["schema_identity"],
                        "release_identity": snapshot["release_identity"],
                        "database_incarnation_id": snapshot["database_incarnation_id"],
                        "row_count": row_count,
                        "artifact": {
                            "format": fmt,
                            "sha256": digest,
                            "byte_count": byte_count,
                            "relpath": relpath,
                        },
                    }
                    evidence_detail = {
                        "export_id": export_id,
                        "evidence_artifact_id": evidence_id,
                        "relpath": relpath,
                        "sha256": digest,
                        "byte_count": byte_count,
                        "format": fmt,
                        "filter_sha256": filter_sha256,
                        "filter_json": filter_json,
                        "created_at": created_at,
                        "retention_deadline": retention_deadline,
                    }
                    _require_report_deadline(
                        export_deadline,
                        reason="export_generation_deadline_exceeded",
                    )
                    store.connection.set_progress_handler(
                        lambda: int(time.monotonic() >= export_deadline),
                        1000,
                    )
                    store.connection.execute("BEGIN IMMEDIATE")
                    try:
                        store.connection.execute(
                            "INSERT INTO runtime_evidence_objects("
                            "evidence_artifact_id,command_id,pipette_operation_id,original_relpath,active_relpath,"
                            "sha256,byte_count,created_at,retention_deadline,legal_hold,expiry_state,expiry_receipt_id,updated_at"
                            ") VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?)",
                            (evidence_id, None, None, relpath, relpath, digest, byte_count, created_at,
                             retention_deadline, 0, "active", None, created_at),
                        )
                        _require_report_deadline(export_deadline, reason="export_generation_deadline_exceeded")
                        store.connection.execute(
                            "INSERT INTO runtime_evidence_events(evidence_artifact_id,event_kind,observed_at,detail_json) VALUES(?,?,?,?)",
                            (evidence_id, "published", created_at, _canonical(evidence_detail)),
                        )
                        _require_report_deadline(export_deadline, reason="export_generation_deadline_exceeded")
                        store.connection.execute(
                            "INSERT INTO runtime_evidence_links(evidence_artifact_id,target_kind,target_identity,command_id,pipette_operation_id,link_kind,created_at) VALUES(?,?,?,?,?,?,?)",
                            (evidence_id, "export", export_id, None, None, f"report_export:{export_id}", created_at),
                        )
                        _require_report_deadline(export_deadline, reason="export_generation_deadline_exceeded")
                        store.connection.execute(
                            "INSERT INTO report_exports(export_id,format,filter_json,filter_sha256,snapshot_json,row_count,sha256,byte_count,status,artifact_relpath,created_at,completed_at) VALUES(?,?,?,?,?,?,?,?,?,?,?,?)",
                            (export_id, fmt, filter_json, filter_sha256, _canonical(receipt), row_count,
                             digest, byte_count, "completed", relpath, created_at, created_at),
                        )
                        _require_report_deadline(export_deadline, reason="export_generation_deadline_exceeded")
                        store.connection.execute("COMMIT")
                        _require_report_deadline(export_deadline, reason="export_generation_deadline_exceeded")
                    except Exception:
                        if store.connection.in_transaction:
                            store.connection.execute("ROLLBACK")
                        raise
                    finally:
                        store.connection.set_progress_handler(None, 0)
        except Exception as exc:
            if published_relpath is not None:
                try:
                    with _report_store_lock(
                        store,
                        deadline=export_deadline,
                        reason="export_cleanup_deadline_exceeded",
                    ):
                        report_bound = store.connection.execute(
                            "SELECT 1 FROM report_exports WHERE export_id=?", (export_id,)
                        ).fetchone()
                        evidence_bound = store.connection.execute(
                            "SELECT 1 FROM runtime_evidence_objects WHERE evidence_artifact_id=?", (evidence_id,)
                        ).fetchone()
                        if (
                            report_bound is None
                            and evidence_bound is None
                            and published_sha256 is not None
                            and published_bytes is not None
                        ):
                            try:
                                _unlink_export_artifact(
                                    store,
                                    published_relpath,
                                    expected_sha256=published_sha256,
                                    expected_bytes=published_bytes,
                                )
                            except FileNotFoundError:
                                pass
                except HTTPException:
                    pass
            if isinstance(exc, HTTPException):
                raise
            raise HTTPException(
                status_code=503,
                detail={"error": "export_generation_failed"},
            ) from exc
        return {
            "export_id": export_id,
            "evidence_artifact_id": evidence_id,
            "status": "completed",
            "format": fmt,
            "row_count": row_count,
            "sha256": digest,
            "byte_count": byte_count,
            "release_identity": export_payload["release_identity"],
            "download": f"/operator/reports/exports/{export_id}/download",
        }

    @router.get("/reports/exports", response_model=_ExportListResponse)
    def export_list(limit: int = Query(100, ge=1, le=1000)) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            rows = connection.execute(
                "SELECT * FROM report_exports ORDER BY created_at DESC,export_id LIMIT ?",
                (limit,),
            ).fetchall()
            items = []
            for row in rows:
                receipt = _validated_export_receipt(connection, row)
                if receipt.get("evidence_available") is True:
                    _read_export_artifact(store, row)
                items.append({
                    "export_id": row["export_id"],
                    "format": row["format"],
                    "row_count": row["row_count"],
                    "sha256": row["sha256"],
                    "byte_count": row["byte_count"],
                    "status": row["status"],
                    "created_at": row["created_at"],
                    "release_identity": receipt["release_identity"],
                    "publication_state": _export_publication_state(receipt),
                    "evidence_state": receipt["evidence_state"],
                    "legal_hold": receipt["legal_hold"],
                    "evidence_available": receipt["evidence_available"],
                    "download": None
                    if receipt.get("public_download_available") is False
                    else f"/operator/reports/exports/{row['export_id']}/download",
                })
            return {"items": items, "returned_count": len(items), "limit": limit}

    @router.get(
        "/reports/exports/{export_id}",
        response_model=_ExportMetadataResponse,
    )
    def export_metadata(export_id: str) -> dict[str, Any]:
        with _read_snapshot(store) as connection:
            row = connection.execute("SELECT * FROM report_exports WHERE export_id=?", (export_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "export_not_found"})
            receipt = _validated_export_receipt(connection, row)
            if receipt.get("evidence_available") is True:
                _read_export_artifact(store, row)
            return {
                "export_id": row["export_id"],
                "format": row["format"],
                "filter": None
                if receipt.get("public_download_available") is False
                else _decode_json(row["filter_json"], {}),
                "filter_sha256": row["filter_sha256"],
                "snapshot": receipt,
                "receipt": receipt,
                "release_identity": receipt["release_identity"],
                "row_count": row["row_count"],
                "sha256": row["sha256"],
                "byte_count": row["byte_count"],
                "status": row["status"],
                "created_at": row["created_at"],
                "completed_at": row["completed_at"],
                "publication_state": _export_publication_state(receipt),
                "evidence_state": receipt["evidence_state"],
                "legal_hold": receipt["legal_hold"],
                "evidence_available": receipt["evidence_available"],
                "download": None
                if receipt.get("public_download_available") is False
                else f"/operator/reports/exports/{export_id}/download",
            }

    @router.get("/reports/exports/{export_id}/download")
    def export_download(export_id: str):
        with _read_snapshot(store) as connection:
            row = connection.execute("SELECT * FROM report_exports WHERE export_id=?", (export_id,)).fetchone()
            if row is None:
                raise HTTPException(status_code=404, detail={"error": "export_not_found"})
            receipt = _validated_export_receipt(connection, row)
            if receipt.get("evidence_available") is not True:
                raise HTTPException(
                    status_code=409,
                    detail={
                        "error": "export_evidence_unavailable",
                        "evidence_state": receipt.get("evidence_state"),
                    },
                )
            if receipt.get("public_download_available") is False:
                raise HTTPException(
                    status_code=409,
                    detail={"error": "legacy_export_not_publicly_releasable"},
                )
            content = _read_export_artifact(store, row)
            headers = {
                "X-Content-SHA256": str(row["sha256"]),
                "Content-Disposition": f'attachment; filename="bioxp-report-{export_id}.{row["format"]}"',
            }
            media_type = "application/json" if row["format"] == "json" else "text/csv"
            return Response(content=content, media_type=media_type, headers=headers)

    @router.get("/audit-health", response_model=_AuditHealthResponse)
    def audit_health() -> dict[str, Any]:
        writer_health: Mapping[str, Any] | None = None
        if writer_health_provider is not None:
            try:
                provided = writer_health_provider()
                if isinstance(provided, Mapping):
                    writer_health = provided
            except Exception:
                writer_health = {
                    "status": "unavailable",
                    "queue_depth": None,
                    "error": "unavailable",
                }
        with _read_snapshot(store) as connection:
            report = audit_health_report(
                store.root,
                connection=connection,
                writer_health=writer_health,
            )
            return _public_health_projection(report)

    return router
