"""Closed direct-liquid receipt projection.

Original BMS model blob dae94c9, SHA256
5d930d075a083460649c483dc20c5cb582433b28c96a2a9787c5b135a3892803.
Original channel/effect validators retained; approved strict semantic truth and
required original stored hardware_query classification added.
No runtime dependency on BMS or current robot authority.
"""
from __future__ import annotations

from typing import Literal
from pydantic import BaseModel, ConfigDict, Field, JsonValue, StrictBool, StrictInt, model_validator


class PipetteReceiptTruth(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    delivery_verified: StrictBool
    controller_acknowledged: StrictBool
    completion_verified: StrictBool
    semantic_query_response_verified: StrictBool
    hardware_precondition_verified: StrictBool
    hardware_postcondition_verified: StrictBool
    physical_effect_verified: Literal[False]
    physical_effect_claim_suppressed: Literal[True]


class PipetteReadbackRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    include_data: StrictBool = False


class PipetteReadbackChannel(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    channel: Literal[0, 1, 2, 3]
    semantic_ok: StrictBool
    firmware: dict[str, JsonValue]
    status: dict[str, JsonValue]
    tip: dict[str, JsonValue]
    pressure: dict[str, JsonValue] | None
    data: dict[str, JsonValue] | None


class PipetteReadbackResponse(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    ok: StrictBool
    semantic_ok: StrictBool
    available: StrictBool
    channel_count: Literal[4]
    channels_constructed_unconditionally: list[Literal[0, 1, 2, 3]] = Field(min_length=4, max_length=4)
    channels: list[PipetteReadbackChannel] = Field(min_length=4, max_length=4)
    include_data: StrictBool
    live_query_performed: Literal[True]
    truth_source: Literal["live_hardware_queries"]
    hardware_truth_level: Literal["hardware_query"]
    delivery_verified: Literal[False]
    controller_acknowledged: Literal[False]
    completion_verified: Literal[False]
    hardware_postcondition_verified: Literal[False]
    physical_effect_verified: Literal[False]
    oem_source_anchor: Literal["ClassPipetteCollection constructor/readback; ClassPipette QueryFirmware/Q1/?31/?57/getData"]
    receipt_id: str = Field(pattern=r"^[0-9a-f]{32}$")
    receipt_truth: PipetteReceiptTruth

    @model_validator(mode="after")
    def validate_four_channel_readback(self):
        expected = [0, 1, 2, 3]
        if self.channels_constructed_unconditionally != expected:
            raise ValueError("channels_constructed_unconditionally must be [0, 1, 2, 3]")
        if [channel.channel for channel in self.channels] != expected:
            raise ValueError("active readback channels must be the ordered unique IDs 0..3")
        if self.include_data is False and any(channel.data is not None for channel in self.channels):
            raise ValueError("channel data must be null when include_data is false")
        if self.include_data is True and any(channel.data is None for channel in self.channels):
            raise ValueError("channel data must be present when include_data is true")
        return self


class PipetteApplicationDependency(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    bound: StrictBool
    authority: str | None = Field(default=None, max_length=240)
    generation: StrictInt
    state: dict[str, JsonValue]
    blockers: list[str] = Field(max_length=32)


class PipetteApplicationStep(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    action: str = Field(min_length=1, max_length=240)
    mutates: StrictBool
    location_id: StrictInt | None = None
    wire_command: str | None = Field(default=None, max_length=120)
    current: StrictInt | None = None
    owner: Literal["deck", "gantry", "z", "pressure", "pipette", "machine_state"]


class PipetteApplicationPlanResponse(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    ok: StrictBool
    operation: Literal["load_tip", "move_to_waste", "detect_fluid", "plunger_up", "plunger_down"]
    mode: Literal["plan_only"]
    execution_admitted: Literal[False]
    motion_commanded: Literal[False]
    liquid_mutation_commanded: Literal[False]
    controller_acknowledged: Literal[False]
    completion_verified: Literal[False]
    physical_effect_verified: Literal[False]
    state_reconciled: Literal[False]
    requested_inputs: dict[str, JsonValue]
    effective_inputs: None = None
    steps: list[PipetteApplicationStep] = Field(min_length=1, max_length=32)
    dependencies: dict[str, PipetteApplicationDependency] = Field(min_length=1, max_length=6)
    required_dependencies: list[str] = Field(min_length=1, max_length=6)
    missing_dependencies: list[str] = Field(max_length=6)
    dependency_blockers: list[str] = Field(max_length=64)
    dependencies_satisfied: StrictBool
    required_completion_evidence: list[str] = Field(max_length=32)
    constants: dict[str, JsonValue]
    oem_source_anchor: str = Field(min_length=1, max_length=1000)
    blocker: Literal["physical_pipette_execution_not_authorized", "application_dependencies_unbound"]
    receipt_id: str = Field(pattern=r"^[0-9a-f]{32}$")
    receipt_truth: PipetteReceiptTruth

    @model_validator(mode="after")
    def validate_plan_dependencies(self):
        required = set(self.required_dependencies)
        if set(self.dependencies) != required:
            raise ValueError("application plan dependency map must match required_dependencies")
        if not set(self.missing_dependencies).issubset(required):
            raise ValueError("application plan has an unknown missing dependency")
        satisfied = not self.missing_dependencies and not self.dependency_blockers
        if self.dependencies_satisfied != satisfied or self.ok != satisfied:
            raise ValueError("application plan dependency status is internally inconsistent")
        expected_blocker = (
            "physical_pipette_execution_not_authorized"
            if satisfied
            else "application_dependencies_unbound"
        )
        if self.blocker != expected_blocker:
            raise ValueError("application plan blocker does not match dependency status")
        return self


class DirectPlanInputs(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    operation: Literal["load_tip", "move_to_waste", "detect_fluid", "plunger_up", "plunger_down"]
    home_z_after: StrictBool
    tip_tray: str | None = Field(default=None, max_length=120)
    tip_well: str | None = Field(default=None, max_length=32)
    tip_type: StrictInt | None = None
    tip_location: Literal[0, 1, 2, 3] | None = None
    fluid_class: Literal["TC", "MS", "OC", "RC", "STRIP"] | None = None

    @model_validator(mode="after")
    def validate_producer_required_inputs(self):
        if self.operation == 'load_tip' and any(getattr(self, k) is None for k in ('tip_tray', 'tip_well', 'tip_type', 'tip_location')):
            raise ValueError('incomplete load-tip request')
        if self.operation == 'detect_fluid' and self.fluid_class is None:
            raise ValueError('missing fluid class')
        return self


class DirectReadbackInputs(PipetteReadbackRequest):
    include_data: StrictBool = Field(...)


class DirectFailureReceipt(BaseModel):
    """Existing record_failure shape; private detail is validated, never returned."""
    model_config = ConfigDict(extra="forbid", strict=True)
    ok: Literal[False]
    outcome: Literal["failed", "rejected"]
    error: str = Field(max_length=8192)
    failure_code: str = Field(min_length=1, max_length=240)
    runtime_binding: dict[str, JsonValue]
    completion_ambiguous: Literal[False]
    reconciliation_required: Literal[False]
    retry_forbidden: Literal[False]
    physical_effect_verified: Literal[False]


class DirectRequestRecord(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    command_id: str = Field(min_length=1, max_length=160)
    pipette_operation_id: str | None = Field(min_length=1, max_length=160)
    canonical_request_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    operation: str = Field(min_length=1, max_length=160)
    entrypoint_id: str = Field(min_length=1, max_length=160)
    caller_class: str = Field(min_length=1, max_length=160)
    control_class: str = Field(min_length=1, max_length=160)
    action_id: str = Field(min_length=1, max_length=240)
    command_status: str = Field(min_length=1, max_length=120, pattern=r'^[A-Za-z0-9_.:-]+$')
    pipette_status: str | None = Field(min_length=1, max_length=120, pattern=r'^[A-Za-z0-9_.:-]+$')
    outcome: str | None = Field(min_length=1, max_length=120, pattern=r'^[A-Za-z0-9_.:-]+$')
    failure_code: str | None = Field(min_length=1, max_length=240, pattern=r'^[A-Za-z0-9_.:-]+$')
    ownership_generation: StrictInt = Field(ge=0)
    connection_generation: StrictInt | None = Field(ge=0)
    requested_inputs: DirectReadbackInputs | DirectPlanInputs
    result: PipetteReadbackResponse | PipetteApplicationPlanResponse | None


class DirectRequestLookupResponse(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True, populate_by_name=True)
    schema_name: Literal["bioxp.direct-liquid.lookup.v1"] = Field(alias="schema")
    request_kind: Literal["readback", "application_plan"]
    idempotency_key: str = Field(pattern=r"^[A-Za-z0-9][A-Za-z0-9._:/-]{7,199}$")
    lookup_state: Literal["unknown", "pending", "incomplete", "resolved", "conflict", "unavailable"]
    reason: Literal["identity_not_found", "nonterminal", "outcome_unresolved", "receipt_incomplete", "identity_scope_conflict", "store_unavailable", "stored_binding_invalid"] | None
    retry_forbidden: Literal[True]
    live_query_performed: Literal[False]
    record: DirectRequestRecord | None


def lookup_envelope(kind, key, state, reason, record=None):
    value = dict(schema="bioxp.direct-liquid.lookup.v1", request_kind=kind,
                 idempotency_key=key, lookup_state=state, reason=reason,
                 retry_forbidden=True, live_query_performed=False, record=record)
    # Validation is closed; preserve original optional-key omission in stored inputs/results.
    DirectRequestLookupResponse.model_validate(value)
    return value


def project_lookup_row(row, kind, key):
    import json
    from ..runtime_audit_store import request_digest

    def finish(state, reason, record=None):
        return lookup_envelope(kind, key, state, reason, record)

    operation = row['operation']
    family = ('live_readback', 'direct.liquid.readback', 'direct_api', 'hardware_query')
    if kind == 'application_plan':
        family = (operation, 'legacy.record', 'legacy', 'pipette_state_command')
        if operation not in {'application_plan:' + v for v in ('load_tip', 'move_to_waste', 'detect_fluid', 'plunger_up', 'plunger_down')}:
            return finish('conflict', 'identity_scope_conflict')
    identity_fields = ('operation', 'entrypoint_id', 'caller_class', 'control_class')
    if tuple(row[k] for k in identity_fields) != family or row['action_id'] != 'pipette.' + operation or row['command_kind'] != 'pipette':
        return finish('conflict', 'identity_scope_conflict')
    requested = json.loads(row['requested_inputs_json'])
    source = json.loads(row['source_identity_json'])
    inputs_model = DirectReadbackInputs if kind == 'readback' else DirectPlanInputs
    normalized = inputs_model.model_validate(requested).model_dump(exclude_none=True)
    if normalized != requested or not isinstance(source, dict) or not source:
        raise ValueError('stored input normalization/source invalid')
    if kind == 'application_plan' and operation != 'application_plan:' + requested['operation']:
        raise ValueError('plan operation binding invalid')
    child = row['pipette_operation_id'] is not None
    if child:
        for field in (*identity_fields, 'action_id', 'ownership_generation', 'connection_generation'):
            if row['p_' + field] != row[field]:
                raise ValueError('child identity binding invalid')
        if json.loads(row['p_requested_inputs_json']) != requested or json.loads(row['p_source_identity_json']) != source:
            raise ValueError('child source/request binding invalid')
    digest_input = {k: row[k] for k in (*identity_fields, 'action_id', 'ownership_generation')}
    digest_input.update(source_identity=source, requested_inputs=requested,
                        lifecycle_stage_id=row['lifecycle_stage_id'], lifecycle_attempt_id=row['lifecycle_attempt_id'])
    if request_digest(digest_input) != row['canonical_request_sha256']:
        raise ValueError('historical digest invalid')
    record = {k: row[k] for k in (*identity_fields, 'command_id', 'pipette_operation_id', 'canonical_request_sha256',
                               'action_id', 'ownership_generation', 'connection_generation', 'outcome', 'failure_code')}
    record.update(command_status=row['status'], pipette_status=row['p_status'], requested_inputs=requested, result=None)
    if not child:
        return finish('incomplete', 'receipt_incomplete', record)
    pending = {'reserved', 'queued', 'admitted', 'dispatched', 'acknowledged', 'executing', 'running', 'blocked'}
    terminal = {'completed', 'observed', 'failed', 'rejected', 'cleared', 'cancelled'}
    if row['status'] in pending and row['p_status'] in pending:
        return finish('pending', 'nonterminal', record)
    if row['status'] != row['p_status'] or row['status'] not in terminal or row['outcome'] != row['p_outcome'] or row['failure_code'] != row['p_failure_code']:
        return finish('incomplete', 'outcome_unresolved', record)
    receipt = json.loads(row['p_receipt_json'])
    if not isinstance(receipt, dict) or receipt.get('schema') != 'bioxp.pipette.receipt.v1':
        if isinstance(receipt, dict) and receipt.get('ok') is False and row['status'] in {'failed', 'rejected', 'cleared', 'cancelled'}:
            failure = DirectFailureReceipt.model_validate(receipt)
            if json.loads(row['response_summary_json']) != receipt or failure.failure_code != row['failure_code'] or failure.outcome != row['outcome']:
                raise ValueError('failure receipt binding invalid')
            if not failure.runtime_binding or any(source.get(k) != v for k, v in failure.runtime_binding.items()):
                raise ValueError('failure runtime source binding invalid')
            if failure.runtime_binding.get('callback_session_id') != row['callback_session_id']:
                raise ValueError('failure callback binding invalid')
            return finish('resolved', None, record)
        return finish('incomplete', 'receipt_incomplete', record)
    if receipt.get('operation') != operation or receipt.get('requested_inputs') != requested or receipt.get('ownership_epoch') != row['ownership_generation']:
        raise ValueError('receipt request/ownership binding invalid')
    runtime = receipt['runtime_binding']
    if not isinstance(runtime, dict) or any(source.get(k) != v for k, v in runtime.items()):
        raise ValueError('receipt runtime/source binding invalid')
    if child and runtime.get('callback_session_id') != row['callback_session_id']:
        raise ValueError('receipt callback binding invalid')
    release = receipt['source_identity']['release_identity']
    authority = receipt['source_identity']['evidence_authority']
    if not isinstance(release, dict) or not isinstance(release.get('source'), dict) or not isinstance(authority, dict):
        raise ValueError('receipt source structure invalid')
    expected = {'release_id': release.get('release_id'), 'release_verified': release.get('verified'),
                'source_manifest_sha256': release.get('source', {}).get('manifest_sha256'),
                'source_aggregate_sha256': release.get('source', {}).get('aggregate_sha256'),
                'registry_sha256': receipt['source_identity'].get('registry_sha256'),
                'evidence_lock_sha256': authority.get('evidence_lock_sha256'),
                'evidence_lock_identity_verified': authority.get('evidence_lock_identity_verified')}
    if any(source.get(k) != v for k, v in expected.items()) or receipt.get('deployment_identity') != release:
        raise ValueError('receipt source binding invalid')
    if not isinstance(receipt['result'], dict):
        raise ValueError('stored result must be a JSON object')
    payload = dict(receipt['result'])
    if json.loads(row['response_summary_json']) != payload:
        raise ValueError('command result binding invalid')
    if 'receipt_id' in payload or 'receipt_truth' in payload:
        raise ValueError('stored result cannot override receipt authority')
    payload['receipt_id'] = receipt['receipt_id']
    payload['receipt_truth'] = receipt['truth']
    if kind == 'readback':
        # Only producer-classified envelope metadata, never arbitrary unknown fields.
        if payload.get('hardware_truth_level') != 'hardware_query' or type(payload.get('semantic_query_response_verified')) is not bool:
            raise ValueError('readback metadata invalid')
        if payload.get('include_data') != requested['include_data']:
            raise ValueError('readback request binding invalid')
        if payload.get('semantic_query_response_verified') != receipt['truth']['semantic_query_response_verified']:
            raise ValueError('semantic truth binding invalid')
        if payload.get('callback_session_id') != row['callback_session_id']:
            raise ValueError('result callback binding invalid')
        for metadata in ('callback_session_id', 'semantic_query_response_verified'):
            payload.pop(metadata, None)
        PipetteReadbackResponse.model_validate(payload)
    else:
        if runtime.get('owner') != 'PipetteApplicationPlanner' or runtime.get('mode') != 'plan_only' or payload.get('operation') != requested['operation']:
            raise ValueError('planner owner/request binding invalid')
        plan_operation = requested['operation']
        if plan_operation == 'load_tip':
            expected_inputs = {k: requested[k] for k in ('tip_tray', 'tip_well', 'tip_type', 'tip_location', 'home_z_after')}
        elif plan_operation == 'detect_fluid':
            expected_inputs = {'fluid_class': requested['fluid_class']}
        elif plan_operation.startswith('plunger_'):
            expected_inputs = {'direction': plan_operation.removeprefix('plunger_')}
        else:
            expected_inputs = {}
        # Match the producer's scalar types as well as values; bool/int and
        # integral-float equality must not bind a different stored projection.
        actual_inputs = payload.get('requested_inputs')
        if (not isinstance(actual_inputs, dict) or actual_inputs.keys() != expected_inputs.keys()
                or any(type(actual_inputs[k]) is not type(v) or actual_inputs[k] != v for k, v in expected_inputs.items())):
            raise ValueError('planner transformed request binding invalid')
        PipetteApplicationPlanResponse.model_validate(payload)
    record['result'] = payload
    return finish('resolved', None, record)
