"""Lossless operational projections, separate from bounded diagnostic trees."""
import json
from collections.abc import Mapping

OUTCOME_FIELDS = frozenset({
    'ok', 'source_return', 'source_return_completed', 'source_call_completed',
    'source_noop', 'controller_command_required', 'controller_command_acknowledged',
    'controller_completion_verified', 'controller_terminal_state_verified',
    'hardware_postcondition_verified', 'physical_effect_verified',
    'delivery_attempted', 'interrupted', 'interrupted_by_terminate',
    'error', 'exception', 'exception_type', 'failure', 'source_identity',
    'command_id', 'receipt_id', 'operation_index', 'requested_control', 'partial_effects',
    'source_anchor', 'source_return_code', 'reason',
    'failed_channel', 'failure_details', 'completion_wait_started', 'source_timeout_termination',
})


def finite_child_outcomes(command_id, evidence):
    rows = []
    for child in evidence.get('children', ()):
        terminal = json.loads(child.get('terminal_evidence_json') or '{}')
        leaf = terminal.get('result') or {}
        rows.append({'command_id': command_id, 'child_order': child['child_order'],
                     'operation': child['operation'], 'status': child['terminal_state'],
                     **compact_operational(leaf)})
    return rows


def application_outcome(value):
    """Preserve per-channel meaning and identity; omit repeated wire captures.

    Canonical pipette receipts retain the transport trees. No width/depth count
    is used here: every application event and selected channel stays present.
    """
    fields = OUTCOME_FIELDS | frozenset({
        'kind', 'operation', 'status', 'channels', 'channel', 'result', 'completion',
        'outcome', 'completion_verified', 'controller_acknowledged', 'delivery_verified',
        'semantic_query_response_verified', 'oem_error_code', 'event_error_code',
        'generation_changed', 'owner_generation', 'completion_owner_token', 'owner_token',
        'transaction_id', 'timeout_ms', 'state_reconciled', 'state_reconciliation_source',
        'volume_ul', 'liquid_level_ul', 'front_air_level_ul', 'rear_air_level_ul',
        'position_steps', 'source_occurrence_id', 'fluid_timestamp', 'pipette_message_state',
        'failed_channel', 'failure_details', 'completion_wait_started', 'source_timeout_termination',
        'wait_mode', 'report', 'report_kind', 'units', 'value', 'raw_ascii', 'raw_bytes', 'parsed',
        'command_effects', 'program', 'buffer_action', 'effects_deferred', 'legacy_counter_basis',
        'scientific_pressure', 'stream_session_id', 'chunk_ids', 'observation_error',
        'baseline_statistics', 'count', 'mean', 'sample_standard_deviation', 'min', 'max',
        'retained_frame_count', 'lost_frame_count', 'malformed_frame_count', 'lost_sample_count',
        'controller_timestamps_available', 'sample_period_basis', 'analysis_basis',
        'anomaly_model', 'classification', 'started_at', 'finished_at', 'reader_generation',
        'persistent', 'warnings', 'requested', 'accounting', 'target_liquid_ul',
        'pneumatic_displacement_delta_ul', 'intended_liquid_delta_ul',
        'initial_fill_known', 'measured_liquid_ul', 'known_pneumatic_delta_ul',
        'known_intended_liquid_delta_ul', 'unknown_liquid_effect', 'unknown_pneumatic_effect', 'basis',
        'target_steps', 'reference_steps', 'speed_native', 'physical_surface_verified',
        'shared_head', 'shared_z', 'inactive_mounted_tips_move', 'independent_channel_z',
        'unequal_authored_surface_references', 'geometry_limit', 'start_mode',
        'commanded_displacement_ul', 'liquid_phase', 'terminal_state',
        'source_return_completed', 'pipette_partial_result', 'message', 'failure_code',
    })
    if isinstance(value, Mapping):
        return {k: (v if k in {"failure_details", "source_timeout_termination", "requested", "warnings", "command_effects", "program", "observation_error"}
                    else application_outcome(v)) for k, v in value.items() if k in fields}
    if isinstance(value, (list, tuple)):
        return [application_outcome(row) for row in value]
    return value


def compact_application(value):
    result = dict(value)
    result['events'] = []
    for event in value.get('events', ()):
        row = {k: v for k, v in event.items() if k not in {'result', 'partial'}}
        for field in ('result', 'partial'):
            if field in event:
                row[field] = application_outcome(event[field])
        result['events'].append(row)
    if 'plld' in result:
        result['plld'] = {**result['plld'], 'channels': application_outcome(result['plld'].get('channels', []))}
    return result


def compact_operational(value):
    """Keep source child order/results, not recursive controller diagnostics."""
    if not isinstance(value, Mapping):
        return value
    result = {key: item for key, item in value.items() if key in OUTCOME_FIELDS}
    for key in ('source_children', 'completed_children', 'provider_results'):
        children = value.get(key)
        if isinstance(children, list):
            result[key] = [{
                **{k: v for k, v in child.items() if k in {
                    'order', 'child_order', 'operation', 'child_id', 'source_anchor', 'inputs', 'arguments', 'status'}},
                'result': compact_operational(child.get('result')),
            } for child in children if isinstance(child, Mapping)]
    return result


def deck_child_outcomes(command):
    deck = command.get('deck_movement') or {}
    return [{
        'command_id': command['command_id'],
        'child_order': child['order'],
        'operation': child['operation'],
        'source_anchor': child.get('source_anchor'),
        'status': child.get('terminal_state'),
        'result': compact_operational(child.get('terminal_evidence') or {}),
        'provider_result': (child.get('terminal_evidence') or {}).get('operational_outcome') or compact_operational((child.get('terminal_evidence') or {}).get('provider_evidence') or {}),
    } for child in deck.get('stages', [])]
