"""Offline producer -> real route/finalizer -> SQLite -> reopened receipt."""
import asyncio
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from bioxp import operator_controls as controls
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
from bioxp.operator_receipt_store import OperatorReceiptStore
from tests.test_z_absolute_wire_contract import make_adapter


@pytest.mark.parametrize('arrives', [True, False])
@pytest.mark.parametrize('minimum', [500, 65000])
@pytest.mark.parametrize('requested', [0, 90000])
@pytest.mark.parametrize('start', [0, 100000])
def test_source_target_survives_route_store_and_reopen(tmp_path, monkeypatch, minimum, requested, start, arrives):
    monkeypatch.setenv("BIOXP_OEM_RUNTIME_ROOT", str(tmp_path))
    import bioxp.api as api
    adapter, frames = make_adapter(monkeypatch, start, arrives=arrives)
    # Populate a real producer response beyond its old diagnostic item budget.
    original_move = adapter.z_move_absolute
    def populated_move(**kwargs):
        result = original_move(**kwargs)
        return {'noncritical': list(range(5000)), **result}
    adapter.z_move_absolute = populated_move
    store = OEMRuntimeStore(tmp_path)
    from bioxp.services.reference_service import ReferenceStateStore
    provider = Serial206OemInitializationProvider(adapter, state_store=store,
        reference_store=ReferenceStateStore(tmp_path / 'bioxp_runtime.db'),
        generation_provider=lambda: 7,
        preparation_provider=SimpleNamespace(current_board_lifecycle_generation=lambda: 1))
    state = provider._load_state()
    state['z_lifecycle'].update(state='referenced_ready', reference_state='referenced', generation=7, board_lifecycle_generation=1)
    state['machine_status']['psudo_z_home_steps'] = minimum
    provider._save_state(state)
    monkeypatch.setattr(api, '_require_serial206_oem_initialization_provider', lambda *args: provider)
    # Keep the real route and dispatch context; substitute only worker scheduling.
    async def run_blocking(label, fn, *, timeout_s):
        return fn()
    monkeypatch.setattr(api, '_run_blocking', run_blocking)
    monkeypatch.setattr(controls, 'current_release_identity', lambda: {'verified': True, 'release_id': 'offline', 'source': {'manifest_sha256': '1'*64, 'aggregate_sha256': '2'*64}})
    monkeypatch.setattr(controls, 'current_authority_identity', lambda: {'evidence_lock_identity_verified': True, 'evidence_lock_sha256': '3'*64})
    monkeypatch.setattr(controls, 'current_registry_sha256', lambda: '4'*64)
    class Hardware:
        ownership_epoch = 7
        def ownership_projection(self):
            return {'ownership_epoch': 7, 'ownership': {'transport': 'owned', 'usb': 'service', 'router': 'running', 'CAN_READY': True}}
        def project(self, *domains, **kwargs):
            observations = {'power': {'safety_valid': True}, 'latch': {'door_sensor': 1, 'latch_sensor': 1}, 'interlock': {'motion_arm': {'armed': True}}}
            return {'snapshot_id': 'offline', 'freshness': {'state': 'fresh', 'age_s': 0.0, 'fresh_for_s': 30.0},
                'domains': {d: {'status': 'observed', 'observation': observations.get(d, {})} for d in domains}}
    monkeypatch.setattr(controls, 'hardware_state', Hardware())
    app = FastAPI()
    app.add_api_route('/motion/oem/manual/absolute', api.motion_oem_manual_absolute, methods=['POST'])
    controls.install_operator_control_plane(app,
        maintenance_state_provider=lambda: {'motion_blocked': False, 'recovery_required': False},
        reference_state_provider=lambda: {'rows': {'z': {'state': 'referenced'}}},
        lifecycle_state_provider=lambda: {'operation_state': 'stopped', 'door': {'door_closed': True, 'latch_closed': True}},
        serial206_initialization_state_provider=lambda: {'bound': True, 'initialize_motors_live_available': True, 'z_authority': provider.z_projection()})
    with TestClient(app) as client:
        response = client.post('/operator/v2/actions/oem.z.move_absolute', json={
            'schema_version': 'bioxp.operator_action_request.v2',
            'expected_ownership_generation': 7, 'expected_board_epoch_by_board': {},
            'idempotency_key': f'z-offline-{minimum}-{requested}-{start}', 'inputs': {'position_steps': requested}})
        assert response.status_code == 200, response.text
        command_id = response.json()['command_id']
        for _ in range(100):
            terminal = client.get(f'/operator/v2/actions/receipts/{command_id}').json()
            if terminal['terminal']:
                break
            time.sleep(0.01)
        assert terminal['status'] == ('completed' if arrives else 'failed'), terminal
        reader = OperatorReceiptStore(tmp_path)
        row = reader.by_command(command_id, include_evidence=True)
        for detail in (False, True):
            reopened = client.get(f'/operator/v2/actions/receipts/{command_id}', params={'detail': detail})
            assert reopened.status_code == 200, reopened.text
            evidence = reopened.json()['z_move']
            assert evidence is not None, json.dumps(row)
            assert evidence['requested_position_steps'] == requested
            assert evidence['effective_position_steps'] == max(minimum, requested)
            assert evidence['before_position_steps'] == start
            assert evidence['after_position_steps'] == (None if arrives else start)
            assert evidence['controller_command_acknowledged'] is True
            assert evidence['controller_terminal_state_verified'] is False
            assert evidence['physical_effect_verified'] is False
            assert evidence['coordinate_mode'] == 'absolute'
            assert evidence['target_clamped'] is (requested < minimum)
            assert evidence['source_return_ok'] is arrives
            if arrives:
                assert evidence['terminal_z_state']['position_steps'] == max(minimum, requested)
                assert evidence['terminal_z_state']['speed_steps_s'] == 0
            else:
                assert evidence['terminal_z_state'] is None  # no new read on source failure
            persisted_source = store.read_serial206_receipt('z', row['authority_receipt_id'])
            assert persisted_source['critical_evidence'] == evidence
            if detail:
                assert reopened.json()['requested_values'] == {'position_steps': requested}
                assert reopened.json()['effective_values']['effective_position_steps'] == max(minimum, requested)
        history = client.get('/operator/actions/history').json()
        assert next(r for r in history['items'] if r['command_id'] == command_id)['z_move'] == evidence
        code = """
import json,sys
from fastapi import FastAPI
from fastapi.testclient import TestClient
from bioxp.operator_controls import install_operator_control_plane
from bioxp.operator_receipt_store import OperatorReceiptStore
app=FastAPI()
install_operator_control_plane(app)
with TestClient(app) as client:
    compact=client.get('/operator/v2/actions/receipts/'+sys.argv[2],params={'detail':False})
    detail=client.get('/operator/v2/actions/receipts/'+sys.argv[2],params={'detail':True})
    assert compact.status_code == detail.status_code == 200
    print(json.dumps({'compact':compact.json(),'detail':detail.json(),
        'raw':OperatorReceiptStore(sys.argv[1]).by_command(sys.argv[2],include_evidence=True)}))
"""
        fresh = json.loads(subprocess.check_output([sys.executable, '-c', code, str(tmp_path), command_id], text=True))
        assert fresh['raw']['z_move'] == fresh['compact']['z_move'] == fresh['detail']['z_move'] == evidence
        assert len(frames) == 1
        # Existing cache plus per-request preview must not reuse another draft.
        for target in (0, 90000, 0):
            response = client.get('/operator/control-catalog', params={'z_target_steps': target})
            if response.status_code == 503:
                response = client.get('/operator/control-catalog', params={'z_target_steps': target})
            assert response.status_code == 200, response.text
            context = response.json()['dashboard']['z_axis']['provider']
            assert context['current_minimum_steps'] == minimum
            assert context['target_preview'] == {'requested_position_steps': target, 'effective_position_steps': max(minimum, target)}
        output = os.environ.get('Z_TARGET_OUTPUT')
        if output:
            with Path(output).open('a') as stream:
                stream.write(json.dumps({'minimum': minimum, 'requested': requested, 'start': start, 'arrives': arrives,
                    'catalog': client.get('/operator/control-catalog', params={'z_target_steps': requested}).json(),
                    'compact': fresh['compact'],
                    'detail': fresh['detail'],
                    'legacy': row}) + '\n')
    store.close()
