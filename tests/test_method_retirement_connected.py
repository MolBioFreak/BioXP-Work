"""Mounted ordinary XY action survives retirement; controller leaves only."""
import time
import pytest
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_near_terminal import NearUSB
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter
from bioxp.serial206_y_provider import Serial206YProvider


@pytest.fixture(autouse=True)
def mount_xy_routes(monkeypatch):
    from bioxp import api, operator_controls
    original = operator_controls.install_operator_control_plane
    def install(app, **kwargs):
        app.add_api_route('/motion/oem/move_xy', api.motion_oem_move_xy, methods=['POST'])
        app.add_api_route('/motion/oem/home_xy', api.motion_oem_home_xy, methods=['POST'])
        return original(app, **kwargs)
    monkeypatch.setattr(operator_controls, 'install_operator_control_plane', install)


def test_removed_routes_have_no_schema_or_dispatch(installed_retained):
    app, provider, primitive, references, root = installed_retained
    before = list(primitive.calls)
    client = TestClient(app)
    for url in ('/operator/methods','/operator/v2/methods'):
        assert client.post(url,json={}).status_code == 404
        assert client.get(url+'/unknown').status_code == 404
    for suffix in ('commands','pause','resume','cancel'):
        response = (client.get if suffix == 'commands' else client.post)('/operator/methods/unknown/'+suffix)
        assert response.status_code == 404
    schema = app.openapi()
    assert not any('/methods' in path for path in schema['paths'])
    assert not any('MethodRequest' in name or 'MethodStep' in name for name in schema['components']['schemas'])
    assert primitive.calls == before


def test_xy_action_real_provider_receipt_and_history(installed_retained, monkeypatch):
    app, provider, primitive, references, root = installed_retained
    qualify_test_references(references)
    # Presence-only lifecycle binding: an ordinary XY action must not initialize.
    monkeypatch.setattr(provider.preparation_provider, 'prepare_for_initialize_motors',
        lambda **kw: pytest.fail('XY action attempted initialization'), raising=False)
    leaf = NearUSB((25029,71755))
    adapter = Serial206ProductionPrimitiveAdapter(leaf,None,authority_provider=lambda:{},
        generation_provider=provider.generation_provider,reference_store=references)
    adapter.y_provider = Serial206YProvider(leaf,state_store=provider.state_store,
        generation_provider=provider.generation_provider,reference_store=references)
    monkeypatch.setattr(primitive,'move_xy',adapter.move_xy,raising=False)
    with TestClient(app) as client:
        body = {'schema_version':'bioxp.operator_action_request.v2',
            'idempotency_key':'retirement-xy-action','expected_ownership_generation':provider.generation_provider(),
            'expected_board_epoch_by_board':{},'inputs':{'x':60571,'y':71755}}
        admitted = client.post('/operator/v2/actions/oem.xy.move_absolute',json=body)
        assert admitted.status_code == 200, admitted.text
        command_id = admitted.json()['command_id']
        deadline = time.monotonic()+10
        while True:
            response = client.get('/operator/v2/actions/receipts/'+command_id+'?detail=true')
            assert response.status_code == 200, response.text
            receipt = response.json()
            if receipt['status'] not in {'queued','running','dispatched','issued_pending'} or time.monotonic()>deadline:
                break
            time.sleep(.01)
        assert receipt['status'] == 'completed', receipt
        assert leaf.moves
        before = list(leaf.moves)
        replay = client.post('/operator/v2/actions/oem.xy.move_absolute',json=body)
        assert replay.status_code == 200 and replay.json()['command_id'] == command_id
        assert leaf.moves == before
        db = app.state.operator_command_plane.store.connection
        row = db.execute('SELECT action_id,receipt_json FROM operator_commands WHERE command_id=?',(command_id,)).fetchone()
        assert row['action_id'] == 'oem.xy.move_absolute'
        assert db.execute('SELECT 1 FROM runtime_retired_records WHERE record_key=json_array(?)',(command_id,)).fetchone() is None
        import json, os
        from pathlib import Path
        if os.environ.get('RETIREMENT_XY_EVIDENCE'):
            Path(os.environ['RETIREMENT_XY_EVIDENCE']).write_text(json.dumps(
                {'receipt':receipt,'moves':leaf.moves,'admission':admitted.json()}, sort_keys=True, default=str))
