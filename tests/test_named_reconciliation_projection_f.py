"""Focused existing named-Home projection regression witness."""
import json
import os
from pathlib import Path
from fastapi.testclient import TestClient
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_home_recovery import native_routes, stopped_failure, homed_replacement, home_body


def test_named_home_decision_projection(homed_replacement):
    app, provider, primitive, refs, root, leaf, data = homed_replacement
    store = app.state.operator_command_plane.store
    client = TestClient(app)
    response = client.post('/operator/recovery/deck/'+data['command_id']+'/reconcile', json=home_body(provider))
    assert response.status_code == 200, response.text
    decision = dict(store.connection.execute('SELECT * FROM operator_plane_deck_recovery_decisions WHERE command_id=?', (data['command_id'],)).fetchone())
    direct = store.get_command(data['command_id'])
    detail = client.get('/operator/v2/actions/receipts/'+data['command_id']+'?detail=true').json()
    Path(os.environ['BIOXP_WORKFLOW_EXPORT']+'.named-projection.json').write_text(json.dumps({
        'decision': decision, 'direct': direct, 'detail': detail, 'response': response.json()}, indent=2))
    assert detail['deck_movement']['recovery_resolution'] is not None
