"""Native closeout: real finite/provider/CAN/store/API, physical leaves only."""
import json
import os
from pathlib import Path
import pytest
from bioxp import api
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.pipette.cavro_liquid import compile_liquid_recipe, Recipe
from bioxp.manual_pipetting import ManualPipettingRequest
from tests.test_cavro_application import rig as wire_rig
from tests.test_cavro_liquid import recipe
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_method_runtime_connected import mount, submit, action, control
from tests.test_protocol_workflow_connected import await_job


@pytest.mark.parametrize('failure', [None, 'device'])
def test_recipe_real_finite_native_dispatch(wire_rig, installed_retained, monkeypatch, tmp_path, failure):
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    provider = installed_retained[1]
    provider.primitives.pipette_transport = wire_rig.group
    receipts = PipetteReceiptStore(installed_retained[4])
    monkeypatch.setattr(api, '_pipette_receipts', receipts)
    monkeypatch.setattr(api, '_require_motion_route_ready', lambda: None)
    raw = recipe()
    raw['channels'] = [0, 1]
    if failure:
        wire_rig.fault[failure] = True
    actions = [action('pipette_manual_physical', {'operation': 'cavro_liquid_recipe', 'recipe': raw},
                      on_error='pause_for_operator' if failure else 'stop'), action('note')]
    job = submit(app, client, actions, key='recipe-' + str(failure))
    held = None
    if failure:
        held = await_job(client, job, lambda r: r['execution']['runtime_state']['workflow']['gate'] == 'error_hold')
        before = list(wire_rig.wire)
        generation = int(provider.generation_provider())
        assert control(client, job, generation, action='continue', gate='ordinary_pause', gate_id='pipette_manual_physical').status_code >= 400
        assert control(client, job, generation, action='abort').status_code in (200, 202)
    done = await_job(client, job, lambda r: r['command']['terminal'])
    rows = done['execution']['runtime_state']['action_results']
    result = rows[0]['pipette_result']
    assert result['requested'] == compile_liquid_recipe(raw)['application']
    assert result['liquid_settings']['resolved_recipe'] == raw
    if failure:
        assert not result['ok'] and len(rows) == 1
        assert wire_rig.wire == before
    else:
        assert done['command']['status'] == 'completed', rows
        assert (0, 'P22.75,1R') in wire_rig.wire
        assert (0, 'D12.75,1R') in wire_rig.wire and (0, 'D10,1R') in wire_rig.wire
        assert wire_rig.wire.count((0, 'A0R')) == 1
    finite_ids = [x for x in done['execution']['runtime_state']['workflow']['child_command_ids'] if not x.startswith('pipette_')]
    assert len(finite_ids) == 1
    finite = app.state.operator_command_plane.store.wp8_operation_evidence(finite_ids[0])
    assert len(finite['children']) == 1 and finite['children'][0]['operation'] == 'sourceCavroApplication'
    assert receipts.read(limit=100)
    assert client.get('/protocol/jobs/' + job).json()['execution']['runtime_state'] == done['execution']['runtime_state']
    if root := os.environ.get('CAVRO_EVIDENCE_ROOT'):
        target = Path(root, 'native-close-exports')
        target.mkdir(exist_ok=True)
        (target / ('recipe-' + str(failure) + '.json')).write_text(json.dumps({'result': done, 'held': held, 'actions': actions, 'wire': wire_rig.wire}, indent=2))


def test_rgb_and_source_ss_real_dispatch(installed_retained, monkeypatch, tmp_path):
    from bioxp.usb_driver import BioXpTester
    app, client = mount(installed_retained, monkeypatch, tmp_path)
    tester = object.__new__(BioXpTester)
    writes = []
    def send(*args, **kwargs):
        writes.append((args, kwargs))
        return {'status': 100}
    tester.send_tmcl_retry = send
    monkeypatch.setattr(api, '_get_tester', lambda: tester)
    job = submit(app, client, [action('led', {'red': 12, 'green': 34, 'blue': 56}), action('seal_separate')], key='rgb-ss')
    done = await_job(client, job, lambda r: r['command']['terminal'])
    assert done['command']['status'] == 'completed', done
    rows = done['execution']['runtime_state']['action_results']
    assert rows[0]['rgb'] == [12, 34, 56]
    assert len(writes) == 3 and [w[0][3] for w in writes] == [0, 1, 2]
    assert rows[1]['source_noop'] and rows[1]['seal_separation_performed'] is None
    assert rows[1]['motion_commanded'] is False


def test_recipe_authoring_and_frozen_native_schema():
    from bioxp.manual_pipetting import compile_manual_pipetting, manual_physical_plan
    raw = recipe()
    doc = compile_manual_pipetting({'protocol_id': 'recipe', 'steps': [{'operation': 'cavro_liquid_recipe', 'recipe': raw}]})
    plan = manual_physical_plan(doc.stages[0].actions[0].params)
    assert len(plan['children']) == 1
    assert plan['children'][0]['operation'] == 'sourceCavroApplication'
    schema = ManualPipettingRequest.model_json_schema()
    assert 'ManualCavroRecipe' in schema['$defs']
    if root := os.environ.get('CAVRO_EVIDENCE_ROOT'):
        from bioxp.protocols.method_contract import method_contract
        Path(root, 'native-close-contract.json').write_text(json.dumps(method_contract(), indent=2))
        Path(root, 'native-close-recipe-schema.json').write_text(json.dumps(Recipe.model_json_schema(), indent=2))
        Path(root, 'native-close-manual-schema.json').write_text(json.dumps(schema, indent=2))
