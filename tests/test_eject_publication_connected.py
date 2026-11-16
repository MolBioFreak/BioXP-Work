"""Actual production eject callers, inert pump drivers and canonical stores."""
import asyncio
import socket
from types import SimpleNamespace

import pytest
_SOCKET = socket.socket
from tests.test_debloat_core import named
from tests.test_wake_setup_debloat import rig


@pytest.mark.parametrize('caller', ['workflow', 'diagnostic', 'direct'])
@pytest.mark.parametrize('selected,publication_fails', [([0,1],False), ([0,1,2,3],False), ([0,1,2,3],True)])
def test_eject_callers_publish_all_channel_presence(named, monkeypatch, caller, selected, publication_fails, request):
    from bioxp import api
    from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport
    from bioxp.pipette.receipts import PipetteReceiptStore
    from bioxp.services.pipette_service import run_pipette_operation
    from bioxp.protocols.models import ProtocolActionKind
    from bioxp.operator_command_plane import OperatorCommandStore
    import bioxp.pipette.receipts as receipt_module
    _, provider, store, _, frames, _ = named
    monkeypatch.setattr(socket, 'socket', lambda family=socket.AF_INET, *a, **k:
        _SOCKET(family, *a, **k) if family == socket.AF_UNIX else pytest.fail('network forbidden'))
    monkeypatch.setattr(receipt_module, 'current_release_identity', lambda: {'verified': True,
        'release_id':'offline-eject', 'source': {'manifest_sha256':'1'*64,'aggregate_sha256':'2'*64}})
    monkeypatch.setattr(receipt_module, 'current_authority_identity', lambda: {
        'evidence_lock_identity_verified':True,'evidence_lock_sha256':'3'*64})
    monkeypatch.setattr(receipt_module, 'current_registry_sha256', lambda:'4'*64)
    receipts = PipetteReceiptStore(store.root)
    from bioxp.services.pipette_service import set_direct_pipette_idempotency_key, reset_direct_pipette_idempotency_key
    token = set_direct_pipette_idempotency_key('direct-eject')
    request.addfinalizer(lambda: reset_direct_pipette_idempotency_key(token))
    mounted, sends = [True] * 4, []
    class Driver:
        def __init__(self, ch): self.ch = ch
        def query_tip_status(self):
            return {'ok':True, 'semantic_ok':True, 'hardware_truth_level':'hardware_query',
                'tip_loaded':mounted[self.ch], 'source_return':int(mounted[self.ch]), 'reader_generation':1}
        def pipette_eject_tip(self, **kwargs):
            sends.append(self.ch)
            mounted[self.ch] = False
            return {'ok':True, 'delivery_verified':True, 'controller_acknowledged':True,
                'completion_verified':True, 'outcome':'completed'}
    channels = [CanPipetteTransport(driver_factory=lambda ch=i:Driver(ch), pipette_id=i) for i in range(4)]
    for channel in channels:
        channel._initialized = channel._tip_loaded = True
    transport = FourPipetteTransport(channels, sleep=lambda _:None)
    provider.primitives.pipette_transport = transport
    monkeypatch.setattr(api, '_pipette_transport', transport)
    monkeypatch.setattr(api, '_pipette_receipts', receipts)
    monkeypatch.setattr(api.app.state, 'oem_deck_provider', provider, raising=False)
    provider.publish_pipette_owner_state(tip_loaded=True, tip_dirty=True, tip_location=-1,
        source_command_id='preceding-pickup')
    publications = []
    publisher = provider._deck_semantic_state_publisher
    def publish(**kwargs):
        publications.append(kwargs['source_command_id'])
        if caller != 'diagnostic':
            row = receipts.connection.execute('SELECT status FROM pipette_operations WHERE command_id=?',
                (kwargs['source_command_id'],)).fetchone()
            assert row['status'] == 'reserved'
        if publication_fails:
            raise OSError('injected publication write failure')
        return publisher(**kwargs)
    provider.bind_deck_semantic_state_publisher(publish)
    async def inline(label, operation, *, timeout_s): return operation()
    monkeypatch.setattr(api, '_run_blocking', inline)
    frames.clear()
    if caller == 'workflow':
        from bioxp.protocols.models import ProtocolDocument
        from bioxp.protocols.runtime_state import ProtocolRuntimeState
        from bioxp.services.protocol_service import bind_protocol_dispatcher
        doc = ProtocolDocument.from_payload({'protocol_id':'eject-workflow', 'stages':[
            {'stage_id':'one','actions':[{'stage_id':'one','action_id':'eject',
                'kind':'tip_eject','params':{'channels':selected}}]}]})
        state = ProtocolRuntimeState.from_document(doc, dry_run=False, job_id='eject-parent')
        captured = []
        def handler(action, state):
            result = api._protocol_live_pipette_handler(action, state)
            captured.append(result)
            return result
        bind_protocol_dispatcher(store, binding_factory=lambda *a, **k:({ProtocolActionKind.TIP_EJECT:handler},{},{}),
            artifact_store=SimpleNamespace(save=lambda bundle:None))
        admission = dict(command_id='eject-parent',idempotency_key='eject-parent',plan_fingerprint='eject',
            requested_inputs={'bundle':{'protocol':{'document':doc.to_payload()},
                'execution':{'runtime_state':state.to_payload()},'operator':{'reviews':[]}}},
            ownership_generation=7, resources=(), board_epochs={})
        store.admit_workflow(**admission)
        claimed = store.claim_next()
        assert claimed['command_id'] == 'eject-parent'
        assert store._renew_owner(lease_seconds=600)
        store._dispatch_worker(lambda c:pytest.fail('unexpected leaf dispatcher'), claimed)
        done = store.get_workflow('eject-parent')
        assert done['command']['status'] == 'completed', done
        assert len(captured) == 1
        result = captured[0]
        assert result['command_id'] in done['execution']['runtime_state']['workflow']['child_command_ids']
    elif caller == 'direct':
        result = asyncio.run(api.liquid_eject_all(api.PipetteEjectAllRequest(channels=selected)))
    else:
        from bioxp.pipette.oem_diagnostics import run_diagnostic_inline
        def record(name, call, command_id, identity, inputs):
            return asyncio.run(run_pipette_operation(name, call, get_transport=api._get_pipette_transport,
                run_blocking=inline, receipt_store=receipts, requested_inputs=inputs,
                runtime_binding={"idempotency_key":identity["source_identity"]}))
        monkeypatch.setattr(provider, '_manual_pipette_receipt_runner', record, raising=False)
        monkeypatch.setattr(provider, '_wp8_execution_fence_checker', lambda *a, **k:None, raising=False)
        result = run_diagnostic_inline(provider, {'action':'eject','channels':selected},
            command_id='diagnostic-eject', owner_identity={'source_identity':'diagnostic-eject:0'})
    assert result['ok'] is True, result
    assert result['deck_state_publication']['status'] == ('blocked' if publication_fails else 'published'), result
    if publication_fails:
        assert result['deck_state_publication']['reason'] == 'injected publication write failure'
    assert len(publications) == 1
    if caller != 'diagnostic':
        assert publications == [result['command_id']]
    assert sends == selected
    assert [ch._tip_loaded for ch in channels] == mounted
    expected_loaded = publication_fails or len(selected) != 4
    expected_height = 500 if expected_loaded else 65000
    cold = OperatorCommandStore(store.root)
    try:
        assert cold.deck_semantic_state()['tip_loaded'] is expected_loaded
        assert cold.deck_semantic_state()['pseudo_z_home'] == expected_height
        assert provider._load_state()['machine_status']['psudo_z_home_steps'] == expected_height
        import json
        if caller != 'diagnostic':
            row = cold.connection.execute('SELECT receipt_json FROM pipette_operations WHERE command_id=?',
                (result['command_id'],)).fetchone()
            retained = json.loads(row[0])
            assert retained['result']['deck_state_publication'] == result['deck_state_publication']
            assert retained['result']['after']['channels_with_tips'] == [i for i in range(4) if i not in selected]
    finally:
        cold.stop()
    assert frames == []
    if caller != 'diagnostic':
        # An old successful key must not clear tips from a later pickup, nor
        # republish an old failed observation as a new physical result.
        provider.bind_deck_semantic_state_publisher(publisher)
        provider.publish_pipette_owner_state(tip_loaded=True, tip_dirty=False,
            tip_location=-1, source_command_id='later-pickup')
        revision = store.deck_semantic_state()['semantic_state_revision']
        if caller == 'workflow':
            replay = store.admit_workflow(**admission)
            assert replay['command']['command_id'] == 'eject-parent'
            assert replay['command']['status'] == 'completed'
            assert store.claim_next() is None
        else:
            replay = asyncio.run(api.liquid_eject_all(api.PipetteEjectAllRequest(channels=selected)))
            assert replay['replayed'] is True
        assert store.deck_semantic_state()['tip_loaded'] is True
        assert store.deck_semantic_state()['semantic_state_revision'] == revision
        assert sends == selected
