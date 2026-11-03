"""Offline pickup effects -> real provider -> canonical SQLite boundary.

Only physical/nested execution leaves are replaced; inventory publication uses
its production provider and store (including supported transition shapes).
"""
from types import SimpleNamespace
import pytest
from tests.test_debloat_core import named  # noqa: F401
from tests.test_wake_setup_debloat import rig  # noqa: F401
from bioxp.pipette.models import PipetteCommandError


def observed(ch, valid=True):
    return {"channel": ch, "tip_loaded": True, "result": {
        "ok": valid, "semantic_ok": valid, "hardware_truth_level": "hardware_query"}}


def sent(ch, complete=True):
    return {"channel": ch, "result": {"ok": True, "completion_verified": complete,
        "delivery_verified": True, "controller_acknowledged": True}}


@pytest.mark.parametrize('failure,remaining,released', [
    ('none', [2], [0, 1, 3]),
    ('home_false', [2], [0, 1, 3]),
    ('home_throw', [2], [0, 1, 3]),
    ('dirty', [2], [0, 1, 3]),
    ('release_receipt', [2], [0, 1, 3]),
    ('query_receipt', [0, 1, 2, 3], []),
    ('release_partial', [1, 2, 3], [0]),
    ('query_partial', [0, 2], []),
    ('lower', [], []),
    ('metadata', [2], [0, 1, 3]),
    ('accounting', [0, 1, 2, 3], [0, 1, 3]),
])
def test_provider_partial_effects_survive_tail_failures(named, monkeypatch, failure, remaining, released):
    _, provider, store, _, frames, _ = named
    provider.publish_tip_tray_transition(tray_id=0, transition='load_all', operation_id='seed',
        command_id='seed', provenance={})
    calls = []
    channels = [SimpleNamespace(_tip_loaded=True) for _ in range(4)]

    def query():
        calls.append('query')
        if failure == 'query_partial':
            raise PipetteCommandError('query interrupted', details={
                'observed_channels': [observed(0), observed(1, False), observed(2)]})
        return {'ok': True, 'channels': [observed(c) for c in range(4)]}

    def eject(drop, *, wait_for_completion):
        assert drop == [0, 1, 3] and wait_for_completion
        calls.append('eject')
        if failure == 'release_partial':
            raise PipetteCommandError('second release unknown', details={
                'sends': [sent(0), sent(1, False)], 'channel': 1})
        return [sent(c) for c in drop]

    def metadata(value, loc):
        calls.append('load_tip_metadata')
        if failure == 'metadata':
            raise RuntimeError('metadata publication failed')
        transport._tip_type = value
        return {'ok': True}

    transport = SimpleNamespace(_transports=channels, query_tip_status_all=query,
        _eject_tip_channels_once=eject, loadTip=metadata, _tip_type=201)
    monkeypatch.setattr(provider.primitives, 'pipette_transport', transport, raising=False)
    monkeypatch.setattr(provider, 'tip_tray_type', lambda tray: 'T50')

    def runner(name, call, *args):
        calls.append(name)
        if name == 'load_tip_metadata' and failure == 'metadata':
            raise RuntimeError('metadata publication failed')
        value = call(transport)
        if (name == 'release_unneeded_tips' and failure == 'release_receipt') or (
                name == 'query_tip_status_all' and failure == 'query_receipt'):
            raise RuntimeError('receipt publication failed after physical return')
        return value

    def nested(*, plan, **kwargs):
        # Keep the real compiler in wp8_manual_pipette_physical.
        name = plan['operation']
        calls.append(name)
        if name == 'pipette_home' and failure.startswith('home'):
            if failure == 'home_throw':
                raise RuntimeError('home interrupted')
            return {'ok': False}
        if name == 'pipette_tip_state' and failure == 'dirty':
            raise RuntimeError('semantic publication failed')
        return {'ok': True}

    def lower(**kwargs):
        calls.append('lower')
        if failure == 'lower':
            return {'ok': False}
        return {'ok': True, 'target_position_steps': 141531}

    monkeypatch.setattr(provider, '_manual_pipette_receipt_runner', runner, raising=False)
    monkeypatch.setattr(provider, '_wp8_execution_fence_checker', lambda *a, **k: None, raising=False)
    monkeypatch.setattr(provider, '_wp8_execute_nested_plan', nested)
    monkeypatch.setattr(provider.primitives, 'z_pipette_position', lower)
    if failure == 'accounting':
        publisher = provider._tip_tray_state_publisher
        def fail_restore(**kwargs):
            if kwargs['transition'] == 'retip':
                raise RuntimeError('inventory publication unavailable')
            return publisher(**kwargs)
        monkeypatch.setattr(provider, '_tip_tray_state_publisher', fail_restore)
    frames.clear()
    result = provider.wp8_manual_pipette_physical('sourceManualLoadTip',
        {'tray': 1, 'well': 'A10', 'keep_channels': [2], 'lift_z': True},
        command_id='partial-pickup', child_order=0, plan_digest='test',
        owner_identity={'source_identity': 'partial-pickup:0'})
    assert result['ok'] is (failure in ('none', 'accounting', 'metadata')), result
    assert transport._tip_type == (201 if failure in ('lower', 'metadata') else 50)
    if failure == 'metadata':
        assert 'bookkeeping_error' in result['pickup_accounting'][0]['result']
    wells = [9, 33, 57, 81]
    state = store.tip_tray_state(0)
    assert [ch for ch, well in enumerate(wells) if not state['occupancy'][well]] == remaining, result
    assert sum(state['occupancy']) == 96 - len(remaining)
    if failure != 'lower':
        assert result['released_channels'] == released
        errors = [r for r in result['pickup_accounting'] if 'error' in r['result']]
        assert bool(errors) is (failure == 'accounting'), result
        if failure == 'release_partial':
            assert result['release_unverified_channels'] == [1, 3]
        elif released:
            assert result['release_unverified_channels'] == []
    if 'eject' in calls:
        assert calls.index('lower') < calls.index('query') < calls.index('pipette_move_z') < calls.index('eject')
    assert calls.count('eject') <= 1 and calls.count('lower') == 1
    if failure in ('release_partial', 'release_receipt', 'query_receipt', 'query_partial', 'lower'):
        assert 'pipette_home' not in calls
    assert frames == []
    # Independent canonical read, not just the returned effect dictionary.
    from bioxp.operator_command_plane import OperatorCommandStore
    reopened = OperatorCommandStore(store.root)
    try:
        assert reopened.tip_tray_state(0)['occupancy'] == state['occupancy']
    finally:
        reopened.stop()


@pytest.mark.parametrize('channels', [[0, 2], [0, 1, 3]])
def test_two_and_three_channel_bookkeeping_uses_supported_store_transitions(named, channels):
    _, provider, store, _, frames, _ = named
    provider.publish_tip_tray_transition(tray_id=0, transition='load_all', operation_id='seed',
        command_id='seed', provenance={})
    calls = []
    def finite(name, inputs):
        calls.append(inputs)
        return provider.publish_tip_tray_transition(**inputs, operation_id=f'partial:{len(calls)}',
            command_id='partial', provenance={})
    frames.clear()
    result = provider._manual_pickup_bookkeeping({'tray': 1, 'well': 'A10'}, finite,
        None, channels=channels)
    assert result['rack_tips_removed'] == [9 + 24 * ch for ch in channels]
    assert len(calls) == len(channels)
    assert all(len(call['well_ids']) == 1 for call in calls)
    assert sum(store.tip_tray_state(0)['occupancy']) == 96 - len(channels)
    assert frames == []

# Actual handler/service/receipt/store integration: only hardware leaves replaced.
import socket
_SOCKET = socket.socket
from tests.deck_scoped_rig import retained_rig  # noqa: F401
from tests.test_cover_carry_release_connected import connected  # noqa: F401


@pytest.mark.parametrize('reopen', [False, True])
@pytest.mark.parametrize('fail_home', [False, True])
def test_pickup_publication_inside_active_receipt(connected, monkeypatch, reopen, fail_home):
    import json
    from bioxp.manual_pipetting import manual_physical_plan
    from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport
    from bioxp.pipette.receipts import PipetteReceiptStore
    from bioxp.services.pipette_service import run_pipette_operation
    p, n, store = connected.provider, connected.native, connected.store
    fault = 'home_false' if fail_home else None
    def local_socket(family=socket.AF_INET, *args, **kwargs):
        assert family == socket.AF_UNIX
        return _SOCKET(family, *args, **kwargs)
    monkeypatch.setattr(socket, 'socket', local_socket)
    original_profile = n._motion_oem_axis_profile
    n._motion_oem_axis_profile = lambda axis, startup=False: original_profile(axis, startup=startup)
    n.motor_oem_require_no_motion_profile = lambda *a, **kw: {'ok': True}
    p.primitives._z_profile_overrides = {}
    homes = []
    def home(**kwargs):
        homes.append(kwargs)
        n.positions[4, 1] = 0
        if len(homes) == 1 and fault == 'lease_expired':
            assert store._renew_owner(lease_seconds=-1)
        if len(homes) == 1 and fault == 'home_throw':
            raise RuntimeError('synthetic native home exception after Z=0')
        return {'ok': not (len(homes) == 1 and fault == 'home_false'), 'source_return_code': 173}
    n.motor_oem_move_z_home = home
    class Driver:
        def query_tip_status(self):
            return {'ok': True, 'semantic_ok': True, 'hardware_truth_level': 'hardware_query',
                    'tip_loaded': True, 'source_return': 1, 'reader_generation': 1}
        def pipette_eject_tip(self, **kwargs):
            return {'ok': True, 'delivery_verified': True, 'controller_acknowledged': True,
                    'completion_verified': True, 'outcome': 'completed'}
    channels = [CanPipetteTransport(driver_factory=Driver, pipette_id=i) for i in range(4)]
    for channel in channels:
        channel._initialized = True
    transport = FourPipetteTransport(channels, sleep=lambda _: None)
    p.primitives.pipette_transport = transport
    assert transport._tip_type == 201
    p.record_tip_tray_type(tray_id=0, tip_type=50)
    p.bind_pipette_collection_state_reader(lambda: {'tip_exists': transport.TipExist})
    p.bind_tip_tray_state_reader(store.tip_tray_state)
    p.bind_tip_tray_state_publisher(store.publish_tip_tray_transition)
    store.publish_tip_tray_transition(tray_id=0, transition='construct', operation_id='rca-rack',
        command_id='rca-rack', provenance={'source':'offline fixture'}, **p.deck_owner_authority_stamps())
    import bioxp.pipette.receipts as receipt_module
    monkeypatch.setattr(receipt_module, 'current_release_identity', lambda: {'verified': True,
        'release_id':'offline-rca', 'source': {'manifest_sha256':'1'*64,'aggregate_sha256':'2'*64}})
    monkeypatch.setattr(receipt_module, 'current_authority_identity', lambda: {
        'evidence_lock_identity_verified':True,'evidence_lock_sha256':'3'*64})
    monkeypatch.setattr(receipt_module, 'current_registry_sha256', lambda:'4'*64)
    receipts = PipetteReceiptStore(connected.root)
    from contextlib import nullcontext
    from types import SimpleNamespace
    from bioxp.manual_pipetting import bind_manual_physical_handler
    from bioxp.oem_deck_movement import make_wp8_operation_executor
    stamps = p.deck_owner_authority_stamps()
    store.bind_deck_owner_authority_reader(lambda: stamps, scope=nullcontext)
    store.bind_workflow_dispatcher(lambda command: None)
    store.admit_workflow(command_id='rca-parent', idempotency_key='rca-parent', plan_fingerprint='fixture',
        requested_inputs={'bundle':{}}, ownership_generation=stamps['ownership_generation'],
        resources=('axis:x','axis:y','axis:z'), board_epochs={})
    assert store.claim_next()['command_id'] == 'rca-parent'
    assert store._renew_owner(lease_seconds=600)
    execute_wp8 = make_wp8_operation_executor(provider_getter=lambda:p, command_store=store)
    admission = {'ownership_generation':stamps['ownership_generation'], 'serial206_initialization_provider':{
        'x_authority':{'current_board_lifecycle_generation':stamps['board_epoch_5']},
        'board4_authority':{'active_board_epoch':stamps['board_epoch_4']}}}
    results = []
    raw = p.wp8_manual_pipette_physical
    def observe(*args, **kwargs):
        result = raw(*args, **kwargs)
        results.append(result)
        return result
    monkeypatch.setattr(p, 'wp8_manual_pipette_physical', observe)
    def execute(plan, action, state):
        # The actual immutable execution plan survives JSON persistence before invoke.
        plan = json.loads(json.dumps(plan))
        admitted = store.admit_internal_wp8_operation(plan['operation'], inputs={}, state=admission,
            idempotency_key=action.action_id, prepared_plan=plan)
        cid = admitted['command_id']
        claimed = store.claim_next()
        assert claimed['command_id'] == cid
        persisted = store.connection.execute('select requested_inputs_json from operator_commands where command_id=?', (cid,)).fetchone()
        execution_plan = json.loads(persisted[0])['prepared_plan']
        assert execution_plan == plan
        response = execute_wp8(command_id=cid, plan=execution_plan)
        store.finish(cid, status='completed', payload={'response':response}, claimed=claimed)
        return response
    handler = bind_manual_physical_handler(command_store=store, execute_plan=execute,
        require_motion_ready=lambda:None, provider_getter=lambda:p, receipt_store_getter=lambda:receipts)
    publications = []
    def publish(**kwargs):
        pending = receipts.connection.execute(
            "select operation,status from pipette_operations where status='pending'").fetchall()
        publications.append([(r['operation'], r['status']) for r in pending])
        return store.publish_tip_tray_transition(**kwargs)
    p.bind_tip_tray_state_publisher(publish)
    if reopen:
        import bioxp.oem_serial206_initialization as mod
        old = p
        p = mod.Serial206OemInitializationProvider(old.primitives, state_store=old.state_store,
            reference_store=old.reference_store, generation_provider=old.generation_provider)
        p.bind_deck_semantic_state_reader(store.deck_semantic_state)
        p.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
        p.bind_tip_tray_state_reader(store.tip_tray_state)
        p.bind_tip_tray_state_publisher(publish)
        p.bind_pipette_collection_state_reader(lambda: {'tip_exists':transport.TipExist})
        raw = p.wp8_manual_pipette_physical
        monkeypatch.setattr(p, 'wp8_manual_pipette_physical', observe)
    action = SimpleNamespace(action_id='connected-pickup', params={'operation':'load_tip',
        'tray':1, 'well':'A10', 'overpress':False, 'lift_z':True,
        'keep_channels':[2], 'release_lift_mm':10.0})
    if fail_home:
        from bioxp.oem_deck_movement import DeckExecutionFailure
        with pytest.raises(DeckExecutionFailure):
            handler(action, SimpleNamespace(job_id='rca-parent'))
    else:
        assert handler(action, SimpleNamespace(job_id='rca-parent'))['ok'] is True
    result = results[-1]
    assert result['ok'] is (not fail_home)
    assert result['picked_up_channels'] == [0,1,2,3]
    assert result['released_channels'] == [0,1,3]
    assert all('error' not in row['result'] and 'bookkeeping_error' not in row['result']
        for row in result['pickup_accounting']), result['pickup_accounting']
    assert transport._tip_type == 50
    assert transport.TipExist
    assert publications and all(publications), publications
    assert publications[0] == [('query_all_pipette_tip_states', 'pending')]
    assert all(rows == [('release_unneeded_tips', 'pending')] for rows in publications[1:])
    from bioxp.operator_command_plane import OperatorCommandStore
    cold = OperatorCommandStore(connected.root)
    try:
        occupancy = cold.tip_tray_state(0)['occupancy']
        assert [occupancy[w] for w in [9,33,57,81]] == [True,True,False,True]
        assert sum(occupancy) == 95
    finally:
        cold.stop()
    assert len(homes) == 1

