"""Offline native door -> finite executor -> SQLite -> workflow handoff."""
import threading
from types import SimpleNamespace
import pytest
from tests.test_debloat_core import named
from tests.test_wake_setup_debloat import rig


def prepare(named, monkeypatch, tmp_path, *, fault=None, stale_read=False):
    from bioxp import api
    from bioxp.oem_deck_movement import make_wp8_operation_executor, compile_finite_plate_operation
    from bioxp.protocols.models import ProtocolDocument, ProtocolActionKind
    from bioxp.protocols.runtime_state import ProtocolRuntimeState
    from bioxp.services.protocol_service import bind_protocol_dispatcher
    _, provider, store, _, frames, _ = named
    state = provider._load_state()
    state['machine_status']['thermal_door_open'] = False
    provider._save_state(state)
    document = ProtocolDocument.from_payload({'protocol_id':'offline-door', 'stages':[{
        'stage_id':'one', 'actions':[
            {'stage_id':'one','action_id':'close','kind':'thermal_door','params':{'door_command':'DC'}},
            {'stage_id':'one','action_id':'after','kind':'led','params':{'red':1,'green':2,'blue':3}}]}]})
    runtime = ProtocolRuntimeState.from_document(document, dry_run=False, job_id='door-parent')
    bundle = {'protocol':{'document':document.to_payload()},
              'execution':{'runtime_state':runtime.to_payload()}, 'operator':{'reviews':[]}}
    called, children = [], []
    def after(action, current):
        called.append(action.action_id)
        return {'ok':True}
    bind_protocol_dispatcher(store, binding_factory=lambda *a, **k: (
        {ProtocolActionKind.THERMAL_DOOR:api._protocol_live_thermal_door_handler,
         ProtocolActionKind.LED:after}, {}, {}),
        artifact_store=SimpleNamespace(save=lambda bundle:None))
    store.admit_workflow(command_id='door-parent',idempotency_key='door-parent',plan_fingerprint='door',
        requested_inputs={'bundle':bundle},ownership_generation=7,
        resources=('axis:x','axis:y','axis:z','axis:door'),board_epochs={})
    claimed = store.claim_next()
    assert claimed['command_id']=='door-parent'
    assert store._renew_owner(lease_seconds=600)
    execute = make_wp8_operation_executor(provider_getter=lambda:provider,command_store=store)
    stamps = provider.deck_owner_authority_stamps()
    admission = {'ownership_generation':7,'serial206_initialization_provider':{
        'x_authority':{'current_board_lifecycle_generation':stamps['board_epoch_5']},
        'board4_authority':{'active_board_epoch':stamps['board_epoch_4']}}}
    def admit(operation, *, inputs, idempotency_key):
        plan = compile_finite_plate_operation(operation, source_leaf_available=True,
            **inputs, **provider.wp8_operation_machine_state(operation, inputs))
        child = store.admit_internal_wp8_operation(operation, inputs=inputs,state=admission,
            idempotency_key=idempotency_key,prepared_plan=plan)
        child_claim = store.claim_next()
        assert child_claim['command_id']==child['command_id']
        result = execute(command_id=child['command_id'],plan=plan)
        store.finish(child['command_id'],status='completed',payload=result,claimed=child_claim,
            controller_acknowledged=result.get('controller_command_acknowledged',False), full_response=result)
        assert result['ok'] is True
        children.append(child['command_id'])
        if fault:
            fault(store)
        return child
    monkeypatch.setattr(api.app.state,'operator_command_plane',SimpleNamespace(store=store),raising=False)
    monkeypatch.setattr(api.app.state,'oem_wp8_operation_admitter',admit,raising=False)
    if stale_read:
        original = store.get_command
        first = set()
        def read(cid, *a, **kw):
            row = original(cid,*a,**kw)
            if cid in children and cid not in first:
                first.add(cid)
                return {**row,'status':'dispatched'}
            return row
        monkeypatch.setattr(store,'get_command',read)
    frames.clear()
    return store, claimed, called, children, frames


def run_worker(store, claimed):
    worker = threading.Thread(target=store._dispatch_worker,args=(lambda c:pytest.fail('unexpected ordinary dispatch'),claimed))
    worker.start()
    worker.join(10)
    assert not worker.is_alive(), 'workflow worker failed to return'
    return store.get_workflow(claimed['command_id'])


@pytest.mark.parametrize('stale_read',[False,True])
def test_real_closed_door_handoff(named, monkeypatch, tmp_path, stale_read):
    store, claimed, called, children, frames = prepare(named,monkeypatch,tmp_path,stale_read=stale_read)
    done = run_worker(store,claimed)
    assert done['command']['status']=='completed', done
    assert called==['after'] and len(children)==1
    assert done['execution']['runtime_state']['workflow']['child_command_ids']==children
    assert store.get_command(children[0])['status']=='completed'
    assert frames==[], 'source already-closed branch must not command hardware'


def test_worker_exception_after_child_and_expired_lease_is_terminal(named,monkeypatch,tmp_path):
    def expire(store):
        with store._transaction() as conn:
            conn.execute('UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1')
    store, claimed, called, children, frames = prepare(named,monkeypatch,tmp_path,fault=expire)
    done = run_worker(store,claimed)
    assert children and store.get_command(children[0])['status']=='completed'
    assert done['command']['status']=='ambiguous', done
    assert called==[] and frames==[]
    for table in ('operator_commands','operator_plane_commands'):
        row = store.connection.execute('SELECT status,finished_at FROM '+table+' WHERE command_id=?',('door-parent',)).fetchone()
        assert row['status']=='ambiguous' and row['finished_at'] is not None
    assert done['protocol']['document']['protocol_id']=='offline-door'
    assert done['error']=='workflow_dispatcher_exception'
    assert done['exception_type']=='ValueError'
    assert done['exception_message']=='workflow_authority_lost'
    assert done['execution']['runtime_state']['workflow']['child_command_ids']==children


@pytest.mark.parametrize('change',["owner_id='replacement'",'dispatcher_epoch=dispatcher_epoch+1'])
def test_terminalization_does_not_write_after_owner_replacement(named,monkeypatch,tmp_path,change):
    store, claimed, *_ = prepare(named,monkeypatch,tmp_path)
    with store._transaction() as conn:
        conn.execute('UPDATE operator_plane_lane SET '+change+' WHERE singleton=1')
    with pytest.raises(ValueError,match='workflow_authority_lost'):
        store.finish_workflow('door-parent',status='ambiguous',payload={},lifecycle_settled=False)
    assert store.get_workflow('door-parent')['command']['status']=='dispatched'

    with store._transaction() as conn:
        conn.execute('UPDATE operator_plane_lane SET owner_id=?,dispatcher_epoch=? WHERE singleton=1',
                     (store.owner_id, claimed['dispatcher_epoch']))


@pytest.mark.parametrize('status',['completed','failed','interrupted','cleared'])
def test_expired_lease_never_authorizes_known_outcome_or_native_entry(named,monkeypatch,tmp_path,status):
    store, claimed, *_ = prepare(named,monkeypatch,tmp_path)
    with store._transaction() as conn:
        conn.execute('UPDATE operator_plane_lane SET owner_lease_until=0 WHERE singleton=1')
    with pytest.raises(ValueError,match='workflow_authority_lost'):
        store.assert_workflow_current('door-parent')
    with pytest.raises(ValueError,match='workflow_authority_lost'):
        store.finish_workflow('door-parent',status=status,payload={},lifecycle_settled=True)
    assert store.get_workflow('door-parent')['command']['status']=='dispatched'


def test_worker_setup_exception_preserves_bundle(named,monkeypatch,tmp_path):
    store, claimed, called, children, frames = prepare(named,monkeypatch,tmp_path)
    def broken(command):
        raise RuntimeError('offline binding setup failure')
    store.bind_workflow_dispatcher(broken)
    done = run_worker(store,claimed)
    assert done['command']['status']=='ambiguous'
    assert done['exception_message']=='offline binding setup failure'
    assert done['exception_type']=='RuntimeError'
    assert done['protocol']['document']['protocol_id']=='offline-door'
    assert children==called==frames==[]
