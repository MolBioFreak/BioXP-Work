"""Reproduce the actual post-move writer/provider inversion without live I/O."""
import os, subprocess, sys, threading, json, sqlite3, shutil
from pathlib import Path
import pytest
from tests.test_deck_scoped_authority import retained_rig, qualify_test_references

@pytest.mark.parametrize('scenario', ['poll', 'owner_drift'])
def test_finalization_lock_order(retained_rig, monkeypatch, scenario):
    if os.environ.get('DECK_FINAL_CHILD') != scenario:
        try:
            r=subprocess.run([sys.executable,'-m','pytest','-p','no:cacheprovider','-p','tests.z_stop_offline_guard','-s','-q',f'{__file__}::test_finalization_lock_order[{scenario}]'],env=dict(os.environ,DECK_FINAL_CHILD=scenario),capture_output=True,text=True,timeout=15)
        except subprocess.TimeoutExpired as e:
            pytest.fail('bounded finalization deadlock:\n'+(e.stderr or b'').decode())
        assert r.returncode==0,r.stdout+r.stderr
        return
    import faulthandler
    faulthandler.dump_traceback_later(8)
    from bioxp.oem_compat.position_table import load_bound_oem_position_table
    from bioxp.oem_deck_movement import make_deck_command_executor, DeckExecutionFailure
    provider, primitive, runtime, references, store, root=retained_rig
    qualify_test_references(references)
    stamps=provider.deck_owner_authority_stamps()
    epochs={'4':stamps['board_epoch_4'],'5':stamps['board_epoch_5']}
    req=dict(schema_version='bioxp.operator_action_request.v2',action_id='oem.deck.move_to_location',expected_ownership_generation=3,expected_board_epoch_by_board=epochs,idempotency_key='finalize-lock',inputs={'target':'LOC_OC','camera_offset':False})
    admitted=store.admit_command(req,assessment={'enabled':True},state={'ownership_generation':3,'serial206_initialization_provider':{'x_authority':{'current_board_lifecycle_generation':epochs['5']},'board4_authority':{'active_board_epoch':epochs['4']}}})
    claimed=store.claim_next();assert claimed['command_id']==admitted['command_id']
    original=store.commit_deck_success
    original_acquire=provider._lock.acquire
    attempts_provider=threading.Event();poll_holds_provider=threading.Event();polls=[]
    finalizer_thread=None
    def acquire(*args,**kwargs):
        if threading.get_ident()==finalizer_thread:attempts_provider.set()
        return original_acquire(*args,**kwargs)
    monkeypatch.setattr(provider._lock,'acquire',acquire)
    def commit(*args,**kwargs):
        nonlocal finalizer_thread
        # All physical stages have already returned before this boundary.
        with sqlite3.connect(root/'bioxp_runtime.db') as c:
            row=c.execute('SELECT terminal_state,terminal_evidence_json FROM operator_plane_deck_stages WHERE command_id=? AND stage_order=3',(admitted['command_id'],)).fetchone()
        assert row[0]=='completed' and json.loads(row[1])['controller_completion_verified'] is True
        def poll():
            with provider._lock:
                poll_holds_provider.set();assert attempts_provider.wait(4)
                # This is the same provider -> shared runtime writer order
                # used by the live /status projection in the captured stack.
                with runtime.serial206_projection_scope():
                    runtime.read_oem_serial206_initialization_state()
                    if scenario=='owner_drift':monkeypatch.setattr(provider,'generation_provider',lambda:4)
                polls.append('projected')
        t=threading.Thread(target=poll,daemon=True);t.start();assert poll_holds_provider.wait(4)
        finalizer_thread=threading.get_ident()
        try:return original(*args,**kwargs)
        finally:
            finalizer_thread=None;t.join(4);assert not t.is_alive()
    monkeypatch.setattr(store,'commit_deck_success',commit)
    execute=make_deck_command_executor(provider_getter=lambda:provider,position_table_provider=load_bound_oem_position_table,command_store=store)
    args=dict(command_id=admitted['command_id'],target='LOC_OC',camera_offset=False,expected_ownership_generation=3,expected_board_epoch_by_board=epochs)
    if scenario=='owner_drift':
        with pytest.raises((DeckExecutionFailure,RuntimeError)) as exc:execute(**args)
        chain=[];current=exc.value
        while current is not None:
            chain.append({'type':type(current).__name__,'message':str(current)})
            current=current.__cause__
        Path(os.environ['DECK_TEST_OUTPUT']+'.owner-drift.json').write_text(json.dumps(
            {'chain':chain,'provider_results':exc.value.provider_results,
             'delivery_attempted':exc.value.delivery_attempted,
             'controller_completion_verified':exc.value.controller_completion_verified},indent=2))
        assert exc.value.delivery_attempted is True
        assert exc.value.controller_completion_verified is True
        assert exc.value.provider_results[-1]['controller_completion_verified'] is True
        assert str(exc.value)=='semantic_commit_failed:RuntimeError'
        assert any(str(c)=='deck_execution_ownership_generation_changed' for c in (exc.value.__cause__,exc.value.__context__))
        assert str(exc.value.__cause__)=='deck_execution_ownership_generation_changed'
        with sqlite3.connect(root/'bioxp_runtime.db') as c:
            assert c.execute('SELECT semantic_state_committed FROM operator_plane_deck_commands WHERE command_id=?',(admitted['command_id'],)).fetchone()[0]==0
    else:
        result=execute(**args);assert result['ok'] and result['semantic_state_committed']
        store.finish(admitted['command_id'],status='completed',payload=result,claimed=claimed,controller_acknowledged=True,full_response=result)
        assert store.command_detail_v2(admitted['command_id'])['status']=='completed'
    assert polls==['projected']
    faulthandler.cancel_dump_traceback_later()

def test_actual_stopped_pool_startup_preserves_motion_and_no_replay(tmp_path):
    from bioxp.operator_command_plane import OperatorCommandStore
    src=Path('/home/dalab/.hermes/profiles/fresh/robot-audit/deck-tip-failure/finalization-repair/stopped-current.db')
    root=tmp_path/'stopped-copy';root.mkdir();shutil.copy2(src,root/'bioxp_runtime.db')
    cid='e13b85a2-7afd-4509-82b0-70581e209291'
    def rows():
        with sqlite3.connect(root/'bioxp_runtime.db') as c:
            return {t:c.execute('SELECT * FROM '+t+' ORDER BY 1,2').fetchall() for t in ('operator_plane_commands','operator_plane_deck_commands','operator_plane_deck_stages','operator_plane_delivery_attempts')}
    before=rows();store=OperatorCommandStore(root);after=rows();receipt=store.command_detail_v2(cid)
    assert receipt['status']!='completed'
    assert receipt['deck_movement']['ambiguity_state'] not in (None,'none')
    assert before['operator_plane_deck_stages']==after['operator_plane_deck_stages']
    assert before['operator_plane_delivery_attempts']==after['operator_plane_delivery_attempts']
    for table in ('operator_plane_commands','operator_plane_deck_commands'):
        assert [r for r in before[table] if r[0]!=cid]==[r for r in after[table] if r[0]!=cid]
    with sqlite3.connect(root/'bioxp_runtime.db') as c:
        assert c.execute("SELECT COUNT(*) FROM operator_plane_commands WHERE status IN ('queued','dispatched')").fetchone()[0]==0
        assert c.execute('SELECT semantic_state_committed FROM operator_plane_deck_commands WHERE command_id=?',(cid,)).fetchone()[0]==0
    out=Path(os.environ['POSTMOVE_RECEIPT_EXPORT']).with_suffix('.stopped-startup.json')
    out.write_text(json.dumps({'receipt':receipt,'older_history_unchanged':True,'completed_stages_and_delivery_markers_unchanged':True,'no_command_added_or_replayed':True,'no_dispatcher_or_hardware_started':True},indent=2))
