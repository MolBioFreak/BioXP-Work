import asyncio
import json
import os
from pathlib import Path
import pytest
from threading import Event, Thread
from types import SimpleNamespace as NS

import z_stop_offline_guard  # noqa: F401
from bioxp.novo_router import NovoRouter
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.pipette.cavro_application import _send_wire
from bioxp.pipette.transport import FourPipetteTransport
from bioxp.protocols.executor import ProtocolExecutor
from bioxp.protocols.models import ProtocolAction, ProtocolActionKind as K, ProtocolDocument, ProtocolStage
from bioxp.services.pipette_service import run_pipette_operation


@pytest.mark.parametrize('mode', ['ordinary', 'interrupted', 'empty', 'overflow', 'writer_error', 'reader_busy'])
def test_failed_stroke_retry_cold_read(tmp_path, monkeypatch, mode):
    monkeypatch.setattr("bioxp.pipette.receipts._current_replay_identity", lambda: {})
    monkeypatch.setattr(PipetteReceiptStore, "_source_identity", staticmethod(lambda: {}))
    OEMRuntimeStore(tmp_path).close()
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=bytes)
    def frame(value, ch=1):
        data = (0x505 | ch << 3).to_bytes(4, 'big') + bytes([3, 49, 0, value])
        router._dispatch(router._decode_record(data, data, router._clock()))
    if mode == 'writer_error':
        def fail(*args, **kwargs): raise OSError('inert writer error')
        monkeypatch.setattr(PipetteReceiptStore, 'retain_scientific_pressure_capture', fail)
    if mode == 'reader_busy':
        prior = router.begin_scientific_pressure_capture(dict(channels=[0], max_frames=1, duration_ms=1000))
    calls = []
    def send(wire, **kw):
        calls.append(wire)
        if mode != 'empty':
            frame(10 + len(calls)); frame(99, 2)
        if mode == 'overflow':
            for _ in range(260): frame(6)
        return {'ok': True}
    driver = NS(bus=NS(router=router), _send_pipette_command=send)
    def wait(timeout, **kw):
        if mode != 'empty': frame(20 + len(calls))
        if mode == 'interrupted' and len(calls) == 1: group._interrupt_epoch += 1
        return {'ok': len(calls) == 2}
    channel = NS(_driver=driver, _get_driver=lambda:driver, _require_initialized=lambda:None,
                 _assert_driver_result=lambda *a:None, _driver_evidence=lambda _: {},
                 apply_completed_effect=lambda *a:{}, wait_for_completion=wait)
    group = FourPipetteTransport([channel]*4); group._tip_type = 50
    async def inline(_, fn, **kw): return fn()
    def leaf(action, state):
        store = PipetteReceiptStore(tmp_path)
        frame(77)
        try:
            return asyncio.run(run_pipette_operation('aspirate',
                lambda t:_send_wire(t, [1], 'P1,1R', 500, effect='aspirate'),
                get_transport=lambda:group, run_blocking=inline, receipt_store=store,
                runtime_binding={'idempotency_key':action.action_id}))
        finally:
            frame(88); store.connection.close()
    held = Event()
    action = ProtocolAction('a','s',K.PIPETTE_ASPIRATE,source_occurrence_id='occ',on_error='pause_for_operator')
    doc = ProtocolDocument('test', stages=(ProtocolStage('s',actions=(action,)),))
    ex = ProtocolExecutor(dry_run=False, job_id='job', before_native_entry=lambda *_:None,
        handlers={K.PIPETTE_ASPIRATE:leaf}, on_state_change=lambda s: held.set() if s.workflow.gate=='error_hold' else None)
    worker = Thread(target=ex.execute,args=(doc,),daemon=True); worker.start()
    try:
        assert held.wait(5), ex._state.action_results
        assert ex._state.action_results[0].get("unresolved_attempt"), repr(ex._state.action_results)
        ex.request_control('recover', control_id='retry', gate_id=ex._state.workflow.gate_id,
            decision='retry', source_occurrence_id='occ', native_action_id='a',expected_attempt=1,overrides={})
        worker.join(5)
        assert ex.outcome=='completed', ex._state.action_results
        assert calls == ['P1,1R']*2
        rows = ex._state.to_payload()["action_results"]
        refs = [r['pressure_traces'][0] for r in rows]
        if mode in ('reader_busy', 'writer_error'):
            assert all(r.get('observation_error') and 'stream_session_id' not in r for r in refs)
            if mode == 'reader_busy': assert router._scientific_pressure_capture is prior
            return
        assert refs[0]['stream_session_id'] != refs[1]['stream_session_id']
        cold = PipetteReceiptStore(tmp_path)
        for n, ref in enumerate(refs, 1):
            assert (ref['job_id'], ref['source_occurrence_id'], ref['native_action_id'], ref['attempt']) == ('job','occ','a',n)
            data = cold.scientific_pressure_capture(ref['stream_session_id'])
            assert data['stream']['terminal_state'] == (('interrupted' if mode=='interrupted' else 'failed') if n==1 else 'completed')
            chunk, = data['chunks']; assert chunk['channel']==1
            values = [r['raw_pressure'] for r in chunk['scientific_result']['samples']]
            assert values == ([] if mode=='empty' else [10+n]+[6]*255 if mode=='overflow' else [10+n,20+n])
            meta = chunk['scientific_result']['scientific_result']
            assert meta['lost_frame_count'] == (6 if mode=='overflow' else 0)
            assert meta['stroke_identity']['attempt'] == n
            assert 'samples' not in ref
            if mode == 'empty': assert 'no selected-channel pressure' in ref['observation_error']
        if mode == 'ordinary' and os.getenv('STROKE_EVIDENCE_DIR'):
            Path(os.environ['STROKE_EVIDENCE_DIR'], 'attempt-results.json').write_text(json.dumps(
                [{k:r[k] for k in ('action_id','source_occurrence_id','attempt','status','pressure_traces')} for r in rows],indent=2))
        cold.connection.close()
    finally:
        ex.interrupt(control_id='teardown'); worker.join(5)
