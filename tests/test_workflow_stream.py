"""Frozen BMS recipe through executor, receipt owner, driver and inert USB."""
import asyncio
import gzip
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import z_stop_offline_guard  # noqa: F401
from z_stop_fixtures import HardwareEndpoints
from bioxp.can_driver import BioXpCanDriver
from bioxp.novo_router import NovoRouter
from bioxp.novo_usb_can import NovoUsbCanBus, novo_decode, novo_encode
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.pipette.cavro_application import run_application_inline
from bioxp.pipette.cavro_liquid import compile_liquid_recipe
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport
from bioxp.protocols.executor import ProtocolExecutor
from bioxp.protocols.models import ProtocolAction, ProtocolActionKind as K, ProtocolDocument, ProtocolStage
from bioxp.services.pipette_service import run_pipette_operation


class Pumps(HardwareEndpoints):
    def __init__(self, fault):
        super().__init__()
        self.fault = fault
        self.channels = [False, False, False, True]
        self.wires = []

    def write(self, frame, timeout=None):
        raw = novo_decode(frame)
        addr = int.from_bytes(raw[:4], 'big')
        ch = (addr >> 3) & 3
        wire = raw[5:].decode().rstrip('\0')
        self.wires.append((ch, wire))
        def rx(fn, data):
            self.replies.put(novo_encode(NovoUsbCanBus.build_payload(0x500 | ch << 3 | fn, data)))
        if wire.startswith('?'):
            assert wire in ('?77', '?67')
            token = 'PSON' if self.channels[ch] else 'PSOFF'
            if self.fault == 'unknown' and ch == 1: token = 'UNKNOWN'
            text = ('1' if self.fault=='buffered' else '0') if wire=='?67' else f'GPI|GPO|{token}|PSMON|TDEON'
            body = b' `'+text.encode()
            parts = [body[i:i+8] for i in range(0,len(body),8)]
            for i,part in enumerate(parts): rx(6 if i==len(parts)-1 else 3 if i==0 else 4,part)
        else:
            rx(1,b'')
            if wire.startswith('o0,'): self.channels[ch] = wire=='o0,1R'
            if self.channels[ch]: rx(5,bytes([49,0,91 if wire.startswith('o') else 31+ch]))
            error = (self.fault=='failure' and wire.startswith('P') and ch==1)
            if self.fault=='stop' and wire.startswith('P') and ch==3: self.group._interrupt_epoch += 1
            if self.fault=='restore' and wire=='o0,0R': error=True
            if (self.fault,wire) in [('enable_ack','o0,1R'),('restore_ack','o0,0R')]: return len(frame)
            if self.fault == 'search_timeout' and wire.startswith('B'): return len(frame)
            rx(1,bytes([35 if error else 32,96]))
        return len(frame)


@pytest.mark.parametrize('marker,fault', [(True,'ok'),(False,'ok'),(None,'ok'),(True,'unknown'),
    (True,'stop'),(True,'failure'),(True,'restore'),(True,'authored'),(True,'buffered'),(True,'enable_ack'),(True,'restore_ack'),(True,'lost_owner'),(True,'search_timeout')])
def test_workflow_recipe_stream(tmp_path, monkeypatch, marker, fault):
    monkeypatch.setattr('bioxp.pipette.receipts._current_replay_identity', lambda:{})
    monkeypatch.setattr(PipetteReceiptStore,'_source_identity',staticmethod(lambda:{}))
    OEMRuntimeStore(tmp_path).close()
    ep=Pumps(fault)
    router=NovoRouter(ep_in=ep, ep_out=ep, decode=novo_decode,read_timeout_ms=10); router.start()
    try:
        bus=NovoUsbCanBus(shared_usb=NS(novo_router=router,ep_in=ep,ep_out=ep))
        transports=[]
        for ch in range(4):
            d=object.__new__(BioXpCanDriver); d.pipette_id=ch; d.bus=bus; d.response_timeout_s=.25; d._pipette_message_state={}
            t=CanPipetteTransport(pipette_id=ch); t._driver=d; t._initialized=True; transports.append(t)
        group=FourPipetteTransport(transports); ep.group=group; group._tip_type=50
        with gzip.open(Path(__file__).parents[1]/'testdata/contracts/attempt_transfer_forms.json.gz','rt') as f:
            params=json.load(f)['single'][0]
        recipe=params['recipe']; recipe['channels']=[1,3]
        if fault=='search_timeout':
            recipe.update(channels=[1], timeout_ms=500, liquid_detection={'location_id':3, 'mode':'by_level'})
        if fault=='authored': recipe['phase_settings']={'leading_air':{'pressure_streaming':False}}
        action=ProtocolAction('a','s',K.PIPETTE_MANUAL_PHYSICAL,params,source_occurrence_id='occ',on_error='stop')
        metadata={'bms_method':{} if marker is None else {'workflow_pressure_streaming':marker}}
        doc=ProtocolDocument('frozen',metadata=metadata,stages=(ProtocolStage('s',actions=(action,)),))
        async def inline(_,fn,**kw): return fn()
        def receipt(name,fn,cmd,owner,req):
            store=PipetteReceiptStore(tmp_path)
            try: return asyncio.run(run_pipette_operation(name,fn,get_transport=lambda:group,run_blocking=inline,
                receipt_store=store,runtime_binding={'idempotency_key':owner['source_identity']}))
            finally: store.connection.close()
        provider=NS(primitives=NS(pipette_transport=group),_wp8_execution_fence_checker=lambda *a,**k:None,
            _wp8_execute_nested_plan=lambda **kw:{'ok':True},_manual_pipette_receipt_runner=receipt)
        if fault == 'search_timeout':
            # The USB fixture does not emulate TR's cancellation protocol. Give
            # the application the same correlated receipt as an owned cleanup.
            def terminate():
                group._interrupt_epoch += 1
                ep.wires.append((-1, 'TERMINATED'))
                return {'ok':True, 'interrupt_epoch':group._interrupt_epoch,
                        'channels':[{'channel':1,'completion':{'ok':True}}]}
            group.terminate = terminate
            monkeypatch.setattr('bioxp.oem_compat.position_table.load_bound_oem_position_table',
                lambda: NS(resolve=lambda **_: NS(z_high=60000,z_low=80000)))
            provider.moveZ = lambda _: {'ok':True}
            provider._offset_deck_semantic_state = lambda **_: {'pseudo_z_home':500}
            provider.primitives.z_set_max_speed = lambda _: {'ok':True}
            provider.primitives.oem_move_z = lambda *a,**kw: {'ok':True}
            provider.primitives._read_axis_position = lambda _: 77984
            provider.primitives.z_stop = lambda: (ep.wires.append((-1,'Z_STOP')), {'ok':True})[1]
        def leaf(a,state):
            app=compile_liquid_recipe(a.to_payload()['params']['recipe'])['application']
            return run_application_inline(provider,app,command_id='job',owner_identity={'source_identity':'a'})
        def fence(*_):
            if fault=='lost_owner' and (3,'P20,1R') in ep.wires: raise RuntimeError('lease lost')
        provider._wp8_execution_fence_checker = lambda *a,**kw: fence()
        ex=ProtocolExecutor(dry_run=False,job_id='job',before_native_entry=fence,handlers={action.kind:leaf})
        row=ex.execute(doc).to_payload()['action_results'][0]
        if fault == 'search_timeout':
            assert row['status'] == 'failed', (row,ep.wires)
            refs = [r for r in row['pressure_traces'] if r['stroke']=='fluid detection']
            ref, = refs
            assert ref['channels']==[1]
            assert ep.wires.index((1,'o0,1R')) < ep.wires.index((1,'BR')) < ep.wires.index((-1,'Z_STOP'))
            assert (1,'o0,0R') in ep.wires, json.dumps([ref, ep.wires, row.get('error'), row.get('detail'), str(row.get('pipette_result',{}))[-1500:]])
            assert ep.wires.index((-1,'Z_STOP')) < ep.wires.index((1,'o0,0R'))
            assert ep.channels[1] is False
            cold=PipetteReceiptStore(tmp_path)
            try:
                cap=cold.scientific_pressure_capture(ref['stream_session_id'])
                assert cap['stream']['terminal_state']=='failed'
                chunk,=cap['chunks']
                assert chunk['channel']==1
                assert [s['raw_pressure'] for s in chunk['scientific_result']['samples']]==[32]
            finally: cold.connection.close()
            return
        assert (row['status']=='completed') == (fault not in ('failure','stop','lost_owner')), (row,ep.wires)
        strokes=[(ch,w) for ch,w in ep.wires if w[0] in 'PDA']
        expected=[(c,w) for w in ('P20,1R','P22.5,1R','P1.25,1R','D22.5,1R','A0R') for c in [1,3]]
        assert strokes==(expected[:2] if fault in ('failure','stop','lost_owner') else expected), ep.wires
        changes=[pair for pair in ep.wires if pair[1].startswith('o')]
        active=marker is True and fault not in ('unknown','buffered','authored')
        assert changes==([(1,'o0,0R'),(3,'o0,0R')] if fault=='authored' else
            [(1,'o0,1R')] if fault in ('stop','enable_ack','lost_owner') else [(1,'o0,1R'),(1,'o0,0R')]*(len(strokes)//2) if active else []),ep.wires
        assert not any(w.startswith(('b','i','U')) or ch not in [1,3] for ch,w in ep.wires)
        if marker is not True or fault=='authored': assert not any(w.startswith('?') for _,w in ep.wires)
        refs=row.get('pressure_traces',[])
        if marker is False: assert not refs; return
        assert len(refs)==len(strokes)//2
        cold=PipetteReceiptStore(tmp_path)
        try:
            for ref in refs:
                data=cold.scientific_pressure_capture(ref['stream_session_id'])
                values={c['channel']:[s['raw_pressure'] for s in c['scientific_result']['samples']] for c in data['chunks']}
                assert values=={1:[32] if active else [],3:[] if fault=='authored' else [34]}
                if fault=='enable_ack' and ref is refs[0]: assert ref['stream_control'][0]['enable']=='unresolved'
                if fault in ('restore','restore_ack','stop','lost_owner'): assert ref['stream_control'][0]['restore']=='unresolved'
            if marker is True and fault=='ok':
                import os
                if out := os.environ.get('STROKE_EVIDENCE_DIR'):
                    Path(out,'recipe-stream.json').write_text(json.dumps({'wires':ep.wires,'traces':refs,
                        'captures':[cold.scientific_pressure_capture(r['stream_session_id']) for r in refs]},indent=2))
        finally: cold.connection.close()
    finally: router.shutdown()
