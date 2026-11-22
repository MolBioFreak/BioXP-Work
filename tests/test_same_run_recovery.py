"""Same-process authored executor with inert leaves; no provider or transport."""
from threading import Event, Thread
import unittest

from bioxp.protocols.executor import ProtocolExecutor
from bioxp.protocols.models import ProtocolAction, ProtocolActionKind as K, ProtocolStage, ProtocolDocument


class RecoveryTest(unittest.TestCase):
    def test_retry(self):
        self.exercise('retry')

    def test_skip(self):
        self.exercise('skip')

    def test_manual_complete(self):
        self.exercise('manual_complete')

    def exercise(self, decision):
        held = Event()
        calls = []
        def leaf(action, state):
            calls.append(action.action_id)
            return {'ok': len(calls) != 1, 'partial': {'amount': None}}
        def publish(state):
            if state.workflow.gate == 'error_hold':
                held.set()
        first = ProtocolAction('first', 'stage', K.PIPETTE_ASPIRATE, on_error='pause_for_operator')
        second = ProtocolAction('second', 'stage', K.LED, {'red':0,'green':0,'blue':0})
        doc = ProtocolDocument('fixture', stages=(ProtocolStage('stage', actions=(first, second)),))
        ex = ProtocolExecutor(dry_run=False, job_id='inert', before_native_entry=lambda *_: None,
                              handlers={K.PIPETTE_ASPIRATE: leaf, K.LED: leaf}, on_state_change=publish)
        worker = Thread(target=ex.execute, args=(doc,), daemon=True)
        worker.start()
        try:
            self.assertTrue(held.wait(3))
            wf = ex._state.workflow
            req = dict(control_id='decision', gate_id=wf.gate_id, decision=decision,
                       source_occurrence_id='first', native_action_id='first', expected_attempt=1, overrides={})
            ex.request_control('recover', **req)
            ex.request_control('recover', **req)
            worker.join(3)
            self.assertEqual(ex.outcome, 'completed')
            rows = ex._state.action_results
            self.assertEqual(rows[0]['partial'], {'amount': None})
            self.assertFalse(rows[0]['ok'])
            if decision == 'retry':
                self.assertEqual([r['attempt'] for r in rows if r['action_id']=='first'], [1, 2])
                self.assertEqual(len(set(calls)), 3)
            else:
                self.assertEqual(rows[0]['resolution'], 'skipped' if decision == 'skip' else 'operator_completed')
                self.assertEqual(calls, ['first', 'second'])
        finally:
            ex.interrupt(control_id='teardown')
            worker.join(3)

    def test_implicit_error_policy_does_not_convert_ambiguous_failure_to_operator_hold(self):
        gates = []
        action = ProtocolAction('first', 'stage', K.PIPETTE_ASPIRATE, on_error='pause_for_operator',
                                metadata={'bms_method': {'default_error_hold': True}})
        doc = ProtocolDocument('fixture', stages=(ProtocolStage('stage', actions=(action,)),))
        ex = ProtocolExecutor(dry_run=False, job_id='inert', before_native_entry=lambda *_: None,
                              handlers={K.PIPETTE_ASPIRATE: lambda *_: {'ok': False, 'uncertain': True}},
                              on_state_change=lambda state: gates.append(state.workflow.gate))
        worker = Thread(target=ex.execute, args=(doc,), daemon=True)
        worker.start()
        try:
            worker.join(2)
            self.assertFalse(worker.is_alive(), 'an implicit default trapped a terminal outcome in an operator hold')
            self.assertNotIn('error_hold', gates)
            self.assertNotEqual(ex.outcome, 'completed')
        finally:
            if worker.is_alive():
                ex.interrupt(control_id='teardown'); worker.join(3)

    def test_sqlite_retry_reopen_and_replay(self):
        import tempfile
        import time
        from types import SimpleNamespace
        from bioxp.oem_runtime_store import OEMRuntimeStore
        from bioxp.operator_command_plane import OperatorCommandStore
        from bioxp.protocols.runtime_state import ProtocolRuntimeState
        from bioxp.services.protocol_service import bind_protocol_dispatcher, control_protocol_job

        with tempfile.TemporaryDirectory() as root:
            OEMRuntimeStore(root).close()
            store = OperatorCommandStore(root)
            prefix = ProtocolAction('prefix','stage',K.PIPETTE_MANUAL_PHYSICAL,
                {'operation':'load_tip', 'tray':1, 'well':'A1'}, source_occurrence_id='first')
            recipe = dict(mode='single',channels=[0],timeout_ms=1000,target_liquid_ul=1,
                commanded_aspiration_ul=1,aspiration_speed_ul_s=1,aspiration_delay_ms=10,
                leading_air=None,trailing_air=None,before_leading_air=[],before_liquid=[],after_liquid=[],
                before_dispense=[],after_dispense=[],dispense_segments=[{'volume_ul':1,'speed_ul_s':1}],
                final_empty_before=[],final_empty_speed_ul_s=None,multi=None)
            first = ProtocolAction('first','stage',K.PIPETTE_MANUAL_PHYSICAL,
                {'operation':'cavro_liquid_recipe','recipe':recipe}, source_occurrence_id='first',on_error='pause_for_operator')
            second = ProtocolAction('second', 'stage', K.LED, {'red':0,'green':0,'blue':0})
            doc = ProtocolDocument('fixture', stages=(ProtocolStage('stage', actions=(prefix, first, second)),))
            job = 'sqlite-witness'
            state = ProtocolRuntimeState.from_document(doc, dry_run=False, job_id=job)
            bundle = {'protocol':{'document':doc.to_payload()}, 'execution':{'runtime_state':state.to_payload()},
                      'operator':{'reviews':[]}}
            calls = []
            def leaf(action, state):
                calls.append(action.action_id)
                return {'ok':action.params.get('operation') != 'cavro_liquid_recipe' or len(calls) >= 6, 'partial':{'amount':None}}
            bind_protocol_dispatcher(store, binding_factory=lambda *a, **kw: ({K.PIPETTE_MANUAL_PHYSICAL:leaf, K.LED:leaf},{},{}),
                                     artifact_store=SimpleNamespace(save=lambda _:None))
            store.admit_workflow(command_id=job, idempotency_key='root', plan_fingerprint='inert',
                                 requested_inputs={'bundle':bundle}, ownership_generation=7, resources=(), board_epochs={})
            def wait(predicate):
                deadline = time.monotonic()+5
                while True:
                    row = store.get_workflow(job)
                    if predicate(row):
                        return row
                    self.assertLess(time.monotonic(), deadline, row)
                    time.sleep(.005)
            def send(req):
                return control_protocol_job(job, req, command_store=store)
            def held(n):
                def matches(row):
                    state = row['execution']['runtime_state']
                    return ((state.get('workflow') or {}).get('gate')=='error_hold' and state['action_results'][-1].get('attempt')==n)
                return wait(matches)['execution']['runtime_state']['workflow']['gate_id']
            store.start(lambda _: self.fail('Unexpected native dispatch'))
            try:
                gate = held(1)
                body = dict(command_id=job, expected_ownership_generation=7, idempotency_key='choose', action='recover',
                            native_action_id='first', source_occurrence_id='first', expected_attempt=1,
                            gate_id=gate, decision='retry', scope='occurrence', overrides={'aspiration_delay_ms':5})
                receipt = send(body)
                next_gate = held(2)
                self.assertNotEqual(next_gate, gate)
                self.assertEqual(send(body), receipt)
                with self.assertRaises(ValueError):
                    send({**body,'idempotency_key':'compete'})
                with self.assertRaises(ValueError):
                    send({**body,'idempotency_key':'wrong-attempt','gate_id':next_gate})
                # A delayed control reply must not become unresolved physical custody.
                callback = store._workflow_controls[job]
                def delayed_reply(control_id, request):
                    reply = callback(control_id, request)
                    wait(lambda row:row['command']['terminal'])
                    return reply
                store._workflow_controls[job] = delayed_reply
                send({**body,'idempotency_key':'choose-again','gate_id':next_gate,'expected_attempt':2})
                done = wait(lambda row:row['command']['terminal'])
                self.assertEqual(done['command']['status'], 'completed', done)
                self.assertEqual(send(body), receipt)
            finally:
                store.stop()
            cold = OperatorCommandStore(root)
            try:
                rows = cold.get_workflow(job)['execution']['runtime_state']['action_results']
                self.assertEqual([r['action_id'] for r in rows], ['prefix','first','prefix','first','prefix','first','second'])
                self.assertEqual(len(set(calls)), 7)
                self.assertEqual(rows[5]['overrides'], {'aspiration_delay_ms':5})
                self.assertEqual(rows[4]['overrides'], {})
                self.assertEqual([r['partial'] for r in (rows[1],rows[3])], [{'amount':None}]*2)
                self.assertEqual(control_protocol_job(job, body, command_store=cold), receipt)
            finally:
                cold.stop()

    def test_repeat_at_pause(self):
        self.repeat_and_return(False)

    def test_repeat_at_error(self):
        self.repeat_and_return(True)

    def repeat_and_return(self, error):
        from queue import Queue
        holds = Queue()
        calls = []
        def leaf(action, state):
            calls.append(action.action_id)
            if not error and action.action_id == 'b':
                ex.request_control('pause', control_id='pause', mode='ordinary')
            return {'ok': not (error and action.action_id == 'b')}
        def publish(state):
            if state.workflow.gate and state.workflow.phase == 'waiting':
                holds.put(state.workflow.to_payload())
        actions = tuple(ProtocolAction(key,'s',K.PIPETTE_DISPENSE, source_occurrence_id='loop/'+key,
                                       on_error='pause_for_operator') for key in 'abc')
        ex = ProtocolExecutor(job_id='inert', dry_run=False, handlers={K.PIPETTE_DISPENSE:leaf},
                              on_state_change=publish, before_native_entry=lambda *_: None)
        worker = Thread(target=ex.execute,args=(ProtocolDocument('fixture',stages=(ProtocolStage('s',actions=actions),)),),daemon=True)
        worker.start()
        try:
            initial = holds.get(timeout=3)
            kind = 'error_hold' if error else 'ordinary_pause'
            self.assertEqual(initial['gate'],kind)
            repeat = dict(gate_id=initial['gate_id'],decision='repeat',native_action_id='a',source_occurrence_id='loop/a',expected_attempt=1)
            ex.request_control('recover',control_id='repeat',**repeat)
            returned = holds.get(timeout=3)
            self.assertEqual((returned['gate'],returned['source_occurrence_id']), (kind,'loop/b'))
            self.assertNotEqual(returned['gate_id'], initial['gate_id'])
            self.assertEqual(calls[:2], ['a','b'])
            self.assertEqual(len(calls), 3)
            with self.assertRaises(ValueError):
                ex.request_control('recover',control_id='stale',**repeat)
            if error:
                ex.request_control('recover',control_id='finish',gate_id=returned['gate_id'],decision='retry',
                                   native_action_id='b',source_occurrence_id='loop/b',expected_attempt=1)
            else:
                ex.request_control('continue',control_id='finish',gate='ordinary_pause',gate_id=returned['gate_id'])
            worker.join(3)
            self.assertEqual(ex.outcome,'completed')
            self.assertEqual(calls.count('c'),1)
            self.assertEqual(ex._state.stage_states['s'].completed_actions,['a','b','c'])
        finally:
            ex.interrupt(control_id='tear-down'); worker.join(3)

    def test_recipe_adjustments_are_attempt_local(self):
        from bioxp.protocols.recovery import adjusted
        recipe = dict(mode='single',channels=[0],timeout_ms=1000,target_liquid_ul=1,
                      commanded_aspiration_ul=1,aspiration_speed_ul_s=1,aspiration_delay_ms=10,
                      leading_air=None,trailing_air=None,before_leading_air=[],before_liquid=[],after_liquid=[],
                      before_dispense=[],after_dispense=[],dispense_segments=[{'volume_ul':1,'speed_ul_s':1}],
                      final_empty_before=[],final_empty_speed_ul_s=None,multi=None)
        action = ProtocolAction('a','s',K.PIPETTE_MANUAL_PHYSICAL,{'operation':'cavro_liquid_recipe','recipe':recipe})
        self.assertEqual(adjusted(action,{'aspiration_delay_ms':5}).params['recipe']['aspiration_delay_ms'],5)
        self.assertEqual(action.params['recipe']['aspiration_delay_ms'],10)
        for overrides in ({'phase_settings':{}},{'pressure':1},{'aspiration_delay_ms':True},{'operations':[]}):
            with self.assertRaises(ValueError):
                adjusted(action,overrides)

    def test_interrupt_after_intent_prevents_dispatch(self):
        self.interrupt_retry('interrupt')

    def test_abort_after_intent_prevents_dispatch(self):
        self.interrupt_retry('abort')

    def interrupt_retry(self, how):
        calls = []
        def leaf(action, state):
            calls.append(action.action_id)
            return {'ok':False}
        def publish(state):
            wf = state.workflow
            if wf.gate == 'error_hold':
                ex.request_control('recover', control_id='key', decision='retry', gate_id=wf.gate_id,
                                   native_action_id='a', source_occurrence_id='a', expected_attempt=1)
            elif state.action_results and state.action_results[-1].get('status') == 'accepted':
                if how == 'interrupt':
                    ex.interrupt(control_id='stop')
                else:
                    ex.request_control('abort',control_id='stop')
        ex = ProtocolExecutor(job_id='inert',dry_run=False,handlers={K.PIPETTE_MIX:leaf},
                              on_state_change=publish,before_native_entry=lambda *_:None)
        action = ProtocolAction('a','s',K.PIPETTE_MIX,on_error='pause_for_operator')
        state = ex.execute(ProtocolDocument('fixture',stages=(ProtocolStage('s',actions=(action,)),)))
        self.assertEqual(calls,['a'])
        self.assertEqual(ex.outcome,'interrupted')
        self.assertNotIn('started_at',state.action_results[-1])
        with self.assertRaises(ValueError):
            ex.request_control('recover',control_id='new',decision='retry',gate_id='gone',
                               native_action_id='a',source_occurrence_id='a',expected_attempt=2)

    def test_whole_occurrence_repeat_and_retry(self):
        from queue import Queue
        holds = Queue()
        trace = []
        def leaf(action, state):
            name = action.params['name']
            trace.append(name)
            return {'ok': not (name == 'y' and trace.count('y') == 1)}
        actions = tuple(ProtocolAction(n, 's', kind, {'name': n}, source_occurrence_id=occ,
                                 on_error='pause_for_operator') for n, kind, occ in (
            ('a', K.PIPETTE_MANUAL_PHYSICAL, 'loop/0'), ('b', K.PIPETTE_POSITION, 'loop/0'),
            ('x', K.PIPETTE_POSITION, 'loop/1'), ('y', K.PIPETTE_DISPENSE, 'loop/1'),
            ('z', K.PIPETTE_ASPIRATE, 'loop/1')))
        ex = ProtocolExecutor(job_id='owner', dry_run=False, handlers={a.kind:leaf for a in actions},
            before_native_entry=lambda *_:None, on_state_change=lambda s: holds.put(s.workflow.gate_id)
            if s.workflow.phase == 'waiting' else None)
        worker = Thread(target=ex.execute, args=(ProtocolDocument('fixture', stages=(ProtocolStage('s',actions=actions),)),), daemon=True)
        worker.start()
        try:
            gate = holds.get(timeout=3)
            def choose(decision, key, occ, gate, cid):
                ex.request_control('recover', control_id=cid, decision=decision, native_action_id=key,
                    source_occurrence_id=occ, expected_attempt=1, scope='occurrence', gate_id=gate)
            with self.assertRaises(ValueError):
                choose('repeat', 'x', 'loop/1', gate, 'partial')
            choose('repeat', 'b', 'loop/0', gate, 'repeat')
            returned = holds.get(timeout=3)
            self.assertNotEqual(returned, gate)
            self.assertEqual(trace, list('abxyab'))
            choose('retry', 'y', 'loop/1', returned, 'retry')
            worker.join(3)
            self.assertEqual(trace, list('abxyabxyz'))
            self.assertEqual(ex.outcome, 'completed')
        finally:
            ex.interrupt(control_id='stop'); worker.join(3)

    def test_shared_move_repeats_only_selected_target_and_returns(self):
        from queue import Queue
        holds, targets = Queue(), []
        members = [{'occurrence_id':o, 'target_index':i} for i,o in enumerate(('outer/0','outer/1'))]
        batch = ProtocolAction('shared','s',K.PIPETTE_POSITION,
            {'operation':'move_sequence', 'targets':[{'location_id':3}, {'location_id':4}]},
            metadata={'bms_method':{'sequence':members}}, source_occurrence_id='outer/0')
        stop = ProtocolAction('stop','s',K.LED, {'red':0,'green':0,'blue':0}, source_occurrence_id='stop')
        def move(a, state):
            targets.append(tuple(t['location_id'] for t in a.params['targets']))
            return {'ok':len(targets) != 2}
        def pause(a, state):
            ex.request_control('pause', control_id='pause', mode='ordinary')
            return {'ok':True}
        ex = ProtocolExecutor(job_id='owner', dry_run=False, handlers={K.PIPETTE_POSITION:move,K.LED:pause},
            before_native_entry=lambda *_:None, on_state_change=lambda s:holds.put(s.workflow.to_payload())
            if s.workflow.phase == 'waiting' else None)
        worker = Thread(target=ex.execute, args=(ProtocolDocument('fixture',stages=(ProtocolStage('s',actions=(batch,stop)),)),), daemon=True)
        worker.start()
        try:
            gate = holds.get(timeout=3)['gate_id']
            choice = dict(gate_id=gate, decision='repeat', source_occurrence_id='outer/1',native_action_id='shared',expected_attempt=1)
            with self.assertRaises(ValueError):
                ex.request_control('recover',control_id='ambiguous',**choice)
            ex.request_control('recover',control_id='repeat',scope='occurrence',**choice)
            nested = holds.get(timeout=3)
            self.assertEqual((nested['gate'],nested['source_occurrence_id']),('error_hold','outer/1'))
            ex.request_control('recover',control_id='retry',scope='occurrence',**{**choice,'gate_id':nested['gate_id'],'decision':'retry','expected_attempt':2})
            returned = holds.get(timeout=3)
            self.assertEqual((returned['gate'],returned['source_occurrence_id']),('ordinary_pause','stop'))
            ex.request_control('continue',control_id='resume',gate='ordinary_pause',gate_id=returned['gate_id'])
            worker.join(3)
            self.assertEqual(ex.outcome,'completed')
            self.assertEqual(targets,[(3,4),(4,),(4,)])
            self.assertEqual(len(batch.params['targets']),2)
        finally:
            ex.interrupt(control_id='teardown'); worker.join(3)
