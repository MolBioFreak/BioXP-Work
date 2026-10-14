"""Manufacturer p32/p40/p56 regression; inert USB frames, original offline guard."""
import json
import time
import pytest
from tests.test_cavro_transport_repairs import receiver, compile_command, export
from tests.test_cavro_application import rig
from bioxp.pipette.cavro_commands import compile_device_program
from bioxp.novo_router import NovoFrame
from bioxp.novo_usb_can import novo_decode

H = {'command':'halt','resume_on':'command_only','input_edge':'falling'}
P = {'command':'plunger_aspirate','value':22.5,'unit':'ul'}
D = {'command':'diagnostic','test':'minimum_moving_current'}

def send(r, command, channels=None):
    return r.group.run_cavro_command(command, channels=channels or [0], timeout_ms=20)

def program(commands, mode='execute'):
    return compile_device_program({'mode':mode,'commands':commands})

def finite(rig, receiver, command):
    return rig.provider._manual_pipette_receipt_runner('device_command',
        lambda _:send(receiver,command), 'offline-finite-owner',
        {'source_identity':'buffer-regression'}, command['requested'])

def test_split_halt_finite_truth(receiver, rig):
    send(receiver,program([H], 'append'))
    result=finite(rig,receiver,compile_command(P))
    assert result['ok']
    assert not result['completion_verified']
    assert not result['receipt_truth']['completion_verified']
    row=result['channels'][0]
    assert row['completion']['ok']  # READY is real; stroke attribution is not.
    assert [wire for _,wire in receiver.wire]==['H1,0','P22.5,1R']
    assert not row['result']['completion_verified']
    assert 'accounting' not in row['result']
    receipt=json.loads(rig.store.connection.execute('SELECT receipt_json FROM pipette_operations WHERE command_id=?',(result['command_id'],)).fetchone()[0])
    assert not receipt['result']['channels'][0]['result']['completion_verified']
    export('split-halt-finite.json',{'result':result,'receipt':receipt,'wire':receiver.wire})

@pytest.mark.parametrize('control',['buffer_execute','buffer_replay','terminate','reset'])
def test_control_cursor_unknown_then_explicit_clear(receiver,control):
    send(receiver,program([H,P],'append'))
    out=send(receiver,compile_command({'command':control}))
    assert out['ok'] and not out['completion_verified']
    out=send(receiver,compile_command(P))
    assert out['ok'] and not out['completion_verified']
    assert 'accounting' not in out['channels'][0]['result']
    send(receiver,compile_command({'command':'buffer_clear'}))
    out=send(receiver,compile_command(P))
    assert out['completion_verified']
    assert out['channels'][0]['result']['accounting']['pneumatic_displacement_delta_ul']==22.5


def replies(receiver, answers, intermediate=None):
    def write(raw, timeout):
        data=novo_decode(raw); tx=int.from_bytes(data[:4],'big')
        receiver.wire.append((tx,data[5:].decode()))
        if tx & 7 in (3,4):
            return len(raw)
        for index, reply in enumerate([b'']+answers):
            receiver.router._dispatch(NovoFrame(tx|0x400,len(reply),reply,b'',time.monotonic(),'pipette'))
            if index==1 and intermediate:
                intermediate(receiver.router._pipette_completions[0])
        return len(raw)
    receiver.router.ep_out.write=write

@pytest.mark.parametrize('terminal',[False,True])
def test_program_single_diagnostic_finite(receiver,rig,terminal):
    replies(receiver,[b'\x20\x60123']+([b'\x20\x60'] if terminal else []))
    command=program([D])
    assert command['ascii']==compile_command(D)['ascii']=='d0R'
    result=finite(rig,receiver,command)
    assert result['ok'] and result['completion_verified']
    reports=result['channels'][0]['result']['reports']
    assert len(reports)==1 and reports[0]['value']==123 and reports[0]['units']=='ma'
    receipt=json.loads(rig.store.connection.execute('SELECT receipt_json FROM pipette_operations WHERE command_id=?',(result['command_id'],)).fetchone()[0])
    assert receipt['result']['channels'][0]['result']['reports'][0]['value']==123
    export('program-diagnostic-'+str(terminal)+'.json',{'compiled':command,'result':result,'receipt':receipt,'wire':receiver.wire})

@pytest.mark.parametrize('second',[False,True])
def test_multiple_diagnostics_not_first_completion(receiver,second):
    seen=[]
    replies(receiver,[b'\x20\x60123']+([b'\x20\x60456'] if second else []),lambda owner: seen.append(owner.event.is_set()))
    result=send(receiver,program([D,D]))
    assert seen==[False]
    assert result['ok'] is second
    assert result['completion_verified'] is second
    assert [r['value'] for r in result['channels'][0]['result']['reports']]==([123,456] if second else [123])
    export('multiple-diagnostic-'+str(second)+'.json',result)


def test_diagnostic_followed_by_stroke_requires_terminal(receiver):
    replies(receiver,[b'\x20\x60123'])
    result=send(receiver,program([D,P]))
    assert not result['completion_verified']
    assert result['channels'][0]['result']['reports'][0]['value']==123


def test_split_append_diagnostic_execute(receiver):
    send(receiver,program([D],'append'))
    replies(receiver,[b'\x20\x60123',b'\x20\x60456'])
    result=send(receiver,program([D]))
    assert result['completion_verified']
    assert [r['value'] for r in result['channels'][0]['result']['reports']]==[123,456]


@pytest.mark.parametrize('answer',[False,True])
def test_append_then_explicit_execute_retains_answers_without_new_failure(receiver,answer):
    send(receiver,program([D],'append'))
    replies(receiver,[b'\x20\x60123'] if answer else [])
    result=send(receiver,compile_command({'command':'buffer_execute'}))
    assert result['ok'] and not result['completion_verified']
    reports=result['channels'][0]['result']['reports']
    assert [r['value'] for r in reports]==([123] if answer else [])
    assert receiver.wire[-1][1]=='R'


def test_diagnostics_halt_resume_replay_clear(receiver):
    replies(receiver,[b'\x20\x60123',b'\x20\x60'])
    result=send(receiver,program([D,H,D]))
    assert result['ok'] and not result['completion_verified']
    assert [r['command_index'] for r in result['channels'][0]['result']['reports']]==[0]
    replies(receiver,[b'\x20\x60456',b'\x20\x60'])
    result=send(receiver,compile_command({'command':'buffer_execute'}))
    assert result['ok'] and not result['completion_verified']
    assert result['channels'][0]['result']['reports'][0]['command_index']==2
    assert result['channels'][0]['result']['reports'][0]['value']==456
    replies(receiver,[b'\x20\x60123',b'\x20\x60'])
    result=send(receiver,compile_command({'command':'buffer_replay'}))
    assert result['channels'][0]['result']['reports'][0]['command_index']==0
    replies(receiver,[])
    send(receiver,compile_command({'command':'buffer_clear'}))
    result=send(receiver,compile_command({'command':'buffer_execute'}))
    assert result['ok'] and not result['completion_verified']
    assert not result['channels'][0]['result'].get('reports')
    export('diagnostic-halt-resume-clear.json',{'result':result,'wire':receiver.wire})


def test_loop_answers_retained_without_fabricated_occurrence(receiver):
    replies(receiver,[b'\x20\x60123',b'\x20\x60456',b'\x20\x60Pass',b'\x20\x60'])
    command=program([{'command':'loop_start'},D,{'command':'loop_end','repetitions':2},
                     {'command':'diagnostic','test':'step_loss_detection'}])
    assert not any(d['terminal_answer'] for d in command['response_descriptors'])
    result=send(receiver,command)
    assert result['ok'] and not result['completion_verified']
    reports=result['channels'][0]['result']['reports']
    assert [r['raw_ascii'] for r in reports]==['123','456','Pass']
    assert all(r['attribution_unknown'] and r['command_index'] is None for r in reports)


def test_export_current_device_contract():
    from bioxp.pipette.cavro_commands import device_command_catalog, device_command_schema, device_program_schema
    export('device-buffer-contract.json',{'catalog':device_command_catalog(),
        'command_schema':device_command_schema(),'program_schema':device_program_schema(),
        'diagnostic_program':program([D,D]),'halt_append':program([H],'append')})


def test_mixed_diagnostics_finite_decode(receiver,rig):
    replies(receiver,[b'\x20\x60123',b'\x20\x60Pass',b'\x20\x60Fail'])
    command=program([D,{'command':'diagnostic','test':'step_loss_detection'},
                     {'command':'diagnostic','test':'pressure_sensor'}])
    result=finite(rig,receiver,command)
    assert result['completion_verified']
    assert [r['value'] for r in result['channels'][0]['result']['reports']]==[123,'Pass','Fail']
    assert [r['command_index'] for r in result['channels'][0]['result']['reports']]==[0,1,2]
    export('mixed-diagnostics-finite.json',result)


def test_error_after_first_answer_keeps_partial(receiver):
    replies(receiver,[b'\x20\x60123',b'\x21\x60'])
    result=send(receiver,program([D,D]))
    assert not result['ok'] and not result['completion_verified']
    assert result['channels'][0]['result']['reports'][0]['value']==123


def test_unmodelled_legacy_command_invalidates_typed_context_only(receiver):
    send(receiver,program([H],'append'))
    driver=receiver.group._transports[0]._get_driver()
    result=driver._send_pipette_command('C',ack_mode='ack',wait_for_completion=False)
    assert result['ok']
    result=send(receiver,compile_command(P))
    assert result['ok'] and not result['completion_verified']
    assert result['channels'][0]['result']['buffer_context']['prior_phase']=='intervening_command_unknown'


def test_channels_do_not_share_halt(receiver):
    send(receiver,program([H],'append'))
    result=send(receiver,compile_command(P),[0,1])
    assert result['ok'] and not result['completion_verified']
    assert not result['channels'][0]['result']['completion_verified']
    assert result['channels'][1]['result']['completion_verified']


def test_diagnostic_partial_send_preserves_received_answer(receiver):
    receiver.fault['missing'] = 1
    with pytest.raises(Exception) as caught:
        send(receiver, program([D]), [0, 1])
    partial = caught.value.pipette_partial_result
    assert partial['failed_channel'] == 1
    assert partial['channels'][0]['result']['reports'][0]['value'] == 123
    assert not partial['channels'][0]['result']['completion_verified']
    export('diagnostic-partial-send.json', partial)


def test_control_transfer_remains_unknown_not_refusal(receiver):
    result=send(receiver,compile_command({'command':'stored_program_run','location':0}))
    assert result['ok'] and not result['completion_verified']
    result=send(receiver,compile_command(P))
    assert result['ok'] and not result['completion_verified']
    assert 'accounting' not in result['channels'][0]['result']


def test_interrupted_halt_context_and_channel_generation(receiver):
    send(receiver,program([H],'append'))
    receiver.group._interrupt_epoch += 1
    result=send(receiver,compile_command(P))
    assert result['ok'] and not result['completion_verified']
    receiver.router._reader_generation += 1
    result=send(receiver,compile_command(P))
    assert result['channels'][0]['result']['buffer_context']['prior_phase']=='unknown_preloaded'
    assert not result['channels'][0]['result']['buffer_context']['preloaded_contents_verified']


def test_interrupt_keeps_answers_not_completion(receiver):
    def interrupt(owner):
        receiver.group._interrupt_epoch += 1
    replies(receiver,[b'\x20\x60123',b'\x20\x60456'],interrupt)
    result=send(receiver,program([D,D]))
    assert not result['completion_verified'] and result['interrupted_by_terminate']
    assert [r['value'] for r in result['channels'][0]['result']['reports']]==[123,456]
