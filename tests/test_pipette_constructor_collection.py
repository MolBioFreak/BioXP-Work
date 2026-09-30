"""Real constructor transport -> SQLite -> Park, with wire exchange only offline."""
import asyncio
import json
from types import SimpleNamespace

import pytest
from bioxp import api
from bioxp.pipette.models import PipetteInitCommand
from bioxp.services.pipette_service import run_pipette_init_command
from tests.test_deck_tip_query_publication import query_rig, named_move
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig
from tests.test_pipette_collection_owner import native_leaves
from tests.test_deck_tip_query_publication_contradiction import submit_named
from tests.protocol_v1_integration_fixture import integrated_rig


def constructor(rig, monkeypatch, key='startup'):
    """No tip query, seeded tip booleans, actors, revisions or verification."""
    transport = rig[8]
    wire = []
    from bioxp.novo_router import NovoRouter
    from bioxp.usb_driver import novo_decode
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=novo_decode)
    monkeypatch.setattr(transport, '_sleep', lambda _: None)
    for channel, leaf in enumerate(transport._transports):
        driver = leaf._get_driver()
        assert not getattr(driver, '_pipette_message_state', {}).get('tip_source_actor')
        monkeypatch.setattr(driver, 'bus', SimpleNamespace(router=router), raising=False)
        driver._pipette_completion_owner_token = None
        driver._sleep = lambda _: None
        def exchange(command, *, command_name, channel=channel, driver=driver, **kwargs):
            assert command in ('WR', 'b15R', 'o0,1R', 'o0,0R', '&1', 'Q1'), command
            wire.append((channel, command))
            driver._pipette_last_command = command_name
            data = [32, 96, 32] if command == 'Q1' else [32, 96, 49] if command == '&1' else []
            driver.process_pipette_message(len(data), data, command_name=command_name)
            return {'ok': True, 'tx_ok': True, 'delivery_verified': True,
                    'controller_acknowledged': True, 'ack': {
                        'received': True, 'outcome': 'ack', 'data': data}}
        def packet(board, data, *, command_name, channel=channel):
            assert board == 128 and data == [32 | channel] * 2
            wire.append((channel, 'wake'))
            return {'ok': True, 'tx_ok': True, 'ack': {'received': True}}
        def completion(channel, timeout, **kwargs):
            return {'ok': True, 'outcome': 'completion', 'data': [32, 96],
                    'observed_rx_dlc': 2, 'observed_rx_id': 0x501 + 8 * channel,
                    'command_name': 'pipette_initialize'}
        monkeypatch.setattr(driver, '_send_pipette_command', exchange)
        monkeypatch.setattr(driver, '_send_packet', packet)
        driver.bus.wait_pipette_completion = completion
    async def inline(_label, operation, **kwargs):
        return operation()
    result = asyncio.run(run_pipette_init_command(PipetteInitCommand(),
        get_transport=lambda: transport, run_blocking=inline, receipt_store=rig[5],
        runtime_binding={'entrypoint_id': 'lifecycle.constructor_pipette_stage',
            'caller_class': 'lifecycle', 'control_class': 'pipette_state_command',
            'lifecycle_stage_id': 'constructor_pipette_stage',
            'lifecycle_attempt_id': key, 'idempotency_key': 'constructor_pipette_stage:' + key}))
    assert result['ok'], result
    assert result['collection_source']['channels'] == [{'tip_loaded': False, 'verified': False}] * 4
    assert all(c['actor'] is None and c['revision'] is None
               for c in result['collection_source']['identity']['channels'])
    assert result['receipt_truth']['completion_verified'] is True
    assert result['receipt_truth']['physical_effect_verified'] is False
    assert rig[6] == []
    row = rig[5].connection.execute('SELECT status,receipt_json FROM pipette_operations WHERE command_id=?',
                                  (result['command_id'],)).fetchone()
    assert row['status'] == 'completed'
    assert json.loads(row['receipt_json'])['result']['collection_source'] == result['collection_source']
    return result, wire


def test_constructor_no_tip_query_reaches_queued_park(query_rig, monkeypatch):
    rig = query_rig
    native_leaves(rig, monkeypatch)
    result, wire = constructor(rig, monkeypatch)
    current = api._pipette_collection_state()
    assert current['tip_exists'] is False
    assert current['hardware_tip_exists'] is None
    before = list(wire)
    named_move(rig)
    authority = rig[1].deck_authority_snapshot(expected_generation=rig[1].generation_provider(), target='LOC_PARK')
    assert authority['dependency_scope'] == 'full'
    assert authority['collection_tip_state']['tip_exists'] == current['tip_exists']
    assert authority['collection_tip_state']['channels'] == result['collection_source']['channels']
    submit_named(rig, 'LOC_PARK', 'constructor-park')
    assert rig[0].state.operator_command_plane.store.deck_semantic_state()['current_location'] == 'LOC_PARK'
    assert wire == before and rig[6] == []
    assert api._pipette_collection_state() == current


@pytest.mark.parametrize('channel', range(4))
@pytest.mark.parametrize('loaded', [False, True])
def test_initialization_preserves_current_oem_boolean(query_rig, monkeypatch, loaded, channel):
    rig = query_rig
    constructor(rig, monkeypatch)
    driver = rig[8]._transports[channel]._driver
    # A literal OEM setter without correlated physical proof still changes TipLoaded.
    driver.process_pipette_message(3, [32, 96, 49 if loaded else 48], command_name='query_tip_status')
    from bioxp.pipette.receipts import PipetteReceiptError
    with pytest.raises(PipetteReceiptError):
        api._pipette_collection_state()  # cannot substitute superseded constructor receipt
    driver.pipette_initialize()
    driver.wait_pipette_initialization_completion(1)
    driver.query_status()
    assert driver._pipette_message_state['tip_loaded'] is loaded
    published = rig[5].publish_collection_source(rig[8], ownership_generation=rig[1].generation_provider())
    assert published['tip_exists'] is loaded
    assert published['hardware_tip_exists'] is None
    assert api._pipette_collection_state() == published


@pytest.mark.parametrize('fault', ['owner', 'reader', 'stop'])
def test_constructor_still_fences_changed_current_execution(query_rig, monkeypatch, fault):
    from bioxp.pipette.receipts import PipetteReceiptError
    rig = query_rig
    constructor(rig, monkeypatch)
    before = api._pipette_collection_state()
    if fault == 'owner':
        rig[8]._collection_source_owner = 'changed-current-owner'
    elif fault == 'reader':
        rig[8]._transports[0]._driver.bus.router = SimpleNamespace(reader_generation=1)
    else:
        rig[8]._interrupt_epoch += 1
    with pytest.raises(PipetteReceiptError):
        api._pipette_collection_state()
    assert before['tip_exists'] is False and before['hardware_tip_exists'] is None


@pytest.mark.parametrize('integrated_rig', ['constructor-no-tip-query'], indirect=True)
@pytest.mark.filterwarnings('error::pytest.PytestUnhandledThreadExceptionWarning')
def test_constructor_no_tip_query_full_native_cover_roundtrip(integrated_rig, query_rig, monkeypatch):
    # Same real native document/SQLite checks as retained-release regression,
    # but fixture startup is constructor-only, never the verified query fixture.
    from tests.test_pipette_retained_collection_execution import test_new_runtime_native_cover_document_keeps_retained_collection
    assert api._pipette_collection_state()['hardware_tip_exists'] is None
    test_new_runtime_native_cover_document_keeps_retained_collection(integrated_rig, query_rig, monkeypatch)
