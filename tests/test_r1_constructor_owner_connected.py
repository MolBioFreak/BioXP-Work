"""Real constructor lifecycle/service/SQLite; inert driver commands only."""
from types import SimpleNamespace
import pytest
from tests.test_deck_tip_query_publication import query_rig, query
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig


def prepare_constructor(rig, monkeypatch):
    from bioxp import api
    from bioxp.lifecycle_state import CanonicalLifecycleOwner
    lifecycle = CanonicalLifecycleOwner()
    lifecycle.transport_changed(True, reason='isolated connected transport')
    monkeypatch.setattr(api, 'lifecycle_state', lifecycle)
    from bioxp.novo_router import NovoRouter
    from bioxp.usb_driver import novo_decode
    group = rig[-1]
    group._sleep = lambda _: None
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=novo_decode)
    for channel, transport in enumerate(group._transports):
        driver = transport._get_driver()
        driver.bus = SimpleNamespace(router=router)
        driver._pipette_completion_owner_token = None
        driver._sleep = lambda _: None
        original_exchange = driver._send_pipette_command
        def exchange(command, *, command_name, driver=driver, original=original_exchange, **kwargs):
            if command == '?31':
                return original(command, command_name=command_name, **kwargs)
            assert command in ('WR', 'b15R', 'o0,1R', 'o0,0R', '&1', 'Q1')
            driver._pipette_last_command = command_name
            data = [32, 96, 32] if command == 'Q1' else [32, 96, 49] if command == '&1' else []
            driver.process_pipette_message(len(data), data, command_name=command_name)
            return dict(ok=True, tx_ok=True, delivery_verified=True,
                controller_acknowledged=True, ack={'received': True, 'outcome': 'ack', 'data': data})
        def packet(board, data, *, command_name, channel=channel):
            assert board == 128 and data == [32 | channel] * 2
            return {'ok': True, 'tx_ok': True, 'ack': {'received': True}}
        def completion(channel, timeout, **kwargs):
            return {'ok': True, 'outcome': 'completion', 'data': [32, 96],
                'observed_rx_dlc': 2, 'observed_rx_id': 0x501 + 8 * channel,
                'command_name': 'pipette_initialize'}
        monkeypatch.setattr(driver, '_send_pipette_command', exchange)
        monkeypatch.setattr(driver, '_send_packet', packet)
        driver.bus.wait_pipette_completion = completion
    return lifecycle


@pytest.mark.parametrize('event', ['restart', 'reconnect'])
def test_park_constructor_reads_current_owner_not_receipt_history(query_rig, monkeypatch, event):
    from bioxp import api
    from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport
    lifecycle = prepare_constructor(query_rig, monkeypatch)
    app, provider, primitive, references, root, receipts, calls, wire, owner = query_rig
    if event == 'reconnect':
        # New collection object on the same service, with a new owner identity.
        old_identity = owner.collection_source_identity()
        owner = FourPipetteTransport([CanPipetteTransport(driver_factory=t._driver_factory,
            pipette_id=i) for i, t in enumerate(owner._transports)], sleep=lambda _: None)
        monkeypatch.setattr(api, '_pipette_transport', owner)
        monkeypatch.setattr(api, '_get_pipette_transport', lambda: owner)
        assert owner.collection_source_identity() != old_identity
    assert not getattr(owner, '_constructor_started', False)
    state = provider._park_collection_state()
    assert owner._constructor_started is True
    assert lifecycle.projection()['startup']['stages']['constructor_pipette_stage']['state'] == 'passed'
    assert calls == []  # no extra tip query added to Park
    count = receipts.connection.execute('SELECT count(*) FROM pipette_operations').fetchone()[0]
    assert count > 0
    assert provider._park_collection_state() == state
    assert receipts.connection.execute('SELECT count(*) FROM pipette_operations').fetchone()[0] == count
    result = query(query_rig, key='current-owner-' + event)
    assert result['ok'] is True
    assert calls == [0, 1, 2, 3]
    current = provider._park_collection_state()
    assert current['tip_exists'] is False
    assert [c['tip_loaded'] for c in current['channels']] == [False] * 4
    wire['data'][1] = [32, 96, 49]
    query(query_rig, key='tip-transition-' + event)
    assert provider._park_collection_state()['tip_exists'] is True
