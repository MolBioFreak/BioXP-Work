"""OEM return/setter semantics, independently of strict observation proof."""
import pytest
from tests.test_deck_tip_query_publication import query_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig


@pytest.mark.parametrize('reply', [[32, 96, 50], None, [31, 96, 49]])
@pytest.mark.parametrize('positive', [0, 3])
def test_source_returns_continue_before_and_after_invalid_proof(query_rig, reply, positive):
    from fastapi.testclient import TestClient
    app, provider, _, _, _, _, calls, wire, transport = query_rig
    wire['data'][1] = reply
    wire['data'][positive] = [32, 96, 49]
    response = TestClient(app).post('/liquid/tip-status', headers={'Idempotency-Key': 'source-order'})
    assert response.status_code == 502, response.text
    result = response.json()['detail']
    assert calls == [0, 1, 2, 3]
    assert result['source_return_completed'] is True and result['source_exception'] is False
    source_loaded = reply is not None and reply[2] == 49
    assert result['source_return'] == (2 if source_loaded else 1)
    assert result['source_tip_exists'] is True
    assert result['hardware_query_verified'] is False
    assert result['semantic_query_response_verified'] is False
    assert result['channels'][1]['tip_loaded'] is None
    assert result['channels'][1]['result']['source_tip_loaded'] is source_loaded
    assert transport._transports[1]._tip_loaded is source_loaded
    assert transport._transports[positive]._tip_loaded is True
    assert result['deck_state_publication']['status'] == 'published', result
    assert app.state.operator_command_plane.store.deck_semantic_state()['tip_loaded'] is True
    for flag in ('delivery_verified', 'controller_acknowledged', 'completion_verified', 'physical_effect_verified'):
        assert result.get(flag) is not True
        assert result['receipt_truth'][flag] is False


@pytest.mark.parametrize('asynchronous', [False, True])
@pytest.mark.parametrize('fault', ['short', 'throw'])
def test_actual_exception_preserves_prior_setters_and_stops(query_rig, fault, asynchronous):
    from bioxp.pipette.models import PipetteCommandError
    _, _, _, _, _, _, calls, wire, transport = query_rig
    wire['data'][0] = [32, 96, 49]
    wire['data'][1] = [32, 96]
    driver = transport._transports[1]._get_driver()
    driver._pipette_message_state = {'tip_loaded': True}
    def receive(channel):
        if channel == 1:
            if asynchronous:
                driver.process_pipette_message(3, [32, 96, 48], arbitration_id=1294,
                    command_name='query_tip_status')
            if fault == 'throw':
                raise RuntimeError('propagated outside OEM caught transmit region')
    wire['during'] = receive
    with pytest.raises(PipetteCommandError) as raised:
        transport.query_tip_status_all()
    assert calls == [0, 1]
    assert raised.value.details['source_return_completed'] is False
    assert raised.value.details['source_exception'] is True
    assert raised.value.details['observed_channels'][0]['tip_loaded'] is True
    assert transport._transports[0]._driver._pipette_message_state['tip_loaded'] is True
    assert driver._pipette_message_state['tip_loaded'] is (not asynchronous)


def test_actual_null_return_is_source_false_without_hardware_absence(monkeypatch):
    from bioxp.can_driver import BioXpCanDriver
    driver = BioXpCanDriver.__new__(BioXpCanDriver)
    driver._pipette_message_state = {'tip_loaded': True}
    monkeypatch.setattr(driver, '_send_pipette_command', lambda *a, **k: None)
    result = driver.query_tip_status()
    assert result['source_tip_loaded'] is False and result['source_return'] == 2
    assert result['source_return_completed'] is True
    assert driver._pipette_message_state['tip_loaded'] is False
    assert result['tip_loaded'] is None and result['semantic_ok'] is False
    assert result['hardware_truth_level'] == 'no_readback'
