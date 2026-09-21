"""Native constructor-free Home provenance at its current authority consumer."""
import copy
import pytest
from tests.test_wake_setup_debloat import rig
from tests.test_oem_cold_startup_contract import startup, run
from bioxp.serial206_y_provider import Serial206YProvider


@pytest.fixture
def initialized(rig, monkeypatch):
    driver, provider, frames, fault, root = rig
    provider.y_provider = Serial206YProvider(driver, state_store=provider.state_store,
        reference_store=provider.reference_store, generation_provider=provider.generation_provider)
    startup(monkeypatch, driver)
    assert run()['ok'] is True
    result = provider.initialize_motors(mode='live')
    assert all(p['published'] for p in result['reference_publications'].values())
    authority = provider.state_store.board4_authority_projection()
    assert authority['axes']['y']['prepared_board_epoch'] is None
    assert provider._native_y_reference_current(authority['board'], authority['axes']['y'])
    assert provider._xy_authority_snapshot(provider._load_state()['x_lifecycle'])['ok']
    return provider, frames


@pytest.mark.parametrize('fault', ['owner', 'y_owner', 'board', 'generation', 'reference',
    'interrupt', 'software_interrupt', 'active_interrupt', 'pending', 'missing', 'partial',
    'unpublished', 'home_proof', 'zero_ack', 'zero_readback', 'receipt'])
def test_native_home_current_authority_rejects_drift(initialized, monkeypatch, fault):
    provider, frames = initialized
    authority = copy.deepcopy(provider.state_store.board4_authority_projection())
    board, axis = authority['board'], authority['axes']['y']
    state = copy.deepcopy(provider._load_state())
    publication = state['movement_ledger']['stages']['y-set-home']['result']['reference_publication']
    if fault == 'owner':
        monkeypatch.setattr(provider, '_home_recovery_owner_id', 'replacement')
    elif fault == 'y_owner':
        monkeypatch.setattr(provider, 'y_provider', object())
    elif fault == 'board':
        board['active_board_epoch'] += 1
        monkeypatch.setattr(provider.state_store, 'board4_authority_projection', lambda: authority)
    elif fault == 'generation':
        axis['ownership_generation'] += 1
    elif fault == 'reference':
        from bioxp.services.reference_service import MarkAxisReferencedCommand
        provider.reference_store.mark_referenced(MarkAxisReferencedCommand(axis='y', position_steps=0, source='other'))
    elif fault in {'interrupt', 'software_interrupt'}:
        axis[fault+'_epoch'] += 1
        monkeypatch.setattr(provider.state_store, 'board4_authority_projection', lambda: authority)
    elif fault == 'active_interrupt':
        axis['software_interrupt_active'] = True
    elif fault == 'pending':
        axis['pending_ticket'] = {'command_id': 'unsettled'}
    elif fault == 'receipt':
        axis['last_receipt_id'] = 'different'
    else:
        if fault == 'missing':
            publication.clear()
        elif fault == 'partial':
            publication['fence'].pop('provider_owner')
        elif fault == 'unpublished':
            publication['published'] = False
        elif fault == 'home_proof':
            publication['controller_home_evidence'].pop('controller_terminal_state_verified')
        elif fault == 'zero_ack':
            publication['zero_write_ack']['status'] = 1
        elif fault == 'zero_readback':
            publication['controller_position_observation']['result']['position'] = 9
        monkeypatch.setattr(provider, '_load_state', lambda: state)
    before = list(frames)
    assert not provider._native_y_reference_current(board, axis)
    assert frames == before  # authority consumption never inserts physical queries


@pytest.mark.parametrize('fault', ['missing_z', 'partial_g', 'different_run', 'unfinished',
    'missing_home_fence', 'receipt_owner', 'receipt_board', 'receipt_interrupt', 'receipt_reference'])
def test_native_initialization_requires_complete_same_invocation(initialized, monkeypatch, fault):
    provider, frames = initialized
    state = copy.deepcopy(provider._load_state())
    ledger = state['movement_ledger']
    stages = ledger['stages']
    if fault == 'missing_z':
        stages['z-home']['result'].pop('recovery_home')
    elif fault == 'partial_g':
        stages['gripper-home']['result']['reference_publication'].pop('controller_home_evidence')
    elif fault == 'different_run':
        stages['x-home']['command_id'] = 'another-run:x-home'
    elif fault == 'unfinished':
        ledger['terminal_state'] = 'running'
    elif fault == 'missing_home_fence':
        stages['y-set-home']['result']['reference_publication'].pop('fence')
    else:
        field = {'receipt_owner': 'owner_id', 'receipt_board': 'board_epoch_4',
            'receipt_interrupt': 'interrupt_epoch', 'receipt_reference': 'reference_version'}[fault]
        stages['z-home']['result']['recovery_home'].pop(field)
    before = list(frames)
    assert provider._native_initialized_home_receipts(state) == {}
    assert frames == before
