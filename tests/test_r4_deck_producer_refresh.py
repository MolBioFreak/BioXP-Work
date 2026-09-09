"""Real provider -> canonical SQLite tests. All stores are disposable tmp_path.
Constructor semantics must not masquerade as observed location, tray or reference.
"""
import pytest
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
from test_oem_deck_install_binding import FakeRuntimeStore, FakeDeckPrimitives, _operator_store
from test_r4_named_destination_vectors import rig

@pytest.fixture
def owners(tmp_path, rig):
    # These regressions exercise an incomplete retained predecessor, not a
    # newly constructed OEM object. Preserve unknown historical state explicitly.
    state = Serial206OemInitializationProvider._new_state()
    state['machine_status'].pop('construction_id', None)
    state['machine_status'].pop('constructed_tip_trays', None)
    state['machine_status'].update(current_location=None, current_well=None, tip_loaded=None, tip_dirty=None)
    state['x_lifecycle']['board_lifecycle_generation'] = 11
    runtime = FakeRuntimeStore(state)
    store = _operator_store(tmp_path)
    provider = Serial206OemInitializationProvider(FakeDeckPrimitives(), state_store=runtime,
                                                generation_provider=lambda: 7)
    provider.bind_deck_semantic_state_reader(store.deck_semantic_state)
    provider.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
    provider.bind_tip_tray_state_reader(store.tip_tray_state)
    provider.bind_tip_tray_state_publisher(store.publish_tip_tray_transition)
    yield provider, runtime, store
    store.stop()

def complete_predecessor(runtime, *, channel=0):
    # Explicit fixture evidence; NOT constructor or homing-derived location.
    runtime.state['machine_status'].update(
        current_location='LOC_MS', current_well=0, tip_loaded=True, tip_dirty=False,
        tip_location=channel, clean_path=False, latch_status=True, latch_closed=True,
        latch_observation_id='fixture-independent-latches')

@pytest.mark.parametrize('channel', [-1,0,1,2,3])
def test_late_same_provider_complete_predecessor_is_consumed(owners, channel):
    provider, runtime, store = owners
    with pytest.raises(RuntimeError, match='deck_bootstrap_semantic_location_unavailable'):
        provider._canonical_deck_semantic_state()
    assert provider.deck_semantic_bootstrap_diagnostic()['reason'] == 'deck_bootstrap_semantic_location_unavailable'
    assert store.deck_semantic_state()['semantic_state_revision'] == 0
    complete_predecessor(runtime, channel=channel)
    canonical = provider._canonical_deck_semantic_state()
    assert canonical['tip_loaded'] is True and canonical['tip_location'] == channel
    assert canonical['current_location'] == 'LOC_MS'
    assert provider.deck_semantic_bootstrap_diagnostic()['status'] == 'published'
    revision = canonical['semantic_state_revision']
    complete_predecessor(runtime, channel=(channel+1)%4)
    assert provider._canonical_deck_semantic_state()['semantic_state_revision'] == revision
    assert provider._canonical_deck_semantic_state()['tip_location'] == channel
    # Migration never constructs a tray or fabricates reference evidence.
    assert store.tip_tray_state(0)['tip_available'] is None
    with pytest.raises(RuntimeError, match='tray_0_tip_availability_unavailable'):
        provider._clean_path_from_tip_tray_authority(**provider.deck_owner_authority_stamps())

@pytest.mark.parametrize('channel', [-1,0,3])
def test_pipette_owner_loaded_group_survives_store_and_consumer(owners, channel):
    provider, runtime, store = owners
    complete_predecessor(runtime)
    provider._canonical_deck_semantic_state()
    provider.publish_pipette_owner_state(tip_loaded=True, tip_dirty=True, tip_location=channel,
                                        source_command_id=f'fixture-tip-owner-{channel}')
    row = provider._canonical_deck_semantic_state()
    assert row['tip_location'] == channel and row['tip_dirty'] is True
    assert store.deck_semantic_state()['producer_operation'] == 'pipette_owner'

def test_bootstrap_rechecks_owner_stamps_inside_durable_transaction(owners):
    provider, runtime, store = owners
    complete_predecessor(runtime)
    snapshot = provider.deck_semantic_bootstrap_snapshot(expected_generation=7)
    runtime.state['x_lifecycle']['board_lifecycle_generation'] = 12
    with pytest.raises(RuntimeError, match='deck_owner_authority_changed'):
        store.bootstrap_deck_semantic_state(snapshot)
    assert store.deck_semantic_state()['semantic_state_revision'] == 0

def test_no_tip_startup_return_cannot_invent_location_or_trays(owners):
    from bioxp.oem_serial206_initialization import SERIAL206_INITIALIZE_MOTION_STAGE_SPECS
    provider, runtime, store = owners
    state = runtime.state
    spec = next(s for s in SERIAL206_INITIALIZE_MOTION_STAGE_SPECS if s.key == 'initializeMotion.tip_loaded_false.no_tip')
    provider._apply_initialize_motion_transition(state, spec, {'ok':True})
    provider._save_state(state)
    result = provider._motion_result(state, ok=True, blockers=[])
    assert result['ok'] is True and result['physical_effect_verified'] is False
    assert result['deck_semantic_bootstrap'] == {'status':'blocked', 'reason':'deck_bootstrap_semantic_location_unavailable', 'error_type':'RuntimeError'}
    assert store.deck_semantic_state()['current_location'] is None
    assert store.tip_tray_state(0)['tip_available'] is None
    assert state['machine_status']['tip_dirty'] is None

@pytest.mark.parametrize('transition', ['construct','reset','camera_missing'])
def test_explicit_tray_owner_provenance_not_tip_query(owners, transition):
    provider, runtime, store = owners
    complete_predecessor(runtime)
    provider._canonical_deck_semantic_state()
    assert store.tip_tray_state(0)['tip_available'] is None
    if transition == 'camera_missing':
        provider.publish_tip_tray_transition(tray_id=0, transition='construct', operation_id='fixture-constructor',
            command_id='fixture-construct', provenance={'source':'ClassTipTray..ctor','kind':'host_semantic_default'})
    row = provider.publish_tip_tray_transition(tray_id=0, transition=transition,
        operation_id=f'fixture-{transition}', command_id=f'fixture-command-{transition}',
        well_ids=list(range(96)) if transition == 'camera_missing' else None,
        provenance={'source':'ClassTipTray', 'kind':'fixture-source-operation'})
    assert row['tip_available'] is (transition != 'camera_missing')
    assert provider._clean_path_from_tip_tray_authority(**provider.deck_owner_authority_stamps()) is (transition == 'camera_missing')
    runtime.state['x_lifecycle']['board_lifecycle_generation'] += 1
    with pytest.raises(RuntimeError, match='tray_0_tip_availability_unavailable'):
        provider._clean_path_from_tip_tray_authority(**provider.deck_owner_authority_stamps())
