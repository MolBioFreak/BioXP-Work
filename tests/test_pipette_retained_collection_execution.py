"""Release metadata cannot replace the OEM's live collection predicates."""
import copy
import pytest
from bioxp import api
from bioxp.pipette import receipts
from tests.test_deck_tip_query_publication import query_rig, query
from tests.protocol_v1_integration_fixture import integrated_rig
from tests.test_deck_scoped_integration import installed_retained
from tests.test_deck_scoped_authority import retained_rig
from tests.test_deck_tip_query_publication_contradiction import warm_no_tip, submit_named


def history(rig):
    return {
        'operations': [tuple(row) for row in rig[5].connection.execute(
            'SELECT * FROM pipette_operations ORDER BY rowid')],
        'collection_events': [tuple(row) for row in rig[5].connection.execute(
            "SELECT * FROM runtime_events WHERE event_source='pipette_collection_owner' ORDER BY event_id")],
    }


@pytest.mark.parametrize('reopen', [False, True])
def test_old_release_collection_reaches_queued_park_without_repair(query_rig, monkeypatch, reopen):
    rig = query_rig
    from tests.test_pipette_collection_owner import native_leaves
    native_leaves(rig, monkeypatch)
    warm_no_tip(rig)  # Actual old-runtime claim and committed query, not SQL replacement.
    before = history(rig)
    old = api._pipette_collection_state()
    calls = list(rig[6])
    release = copy.deepcopy(receipts.current_release_identity())
    release['release_id'] = 'next-runtime'
    release['source'] = {'manifest_sha256': 'a'*64, 'aggregate_sha256': 'b'*64}
    monkeypatch.setattr(receipts, 'current_release_identity', lambda: release)
    if reopen:
        monkeypatch.setattr(api, '_pipette_receipts', receipts.PipetteReceiptStore(rig[4]))
    # Exact reachable path: API -> collection store -> provider full Park
    # authority -> real queued operator dispatch and SQLite terminal commit.
    assert api._pipette_collection_state() == old
    authority = rig[1].deck_authority_snapshot(
        expected_generation=rig[1].generation_provider(), target='LOC_PARK')
    assert authority['dependency_scope'] == 'full'
    assert authority['collection_tip_state'] == old
    submit_named(rig, 'LOC_PARK', 'retained-collection-park')
    assert rig[0].state.operator_command_plane.store.deck_semantic_state()['current_location'] == 'LOC_PARK'
    assert rig[6] == calls
    assert history(rig) == before
    claim = rig[5].connection.execute(
        'SELECT pipette_operation_id FROM pipette_operations WHERE command_id=?',
        (old['command_id'],)).fetchone()
    with pytest.raises(receipts.PipetteReceiptError, match='pipette replay release_id is stale'):
        rig[5].replay_result(command_id=old['command_id'], pipette_operation_id=claim[0])


@pytest.mark.parametrize('fault', ['owner', 'reader', 'generation', 'stop'])
def test_retained_read_does_not_authorize_invalid_current_publication(query_rig, fault):
    rig = query_rig
    warm_no_tip(rig)
    old = api._pipette_collection_state()
    before = history(rig)
    generation = rig[1].generation_provider()
    if fault == 'owner': rig[8]._collection_source_owner = 'another-owner'
    if fault == 'reader': rig[8]._transports[0]._driver.bus.router.reader_generation += 1
    if fault == 'generation': generation += 1
    if fault == 'stop': rig[8]._interrupt_epoch += 1
    with pytest.raises(receipts.PipetteReceiptError):
        rig[5].publish_collection_source(rig[8], ownership_generation=generation)
    with pytest.raises(receipts.PipetteReceiptError):
        rig[5].collection_state(identity=rig[8].collection_source_identity(),
            ownership_generation=generation)
    assert history(rig) == before


@pytest.mark.filterwarnings("error::pytest.PytestUnhandledThreadExceptionWarning")
def test_new_runtime_native_cover_document_keeps_retained_collection(integrated_rig, query_rig, monkeypatch):
    rig = integrated_rig
    # The reused manual-button leaf asserts STA; native covers use MTA WaitAll.
    # Replace only that transport wait, with the same per-axis wire events.
    from tests.test_deck_near_terminal import NearUSB
    def wait_many(leaf, axes, **kwargs):
        return {'ok': True, 'per_axis': {
            axis: leaf.motor_wait_target_reached(board)
            for axis, board in (('x', 5), ('y', 4))}}
    monkeypatch.setattr(NearUSB, 'motor_wait_target_reached_many', wait_many)
    rig.native.tester.motor_wait_target_reached_many = wait_many.__get__(
        rig.native.tester.motor_wait_target_reached.__self__)
    before = history(query_rig)
    old = api._pipette_collection_state()
    release = copy.deepcopy(receipts.current_release_identity())
    release['release_id'] = 'native-next-runtime'
    monkeypatch.setattr(receipts, 'current_release_identity', lambda: release)
    payload = {'source_type': 'native', 'dry_run': False, 'idempotency_key': 'retained-native',
        'live_execution': {'live_execution_ack': True}, 'document': {
        'protocol_id': 'retained-native', 'stages': [{'stage_id': 'moves', 'actions': [
            {'action_id': 'output-out', 'kind': 'move_cover',
             'params': {'cover_id': 'CV_OUTPUT', 'target_location': 'LOC_OCS'}},
            {'action_id': 'output-back', 'kind': 'move_cover',
             'params': {'cover_id': 'CV_OUTPUT', 'target_location': 'LOC_OC'}},
        ]}]}}
    job = rig.start(payload)
    terminal = rig.terminal(job)
    import json
    assert terminal['command']['status'] == 'completed', json.dumps(terminal)
    # These are nested source tasks, not persisted root-plan background
    # children. Their real workers still run and retain actual outcomes.
    tasks = list(rig.provider._wp8_tasks.values())
    assert len(tasks) == 4
    for task in tasks:
        task['thread'].join(3)
        assert not task['thread'].is_alive()
        assert task['state'] == 'completed', task
    assert len(rig.children(job)) >= 2
    assert rig.store.deck_semantic_state()['movable_plate_locations']['OUTPUT_COVER'] == 'LOC_OC_COVER'
    assert api._pipette_collection_state() == old
    assert history(query_rig) == before
    assert query_rig[6] == []


@pytest.mark.parametrize('wire,expected', [
    ([[32, 96, 49], [32, 96, 50], [32, 96, 48], [32, 96, 48]], True),
    ([[32, 96, 48], [32, 96, 50], [32, 96, 48], [32, 96, 48]], None),
])
def test_retained_partial_channels_are_not_replaced_by_machine_tiploaded(query_rig, monkeypatch, wire, expected):
    from tests.test_deck_tip_query_publication_contradiction import invoke
    rig = query_rig
    warm_no_tip(rig)
    rig[7]['data'] = wire
    assert invoke(rig, 'partial-retained').status_code == 502
    before = history(rig)
    monkeypatch.setattr(receipts, '_current_replay_identity', lambda: (_ for _ in ()).throw(
        AssertionError('historical reads must not resolve release authority')))
    current = api._pipette_collection_state()
    assert current['tip_exists'] is expected
    assert history(rig) == before


def test_retained_event_and_current_publication_keep_durable_order(query_rig, monkeypatch):
    from tests.test_pipette_collection_owner import async_set
    rig = query_rig
    warm_no_tip(rig)
    old = api._pipette_collection_state()
    async_set(rig, 0, 49)
    # Neither an uncommitted new value nor the superseded false is authority.
    with pytest.raises(receipts.PipetteReceiptError):
        api._pipette_collection_state()
    published = rig[5].publish_collection_source(rig[8],
        ownership_generation=rig[1].generation_provider())
    assert published['tip_exists'] is True and published['event_id']
    assert api._pipette_collection_state() == published
    # A new genuine claim supersedes the previous command's event, not vice versa.
    query(rig, key='next-owner-query')
    latest = api._pipette_collection_state()
    assert latest['tip_exists'] is False and latest['event_id'] is None
    assert latest['command_id'] != published['command_id']
