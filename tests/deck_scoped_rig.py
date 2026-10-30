"""Source-owned deck regressions. All controller primitives are offline doubles."""
import copy
from types import SimpleNamespace

import pytest

from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
from bioxp.oem_deck_movement import DeckAuthoritySnapshot
from bioxp.operator_controls import _deck_authority_diagnostic
from bioxp.operator_command_plane import OperatorCommandStore
from tests.oem_machine_bundle_test_support import bind_serial206_oem_snapshot


class Primitive:
    def __init__(self, home=False, position=50):
        self.home, self.position, self.calls = home, position, []
        self.tester = self
    def motor_query_home_switch(self, board, motor):
        self.calls.append(('home', board, motor))
        return {'ok': True, 'reply_valid': True, 'home': self.home}
    def motor_get_position(self, board, motor):
        self.calls.append(('position', board, motor))
        return {'ok': True, 'position': self.position, 'ack': {'status': 100, 'value': self.position}}
    def _read_axis_position(self, axis):
        self.calls.append(('xyz', axis))
        return 0
    def query_latch(self):
        self.calls.append(('latch',))
        return {'ok': True, 'value': 1}
    def read_oem_latch_status(self):
        return {'ok': True, 'value': True, 'observation_id': 'host-1'}
    def oem_move_to(self, *args, **kwargs):
        self.calls.append(('move', args, kwargs))
        return {'ok': True, 'controller_command_acknowledged': True, 'controller_completion_verified': True}


@pytest.fixture
def rig(monkeypatch):
    bind_serial206_oem_snapshot(monkeypatch)
    primitive = Primitive(home=True)
    provider = Serial206OemInitializationProvider(primitive, generation_provider=lambda: 3)
    # Explicit retained null predecessor, not a call to the constructor.
    state = {'schema_version': provider.schema, 'machine_status': {'current_location': None,
             'current_well': None, 'tip_loaded': False, 'tip_dirty': None,
             'tip_location': None, 'psudo_z_home_steps': 65000},
             'x_lifecycle': {'board_lifecycle_generation': 9}}
    monkeypatch.setattr(provider, '_load_state', lambda: copy.deepcopy(state))
    monkeypatch.setattr(provider, '_save_state', lambda value: state.update(copy.deepcopy(value)) or value)
    provider.state_store = SimpleNamespace(board4_authority_projection=lambda: {'board': {'active_board_epoch': 7, 'state': 'active'}})
    provider.reference_store = SimpleNamespace(snapshot=lambda axes: {'rows': {a: {'state': 'referenced', 'state_version': 1} for a in axes}})
    semantic = {'semantic_state_revision': 0, 'ambiguity_state': 'none', 'transition_provenance': {},
                'current_location': None, 'current_well': None, 'tip_loaded': None,
                'tip_dirty': None, 'tip_location': None, 'clean_path': None,
                'plate_on_gantry': None, 'pseudo_z_home': 65000,
                'ownership_generation': None, 'board_epoch_4': None, 'board_epoch_5': None}
    provider.bind_deck_semantic_state_reader(lambda: copy.deepcopy(semantic))
    return provider, primitive, semantic, state


















@pytest.fixture
def retained_rig(monkeypatch, tmp_path):
    import os, shutil
    from pathlib import Path
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.services.reference_service import ReferenceStateStore
    bind_serial206_oem_snapshot(monkeypatch)
    baseline = os.environ.get('DECK_RETAINED_BASELINE')
    if not baseline:
        pytest.skip('optional captured retained-state qualification requires DECK_RETAINED_BASELINE')
    root = tmp_path / 'retained-copy'
    shutil.copytree(Path(baseline), root)
    runtime = OEMRuntimeStore(root)
    references = ReferenceStateStore(root / 'bioxp_runtime.db')
    primitive = Primitive(home=True)
    provider = Serial206OemInitializationProvider(primitive, state_store=runtime,
        reference_store=references, generation_provider=lambda: 3)
    store = OperatorCommandStore(root)
    provider.bind_deck_semantic_state_reader(store.deck_semantic_state)
    provider.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
    yield provider, primitive, runtime, references, store, root
    store.stop()
    runtime.close()


def qualify_test_references(references):
    from bioxp.services.reference_service import MarkAxisReferencedCommand
    # Explicit isolated evidence owner event, never source-store or hardware mutation.
    result = references.mark_referenced_many([MarkAxisReferencedCommand(a, 0,
        source='isolated_controller_fixture', note='No physical acceptance') for a in ('x','y','z','g')])
    assert result['ok'] is True




