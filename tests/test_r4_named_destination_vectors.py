"""OEM CI btnLOC1_Click IL_065e..076f; serial206 config 33aadf87...475.
Independent panel inventory and raw XML XY, not copied provider output.
The real provider and production oem_move_to lowering run; only axis I/O is fake.
"""
from dataclasses import asdict, replace

import pytest
from bioxp import oem_serial206_initialization as module
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider
from bioxp.oem_compat.position_table import PositionTable
from bioxp.oem_deck_catalog import DeckCatalog
from bioxp.oem_deck_movement import NamedLocationIntent, compile_named_location
from test_oem_named_location_command_plane import _authority
from test_oem_deck_install_binding import _ProductionMoveReceiptAdapter

# target, ordinal, XML x, XML y; CI:1822-1924 exact panel inventory.
ROWS = [
    ('LOC_MS',0,26213,9241), ('LOC_OC',1,26213,42413),
    ('TECANRACK2',8,25029,71755), ('TECANRACK4',10,25029,95449),
    ('LOC_P_MS',25,1125,8779), ('LOC_P_OC',21,1125,41950),
    ('LOC_OC_COVER',17,1324,42129), ('LOC_TC',2,67606,9241),
    ('LOC_TC_BARCODE',2,67606,9241), ('LOC_RC',3,67677,45256),
    ('LOC_RC_BARCODE',3,67677,45256), ('TECANRACK1',7,60571,71755),
    ('TECANRACK3',9,60571,95449), ('LOC_P_TC',23,42518,8779),
    ('LOC_BSC',5,42518,8779), ('LOC_RC_COVER',19,42788,44972),
    ('LOC_STRIP1',11,75022,83403), ('LOC_STRIP2',12,72890,83403),
    ('LOC_STRIP3',13,70758,83403), ('LOC_STRIP4',14,68625,83403),
    ('LOC_OC_COVER_STORAGE',18,84252,6057), ('LOC_RC_COVER_STORAGE',20,84252,36267),
    ('LOC_TROUGH',16,85555,63278), ('LOC_BSCS',4,82868,61772),
    ('WASTE_BIN',6,92049,93211), ('LOC_PARK',28,1506,71),
]
CASES = [(r, offset) for r in ROWS for offset in
         ([False] if r[0].endswith('BARCODE') or r[0] == 'LOC_PARK' else [False, True])]

@pytest.fixture
def rig(monkeypatch):
    table = PositionTable.from_rows([
        dict(name=n, x=x, y=y, zLow=60000, zDelta=10000, inc_factor=0)
        for n, _, x, y in ROWS if not n.endswith('BARCODE')
    ] + [dict(name='CAMERA_OFFSET', x=3499, y=-7744, zLow=3145, zDelta=6842, inc_factor=0)])
    monkeypatch.setattr(module, 'load_bound_oem_position_table', lambda: table)
    adapter = _ProductionMoveReceiptAdapter()
    provider = Serial206OemInitializationProvider(adapter, generation_provider=lambda: 4)
    authority = replace(_authority(table), pseudo_z_home=500,
                        semantic_state_provenance_digest='a'*64)
    return table, provider, adapter, authority

@pytest.mark.parametrize('row,offset', CASES, ids=[f'{r[0]}-camera={o}' for r,o in CASES])
def test_panel_vectors_through_real_provider(rig, row, offset):
    table, provider, adapter, authority = rig
    name, ordinal, x, y = row
    catalog = DeckCatalog.from_position_table(table)
    assert {r['target'] for r in catalog.rows()} == {r[0] for r in ROWS}
    assert catalog.resolve(name).location_id == ordinal
    if name == 'LOC_PARK':
        # Source early-return branch must not fabricate a motion/physical proof.
        authority = replace(authority, current_location_id='LOC_PARK')
    plan = compile_named_location(NamedLocationIntent(name, offset), catalog, table, authority)
    results = [getattr(provider, s.operation)(**dict(s.arguments or {}), authority_snapshot=asdict(authority))
               for s in plan.steps[3:]]
    if name == 'LOC_PARK':
        assert adapter.motion_calls == []
        assert results[0]['source_noop'] is True
        return
    if name.endswith('BARCODE'):
        x += {2:-11847,3:-23930}[ordinal] + 3499
        y += 7582 - 7744
    elif offset:
        x += 3499
        y -= 7744
    assert adapter.motion_calls[0] == ('moveTo', min(x,90213), min(y,102906))
    if name.endswith('BARCODE'):
        assert adapter.motion_calls[1:] == [('moveZ', {2:1794,3:3145}[ordinal],31,True)]
    else:
        assert len(adapter.motion_calls) == 1
    assert all(r['ok'] for r in results)

@pytest.mark.parametrize('channel', [-1,0,1,2,3])
def test_loaded_group_and_single_channel_authority(rig, channel):
    _, _, _, authority = rig
    assert replace(authority, tip_loaded=True, tip_location=channel).tip_location == channel

@pytest.mark.parametrize('channel', [-2,4,True,None])
def test_invalid_tip_domain_stays_closed(rig, channel):
    with pytest.raises(ValueError):
        replace(rig[3], tip_loaded=True, tip_location=channel)
