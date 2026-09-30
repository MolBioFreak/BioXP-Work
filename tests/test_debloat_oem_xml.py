"""Retained generator regressions; offline, never hardware or inferred science."""
from pathlib import Path
from collections import Counter
import hashlib
import pytest

from bioxp.protocols.oem_xml_expand import expand_oem_xml_protocol, system_check_protocol, UnexpandedOemGenerator
from bioxp.protocols.oem_xml_import import import_oem_xml_protocol
from bioxp.protocols.models import OEM_SOURCE_DEFAULT_NOOPS
from bioxp.protocols.executor import SOURCE_DOMAINS

ROOT = Path(__file__).resolve().parents[1]
SETTINGS = {"MotionOnly": False, "StartMode": 0, "DeckInspection": False,
            "CheckSnapTips": False, "LogPressure": False, "JobName": None,
            "CameraInstalled": False, "CameraCalibrated": False}


def actions(doc):
    return [a for s in doc.stages for a in s.actions]


def generated(tmp_path, commands, settings=None):
    path = tmp_path / 'source.xml'
    path.write_text('<WpfGenBotCommonLib><experiment><data name="test"/></experiment><script>' +
                    ''.join(f'<line{i} cmd="{cmd}"/>' for i, cmd in enumerate(commands, 1)) +
                    '</script></WpfGenBotCommonLib>')
    return expand_oem_xml_protocol(path, source_settings=settings or SETTINGS)


def test_system_check_single_xml_definition_and_children():
    doc = system_check_protocol(source_settings=SETTINGS)
    source = ROOT / 'scripts/Inital Test Script.xml'
    assert doc.metadata['xml_source_sha256'] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert doc.metadata['imported_command_count'] == 30
    counts = Counter(a.oem_opcode for a in actions(doc))
    assert counts == {'iniPipette': 3, 'step': 4, 'led': 4, 'wait': 3, 'cc': 2,
                      'pressp': 4, '//': 16, 'dopen': 2, 'dclose': 2,
                      'SS': 1, 'catchPlate': 12, 'releasePlate': 12}
    assert actions(doc)[0].oem_opcode == 'iniPipette'
    releases = [a.params['arguments'] for a in actions(doc) if a.oem_opcode == 'releasePlate']
    assert releases == [('LOC_BSCS',), ('LOC_BSC',), ('LOC_OC_COVER',), ('LOC_RC_COVER',),
                        ('LOC_OC_COVER_STORAGE',), ('LOC_RC_COVER_STORAGE',), ('LOC_BSCS',),
                        ('LOC_P_MS','pressplate'), ('LOC_P_TC','pressplate'),
                        ('LOC_P_MS','pressplate'), ('LOC_P_TC','pressplate'), ('LOC_BSC',)]
    assert len({a.source_occurrence_id for a in actions(doc)}) == len(actions(doc))
    assert [a.source_key for a in actions(doc)] == list(range(10, (len(actions(doc))+1)*10, 10))
    assert not doc.metadata.get('requires_generator_expansion')
    assert import_oem_xml_protocol(source).document.metadata['requires_generator_expansion'] is True


@pytest.mark.parametrize('name', ['Inital Test Script.xml', 'demo.xml', 'tp506.xml'])
def test_retained_mechanical_thermal_scripts_expand(name):
    doc = expand_oem_xml_protocol(ROOT / 'scripts' / name, source_settings=SETTINGS)
    assert doc.metadata['input_mode'] == 'oem_prepared'
    assert all(a.oem_opcode in SOURCE_DOMAINS for a in actions(doc))


@pytest.mark.parametrize('name,verbs', [('lifetest.xml', {'MT','FP','LA','SA'}),
                                     ('TP015 48 HOUR SYSTEM BURN-IN.xml', {'MT'})])
def test_retained_liquid_gap_is_explicit_no_executable_prefix(name, verbs):
    with pytest.raises(UnexpandedOemGenerator) as error:
        expand_oem_xml_protocol(ROOT / 'scripts' / name, source_settings=SETTINGS)
    assert {c['verb'] for c in error.value.commands} == verbs


def test_default_tokens_are_not_seals_or_scientific_macros(tmp_path):
    doc = generated(tmp_path, ['SS','RT any','ST any','SW any','TT any','ZW any'])
    assert [a.oem_opcode for a in actions(doc)] == ['iniPipette','SS','RT','ST','SW','TT','ZW']
    assert all(SOURCE_DOMAINS[op] == () for op in OEM_SOURCE_DEFAULT_NOOPS)
    parsed = import_oem_xml_protocol(tmp_path / 'source.xml').document
    assert all(a.kind.value == 'note' for a in actions(parsed))


def test_dwell_accumulates_across_sp_and_iterations_without_wait(tmp_path):
    doc = generated(tmp_path, ['LOOP 2', 'DWELL 2', 'DWELL 3', 'SP T80 DUR10 R2',
                              'SP T40 DUR7 R4', 'DWELL -1', 'SP T30 DUR2 R1', 'LOOP'])
    sp = [a for a in actions(doc) if a.oem_opcode == 'sp']
    assert [a.params['arguments'] for a in sp] == [
        ('80.00','15','-2'), ('40.00','7','4'), ('30.00','6','-1'),
        ('80.00','19','-2'), ('40.00','7','4'), ('30.00','10','-1')]
    assert [a.metadata['loop_iteration'] for a in sp] == [0,0,0,1,1,1]
    assert 'wait' not in [a.oem_opcode for a in actions(doc)]
    assert [a.oem_opcode for a in actions(doc)][:2] == ['iniPipette','fon']
    assert not any(a.oem_opcode == 'splid' for a in actions(doc))


def test_thermal_close_park_and_signed_cooling(tmp_path):
    doc = generated(tmp_path, ['TCD DC','SP T80 DUR0 R2','SP T20 DUR0 R2'])
    assert [(a.oem_opcode, a.params['arguments']) for a in actions(doc) if a.oem_opcode != '//'] == [
        ('iniPipette',()), ('dclose',()), ('fon',()), ('park',()),
        ('splid',('86.00','10','1.0','T')), ('sp',('80.00','0','2')),
        ('splid',('26.00','10','-20.0','F')), ('sp',('20.00','0','-2'))]


def test_motion_only_selects_source_not_fake_execution_mode(tmp_path):
    doc = generated(tmp_path, ['CC OC 10','WAIT 99','SP T80 DUR9 R2','LOOP 2','DWELL 5','LOOP','LED 1 2 3'],
                    {**SETTINGS, 'MotionOnly': True})
    assert [a.oem_opcode for a in actions(doc)] == ['iniPipette','LOOP','DWELL','LOOP','led']
    assert doc.metadata['execution_mode'] == 'normal'
    assert doc.metadata['source_settings']['MotionOnly'] is True


def test_et_no_tip_still_moves_once_and_tradeshow_uses_null_destination(tmp_path):
    for mode, destination in [(0,6),(3,None)]:
        doc = generated(tmp_path, ['ET','ET'], {**SETTINGS,'StartMode':mode})
        assert [a.oem_opcode for a in actions(doc)] == ['iniPipette','mov']
        assert actions(doc)[1].params['arguments']['m_destination'] == destination


def test_source_door_exception_not_new_observation_gate(tmp_path):
    with pytest.raises(ValueError, match='Cannot move bio-security cover when door is closed'):
        generated(tmp_path, ['MC CV_BIOSECURITY LOC_BSCS'])
    with pytest.raises(ValueError, match='Cannot access Reagent Cover Storage when door is open'):
        generated(tmp_path, ['TCD DO','MC CV_REAGENT LOC_RCS'])
