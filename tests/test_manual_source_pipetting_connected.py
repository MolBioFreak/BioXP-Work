"""Actual source-selected pickup -> canonical alignment -> addressed strokes.

Both SQLite owners, finite queue claims and production provider/adapters run;
only hardware IO/configuration are offline fixtures. No selected alignment seed.
"""
import socket
from contextlib import nullcontext
from types import SimpleNamespace
import pytest
from tests.test_oem_pipette_calibration import rig
from bioxp.manual_pipetting import (compile_manual_pipetting, bind_manual_physical_handler,
    bind_manual_position_handler, ManualPipettingRequest, manual_physical_plan)
from bioxp.protocols.executor import ProtocolExecutor
from bioxp.protocols.models import ProtocolActionKind, ProtocolDocument
from bioxp.oem_deck_movement import make_wp8_operation_executor
from bioxp.pipette.receipts import PipetteReceiptStore

_SOCKET = socket.socket


@pytest.fixture
def connected(rig, monkeypatch, tmp_path):
    from bioxp import api
    from tests.test_deck_tip_query_publication import bind_collection_test_identity
    bind_collection_test_identity(monkeypatch)
    monkeypatch.setattr(socket, "socket", lambda family=socket.AF_INET, *a, **kw:
        _SOCKET(family, *a, **kw) if family == socket.AF_UNIX else pytest.fail("network socket"))
    snapshot = SimpleNamespace(operation_parameters={"Mode": "WebMode"},
        config_sections={"offsets": {"m_Z_MOTOR_MAX_CURRENT_DOWN": 17}},
        fields={"machine.camera_installed": SimpleNamespace(value=False)}, camera_calibrated=False)
    monkeypatch.setattr("bioxp.oem_machine_bundle.get_active_oem_machine_snapshot", lambda: snapshot)
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_oem_parity_config",
                        lambda _: SimpleNamespace(blockers=[], values={}))
    monkeypatch.setattr("bioxp.runtime_state.get_active_oem_runtime_state_store", lambda: object())
    monkeypatch.setattr("bioxp.pipette.manual_settings.read_pipette_operation_settings",
        lambda _: {"runtime_values": {"LogPressure": False, "CheckForStaticTipLoss": False}})
    p, store = rig.provider, rig.store
    from bioxp.serial206_y_provider import Serial206YProvider
    p.primitives.y_provider = Serial206YProvider(rig.native, state_store=None,
                                                 generation_provider=lambda: 1)
    from bioxp.oem_compat.position_table import PositionTable, PositionTarget
    from bioxp.oem_compat.pathing import LOCATION_ID_TO_NAME
    table = PositionTable([PositionTarget(name, base_coordinates={"x": 30000,
        "y": 45000 if i == 3 else 50000 if i == 1 else 40000},
        z_high=30000, z_low=90000, inc_factor=1) for i,name in LOCATION_ID_TO_NAME.items()])
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_bound_oem_position_table", lambda: table)
    monkeypatch.setattr("bioxp.oem_compat.position_table.load_bound_oem_position_table", lambda: table)
    p._oem_pipette_rgb_writer = lambda *a: {"ok": True}
    p.sleep = lambda _: None
    monkeypatch.setattr(rig.native, "motor_prepare_axis", lambda *a, **kw: {"ok": True}, raising=False)
    def relative(board, delta, *, motor=0):
        rig.native.positions[board, motor] += delta
        return {"ok": True, "ack": {"status": 100}}
    monkeypatch.setattr(rig.native, "motor_move_relative", relative, raising=False)
    monkeypatch.setattr(rig.native, "motor_wait_stopped", lambda *a, **kw: {"ok": True, "stopped": True}, raising=False)
    p._read_constructed_tip_tray = lambda i: {"tip_type": "T200" if i == 3 else "T50",
        "location": (7,8,9,10,15)[i], "construction_id": "fixture", "tip_available": i != 4,
        "occupancy": [i != 4] * 96}
    store.bind_tip_tray_constructor_reader(p._read_constructed_tip_tray)
    p.bind_tip_tray_state_reader(store.tip_tray_state)
    p.bind_tip_tray_state_publisher(store.publish_tip_tray_transition)
    stamps = p.deck_owner_authority_stamps()
    store.bind_deck_owner_authority_reader(lambda: stamps, scope=nullcontext)
    store.bind_workflow_dispatcher(lambda command: None)
    receipts = PipetteReceiptStore(tmp_path)
    monkeypatch.setattr(api, "_get_pipette_transport", lambda: rig.group)
    monkeypatch.setattr(api, "_pipette_receipts", receipts)
    monkeypatch.setattr(api, "_liquid_reference_preflight", lambda *a: None)
    for t in rig.group._transports:
        t._tip_loaded = False
    rig.group._tip_type = 201
    # Pickup is simulated only at the hardware seam: lowerPipette's native
    # press makes tips present; KeepTip's real transport ejects the others.
    loaded = [False] * 4
    press_count = [0]
    original_move = rig.native.motor_oem_move_absolute
    def move(*args, **kw):
        result = original_move(*args, **kw)
        if args[:2] == (4, 90000) and kw.get("motor") == 1:
            loaded[:] = [True] * 4
            press_count[0] += 1
        return result
    # Native fake exposes the same positional signature as the real tester.
    monkeypatch.setattr(rig.native, "motor_oem_move_absolute", move)
    strokes = []
    for d in rig.drivers:
        d.bus = SimpleNamespace(router=SimpleNamespace(reader_generation=1))
        def query(d=d):
            d.events.append(("query", d.channel))
            value = loaded[d.channel]
            return {"ok": True, "semantic_ok": True, "tip_loaded": value,
                "source_return_completed": True, "source_return": int(value),
                "source_tip_loaded": value, "hardware_truth_level": "hardware_query", "reader_generation": 1}
        def eject(d=d, **kw):
            loaded[d.channel] = False
            return d.issue("eject", **kw)
        def stroke(name, volume, d=d, **kw):
            strokes.append((name, d.channel, volume, dict(rig.native.positions)))
            return d.issue(name, volume=volume, **kw)
        monkeypatch.setattr(d, "query_tip_status", query)
        monkeypatch.setattr(d, "pipette_eject_tip", eject, raising=False)
        monkeypatch.setattr(d, "aspirate", lambda v, stroke=stroke, **kw: stroke("aspirate", v, **kw), raising=False)
        monkeypatch.setattr(d, "dispense", lambda v, stroke=stroke, **kw: stroke("dispense_liquid", v, **kw), raising=False)
        monkeypatch.setattr(d, "set_top_speed", lambda v, d=d, **kw: d.issue("speed", volume=v, **kw), raising=False)
    finite = make_wp8_operation_executor(provider_getter=lambda: p, command_store=store)
    admission = {"ownership_generation": 1, "serial206_initialization_provider": {
        "x_authority": {"current_board_lifecycle_generation": 1}, "board4_authority": {"active_board_epoch": 1}}}
    results = []
    def execute(plan, action, state):
        admitted = store.admit_internal_wp8_operation(plan["operation"], inputs={}, state=admission,
            idempotency_key=f"{state.job_id}:{action.action_id}", prepared_plan=plan)
        cid = admitted["command_id"]
        claim = store.claim_next()
        assert store._renew_owner(lease_seconds=120.0)
        assert claim["command_id"] == cid
        try:
            result = finite(command_id=cid, plan=plan)
        except Exception as exc:
            store.finish(cid, status="failed", payload={"error": str(exc)}, claimed=claim)
            raise
        store.finish(cid, status="completed", payload={"response": result}, claimed=claim)
        results.append(result)
        return result
    handlers = {
        ProtocolActionKind.PIPETTE_MANUAL_PHYSICAL: bind_manual_physical_handler(command_store=store,
            execute_plan=execute, require_motion_ready=lambda: None, provider_getter=lambda: p,
            receipt_store_getter=lambda: receipts),
        ProtocolActionKind.PIPETTE_POSITION: bind_manual_position_handler(command_store=store,
            execute_plan=execute, require_motion_ready=lambda: None),
        ProtocolActionKind.PIPETTE_ASPIRATE: api._protocol_live_pipette_handler,
        ProtocolActionKind.PIPETTE_DISPENSE: api._protocol_live_pipette_handler}
    runs = []
    def run(steps):
        job = f"manual-source-{len(runs)}"
        doc = compile_manual_pipetting({"protocol_id": job, "steps": steps})
        assert ProtocolDocument.from_payload(doc.to_payload()) == doc
        store.admit_workflow(command_id=job, idempotency_key=job, plan_fingerprint=job,
            requested_inputs={}, ownership_generation=1,
            resources=("axis:x", "axis:y", "axis:z", "pipette"), board_epochs={})
        claim = store.claim_next()
        assert store._renew_owner(lease_seconds=120.0)
        if not runs:
            for index in range(5):
                p.publish_tip_tray_transition(tray_id=index, transition="construct",
                    operation_id=f"{job}:construct:{index}", command_id=job,
                    provenance={"source_operation": "ClassMachineStatus.constructor"})
        state = ProtocolExecutor(dry_run=False, job_id=job, before_native_entry=lambda *a:
            store.assert_workflow_current(job), handlers=handlers).execute(doc)
        store.finish_workflow(job, status="completed" if state.completed else "failed", payload={}, lifecycle_settled=True)
        runs.append(state)
        return state
    yield SimpleNamespace(**vars(rig), run=run, results=results, receipts=receipts,
                          strokes=strokes, loaded=loaded, press_count=press_count)
    receipts.connection.close()


def load(pipette, force=True):
    return {"operation": "source_load_tips", "tip_type": 50, "pipette": pipette, "force_new_tip": force}


def transfer(pipette):
    channels = list(range(4)) if pipette == -1 else [pipette]
    return [{"operation": "move", "location_id": 3, "well": "G2", "position_flag": 1},
        {"operation": "aspirate", "channels": channels, "volume_ul": 10., "speed": 20.},
        {"operation": "move", "location_id": 1, "well": "H3", "position_flag": 1},
        {"operation": "dispense", "channels": channels, "volume_ul": 10., "speed": 20.}]


@pytest.mark.parametrize("pipette", [-1,0,1,2,3])
def test_source_selection_then_transfer_and_consecutive_requests(connected, pipette):
    c = connected
    assert c.store.deck_semantic_state()["tip_location"] == -1
    state = c.run([load(pipette), *transfer(pipette)])
    assert state.completed, str([{k:v for k,v in row.items() if k in ('error', 'message', 'error_type')} for row in state.to_payload()['action_results']])
    assert c.store.deck_semantic_state()["tip_location"] == pipette
    assert c.group._tip_location == pipette
    assert c.press_count[0] == 1
    offset = 0 if pipette == -1 else pipette * 2
    assert c.strokes[0][3][4,0] == 45000 + 2132 * (6-offset)
    assert c.strokes[-1][3][4,0] == 50000 + 2132 * (7-offset)
    assert c.strokes[0][3][5,0] == 30000 - 2132
    assert c.strokes[-1][3][5,0] == 30000 - 4264
    first = c.results[0]["completed_children"][0]["result"]
    assert first["alignment_published"] and not first["already_matching_tip_type"]
    first_names = [row["operation"] for row in first["native_results"]]
    assert ("KeepTip" in first_names) is (pipette != -1)
    assert first_names.index("lowerPipette") < first_names.index("loadTip")
    # Different requested channel does NOT relabel matching retained tips.
    other = 1 if pipette != 1 else 2
    state = c.run([load(other, False), *transfer(pipette)])
    assert state.completed, str([{k:v for k,v in row.items() if k in ('error', 'message', 'error_type')} for row in state.to_payload()['action_results']])
    assert c.store.deck_semantic_state()["tip_location"] == pipette
    assert c.press_count[0] == 1
    retained = c.results[3]["completed_children"][0]["result"]
    assert retained["already_matching_tip_type"] and not retained["alignment_published"]
    assert retained["requested_pipette"] == other and retained["tip_location"] == pipette
    # Forced reload queries/ejects and selects from retained inventory.
    state = c.run([load(pipette), *transfer(pipette)])
    assert state.completed, str([{k:v for k,v in row.items() if k in ('error', 'message', 'error_type')} for row in state.to_payload()['action_results']])
    assert c.press_count[0] == 2
    occupancy = c.store.tip_tray_state(0)["occupancy"]
    start = 0 if pipette == -1 else pipette*24
    assert not occupancy[start] and not occupancy[start+12]
    ids = [row[0] for row in c.receipts.connection.execute("SELECT command_id FROM pipette_operations")]
    assert len(ids) > 10 and len(ids) == len(set(ids))


def test_pickup_failure_prevents_transfer_without_cleanup(connected):
    c = connected
    c.native.fail_at = ("z", 90000)
    state = c.run([load(2), *transfer(2)])
    assert not state.completed
    assert not c.strokes
    assert c.store.deck_semantic_state()["tip_location"] == -1
    assert c.group._tip_location == -1


def test_typed_contract_keeps_source_and_manual_mix_distinct():
    steps = [load(3), {"operation": "source_mix", "volume_ul": 20.},
        {"operation": "source_aspirate_air", "volume_ul": 5.},
        {"operation": "source_dispense_air", "volume_ul": 5.},
        {"operation": "source_purge"}]
    doc = compile_manual_pipetting({"protocol_id": "ui", "steps": steps})
    assert len(doc.stages[0].actions) == len(steps)
    for action in doc.stages[0].actions:
        assert manual_physical_plan(action.params)["children"][0]["operation"] == "sourceManualPipette"
    assert "SourceLoadTips" in ManualPipettingRequest.model_json_schema()["$defs"]


@pytest.mark.parametrize("operation", ["source_mix", "source_aspirate_air", "source_dispense_air", "source_purge"])
def test_scientific_source_calls_use_native_inline_owner(connected, monkeypatch, operation):
    c = connected
    for d in c.drivers:
        monkeypatch.setattr(d, "query_pressure", lambda: {"ok": True, "semantic_ok": True, "pressure": 1.0}, raising=False)
        monkeypatch.setattr(d, "dispense_air", lambda v, d=d, **kw: d.issue("dispense_air", volume=v, **kw), raising=False)
    step = {"operation": operation}
    if operation != "source_purge":
        step["volume_ul"] = 20.0 if operation == "source_mix" else 5.0
    if operation == "source_mix":
        step.update(air_ul=0.0, aspirate_delay_ms=0, dispense_delay_ms=0, cycles=1)
    state = c.run([load(-1), {"operation": "move", "location_id": 3,
        "well": "A2", "position_flag": 1}, step])
    assert state.completed, str([{k:v for k,v in row.items() if k in ('error', 'message', 'error_type')}
                                for row in state.to_payload()['action_results']])
    body = c.results[-1]["completed_children"][0]["result"]
    names = [row["operation"] for row in body["native_results"]]
    expected = {"source_mix": "aspirate", "source_aspirate_air": "aspirate_air",
                "source_dispense_air": "dispense_air_source", "source_purge": "dispense_all"}
    assert expected[operation] in names
    assert len(c.receipts.read(limit=100)) > 4


def test_exported_contract_matches_models_and_native_documents():
    import json
    from pathlib import Path
    from pydantic import TypeAdapter, Field
    from typing import Annotated, Union
    from bioxp.manual_pipetting import SourceLoadTips, SourceMix, SourceAir, SourcePurge
    contract = json.loads((Path(__file__).parents[1] /
        "testdata/contracts/manual_source_pipetting_v1.json").read_text())
    adapter = TypeAdapter(Annotated[Union[SourceLoadTips, SourceMix, SourceAir, SourcePurge],
                                    Field(discriminator="operation")])
    assert contract["step_schema"] == adapter.json_schema()
    assert len(contract["examples"]) == 6
    for example in contract["examples"]:
        doc = compile_manual_pipetting(example["request"])
        assert ProtocolDocument.from_payload(example["document"]) == doc
        for action in doc.stages[0].actions:
            if action.kind == ProtocolActionKind.PIPETTE_MANUAL_PHYSICAL:
                manual_physical_plan(action.params)


def test_stroke_failure_after_actual_selected_pickup(connected, monkeypatch):
    c = connected
    monkeypatch.setattr(c.drivers[2], "aspirate", lambda *a, **kw: {"ok": False})
    state = c.run([load(2), *transfer(2)])
    assert not state.completed
    assert c.store.deck_semantic_state()["tip_location"] == 2
    assert c.group._tip_location == 2
    assert not c.strokes  # failed first aspirate never reaches dispense
    assert c.store.deck_semantic_state()["current_location"] == "LOC_RC"


def test_forced_reload_failure_retains_source_ejection_reset(connected):
    c = connected
    assert c.run([load(2), *transfer(2)]).completed
    c.native.fail_at = ("z", 90000)
    state = c.run([load(1), *transfer(1)])
    assert not state.completed
    assert c.store.deck_semantic_state()["tip_location"] == -1
    assert c.group._tip_location == -1
    assert not any(c.loaded)


def test_source_press_retry_is_not_an_outer_retry(connected, monkeypatch):
    c = connected
    for d in c.drivers:
        original = d.query_tip_status
        def query(original=original):
            if c.press_count[0] == 1:
                c.loaded[:] = [False] * 4
            return original()
        monkeypatch.setattr(d, "query_tip_status", query)
    state = c.run([load(-1), *transfer(-1)])
    assert state.completed, str([{k:v for k,v in row.items() if k in ('error','message')}
                                for row in state.to_payload()['action_results']])
    assert c.press_count[0] == 2
    body = c.results[0]["completed_children"][0]["result"]
    names = [row["operation"] for row in body["native_results"]]
    assert names.count("lowerPipette") == 2
    assert names.count("loadTip") == 1
