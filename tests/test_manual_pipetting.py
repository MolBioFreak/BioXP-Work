"""Offline manual compiler -> ordinary executor -> SQLite WP8 -> native seams."""
from contextlib import nullcontext
from types import SimpleNamespace

import pytest

from bioxp.manual_pipetting import (ManualPipettingRequest, compile_manual_pipetting,
    manual_position_plan, bind_manual_position_handler)
from bioxp.protocols.models import ProtocolActionKind, ProtocolDocument
from bioxp.protocols.executor import ProtocolExecutor
from tests.test_protocol_oem_provider_bindings import rig


def program():
    return {"protocol_id": "manual-test", "steps": [
        {"operation": "move", "location_id": 3, "well": "C2", "position_flag": 1},
        {"operation": "lower", "location_id": 3},
        {"operation": "aspirate", "channels": [1], "volume_ul": 12.0, "speed": 20.0},
        {"operation": "lift", "location_id": 3, "height_steps": None},
        {"operation": "move", "location_id": 1, "well": "D3", "position_flag": 1},
        {"operation": "lower", "location_id": 1},
        {"operation": "dispense", "channels": [1], "volume_ul": 12.0, "speed": 22.0},
        {"operation": "mix", "channels": [1], "volume_ul": 7.0,
         "aspirate_speed": 15.0, "dispense_speed": 16.0, "cycles": 2},
        {"operation": "lift", "location_id": 1, "height_steps": 1000},
    ]}


def test_compiler_is_lossless_explicit_ordinary_document():
    doc = compile_manual_pipetting(program())
    assert ProtocolDocument.from_payload(doc.to_payload()) == doc
    kinds = [a.kind.value for a in doc.stages[0].actions]
    assert kinds == ["pipette_position", "pipette_position", "pipette_aspirate",
        "pipette_position", "pipette_position", "pipette_position", "pipette_dispense",
        "pipette_aspirate", "pipette_dispense", "pipette_aspirate", "pipette_dispense", "pipette_position"]
    assert "input_mode" not in doc.metadata
    assert all(a.oem_opcode is None for a in doc.stages[0].actions)
    assert all(a.params["channels"] == (1,) for a in doc.stages[0].actions if "channels" in a.params)
    assert "tip_loaded" not in repr(doc.to_payload())
    assert len(ManualPipettingRequest.model_json_schema()["$defs"]) >= 5
    from bioxp.services.protocol_service import compile_protocol_source, _workflow_resources
    assert compile_protocol_source({"source_type": "native", "document": doc.to_payload()}).document == doc
    assert {"axis:x", "axis:y", "axis:z", "pipette"} <= set(_workflow_resources(doc))
    from bioxp.protocols.validators import infer_required_capability
    assert infer_required_capability(ProtocolActionKind.PIPETTE_POSITION).value == "motion"
    with pytest.raises(ValueError):
        manual_position_plan({"operation": "move", "location_id": 3, "well": "A1", "position_flag": 1, "tip_loaded": True})


@pytest.mark.parametrize("step", [
    {"operation": "move", "location_id": 3, "well": "J1", "position_flag": 1},
    {"operation": "move", "location_id": 3, "well": "A1"},
    {"operation": "move", "location_id": 3, "well": "A1", "position_flag": True},
    {"operation": "move", "location_id": 32, "well": "A1", "position_flag": 1},
    {"operation": "lower", "location_id": 3, "tip_loaded": True},
    {"operation": "lift", "location_id": 3},
    {"operation": "aspirate", "channels": [1], "volume_ul": 12.0},
    {"operation": "dispense", "channels": [4], "volume_ul": 12.0, "speed": 20.0},
    {"operation": "mix", "channels": [1], "volume_ul": 12.0, "cycles": 0,
     "aspirate_speed": 10.0, "dispense_speed": 10.0},
])
def test_invalid_authored_values(step):
    with pytest.raises(ValueError):
        compile_manual_pipetting({"protocol_id": "invalid", "steps": [step]})


@pytest.mark.parametrize("tip_location", [-1, 0, 1, 2, 3])
@pytest.mark.parametrize("location", [1, 3, 7, 16])
def test_connected_calibrated_move_and_lower_lift(rig, monkeypatch, tmp_path, tip_location, location):
    import bioxp.oem_serial206_initialization as mod
    from bioxp.oem_runtime_store import OEMRuntimeStore
    from bioxp.operator_command_plane import OperatorCommandStore
    from bioxp.oem_deck_movement import make_wp8_operation_executor
    from bioxp.oem_compat.position_table import PositionTable, PositionTarget
    from bioxp.oem_compat.pathing import LOCATION_ID_TO_NAME
    from tests.test_cover_carry_release_connected import TransferNative
    p, _, trace, _, _, machine = rig
    # Synthetic calibration is explicitly offline, never a machine default.
    table = PositionTable([PositionTarget(name, base_coordinates={"x": 40000, "y": 40000},
        z_low=90000, z_high=30000, inc_factor=1) for name in LOCATION_ID_TO_NAME.values()])
    monkeypatch.setattr(mod, "load_bound_oem_position_table", lambda: table)
    monkeypatch.setattr(mod, "load_oem_parity_config", lambda _: SimpleNamespace(blockers=[], values={}))
    native = TransferNative()
    adapter = object.__new__(mod.Serial206ProductionPrimitiveAdapter)
    adapter.tester, adapter.y_provider, adapter.reference_store = native, None, None
    adapter._reference_snapshot = lambda axes, context: {"offline": True}
    adapter._z_profile_overrides = {}
    p.primitives = adapter
    p.scriptmoveTo = mod.Serial206OemInitializationProvider.scriptmoveTo.__get__(p)
    p.moveZ = mod.Serial206OemInitializationProvider.moveZ.__get__(p)
    p._deck_gripper_confirmed = lambda: True
    machine.update(tip_location=tip_location, clean_path=False, current_location=3, pseudo_z_home=500)
    init = OEMRuntimeStore(tmp_path); init.close()
    store = OperatorCommandStore(tmp_path)
    stamps = p.deck_owner_authority_stamps()
    store.bind_deck_owner_authority_reader(lambda: stamps, scope=nullcontext)
    store.bind_workflow_dispatcher(lambda command: None)
    p.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
    store.publish_deck_owner_state(source_operation="pipette_owner", source_command_id="fixture-tip-observation",
        updates={"tip_loaded": False, "tip_dirty": False, "tip_location": tip_location}, **stamps)
    store.admit_workflow(command_id="parent", idempotency_key="parent", plan_fingerprint="fixture",
        requested_inputs={"bundle": {}}, ownership_generation=1,
        resources=("axis:x", "axis:y", "axis:z"), board_epochs={})
    assert store.claim_next()["command_id"] == "parent"
    execute_wp8 = make_wp8_operation_executor(provider_getter=lambda: p, command_store=store)
    admission = {"ownership_generation": 1, "serial206_initialization_provider": {
        "x_authority": {"current_board_lifecycle_generation": 1}, "board4_authority": {"active_board_epoch": 1}}}
    children = []
    def execute(plan, action, state):
        admitted = store.admit_internal_wp8_operation(plan["operation"], inputs={}, state=admission,
            idempotency_key=action.action_id, prepared_plan=plan)
        cid = admitted["command_id"]; children.append(cid)
        claimed = store.claim_next()
        assert claimed["command_id"] == cid
        response = execute_wp8(command_id=cid, plan=plan)
        store.finish(cid, status="completed", payload={"response": response}, claimed=claimed)
        return response
    handler = bind_manual_position_handler(command_store=store, execute_plan=execute, require_motion_ready=lambda: None)
    doc = compile_manual_pipetting({"protocol_id": "connected", "steps": [
        {"operation": "move", "location_id": location, "well": "G2", "position_flag": 1},
        {"operation": "lower", "location_id": location},
        {"operation": "lift", "location_id": location, "height_steps": None},
        {"operation": "lift", "location_id": location, "height_steps": 1000},
    ]})
    try:
        state = ProtocolExecutor(dry_run=False, job_id="parent", before_native_entry=lambda identity, state: store.assert_workflow_current("parent"), handlers={ProtocolActionKind.PIPETTE_POSITION: handler}).execute(doc)
        assert state.completed, state.to_payload()
        assert len(children) == 4
        effective_row = 6 if location in (7, 16) else 6 - (0 if tip_location == -1 else tip_location) * 2
        assert native.positions[5, 0] == 40000 - 2132
        assert native.positions[4, 0] == 40000 + 2132 * effective_row
        assert native.positions[4, 1] == 89000
        assert [(e[1], e[2]) for e in native.events if e[0] == "move"][-3:] == [("z", 90000), ("z", 30000), ("z", 89000)]
        semantic = store.deck_semantic_state()
        assert semantic["current_location"] == LOCATION_ID_TO_NAME[location]
        assert semantic["current_well"] == 73
        assert semantic["tip_location"] == tip_location and semantic["tip_loaded"] is False
        assert not any(e[1] in ("g", "d") for e in native.events)
        assert [c["operation"] for c in store.wp8_operation_evidence(children[0])["children"]] == ["scriptmoveTo", "updateLocation"]
        assert all(store.get_command(cid)["status"] == "completed" for cid in children)
    finally:
        store.stop()


@pytest.mark.parametrize("fail", [None, "move", "aspirate", "dispense"])
@pytest.mark.parametrize("channels", [[1], [0, 1, 2, 3]])
def test_ordered_transfer_reaches_real_liquid_owner_and_stops_on_failure(rig, monkeypatch, fail, channels):
    from bioxp import api
    import bioxp.oem_serial206_initialization as mod
    from bioxp.oem_deck_movement import execute_finite_plate_operation
    from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport
    from tests.test_cover_carry_release_connected import TransferNative
    p, _, _, _, _, machine = rig
    native = TransferNative()
    adapter = object.__new__(mod.Serial206ProductionPrimitiveAdapter)
    adapter.tester, adapter.y_provider, adapter.reference_store = native, None, None
    adapter._reference_snapshot = lambda axes, context: {"offline": True}
    adapter._z_profile_overrides = {}
    p.primitives = adapter
    p.scriptmoveTo = mod.Serial206OemInitializationProvider.scriptmoveTo.__get__(p)
    p.moveZ = mod.Serial206OemInitializationProvider.moveZ.__get__(p)
    p._deck_gripper_confirmed = lambda: True
    from bioxp.oem_compat.position_table import PositionTable, PositionTarget
    from bioxp.oem_compat.pathing import LOCATION_ID_TO_NAME
    table = PositionTable([PositionTarget(name, base_coordinates={"x": 40000, "y": 40000},
        z_low=90000, z_high=30000, inc_factor=1) for name in LOCATION_ID_TO_NAME.values()])
    monkeypatch.setattr(mod, "load_bound_oem_position_table", lambda: table)
    # Deliberate fixture configuration; no actual controller/calibration loaded.
    monkeypatch.setattr(mod, "load_oem_parity_config", lambda _: SimpleNamespace(blockers=[], values={}))
    machine.update(tip_location=-1 if len(channels) == 4 else 1, clean_path=False,
        current_location=3, pseudo_z_home=500)
    publications = []
    p.bind_deck_semantic_state_publisher(lambda **kw: publications.append(kw) or {})
    # Both motion and liquid transports are closed synthetic native leaves.
    calls = []
    class Driver:
        def __init__(self, channel):
            self.channel = channel
        def result(self, operation, value):
            calls.append((operation, self.channel, value, dict(native.positions)))
            return {"ok": operation != fail, "delivery_verified": True,
                    "controller_acknowledged": True, "completion_verified": False}
        def set_top_speed(self, value, **kw):
            return self.result("speed", value)
        def aspirate(self, value, **kw):
            return self.result("aspirate", value)
        def dispense(self, value, **kw):
            return self.result("dispense", value)
        def wait_pipette_command_completion(self, timeout_s, **kw):
            return {"ok": True}
    transports = []
    for channel in range(4):
        t = CanPipetteTransport(driver_factory=lambda channel=channel: Driver(channel), pipette_id=channel)
        # Explicit offline starting observations, not produced by the compiler.
        t._initialized = t._tip_loaded = True
        transports.append(t)
    group = FourPipetteTransport(transports, sleep=lambda seconds: None)
    monkeypatch.setattr(api, "_get_pipette_transport", lambda: group)
    monkeypatch.setattr(api, "_pipette_receipts", None)
    monkeypatch.setattr(api, "_liquid_reference_preflight", lambda *a: None)
    plans = []
    def execute(plan, action, state):
        plans.append(plan)
        def invoke(child):
            if fail == "move" and child["operation"] == "scriptmoveTo":
                native.exception = "x"
            result = p.execute_wp8_child(child, command_id=action.action_id,
                child_order=child["order"], plan_digest=plan["plan_digest"])
            if result.get("ok") is not True:
                raise RuntimeError("native child failed")
            return result
        return execute_finite_plate_operation(plan, invoke)
    store = SimpleNamespace(workflow_context=lambda *a, **kw: nullcontext(), assert_workflow_current=lambda *a: None)
    handler = bind_manual_position_handler(command_store=store, execute_plan=execute, require_motion_ready=lambda: None)
    request = program()
    for step in request["steps"]:
        if "channels" in step:
            step["channels"] = channels
    doc = compile_manual_pipetting(request)
    # Any OEM job lifecycle entry would fail this test.
    lifecycle = {name: lambda *a, **kw: pytest.fail("unrequested full-job lifecycle")
                 for name in ("prepare", "run_job", "epilogue_sweep", "epilogue_lid", "epilogue_park")}
    state = ProtocolExecutor(dry_run=False, job_id="parent", before_native_entry=lambda *a: None, handlers={
        ProtocolActionKind.PIPETTE_POSITION: handler,
        ProtocolActionKind.PIPETTE_ASPIRATE: api._protocol_live_pipette_handler,
        ProtocolActionKind.PIPETTE_DISPENSE: api._protocol_live_pipette_handler,
    }, lifecycle_handlers=lifecycle).execute(doc)
    liquid = [c for c in calls if c[0] != "speed"]
    assert state.completed is (fail is None), state.to_payload()
    assert all(c[1] in channels for c in calls)
    assert group._tip_location == -1  # plunger selection never fabricates custody
    assert not any(p["source_operation"] == "pipette_owner" for p in publications)
    if fail == "move":
        assert not calls and not publications and len(plans) == 1
    elif fail == "aspirate":
        assert [c[0] for c in liquid] == ["aspirate"]
        assert len(plans) == 2  # no automatic lift/cleanup on failure
    elif fail == "dispense":
        assert [c[0] for c in liquid] == ["aspirate"] * len(channels) + ["dispense"]
        assert len(plans) == 5
    else:
        assert len(liquid) == 6 * len(channels)
        assert [c[0] for c in liquid] == [op for op in ("aspirate", "dispense") * 3 for _ in channels]
        assert [c[2] for c in liquid] == [v for v in (12.0, 12.0, 7.0, 7.0, 7.0, 7.0) for _ in channels]
        row_offset = 0 if len(channels) == 4 else 2
        assert liquid[0][3][5, 0] == 40000 - 2132
        assert liquid[0][3][4, 0] == 40000 + 2132 * (2 - row_offset)
        assert liquid[0][3][4, 1] == 90000
        assert liquid[len(channels)][3][5, 0] == 40000 - 2132 * 2
        assert liquid[len(channels)][3][4, 0] == 40000 + 2132 * (3 - row_offset)
        assert liquid[len(channels)][3][4, 1] == 90000
        assert native.positions[4, 1] == 89000
