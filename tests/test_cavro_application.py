"""Original ADP application -> service -> SQLite -> real CAN/router, offline.

Only board exchange and native Z leaves are replaced. Wire replies below are
explicit controlled test inputs, not claimed observations of physical pumps.
"""
import asyncio
import copy
import time
from types import SimpleNamespace

import pytest

from bioxp.can_driver import BioXpCanDriver
from bioxp.novo_router import NovoRouter, NovoFrame
from bioxp.pipette.cavro_application import (IMPLEMENTATION, PRESSURE_PARAMETERS,
    compile_application, capability_catalog, run_application_inline, _send_wire)
from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport
from bioxp.pipette.receipts import PipetteReceiptStore
from bioxp.services.pipette_service import run_pipette_operation
from tests.test_deck_tip_query_publication import bind_collection_test_identity


def request(*operations):
    return {"implementation": IMPLEMENTATION, "operations": list(operations)}


def stroke(kind="aspirate", volume=22.5, speed=50, channels=None):
    return {"operation": kind, "volume_ul": volume, "speed_ul_s": speed,
            "channels": [0, 1] if channels is None else channels, "timeout_ms": 30}


@pytest.fixture
def rig(monkeypatch, tmp_path):
    bind_collection_test_identity(monkeypatch)
    router = NovoRouter(ep_in=object(), ep_out=object(), decode=lambda x: x)
    wire, waits, native = [], [], []
    fault = {}
    drivers = []
    def transact(msg, *, channel, matcher_name, expected_function, **kwargs):
        if hasattr(msg, "arbitration_id"):
            assert msg.arbitration_id == (0x100 | expected_function | (channel << 3))
        command = bytes(msg.data).decode("ascii")
        wire.append((channel, command))
        started = time.monotonic()
        txid = f"offline-cavro-{len(wire)}"
        token = router.prepare_pipette_completion(channel, .04, command_family=expected_function,
            command_name=matcher_name, expected_rx_id=0x500 + expected_function + channel * 8)
        router.bind_pipette_completion(channel, owner_token=token, transaction_id=txid, tx_started_at=started)
        def rx(data):
            router._dispatch(NovoFrame(0x500 + expected_function + channel * 8, len(data), bytes(data),
                b"", time.monotonic(), "pipette"))
        rx([])
        if fault.get("generation") and command.startswith("P") and channel == 1:
            router.reader_generation += 1
        if not (fault.get("missing") and command.startswith("P") and channel == 1):
            rx([33 if fault.get("device") and command.startswith("P") and channel == 1 else 32, 96])
        if fault.get("interrupt") and command.startswith("P") and channel == 1:
            group._interrupt_epoch += 1
        return {"ok": True, "immediate_ack_received": True, "completion_deferred": True,
            "completion_received": False, "completion_owner_token": token,
            "tx_ok": True, "transaction_id": txid, "channel": channel,
            "owner_generation": router.reader_generation, "tx_timestamp": started,
            "receive_timestamp": time.monotonic(), "outcome": "ack", "frames": [
                {"data": [], "dlc": 0, "arbitration_id": 0x500 + expected_function + channel * 8,
                 "received_at": time.monotonic()}]}
    def wait(channel, timeout_s, **kwargs):
        waits.append((channel, kwargs.get("owner_token")))
        native.append(("pipette_wait", channel))
        return router.wait_pipette_completion(channel, timeout_s, **kwargs)
    leaves = []
    for channel in range(4):
        driver = BioXpCanDriver.__new__(BioXpCanDriver)
        driver.pipette_id = channel
        driver.response_timeout_s = .04
        driver._pipette_completion_owner_token = None
        driver._sleep = lambda _: None
        def transact_many(messages, **kwargs):
            assert len(messages) > 1
            assert all(len(m.data) <= 8 for m in messages)
            channel = kwargs["channel"]
            assert [m.arbitration_id for m in messages] == [
                0x103 | (channel << 3),
                *([0x104 | (channel << 3)] * (len(messages) - 2)),
                0x101 | (channel << 3)]
            joined = SimpleNamespace(data=b"".join(bytes(m.data) for m in messages))
            return transact(joined, **kwargs)
        driver.bus = SimpleNamespace(router=router, transact_can=transact,
                                      transact_can_many=transact_many,
                                      wait_pipette_completion=wait, send=lambda _: None)
        leaf = CanPipetteTransport(driver_factory=lambda d=driver: d, pipette_id=channel)
        leaf._initialized = True
        leaves.append(leaf); drivers.append(driver)
    group = FourPipetteTransport(leaves, sleep=lambda _: None)
    from bioxp.oem_runtime_store import OEMRuntimeStore
    runtime = OEMRuntimeStore(tmp_path)
    store = PipetteReceiptStore(tmp_path)
    from bioxp.operator_command_plane import OperatorCommandStore
    owner = OperatorCommandStore(tmp_path)
    owner.bind_workflow_dispatcher(lambda _: None)
    owner.admit_workflow(command_id="offline-finite-owner", idempotency_key="offline-finite-owner",
        plan_fingerprint="cavro-test", requested_inputs={"bundle": {"execution": {"runtime_state": {}}}},
        ownership_generation=1, resources=("axis:z", "pipette"), board_epochs={})
    owner.claim_next()
    fences = []
    def fence(command_id, *, boundary):
        fences.append((command_id, boundary))
        if fault.get("owner_lost") and len(wire) >= 2:
            owner.finish_workflow(command_id, status="interrupted", payload={}, lifecycle_settled=True)
        owner.assert_workflow_current(command_id)
    async def inline(label, body, **kwargs):
        return body()
    def receipt(name, call, command_id, identity, inputs):
        return asyncio.run(run_pipette_operation(name, call, get_transport=lambda: group,
            run_blocking=inline, receipt_store=store, requested_inputs={"application": inputs},
            runtime_binding={"idempotency_key": identity["source_identity"],
                "entrypoint_id": "protocol.pipette_manual_physical", "caller_class": "protocol_manual",
                "parent_operator_command_id": command_id}))
    def z(name, value=None):
        native.append((name, value))
        return {"ok": True, "position_steps": value}
    provider = SimpleNamespace(primitives=SimpleNamespace(pipette_transport=group,
        z_set_max_speed=lambda v: z("z_speed", v),
        oem_move_z=lambda v, **kwargs: z("z_search", v),
        z_stop=lambda: z("z_stop"), _read_axis_position=lambda axis: z("z_observation", 54321)["position_steps"]),
        moveZ=lambda v: z("z_move", v),
        _offset_deck_semantic_state=lambda **_: {"pseudo_z_home": 0},
        _wp8_execution_fence_checker=fence, _manual_pipette_receipt_runner=receipt)
    yield SimpleNamespace(group=group, provider=provider, router=router, wire=wire, waits=waits,
        native=native, fault=fault, store=store, root=tmp_path, drivers=drivers, owner=owner)
    owner.stop()
    runtime.close()


def run(rig, payload):
    return run_application_inline(rig.provider, payload, command_id="offline-finite-owner",
                                  owner_identity={"source_identity": "offline-occurrence"})


def test_real_driver_service_store_segmented_and_phase_air(rig):
    payload = request(stroke("leading_air", 20, 30), stroke(),
        {"operation": "delay", "duration_ms": 0}, stroke("trailing_air", 1.25, 40),
        stroke("dispense", 12.5, 375), stroke("dispense", 10, 875),
        {"operation": "empty", "channels": [0, 1], "timeout_ms": 30, "speed_ul_s": 875})
    out = run(rig, payload)
    assert out["ok"], out
    assert [w for ch, w in rig.wire if ch == 0] == [
        "V30,1R", "P20,1R", "V50,1R", "P22.5,1R", "V40,1R", "P1.25,1R",
        "V375,1R", "D12.5,1R", "V875,1R", "D10,1R", "V875,1R", "A0R"]
    assert len(rig.waits) == len(rig.wire)
    assert len({token for _, token in rig.waits}) == len(rig.waits)
    rows = rig.store.connection.execute("SELECT status,receipt_json FROM pipette_operations").fetchall()
    assert len(rows) == 12
    assert all(row["status"] == "completed" for row in rows)
    reopened = PipetteReceiptStore(rig.root)
    assert reopened.connection.execute("SELECT count(*) FROM pipette_operations").fetchone()[0] == 12
    assert all(e.get("reported_applied", {}).get("readback") is None for e in out["events"] if e["operation"] != "delay")
    assert out["physical_effect_verified"] is False


@pytest.mark.parametrize("fault", ["missing", "device", "generation", "interrupt", "owner_lost"])
def test_partial_channels_and_owner_interruption_never_replay(rig, fault):
    rig.fault[fault] = True
    out = run(rig, request(stroke(), stroke("dispense", 22.5)))
    assert out["ok"] is False, out
    assert not any(w.startswith("D") for _, w in rig.wire)
    assert rig.wire.count((0, "P22.5,1R")) <= 1
    assert rig.wire.count((1, "P22.5,1R")) <= 1
    assert out["events"][-1]["status"] == "failed"


def test_plld_is_send_start_z_wait_stop_observe_not_classifier(rig):
    out = run(rig, request({"operation": "plld", "channels": [0, 1], "timeout_ms": 30,
        "start_steps": 40000, "search_target_steps": 60000, "search_speed_native": 300,
        "z_motor_current": 20}))
    assert out["ok"], out
    assert rig.wire == [(0, "BR"), (1, "BR")]
    assert [name for name, _ in rig.native] == ["z_speed", "z_move", "z_search",
        "pipette_wait", "pipette_wait", "z_stop", "z_observation"]
    assert all(isinstance(stamp, float) for stamp in out["plld"]["fluid_timestamps"].values())


def test_settings_every_p_parameter_slope_and_no_water_invention():
    values = {name: spec[3] for name, spec in PRESSURE_PARAMETERS.items()}
    values.update(slope=[20, 10], pressure_streaming=True)
    raw = request({"operation": "settings", "channels": [0], "timeout_ms": 30, "values": values})
    before = copy.deepcopy(raw)
    out = compile_application(raw)
    assert raw == before
    assert not out["issues"]
    wires = [c["ascii"] for c in out["operations"][0]["wire_commands"]]
    assert wires == ["p0,15R", "p1,5R", "p2,0R", "p3,150R", "p4,44000R", "p5,0R",
                     "p6,20R", "p7,3R", "p8,50R", "L20,10R", "b15R", "o0,1R"]


@pytest.mark.parametrize("field,value", [("start_speed_ul_s", 100.001), ("cutoff_speed_ul_s", 200.001),
    ("start_speed_ul_s", 2.499), ("cutoff_speed_ul_s", 2.499),
    ("start_speed_ul_s", "25.0001"), ("cutoff_speed_ul_s", "50.0001"),
    ("start_speed_ul_s", True), ("cutoff_speed_ul_s", None),
    ("slope", [20]), ("plld_persistence_ms", None),
    ("unknown", 3)])
def test_no_truncated_executable_prefix(rig, field, value):
    payload = request(stroke(), {"operation": "settings", "channels": [0], "timeout_ms": 30,
                               "values": {field: value}})
    plan = compile_application(payload)
    assert plan["operations"] is None and plan["issues"]
    assert plan["requested"] == payload
    assert not run(rig, payload)["ok"]
    assert not rig.wire


def test_partial_send_exception_keeps_earlier_channel(rig, monkeypatch):
    def fail(*args, **kwargs):
        raise RuntimeError("physical leaf exception")
    monkeypatch.setattr(rig.drivers[1], "_send_pipette_command", fail)
    with pytest.raises(Exception) as raised:
        _send_wire(rig.group, [0, 1, 2], "P1,1R", 30, effect="aspirate")
    assert [r["channel"] for r in raised.value.details["channels"]] == [0]
    assert rig.wire == [(0, "P1,1R")]


def test_native_frozen_application_preserves_null_and_metadata():
    from types import MappingProxyType
    from bioxp.manual_pipetting import ManualCavroApplication
    raw = MappingProxyType({"implementation": IMPLEMENTATION,
        "operations": (MappingProxyType({"operation": "delay", "duration_ms": 0}),),
        "liquid_settings": MappingProxyType({"requested": None, "unknown_future": "1.2300"})})
    parsed = ManualCavroApplication.model_validate({"operation": "cavro_application", "application": raw})
    assert parsed.application["liquid_settings"] == {"requested": None, "unknown_future": "1.2300"}
    assert parsed.application["operations"] == [{"operation": "delay", "duration_ms": 0}]


def test_all_unsupported_settings_have_exact_paths():
    out = compile_application(request({"operation": "settings", "channels": [0], "timeout_ms": 30,
        "values": {"unknown_a": 1, "unknown_b": 2}}))
    assert {i["path"] for i in out["issues"]} == {"/operations/0/values/unknown_a", "/operations/0/values/unknown_b"}
    assert out["operations"] is None


@pytest.mark.parametrize("start,cutoff,start_wire,cutoff_wire", [
    (2.5, 2.5, "v2.5,1R", "c2.5,1R"),
    (100, 200, "v100,1R", "c200,1R"),
    ("25.1250", "199.875", "v25.125,1R", "c199.875,1R"),
])
def test_original_manual_speed_setters_real_can_owner_and_receipts(rig, start, cutoff, start_wire, cutoff_wire):
    payload = request({"operation": "settings", "channels": [0, 2], "timeout_ms": 30,
                       "values": {"start_speed_ul_s": start, "cutoff_speed_ul_s": cutoff}})
    before = copy.deepcopy(payload)
    result = run(rig, payload)
    assert result["ok"], result
    assert payload == before == result["requested"]
    # 25.125/199.875 exercise the actual multipart CAN encoder and token owner.
    assert rig.wire == [(ch, wire) for wire in [start_wire, cutoff_wire] for ch in [0, 2]]
    assert len(rig.waits) == len(rig.wire)
    assert len({token for _, token in rig.waits}) == len(rig.waits)
    rows = rig.store.connection.execute("SELECT status,receipt_json FROM pipette_operations").fetchall()
    assert len(rows) == 2 and all(row["status"] == "completed" for row in rows)
    assert [e["inputs"]["field"] for e in result["events"]] == ["start_speed_ul_s", "cutoff_speed_ul_s"]
    assert all(e["reported_applied"]["readback"] is None for e in result["events"])
    assert result["physical_effect_verified"] is False
    catalog = capability_catalog()
    assert catalog["start_cutoff_speed"]["status"] == "implemented"
    assert catalog["start_cutoff_speed"]["automatically_applied"] is False
    assert catalog["installed_firmware"] is None


@pytest.mark.parametrize("field,wire", [("start_speed_ul_s", "v25,1R"), ("cutoff_speed_ul_s", "c200,1R")])
def test_speed_setter_partial_channel_exception_does_not_replay(rig, monkeypatch, field, wire):
    def fail(*args, **kwargs):
        raise RuntimeError("offline setter channel exception")
    monkeypatch.setattr(rig.drivers[1], "_send_pipette_command", fail)
    value = 25 if field == "start_speed_ul_s" else 200
    result = run(rig, request({"operation": "settings", "channels": [0, 1], "timeout_ms": 30,
                               "values": {field: value}}, stroke()))
    assert result["ok"] is False
    assert rig.wire == [(0, wire)]
    assert result["events"][-1]["status"] == "failed"
    assert result["events"][-1]["inputs"]["field"] == field
    assert result["partial_effects"] is True
    assert result["requested_control"] == "stop"


@pytest.mark.parametrize("value,wire", [(0, "K0R"), (12, "K12R"), (500, "K500R")])
def test_backlash_emits_documented_command(value, wire):
    from bioxp.pipette.cavro_application import setting_commands
    assert setting_commands({"backlash_increments": value}) == [("backlash_increments", wire)]


@pytest.mark.parametrize("value", [-1, 501, 1.5, True, None])
def test_backlash_rejects_out_of_range(value):
    out = compile_application(request({"operation": "settings", "channels": [0], "timeout_ms": 30,
        "values": {"backlash_increments": value}}))
    assert [i["path"] for i in out["issues"]] == ["/operations/0/values/backlash_increments"]
    assert out["operations"] is None
