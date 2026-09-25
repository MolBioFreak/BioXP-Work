"""Offline calibration -> real provider/adapters/transports; native IO doubled.

Source-state readers are fixtures. No implementation under test is replaced.
The finite child dispatcher and SQLite semantic publisher remain real.
"""
from types import SimpleNamespace
import pytest

from bioxp.pipette.oem_calibration import (
    ManualTipLoadRequest, FluidHeightRequest, CalibrationExecutionError,
    manual_load_tip, measure_fluid_height, bind_calibration_provider,
    evaluate_fluid_timing, calibration_samples, calibration_adjustment,
    detect_fluid_with_motion, aspirate_calibration_air,
)
from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport
from bioxp.oem_deck_movement import compile_finite_plate_operation, execute_finite_plate_operation
from bioxp.oem_serial206_initialization import Serial206ProductionPrimitiveAdapter, Serial206OemInitializationProvider
from bioxp.oem_compat.position_table import PositionTable, PositionTarget
from bioxp.oem_compat.pathing import LOCATION_ID_TO_NAME
from bioxp.operator_command_plane import OperatorCommandStore
from tests.test_cover_carry_release_connected import TransferNative


class MotionNative(TransferNative):
    def _motion_oem_axis_profile(self, axis, *, startup=False):
        return super()._motion_oem_axis_profile(axis, startup=startup)

    def motor_oem_require_no_motion_profile(self, axis, **kwargs):
        return {"ok": True}

    def motor_oem_move_z_home(self, **kwargs):
        self.events.append(("home_z", kwargs))
        self.positions[4, 1] = 0
        return {"ok": True, "source_return_code": -101}

    def motor_oem_board_stop(self, board, *, motor, axis_name):
        self.events.append(("stop_z",))
        self.positions[4, 1] = 88000
        return {"source_call_completed": True, "first_delivery": {"status": 100},
                "second_delivery": {"status": 100}}


class PipetteNative:
    def __init__(self, channel, events):
        self.channel, self.events = channel, events
        self.token = None
        self.failed = False
        self.stamp = 102.0 + channel * .1

    def issue(self, command, **kwargs):
        self.token = f"{command}:{self.channel}"
        self.events.append((command, self.channel, kwargs))
        return {"ok": True, "tx_ok": True, "ack": {"ok": True, "received": True},
                "completion_verified": False}

    def aspirate_air(self, volume, **kwargs):
        return self.issue("air", volume=volume, **kwargs)

    def start_fluid_detection(self, **kwargs):
        return self.issue("BR", **kwargs)

    def dispense_all(self, **kwargs):
        return self.issue("dispense", **kwargs)

    def terminate_pipette(self, **kwargs):
        return self.issue("TR", **kwargs)

    def query_tip_status(self):
        self.events.append(("query", self.channel))
        return {"ok": True, "semantic_ok": True, "tip_loaded": True,
                "source_return_completed": True, "source_return": 1,
                "source_tip_loaded": True, "hardware_truth_level": "hardware_query"}

    def current_pipette_completion_owner_token(self):
        return self.token

    def wait_pipette_command_completion(self, timeout, *, owner_token):
        self.events.append(("wait", self.channel, owner_token, timeout))
        return {"ok": not (self.failed and owner_token.startswith("BR")),
                "pipette_message_state": {"fluid_timestamp": self.stamp}}


@pytest.fixture
def rig(monkeypatch, tmp_path):
    import socket
    monkeypatch.setattr(socket, "socket", lambda *a, **kw: pytest.fail("hardware socket attempted"))
    monkeypatch.setattr("bioxp.oem_serial206_initialization.time.sleep", lambda _: None)
    events = []
    drivers = [PipetteNative(i, events) for i in range(4)]
    transports = [CanPipetteTransport(driver_factory=lambda d=d: d, pipette_id=i) for i, d in enumerate(drivers)]
    for t in transports:
        t._initialized = True
        t._tip_loaded = True
    group = FourPipetteTransport(transports, sleep=lambda s: events.append(("sleep", s)))
    native = MotionNative()
    # Keep one common chronological trace across both hardware seams.
    native.events = events
    adapter = Serial206ProductionPrimitiveAdapter(native, group, authority_provider=lambda: None, generation_provider=lambda: 1)
    provider = Serial206OemInitializationProvider(adapter)
    table = PositionTable([PositionTarget(name, base_coordinates={"x": 30000, "y": 40000},
                            z_high=30000, z_low=90000, inc_factor=1) for name in LOCATION_ID_TO_NAME.values()])
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_bound_oem_position_table", lambda: table)
    monkeypatch.setattr("bioxp.oem_compat.position_table.load_bound_oem_position_table", lambda: table)
    stamps = dict(ownership_generation=1, board_epoch_4=1, board_epoch_5=1)
    provider.deck_owner_authority_stamps = lambda: stamps
    from bioxp.oem_runtime_store import OEMRuntimeStore
    runtime = OEMRuntimeStore(tmp_path)
    store = OperatorCommandStore(tmp_path)
    provider.bind_deck_semantic_state_publisher(store.publish_deck_owner_state)
    provider.bind_deck_semantic_state_reader(store.deck_semantic_state)
    for op, changes in [("updateLocation", dict(current_location="LOC_RC", current_well=1)),
                        ("pipette_owner", dict(tip_loaded=False, tip_dirty=True, tip_location=-1)),
                        ("sourceForceToHighHome", dict(pseudo_z_home=500)),
                        ("clean_path_calculation", dict(clean_path=False))]:
        store.publish_deck_owner_state(source_operation=op, source_command_id=op, updates=changes, **stamps)
    # Captured source reader, not an override of a motion body.
    def facts():
        state = store.deck_semantic_state()
        return {**state, "current_location": next(i for i,n in LOCATION_ID_TO_NAME.items() if n == state["current_location"])}
    provider.mov_execution_machine_state = facts
    provider._offset_deck_semantic_state = lambda **_: facts()
    plans = []
    def finite(operation, inputs):
        plan = compile_finite_plate_operation(operation, source_leaf_available=True, **inputs)
        plans.append(plan)
        results = []
        def invoke(child):
            result = provider.execute_wp8_child(child, command_id=f"offline-{len(plans)}", child_order=child["order"], plan_digest=plan["plan_digest"])
            results.append(result)
            return result
        result = execute_finite_plate_operation(plan, invoke)
        # Source values are returned by the actual child, not synthetic values.
        return {**result, **results[-1]}
    bindings = bind_calibration_provider(provider, finite=finite,
        native=lambda name, call, inputs: call(), pipette=lambda name, call: call(group),
        tray_location=lambda tray: (7, 8, 9, 10, 15)[tray], z_current_down=17,
        sleep=lambda s: events.append(("sleep", s)), clock=lambda: 100.)
    yield SimpleNamespace(b=bindings, group=group, drivers=drivers, native=native,
                          provider=provider, events=events, store=store, plans=plans)
    store.stop()
    runtime.close()


@pytest.mark.parametrize("tray,well,overpress,lift", [(1,"A1",False,False),(4,"B12",True,True),(5,"A2",False,True)])
def test_manual_pickup_reaches_address_and_publishes_source_state(rig, tray, well, overpress, lift):
    result = manual_load_tip(ManualTipLoadRequest(tray, well, overpress, lift), rig.b)
    assert result["ok"] and not result["physical_effect_verified"]
    plan = rig.plans[0]
    assert plan["operation"] == "pipette_hotel"
    moves = [e for e in rig.events if e[0] == "move"]
    assert moves[-1][1:3] == ("z", 94030 if overpress else 90000)
    assert rig.store.deck_semantic_state()["tip_dirty"] is False
    assert rig.store.deck_semantic_state()["tip_loaded"] is False  # source never writes true
    assert rig.store.deck_semantic_state()["current_well"] == (12 if well[0] == "B" else 0) + int(well[1:]) - 1
    assert rig.native.positions[5, 0] == 30000 - 2132 * (int(well[1:]) - 1)
    assert rig.native.positions[4, 0] == 40000 + (2132 if well[0] == "B" else 0)
    if lift:
        assert result["lost_steps_warning"] and result["lost_steps"] == -101
        assert next(e for e in rig.events if e[0] == "home_z")[1]["rehome"] is False
    assert not any(e[0] in {"BR", "air", "dispense"} for e in rig.events)


def test_manual_native_failure_does_not_clear_dirty_or_query(rig):
    rig.native.fail_at = ("z", 94030)
    with pytest.raises(CalibrationExecutionError) as error:
        manual_load_tip(ManualTipLoadRequest(1, "A1", True), rig.b)
    assert not error.value.evidence["ok"]
    assert rig.store.deck_semantic_state()["tip_dirty"] is True
    assert not any(e[0] == "query" for e in rig.events)


def test_measurement_orders_br_before_nonblocking_z_before_wait(rig):
    result = measure_fluid_height(FluidHeightRequest(), rig.b)
    assert result["ok"] and result["position_steps"] == 88000
    assert result["detection_target_steps"] == 92015
    assert result["timing"]["delays_s"] == pytest.approx([2,2.1,2.2,2.3])
    br = [i for i,e in enumerate(rig.events) if e[0] == "BR"]
    z = next(i for i,e in enumerate(rig.events) if e[:3] == ("move", "z", 92015))
    waits = [i for i,e in enumerate(rig.events) if e[0] == "wait" and e[2].startswith("BR")]
    assert max(br) < z < min(waits)
    assert ("z", 92015, False) in rig.native.moves
    assert rig.native.parameters[4,1,4] == 1791
    assert any(e[:1] == ("stop_z",) for e in rig.events)
    assert not result["calibration_persisted"]


def test_detection_timeout_terminates_without_source_success_tail(rig):
    rig.drivers[2].failed = True
    with pytest.raises(CalibrationExecutionError, match="Detect fluid level timeout") as error:
        measure_fluid_height(FluidHeightRequest(300), rig.b)
    assert not error.value.evidence["ok"]
    assert [e[1] for e in rig.events if e[0] == "TR"] == [0,1,2,3]
    assert not any(e[0] in {"stop_z", "dispense"} for e in rig.events)
    assert rig.native.parameters[4,1,4] == 300


def test_source_timing_failure_keeps_speed_and_no_dispense(rig):
    rig.drivers[3].stamp = 111.
    with pytest.raises(CalibrationExecutionError, match="Fluilid clibration failure") as error:
        measure_fluid_height(FluidHeightRequest(), rig.b)
    assert error.value.evidence["timing"]["failed_pipettes_1_based"] == [4]
    assert rig.native.parameters[4,1,4] == 300
    assert not any(e[0] == "dispense" for e in rig.events)


def test_missing_timestamp_is_evidence_not_extra_gate(rig):
    rig.drivers[3].stamp = None
    result = measure_fluid_height(FluidHeightRequest(), rig.b)
    assert result["ok"] and result["timing"] is None
    assert rig.native.parameters[4,1,4] == 1791


def test_stop_during_z_keeps_br_completion_owners_and_interrupt_epoch(rig):
    def interrupted_move():
        return rig.group.terminate()
    result = detect_fluid_with_motion(rig.group, interrupted_move, sleep=lambda _: None)
    assert not result["ok"] and result["interrupted_by_terminate"]
    assert [row["completion_owner_token"] for row in result["channels"]] == [f"BR:{i}" for i in range(4)]
    waits = [e for e in rig.events if e[0] == "wait" and e[2].startswith("BR")]
    assert len(waits) == 4


def test_source_air_preserves_false_wait_and_channel_selection(rig, monkeypatch):
    rig.group._tip_location = 2
    rig.group._allow_to_stop = True
    original = rig.drivers[2].wait_pipette_command_completion
    def missing_completion(timeout, *, owner_token):
        return {**original(timeout, owner_token=owner_token), "ok": False}
    monkeypatch.setattr(rig.drivers[2], "wait_pipette_command_completion", missing_completion)
    result = aspirate_calibration_air(rig.group)
    assert result["ok"] and result["source_wait_return"] is False
    assert rig.group._allow_to_stop is True
    assert [e[1] for e in rig.events if e[0] == "air"] == [2]
    assert ("sleep", .001) in rig.events and ("sleep", .020) in rig.events
    assert result["front_air"] is False


def test_sample_order_and_exact_adjustment_math():
    assert calibration_samples("TC") == ("A1","B1","A5","B5","A9","B9")
    assert calibration_samples("STRIP",1) == ("A1","B1","A2","B2","A3","B3")
    ref = {"REVISION": 2, "FLUID_TC_OFFSET": 10,"FLUID_RC_OFFSET":20,"FLUID_STRIP_OFFSET":30}
    result = calibration_adjustment("STRIP", 80000, z_lows={11:91000,12:90000,13:92000,14:93000}, fluid_reference=ref)
    assert result["position_table_z_low_updates"] == {12:78530,11:79530,13:80530,14:81530}
    assert result["proposal_only"] and not result["calibration_persisted"]
    assert calibration_adjustment("OCMS", 80000, z_lows={}, fluid_reference=ref)["settings_updates"]["m_OPBufferLow"] == 80020
    assert evaluate_fluid_timing([1,1,1,10])["failed_pipettes_1_based"] == [4]


@pytest.mark.parametrize("tray,well", [(0,"A1"),(6,"A1"),(1,"C1"),(1,"A13"),(1,"OVER")])
def test_invalid_identity_never_becomes_raw_motion(tray, well):
    with pytest.raises(ValueError): ManualTipLoadRequest(tray, well)
