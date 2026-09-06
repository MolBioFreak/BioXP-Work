"""F12 ordering proof through the API, provider, adapter and USB router.

Only USB endpoints/configuration are synthetic. Event synchronization is a
software ordering oracle, not a hardware latency or physical-stop claim.
"""
from __future__ import annotations

import asyncio
import queue
import threading
from types import SimpleNamespace

import pytest
from fastapi import HTTPException

import bioxp.api as api
from bioxp import operator_controls
from bioxp.novo_router import NovoRouter
from bioxp.novo_usb_can import NovoUsbCanBus, novo_decode, novo_encode
from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_serial206_initialization import (
    Serial206OemInitializationProvider,
    Serial206ProductionPrimitiveAdapter,
)
from bioxp.services.reference_service import ReferenceStateStore
from bioxp.usb_driver import BioXpTester


class HardwareEndpoints:
    def __init__(self, hold_sync_reply=False):
        self.hold_sync_reply = hold_sync_reply
        self.sync_withheld = threading.Event()
        self.held_reply = None
        self.replies = queue.Queue()
        self.stop_delivered = threading.Event()
        self.stop_writes = 0
        self.events = []

    def write(self, frame, timeout=None):
        # Inspect the actual frame, including the existing (F04-owned) framing.
        payload = bytes(frame)[1:-2]
        board, command, motor = payload[3], payload[5], payload[7]
        if command == 3:
            self.stop_writes += 1
            self.events.append(f"stop_write_{self.stop_writes}")
            if self.stop_writes == 2:
                self.stop_delivered.set()
        value = 1000 if command == 6 else 0
        data = bytes([board, 100, command]) + value.to_bytes(4, "big") + b"\x00"
        reply = novo_encode(NovoUsbCanBus.build_payload(0, data, 8))
        if self.hold_sync_reply and self.held_reply is None and command != 3:
            self.held_reply = reply
            self.events.append("sync_reply_withheld")
            self.sync_withheld.set()
        else:
            self.replies.put(reply)
        return len(frame)

    def release_sync_reply(self):
        if self.held_reply is not None and self.hold_sync_reply:
            self.events.append("sync_reply_released")
            self.hold_sync_reply = False
            self.replies.put(self.held_reply)

    def read(self, size, timeout=None):
        try:
            return self.replies.get(timeout=(timeout or 10) / 1000)
        except queue.Empty:
            raise TimeoutError("synthetic USB receive timeout")


class ObservedRouter(NovoRouter):
    def transact(self, *args, **kwargs):
        result = super().transact(*args, **kwargs)
        self.results.append(result)
        return result


class ObservedTransportLock:
    """Observe failed Stop acquisition while retaining the underlying RLock."""

    def __init__(self, tester):
        self.lock = threading.RLock()
        self.tester = tester
        self.stop_blocked = threading.Event()

    def __enter__(self):
        if not self.lock.acquire(blocking=False):
            if threading.get_ident() == getattr(self.tester, "stop_thread_id", None):
                self.stop_blocked.set()
            self.lock.acquire()
        return self

    def __exit__(self, *args):
        self.lock.release()


class ObservedTester(BioXpTester):
    def send_tmcl(self, board_id, command, *args, **kwargs):
        if command == 3:
            self.stop_thread_id = threading.get_ident()
        return super().send_tmcl(board_id, command, *args, **kwargs)

    def motor_oem_wait_target_reached(self, *args, **kwargs):
        self.hardware.events.append("move_wait_entered")
        self.move_wait_entered.set()
        self.move_wait_result = super().motor_oem_wait_target_reached(*args, **kwargs)
        return self.move_wait_result


@pytest.mark.parametrize("hold_sync_reply", [False, True], ids=["async-wait", "sync-reply-delay"])
def test_z_stop_reaches_usb_before_ordinary_api_move_releases_lease(tmp_path, monkeypatch, hold_sync_reply):
    hardware = HardwareEndpoints(hold_sync_reply)
    tester = ObservedTester.__new__(ObservedTester)  # Never discover/open USB.
    tester.hardware = hardware
    tester.move_wait_entered = threading.Event()
    tester._transport_lock = ObservedTransportLock(tester)
    tester._motor_last_tx_ts = {}
    tester._motor_noresp_streak = {}
    tester._oem_board_initialized = {4: True}
    tester._oem_active_board_lifecycle_generation = 1
    tester._oem_position_cache = {(4, 1): 1000}
    monkeypatch.setattr(tester, "_machine_config_bundle", lambda: {
        "ok": True, "config": {
            "config": {"GripperVersion": 1}, "calibration": {"Calibrated": 1},
            "axis_limits": {"z": {"max_steps": 160000}},
            "offsets": {
                "m_Z_MOTOR_MAX_CURRENT_DOWN": 25,
                "m_Z_MOTOR_MAX_CURRENT_UP": 31,
                "m_Z_MOTOR_STALL_GUARD_THRESHOLD": 3,
            },
        },
    })
    monkeypatch.setattr("bioxp.oem_serial206_initialization.load_oem_parity_config", lambda _path: SimpleNamespace(
        blockers=[], values={"SerialNumber": 206, "CameraCalibrated": False},
        calibration_source="synthetic-f12-hardware-configuration",
    ))
    router = ObservedRouter(ep_in=hardware, ep_out=hardware, decode=novo_decode, read_timeout_ms=10)
    router.results = []
    tester.novo_router = router
    store = OEMRuntimeStore(tmp_path / "runtime")
    references = ReferenceStateStore(tmp_path / "runtime" / "references.json")
    adapter = Serial206ProductionPrimitiveAdapter(
        tester, None, authority_provider=lambda: None,
        generation_provider=lambda: 1, reference_store=references,
    )
    provider = Serial206OemInitializationProvider(
        adapter, state_store=store, reference_store=references,
        generation_provider=lambda: 1,
    )
    state = provider._load_state()
    state["z_lifecycle"].update({
        "state": "referenced_ready", "reference_state": "referenced",
        "generation": 1, "board_lifecycle_generation": 1,
    })
    provider._save_state(state)
    monkeypatch.setattr(api, "_serial206_oem_initialization_provider", provider)
    monkeypatch.setattr(api, "_tester", tester)
    monkeypatch.setattr(api, "_tester_transition_lock", asyncio.Lock())
    outcomes = {}

    def invoke(intent):
        token = operator_controls._DISPATCH_CONTEXT.set({
            "operator_command_id": f"f12-{intent}",
            "idempotency_key": f"f12-{intent}-ordering",
            "expected_ownership_generation": 1,
            "action_id": f"oem.z.{intent}",
        })
        try:
            if intent == "stop":
                outcomes[intent] = asyncio.run(api.motion_oem_z_stop())
            else:
                outcomes[intent] = api._execute_provider_z_intent(
                    intent, {"steps": 100, "wait_timeout_s": 8.0}
                )
        except HTTPException as exc:
            # Existing provider terminal-evidence policy is not F12's scope.
            outcomes[intent] = exc.detail
        except BaseException as exc:
            outcomes[intent] = exc
        finally:
            operator_controls._DISPATCH_CONTEXT.reset(token)
            hardware.events.append(f"{intent}_returned")

    move = threading.Thread(target=invoke, args=("move_steps",))
    stop = threading.Thread(target=invoke, args=("stop",))
    router.start()
    try:
        assert provider.capability_status()["initialize_motors_live_available"] is True, provider.capability_status()
        move.start()
        if hold_sync_reply:
            assert hardware.sync_withheld.wait(2), repr(outcomes)
        else:
            assert tester.move_wait_entered.wait(2), repr(outcomes)
        stop.start()
        if hold_sync_reply:
            # Signal only after a real failed lock acquisition in the Stop thread.
            assert tester._transport_lock.stop_blocked.wait(2), repr(outcomes)
            with router._pending_lock:
                pending = router._pending
                assert pending is not None and not pending.event.is_set()
            acquired = router.transaction_lock.acquire(blocking=False)
            if acquired:
                router.transaction_lock.release()
            assert not acquired
            assert hardware.stop_writes == 0
            assert not tester.move_wait_entered.is_set()
            assert move.is_alive() and "stop" not in outcomes
            hardware.events.append("stop_observed_blocked")
            hardware.release_sync_reply()  # Accepted response; no timeout override.
            assert tester.move_wait_entered.wait(2), repr(outcomes)
            held_transaction = next(row for row in router.results if row["transaction_id"] == pending.transaction_id)
            assert held_transaction["ok"] is True
            assert held_transaction["outcome"] == "completion"
            assert held_transaction["timeout_ms"] == 60000
        delivered_while_move_held = hardware.stop_delivered.wait(1)
        # No async completion event has been supplied: the real move wait still
        # owns the normal provider lifecycle lease when Stop reaches USB.
        assert move.is_alive(), outcomes
        assert "stop" not in outcomes  # reconciliation must still await the lease
        assert delivered_while_move_held, "Z Stop waited behind the ordinary API motion lease"
        assert hardware.stop_writes == 2
        assert not any(row.get("intent") == "stop" for row in provider._memory_state["z_lifecycle"]["receipts"])
    finally:
        hardware.release_sync_reply()
        # Release only the hardware completion wait, including on RED failure.
        event = bytes([4, 128, 0, 0, 0, 0, 1, 0])
        hardware.replies.put(novo_encode(NovoUsbCanBus.build_payload(0, event, 8)))
        if move.ident is not None:
            move.join(10)
        if stop.ident is not None:
            stop.join(10)
        router.shutdown()
        store.close()

    assert not move.is_alive() and not stop.is_alive(), outcomes
    assert tester.move_wait_result["target_reached"] is True
    assert isinstance(outcomes["stop"], dict), outcomes
    receipt = outcomes["stop"]["authority_receipt"]
    assert receipt["intent"] == "stop"
    assert receipt["interrupted_command_ids"] == ["f12-move_steps"]
    assert receipt["physical_effect_verified"] is False
    assert receipt["result"]["source_call_completed"] is True
    assert receipt["result"]["source_return_code"] == 0
    reopened = OEMRuntimeStore(tmp_path / "runtime")
    try:
        recovered = Serial206OemInitializationProvider(
            adapter, state_store=reopened, reference_store=references,
            generation_provider=lambda: 1,
        )._load_state()["z_lifecycle"]
        assert recovered["state"] == "failed_latched"
        assert recovered["active_receipt"] is None
        assert recovered["reference_state"] == "desynced"
        assert any(row.get("command_id") == "f12-stop" for row in recovered["receipts"])
    finally:
        reopened.close()
    assert hardware.events.index("stop_write_2") < hardware.events.index("move_steps_returned")
    if hold_sync_reply:
        assert hardware.events.index("sync_reply_withheld") < hardware.events.index("stop_observed_blocked")
        assert hardware.events.index("stop_observed_blocked") < hardware.events.index("sync_reply_released")
        assert hardware.events.index("sync_reply_released") < hardware.events.index("stop_write_1")
    print({"case": "sync-reply-delay" if hold_sync_reply else "async-wait",
           "events": hardware.events, "stop_writes": hardware.stop_writes,
           "first_transaction_outcome": router.results[0]["outcome"],
           "first_transaction_timeout_ms": router.results[0]["timeout_ms"],
           "physical_effect_verified": receipt["physical_effect_verified"]})
