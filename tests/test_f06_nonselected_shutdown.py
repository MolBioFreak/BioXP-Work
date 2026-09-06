"""S1: offline pre-generation shutdown window, not physical delivery proof.

The real reader decodes/dispatches endpoint bytes. Only the pending Event wait
boundary is controlled: after terminal Set (or a false wait), request stop before
transact constructs its result, without running shutdown's join/generation tail.
"""
from contextlib import contextmanager
import queue
import threading
from types import SimpleNamespace

import pytest

from bioxp.novo_router import NovoRouter
from bioxp.novo_usb_can import NovoUsbCanBus, novo_decode, novo_encode


@contextmanager
def transport(kind, boundary="stop"):
    incoming = queue.Queue()
    classified = threading.Event()
    waits = []

    class Input:
        def read(self, size, timeout):
            try:
                return incoming.get(timeout=.005)
            except queue.Empty:
                raise TimeoutError("offline idle")

    class Wait:
        def __init__(self, event):
            self.event = event

        def set(self):
            self.event.set()

        def is_set(self):
            return self.event.is_set()

        def wait(self, timeout):
            waits.append(timeout)
            if kind in {"completion", "ack"}:
                assert self.event.wait(.5), "real reader did not dispatch terminal"
                assert router._pending is None
                assert endpoint.pending.outcome == kind
                completed = True
            else:
                if kind == "malformed":
                    assert classified.wait(.5), "real reader did not classify malformed"
                completed = False
            assert router.reader_generation == generation
            if boundary == "stop":
                router._stop.set()
            elif boundary == "generation":
                router._reader_generation += 1
            elif boundary == "pending_shutdown":
                endpoint.pending.outcome = "shutdown"
                self.event.set()
                completed = True
            return completed

    class Output:
        def __init__(self):
            self.writes = []

        def write(self, raw, timeout):
            self.writes.append((bytes(raw), timeout))
            assert len(self.writes) == 1, "nonselected or invalidated call resubmitted"
            self.pending = router._pending
            self.pending.event = Wait(self.pending.event)
            if kind in {"completion", "ack"}:
                function, data = (6, b"\x20\x60\x31") if kind == "completion" else (1, b"")
                incoming.put(novo_encode(NovoUsbCanBus.build_payload(0x500 | function, data)))
            elif kind == "malformed":
                incoming.put(b"not a Novo frame")
            return len(raw)

    endpoint = Output()
    router = NovoRouter(ep_in=Input(), ep_out=endpoint, decode=novo_decode)
    # Observe completed real skipped-frame publication; do not synthesize it.
    record_skipped = router._record_skipped

    def skipped(row):
        record_skipped(row)
        classified.set()

    router._record_skipped = skipped
    router.start()
    generation = router.reader_generation
    shared = SimpleNamespace(ep_in=router.ep_in, ep_out=endpoint, novo_router=router)
    bus = NovoUsbCanBus(shared_usb=shared)
    try:
        yield router, endpoint, bus, waits, generation
    finally:
        router.shutdown()


@pytest.mark.parametrize("kind", ["completion", "ack"])
def test_pipette_terminal_survives_stop_before_generation(kind):
    with transport(kind) as (router, endpoint, bus, waits, generation):
        # Spy on the real projection entering the real adapter, without replacing
        # transact, matcher, decoder, dispatch or adapter conversion.
        projected = []
        finish = bus._finish_pipette_response

        def capture(**kwargs):
            projected.append(kwargs["response"])
            return finish(**kwargs)

        bus._finish_pipette_response = capture
        function, data = (6, b"?31") if kind == "completion" else (1, b"P1,1R")
        result = bus.transact_can(
            SimpleNamespace(arbitration_id=0x100 | function, data=data, dlc=len(data)),
            channel=0, expected_function=function, timeout_s=1.0,
            matcher_name="s1-pipette", wait_for_completion=False,
        )
        response = projected[0]
        assert router.reader_generation == generation
        assert router._stop.is_set()
        assert type(response["command_family"]) is int
        assert response["outcome"] == kind
        assert response["ok"] is (kind == "completion")
        assert response["completion_received"] is (kind == "completion")
        assert response["ack_received"] is (kind == "ack")
        assert response["frames"][0]["classification"] == "pipette"
        assert response["observed_rx_id"] == 0x500 | function
        assert response["observed_rx_dlc"] == (3 if kind == "completion" else 0)
        assert result["outcome"] == kind and result["ok"] is True
        assert result["completion_received"] is False
        if kind == "completion":
            assert result["semantic_query_response_verified"] is True
        else:
            assert result["immediate_ack_received"] is True
            assert result["controller_acknowledged"] is True
            assert result["completion_deferred"] is True
        assert len(endpoint.writes) == 1 and waits == [1.0]


@pytest.mark.parametrize("kind", ["timeout", "malformed"])
@pytest.mark.parametrize("boundary", ["stop", "generation", "pending_shutdown"])
def test_nonselected_false_wait_preserves_classification_and_lifecycle(kind, boundary):
    with transport(kind, boundary) as (router, endpoint, bus, waits, generation):
        result = router.transact(
            novo_encode(bus.build_payload(0x106, b"?31")),
            matcher=router.pipette_matcher(channel=0, expected_function=6),
            matcher_name="s1-query", timeout_s=1.0, write_timeout_ms=2000,
            provenance={"channel": 0, "command_family": 6},
        )
        expected = {"stop": kind, "generation": "transport_rebound",
                    "pending_shutdown": "shutdown"}[boundary]
        assert result["outcome"] == expected
        assert result["ok"] is False
        assert result["completion_received"] is False
        assert result["ack_received"] is False
        assert result["skipped_count"] == (1 if kind == "malformed" else 0)
        assert len(endpoint.writes) == 1 and waits == [1.0]
        assert router._pending is None


def test_selected_false_wait_stop_flag_still_prevents_second_write():
    with transport("timeout") as (router, endpoint, bus, waits, generation):
        result = router.transact(
            novo_encode(bus.build_payload(5, bytes([6, 1, 0, 0, 0, 0, 0]))),
            matcher=router.tmcl_matcher(board_id=5, command=6),
            matcher_name="s1-selected", timeout_s=60.0, write_timeout_ms=80,
            provenance={"command_family": "tmcl"}, ordinary_motor_retry=True,
        )
        assert router.reader_generation == generation
        assert result["outcome"] == "shutdown" and result["ok"] is False
        assert len(endpoint.writes) == len(result["attempts"]) == 1
        assert result["attempts"][0]["wait_signaled"] is False
        assert waits == [60.0]
