from __future__ import annotations

from typing import Any, cast

from src.bioxp.can_driver import BioXpCanDriver, process_pipette_message
from src.bioxp.novo_router import NovoRouter


def test_driver_process_message_publishes_nonzero_oem_error_event():
    driver = BioXpCanDriver.__new__(BioXpCanDriver)
    events = []
    driver.pipette_id = 3
    driver._pipette_message_state = {}
    driver._pipette_error_callback = lambda channel, code: events.append((channel, code))

    result = driver.process_pipette_message(
        3,
        [0x07, 0x60, 0x00],
        arbitration_id=0x519,
        command_name="dispense",
    )

    assert result["oem_error_code"] == 0x07
    assert events == [(3, 0x07)]


def test_four_channel_collection_publishes_driver_errors_and_retains_last_error():
    from src.bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport

    callbacks = []
    transports = []
    for channel in range(4):
        driver = BioXpCanDriver.__new__(BioXpCanDriver)
        driver.pipette_id = channel
        driver._pipette_message_state = {}
        driver._pipette_last_command = None
        transports.append(
            CanPipetteTransport(
                driver_factory=lambda driver=driver: driver,
                pipette_id=channel,
            )
        )
    collection = FourPipetteTransport(transports, error_callback=lambda channel, code: callbacks.append((channel, code)))
    driver = collection._transports[2]._get_driver()
    result = driver.process_pipette_message(
        3,
        [0x07, 0x60, 0x00],
        arbitration_id=0x511,
        command_name="dispense",
    )

    assert result["oem_error_code"] == 0x07
    assert callbacks == [(2, 0x07)]
    assert collection.get_status()["last_error"] == {
        "channel": 2,
        "error_code": 0x07,
        "source": "ClassPipetteCollection.handlePipetteMessage",
    }


def test_pipette_matcher_rejects_middle_part_before_first_part():
    from src.bioxp.novo_router import NovoFrame

    matcher = NovoRouter.pipette_matcher(
        channel=0,
        expected_function=1,
        allow_multipart=True,
    )
    middle = NovoFrame(
        arbitration_id=0x504,
        dlc=8,
        data=b"middle!!",
        raw=b"",
        received_at=0.0,
        classification="pipette",
    )
    first = NovoFrame(
        arbitration_id=0x503,
        dlc=8,
        data=b"first!!!",
        raw=b"",
        received_at=0.0,
        classification="pipette",
    )
    final = NovoFrame(
        arbitration_id=0x501,
        dlc=3,
        data=b"end",
        raw=b"",
        received_at=0.0,
        classification="pipette",
    )

    assert matcher(middle).matched is False
    assert matcher(middle).classification == "pipette_multipart_wrong_order"
    assert matcher(first).matched is True
    assert matcher(first).terminal is False
    assert matcher(final).matched is True
    assert matcher(final).terminal is True



def test_eject_all_tips_retries_remaining_tips_like_oem_verify_eject():
    from src.bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport

    class Driver:
        def __init__(self):
            self.tip_loaded = True
            self.eject_calls = 0
            self.wait_calls = 0

        def query_tip_status(self):
            return {
                "ok": True,
                "semantic_ok": True,
                "hardware_truth_level": "hardware_query",
                "tip_loaded": self.tip_loaded,
            }

        def pipette_eject_tip(self, *, initialized, wait_for_completion=True):
            del initialized, wait_for_completion
            self.eject_calls += 1
            if self.eject_calls >= 2:
                self.tip_loaded = False
            return {"ok": True, "ack": {"outcome": "completion"}}

        def wait_pipette_command_completion(self, timeout_s):
            del timeout_s
            self.wait_calls += 1
            return {"ok": True, "outcome": "completion"}

    drivers = [Driver() for _ in range(4)]
    transports = [
        CanPipetteTransport(driver_factory=lambda driver=driver: driver, pipette_id=channel)
        for channel, driver in enumerate(drivers)
    ]
    sleeps = []
    collection = FourPipetteTransport(
        transports,
        sleep=sleeps.append,
        liquid_mutation_enabled=True,
    )
    for transport in transports:
        transport._initialized = True

    result = collection.eject_all_tips(check_missing_tip=True, wait=False)

    assert result["ok"] is True
    assert result["after"]["tip_count"] == 0
    assert [driver.eject_calls for driver in drivers] == [2, 2, 2, 2]
    assert 0.5 in sleeps



def test_initialization_accepts_router_immediate_ack_before_delayed_completion():
    class Bus:
        def transact_can(self, message, **kwargs):
            del message, kwargs
            return {
                "ok": False,
                "outcome": "ack",
                "frames": [
                    {
                        "arbitration_id": 0x500,
                        "dlc": 0,
                        "data": [],
                        "raw": [],
                        "received_at": 1.0,
                    }
                ],
                "receive_timestamp": 1.0,
            }

    driver = BioXpCanDriver.__new__(BioXpCanDriver)
    driver.bus = Bus()
    driver.pipette_id = 0
    driver.response_timeout_s = 1.0
    driver._pipette_message_state = {}
    driver._pipette_last_command = None
    driver._pipette_error_callback = None

    result = driver._send_packet(
        0x101,
        [ord("W"), ord("R")],
        require_ack=True,
        command_name="pipette_initialize",
    )

    assert result["ok"] is True
    assert result["ack"]["outcome"] == "ack"



def test_transport_status_exposes_persistent_oem_message_state():
    from types import SimpleNamespace

    from src.bioxp.pipette.transport import CanPipetteTransport

    transport = CanPipetteTransport(driver_factory=lambda: None, pipette_id=0)
    transport._driver = SimpleNamespace(
        _pipette_message_state={
            "initialization_counter": 3,
            "diagnosis": "diagnostic-ok",
            "error_queue": [0x45],
        }
    )

    payload = transport._status_payload()

    assert payload["oem_initialization_counter"] == 3
    assert payload["oem_diagnosis"] == "diagnostic-ok"
    assert payload["oem_error_queue"] == [0x45]



def test_group_mix_is_one_channel_without_constituent_completion_waits():
    from src.bioxp.pipette.models import PipetteMixCommand
    from src.bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport

    class Driver:
        def __init__(self):
            self.wait_values = []

        def query_tip_status(self):
            return {
                "ok": True,
                "semantic_ok": True,
                "hardware_truth_level": "hardware_query",
                "tip_loaded": True,
            }

        def mix(self, volume, count, *, tip_pressure_profile, wait_for_completion):
            del volume, count, tip_pressure_profile
            self.wait_values.append(bool(wait_for_completion))
            return {"ok": True, "cycles": []}

    drivers = [Driver() for _ in range(4)]
    transports = [
        CanPipetteTransport(driver_factory=lambda driver=driver: driver, pipette_id=channel)
        for channel, driver in enumerate(drivers)
    ]
    for transport in transports:
        transport._initialized = True
    collection = FourPipetteTransport(transports, sleep=lambda _seconds: None, liquid_mutation_enabled=True)
    collection.loadTip(200, 2)

    result = collection.mix(PipetteMixCommand(volume_ul=2.0, cycles=2))

    assert result["ok"] is True
    assert result["wait_policy"] == "oem_composite_sequence_no_constituent_completion_waits"
    assert [driver.wait_values for driver in drivers] == [[], [], [False], []]



def test_aspirate_air_matches_oem_no_tip_precondition_and_tracks_rear_air():
    from src.bioxp.pipette.transport import CanPipetteTransport

    class Driver:
        def aspirate_air(self, volume, *, air_type, wait_for_completion):
            assert volume == 5.0
            assert air_type == 1
            assert wait_for_completion is True
            return {"ok": True, "outcome": "completion"}

    transport = CanPipetteTransport(driver_factory=Driver, pipette_id=0)
    transport._initialized = True
    result = transport.aspirate_air(5, front_air=False)

    assert result["ok"] is True
    assert result["rear_air_level_ul"] == 5.0
    assert result["hardware_tip_status"] is None


def test_process_message_tracks_init_counter_and_preserves_queue_reset_semantics():
    initialized = process_pipette_message(
        3,
        [0x20, 0x60, 0x20],
        command_name="pipette_initialize",
    )
    assert initialized["oem_error_code"] == 0x20
    assert initialized["initialization_counter"] == 1

    status = process_pipette_message(
        3,
        [0x20, 0x60, 0x20],
        command_name="query_status",
        state={"error_queue": [0x07], "initialization_counter": 1},
    )
    assert status["oem_error_code"] == 0x20
    assert status["error_queue"] == []


def test_novo_router_transact_many_writes_each_encoded_frame_in_order():
    class Endpoint:
        def __init__(self):
            self.writes = []

        def write(self, payload, timeout):
            self.writes.append((bytes(payload), int(timeout)))
            return len(payload)

    class RunningReader:
        def is_alive(self):
            return True

        def join(self, timeout):
            del timeout

    endpoint = Endpoint()
    router = NovoRouter(ep_in=object(), ep_out=endpoint, decode=lambda raw: raw)
    cast(Any, router)._reader = RunningReader()
    result = router.transact_many(
        [b"first", b"second", b"final"],
        matcher=None,
        matcher_name="multipart-test",
        timeout_s=1.0,
        write_timeout_ms=17,
        provenance={"test": True},
    )

    assert result["ok"] is True
    assert [payload for payload, _timeout in endpoint.writes] == [b"first", b"second", b"final"]
    assert all(timeout == 17 for _payload, timeout in endpoint.writes)
    assert result["tx_frame_count"] == 3
    assert result["tx_write_policy"] == "one_frame_per_oem_sendcommand"
