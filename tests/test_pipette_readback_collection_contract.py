"""Real collection readback/snapshot with hardware query methods replaced only."""
from copy import deepcopy
from types import SimpleNamespace

import pytest
from pydantic import ValidationError

from bioxp.pipette.direct_requests import PipetteReadbackResponse
from bioxp.pipette.transport import CanPipetteTransport, FourPipetteTransport


class QueryDriver:
    def __init__(self, channel, known):
        self.channel = channel
        self.bus = SimpleNamespace(router=SimpleNamespace(reader_generation=1))
        self._pipette_message_state = {}
        if known:
            self._pipette_message_state.update(
                tip_loaded=False, tip_source_actor=f"offline-{channel}",
                tip_source_revision=4, tip_source_verified=True,
                tip_source_transaction_id=f"query-{channel}",
                tip_source_reader=id(self.bus.router), tip_source_reader_generation=1,
            )

    def query_firmware(self, number):
        assert number == 1
        return {"ok": True, "semantic_ok": True, "value": "offline firmware"}

    def query_status(self):
        return {"ok": True, "semantic_ok": True, "value": 0}

    def query_tip_status(self):
        return {"ok": True, "semantic_ok": True, "tip_loaded": False}


@pytest.fixture
def response():
    def produce(known=True):
        drivers = [QueryDriver(channel, known) for channel in range(4)]
        collection = FourPipetteTransport([
            CanPipetteTransport(driver_factory=lambda driver=driver: driver, pipette_id=channel)
            for channel, driver in enumerate(drivers)
        ], sleep=lambda _: None)
        result = collection.readback_all(include_data=False)
        # run_pipette_operation attaches this actual source snapshot to readbacks.
        result["collection_source"] = collection.collection_source_snapshot()
        result["receipt_id"] = "a" * 32
        result["receipt_truth"] = {
            "delivery_verified": False, "controller_acknowledged": False,
            "completion_verified": False, "semantic_query_response_verified": True,
            "hardware_precondition_verified": False, "hardware_postcondition_verified": False,
            "physical_effect_verified": False, "physical_effect_claim_suppressed": True,
        }
        return result
    return produce


@pytest.mark.parametrize("known", [True, False])
def test_real_collection_source_survives_readback_contract(response, known):
    result = response(known)
    parsed = PipetteReadbackResponse.model_validate(result).model_dump()
    assert parsed == result
    assert parsed["collection_source"]["channels"] == [
        {"tip_loaded": False if known else None, "verified": known}
    ] * 4
    assert not parsed["physical_effect_verified"]
    assert not parsed["receipt_truth"]["physical_effect_verified"]


def test_legacy_readback_without_source_remains_valid(response):
    result = response()
    del result["collection_source"]
    assert PipetteReadbackResponse.model_validate(result).model_dump(exclude_unset=True) == result


def test_explicit_unavailable_source_remains_valid(response):
    result = response()
    result["collection_source"] = None
    assert PipetteReadbackResponse.model_validate(result).collection_source is None


@pytest.mark.parametrize("mutation", [
    lambda source: source.update(unexpected=True),
    lambda source: source["channels"].pop(),
    lambda source: source["identity"]["channels"].pop(),
    lambda source: source["channels"][0].update(tip_loaded="false"),
    lambda source: source["channels"][0].update(verified=1),
    lambda source: source["identity"]["channels"][0].update(revision=True),
    lambda source: source["identity"]["channels"][0].update(reader_generation="1"),
])
def test_source_shape_stays_closed_without_coercion(response, mutation):
    result = deepcopy(response())
    mutation(result["collection_source"])
    with pytest.raises(ValidationError):
        PipetteReadbackResponse.model_validate(result)


def test_unrelated_extra_and_physical_claim_still_rejected(response):
    for field, value in [("unrelated", True), ("physical_effect_verified", True)]:
        result = response()
        result[field] = value
        with pytest.raises(ValidationError):
            PipetteReadbackResponse.model_validate(result)
