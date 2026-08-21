from src.bioxp.api import app


def test_serial206_y_typed_routes_are_registered_with_closed_paths():
    routes = {
        (route.path, tuple(sorted(route.methods or [])))
        for route in app.routes
        if hasattr(route, "path") and hasattr(route, "methods")
    }
    assert ("/motion/oem/y/status", ("GET",)) in routes
    assert ("/motion/oem/y/prepare", ("POST",)) in routes
    assert ("/motion/oem/y/move_steps", ("POST",)) in routes
    assert ("/motion/oem/y/move_absolute", ("POST",)) in routes
    assert ("/motion/oem/y/home", ("POST",)) in routes
    assert ("/motion/oem/y/set_home", ("POST",)) in routes
    assert ("/motion/oem/y/stop", ("POST",)) in routes
    assert not any(path == "/motion/oem/y/move_absolute/terminalize" for path, _ in routes)


def test_serial206_y_request_models_reject_extra_fields():
    from src.bioxp.api import OemYMoveAbsoluteRequest, OemYMoveStepsRequest

    assert OemYMoveStepsRequest(steps=20).steps == 20
    assert OemYMoveAbsoluteRequest(target_steps=0).target_steps == 0
    try:
        OemYMoveAbsoluteRequest(position_steps=0, speed=1800)
    except Exception as exc:
        assert "extra" in str(exc).lower()
    else:
        raise AssertionError("extra Y profile inputs must be rejected")


def test_internal_y_absolute_sources_have_closed_typed_models():
    from src.bioxp.api import OemYAccelerationOverloadRequest, OemYBoardTestMyRequest

    overload = OemYAccelerationOverloadRequest(target_steps=100, acceleration_override=250)
    assert overload.acceleration_override == 250
    assert OemYBoardTestMyRequest(target_steps=100).target_steps == 100
    try:
        OemYBoardTestMyRequest(target_steps=100, acceleration_override=250)
    except Exception as exc:
        assert "extra" in str(exc).lower()
    else:
        raise AssertionError("board_test_my must own its fixed source acceleration")


def test_legacy_generic_y_mutation_routes_are_retired_without_controller_access():
    from fastapi.testclient import TestClient

    with TestClient(app) as client:
        assert client.post("/motion/oem/manual/relative", json={"axis": "y", "steps": 100}).status_code == 410
        assert client.post("/motion/oem/manual/absolute", json={"axis": "y", "position_steps": 100}).status_code == 410
        assert client.post("/motion/oem/manual/home", json={"axis": "y"}).status_code == 410
        assert client.post("/motion/oem/manual/sethome", json={"axis": "y", "operator_ack": "SET_HOME_CURRENT_POSITION"}).status_code == 410
        legacy_command = {
            "schema_version": "bioxp.operator_command_request.v1",
            "idempotency_key": "legacy-y-command-1",
            "expected_ownership_generation": 0,
            "action_id": "oem.y.move_steps",
            "inputs": {"steps": 100},
        }
        assert client.post("/operator/commands", json=legacy_command).status_code == 410
        legacy_method = {
            "schema_version": "bioxp.operator_method_request.v1",
            "name": "legacy-y",
            "idempotency_key": "legacy-y-method-1",
            "expected_ownership_generation": 0,
            "failure_policy": "fail_fast",
            "steps": [{"action_id": "oem.y.move_steps", "inputs": {"steps": 100}, "repeat": 1}],
            "metadata": {},
        }
        assert client.post("/operator/methods", json=legacy_method).status_code == 410
        legacy_action = {"expected_generation": 0, "idempotency_key": "legacy-y-action-1", "inputs": {"steps": 100}}
        assert client.post("/operator/actions/oem.y.move_steps", json=legacy_action).status_code == 410
