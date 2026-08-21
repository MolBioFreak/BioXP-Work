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


def test_legacy_generic_y_mutation_routes_are_retired_without_controller_access():
    from fastapi.testclient import TestClient

    with TestClient(app) as client:
        assert client.post("/motion/oem/manual/relative", json={"axis": "y", "steps": 100}).status_code == 410
        assert client.post("/motion/oem/manual/absolute", json={"axis": "y", "position_steps": 100}).status_code == 410
        assert client.post("/motion/oem/manual/home", json={"axis": "y"}).status_code == 410
        assert client.post("/motion/oem/manual/sethome", json={"axis": "y", "operator_ack": "SET_HOME_CURRENT_POSITION"}).status_code == 410
