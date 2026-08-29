from src.bioxp.api import app


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
