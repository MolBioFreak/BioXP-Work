
from bioxp.oem_parity_predicates import SWITCH_PREDICATES, classify_home_switch, predicate_matrix


def test_switch_matrix_declares_all_axes_and_polarity():
    matrix = predicate_matrix()
    assert set(matrix) == {"x", "y", "z", "g", "door"}
    assert matrix["x"].home_query == "queryHome"
    assert matrix["x"].home_active_value == 0
    assert matrix["door"].left_switch_can_be_disabled is True
    assert matrix["g"].requires_current_restore is True


def test_classify_home_switch_fail_closed_for_unknown_axis_or_value():
    assert classify_home_switch("x", 0)["home"] is True
    assert classify_home_switch("x", 1)["home"] is False
    assert classify_home_switch("x", None)["classification"] == "UNKNOWN_SWITCH_TRUTH"
    assert classify_home_switch("bogus", 0)["classification"] == "UNKNOWN_AXIS"
