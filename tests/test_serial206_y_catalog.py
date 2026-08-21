from src.bioxp.api import app
from src.bioxp.operator_controls import _build_catalog


def test_operator_catalog_contains_canonical_y_actions_and_exact_bounds():
    actions, dispatch = _build_catalog(app)
    by_id = {row["action_id"]: row for row in actions}
    for action_id in ("oem.y.status", "oem.y.prepare", "oem.y.move_steps", "oem.y.move_absolute", "oem.y.manual_panel_home", "oem.y.stop"):
        assert action_id in by_id
        assert action_id in dispatch or action_id == "oem.y.stop"
    relative = {row["name"]: row for row in by_id["oem.y.move_steps"]["inputs"]}
    absolute = {row["name"]: row for row in by_id["oem.y.move_absolute"]["inputs"]}
    assert relative["steps"]["minimum"] == -102936
    assert relative["steps"]["maximum"] == 102936
    assert absolute["target_steps"]["maximum"] == 102956
    assert by_id["oem.y.manual_panel_home"]["inputs"] == []
    assert dispatch["oem.y.manual_panel_home"]["fixed_inputs"]["source_mode"] == "manual_panel"
