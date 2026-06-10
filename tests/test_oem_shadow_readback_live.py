
from bioxp.oem_shadow_readback_live import build_shadow_readback_artifact


class FakeProvider:
    def axis_snapshot(self, axis):
        return {
            "position": 10 if axis == "x" else 0,
            "speed": 0,
            "gap9_left_raw": 1 if axis == "x" else 0,
            "gap10_right_raw": 0,
            "left_disabled": False,
            "right_disabled": True,
            "run_current": 10 if axis == "g" else 31,
            "standby_current": 10 if axis == "g" else 20,
        }

    def interlocks(self):
        return {"latch_sensor_raw": 1, "rail_24v_no24v": False, "override_latch_sensor": False, "override_rail_24v": False}

    def reference_state(self):
        return {"x": "desynced", "y": "desynced"}


def test_shadow_readback_artifact_is_query_only_and_preserves_raw_truth():
    artifact = build_shadow_readback_artifact(FakeProvider(), axes=("x", "g"))
    assert artifact["ok"] is True
    assert artifact["motion_commanded"] is False
    assert artifact["current_mutation_commanded"] is False
    assert artifact["switch_mask_mutation_commanded"] is False
    assert artifact["axes"]["x"]["gap9_left_raw"] == 1
    assert artifact["axes"]["x"]["right_disabled"] is True
    assert artifact["axes"]["x"]["right_active_effective"] is False
    assert artifact["g_current_invariant"]["classification"] == "G_CURRENT_IDLE_SAFE"


def test_shadow_readback_fails_closed_on_hot_idle_g_current():
    class HotProvider(FakeProvider):
        def axis_snapshot(self, axis):
            row = super().axis_snapshot(axis)
            if axis == "g":
                row["run_current"] = 31
                row["standby_current"] = 31
            return row
    artifact = build_shadow_readback_artifact(HotProvider(), axes=("g",))
    assert artifact["ok"] is False
    assert artifact["failed_closed"] is True
    assert artifact["g_current_invariant"]["classification"] == "G_CURRENT_UNSAFE_HOT_IDLE"
