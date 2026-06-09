
from bioxp.oem_shadow_readback import OemShadowReadbackProvider, run_shadow_readback


class FakeProvider(OemShadowReadbackProvider):
    def axis_speed(self, axis):
        return 0
    def axis_current(self, axis):
        return {"param6": 10, "param7": 10}
    def home_switch(self, axis):
        return 0 if axis != "door" else 1
    def interlocks(self):
        return {"latch_sensor": 1, "rail_24v": {"no24v": False}}


def test_shadow_readback_captures_truth_without_motion():
    result = run_shadow_readback(FakeProvider(), axes=["x", "g", "door"])
    assert result["ok"] is True
    assert result["mode"] == "shadow_readback"
    assert result["physical_motion"] is False
    assert result["opened_usb"] == "provider_owned"
    assert result["axes"]["g"]["speed"] == 0
    assert result["axes"]["g"]["current"]["param6"] == 10
    assert result["axes"]["g"]["home_switch"]["classification"] == "HOME_ACTIVE"
    assert result["g_current_invariant"]["classification"] == "G_CURRENT_IDLE_SAFE"


class HotGProvider(FakeProvider):
    def axis_current(self, axis):
        if axis == "g":
            return {"param6": 31, "param7": 31}
        return super().axis_current(axis)


def test_shadow_readback_flags_hot_idle_g_current():
    result = run_shadow_readback(HotGProvider(), axes=["g"])
    assert result["ok"] is False
    assert result["g_current_invariant"]["classification"] == "G_CURRENT_UNSAFE_HOT_IDLE"
