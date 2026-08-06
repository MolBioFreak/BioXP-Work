
from src.bioxp.oem_stepwise_live_gates import build_stepwise_live_contract


def test_live_contract_rejects_without_ack_or_shadow_safe():
    rejected = build_stepwise_live_contract("initialize_motors", "z.axisSearchHome", operator_ack=None, shadow_readback={})
    assert rejected["ok"] is False
    assert rejected["live_allowed"] is False
    assert "operator_ack_required" in rejected["blockers"]
    rejected2 = build_stepwise_live_contract("initialize_motors", "z.axisSearchHome", operator_ack="OEM_STEPWISE_LIVE", shadow_readback={"ok": False})
    assert rejected2["ok"] is False
    assert "shadow_readback_not_safe" in rejected2["blockers"]


def test_live_contract_requires_named_step_and_preserves_motion_warning():
    shadow = {"ok": True, "g_current_invariant": {"classification": "G_CURRENT_IDLE_SAFE"}}
    contract = build_stepwise_live_contract("initialize_motors", "z.axisSearchHome", operator_ack="OEM_STEPWISE_LIVE", shadow_readback=shadow)
    assert contract["ok"] is True
    assert contract["live_allowed"] is True
    assert contract["program"] == "initialize_motors"
    assert contract["step_id"] == "z.axisSearchHome"
    assert contract["operator_ack"] == "OEM_STEPWISE_LIVE"
    assert contract["motion_may_be_commanded"] is True
    assert contract["requires_operator_present"] is True
