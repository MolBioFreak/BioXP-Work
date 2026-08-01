from __future__ import annotations


def test_api_exposes_fail_closed_serial206_provider_binding_contract():
    import src.bioxp.api as api

    assert hasattr(api, "bind_serial206_oem_initialization_provider")
    assert hasattr(api, "serial206_oem_initialization_provider_status")
    assert "stage_approvals" in api.OemInitializationRunRequest.model_fields
    assert "commissioning" in api.OemInitializationRunRequest.model_fields
    assert "motor_stage_approvals" in api.OemInitializeMotionRequest.model_fields
    assert "motion_stage_approvals" in api.OemInitializeMotionRequest.model_fields
