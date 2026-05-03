from fastapi.testclient import TestClient


def test_oem_compat_capability_matrix_exposes_test_prep_truth():
    from src.bioxp.api import app

    client = TestClient(app)
    response = client.get("/oem-compat/capabilities/test-prep")

    assert response.status_code == 200
    body = response.json()
    assert body["schema_version"] == "bioxp.oem_compat.capability_matrix.v1"
    assert body["robot_hardware_assumption"] == "functional_under_oem"
    assert body["truth_source"] == "robot_local_oem_compat_layer"
    assert body["bms_role"] == "thin_operator_surface"
    assert body["capabilities"]["motion"]["prep_ready"] is True
    assert body["capabilities"]["deck_semantics"]["prep_ready"] is True
    assert body["capabilities"]["oem_xml_jobs"]["supported_verbs_include"] == ["MT", "FP"]
    assert body["capabilities"]["pipette"]["ack_readback_required"] is True
    assert body["capabilities"]["vision"]["artifacted_failures"] is True
