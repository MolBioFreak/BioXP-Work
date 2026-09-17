"""Compute migration's new exact physical-schema fingerprint offline."""
from bioxp import oem_runtime_store as owner
from tests.test_deck_scoped_authority import retained_rig


def test_retained_workflow_schema(retained_rig):
    assert retained_rig is not None


def test_workflow_schema_fingerprint():
    connection = owner._expected_foundation_connection()
    connection.create_function('authority_write_allowed', 0, lambda: 1)
    owner._create_v1_runtime_schema(connection)
    owner._create_v2_authority_schema(connection)
    owner._apply_operator_command_plane_schema_v1(connection)
    for statement in owner._REPORT_IDENTITY_TRIGGER_DDL:
        connection.execute(statement)
    assert owner._runtime_physical_schema_sha256(connection) == owner._RUNTIME_PHYSICAL_SCHEMA_SHA256_BY_VERSION[10]
    owner._apply_workflow_schema(connection)
    actual = owner._runtime_physical_schema_sha256(connection)
    assert actual == owner._RUNTIME_PHYSICAL_SCHEMA_SHA256_BY_VERSION.get(11), actual
