"""V9's retained migration identity hashes its entire original module.

Import-only cleanup is not schema-neutral to that already-persisted identity.
Keep the frozen module byte-identical instead of rewriting retained ledgers or
loosening the existing store verification.
"""
from bioxp import oem_deck_schema_v9
from bioxp.oem_runtime_store import canonical_runtime_migration_registry


def test_v9_identity_matches_the_installed_and_retained_migration():
    expected = '3ee5098aaf0187a8b109f44d09ecb094f0d13e0535991f156129195be1c45fdc'
    actual = oem_deck_schema_v9.migration_identity()
    assert actual.version == 9
    assert actual.name == 'oem_deck_loaded_unknown_v9'
    assert actual.ddl_sha256 == expected
    registered = next(row for row in canonical_runtime_migration_registry() if row.version == 9)
    assert registered.ddl_sha256 == expected
