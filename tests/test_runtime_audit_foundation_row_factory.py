import sqlite3

from bioxp.runtime_audit_store import (
    _expected_foundation_connection,
    verify_runtime_audit_foundation,
)


def test_foundation_verification_accepts_default_tuple_row_factory():
    connection = _expected_foundation_connection()
    try:
        connection.row_factory = None
        verify_runtime_audit_foundation(connection)
        assert connection.row_factory is None
    finally:
        connection.close()
