"""Compact receipt fallback: the legacy history reader must expose the
``get_command_summary`` entry point used by non-detail V2 receipt reads.

Regression for the fallback path in ``operator_controls`` where a command is
absent from both the retained projection and the command plane: the legacy
store previously lacked the summary entry point, so a compact poll raised
AttributeError instead of returning the projected receipt.
"""
from bioxp.operator_receipt_store import OperatorHistoryReader


def test_legacy_reader_exposes_summary_entry_point(tmp_path):
    reader = OperatorHistoryReader(root=tmp_path)
    assert hasattr(reader, "get_command_summary")
    # No legacy database exists in a fresh root: both reads agree on None.
    assert reader.get_command("missing") is None
    assert reader.get_command_summary("missing") is None


def test_summary_delegates_to_get_command(tmp_path, monkeypatch):
    reader = OperatorHistoryReader(root=tmp_path)
    sentinel = {"command_id": "c", "status": "completed"}
    monkeypatch.setattr(reader, "get_command", lambda command_id: sentinel)
    assert reader.get_command_summary("c") is sentinel
