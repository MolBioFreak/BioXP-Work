"""Ordinary OEM roster closure through the real API factory; no hardware.

This is binding coverage only. Connected execution and native recorder cases
qualify the bodies separately; synthetic source settings are not live readiness.
"""
from bioxp import api
from bioxp.protocols.executor import ProtocolExecutor
from bioxp.protocols.models import OEM_OPERATION_FORMS
from tests.test_protocol_oem_bindings import rig, document


def test_actual_factory_binds_complete_ordinary_lifecycle_and_opcode_roster(rig, monkeypatch):
    _, _, _, trace = rig
    def no_entry(*args, **kwargs):
        raise AssertionError("Binding discovery must not invoke a native adapter")
    # Installed-adapter prerequisites only; the connected suite exercises the
    # actual adapter implementation and canonical claims rather than this guard.
    monkeypatch.setattr(api.app.state, "oem_workflow_lifecycle_control_executor", no_entry, raising=False)
    monkeypatch.setattr(api.app.state, "oem_mov_execution_admitter", no_entry, raising=False)
    doc = document("delaypoint", settings={"JobName": None})
    bindings = api._protocol_bindings({"protocol": {"document": doc.to_payload()}})
    ordinary, native, lifecycle = bindings
    executor = ProtocolExecutor(
        dry_run=False, handlers=ordinary, oem_handlers=native,
        lifecycle_handlers=lifecycle,
        source_script_begin=bindings.source_script_begin,
        source_script_returned=bindings.source_script_returned,
        before_native_entry=lambda *args: (_ for _ in ()).throw(
            AssertionError("Binding discovery must not enter hardware")),
    )
    assert trace == []
    assert executor.preflight(doc)["missing"] == []
    assert set(OEM_OPERATION_FORMS) - {"step", "wait", "delaypoint"} <= set(native)
    assert callable(bindings.source_script_begin)
    assert callable(bindings.source_script_returned)
    assert trace == []


def test_optional_prepare_is_not_falsely_advertised_as_completed(rig):
    # Ordinary lifecycle closure must not accidentally claim optional barcode/CV
    # preparation; that was explicitly outside the orchestration implementation.
    doc = document("delaypoint", settings={"JobName": None})
    bindings = api._protocol_bindings({"protocol": {"document": doc.to_payload()}})
    assert "prepare" not in bindings[2]
