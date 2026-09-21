"""Offline persistence failures must not become usable Z reference authority."""
from types import SimpleNamespace

import pytest

from bioxp.oem_runtime_store import OEMRuntimeStore
from bioxp.oem_serial206_initialization import Serial206OemInitializationProvider as Provider


INTENTS = ("manual_home", "move_z_home", "diagnostic_home_axis", "set_home", "self_test", "resume_after_abort")


def make_provider(tmp_path, *, fault=None, invalidation_fails=False):
    calls = []
    class References:
        state = "desynced"
        def mark_referenced(self, command):
            if fault == "raise":
                raise OSError("offline reference publication failure")
            if fault == "unverified":
                return {"ok": True, "state": "referenced", "persisted": False}
            self.state = "referenced"
            return {"ok": True, "state": self.state, "persisted": True, "state_version": 2}
        def mark_desynced(self, command):
            if invalidation_fails:
                raise OSError("offline reference invalidation failure")
            self.state = "desynced"
            return {"ok": True, "state": self.state, "persisted": True}
    def native(**kw):
        calls.append(dict(kw))
        return {"ok": True, "source_return_ok": True, "source_return_code": 0,
            "controller_command_acknowledged": True, "controller_terminal_state_verified": True,
            "home_summary": {"controller_home_proof_verified": True}, "self_test_pass": True}
    primitive = SimpleNamespace(**{name: native for name in (
        "z_manual_home", "z_move_z_home", "z_diagnostic_home_axis", "z_set_home", "z_self_test", "z_resume_after_abort")})
    runtime = OEMRuntimeStore(tmp_path)
    references = References()
    provider = Provider(primitive, state_store=runtime, reference_store=references,
        generation_provider=lambda: 7,
        preparation_provider=SimpleNamespace(current_board_lifecycle_generation=lambda: 3))
    state = provider._new_state()
    state["x_lifecycle"]["board_lifecycle_generation"] = 3
    state["z_lifecycle"].update(state="prepared_unreferenced", reference_state="desynced",
                               generation=7, board_lifecycle_generation=3)
    provider._save_state(state)
    return provider, references, runtime, calls


@pytest.mark.parametrize("intent", INTENTS)
@pytest.mark.parametrize("fault", ["raise", "unverified"])
def test_reference_failure_does_not_publish_ready(tmp_path, intent, fault):
    provider, refs, runtime, calls = make_provider(tmp_path, fault=fault)
    try:
        result = provider.execute_z_intent(intent, expected_generation=7, idempotency_key="offline-home")
        assert result["ok"] is False
        assert result["result"]["failure"] == "z_reference_persistence_failed"
        assert result["result"]["reference_persistence_ok"] is False
        assert result["result"]["source_return_ok"] is True
        assert result["result"]["controller_terminal_state_verified"] is True
        assert result["z_state"] == "failed_latched"
        assert provider._load_state()["z_lifecycle"]["reference_state"] == "desynced"
        assert result["authority_receipt"]["status"] == "failed"
        saved = runtime.read_serial206_receipt("z", result["authority_receipt"]["command_id"])
        assert saved["status"] == "failed"
        assert saved["result"]["reference_persistence_ok"] is False
        # Retaining failure never automatically reissues a successful physical home.
        replay = provider.execute_z_intent(intent, expected_generation=7, idempotency_key="offline-home")
        assert replay["replayed"] is True and replay["ok"] is False
        assert len(calls) == 1
    finally:
        runtime.close()


def test_both_reference_writes_failing_still_retains_failed_lifecycle(tmp_path):
    provider, refs, runtime, calls = make_provider(tmp_path, fault="raise", invalidation_fails=True)
    try:
        result = provider.execute_z_intent("manual_home", expected_generation=7, idempotency_key="offline-home")
        assert result["ok"] is False
        assert result["z_state"] == "failed_latched"
        assert result["result"]["reference_invalidation_ok"] is False
        assert "offline reference invalidation failure" in result["result"]["reference_invalidation_error"]
        saved = runtime.read_serial206_receipt("z", result["authority_receipt"]["command_id"])
        assert saved["status"] == "failed" and len(calls) == 1
    finally:
        runtime.close()


@pytest.mark.parametrize("intent", INTENTS)
def test_successful_reference_publication_still_completes(tmp_path, intent):
    provider, refs, runtime, calls = make_provider(tmp_path)
    try:
        result = provider.execute_z_intent(intent, expected_generation=7, idempotency_key="offline-home")
        assert result["ok"] is True, result
        assert result["z_state"] == "referenced_ready"
        assert result["result"]["reference_persistence_ok"] is True
        assert refs.state == "referenced" and len(calls) == 1
    finally:
        runtime.close()
