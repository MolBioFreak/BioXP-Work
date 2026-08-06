from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_PATH = ROOT / "scripts" / "bioxp_handlerctl.py"
INSTALLER_PATH = ROOT / "scripts" / "install_bioxp_handler_lifecycle.sh"

spec = importlib.util.spec_from_file_location("bioxp_handlerctl", CONTROLLER_PATH)
assert spec is not None and spec.loader is not None
handlerctl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(handlerctl)


def test_running_stage_detection_and_refusal():
    status = {
        "startup": {
            "stages": {
                "constructor_pipette_stage": {"state": "passed"},
                "initialization_without_motion": {"state": "running"},
            }
        }
    }
    assert handlerctl.running_stages(status) == ["initialization_without_motion"]
    with pytest.raises(RuntimeError, match="refusing lifecycle mutation"):
        handlerctl.refuse_running(status, force=False)
    handlerctl.refuse_running(status, force=True)


def test_handoff_authorization_preflight_happens_before_recovery_stop(monkeypatch):
    calls: list[tuple[str, ...]] = []

    monkeypatch.setattr(handlerctl, "get_status", lambda required=False: {"startup": {"stages": {}}})

    def fake_systemctl(*args, **kwargs):
        calls.append(("system", *args))
        raise RuntimeError("authorization denied")

    def fake_user_systemctl(*args, **kwargs):
        calls.append(("user", *args))
        raise AssertionError("user service must not be touched before authorization succeeds")

    monkeypatch.setattr(handlerctl, "systemctl", fake_systemctl)
    monkeypatch.setattr(handlerctl, "user_systemctl", fake_user_systemctl)

    with pytest.raises(RuntimeError, match="authorization denied"):
        handlerctl.handoff(force=False)
    assert calls == [("system", "reset-failed", "bioxp-api.service")]


def test_installer_policy_is_exact_unit_and_has_no_general_sudo():
    text = INSTALLER_PATH.read_text()
    assert 'action.lookup("unit") !== "bioxp-api.service"' in text
    assert 'subject.user !== "molbiofreak"' in text
    assert "org.freedesktop.systemd1.manage-units" in text
    assert "sudoers" not in text.lower()
    assert "NOPASSWD" not in text
    assert '"reboot"' not in text
    assert "systemctl reboot" not in text
    assert "Restart=on-failure" in text
    assert "KillMode=control-group" in text
    assert "TimeoutStopSec=20s" in text


def test_controller_never_invokes_sudo_or_arbitrary_unit():
    text = CONTROLLER_PATH.read_text()
    assert 'UNIT = "bioxp-api.service"' in text
    assert '"sudo"' not in text
    assert "shell=True" not in text
    assert "motion" not in handlerctl.RECOVERY_COMMAND
