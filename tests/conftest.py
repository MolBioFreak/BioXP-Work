from __future__ import annotations

import importlib
import os
import tempfile
from pathlib import Path

import pytest


_SESSION_RUNTIME_ROOT = Path(tempfile.mkdtemp(prefix="bioxp-pytest-runtime-"))
os.environ["BIOXP_OEM_RUNTIME_STATE_ROOT"] = str(_SESSION_RUNTIME_ROOT)
os.environ.pop("BIOXP_OEM_RUNTIME_ROOT", None)
for _module_name in (
    "src.bioxp.runtime_audit_store",
    "bioxp.runtime_audit_store",
):
    _module = importlib.import_module(_module_name)
    setattr(_module, "CANONICAL_RUNTIME_ROOT", _SESSION_RUNTIME_ROOT)


@pytest.fixture(autouse=True)
def isolate_canonical_runtime_root(monkeypatch, tmp_path):
    """Keep default BioXP runtime state out of the robot production path."""
    root = tmp_path / "canonical-oem-runtime"
    monkeypatch.delenv("BIOXP_OEM_RUNTIME_STATE_ROOT", raising=False)
    monkeypatch.delenv("BIOXP_OEM_RUNTIME_ROOT", raising=False)
    for module_name in (
        "src.bioxp.runtime_audit_store",
        "bioxp.runtime_audit_store",
    ):
        module = importlib.import_module(module_name)
        monkeypatch.setattr(module, "CANONICAL_RUNTIME_ROOT", root)
