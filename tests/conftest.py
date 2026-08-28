from __future__ import annotations

import importlib
import os
import tempfile
from pathlib import Path


_SESSION_RUNTIME_ROOT = Path(tempfile.mkdtemp(prefix="bioxp-pytest-runtime-"))
os.environ.pop("BIOXP_OEM_RUNTIME_STATE_ROOT", None)
os.environ.pop("BIOXP_OEM_RUNTIME_ROOT", None)
for _module_name in (
    "src.bioxp.runtime_audit_store",
    "bioxp.runtime_audit_store",
):
    _module = importlib.import_module(_module_name)
    setattr(_module, "CANONICAL_RUNTIME_ROOT", _SESSION_RUNTIME_ROOT)
