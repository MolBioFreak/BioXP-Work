import importlib
import sys
import types


class Completed:
    def __init__(self, stdout=""):
        self.stdout = stdout
        self.stderr = ""
        self.returncode = 0


def load_api(monkeypatch):
    usb_pkg = types.ModuleType("usb")
    usb_core = types.ModuleType("usb.core")
    usb_util = types.ModuleType("usb.util")
    usb_pkg.core = usb_core
    usb_pkg.util = usb_util
    monkeypatch.setitem(sys.modules, "usb", usb_pkg)
    monkeypatch.setitem(sys.modules, "usb.core", usb_core)
    monkeypatch.setitem(sys.modules, "usb.util", usb_util)
    for name in ["src.bioxp.api", "src.bioxp.usb_driver", "src.bioxp"]:
        sys.modules.pop(name, None)
    return importlib.import_module("src.bioxp.api")


def test_camera_stream_reset_reports_software_reset_provenance(monkeypatch):
    api = load_api(monkeypatch)
    monkeypatch.setattr(api.subprocess, "run", lambda *args, **kwargs: Completed(""))
    monkeypatch.setattr(api.os.path, "exists", lambda path: False)

    result = api._camera_reset_local("/dev/video99")

    provenance = result["reset_provenance"]
    assert provenance["schema_version"] == "bioxp.reset_provenance.v1"
    assert provenance["subsystem"] == "camera"
    assert provenance["source"] == "camera_reset_local"
    assert provenance["requested_device"] == "/dev/video99"
    assert provenance["resolved_device"] == "/dev/video0"
    assert provenance["reset_scope"] == "ffmpeg_process_and_stream_lock"
    assert provenance["hardware_usb_reset_performed"] is False
    assert provenance["software_recovery"] is True
    assert provenance["hardware_component_fault_proven"] is False
