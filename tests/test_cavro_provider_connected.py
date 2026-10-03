"""Inline application with real production Z adapter and workflow/receipt stores.

Finite dispatcher registration is owned/tested in the runtime lane. This shard
exercises that dispatcher's callable body, not a claim of API registration.
"""
import socket
from bioxp.pipette.cavro_application import run_application_inline
from tests.test_cavro_application import rig as wire_rig, request
from tests.test_oem_pipette_calibration import rig as native_rig

_SOCKET = socket.socket


def test_real_provider_plld_and_canonical_workflow_owner(wire_rig, native_rig, monkeypatch):
    # Calibration fixture blocks every socket; asyncio itself requires AF_UNIX.
    monkeypatch.setattr(socket, "socket", lambda family=socket.AF_INET, *a, **kw:
        _SOCKET(family, *a, **kw) if family == socket.AF_UNIX else (_ for _ in ()).throw(RuntimeError("offline network")))
    provider = native_rig.provider
    provider.primitives.pipette_transport = wire_rig.group
    provider._manual_pipette_receipt_runner = wire_rig.provider._manual_pipette_receipt_runner
    store = wire_rig.owner
    provider._wp8_execution_fence_checker = lambda command_id, **_: store.assert_workflow_current(command_id)
    source = request({"operation": "plld", "channels": [0, 1, 2, 3], "timeout_ms": 30,
        "start_steps": 40000, "search_target_steps": 60000, "search_speed_native": 300,
        "z_motor_current": 17})
    result = run_application_inline(provider, source, command_id="offline-finite-owner",
        owner_identity={"source_identity": "cavro-connected:0"})
    assert result["ok"], (result.get("error"), result["events"])
    assert ("z", 60000, False) in native_rig.native.moves
    assert any(event[0] == "stop_z" for event in native_rig.events)
    assert wire_rig.wire == [(i, "BR") for i in range(4)]
    store.publish_workflow("offline-finite-owner", payload={"application_result": result})
    assert store.get_workflow("offline-finite-owner")["application_result"]["plld"]["final_position_steps"] is not None
    assert wire_rig.store.connection.execute("SELECT count(*) FROM pipette_operations").fetchone()[0] == 1
    import os, json
    from pathlib import Path
    if root := os.environ.get("CAVRO_EVIDENCE_ROOT"):
        Path(root, "cavro-producer-example.json").write_text(json.dumps({"application": source,
            "result": result, "qualification": "offline real provider/store/router; physical leaves replaced"}, indent=2))
