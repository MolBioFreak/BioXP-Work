"""Mutation tests for the static BioXP OEM movement registry verifier.

These tests use only temporary JSON copies. They never contact hardware, a runtime,
or any remote service.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import subprocess

import pytest
from pathlib import Path

from scripts.verify_oem_movement_registry import verify

ROOT = Path(__file__).resolve().parents[1]
REGISTRY_PATH = ROOT / "docs/specs/2026-07-23-oem-movement-method-source-binary-registry.json"


def _write_candidate(tmp_path: Path, mutate) -> Path:
    registry = json.loads(REGISTRY_PATH.read_text())
    mutate(registry)
    candidate = tmp_path / "registry.json"
    candidate.write_text(json.dumps(registry, indent=2, sort_keys=True) + "\n")
    return candidate


def _errors(candidate: Path) -> list[str]:
    result = verify(candidate)
    assert not result["ok"], "semantic mutation was falsely accepted"
    return result["errors"]


def test_baseline_registry_passes() -> None:
    result = verify(REGISTRY_PATH)
    assert result["ok"], result["errors"]


def test_rendered_markdown_is_current_and_deterministic() -> None:
    result = subprocess.run(
        ["python3", str(ROOT / "scripts/render_oem_movement_registry.py"), "--check"],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr or result.stdout


@pytest.mark.parametrize(
    "edge_id",
    [
        "environment_queue_initialize",
        "warning_update_failure_queue_initialize",
        "software_update_cancel_queue_initialize",
        "fetch_queue_initialize",
        "warning_initialize_motion",
        "door_event_queue_initialize",
        "door_event_queue_wakefrompause",
        "wake_rehome",
        "rehome_initialize_motors",
    ],
)
def test_rejects_missing_reachable_physical_entry_edge(tmp_path: Path, edge_id: str) -> None:
    candidate = _write_candidate(
        tmp_path,
        lambda d: d.__setitem__(
            "required_call_edges",
            [e for e in d["required_call_edges"] if e["edge_id"] != edge_id],
        ),
    )
    assert any(edge_id in error for error in _errors(candidate))


def test_rejects_missing_can_not_ready_terminal_branch(tmp_path: Path) -> None:
    candidate = _write_candidate(
        tmp_path,
        lambda d: d.__setitem__(
            "required_branch_outcomes",
            [b for b in d["required_branch_outcomes"] if b["branch_id"] != "environment_not_ready_nonmanual_return"],
        ),
    )
    assert any("environment_not_ready_nonmanual_return" in error for error in _errors(candidate))


def test_rejects_omitted_direct_initializer_wait(tmp_path: Path) -> None:
    def mutate(d: dict) -> None:
        seq = next(s for s in d["ordered_call_sequences"] if s["sequence_id"] == "initialize_motors_direct_oem")
        seq["steps"] = [s for s in seq["steps"] if s["line"] != 3370]

    candidate = _write_candidate(tmp_path, mutate)
    assert any("ordered sequence required steps missing" in error and "3370" in error for error in _errors(candidate))


def test_rejects_evidence_lock_hash_drift(tmp_path: Path) -> None:
    candidate = _write_candidate(
        tmp_path,
        lambda d: d["authority"].__setitem__("evidence_lock_sha256", "0" * 64),
    )
    assert "evidence lock hash mismatch" in _errors(candidate)


def test_rejects_source_omitted_from_canonical_lock(tmp_path: Path) -> None:
    lock_source = Path(json.loads(REGISTRY_PATH.read_text())["authority"]["evidence_lock_path"])
    lock_copy = tmp_path / "OEM_EVIDENCE_LOCK.json"
    shutil.copyfile(lock_source, lock_copy)
    lock = json.loads(lock_copy.read_text())
    omitted = "decompiled_src_vision/CVisionLib/ClassFrameGrabber.cs"
    lock["decompile_binary_map"]["CVisionLib.dll"].remove(omitted)
    lock_copy.write_text(json.dumps(lock, indent=2) + "\n")

    def mutate(d: dict) -> None:
        d["authority"]["evidence_lock_path"] = str(lock_copy)
        d["authority"]["evidence_lock_sha256"] = hashlib.sha256(lock_copy.read_bytes()).hexdigest()

    candidate = _write_candidate(tmp_path, mutate)
    assert any("source absent from canonical evidence lock: vision" in error for error in _errors(candidate))
