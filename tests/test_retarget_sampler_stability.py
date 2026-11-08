"""Real API/SQLite/provider/driver; only TMCL transport is inert."""
import threading
import time
from concurrent.futures import ThreadPoolExecutor

from tests.test_movement_modes import connected, rig, submit  # noqa: F401


def test_missing_display_reply_does_not_hold_next_retarget_for_motion_timeout(connected, monkeypatch):
    http, provider, frames, fault, driver, positions = connected
    sampled = threading.Event()
    release = threading.Event()
    reads = []
    original = driver.send_tmcl

    def exchange(board, command, typ, motor, value, **kw):
        if threading.current_thread().name == "bioxp-retarget-pose":
            reads.append((board, command, typ, motor, dict(kw)))
            sampled.set()
            # Model the actual response budget, without spending a minute in CI.
            # Ordinary motion queries block; existing bounded query-only reads return null.
            if kw.get("read_timeout_ms", 0) >= 60000:
                release.wait(5)
            return None
        return original(board, command, typ, motor, value, **kw)

    monkeypatch.setattr(driver, "send_tmcl", exchange)
    monkeypatch.setattr(driver, "send_tmcl_retry", exchange)
    _, first = submit(http, "oem.xy.retarget", "sampler-first-01",
                      {"target": {"x": 20000, "y": 30000}})
    assert first["status"] == "completed", first
    assert sampled.wait(2), "display query never entered transport"
    pool = ThreadPoolExecutor(1)
    future = pool.submit(submit, http, "oem.xy.retarget", "sampler-next-001",
                         {"target": {"x": 25000, "y": 35000}})
    try:
        deadline = time.monotonic() + 2
        while not future.done() and time.monotonic() < deadline:
            time.sleep(.01)
        assert future.done(), "display query held provider lock and stranded the next retarget"
        _, second = future.result()
        assert second["status"] == "completed", second
        assert second["source_receipt"]["xy_retarget"]["motion_pending"] is True
        assert second["physical_effect_verified"] is False
        assert reads
        assert all((command, typ, motor) == (6, 1, 0)
                   and kw["read_timeout_ms"] == 90
                   and kw.get("attempts") == 1
                   and not kw.get("ordinary_motor_retry", False)
                   for board, command, typ, motor, kw in reads)
    finally:
        release.set()
        future.result(timeout=10)
        pool.shutdown()
        with provider._lock:
            provider._pending_xy_retarget = None
        provider._retarget_pose_thread.join(2)
