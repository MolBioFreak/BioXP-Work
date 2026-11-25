"""Offline regression tests for handler repairs 4 (USB receiver backoff) and 5 (camera owner verification)."""
from __future__ import annotations

import errno
import os
import signal
import stat
import subprocess
import threading
import time
from types import SimpleNamespace

import pytest

from bioxp.novo_router import NovoRouter
from bioxp.novo_usb_can import NovoUsbCanBus, novo_decode, novo_encode
from bioxp.usb_driver import BioXpTester


class _FakeUSBTimeoutError(Exception):
    pass


_FakeUSBTimeoutError.__name__ = "USBTimeoutError"


class _FakeUSBError(Exception):
    def __init__(self, message, *, err_no=None, backend_error_code=None):
        super().__init__(message)
        self.errno = err_no
        self.backend_error_code = backend_error_code


def test_novo_router_continuous_read_faults_bounded_with_exponential_backoff_and_enodev_classification():
    now = [100.0]
    waits = []
    read_calls = [0]
    done = threading.Event()

    class FaultyEpIn:
        def read(self, size, timeout=None):
            read_calls[0] += 1
            if read_calls[0] >= 7:
                done.set()
            raise _FakeUSBError("No such device", err_no=errno.ENODEV, backend_error_code=-4)

    class DummyEpOut:
        def __init__(self):
            self.writes = 0

        def write(self, data, timeout=None):
            self.writes += 1
            return len(data)

    ep_out = DummyEpOut()

    def fake_wait(delay_s):
        waits.append(round(float(delay_s), 6))
        now[0] += float(delay_s)
        if len(waits) >= 7:
            router._stop.set()
            return True
        return router._stop.is_set()

    router = NovoRouter(
        ep_in=FaultyEpIn(),
        ep_out=ep_out,
        decode=novo_decode,
        clock=lambda: now[0],
        error_backoff_initial_s=0.01,
        error_backoff_max_s=0.25,
        stop_wait=fake_wait,
    )
    router.start()
    assert done.wait(timeout=3.0)
    router.shutdown(join_timeout_s=2.0)

    assert waits == [0.01, 0.02, 0.04, 0.08, 0.16, 0.25, 0.25]
    assert read_calls[0] == 7
    assert ep_out.writes == 0
    diags = router.diagnostics_snapshot()
    assert len(diags) == 7
    assert all(d.get("device_lost") is True for d in diags)
    assert all(d.get("error_kind") == "device_disconnected" for d in diags)
    assert [d.get("consecutive_errors") for d in diags] == [1, 2, 3, 4, 5, 6, 7]


def test_novo_router_transient_fault_recovers_resets_backoff_and_never_replays_tx():
    now = [200.0]
    waits = []
    frame_bytes = novo_encode(NovoUsbCanBus.build_payload(0, bytes([4, 128, 0, 0, 0, 0, 0, 0]), 8))
    script = [
        _FakeUSBError("EIO", err_no=errno.EIO),
        _FakeUSBError("EIO", err_no=errno.EIO),
        frame_bytes,
        _FakeUSBError("EIO", err_no=errno.EIO),
    ]
    finished = threading.Event()

    class ScriptedEpIn:
        def read(self, size, timeout=None):
            if not script:
                finished.set()
                router._stop.set()
                raise TimeoutError("done")
            nxt = script.pop(0)
            if isinstance(nxt, Exception):
                raise nxt
            return nxt

    class DummyEpOut:
        def __init__(self):
            self.writes = 0

        def write(self, data, timeout=None):
            self.writes += 1
            return len(data)

    ep_out = DummyEpOut()

    def fake_wait(delay_s):
        waits.append(round(float(delay_s), 6))
        now[0] += float(delay_s)
        return router._stop.is_set()

    router = NovoRouter(
        ep_in=ScriptedEpIn(),
        ep_out=ep_out,
        decode=novo_decode,
        clock=lambda: now[0],
        error_backoff_initial_s=0.01,
        error_backoff_max_s=0.25,
        stop_wait=fake_wait,
    )
    router.start()
    assert finished.wait(timeout=3.0)
    router.shutdown(join_timeout_s=2.0)

    # First two errors escalate (0.01, 0.02); successful frame resets backoff so next error starts at 0.01!
    assert waits == [0.01, 0.02, 0.01]
    assert ep_out.writes == 0
    assert len(router.queue_snapshot("valid_async")) == 1


def test_novo_router_shutdown_interrupts_backoff_promptly():
    entered_backoff = threading.Event()

    class FaultyEpIn:
        def read(self, size, timeout=None):
            raise _FakeUSBError("EIO", err_no=errno.EIO)

    def tracking_wait(delay_s):
        entered_backoff.set()
        return router._stop.wait(delay_s)

    router = NovoRouter(
        ep_in=FaultyEpIn(),
        ep_out=SimpleNamespace(write=lambda *a, **k: 0),
        decode=novo_decode,
        error_backoff_initial_s=10.0,
        error_backoff_max_s=10.0,
        stop_wait=tracking_wait,
    )
    router.start()
    assert entered_backoff.wait(timeout=2.0)
    t0 = time.monotonic()
    router.shutdown(join_timeout_s=1.0)
    assert time.monotonic() - t0 < 0.5


def test_novo_router_idle_timeouts_unaffected_by_error_backoff():
    waits = []
    timeout_count = [0]
    done = threading.Event()

    class IdleEpIn:
        def read(self, size, timeout=None):
            timeout_count[0] += 1
            if timeout_count[0] >= 5:
                done.set()
                router._stop.set()
            raise _FakeUSBTimeoutError("idle timeout")

    router = NovoRouter(
        ep_in=IdleEpIn(),
        ep_out=SimpleNamespace(write=lambda *a, **k: 0),
        decode=novo_decode,
        stop_wait=lambda d: (waits.append(d), router._stop.is_set())[1],
    )
    router.start()
    assert done.wait(timeout=2.0)
    router.shutdown(join_timeout_s=1.0)

    assert waits == []
    assert router.diagnostics_snapshot() == []


# ---------------------------------------------------------------------------
# Item 5: Camera busy-owner release (mocked OS/proc, zero real signaling)
# ---------------------------------------------------------------------------


def _mock_proc_env(monkeypatch, *, device_stats, proc_table, fuser_result):
    signals = []
    monkeypatch.setattr(time, "sleep", lambda s: None)
    monkeypatch.setattr("shutil.which", lambda name: "/usr/bin/fuser" if name == "fuser" else None)
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: fuser_result)

    def fake_stat(path, *args, **kwargs):
        path_s = str(path)
        if path_s in device_stats:
            return device_stats[path_s]
        if path_s.startswith("/proc/"):
            parts = path_s.split("/")
            if len(parts) == 5 and parts[3] == "fd":
                pid = int(parts[2])
                fd = parts[4]
                info = proc_table.get(pid)
                if info is None or not info.get("alive", True):
                    raise FileNotFoundError(path_s)
                fds = info.get("fds", {})
                if fd not in fds:
                    raise FileNotFoundError(path_s)
                return fds[fd]
        raise FileNotFoundError(path_s)

    def fake_listdir(path):
        path_s = str(path)
        parts = path_s.split("/")
        if len(parts) == 4 and parts[1] == "proc" and parts[3] == "fd":
            pid = int(parts[2])
            info = proc_table.get(pid)
            if info is None or not info.get("alive", True):
                raise FileNotFoundError(path_s)
            if info.get("fd_perm_error"):
                raise PermissionError(path_s)
            return list(info.get("fds", {}).keys())
        raise FileNotFoundError(path_s)

    def fake_kill(pid, sig):
        signals.append((int(pid), int(sig)))
        info = proc_table.get(int(pid))
        if info is None or not info.get("alive", True):
            raise ProcessLookupError(pid)
        if sig == 0:
            return
        on_sig = info.get("on_signal")
        if callable(on_sig):
            on_sig(sig, info)

    orig_open = open

    def fake_open(path, *args, **kwargs):
        path_s = str(path)
        if path_s.startswith("/proc/") and path_s.endswith("/stat"):
            pid = int(path_s.split("/")[2])
            info = proc_table.get(pid)
            if info is None or not info.get("alive", True):
                raise FileNotFoundError(path_s)
            import io
            comm = info.get("comm", "python3 (cam)")
            starttime = info["starttime"]
            # 22nd field (index 19 after ') '): fields 3..21 are 19 tokens, field 22 is starttime
            fields = ["S"] + ["0"] * 18 + [str(starttime), "0"]
            return io.StringIO(f"{pid} ({comm}) " + " ".join(fields) + "\n")
        return orig_open(path, *args, **kwargs)

    monkeypatch.setattr(os, "stat", fake_stat)
    monkeypatch.setattr(os, "listdir", fake_listdir)
    monkeypatch.setattr(os, "kill", fake_kill)
    monkeypatch.setattr("builtins.open", fake_open)
    return signals


def _chr_stat(dev, rdev):
    return SimpleNamespace(st_mode=stat.S_IFCHR | 0o660, st_dev=dev, st_rdev=rdev, st_ino=100)


def test_camera_release_signals_only_verified_owner_and_handles_symlink(monkeypatch):
    target_stat = _chr_stat(dev=5, rdev=20736)
    other_stat = _chr_stat(dev=5, rdev=20737)

    def exit_on_kill(sig, info):
        if sig == signal.SIGKILL:
            info["alive"] = False

    proc_table = {
        4242: {"alive": True, "starttime": 123456, "fds": {"3": target_stat}, "on_signal": exit_on_kill},
        5555: {"alive": True, "starttime": 222222, "fds": {"3": other_stat}},
    }
    fuser_res = subprocess.CompletedProcess(
        ["fuser", "/dev/v4l/by-id/usb-cam"], 0,
        stdout=" 4242 5555\n",
        stderr="/dev/v4l/by-id/usb-cam:  0 1 9999m\n",
    )
    signals = _mock_proc_env(
        monkeypatch,
        device_stats={"/dev/v4l/by-id/usb-cam": target_stat},
        proc_table=proc_table,
        fuser_result=fuser_res,
    )

    res = BioXpTester.camera_release_busy_owners(device="/dev/v4l/by-id/usb-cam")
    assert res["ok"] is True
    assert res["pids"] == [4242]
    assert res["killed"] == [4242]
    assert res["alive"] == []
    # Only verified owner 4242 got SIGTERM then SIGKILL; unrelated 5555 and stderr numbers 0/1/9999 got ZERO real signals.
    real_sigs = [(pid, sig) for pid, sig in signals if sig != 0]
    assert real_sigs == [(4242, int(signal.SIGTERM)), (4242, int(signal.SIGKILL))]


def test_camera_release_ignores_stderr_numbers_and_fails_closed_on_ambiguous_fuser(monkeypatch):
    target_stat = _chr_stat(dev=5, rdev=20736)
    proc_table = {
        4242: {"alive": True, "starttime": 123456, "fds": {"3": target_stat}},
    }

    # Case 1: exit 1 with empty stdout & stderr -> clean empty.
    sigs = _mock_proc_env(
        monkeypatch,
        device_stats={"/dev/video0": target_stat},
        proc_table=proc_table,
        fuser_result=subprocess.CompletedProcess(["fuser", "/dev/video0"], 1, stdout="", stderr=""),
    )
    res = BioXpTester.camera_release_busy_owners(device="/dev/video0")
    assert res["ok"] is True and res["pids"] == [] and sigs == []

    # Case 2: exit 1 with stderr error text containing numbers (like /dev/video0) -> fail closed, no signals.
    sigs = _mock_proc_env(
        monkeypatch,
        device_stats={"/dev/video0": target_stat},
        proc_table=proc_table,
        fuser_result=subprocess.CompletedProcess(["fuser", "/dev/video0"], 1, stdout="", stderr="/dev/video0: permission denied 4242\n"),
    )
    res = BioXpTester.camera_release_busy_owners(device="/dev/video0")
    assert res["ok"] is False and sigs == []

    # Case 3: nonzero exit 2 -> fail closed.
    sigs = _mock_proc_env(
        monkeypatch,
        device_stats={"/dev/video0": target_stat},
        proc_table=proc_table,
        fuser_result=subprocess.CompletedProcess(["fuser", "/dev/video0"], 2, stdout="4242", stderr="fatal"),
    )
    res = BioXpTester.camera_release_busy_owners(device="/dev/video0")
    assert res["ok"] is False and sigs == []

    # Case 4: malformed stdout -> fail closed.
    sigs = _mock_proc_env(
        monkeypatch,
        device_stats={"/dev/video0": target_stat},
        proc_table=proc_table,
        fuser_result=subprocess.CompletedProcess(["fuser", "/dev/video0"], 0, stdout="4242 badtoken", stderr=""),
    )
    res = BioXpTester.camera_release_busy_owners(device="/dev/video0")
    assert res["ok"] is False and sigs == []


def test_camera_release_excludes_system_self_and_defeats_pid_reuse_before_sigkill(monkeypatch):
    target_stat = _chr_stat(dev=5, rdev=20736)
    this_pid = os.getpid()

    def reuse_pid_after_sigterm(sig, info):
        if sig == signal.SIGTERM:
            # Simulate original process exiting and OS reusing PID with new starttime!
            info["starttime"] = 999999

    proc_table = {
        0: {"alive": True, "starttime": 1, "fds": {"3": target_stat}},
        1: {"alive": True, "starttime": 1, "fds": {"3": target_stat}},
        this_pid: {"alive": True, "starttime": 10, "fds": {"3": target_stat}},
        7001: {"alive": True, "starttime": 5000, "fds": {"3": target_stat}, "on_signal": reuse_pid_after_sigterm},
    }
    sigs = _mock_proc_env(
        monkeypatch,
        device_stats={"/dev/video0": target_stat},
        proc_table=proc_table,
        fuser_result=subprocess.CompletedProcess(["fuser", "/dev/video0"], 0, stdout=f"0 1 {this_pid} 7001", stderr=""),
    )
    res = BioXpTester.camera_release_busy_owners(device="/dev/video0")
    assert res["ok"] is True
    assert res["pids"] == [7001]
    real_sigs = [(pid, sig) for pid, sig in sigs if sig != 0]
    # PID 7001 received SIGTERM, then starttime changed -> MUST NOT receive SIGKILL!
    assert real_sigs == [(7001, int(signal.SIGTERM))]
