import threading

from src.bioxp.oem_homing_routes import _execute_oem_step_live


class _ParallelExecutor:
    def __init__(self):
        self.barrier = threading.Barrier(2, timeout=2.0)
        self.calls = []

    def oem_move_axis_absolute(self, axis, target, *, wait_for_stop):
        self.calls.append((axis, target, wait_for_stop))
        self.barrier.wait()
        return {"ok": True}


class _MoveToExecutor:
    def __init__(self):
        self.calls = []

    def oem_move_to(self, x, y, z, **kwargs):
        self.calls.append((x, y, z, kwargs))
        return {"ok": True, "branch": "all_zero_home"}


def test_parallel_path_executes_exact_move_x_and_move_y_task_leaves_concurrently():
    executor = _ParallelExecutor()

    result = _execute_oem_step_live(
        {"op": "parallel", "steps": [{"op": "moveX", "x": 101}, {"op": "moveY", "y": 202}]},
        executor,
        wait_timeout_s=5.0,
        speed=None,
        acc=None,
        path="0",
        pseudo_z_home_steps=500,
    )

    assert result["ok"] is True
    assert result["parallel_semantics"] == "source_task_wait_all_move_x_move_y"
    assert sorted(executor.calls) == [("x", 101, True), ("y", 202, True)]
    assert [row["path"] for row in result["results"]] == ["0.parallel[0]", "0.parallel[1]"]


def test_parallel_path_rejects_non_source_shapes_without_motion():
    executor = _ParallelExecutor()

    result = _execute_oem_step_live(
        {"op": "parallel", "steps": [{"op": "moveX", "x": 1}, {"op": "moveZ", "z": 2}]},
        executor,
        wait_timeout_s=5.0,
        speed=None,
        acc=None,
        path="0",
    )

    assert result["ok"] is False
    assert result["parallel_semantics"] == "unsupported_parallel_shape"
    assert executor.calls == []


def test_all_zero_move_to_dispatches_provider_owned_compound_home_branch():
    executor = _MoveToExecutor()
    authority = {
        "gripper_confirmed": True,
        "tip_loaded": False,
        "plate_on_gantry": None,
    }

    result = _execute_oem_step_live(
        {"op": "moveTo", "x": 0, "y": 0, "z": 0, "move_to_authority": authority},
        executor,
        wait_timeout_s=5.0,
        speed=None,
        acc=None,
        path="0",
        pseudo_z_home_steps=500,
    )

    assert result["ok"] is True
    assert executor.calls[0][:3] == (0, 0, 0)
    assert result["results"][0]["result"]["branch"] == "all_zero_home"
