"""Source-grounded BioXP motion preparation and physical interrupt kernels.

These helpers contain no transport construction.  Callers provide an already-owned
controller driver and the immutable serial-206 authority.  Preparation never sends
MVP/ROR/ROL/RFS/MST and never treats the legacy raw relay helper as a global 24 V
switch.  Emergency stop is the separate ClassMotor.StopMotor interrupt path.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

from .oem_machine_bundle import OEM_MACHINE_SERIAL, get_active_oem_machine_snapshot


PREPARE_SCHEMA = "bioxp.oem_prepare_without_motion.v2"
STOP_SCHEMA = "bioxp.physical_aggregate_stop.v1"

# ClassControlInterface construction for the accepted BioXP 3200 serial-206 machine.
# Board 7 is the chiller/temperature controller.  Its status-2 reply to command 64 is
# not a motor-board activation ACK and is therefore represented explicitly rather
# than accepted as generic success or silently omitted.
_SERIAL206_COMPONENTS = (
    ("x", 5, 0, True),
    ("y", 4, 0, True),
    ("z", 4, 1, True),
    ("g", 4, 2, True),
    ("door", 6, 0, True),
)
_SERIAL206_ACTIVATION_BOARDS = (4, 5, 6, 7)


@dataclass(frozen=True)
class Serial206MotionAuthority:
    machine_serial: int
    acquisition_id: str
    evidence_lock_sha256: str
    mutation_authorized: bool
    components: tuple[tuple[str, int, int, bool], ...] = _SERIAL206_COMPONENTS
    activation_boards: tuple[int, ...] = _SERIAL206_ACTIVATION_BOARDS

    @classmethod
    def from_snapshot_projection(cls, projection: Mapping[str, Any]) -> "Serial206MotionAuthority":
        serial = projection.get("serial", projection.get("machine_serial"))
        if serial != OEM_MACHINE_SERIAL:
            raise ValueError(f"serial-206 motion authority required, got {serial!r}")
        lock_sha = projection.get("lock_sha256")
        acquisition_id = projection.get("acquisition_id")
        if not isinstance(lock_sha, str) or len(lock_sha) != 64:
            raise ValueError("serial-206 evidence lock SHA-256 is missing")
        if not isinstance(acquisition_id, str) or not acquisition_id:
            raise ValueError("serial-206 acquisition identity is missing")
        return cls(
            machine_serial=int(serial),
            acquisition_id=acquisition_id,
            evidence_lock_sha256=lock_sha,
            mutation_authorized=projection.get("mutation_authorized") is True,
        )

    @classmethod
    def from_active_snapshot(cls) -> "Serial206MotionAuthority":
        return cls.from_snapshot_projection(get_active_oem_machine_snapshot().config_status_projection())

    def controller_evidence(self) -> dict[str, Any]:
        return {
            "machine_serial": self.machine_serial,
            "acquisition_id": self.acquisition_id,
            "evidence_lock_sha256": self.evidence_lock_sha256,
            "mutation_authorized": self.mutation_authorized,
            "component_source": "serial-206 ClassControlInterface motor construction and m_AxisIODesignater",
        }


def _ack_status(value: Any) -> int | None:
    ack = value.get("ack") if isinstance(value, Mapping) else None
    status = ack.get("status") if isinstance(ack, Mapping) else None
    return int(status) if type(status) is int else None


def _board_cycle_status(rows: Any, board: int) -> int | None:
    row = rows.get(int(board)) if isinstance(rows, Mapping) else None
    if isinstance(row, Mapping) and isinstance(row.get("ack"), Mapping):
        row = row["ack"]
    status = row.get("status") if isinstance(row, Mapping) else None
    return int(status) if type(status) is int else None


def _board_cycle_ok(rows: Any, boards: tuple[int, ...]) -> bool:
    if not isinstance(rows, Mapping) or any(int(board) not in rows for board in boards):
        return False
    return all(
        _board_cycle_status(rows, board) == 100
        or (int(board) == 7 and _board_cycle_status(rows, board) == 2)
        for board in boards
    )


def _stage(stage_id: str, status: str, source_anchor: str, evidence: Any) -> dict[str, Any]:
    return {
        "stage_id": stage_id,
        "status": status,
        "source_anchor": source_anchor,
        "controller_evidence": evidence,
        "physical_motion": False,
    }


def _preparation_result(authority: Serial206MotionAuthority, ledger: list[dict[str, Any]]) -> dict[str, Any]:
    ok = all(row["status"] in {"passed", "not_applicable"} for row in ledger)
    lifecycle_row = next((row for row in ledger if row.get("stage_id") == "boardLifecycleGeneration"), None)
    lifecycle_evidence = lifecycle_row.get("controller_evidence") if isinstance(lifecycle_row, Mapping) else None
    board_generation = (
        lifecycle_evidence.get("board_lifecycle_generation")
        if isinstance(lifecycle_evidence, Mapping)
        else None
    )
    return {
        "schema_version": PREPARE_SCHEMA,
        "ok": ok,
        "state": "completed" if ok else "failed_closed",
        "machine_serial": authority.machine_serial,
        "controller_evidence": authority.controller_evidence(),
        "stage_ledger": ledger,
        "stage_receipts": ledger,
        "board_lifecycle_generation": board_generation,
        "physical_motion": False,
        "physical_motion_commanded": False,
        "homing_performed": False,
        "motor_output_state": "unknown",
        "motor_torque_verified": False,
        "global_24v_switch_claimed": False,
    }


def prepare_motion_without_motion(
    driver: Any,
    authority: Serial206MotionAuthority,
    *,
    components: tuple[str, ...] | None = None,
    reuse_current_board_lifecycle: bool = False,
) -> dict[str, Any]:
    """Run bounded OEM activation, literal no-motion parameterization, and queries."""
    ledger: list[dict[str, Any]] = []

    def invalidate_profile(reason: str) -> None:
        invalidate = getattr(driver, "_invalidate_oem_no_motion_profiles", None)
        if callable(invalidate):
            invalidate(reason=reason)
    selected = tuple(dict.fromkeys(
        str(component).strip().lower()
        for component in (components or tuple(row[0] for row in authority.components))
    ))
    known = {row[0] for row in authority.components}
    if not selected or any(component not in known for component in selected):
        raise ValueError(f"invalid serial-206 preparation component selection: {selected}")
    activation_boards = tuple(sorted({
        int(board) for name, board, _motor, required in authority.components
        if required and name in selected
    }))
    authority_ok = authority.machine_serial == OEM_MACHINE_SERIAL and authority.mutation_authorized is True
    ledger.append(_stage(
        "authority",
        "passed" if authority_ok else "failed",
        "immutable serial-206 OEM evidence lock",
        authority.controller_evidence(),
    ))
    if not authority_ok:
        return _preparation_result(authority, ledger)

    # Recovered ControlLib.initialCheck evaluates the machine environment before
    # its ESM(false) -> ESM(true) command-64 transition. Keep these reads fresh;
    # they are controller/interlock evidence, never torque or movement proof.
    try:
        rail = driver.motor_query_24v_sensor()
    except Exception as exc:
        rail = {"ack": None, "error": f"{type(exc).__name__}: {exc}"}
    rail_ok = bool(
        isinstance(rail, Mapping)
        and _ack_status(rail) == 100
        and rail.get("reply_valid") is True
        and rail.get("sample_valid") is True
        and rail.get("safety_valid") is True
        and rail.get("oem_scalar") == 0
    )
    ledger.append(_stage(
        "rail_24v_readback",
        "passed" if rail_ok else "failed",
        "ClassIOControl.query24VSensor lines 92-110",
        rail,
    ))
    if not rail_ok:
        return _preparation_result(authority, ledger)

    for stage_id, io_type, label in (("door_readback", 1, "door"), ("latch_readback", 3, "latch")):
        try:
            observation = driver.deck_io_query_type(io_type)
        except Exception as exc:
            observation = {"ack": None, "value": None, "error": f"{type(exc).__name__}: {exc}"}
        observed_value = observation.get("value") if isinstance(observation, Mapping) else None
        observed_ok = bool(
            isinstance(observation, Mapping)
            and _ack_status(observation) == 100
            and type(observed_value) is int
            and observed_value in {0, 1}
        )
        ledger.append(_stage(
            stage_id,
            "passed" if observed_ok else "failed",
            f"ClassIOControl query {label} sensor read-only; value is source observation, not an invented equality gate",
            observation,
        ))
        if not observed_ok:
            return _preparation_result(authority, ledger)

    # A component refresh inside an already established lifecycle must not run
    # initialCheck's machine-wide cmd64 cycle. In particular, X is on board 5
    # while the gravity-loaded Z head is on board 4. Cycling every board and
    # then restoring only X removes Z holding current. OEM initialCheck avoids
    # that incomplete state by following the cycle with full motor setup.
    cycle_boards = tuple(int(board) for board in authority.activation_boards)
    if reuse_current_board_lifecycle:
        generation_fn = getattr(driver, "oem_current_board_lifecycle_generation", None)
        state_fn = getattr(driver, "_oem_board_state", None)
        current_generation = generation_fn() if callable(generation_fn) else None
        board_state = state_fn() if callable(state_fn) else None
        lifecycle_ok = bool(
            type(current_generation) is int
            and isinstance(board_state, Mapping)
            and all(board_state.get(board) is True for board in cycle_boards)
        )
        lifecycle = {
            "ok": lifecycle_ok,
            "board_lifecycle_generation": current_generation if lifecycle_ok else None,
            "reused": True,
            "command64_emitted": False,
            "required_initialized_boards": list(cycle_boards),
            "initialized_boards": sorted(
                int(board) for board in cycle_boards
                if isinstance(board_state, Mapping) and board_state.get(board) is True
            ),
            "failure": None if lifecycle_ok else "current_board_lifecycle_unavailable_or_incomplete",
        }
    else:
        # Normal production initialCheck is a complete ESM(false) -> ESM(true)
        # transition over all four constructed OEM boards. Activation-only
        # recovery is not equivalent and must not preserve a stale profile.
        try:
            deactivation = driver.deactivate_boards(expect_reply=True, fail_fast=True)
        except Exception as exc:
            deactivation = {"error": f"{type(exc).__name__}: {exc}"}
        deactivation_ok = _board_cycle_ok(deactivation, cycle_boards)
        ledger.append(_stage(
            "deactivateBoard",
            "passed" if deactivation_ok else "failed",
            "ControlLib.initialCheck ESM(head,false) -> Class*Board.deactivateBoard cmd64=0",
            deactivation,
        ))
        if not deactivation_ok:
            return _preparation_result(authority, ledger)

        try:
            activation = driver.activate_boards(expect_reply=True, fail_fast=True)
        except Exception as exc:
            activation = {"error": f"{type(exc).__name__}: {exc}"}
        activation_ok = _board_cycle_ok(activation, cycle_boards)
        ledger.append(_stage(
            "activateBoard",
            "passed" if activation_ok else "failed",
            "ControlLib.initialCheck ESM(head,true) -> Class*Board.activateBoard cmd64=1",
            activation,
        ))
        if not activation_ok:
            return _preparation_result(authority, ledger)

        try:
            lifecycle = driver.oem_begin_board_lifecycle_generation(
                deactivation=deactivation,
                activation=activation,
            )
        except Exception as exc:
            lifecycle = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
        lifecycle_ok = isinstance(lifecycle, Mapping) and lifecycle.get("ok") is True
    ledger.append(_stage(
        "boardLifecycleGeneration",
        "passed" if lifecycle_ok else "failed",
        (
            "Existing complete acknowledged OEM board lifecycle; component profile refresh emits no cmd64"
            if reuse_current_board_lifecycle
            else "Linux evidence binding after complete acknowledged OEM cmd64=0 -> cmd64=1"
        ),
        lifecycle,
    ))
    if not lifecycle_ok:
        return _preparation_result(authority, ledger)

    try:
        board_wait = driver.motor_oem_wait_for_board()
    except Exception as exc:
        board_wait = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    ledger.append(_stage(
        "waitForBoard",
        "passed" if isinstance(board_wait, Mapping) and board_wait.get("ok") is True else "failed",
        "ClassControlInterface.waitForBoard lines 3507-3534",
        board_wait,
    ))
    if not isinstance(board_wait, Mapping) or board_wait.get("ok") is not True:
        return _preparation_result(authority, ledger)

    try:
        initialized = driver.oem_initialize_without_motion_test_case(
            board_wait=board_wait,
            components=selected,
        )
    except Exception as exc:
        initialized = {"ok": False, "physical_motion": False, "error": f"{type(exc).__name__}: {exc}"}
    transcript = initialized.get("transcript") if isinstance(initialized, Mapping) else None
    command_rows = [row for row in transcript if isinstance(row, Mapping) and "ack" in row] if isinstance(transcript, list) else []
    initialization_ok = bool(
        isinstance(initialized, Mapping)
        and initialized.get("ok") is True
        and initialized.get("physical_motion") is False
        and initialized.get("homing_performed") is False
        and command_rows
        and all(_ack_status(row) == 100 and row.get("ok") is True for row in command_rows)
    )
    ledger.append(_stage(
        "initializeMotorsWithoutMotion",
        "passed" if initialization_ok else "failed",
        "ClassControlInterface.initializeMotorsWithoutMotion lines 3181-3265",
        initialized,
    ))
    if not initialization_ok:
        invalidate_profile("initialize_without_motion_evidence_failed")
        return _preparation_result(authority, ledger)

    readbacks: list[dict[str, Any]] = []
    mismatches: list[dict[str, Any]] = []
    for command in command_rows:
        if command.get("command") != 5:  # only SAP parameter writes have exact GAP readback
            continue
        board = int(command["board"])
        motor = int(command["motor"])
        parameter = int(command["type"])
        expected = int(command["value"])
        try:
            observed = driver.motor_get_axis_param(board, parameter, motor=motor)
        except Exception as exc:
            observed = {"ack": None, "value": None, "error": f"{type(exc).__name__}: {exc}"}
        actual = observed.get("value") if isinstance(observed, Mapping) else None
        matched = _ack_status(observed) == 100 and type(actual) is int and actual == expected
        readback = {
            "label": command.get("label"),
            "board": board,
            "motor": motor,
            "parameter": parameter,
            "expected": expected,
            "actual": actual,
            "ack": observed.get("ack") if isinstance(observed, Mapping) else None,
            "matched": matched,
        }
        readbacks.append(readback)
        if not matched:
            mismatches.append(readback)
    readback_ok = bool(readbacks) and not mismatches
    ledger.append(_stage(
        "parameter_readback",
        "passed" if readback_ok else "failed",
        "ClassMotor SAP/GAP exact controller readback",
        {"readbacks": readbacks, "mismatches": mismatches},
    ))
    if not readback_ok:
        invalidate_profile("parameter_readback_failed")
        return _preparation_result(authority, ledger)

    return _preparation_result(authority, ledger)


def physical_aggregate_stop(
    driver: Any,
    authority: Serial206MotionAuthority,
    *,
    terminal_timeout_s: float = 3.0,
) -> dict[str, Any]:
    """Fan out ClassMotor StopMotor first, then verify each controller is stopped.

    Controller ACK plus zero speed proves the electronic terminal state. It does
    not prove that a mechanism physically moved or stopped, so operator-observed
    physical-effect verification remains explicitly false.
    """
    force_abort_latch = None
    components: list[dict[str, Any]] = []
    present_rows: list[tuple[dict[str, Any], int, int]] = []
    for component, board, motor, present in authority.components:
        if present is not True:
            components.append({
                "component": component,
                "board": board,
                "motor": motor,
                "status": "not_present_by_authority",
                "delivery_attempted": False,
                "stop_acknowledged": False,
                "zero_speed_verified": False,
                "applicable": False,
                "authority": authority.controller_evidence(),
            })
            continue
        try:
            stop = driver.motor_oem_board_stop(board, motor=motor, axis_name=component)
        except Exception as exc:
            stop = {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
        first = stop.get("first_delivery") if isinstance(stop, Mapping) else None
        second = stop.get("second_delivery") if isinstance(stop, Mapping) else None
        acknowledged = bool(
            isinstance(stop, Mapping)
            and stop.get("source_call_completed") is True
            and stop.get("source_return_code") == 0
            and _ack_status(first) == 100
            and _ack_status(second) == 100
        )
        row = {
            "component": component,
            "board": board,
            "motor": motor,
            "status": "stop_delivered" if acknowledged else "failed",
            "delivery_attempted": True,
            "stop_acknowledged": acknowledged,
            "zero_speed_verified": False,
            "terminal_speed": None,
            "applicable": True,
            "stop": stop,
            "terminal_readback": None,
            "source_anchor": "ClassMotor.StopMotor line 161+",
        }
        components.append(row)
        present_rows.append((row, board, motor))

    # ClassMotor.StopMotor must run before forceAbortMotion latches No24V.
    # The source StopMotor path rejects delivery after that latch is active.
    latch = getattr(driver, "motor_oem_force_abort_motion", None)
    force_abort_latch = latch(reason="forceAbortMotion") if callable(latch) else None

    # Complete the fan-out before polling so one slow component cannot delay a
    # StopMotor command to another component that may still be moving.
    for row, board, motor in present_rows:
        try:
            terminal = driver.motor_wait_stopped(
                board,
                motor=motor,
                timeout_s=max(0.1, float(terminal_timeout_s)),
                require_seen_nonzero=False,
            )
        except Exception as exc:
            terminal = {"ok": False, "terminal_speed": None, "error": f"{type(exc).__name__}: {exc}"}
        speed = terminal.get("terminal_speed") if isinstance(terminal, Mapping) else None
        if speed is None and isinstance(terminal, Mapping):
            speed = terminal.get("speed")
        if speed is None and isinstance(terminal, Mapping):
            speed = terminal.get("last_speed")
        terminal_completion_verified = bool(
            isinstance(terminal, Mapping)
            and (
                terminal.get("stopped") is True
                if "stopped" in terminal
                else terminal.get("ok") is True
            )
        )
        zero_speed = bool(terminal_completion_verified and type(speed) is int and speed == 0)
        verified = row["stop_acknowledged"] is True and zero_speed
        row.update({
            "status": "stopped" if verified else "failed",
            "zero_speed_verified": zero_speed,
            "terminal_speed": speed,
            "terminal_readback": terminal,
        })

    all_stopped = all(row["status"] in {"stopped", "not_present_by_authority"} for row in components)
    return {
        "schema_version": STOP_SCHEMA,
        "ok": all_stopped,
        "state": "stopped" if all_stopped else "failed_ambiguous",
        "machine_serial": authority.machine_serial,
        "controller_evidence": authority.controller_evidence(),
        "components": components,
        "stage_receipts": components,
        "delivery_attempted": any(row["delivery_attempted"] for row in components),
        "oem_24v_latch": force_abort_latch,
        "controller_terminal_state_verified": all_stopped,
        "physical_effect_verified": False,
        "physical_effect_verification_required": True,
        "source_anchors": [
            "ClassMotor.StopMotor line 161+",
            "ControlLib.forceAbortMotion lines 10564-10606",
            "ClassControlInterface.forceAbortMotion lines 5095-5104",
        ],
    }
