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


PREPARE_SCHEMA = "bioxp.oem_prepare_without_motion.v1"
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
_SERIAL206_ACTIVATION_BOARDS = (4, 5, 6)


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
    return {
        "schema_version": PREPARE_SCHEMA,
        "ok": ok,
        "state": "completed" if ok else "failed_closed",
        "machine_serial": authority.machine_serial,
        "controller_evidence": authority.controller_evidence(),
        "stage_ledger": ledger,
        "stage_receipts": ledger,
        "physical_motion": False,
        "physical_motion_commanded": False,
        "homing_performed": False,
        "global_24v_switch_claimed": False,
    }


def prepare_motion_without_motion(driver: Any, authority: Serial206MotionAuthority) -> dict[str, Any]:
    """Run only OEM board activation, no-motion parameterization, and queries."""
    ledger: list[dict[str, Any]] = []
    authority_ok = authority.machine_serial == OEM_MACHINE_SERIAL and authority.mutation_authorized is True
    ledger.append(_stage(
        "authority",
        "passed" if authority_ok else "failed",
        "immutable serial-206 OEM evidence lock",
        authority.controller_evidence(),
    ))
    if not authority_ok:
        return _preparation_result(authority, ledger)

    activation_rows: dict[int, Any] = {}
    for board in authority.activation_boards:
        try:
            row = driver.board_activate(board)
        except Exception as exc:
            row = {"board": board, "ack": None, "error": f"{type(exc).__name__}: {exc}"}
        activation_rows[board] = row
        ack_ok = _ack_status(row) == 100
        ledger.append(_stage(
            f"activate_board_{board}",
            "passed" if ack_ok else "failed",
            "ClassControlInterface.activateBoard lines 3474-3493",
            row,
        ))
        if not ack_ok:
            return _preparation_result(authority, ledger)

    board_wait = {
        "ok": True,
        "required_boards": list(authority.activation_boards),
        "initialized_boards": list(authority.activation_boards),
        "activation_receipts": activation_rows,
        "source_anchor": "ClassControlInterface.activateBoard lines 3474-3493",
    }
    ledger.append(_stage(
        "board_7_resolution",
        "not_applicable",
        "serial-206 board authority: board 7 is chiller, not a ClassMotor movement board",
        {
            "board": 7,
            "role": "chiller_auxiliary_temperature_controller",
            "activation_required": False,
            "command_sent": False,
            "invalid_command_status": 2,
            "resolution": "status 2 is not promoted to motor activation success",
        },
    ))

    try:
        initialized = driver.oem_initialize_without_motion_test_case(board_wait=board_wait)
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
        return _preparation_result(authority, ledger)

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
        observed_ok = bool(_ack_status(observation) == 100 and observation.get("value") == 1)
        ledger.append(_stage(
            stage_id,
            "passed" if observed_ok else "failed",
            f"ClassIOControl query {label} sensor read-only",
            observation,
        ))
        if not observed_ok:
            break

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
            stop = driver.motor_stop(board, motor=motor)
        except Exception as exc:
            stop = {"ack": None, "ok": False, "error": f"{type(exc).__name__}: {exc}"}
        acknowledged = bool(_ack_status(stop) == 100 and stop.get("ok") is True)
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
        zero_speed = bool(isinstance(terminal, Mapping) and terminal.get("ok") is True and type(speed) is int and speed == 0)
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
        "controller_terminal_state_verified": all_stopped,
        "physical_effect_verified": False,
        "physical_effect_verification_required": True,
        "source_anchors": [
            "ClassMotor.StopMotor line 161+",
            "ControlLib.forceAbortMotion lines 10564-10606",
            "ClassControlInterface.forceAbortMotion lines 5095-5104",
        ],
    }
