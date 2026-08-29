import struct
import time
import math
from enum import IntEnum
from typing import Any, Callable

try:
    import can  # type: ignore
except ModuleNotFoundError:  # pragma: no cover - exercised indirectly by offline tests
    class _OfflineCanError(Exception):
        pass

    class _OfflineMessage:
        def __init__(self, *, arbitration_id: int, data, is_extended_id: bool = False):
            self.arbitration_id = int(arbitration_id)
            self.data = list(data)
            self.dlc = len(self.data)
            self.is_extended_id = bool(is_extended_id)

    class _OfflineInterface:
        @staticmethod
        def Bus(*args, **kwargs):
            raise RuntimeError("python-can is required for live SocketCAN hardware access")

    class _OfflineCanModule:
        CanError = _OfflineCanError
        Message = _OfflineMessage
        interface = _OfflineInterface()

    can = _OfflineCanModule()


class BoardAssy(IntEnum):
    THERMAL_CONTROLLER = 0x05  # Example ID based on CAN topology
    CHILLER_BOARD = 0x07       # Derived from ClassChillerBoard init
    MOTOR_CONTROLLER = 0x03    # Example ID for Gantry
    # OEM ClassPipette uses 0x101 | (pipette_id << 3) for the command frame.
    # Keep this enum as the first pipette command address for compatibility;
    # query/control/fragment addresses are derived by BioXpCanDriver.pipette_can_ids().
    PIPETTE_CONTROLLER = 0x101


class MotorAxis(IntEnum):
    X = 0
    Y = 1
    Z = 2
    GRIPPER = 3


_PIPETTE_COMPLETION_CATEGORIES = frozenset(
    {
        "pipette_initialize",
        "pipette_initiate_group",
        "pipette_eject_tip",
        "dispense_all",
        "dispense",
        "aspirate",
        "aspirate_air",
        "set_top_speed",
        "terminate_pipette",
        "start_fluid_detection",
    }
)

PIPETTE_DATA_QUERY_LABELS: dict[str, tuple[str, str | None]] = {
    "?40": ("total_operating_time", "minutes"),
    "?41": ("device_power_ups", None),
    "?42": ("pump_initializations", None),
    "?44": ("plunger_distance", "meters"),
    "?45": ("plunger_movements", None),
    "?47": ("tip_ejections", None),
    "?51": ("highest_power_supply_voltage", "volts"),
    "?52": ("lowest_power_supply_voltage", "volts"),
    "?53": ("current_temperature", "fahrenheit"),
    "?54": ("highest_temperature", "fahrenheit"),
    "?55": ("lowest_temperature", "fahrenheit"),
    "?58": ("highest_pressure", "psi"),
    "?59": ("lowest_pressure", "psi"),
}


def process_pipette_message(
    dlc: int,
    message: list[int] | bytes | bytearray,
    *,
    arbitration_id: int | None = None,
    command_name: str | None,
    state: dict[str, Any] | None = None,
    received_at: float | None = None,
) -> dict[str, Any]:
    """Apply the literal serial-206 ``ClassPipette.processMessage`` rules.

    The returned projection keeps the OEM error/event result separate from the
    Linux transport validity gate.  ``message`` is the decoded pipette data
    array, not the enclosing Novo record.
    """
    current = dict(state or {})
    current.setdefault("initialized", False)
    current.setdefault("error_code", 0)
    current.setdefault("error_queue", [])
    current.setdefault("tip_loaded", False)
    current.setdefault("diagnosis", None)
    current.setdefault("fluid_timestamp", None)
    current.setdefault("initialization_counter", 0)
    data = [int(value) & 0xFF for value in message]
    selected_dlc = int(dlc)
    if command_name == "query_status":
        # ClassPipette.QueryStatus() resets the local status/error-vector
        # projection before issuing Q1.  The asynchronous reply then repopulates
        # the raw channel error and any returned queue entries.
        current["initialized"] = False
        current["error_queue"] = []
    bounded = data[:selected_dlc]
    can_id = int(arbitration_id or 0) & 0xFFFF
    oem_message = [can_id & 0xFF, (can_id >> 8) & 0xFF, *bounded]
    if selected_dlc == 0:
        return {
            **current,
            "ok": True,
            "reply_presence": "immediate_ack",
            "immediate_ack_received": True,
            "command_name": command_name,
            "completion_signal": False,
            "oem_error_code": int(current.get("error_code", 0)),
            "event_error_code": 0,
            "message": oem_message,
        }
    if selected_dlc < 0 or len(data) != selected_dlc:
        return {
            **current,
            "ok": False,
            "reply_presence": "malformed",
            "command_name": command_name,
            "completion_signal": False,
            "oem_error_code": None,
            "event_error_code": None,
            "message": oem_message,
            "error": "pipette_async_message_dlc_mismatch",
        }

    error_code = int(oem_message[2])
    current["error_code"] = error_code
    if error_code == 0x20:
        current["initialized"] = True
    event_error_code = error_code
    if error_code == 42 and command_name == "pipette_eject_tip":
        event_error_code = 0

    completion_signal = selected_dlc == 2 and command_name in _PIPETTE_COMPLETION_CATEGORIES
    if selected_dlc == 2 and command_name == "start_fluid_detection":
        current["fluid_timestamp"] = float(received_at if received_at is not None else time.time())
    elif selected_dlc > 2 and command_name in {"query_error_log", "query_status"}:
        queue = list(current.get("error_queue", []))
        for value in oem_message[3:selected_dlc]:
            if value == 0x40:
                queue.clear()
            else:
                queue.append(int(value))
        current["error_queue"] = queue
    elif selected_dlc > 2 and command_name in {"pipette_initialize", "pipette_initiate_group"}:
        current["initialization_counter"] = int(current.get("initialization_counter", 0)) + 1
    elif selected_dlc > 2 and command_name == "query_tip_status":
        # OEM processMessage indexes the raw asynchronous message at msg[4].
        # A short frame is retained as invalid rather than being normalized to
        # a successful no-tip state by the Linux adapter.
        current["tip_loaded"] = bool(len(oem_message) > 4 and oem_message[4] == ord("1"))
    elif selected_dlc > 2 and command_name == "execute_diagnoses":
        current["diagnosis"] = bytes(oem_message).decode("ascii", errors="replace")[2:]
    elif selected_dlc > 2 and command_name == "start_fluid_detection":
        current["fluid_timestamp"] = float(received_at if received_at is not None else time.time())

    return {
        **current,
        "ok": True,
        "reply_presence": "nonempty",
        "command_name": command_name,
        "completion_signal": completion_signal,
        "oem_error_code": error_code,
        "event_error_code": event_error_code,
        "tip_lost_suppressed_for_eject": bool(error_code == 42 and command_name == "pipette_eject_tip"),
        "message": oem_message,
    }


class BioXpCanDriver:
    """
    Python CAN Driver for the BioXP 3200.
    Bypasses the proprietary Windows .NET DLLs and sends raw byte
    payloads directly over the SocketCAN Linux interface.
    """

    PIPETTE_TRANSACTION_TIMEOUT_S = 60.0
    PIPETTE_CHANNELS = frozenset(range(4))

    @classmethod
    def _validate_pipette_id(cls, pipette_id: int) -> int:
        selected = int(pipette_id)
        if selected not in cls.PIPETTE_CHANNELS:
            raise ValueError(f"pipette_id must be one of 0..3, got {selected}")
        return selected

    def __init__(self, channel='can0', bitrate=1000000, *, pipette_id: int = 0, response_timeout_s: float = 60.0, pipette_error_callback: Callable[[int, int], None] | None = None):
        # The BioXP USB-to-CAN adapter should map to can0 in Linux
        self.bus = can.interface.Bus(bustype='socketcan', channel=channel, bitrate=bitrate)
        self.channel = channel
        self.bitrate = int(bitrate)
        self.pipette_id = self._validate_pipette_id(pipette_id)
        self.response_timeout_s = float(response_timeout_s)
        self._sleep = time.sleep
        self._pipette_message_state: dict[str, Any] = {}
        self._pipette_last_command: str | None = None
        self._pipette_completion_owner_token: str | None = None
        self._pipette_error_callback = pipette_error_callback

    def pipette_can_ids(self) -> dict[str, int]:
        pipette_id = self._validate_pipette_id(self.pipette_id)
        self.pipette_id = pipette_id
        tx = {
            "control": int(0x100 | (pipette_id << 3)),
            "command": int(0x101 | (pipette_id << 3)),
            "first_part_command": int(0x103 | (pipette_id << 3)),
            "middle_part_command": int(0x104 | (pipette_id << 3)),
            "report": int(0x106 | (pipette_id << 3)),
        }
        return {**tx, **{f"{name}_rx": value | 0x400 for name, value in tx.items()}}

    @staticmethod
    def _frame_payload(command: list[int] | bytes | bytearray) -> list[int]:
        payload = [int(byte) & 0xFF for byte in command]
        if len(payload) > 8:
            raise ValueError("classic CAN payload exceeds 8 bytes")
        return payload

    @staticmethod
    def _reply_payload(reply: Any) -> dict[str, Any]:
        data = [int(byte) & 0xFF for byte in getattr(reply, 'data', [])]
        ascii_text = ''.join(chr(byte) if 32 <= byte <= 126 else '.' for byte in data)
        return {
            "arbitration_id": int(getattr(reply, 'arbitration_id', -1)),
            "data": data,
            "ascii": ascii_text,
            "dlc": int(getattr(reply, 'dlc', len(data))),
            "is_extended_id": bool(getattr(reply, 'is_extended_id', False)),
        }

    @staticmethod
    def _ack_ok(data: list[int], *, ack_mode: str) -> bool:
        if not data:
            return ack_mode == "command"
        if ack_mode == "query":
            return True
        # OEM ClassPipette treats 0x20 as the no-error completion code. Depending
        # on whether the caller observes the raw CAN frame or ClassCanLib's trimmed
        # return array, that byte may be first or at offset 2.
        if data[0] == 0x20:
            return True
        if len(data) > 2 and data[2] == 0x20:
            return True
        return False

    def _receive_reply(self, *, timeout_s: float, ack_mode: str, expected_arbitration_id: int | None = None) -> dict[str, Any]:
        recv = getattr(self.bus, 'recv', None)
        if not callable(recv):
            return {
                "ok": False,
                "received": False,
                "error": "bus_recv_unavailable",
                "timeout_s": float(timeout_s),
            }
        skipped: list[dict[str, Any]] = []
        skipped_total = 0
        max_skipped_frames = 12
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        inverse_pipette_ids = {int(v): str(k) for k, v in self.pipette_can_ids().items()}
        while True:
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0:
                return {
                    "ok": False,
                    "received": False,
                    "error": "ack_timeout",
                    "timeout_s": float(timeout_s),
                    "demux": {
                        "expected_arbitration_id": expected_arbitration_id,
                        "matched_address": None,
                        "skipped_count": skipped_total,
                        "skipped_frames": skipped,
                        "skipped_frames_truncated": skipped_total > len(skipped),
                    },
                }
            reply = recv(timeout=float(remaining_s))
            if reply is None:
                return {
                    "ok": False,
                    "received": False,
                    "error": "ack_timeout",
                    "timeout_s": float(timeout_s),
                    "demux": {
                        "expected_arbitration_id": expected_arbitration_id,
                        "matched_address": None,
                        "skipped_count": skipped_total,
                        "skipped_frames": skipped,
                        "skipped_frames_truncated": skipped_total > len(skipped),
                    },
                }
            payload = self._reply_payload(reply)
            if expected_arbitration_id is not None and int(payload["arbitration_id"]) != int(expected_arbitration_id):
                skipped_total += 1
                if len(skipped) < max_skipped_frames:
                    skipped.append(payload)
                continue
            data = payload["data"]
            ack_ok = self._ack_ok(data, ack_mode=ack_mode)
            payload.update(
                {
                    "ok": ack_ok,
                    "received": True,
                    "ack_mode": ack_mode,
                    "error_code": data[0] if data else None,
                    "oem_no_error_code_present": 0x20 in data,
                    "demux": {
                        "expected_arbitration_id": expected_arbitration_id,
                        "matched_address": inverse_pipette_ids.get(int(payload["arbitration_id"])),
                        "skipped_count": skipped_total,
                        "skipped_frames": skipped,
                        "skipped_frames_truncated": skipped_total > len(skipped),
                    },
                }
            )
            if not ack_ok:
                payload["error"] = "pipette_reply_error"
            return payload

    def _send_packet(
        self,
        board_id: int,
        command: list[int] | bytes | bytearray,
        *,
        require_ack: bool = False,
        response_timeout_s: float | None = None,
        ack_mode: str = "command",
        command_name: str | None = None,
        wait_for_completion: bool = True,
    ) -> dict[str, Any]:
        """
        Broadcast a single 8-byte CAN packet and optionally require a response.

        For pipette operations this must be used with require_ack=True; TX success
        alone is only proof that SocketCAN accepted the frame, not that the ADP
        pipette firmware executed it.
        """
        payload = self._frame_payload(command)
        msg = can.Message(
            arbitration_id=int(board_id),
            data=payload,
            is_extended_id=False,
        )
        base = {
            "ok": False,
            "delivery_verified": False,
            "controller_acknowledged": False,
            "completion_verified": False,
            "board_id": int(board_id),
            "payload": payload,
            "dlc": len(payload),
            "command_name": command_name,
            "ack_required": bool(require_ack),
        }
        timeout_s = float(response_timeout_s if response_timeout_s is not None else self.response_timeout_s)
        transact_can = getattr(self.bus, "transact_can", None)
        if callable(transact_can) and require_ack:
            function = int(board_id) & 0x7
            try:
                initialization = command_name in {"pipette_initialize", "pipette_initiate_group"}
                provenance = transact_can(
                    msg,
                    channel=int(self.pipette_id),
                    expected_function=function,
                    timeout_s=timeout_s,
                    matcher_name=command_name or f"pipette_function_{function}",
                    initialization=initialization,
                    completion_timeout_s=10.0 if command_name == "pipette_initiate_group" else 60.0,
                    allow_multipart=ack_mode == "query",
                    wait_for_completion=wait_for_completion,
                )
            except Exception as exc:
                return {**base, "tx_ok": False, "error": str(exc), "provenance": None}
            if isinstance(provenance, dict):
                owner_token = provenance.get("completion_owner_token")
                if isinstance(owner_token, str) and owner_token:
                    self._pipette_completion_owner_token = owner_token
            frames = list(provenance.get("frames", []))
            data: list[int] = []
            for frame in frames:
                data.extend(int(byte) for byte in frame.get("data", []))
            observed = frames[-1] if frames else {}
            provenance_ok = bool(provenance.get("ok")) if isinstance(provenance, dict) else False
            immediate_ack_received = bool(
                isinstance(provenance, dict)
                and (
                    provenance.get("immediate_ack_received") is True
                    or (
                        provenance.get("outcome") == "ack"
                        and any(frame.get("dlc") == 0 for frame in frames)
                    )
                )
            )
            ack_ok = bool(
                provenance_ok and self._ack_ok(data, ack_mode=ack_mode)
                if ack_mode == "query"
                else immediate_ack_received
            )
            ack = {
                "ok": ack_ok,
                "received": bool(frames),
                "arbitration_id": observed.get("arbitration_id"),
                "dlc": observed.get("dlc"),
                "data": data,
                "raw": observed.get("raw"),
                "ascii": ''.join(chr(byte) if 32 <= byte <= 126 else '.' for byte in data),
                "outcome": "ack" if immediate_ack_received else provenance.get("outcome"),
                "immediate_ack": next((frame for frame in frames if frame.get("dlc") == 0), None),
                "completion": next((frame for frame in reversed(frames) if frame.get("dlc", 0) > 0), None),
            }
            message_state = self._apply_pipette_provenance(provenance, command_name)
            if not ack_ok:
                ack["error"] = "ack_timeout" if provenance.get("outcome") == "timeout" else "pipette_reply_error"
            return {
                **base,
                "ok": ack_ok,
                "tx_ok": True,
                "delivery_verified": True,
                "immediate_ack_received": immediate_ack_received,
                "controller_acknowledged": immediate_ack_received,
                "completion_verified": bool(
                    ack_mode != "query"
                    and provenance.get("completion_received", False)
                    if isinstance(provenance, dict)
                    else False
                ),
                "semantic_query_response_verified": bool(
                    ack_mode == "query"
                    and isinstance(provenance, dict)
                    and provenance.get("semantic_query_response_verified", provenance_ok)
                ),
                "completion_deferred": bool(
                    provenance.get("completion_deferred", False) if isinstance(provenance, dict) else False
                ),
                "completion_owner_token": (
                    provenance.get("completion_owner_token") if isinstance(provenance, dict) else None
                ),
                "ack": ack,
                "provenance": provenance,
                "pipette_message_state": message_state,
                "error": None if ack_ok else ack["error"],
            }
        if not wait_for_completion:
            raise ValueError("deferred pipette completion requires the shared NovoRouter transaction owner")
        try:
            self.bus.send(msg)
            print(f"[CAN TX] ID: {hex(int(board_id))} | Data: {[hex(b) for b in payload]}")
            base["tx_ok"] = True
        except can.CanError as e:
            print(f"CAN Bus Error: {e}")
            return {
                **base,
                "tx_ok": False,
                "error": str(e),
            }
        if not require_ack:
            return {
                **base,
                "ok": True,
                "delivery_verified": True,
                "controller_acknowledged": False,
                "completion_verified": False,
            }
        ack = self._receive_reply(
            timeout_s=timeout_s,
            ack_mode=ack_mode,
            expected_arbitration_id=int(board_id) | 0x400,
        )
        if ack_mode != "query":
            return {
                **base,
                "ok": False,
                "tx_ok": True,
                "delivery_verified": True,
                "controller_acknowledged": False,
                "completion_verified": False,
                "ack": ack,
                "pipette_message_state": dict(getattr(self, "_pipette_message_state", {})),
                "error": "shared_novo_router_required_for_command_lifecycle",
            }
        message_state = self.process_pipette_message(
            int(ack.get("dlc", len(ack.get("data", [])))),
            list(ack.get("data", [])),
            arbitration_id=ack.get("arbitration_id"),
            command_name=command_name,
            received_at=ack.get("received_at"),
        )
        return {
            **base,
            "ok": bool(ack.get("ok")),
            "tx_ok": True,
            "delivery_verified": bool(base.get("tx_ok")),
            "controller_acknowledged": False,
            "completion_verified": False,
            "ack": ack,
            "pipette_message_state": message_state,
            "error": None if ack.get("ok") else ack.get("error", "query_failed"),
        }

    def _send_ascii_packet(
        self,
        board_id: int,
        ascii_command: str,
        *,
        require_ack: bool = False,
        response_timeout_s: float | None = None,
        ack_mode: str = "command",
        command_name: str | None = None,
        wait_for_completion: bool = True,
    ) -> dict[str, Any]:
        encoded = str(ascii_command).encode('ascii')
        if len(encoded) > 8:
            ids = self.pipette_can_ids()
            chunks = [encoded[index:index + 8] for index in range(0, len(encoded), 8)]
            tx_ids = [
                ids["first_part_command"],
                *([ids["middle_part_command"]] * max(0, len(chunks) - 2)),
                ids["report" if int(board_id) == ids["report"] else "command"],
            ]
            messages = [
                can.Message(arbitration_id=int(tx_id), data=list(chunk), is_extended_id=False)
                for tx_id, chunk in zip(tx_ids, chunks)
            ]
            transact_many = getattr(self.bus, "transact_can_many", None)
            if not callable(transact_many):
                raise ValueError("fragmented pipette TX requires the shared NovoRouter multi-frame owner")
            initialization = command_name in {"pipette_initialize", "pipette_initiate_group"}
            provenance = transact_many(
                messages,
                channel=int(self.pipette_id),
                expected_function=int(tx_ids[-1]) & 0x7,
                timeout_s=float(response_timeout_s if response_timeout_s is not None else self.response_timeout_s),
                matcher_name=command_name or str(ascii_command),
                initialization=initialization,
                completion_timeout_s=10.0 if command_name == "pipette_initiate_group" else 60.0,
                allow_multipart=ack_mode == "query",
                wait_for_completion=wait_for_completion,
            )
            if isinstance(provenance, dict):
                owner_token = provenance.get("completion_owner_token")
                if isinstance(owner_token, str) and owner_token:
                    self._pipette_completion_owner_token = owner_token
            frames = list(provenance.get("frames", []))
            data = [int(byte) for frame in frames for byte in frame.get("data", [])]
            observed = frames[-1] if frames else {}
            provenance_ok = bool(provenance.get("ok")) if isinstance(provenance, dict) else False
            immediate_ack_received = bool(
                isinstance(provenance, dict)
                and (
                    provenance.get("immediate_ack_received") is True
                    or (
                        provenance.get("outcome") == "ack"
                        and any(frame.get("dlc") == 0 for frame in frames)
                    )
                )
            )
            ack_ok = bool(
                provenance_ok and self._ack_ok(data, ack_mode=ack_mode)
                if ack_mode == "query"
                else immediate_ack_received
            )
            ack = {
                "ok": ack_ok,
                "received": bool(frames),
                "arbitration_id": observed.get("arbitration_id"),
                "dlc": observed.get("dlc"),
                "data": data,
                "raw": observed.get("raw"),
                "ascii": ''.join(chr(byte) if 32 <= byte <= 126 else '.' for byte in data),
                "outcome": "ack" if immediate_ack_received else provenance.get("outcome"),
                "immediate_ack": next((frame for frame in frames if frame.get("dlc") == 0), None),
                "completion": next((frame for frame in reversed(frames) if frame.get("dlc", 0) > 0), None),
            }
            message_state = self._apply_pipette_provenance(provenance, command_name)
            if not ack_ok:
                ack["error"] = "ack_timeout" if provenance.get("outcome") == "timeout" else "pipette_reply_error"
            return {
                "ok": ack_ok,
                "tx_ok": True,
                "delivery_verified": True,
                "immediate_ack_received": immediate_ack_received,
                "controller_acknowledged": immediate_ack_received,
                "completion_verified": bool(
                    ack_mode != "query"
                    and provenance.get("completion_received", False)
                    if isinstance(provenance, dict)
                    else False
                ),
                "semantic_query_response_verified": bool(
                    ack_mode == "query"
                    and isinstance(provenance, dict)
                    and provenance.get("semantic_query_response_verified", provenance_ok)
                ),
                "completion_deferred": bool(
                    provenance.get("completion_deferred", False) if isinstance(provenance, dict) else False
                ),
                "completion_owner_token": (
                    provenance.get("completion_owner_token") if isinstance(provenance, dict) else None
                ),
                "ack": ack,
                "provenance": provenance,
                "pipette_message_state": message_state,
                "error": None if ack_ok else ack["error"],
                "ascii_command": str(ascii_command),
                "length": len(encoded),
                "fragmented": True,
                "tx_frame_ids": tx_ids,
                "tx_frame_count": len(chunks),
                "fragment_policy": "first_103_middle_104_final_expected_family",
            }
        return {
            **self._send_packet(
                int(board_id),
                list(encoded),
                require_ack=require_ack,
                response_timeout_s=response_timeout_s,
                ack_mode=ack_mode,
                command_name=command_name or str(ascii_command),
                wait_for_completion=wait_for_completion,
            ),
            "ascii_command": str(ascii_command),
            "length": len(encoded),
        }

    def _send_pipette_command(
        self,
        ascii_command: str,
        *,
        address: str = "command",
        ack_mode: str = "command",
        command_name: str | None = None,
        wait_for_completion: bool = True,
    ) -> dict[str, Any]:
        ids = self.pipette_can_ids()
        if address not in ids:
            raise ValueError(f"Unknown pipette CAN address kind: {address!r}")
        self._pipette_last_command = command_name or f"pipette:{ascii_command}"
        return self._send_ascii_packet(
            ids[address],
            ascii_command,
            require_ack=True,
            response_timeout_s=self.response_timeout_s,
            ack_mode=ack_mode,
            command_name=command_name or f"pipette:{ascii_command}",
            wait_for_completion=wait_for_completion,
        )

    def process_pipette_message(
        self,
        dlc: int,
        message: list[int] | bytes | bytearray,
        *,
        arbitration_id: int | None = None,
        command_name: str | None = None,
        received_at: float | None = None,
    ) -> dict[str, Any]:
        selected = command_name or self._pipette_last_command
        self._pipette_message_state = process_pipette_message(
            dlc,
            message,
            arbitration_id=arbitration_id,
            command_name=selected,
            state=getattr(self, "_pipette_message_state", {}),
            received_at=received_at,
        )
        callback = getattr(self, "_pipette_error_callback", None)
        event_error_code = self._pipette_message_state.get("event_error_code")
        if callable(callback) and isinstance(event_error_code, int) and event_error_code not in {0, 0x20}:
            try:
                callback(int(self.pipette_id), int(event_error_code))
            except Exception as exc:  # event publication must not kill the reader owner
                self._pipette_message_state["error_callback_error"] = repr(exc)
        return dict(self._pipette_message_state)

    def _apply_pipette_provenance(self, provenance: Any, command_name: str | None) -> dict[str, Any]:
        state = dict(getattr(self, "_pipette_message_state", {}))
        for frame in provenance.get("frames", []) if isinstance(provenance, dict) else []:
            if not isinstance(frame, dict):
                continue
            state = self.process_pipette_message(
                int(frame.get("dlc", len(frame.get("data", [])))),
                list(frame.get("data", [])),
                arbitration_id=frame.get("arbitration_id"),
                command_name=command_name,
                received_at=frame.get("received_at"),
            )
        completion = provenance.get("completion") if isinstance(provenance, dict) else None
        if isinstance(completion, dict) and isinstance(completion.get("data"), list):
            state = self.process_pipette_message(
                int(completion.get("observed_rx_dlc", len(completion["data"]))),
                list(completion["data"]),
                arbitration_id=completion.get("observed_rx_id"),
                command_name=completion.get("command_name") or command_name,
                received_at=completion.get("receive_timestamp"),
            )
        return state

    def close(self):
        shutdown = getattr(self.bus, 'shutdown', None)
        if callable(shutdown):
            shutdown()

    # ==========================================
    # THERMAL CONTROL SYSTEM (ClassThermalControl)
    # ==========================================
    def set_thermal_temperature(self, target_temp_c: float):
        """
        Reverse Engineered from `ClassThermalControl.setTargetTemperature(double temp)`
        Multiplies the float by 1000 to cast to a 32-bit int, then packs it Little Endian.
        Action ID: 140 (0x8C)
        """
        # DLL constraint logic (safety cap)
        if target_temp_c > 100.0:
            target_temp_c = 100.0

        temp_scaled = int(target_temp_c * 1000.0)

        # Pack into 4 bytes (Little Endian)
        packed_temp = struct.pack('<i', temp_scaled)

        # Action 140 packet structure: [140, 0, AxisID, bytes3, bytes2, bytes1, bytes0]
        # Note: Axis ID 0 is often used for the main block.
        axis_id = 0

        packet = [
            140,
            0,
            axis_id,
            packed_temp[3],
            packed_temp[2],
            packed_temp[1],
            packed_temp[0]
        ]

        return self._send_packet(BoardAssy.THERMAL_CONTROLLER, packet)

    # ==========================================
    # PIPETTE SYSTEM (OEM ClassPipette-compatible ADP CAN framing)
    # ==========================================
    @staticmethod
    def _format_pipette_volume(volume_ul: float) -> str:
        volume_ul = float(volume_ul)
        if not math.isfinite(volume_ul) or volume_ul < 0.0:
            raise ValueError("pipette volume must be a finite non-negative number")
        if volume_ul >= 100.0:
            return str(int(volume_ul + 0.5))
        if volume_ul == float(int(volume_ul)):
            return f"{int(volume_ul)}"
        return f"{volume_ul:.1f}"

    @staticmethod
    def _require_oem_pressure_profile(profile: str) -> str:
        selected = str(profile).upper()
        if selected != "1R":
            raise ValueError("serial-206 OEM pipette commands require pressure profile 1R")
        return selected

    @staticmethod
    def _parse_tip_loaded(result: dict[str, Any]) -> bool | None:
        ack = result.get("ack", {}) if isinstance(result, dict) else {}
        data = ack.get("data", []) if isinstance(ack, dict) else []
        if len(data) != 3 or list(data[:2]) != [0x20, 0x60]:
            return None
        if data[2] == ord('1'):
            return True
        if data[2] == ord('0'):
            return False
        return None

    @staticmethod
    def _parse_numeric_ascii_from_ack(result: dict[str, Any]) -> float | None:
        ack = result.get("ack", {}) if isinstance(result, dict) else {}
        data = ack.get("data", []) if isinstance(ack, dict) else []
        if len(data) <= 2 or list(data[:2]) != [0x20, 0x60]:
            return None
        try:
            text = bytes(data[2:]).decode("ascii")
            if not text or text.strip() != text:
                return None
            value = float(text)
            return value if math.isfinite(value) else None
        except (UnicodeDecodeError, ValueError):
            return None

    def pipette_initialize(self, pressure_profile='1R'):
        self._require_oem_pressure_profile(pressure_profile)
        wake_byte = 0x20 | int(self.pipette_id)
        wake = self._send_packet(0x080, [wake_byte, wake_byte], command_name="pipette_wake_address")
        self._sleep(0.100)
        result = self._send_pipette_command(
            "WR",
            command_name="pipette_initialize",
            wait_for_completion=False,
        )
        result["wake"] = wake
        result["wake_delay_ms"] = 100
        result["wake_transport_tx"] = bool(wake.get("tx_ok"))
        result["wake_response_observed"] = bool(wake.get("ack", {}).get("received"))
        result["wake_nonspace_observation"] = None
        result["wake_continuation_gate"] = "none_oem_source_does_not_gate_on_wake_reply"
        result["immediate_ack_received"] = bool(
            result.get("ok") and result.get("ack", {}).get("outcome") == "ack"
        )
        result["initialized_after_valid_completion"] = False
        result["ok"] = bool(result["immediate_ack_received"])
        result["requested_pressure_profile"] = str(pressure_profile).upper()
        return result

    def _enrich_pipette_completion(self, result: dict[str, Any]) -> dict[str, Any]:
        data = result.get("data")
        if not isinstance(data, list):
            return result
        state = self.process_pipette_message(
            int(result.get("observed_rx_dlc", len(data))),
            data,
            arbitration_id=result.get("observed_rx_id"),
            command_name=result.get("command_name") or self._pipette_last_command,
            received_at=result.get("receive_timestamp"),
        )
        event_error = state.get("event_error_code")
        result["pipette_message_state"] = state
        result["oem_error_code"] = state.get("oem_error_code")
        result["event_error_code"] = event_error
        result["ok"] = bool(result.get("ok") and event_error in {0, 0x20})
        if not result["ok"] and result.get("outcome") == "completion":
            result["outcome"] = "oem_error"
        return result

    def _wait_owned_pipette_completion(self, timeout_s: float) -> dict[str, Any]:
        wait = getattr(self.bus, "wait_pipette_completion", None)
        if not callable(wait):
            return {"ok": False, "channel": int(self.pipette_id), "outcome": "completion_wait_unavailable"}
        owner_token = self._pipette_completion_owner_token
        try:
            raw = wait(
                int(self.pipette_id),
                float(timeout_s),
                owner_token=owner_token,
            )
        except TypeError:
            if owner_token is not None:
                raise
            raw = wait(int(self.pipette_id), float(timeout_s))
        if owner_token == self._pipette_completion_owner_token:
            self._pipette_completion_owner_token = None
        if not isinstance(raw, dict):
            return {"ok": False, "channel": int(self.pipette_id), "outcome": "invalid_completion_result", "result": repr(raw)}
        return self._enrich_pipette_completion(raw)

    def wait_pipette_initialization_completion(self, timeout_s: float):
        return self._wait_owned_pipette_completion(timeout_s)

    def wait_pipette_command_completion(self, timeout_s: float):
        return self._wait_owned_pipette_completion(timeout_s)

    def pipette_initiate_group(self):
        result = self._send_pipette_command(
            "WR",
            command_name="pipette_initiate_group",
            wait_for_completion=False,
        )
        result["immediate_ack_received"] = bool(
            result.get("ok") and result.get("ack", {}).get("outcome") == "ack"
        )
        result["initialized_after_valid_completion"] = False
        result["ok"] = bool(result["immediate_ack_received"])
        result["group_completion_timeout_ms"] = 10_000
        return result

    def query_firmware(self, number: int = 1):
        result = self._send_pipette_command(f"&{int(number)}", address="report", ack_mode="query", command_name="query_firmware")
        ack = result.get("ack", {}) if isinstance(result, dict) else {}
        data = list(ack.get("data", [])) if isinstance(ack, dict) else []
        reply_received = bool(ack.get("received")) if isinstance(ack, dict) else False
        firmware = bytes(data[2:]).decode("ascii", errors="replace") if len(data) >= 2 else None
        semantic_ok = bool(reply_received and firmware is not None and firmware != "")
        return {
            **result,
            "ok": semantic_ok,
            "query": f"&{int(number)}",
            "reply_received": reply_received,
            "firmware": firmware,
            "semantic_ok": semantic_ok,
            "hardware_truth_level": "hardware_query" if semantic_ok else ("unparsed_hardware_reply" if reply_received else "no_readback"),
            "oem_source_anchor": "ClassPipette.QueryFirmware:599-619",
        }

    def query_status(self):
        result = self._send_pipette_command("Q1", address="report", ack_mode="query", command_name="query_status")
        ack = result.get("ack", {}) if isinstance(result, dict) else {}
        data = list(ack.get("data", [])) if isinstance(ack, dict) else []
        reply_received = bool(ack.get("received")) if isinstance(ack, dict) and "received" in ack else bool(data)
        semantic_ok = bool(len(data) >= 3 and data[:2] == [0x20, 0x60])
        raw_statuses = [int(value) for value in data[2:]] if semantic_ok else []
        raw_status = raw_statuses[0] if raw_statuses else None
        error_codes: list[int] = []
        if semantic_ok:
            # Exact ClassPipette.processMessage()/GetErrorCode() behavior for
            # Query_Error: byte index 2 is m_error_code, while only indexes
            # 3..dlc-1 populate m_ErrorCollection. 0x40 clears that queue.
            for value in data[3:]:
                if int(value) == 0x40:
                    error_codes.clear()
                else:
                    error_codes.append(int(value))
        error_code = error_codes[0] if error_codes else (0 if semantic_ok else None)
        error_free = bool(semantic_ok and not error_codes)
        error = result.get("error")
        if reply_received and not semantic_ok:
            error = "malformed_status_reply"
        elif reply_received and not error_free and error_code is not None:
            error = f"pipette_reported_error_0x{error_code:02x}"
        elif reply_received and error_free:
            error = None
        return {
            **result,
            "ok": bool(reply_received and semantic_ok and error_free),
            "reply_received": reply_received,
            "semantic_ok": semantic_ok,
            "oem_raw_status": raw_status,
            "oem_process_error_code": int(data[2]) if semantic_ok else None,
            "oem_raw_statuses": raw_statuses,
            "oem_error_code": error_code,
            "oem_error_codes": error_codes,
            "oem_error_free": error_free,
            "error": error,
            "oem_source_anchor": "ClassPipette.QueryStatus/processMessage/GetErrorCode: Q1",
        }

    def enable_pressure_stream(self, enabled: bool):
        setup = self._send_pipette_command("b15R", command_name="set_pressure_stream_parameter") if enabled else None
        result = self._send_pipette_command("o0,1R" if enabled else "o0,0R", command_name="enable_pressure_stream")
        result["parameter_setup"] = setup
        return result

    def query_tip_status(self):
        result = self._send_pipette_command("?31", address="report", ack_mode="query", command_name="query_tip_status")
        tip_loaded = self._parse_tip_loaded(result)
        ack = result.get("ack", {}) if isinstance(result, dict) else {}
        reply_received = bool(ack.get("received")) if isinstance(ack, dict) else False
        semantic_ok = tip_loaded is not None
        if reply_received and not semantic_ok and isinstance(result.get("provenance"), dict):
            result["provenance"]["outcome"] = "malformed"
        return {
            **result,
            "ok": bool(reply_received and semantic_ok),
            "reply_received": reply_received,
            "error": result.get("error") if semantic_ok else "malformed_tip_status_reply",
            "tip_loaded": tip_loaded,
            "semantic_ok": semantic_ok,
            "hardware_truth_level": "hardware_query" if reply_received and tip_loaded is not None else ("unparsed_hardware_reply" if reply_received else "no_readback"),
            "oem_source_anchor": "ClassPipette.QueryTipStatus: ?31",
        }

    def query_pressure(self):
        result = self._send_pipette_command("?57", address="report", ack_mode="query", command_name="query_pressure")
        pressure = self._parse_numeric_ascii_from_ack(result)
        ack = result.get("ack", {}) if isinstance(result, dict) else {}
        reply_received = bool(ack.get("received")) if isinstance(ack, dict) else False
        semantic_ok = pressure is not None
        if reply_received and not semantic_ok and isinstance(result.get("provenance"), dict):
            result["provenance"]["outcome"] = "malformed"
        return {
            **result,
            "ok": bool(reply_received and semantic_ok),
            "reply_received": reply_received,
            "error": result.get("error") if semantic_ok else "malformed_pressure_reply",
            "pressure": pressure,
            "semantic_ok": semantic_ok,
            "hardware_truth_level": "hardware_query" if reply_received and pressure is not None else ("unparsed_hardware_reply" if reply_received else "no_readback"),
            "oem_source_anchor": "ClassPipette.QueryPressure: ?57",
        }

    def pipette_load_tip(self):
        # OEM code exposes QueryTipStatus and operator/UI tip selection, not a
        # load-tip actuator command. A load request is therefore a hardware
        # verification step, not a TX-only state mutation.
        result = self.query_tip_status()
        tip_loaded = result.get("tip_loaded") is True
        return {
            **result,
            "ok": bool(result.get("ok") and tip_loaded),
            "command_name": "verify_manual_tip_load",
            "error": None if tip_loaded else "tip_not_detected",
        }

    def pipette_eject_tip(self, *, initialized: bool = True, wait_for_completion: bool = True):
        # OEM ClassPipette.ejectTip uses E1R after initialization and E0R otherwise.
        return self._send_pipette_command(
            "E1R" if initialized else "E0R",
            command_name="pipette_eject_tip",
            wait_for_completion=wait_for_completion,
        )

    def terminate_pipette(self):
        return self._send_pipette_command("TR", address="control", command_name="terminate_pipette")

    def set_top_speed(self, velocity: float):
        value = float(velocity)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError("pipette top speed must be a finite positive number")
        encoded = format(value, "g")
        result = self._send_pipette_command(f"V{encoded},1R", command_name="set_top_speed")
        result["effective_top_speed"] = value
        return result

    def dispense_all(self, *, wait_for_completion: bool = True):
        return self._send_pipette_command(
            "A0R",
            command_name="dispense_all",
            wait_for_completion=wait_for_completion,
        )

    def aspirate_air(
        self,
        volume_ul: float,
        *,
        air_type: int = 1,
        tip_pressure_profile: str = "1R",
        wait_for_completion: bool = True,
    ):
        if int(air_type) != 1:
            raise ValueError("OEM AspirateAir uses pressure/type literal 1R")
        self._require_oem_pressure_profile(tip_pressure_profile)
        formatted_vol = self._format_pipette_volume(volume_ul)
        return self._send_pipette_command(
            f"P{formatted_vol},1R",
            command_name="aspirate_air",
            wait_for_completion=wait_for_completion,
        )

    def dispense_air(self, volume_ul: float, *, dispense_type: int = 0, wait_for_completion: bool = True):
        formatted_vol = self._format_pipette_volume(volume_ul)
        return self._send_pipette_command(
            f"D{formatted_vol},{int(dispense_type)}R",
            command_name="dispense_air",
            wait_for_completion=wait_for_completion,
        )

    def heartbeat(self, enabled: bool):
        return self._send_pipette_command("U60R" if enabled else "U61R", command_name="heartbeat")

    def execute_diagnoses(self, number: int, *, wait_for_completion: bool = True):
        result = self._send_pipette_command(
            f"d{int(number)}R",
            command_name="execute_diagnoses",
            wait_for_completion=wait_for_completion,
        )
        self._sleep(1.500)
        ack = result.get("ack", {}) if isinstance(result, dict) else {}
        data = list(ack.get("data", [])) if isinstance(ack, dict) else []
        result["diagnosis"] = bytes(data[2:]).decode("ascii", errors="replace") if len(data) >= 2 else None
        result["diagnostic_wait_ms"] = 1_500
        return result

    def query_error_log(self, raw_byte: int = 0):
        value = int(raw_byte)
        if value < 0 or value > 255:
            raise ValueError("raw_byte must be between 0 and 255")
        ids = self.pipette_can_ids()
        payload = [ord("Q"), ord(":"), value, ord("1")]
        result = self._send_packet(
            ids["report"],
            payload,
            require_ack=True,
            response_timeout_s=self.response_timeout_s,
            ack_mode="query",
            command_name="query_error_log",
        )
        ack = result.get("ack", {}) if isinstance(result, dict) else {}
        data = list(ack.get("data", [])) if isinstance(ack, dict) else []
        reply_received = bool(ack.get("received")) if isinstance(ack, dict) else False
        return {
            **result,
            "raw_query": payload,
            "oem_error_code": int(data[0]) if data else 0,
            "oem_null_reply_default": 0,
            "reply_received": reply_received,
            "semantic_ok": bool(reply_received and data),
            "query_error_queue": list(self._pipette_message_state.get("error_queue", [])),
        }

    def get_data(self, query: str, *, wake_if_needed: bool = True):
        selected = str(query).strip()
        if selected not in PIPETTE_DATA_QUERY_LABELS:
            raise ValueError(
                "diagnostic data query must be one of "
                + ", ".join(sorted(PIPETTE_DATA_QUERY_LABELS))
            )
        wake = None
        if wake_if_needed and self._pipette_message_state.get("initialized") is not True:
            wake_byte = 0x20 | int(self.pipette_id)
            wake = self._send_packet(0x080, [wake_byte, wake_byte], command_name="pipette_wake_address")
        result = self._send_pipette_command(selected, address="report", ack_mode="query", command_name="get_data")
        ack = result.get("ack", {}) if isinstance(result, dict) else {}
        data = list(ack.get("data", [])) if isinstance(ack, dict) else []
        label, unit = PIPETTE_DATA_QUERY_LABELS[selected]
        result["query"] = selected
        result["label"] = label
        result["unit"] = unit
        result["wake"] = wake
        prefix_ok = len(data) >= 3 and data[:2] == [0x20, 0x60]
        value_bytes = data[2:] if prefix_ok else []
        ascii_ok = bool(value_bytes) and all(0x20 <= int(value) <= 0x7E for value in value_bytes)
        value_ascii = bytes(value_bytes).decode("ascii") if ascii_ok else None
        value = value_ascii.strip() if value_ascii is not None else None
        semantic_ok = bool(
            result.get("semantic_query_response_verified")
            and prefix_ok
            and ascii_ok
            and value
        )
        result["ok"] = semantic_ok
        result["value_bytes"] = value_bytes
        result["value_ascii"] = value_ascii
        result["value"] = value
        result["semantic_ok"] = semantic_ok
        return result

    def start_fluid_detection(self, *, wait_for_completion: bool = True):
        return self._send_pipette_command(
            "BR",
            command_name="start_fluid_detection",
            wait_for_completion=wait_for_completion,
        )

    def aspirate(self, volume_ul: float, tip_pressure_profile='1R', *, wait_for_completion: bool = True):
        """
        Reverse Engineered from `ClassPipette.Aspirate(double volume)`.
        The DLL uses ASCII commands over the pipette CAN command ID.
        """
        self._require_oem_pressure_profile(tip_pressure_profile)
        formatted_vol = self._format_pipette_volume(volume_ul)
        ascii_command = f"P{formatted_vol},1R"
        return self._send_pipette_command(
            ascii_command,
            command_name="aspirate",
            wait_for_completion=wait_for_completion,
        )

    def dispense(
        self,
        volume_ul: float,
        tip_pressure_profile='1R',
        blow_out=False,
        dispense_type: int = 0,
        *,
        wait_for_completion: bool = True,
    ):
        self._require_oem_pressure_profile(tip_pressure_profile)
        selected_type = int(dispense_type)
        if selected_type not in {0, 1, 2}:
            raise ValueError("OEM dispense type must be 0, 1, or 2")
        formatted_vol = self._format_pipette_volume(volume_ul)
        ascii_command = f"D{formatted_vol},{selected_type}R"
        result = self._send_pipette_command(
            ascii_command,
            command_name="dispense",
            wait_for_completion=wait_for_completion,
        )
        result["dispense_type"] = selected_type
        if blow_out:
            result["blow_out"] = True
        return result

    def mix(self, volume_ul: float, count: int, tip_pressure_profile: str = "1R", *, wait_for_completion: bool = True):
        """Execute OEM ``Mix`` as its composite P/D command sequence.

        The serial-206 OEM has no dedicated Mix wire command.  For ``count - 1``
        cycles it sends Aspirate, waits 1500 ms, sends Dispense, waits 1500 ms;
        the final cycle sends only Aspirate and waits 1500 ms.
        """
        self._require_oem_pressure_profile(tip_pressure_profile)
        cycles = int(count)
        if cycles < 1:
            raise ValueError("mix count must be at least one")
        rows: list[dict[str, Any]] = []
        for cycle in range(1, cycles):
            aspirate = self.aspirate(
                volume_ul,
                tip_pressure_profile=tip_pressure_profile,
                wait_for_completion=wait_for_completion,
            )
            self._sleep(1.500)
            dispense = self.dispense(
                volume_ul,
                tip_pressure_profile=tip_pressure_profile,
                blow_out=False,
                wait_for_completion=wait_for_completion,
            )
            self._sleep(1.500)
            rows.append({"cycle": cycle, "aspirate": aspirate, "dispense": dispense})
        final_aspirate = self.aspirate(
            volume_ul,
            tip_pressure_profile=tip_pressure_profile,
            wait_for_completion=wait_for_completion,
        )
        self._sleep(1.500)
        rows.append({"cycle": cycles, "aspirate": final_aspirate, "dispense": None})
        return {
            "ok": all(
                isinstance(row.get("aspirate"), dict)
                and row["aspirate"].get("ok")
                and (row.get("dispense") is None or row["dispense"].get("ok"))
                for row in rows
            ),
            "command_name": "mix",
            "volume_ul": float(volume_ul),
            "count": cycles,
            "cycles": rows,
            "oem_wire_semantics": "composite_P_D_no_dedicated_mix_command",
            "inter_command_wait_ms": 1_500,
        }

    # ==========================================
    # GANTRY MOTORS (ClassMotor)
    # ==========================================
    def move_axis(self, axis: MotorAxis, target_position_steps: int):
        """
        Reverse Engineered from `ClassMotor.moveToAbs(int position)`.
        Action ID: 4 (Move to Absolute Position)
        """
        # Pack 32-bit position (Little Endian)
        packed_pos = struct.pack('<i', target_position_steps)

        # Packet Structure (From decompilation):
        # [Action, SubAction, Axis, byte3, byte2, byte1, byte0]
        packet = [
            4,          # Action ID 4 = Move Absolute
            0,          # Sub-action
            axis.value, # X, Y, or Z
            packed_pos[3],
            packed_pos[2],
            packed_pos[1],
            packed_pos[0]
        ]

        return self._send_packet(BoardAssy.MOTOR_CONTROLLER, packet)


if __name__ == "__main__":
    print("BIOXP 3200 PYTHON-CAN DRIVER INITIALIZED")
    print("----------------------------------------")

    # NOTE: The physical unit must be attached to test actual CAN broadcasting.
    # The following commands will just demonstrate the packing algorithm natively.

    import warnings
    warnings.filterwarnings('ignore')  # Ignore 'can0' interface missing for offline demo

    try:
        # We wrap in a try-except to allow testing without physical hardware
        robot = BioXpCanDriver(channel='vcan0')  # Use virtual CAN for safety

        robot.set_thermal_temperature(95.0)  # Sets TC block to 95C
        robot.move_axis(MotorAxis.Z, 80000)  # Move Z axis down 80000 steps
        robot.aspirate(0.5)                  # Aspirate 0.5uL

    except OSError:
        print("Hardware SocketCAN interface not found. Cannot broadcast packets.")
