import re
import struct
import time
from enum import IntEnum
from typing import Any

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


class BioXpCanDriver:
    """
    Python CAN Driver for the BioXP 3200.
    Bypasses the proprietary Windows .NET DLLs and sends raw byte
    payloads directly over the SocketCAN Linux interface.
    """

    def __init__(self, channel='can0', bitrate=1000000, *, pipette_id: int = 0, response_timeout_s: float = 1.0):
        # The BioXP USB-to-CAN adapter should map to can0 in Linux
        self.bus = can.interface.Bus(bustype='socketcan', channel=channel, bitrate=bitrate)
        self.channel = channel
        self.bitrate = int(bitrate)
        self.pipette_id = int(pipette_id)
        self.response_timeout_s = float(response_timeout_s)

    def pipette_can_ids(self) -> dict[str, int]:
        pipette_id = int(self.pipette_id)
        return {
            "control": int(0x100 | (pipette_id << 3)),
            "command": int(0x101 | (pipette_id << 3)),
            "first_part_command": int(0x103 | (pipette_id << 3)),
            "middle_part_command": int(0x104 | (pipette_id << 3)),
            "report": int(0x106 | (pipette_id << 3)),
        }

    @staticmethod
    def _frame_payload(command: list[int] | bytes | bytearray) -> list[int]:
        payload = [int(byte) & 0xFF for byte in command]
        return (payload + [0] * (8 - len(payload)))[:8]

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
            return False
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
            "board_id": int(board_id),
            "payload": payload,
            "command_name": command_name,
            "ack_required": bool(require_ack),
        }
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
            }
        ack = self._receive_reply(
            timeout_s=float(response_timeout_s if response_timeout_s is not None else self.response_timeout_s),
            ack_mode=ack_mode,
            expected_arbitration_id=int(board_id),
        )
        return {
            **base,
            "ok": bool(ack.get("ok")),
            "ack": ack,
            "error": None if ack.get("ok") else ack.get("error", "ack_failed"),
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
    ) -> dict[str, Any]:
        encoded = str(ascii_command).encode('ascii')
        if len(encoded) > 8:
            raise ValueError(f"ASCII pipette command exceeds 8-byte frame budget: {ascii_command!r}")
        return {
            **self._send_packet(
                int(board_id),
                list(encoded),
                require_ack=require_ack,
                response_timeout_s=response_timeout_s,
                ack_mode=ack_mode,
                command_name=command_name or str(ascii_command),
            ),
            "ascii_command": str(ascii_command),
            "length": len(encoded),
        }

    def _send_pipette_command(self, ascii_command: str, *, address: str = "command", ack_mode: str = "command", command_name: str | None = None) -> dict[str, Any]:
        ids = self.pipette_can_ids()
        if address not in ids:
            raise ValueError(f"Unknown pipette CAN address kind: {address!r}")
        return self._send_ascii_packet(
            ids[address],
            ascii_command,
            require_ack=True,
            response_timeout_s=self.response_timeout_s,
            ack_mode=ack_mode,
            command_name=command_name or f"pipette:{ascii_command}",
        )

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
        if volume_ul >= 100.0:
            return f"{round(volume_ul):.0f}"
        if volume_ul == float(int(volume_ul)):
            return f"{int(volume_ul)}"
        return f"{volume_ul:.1f}"

    @staticmethod
    def _parse_tip_loaded(result: dict[str, Any]) -> bool | None:
        ack = result.get("ack", {}) if isinstance(result, dict) else {}
        data = ack.get("data", []) if isinstance(ack, dict) else []
        # OEM report frames seen during shadow/live prep can carry the ASCII
        # truth byte in either the trimmed ClassCanLib position (index 2) or in
        # the full/raw frame position (index 4). Treat an explicit ASCII 1 in
        # either known slot as tip-present; only call it false when a known slot
        # is present and neither reports 1.
        known_slots = [idx for idx in (2, 4) if len(data) > idx]
        if any(data[idx] == ord('1') for idx in known_slots):
            return True
        if any(data[idx] == ord('0') for idx in known_slots):
            return False
        return None

    @staticmethod
    def _parse_numeric_ascii_from_ack(result: dict[str, Any]) -> float | None:
        ack = result.get("ack", {}) if isinstance(result, dict) else {}
        ascii_text = ack.get("ascii", "") if isinstance(ack, dict) else ""
        match = re.search(r"(?:^|\b)P\s*=\s*([-+]?\d+(?:\.\d+)?)", str(ascii_text))
        if match is None:
            return None
        return float(match.group(1))

    def pipette_initialize(self, pressure_profile='1R'):
        # OEM ClassPipette.initiate sends WR on m_CANCommandid after any wake frame.
        result = self._send_pipette_command("WR", command_name="pipette_initialize")
        result["requested_pressure_profile"] = str(pressure_profile).upper()
        return result

    def query_tip_status(self):
        result = self._send_pipette_command("?31", address="report", ack_mode="query", command_name="query_tip_status")
        tip_loaded = self._parse_tip_loaded(result)
        return {
            **result,
            "tip_loaded": tip_loaded,
            "semantic_ok": tip_loaded is not None,
            "hardware_truth_level": "hardware_query" if result.get("ok") and tip_loaded is not None else ("unparsed_hardware_reply" if result.get("ok") else "no_readback"),
            "oem_source_anchor": "ClassPipette.QueryTipStatus: ?31",
        }

    def query_pressure(self):
        result = self._send_pipette_command("?57", address="report", ack_mode="query", command_name="query_pressure")
        pressure = self._parse_numeric_ascii_from_ack(result)
        return {
            **result,
            "pressure": pressure,
            "semantic_ok": pressure is not None,
            "hardware_truth_level": "hardware_query" if result.get("ok") and pressure is not None else ("unparsed_hardware_reply" if result.get("ok") else "no_readback"),
            "oem_source_anchor": "ClassPipette.QueryPressure: ?57",
        }

    def pipette_load_tip(self):
        # OEM code exposes QueryTipStatus and operator/UI tip selection, not a
        # load-tip actuator command. A load request is therefore a hardware
        # verification step, not a fake TX-only state mutation.
        result = self.query_tip_status()
        tip_loaded = result.get("tip_loaded") is True
        return {
            **result,
            "ok": bool(result.get("ok") and tip_loaded),
            "command_name": "verify_manual_tip_load",
            "error": None if tip_loaded else "tip_not_detected",
        }

    def pipette_eject_tip(self):
        # OEM ClassPipette.ejectTip sends E1R after initialization.
        return self._send_pipette_command("E1R", command_name="pipette_eject_tip")

    def aspirate(self, volume_ul: float, tip_pressure_profile='1R'):
        """
        Reverse Engineered from `ClassPipette.Aspirate(double volume)`.
        The DLL uses ASCII commands over the pipette CAN command ID.
        """
        formatted_vol = self._format_pipette_volume(volume_ul)
        ascii_command = f"P{formatted_vol},{str(tip_pressure_profile).upper()}"
        return self._send_pipette_command(ascii_command, command_name="aspirate")

    def dispense(self, volume_ul: float, tip_pressure_profile='1R', blow_out=False):
        formatted_vol = self._format_pipette_volume(volume_ul)
        ascii_command = f"D{formatted_vol},{str(tip_pressure_profile).upper()}"
        result = self._send_pipette_command(ascii_command, command_name="dispense")
        if blow_out:
            result["blow_out"] = True
        return result

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
