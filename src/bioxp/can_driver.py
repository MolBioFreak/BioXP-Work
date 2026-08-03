import struct
import time
import math
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

    PIPETTE_TRANSACTION_TIMEOUT_S = 60.0
    PIPETTE_CHANNELS = frozenset(range(4))

    @classmethod
    def _validate_pipette_id(cls, pipette_id: int) -> int:
        selected = int(pipette_id)
        if selected not in cls.PIPETTE_CHANNELS:
            raise ValueError(f"pipette_id must be one of 0..3, got {selected}")
        return selected

    def __init__(self, channel='can0', bitrate=1000000, *, pipette_id: int = 0, response_timeout_s: float = 60.0):
        # The BioXP USB-to-CAN adapter should map to can0 in Linux
        self.bus = can.interface.Bus(bustype='socketcan', channel=channel, bitrate=bitrate)
        self.channel = channel
        self.bitrate = int(bitrate)
        self.pipette_id = self._validate_pipette_id(pipette_id)
        self.response_timeout_s = float(response_timeout_s)
        self._sleep = time.sleep

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
                )
            except Exception as exc:
                return {**base, "tx_ok": False, "error": str(exc), "provenance": None}
            frames = list(provenance.get("frames", []))
            data: list[int] = []
            for frame in frames:
                data.extend(int(byte) for byte in frame.get("data", []))
            observed = frames[-1] if frames else {}
            ack_ok = bool(provenance.get("ok")) and self._ack_ok(data, ack_mode=ack_mode)
            ack = {
                "ok": ack_ok,
                "received": bool(frames),
                "arbitration_id": observed.get("arbitration_id"),
                "dlc": observed.get("dlc"),
                "data": data,
                "raw": observed.get("raw"),
                "ascii": ''.join(chr(byte) if 32 <= byte <= 126 else '.' for byte in data),
                "outcome": provenance.get("outcome"),
                "immediate_ack": next((frame for frame in frames if frame.get("dlc") == 0), None),
                "completion": next((frame for frame in reversed(frames) if frame.get("dlc", 0) > 0), None),
            }
            if not ack_ok:
                ack["error"] = "ack_timeout" if provenance.get("outcome") == "timeout" else "pipette_reply_error"
            return {
                **base,
                "ok": ack_ok,
                "tx_ok": True,
                "ack": ack,
                "provenance": provenance,
                "error": None if ack_ok else ack["error"],
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
            timeout_s=timeout_s,
            ack_mode=ack_mode,
            expected_arbitration_id=int(board_id) | 0x400,
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
            )
            frames = list(provenance.get("frames", []))
            data = [int(byte) for frame in frames for byte in frame.get("data", [])]
            observed = frames[-1] if frames else {}
            ack_ok = bool(provenance.get("ok")) and self._ack_ok(data, ack_mode=ack_mode)
            ack = {
                "ok": ack_ok,
                "received": bool(frames),
                "arbitration_id": observed.get("arbitration_id"),
                "dlc": observed.get("dlc"),
                "data": data,
                "raw": observed.get("raw"),
                "ascii": ''.join(chr(byte) if 32 <= byte <= 126 else '.' for byte in data),
                "outcome": provenance.get("outcome"),
                "immediate_ack": next((frame for frame in frames if frame.get("dlc") == 0), None),
                "completion": next((frame for frame in reversed(frames) if frame.get("dlc", 0) > 0), None),
            }
            if not ack_ok:
                ack["error"] = "ack_timeout" if provenance.get("outcome") == "timeout" else "pipette_reply_error"
            return {
                "ok": ack_ok,
                "tx_ok": True,
                "ack": ack,
                "provenance": provenance,
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
        wake_byte = 0x20 | int(self.pipette_id)
        wake = self._send_packet(0x080, [wake_byte, wake_byte], command_name="pipette_wake_address")
        self._sleep(0.100)
        result = self._send_pipette_command("WR", command_name="pipette_initialize")
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

    def wait_pipette_initialization_completion(self, timeout_s: float):
        wait = getattr(self.bus, "wait_pipette_completion", None)
        if not callable(wait):
            return {"ok": False, "channel": int(self.pipette_id), "outcome": "completion_wait_unavailable"}
        return wait(int(self.pipette_id), float(timeout_s))

    def pipette_initiate_group(self):
        result = self._send_pipette_command("WR", command_name="pipette_initiate_group")
        result["immediate_ack_received"] = bool(
            result.get("ok") and result.get("ack", {}).get("outcome") == "ack"
        )
        result["initialized_after_valid_completion"] = False
        result["ok"] = bool(result["immediate_ack_received"])
        result["group_completion_timeout_ms"] = 10_000
        return result

    def query_firmware(self, number: int = 1):
        return self._send_pipette_command(f"&{int(number)}", address="report", ack_mode="query", command_name="query_firmware")

    def query_status(self):
        result = self._send_pipette_command("Q1", address="report", ack_mode="query", command_name="query_status")
        ack = result.get("ack", {}) if isinstance(result, dict) else {}
        data = list(ack.get("data", [])) if isinstance(ack, dict) else []
        reply_received = bool(result.get("ok"))
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
        reply_received = bool(result.get("ok"))
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
        reply_received = bool(result.get("ok"))
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

    def pipette_eject_tip(self, *, initialized: bool = True):
        # OEM ClassPipette.ejectTip uses E1R after initialization and E0R otherwise.
        return self._send_pipette_command("E1R" if initialized else "E0R", command_name="pipette_eject_tip")

    def terminate_pipette(self):
        return self._send_pipette_command("TR", address="control", command_name="terminate_pipette")

    def set_top_speed(self, velocity: int):
        value = int(velocity)
        result = self._send_pipette_command(f"V{value},1R", command_name="set_top_speed")
        result["effective_top_speed"] = value
        return result

    def dispense_all(self):
        return self._send_pipette_command("A0R", command_name="dispense_all")

    def aspirate_air(self, volume_ul: float, *, air_type: int = 1, tip_pressure_profile: str = "1R"):
        del air_type
        formatted_vol = self._format_pipette_volume(volume_ul)
        return self._send_pipette_command(
            f"P{formatted_vol},{str(tip_pressure_profile).upper()}",
            command_name="aspirate_air",
        )

    def dispense_air(self, volume_ul: float, *, dispense_type: int = 0):
        formatted_vol = self._format_pipette_volume(volume_ul)
        return self._send_pipette_command(
            f"D{formatted_vol},{int(dispense_type)}R",
            command_name="dispense_air",
        )

    def heartbeat(self, enabled: bool):
        return self._send_pipette_command("U60R" if enabled else "U61R", command_name="heartbeat")

    def execute_diagnoses(self, number: int):
        result = self._send_pipette_command(f"d{int(number)}R", command_name="execute_diagnoses")
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
        return {
            **self._send_packet(
                ids["report"],
                payload,
                require_ack=True,
                response_timeout_s=self.response_timeout_s,
                ack_mode="query",
                command_name="query_error_log",
            ),
            "raw_query": payload,
        }

    def get_data(self, query: str):
        selected = str(query)
        if not selected.startswith("?") or len(selected) != 3 or not selected[1:].isdigit():
            raise ValueError("diagnostic data query must be an OEM ?<two-digit> literal")
        result = self._send_pipette_command(selected, address="report", ack_mode="query", command_name="get_data")
        ack = result.get("ack", {}) if isinstance(result, dict) else {}
        data = list(ack.get("data", [])) if isinstance(ack, dict) else []
        result["query"] = selected
        result["value_bytes"] = data[2:] if len(data) >= 2 else []
        result["value_ascii"] = bytes(data[2:]).decode("ascii", errors="replace") if len(data) >= 2 else None
        return result

    def start_fluid_detection(self):
        return self._send_pipette_command("BR", command_name="start_fluid_detection")

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
