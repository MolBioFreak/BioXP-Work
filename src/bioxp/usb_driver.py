import struct
import sys
import time
import os
import fcntl
import ctypes
import subprocess
import shutil
import re
import signal
import json

import usb.core
import usb.util

TMCL_STATUS = {
    100: "Success",
    1: "Wrong checksum",
    2: "Invalid command",
    3: "Wrong type",
    4: "Invalid value",
    5: "EEPROM locked",
    6: "Command not available",
    7: "Busy",
    128: "Target position reached",
    129: "Not initialized",
    130: "Stall guard detected",
    132: "Door/Latch sensor changed",
}


class UvcXuControlQuery(ctypes.Structure):
    _fields_ = [
        ("unit", ctypes.c_uint8),
        ("selector", ctypes.c_uint8),
        ("query", ctypes.c_uint8),
        ("size", ctypes.c_uint16),
        ("data", ctypes.c_void_p),
    ]


class BioXpTester:
    BOARD_HEAD = 0x04
    BOARD_DECK = 0x05
    BOARD_THERMAL = 0x06
    BOARD_CHILLER = 0x07
    BOARDS = [BOARD_HEAD, BOARD_DECK, BOARD_THERMAL, BOARD_CHILLER]
    MOTOR_BOARDS = [BOARD_HEAD, BOARD_DECK, BOARD_THERMAL]
    MOTOR_AXIS_PRESETS = {
        # OEM mapping from ClassControlInterface m_AxisIODesignater:
        #   MotorX -> board 0x05 axis 0
        #   MotorY -> board 0x04 axis 0
        #   MotorZ -> board 0x04 axis 1
        #   MotorGrip -> board 0x04 axis 2
        # OEM initializeMotorsWithoutMotion writes speed/acc/current/stall and switch masks,
        # but does not apply ramp mode or RDIV/PDIV on XYZ.
        "x": {
            "label": "X",
            "board": BOARD_DECK,
            "motor": 0,
            "speed": 1700,
            "acc": 350,
            "run_current": 31,
            "standby_current": 10,
            "stall_guard": 16,
            "disable_right": False,
            "disable_left": False,
            "warm_enable": True,
        },
        "y": {
            "label": "Y",
            "board": BOARD_HEAD,
            "motor": 0,
            "speed": 1800,
            "acc": 400,
            "run_current": 31,
            "standby_current": 10,
            "stall_guard": 16,
            "disable_right": True,
            "disable_left": False,
            "warm_enable": True,
        },
        "z": {
            "label": "Z",
            "board": BOARD_HEAD,
            "motor": 1,
            "speed": 1791,
            "acc": 576,
            "run_current": 31,
            "standby_current": 10,
            "stall_guard": 16,
            "disable_right": False,
            "disable_left": False,
            "warm_enable": True,
        },
        "g": {
            "label": "GRIPPER",
            "board": BOARD_HEAD,
            "motor": 2,
            "speed": 600,
            "acc": 20,
            "run_current": 20,
            "standby_current": 10,
            "stall_guard": 5,
            "rdiv": 6,
            "pdiv": 2,
        },
    }
    # OEM-observed operating envelopes for speed/acc on primary gantry axes.
    # Defaults come from initializeMotorsWithoutMotion/setMaxSpeed/setMaxAcc.
    # Upper bounds include values explicitly used in OEM motion tests.
    MOTOR_SPEED_ACC_NORMS = {
        "x": {"speed_min": 50, "speed_max": 1700, "acc_min": 20, "acc_max": 400, "speed_oem": 1700, "acc_oem": 350},
        "y": {"speed_min": 50, "speed_max": 1800, "acc_min": 20, "acc_max": 750, "speed_oem": 1800, "acc_oem": 400},
        "z": {"speed_min": 50, "speed_max": 1791, "acc_min": 20, "acc_max": 576, "speed_oem": 1791, "acc_oem": 576},
    }
    MOTOR_FUNCTION_PRESETS = {
        "x": dict(MOTOR_AXIS_PRESETS["x"]),
        "y": dict(MOTOR_AXIS_PRESETS["y"]),
        "z": dict(MOTOR_AXIS_PRESETS["z"]),
        "g": dict(MOTOR_AXIS_PRESETS["g"]),
        # OEM mapping from ClassControlInterface:
        #   ThermalDoor -> board index 2 -> board id 0x06, axis 0
        "door": {
            "label": "THERMAL_DOOR",
            "board": BOARD_THERMAL,
            "motor": 0,
            "speed": 600,
            "acc": 200,
            "run_current": 20,
            "standby_current": 10,
            "stall_guard": 6,
            "disable_right": True,
            "disable_left": True,
        },
    }
    MOTOR_DRIVER_DIAG_PARAMS = [
        3,   # actual speed
        4,   # max speed
        5,   # max acc
        6,   # run current
        7,   # standby current
        8,   # target reached
        9,   # left switch
        10,  # right switch
        12,  # right switch disable
        13,  # left switch disable
        138, # ramp mode
        153, # rdiv
        154, # pdiv
        162, # chopper blank time
        163, # chopper const toff
        164, # disable short to gnd
        165, # high side sense
        166, # chopper mode
        167, # hysteresis dec
        168, # hysteresis end
        169, # hysteresis start
        170, # chopper off time
        171, # smart energy min current
        172, # smart energy current down
        173, # smart energy hysteresis
        174, # smart energy current up
        175, # smart energy hysteresis start
        176, # smart energy hysteresis end
        177, # stallguard filter
        178, # stallguard threshold
        179, # slope control high side
        180, # slope control low side
        181, # short detect timer
        182, # short detect recovery
        204, # freewheeling
        205, # stall guard threshold (OEM)
    ]
    MOTOR_DRIVER_COPY_PARAMS = [
        138, 153, 154, 162, 163, 164, 165, 166, 167, 168, 169, 170,
        171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 204
    ]
    CHILLER_BANK_RC = 0
    CHILLER_BANK_OC = 1
    CHILLER_SET_AXIS = {CHILLER_BANK_RC: 0, CHILLER_BANK_OC: 1}
    # OEM Windows code reads these channels for feedback:
    # RC temp=0, RC pedestal=1, OC temp=3, OC pedestal=4.
    CHILLER_TEMP_AXES = (
        ("rc_temp_c", 0),
        ("rc_pedestal_c", 1),
        ("oc_temp_c", 3),
        ("oc_pedestal_c", 4),
    )
    CHILLER_GP_SCALE = {
        2: 1000.0,   # max heating current (A)
        3: 1000.0,   # max cooling current (A)
        7: 1000.0,   # heat ramp (C/s)
        8: 1000.0,   # cool ramp (C/s)
        9: 1000.0,   # proportional gain
        10: 1000.0,  # integral gain
        11: 1000.0,  # derivative gain
        12: 1000.0,  # feed-forward gain
        13: 1000.0,  # TEC current (A)
        14: 1000.0,  # current reference (A)
        22: 1000.0,  # temperature reference (C)
    }
    CHILLER_SAFE_WRITE_PARAMS = {7, 8, 21, 22, 23}
    CHILLER_MIN_CMD_GAP_S = 0.060
    CHILLER_WRITE_SETTLE_S = 0.120
    CHILLER_VERIFY_SETTLE_S = 0.180
    THERMAL_BANK_NEST = 0
    THERMAL_BANK_LID = 1
    THERMAL_GP_SCALE = {
        2: 1000.0,   # max heating current
        3: 1000.0,   # max cooling current
        7: 1000.0,   # heat ramp (C/s)
        8: 1000.0,   # cool ramp (C/s)
        9: 1000.0,   # proportional gain
        10: 1000.0,  # integral gain
        11: 1000.0,  # derivative gain
        12: 1000.0,  # feed-forward gain
        13: 1000.0,  # TEC current
        14: 1000.0,  # current reference
        22: 1000.0,  # temperature reference
    }
    THERMAL_SAFE_WRITE_PARAMS = {7, 8, 19, 20, 21, 22, 23}
    THERMAL_MIN_CMD_GAP_S = 0.060
    THERMAL_WRITE_SETTLE_S = 0.120
    THERMAL_VERIFY_SETTLE_S = 0.180
    MOTOR_MIN_CMD_GAP_S = 0.045
    MOTOR_POST_RECOVER_GAP_S = 0.100
    MOTOR_RECOVERY_BOARDS = (BOARD_HEAD, BOARD_DECK)
    # Fail-intolerant motion probe error catalog.
    MOTION_ERROR_CODES = {
        "AXIS_PROBE_ERROR": 9100,
        "ACK_FWD_FAIL": 9101,
        "ACK_BACK_FAIL": 9102,
        "WAIT_FWD_TIMEOUT": 9103,
        "WAIT_BACK_TIMEOUT": 9104,
        "ACK_SUCCESS_NO_DELTA_FWD": 9105,
        "ACK_SUCCESS_NO_DELTA_BACK": 9106,
        "ESCAPE_SWITCH_NOT_CLEARED": 9107,
        "REPORTED_MOVE_SWITCH_MISMATCH": 9108,
        "NO_24V_DURING_PROBE": 9109,
        "STRICT_INIT_FAILED": 9201,
        "STRICT_HOMING_FAILED": 9202,
        "MOTION_NOT_ARMED": 9203,
        "MOTION_SENSOR_GATE_FAILED": 9204,
        "MOTION_GATE_IO_UNAVAILABLE": 9205,
    }
    MOTION_ERROR_TEXT = {
        "AXIS_PROBE_ERROR": "Axis probe returned an internal error.",
        "ACK_FWD_FAIL": "Forward move did not return Success ack.",
        "ACK_BACK_FAIL": "Return move did not return Success ack.",
        "WAIT_FWD_TIMEOUT": "Forward move did not stop within timeout.",
        "WAIT_BACK_TIMEOUT": "Return move did not stop within timeout.",
        "ACK_SUCCESS_NO_DELTA_FWD": "Forward move acked Success but reported zero position delta.",
        "ACK_SUCCESS_NO_DELTA_BACK": "Return move acked Success but reported zero position delta.",
        "ESCAPE_SWITCH_NOT_CLEARED": "Move away from active limit did not clear that limit.",
        "REPORTED_MOVE_SWITCH_MISMATCH": "Driver reported position delta but active limit did not clear.",
        "NO_24V_DURING_PROBE": "24V sensor reported NO_24V during motion probe.",
        "STRICT_INIT_FAILED": "Strict startup init failed one or more gate checks.",
        "STRICT_HOMING_FAILED": "Strict startup init failed during full homing.",
        "MOTION_NOT_ARMED": "XYZ motion blocked because strict startup arm is not active.",
        "MOTION_SENSOR_GATE_FAILED": "XYZ motion blocked by live sensor gate mismatch.",
        "MOTION_GATE_IO_UNAVAILABLE": "XYZ motion blocked because deck IO gate signals were unreadable.",
    }
    VIDIOC_G_CTRL = 0xC008561B
    VIDIOC_S_CTRL = 0xC008561C
    V4L2_CID_BACKLIGHT_COMPENSATION = 0x0098091C
    V4L2_CID_BRIGHTNESS = 0x00980900
    V4L2_CID_CONTRAST = 0x00980901
    V4L2_CID_SATURATION = 0x00980902
    V4L2_CID_GAIN = 0x00980913
    V4L2_CID_EXPOSURE_AUTO = 0x009A0901
    V4L2_CID_EXPOSURE_ABSOLUTE = 0x009A0902
    V4L2_CID_FOCUS_AUTO = 0x009A090C
    V4L2_CID_FOCUS_ABSOLUTE = 0x009A090A
    VIDIOC_QUERYCTRL = 0xC0445624
    V4L2_CTRL_FLAG_DISABLED = 0x00000002
    V4L2_CTRL_FLAG_NEXT_CTRL = 0x80000000

    V4L2_EXPOSURE_AUTO = 0
    V4L2_EXPOSURE_MANUAL = 1
    V4L2_EXPOSURE_SHUTTER_PRIORITY = 2
    V4L2_EXPOSURE_APERTURE_PRIORITY = 3
    UVC_SET_CUR = 0x01
    UVC_GET_CUR = 0x81
    UVC_GET_LEN = 0x85
    UVC_GET_INFO = 0x86

    IOC_NRBITS = 8
    IOC_TYPEBITS = 8
    IOC_SIZEBITS = 14
    IOC_DIRBITS = 2
    IOC_NRSHIFT = 0
    IOC_TYPESHIFT = IOC_NRSHIFT + IOC_NRBITS
    IOC_SIZESHIFT = IOC_TYPESHIFT + IOC_TYPEBITS
    IOC_DIRSHIFT = IOC_SIZESHIFT + IOC_SIZEBITS
    IOC_WRITE = 1
    IOC_READ = 2

    KNOWN_HEARTBEATS = {
        (0x7E, 0x00, 0x00, 0x04, 0x82, 0x00, 0x86, 0x7E),
        (0x7E, 0x00, 0x00, 0x04, 0x8A, 0x00, 0x8E, 0x7E),
        (0x7E, 0x00, 0x00, 0x04, 0x92, 0x00, 0x96, 0x7E),
        (0x7E, 0x00, 0x00, 0x04, 0x9A, 0x00, 0x9E, 0x7E),
        (0x7E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x04, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8D, 0x7E),
        (0x7E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x05, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8E, 0x7E),
        (0x7E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x06, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x7E),
        (0x7E, 0x00, 0x00, 0x00, 0x00, 0x08, 0x07, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x90, 0x7E),
    }

    def __init__(self, alt=1):
        self.dev = None
        self.ep_out = None
        self.ep_in = None
        self.alt = alt
        self._motor_last_tx_ts = {int(bid): 0.0 for bid in self.MOTOR_BOARDS}
        self._motor_noresp_streak = {int(bid): 0 for bid in self.MOTOR_BOARDS}
        self._chiller_last_tx_ts = 0.0
        self._chiller_noresp_streak = 0
        self._thermal_last_tx_ts = 0.0
        self._thermal_noresp_streak = 0
        self._led_state_path = self._default_led_state_path()
        self._led_state_cache = None
        self._libc = ctypes.CDLL(None, use_errno=True)
        self.UVCIOC_CTRL_QUERY = self._build_uvc_ioctl_query()
        self._motion_arm = {
            "armed": False,
            "reason": "startup",
            "note": "strict init not yet run",
            "updated_ms": int(time.time() * 1000),
            "arm_seq": 0,
        }
        self._motion_latch_override = {
            "enabled": False,
            "reason": "startup",
            "note": "auto-lock mode",
            "updated_ms": int(time.time() * 1000),
            "seq": 0,
        }
        self._motion_last_strict_init = None
        self._connect()
        self._led_state_cache = self.load_led_state()

    @classmethod
    def _ioc(cls, direction, ioc_type, nr, size):
        return (
            (int(direction) << cls.IOC_DIRSHIFT)
            | (int(ioc_type) << cls.IOC_TYPESHIFT)
            | (int(nr) << cls.IOC_NRSHIFT)
            | (int(size) << cls.IOC_SIZESHIFT)
        )

    @classmethod
    def _build_uvc_ioctl_query(cls):
        return cls._ioc(cls.IOC_READ | cls.IOC_WRITE, ord("u"), 0x21, ctypes.sizeof(UvcXuControlQuery))

    def _connect(self):
        self.dev = usb.core.find(idVendor=0x03EB, idProduct=0x2423)
        if self.dev is None:
            raise ValueError("BioXP USB device not found.")

        if self.dev.is_kernel_driver_active(0):
            try:
                self.dev.detach_kernel_driver(0)
            except usb.core.USBError:
                pass

        self.dev.set_configuration()
        usb.util.claim_interface(self.dev, 0)
        self.dev.set_interface_altsetting(interface=0, alternate_setting=self.alt)

        cfg = self.dev.get_active_configuration()
        intf = cfg[(0, self.alt)]
        self.ep_out = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT,
        )
        self.ep_in = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN,
        )

    def reconnect(self):
        try:
            usb.util.release_interface(self.dev, 0)
        except Exception:
            pass
        try:
            self.dev.reset()
        except Exception:
            pass
        time.sleep(0.12)
        self._connect()
        self._chiller_noresp_streak = 0
        self._chiller_last_tx_ts = 0.0
        self._thermal_noresp_streak = 0
        self._thermal_last_tx_ts = 0.0
        self._motor_last_tx_ts = {int(bid): 0.0 for bid in self.MOTOR_BOARDS}
        self._motor_noresp_streak = {int(bid): 0 for bid in self.MOTOR_BOARDS}

    @staticmethod
    def _default_led_state_path():
        root_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        return os.path.join(root_dir, "output", "led_state.json")

    @staticmethod
    def _normalize_led_rgb(rgb):
        if not isinstance(rgb, (list, tuple)) or len(rgb) != 3:
            return None
        out = []
        for item in rgb:
            try:
                v = int(item)
            except Exception:
                return None
            if v < 0 or v > 255:
                return None
            out.append(v)
        return tuple(out)

    def load_led_state(self):
        try:
            with open(self._led_state_path, "r", encoding="utf-8") as f:
                raw = json.load(f)
        except FileNotFoundError:
            return None
        except Exception:
            return None
        if not isinstance(raw, dict):
            return None
        rgb = self._normalize_led_rgb(raw.get("rgb"))
        if rgb is None:
            return None
        preset = raw.get("preset")
        if not (isinstance(preset, str) and preset.strip()):
            preset = None
        ts = raw.get("updated_epoch_s")
        if not isinstance(ts, (int, float)):
            ts = None
        return {
            "rgb": rgb,
            "preset": preset.strip() if isinstance(preset, str) else None,
            "updated_epoch_s": float(ts) if ts is not None else None,
        }

    def led_state_cached(self, refresh=False):
        if refresh:
            self._led_state_cache = self.load_led_state()
        if not isinstance(self._led_state_cache, dict):
            return None
        rgb = self._normalize_led_rgb(self._led_state_cache.get("rgb"))
        if rgb is None:
            return None
        return {
            "rgb": rgb,
            "preset": self._led_state_cache.get("preset"),
            "updated_epoch_s": self._led_state_cache.get("updated_epoch_s"),
        }

    def save_led_state(self, r, g, b, preset=None):
        rgb = (self._clamp_u8(r), self._clamp_u8(g), self._clamp_u8(b))
        payload = {
            "rgb": [rgb[0], rgb[1], rgb[2]],
            "preset": preset.strip() if isinstance(preset, str) and preset.strip() else None,
            "updated_epoch_s": round(time.time(), 3),
        }
        tmp_path = self._led_state_path + ".tmp"
        try:
            os.makedirs(os.path.dirname(self._led_state_path), exist_ok=True)
            with open(tmp_path, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2, sort_keys=True)
                f.write("\n")
            os.replace(tmp_path, self._led_state_path)
            self._led_state_cache = {
                "rgb": rgb,
                "preset": payload.get("preset"),
                "updated_epoch_s": payload.get("updated_epoch_s"),
            }
            return {
                "ok": True,
                "path": self._led_state_path,
                "state": self.led_state_cached(refresh=False),
            }
        except Exception as e:
            return {"ok": False, "error": str(e), "path": self._led_state_path}

    def apply_saved_led_state(self, reconnect_first=False):
        state = self.led_state_cached(refresh=True)
        if state is None:
            return {"ok": False, "error": "no persisted LED state"}
        rgb = state["rgb"]
        out = self.strip_set_rgb(rgb[0], rgb[1], rgb[2], reconnect_first=bool(reconnect_first))
        out["ok"] = True
        out["persisted"] = state
        return out

    def drain(self, max_reads=16, timeout_ms=8):
        for _ in range(max_reads):
            try:
                self.ep_in.read(64, timeout=timeout_ms)
            except usb.core.USBTimeoutError:
                break

    @staticmethod
    def _build_frame(board_id, command, cmd_type, motor, value):
        val = struct.pack(">i", int(value))
        inner = bytearray(
            [
                0x00,
                0x00,
                0x00,
                board_id,
                0x08,
                command,
                cmd_type,
                motor,
                val[0],
                val[1],
                val[2],
                val[3],
                0x00,
            ]
        )
        chk = sum(inner) & 0xFF
        return bytearray([0x7E]) + inner + bytearray([chk, 0x7E])

    def send_tmcl(
        self,
        board_id,
        command,
        cmd_type,
        motor,
        value,
        wait_reply=True,
        write_timeout_ms=80,
        read_timeout_ms=30,
        max_reads=12,
        strict_match=True,
    ):
        frame = self._build_frame(board_id, command, cmd_type, motor, value)
        # Keep stale async frames from poisoning strict response matching.
        self.drain(max_reads=6, timeout_ms=2)
        try:
            self.ep_out.write(frame, timeout=write_timeout_ms)
        except usb.core.USBTimeoutError:
            return None

        if not wait_reply:
            return {"status": 100, "status_str": "sent", "board": board_id, "cmd": command, "value": value}

        return self._wait_for_reply(
            board_id=board_id,
            command=command,
            strict_match=strict_match,
            read_timeout_ms=read_timeout_ms,
            max_reads=max_reads,
        )

    def _wait_for_reply(self, board_id, command, strict_match, read_timeout_ms, max_reads):
        # Deadline-based wait survives bursty async traffic better than fixed read-count loops.
        deadline = time.monotonic() + max(0.10, (read_timeout_ms * max_reads) / 1000.0)
        while time.monotonic() < deadline:
            try:
                resp = list(self.ep_in.read(64, timeout=read_timeout_ms))
            except usb.core.USBTimeoutError:
                continue
            if tuple(resp) in self.KNOWN_HEARTBEATS or len(resp) < 14:
                continue
            if strict_match and (resp[6] != board_id or resp[8] != command):
                continue
            status = resp[7]
            return {
                "status": status,
                "status_str": TMCL_STATUS.get(status, f"?({status})"),
                "board": resp[6],
                "cmd": resp[8],
                "value": struct.unpack(">i", bytes(resp[9:13]))[0],
                "raw": resp,
            }
        return None

    def collect_bus_events(self, duration_s=0.50, timeout_ms=20, max_events=96):
        """
        Capture asynchronous TMCL frames for diagnostics.
        """
        events = []
        deadline = time.monotonic() + max(0.05, float(duration_s))
        while time.monotonic() < deadline and len(events) < int(max_events):
            try:
                resp = list(self.ep_in.read(64, timeout=int(timeout_ms)))
            except usb.core.USBTimeoutError:
                continue
            if tuple(resp) in self.KNOWN_HEARTBEATS or len(resp) < 14:
                continue
            status = resp[7]
            ev = {
                "board": resp[6],
                "status": status,
                "status_str": TMCL_STATUS.get(status, f"?({status})"),
                "cmd": resp[8],
                "value": struct.unpack(">i", bytes(resp[9:13]))[0],
                "raw": resp,
            }
            events.append(ev)
        return events

    def send_tmcl_retry(self, board_id, command, cmd_type, motor, value, attempts=3, **kwargs):
        for _ in range(max(1, attempts)):
            r = self.send_tmcl(board_id, command, cmd_type, motor, value, **kwargs)
            if r is not None:
                return r
        return None

    def activate_boards(self, expect_reply=True):
        out = {}
        for bid in self.BOARDS:
            out[bid] = self.send_tmcl_retry(
                bid,
                64,
                0,
                0,
                1,
                attempts=2 if expect_reply else 1,
                wait_reply=expect_reply,
                write_timeout_ms=35 if expect_reply else 20,
                read_timeout_ms=30,
                max_reads=12,
                strict_match=True,
            )
            time.sleep(0.005)
        return out

    def board_activate(self, board_id):
        ack = self.send_tmcl_retry(
            int(board_id),
            64,
            0,
            0,
            1,
            attempts=2,
            wait_reply=True,
            write_timeout_ms=45,
            read_timeout_ms=55,
            max_reads=16,
            strict_match=True,
        )
        return {"board": int(board_id), "ack": ack}

    def board_deactivate(self, board_id):
        ack = self.send_tmcl_retry(
            int(board_id),
            64,
            0,
            0,
            0,
            attempts=2,
            wait_reply=True,
            write_timeout_ms=45,
            read_timeout_ms=55,
            max_reads=16,
            strict_match=True,
        )
        return {"board": int(board_id), "ack": ack}

    def board_query_firmware(self, board_id):
        ack = self.send_tmcl_retry(
            int(board_id),
            173,
            0,
            0,
            0,
            attempts=2,
            wait_reply=True,
            write_timeout_ms=55,
            read_timeout_ms=70,
            max_reads=20,
            strict_match=True,
        )
        fw_hex = None
        if ack and isinstance(ack.get("raw"), list) and len(ack["raw"]) >= 13:
            fw_hex = "-".join(f"{b:02X}" for b in ack["raw"][9:13])
        return {"board": int(board_id), "ack": ack, "fw_hex": fw_hex}

    def io_snapshot(self, board_id=BOARD_DECK, retries_per_channel=3):
        # cmd15 types: 0=24V, 1=door sensor, 2=solenoid state, 3=latch sensor
        vals = {}
        bid = int(board_id)
        self.drain(max_reads=8, timeout_ms=6)
        for ch in (0, 1, 2, 3):
            ack = None
            for i in range(max(1, int(retries_per_channel))):
                ack = self._send_motor(
                    bid,
                    15,
                    int(ch),
                    0,
                    0,
                    attempts=2,
                    wait_reply=True,
                    write_timeout_ms=55,
                    read_timeout_ms=70,
                    max_reads=18,
                    strict_match=True,
                )
                if ack is not None:
                    break
                if i == 0 and bid in self.MOTOR_RECOVERY_BOARDS:
                    # If IO polling collides with recent traffic, refresh board wake once.
                    self.activate_boards(expect_reply=False)
                    time.sleep(0.04)
            vals[ch] = None if ack is None else ack.get("value")
            time.sleep(0.01)
        return vals

    def deck_io_query_type(self, io_type):
        """
        Deck IO query wrapper.
        OEM mapping in ClassIOControl:
          cmd15/type0=24V, type1=door sensor, type2=solenoid state, type3=latch sensor.
        """
        ack = self._send_motor(
            self.BOARD_DECK,
            15,
            int(io_type),
            0,
            0,
            attempts=2,
            wait_reply=True,
            write_timeout_ms=55,
            read_timeout_ms=70,
            max_reads=18,
            strict_match=True,
        )
        return {
            "board": int(self.BOARD_DECK),
            "cmd": 15,
            "type": int(io_type),
            "ack": ack,
            "value": None if ack is None else ack.get("value"),
            "ok": self._tmcl_success(ack),
        }

    def deck_io_set_type(self, io_type, value):
        """
        Deck IO set wrapper (SIO command).
        OEM known path: cmd14/type2/value{0|1} controls latch solenoid.
        """
        ack = self._send_motor(
            self.BOARD_DECK,
            14,
            int(io_type),
            0,
            int(value),
            attempts=2,
            wait_reply=True,
            write_timeout_ms=55,
            read_timeout_ms=70,
            max_reads=18,
            strict_match=True,
        )
        return {
            "board": int(self.BOARD_DECK),
            "cmd": 14,
            "type": int(io_type),
            "value_set": int(value),
            "ack": ack,
            "ok": self._tmcl_success(ack),
        }

    def deck_io_query_matrix(self, io_types=(0, 1, 2, 3, 4, 5, 6, 7)):
        rows = []
        for t in io_types:
            rows.append(self.deck_io_query_type(int(t)))
            time.sleep(0.01)
        return {"rows": rows}

    def deck_io_sio_probe(self, set_types=(2,), values=(0, 1), verify_types=(0, 1, 2, 3), settle_s=0.20):
        """
        Probe deck SIO/GIO behavior.
        Default is conservative: toggle only known solenoid set channel (type2),
        then verify with known query channels.
        """
        t0 = time.time()
        self.reconnect()
        board_status = self.activate_boards(expect_reply=True)
        baseline = self.deck_io_query_matrix(io_types=tuple(int(t) for t in verify_types))
        rail0 = self.motor_query_24v_sensor()
        rows = []
        for st in tuple(int(t) for t in set_types):
            for val in tuple(int(v) for v in values):
                wr = self.deck_io_set_type(st, val)
                time.sleep(max(0.05, float(settle_s)))
                ver = self.deck_io_query_matrix(io_types=tuple(int(t) for t in verify_types))
                rail = self.motor_query_24v_sensor()
                rows.append(
                    {
                        "set_type": int(st),
                        "set_value": int(val),
                        "write": wr,
                        "verify": ver,
                        "rail_24v": rail,
                    }
                )
        final = self.deck_io_query_matrix(io_types=tuple(int(t) for t in verify_types))
        rail1 = self.motor_query_24v_sensor()
        return {
            "board_status": board_status,
            "baseline": baseline,
            "baseline_24v": rail0,
            "rows": rows,
            "final": final,
            "final_24v": rail1,
            "elapsed_ms": int((time.time() - t0) * 1000),
        }

    @classmethod
    def _motion_error_entry(cls, key, axis=None, detail=None):
        k = str(key)
        row = {
            "key": k,
            "code": int(cls.MOTION_ERROR_CODES.get(k, 9999)),
            "message": cls.MOTION_ERROR_TEXT.get(k, "Unclassified motion probe failure."),
        }
        if axis is not None:
            row["axis"] = str(axis)
        if detail is not None:
            row["detail"] = str(detail)
        return row

    def _validate_axis_micro_motion_probe(self, probe, rail_24v=None):
        errors = []
        axis = None
        if isinstance(probe, dict):
            axis = probe.get("axis")

        if not isinstance(probe, dict):
            errors.append(self._motion_error_entry("AXIS_PROBE_ERROR", axis=axis, detail="probe payload missing"))
            return {"ok": False, "error_code": int(errors[0]["code"]), "error_count": len(errors), "errors": errors}

        if probe.get("error"):
            errors.append(self._motion_error_entry("AXIS_PROBE_ERROR", axis=axis, detail=probe.get("error")))

        ts = probe.get("test", {}) if isinstance(probe.get("test"), dict) else {}
        step_cmd = probe.get("step_cmd")
        fwd_ack = ts.get("fwd", {}).get("ack") if isinstance(ts.get("fwd"), dict) else None
        back_ack = ts.get("back", {}).get("ack") if isinstance(ts.get("back"), dict) else None
        d1 = ts.get("delta_fwd")
        d2 = ts.get("delta_back")
        sw_before = probe.get("switch_before", {}) if isinstance(probe.get("switch_before"), dict) else {}
        sw_mid = ts.get("mid_switches", {}) if isinstance(ts.get("mid_switches"), dict) else {}
        pre_left = sw_before.get("left_state")
        pre_right = sw_before.get("right_state")
        mid_left = sw_mid.get("left_state")
        mid_right = sw_mid.get("right_state")

        fwd_ok = self._tmcl_success(fwd_ack)
        back_ok = self._tmcl_success(back_ack)
        if not fwd_ok:
            errors.append(self._motion_error_entry("ACK_FWD_FAIL", axis=axis))
        if not back_ok:
            errors.append(self._motion_error_entry("ACK_BACK_FAIL", axis=axis))

        wait_fwd = ts.get("wait_fwd", {}) if isinstance(ts.get("wait_fwd"), dict) else {}
        wait_back = ts.get("wait_back", {}) if isinstance(ts.get("wait_back"), dict) else {}
        if wait_fwd.get("stopped") is not True:
            errors.append(self._motion_error_entry("WAIT_FWD_TIMEOUT", axis=axis))
        if wait_back.get("stopped") is not True:
            errors.append(self._motion_error_entry("WAIT_BACK_TIMEOUT", axis=axis))

        if fwd_ok and isinstance(d1, int) and int(d1) == 0:
            errors.append(self._motion_error_entry("ACK_SUCCESS_NO_DELTA_FWD", axis=axis))
        if back_ok and isinstance(d2, int) and int(d2) == 0:
            errors.append(self._motion_error_entry("ACK_SUCCESS_NO_DELTA_BACK", axis=axis))

        # Strong physical sanity check: when starting on a hard-limit and commanding
        # away from it, the forward leg should clear that limit.
        if isinstance(step_cmd, int):
            if pre_left == 0 and pre_right == 1 and int(step_cmd) > 0 and mid_left == 0:
                if isinstance(d1, int) and int(d1) != 0:
                    errors.append(self._motion_error_entry("REPORTED_MOVE_SWITCH_MISMATCH", axis=axis))
                else:
                    errors.append(self._motion_error_entry("ESCAPE_SWITCH_NOT_CLEARED", axis=axis))
            if pre_right == 0 and pre_left == 1 and int(step_cmd) < 0 and mid_right == 0:
                if isinstance(d1, int) and int(d1) != 0:
                    errors.append(self._motion_error_entry("REPORTED_MOVE_SWITCH_MISMATCH", axis=axis))
                else:
                    errors.append(self._motion_error_entry("ESCAPE_SWITCH_NOT_CLEARED", axis=axis))

        if isinstance(rail_24v, dict) and rail_24v.get("no24v") is True:
            errors.append(self._motion_error_entry("NO_24V_DURING_PROBE", axis=axis, detail=f"raw={rail_24v.get('raw')}"))

        return {
            "ok": len(errors) == 0,
            "error_code": None if len(errors) == 0 else int(errors[0]["code"]),
            "error_count": len(errors),
            "errors": errors,
        }

    def motor_validate_step_test(self, board_id, steps, test_out, motor=0, axis_label=None, rail_24v=None):
        axis = axis_label
        if axis is None:
            key = self.motor_axis_key_for_channel(board_id, motor=motor)
            p = self.motor_axis_preset(key) if key is not None else None
            if isinstance(p, dict):
                axis = str(p.get("label", key)).upper()
            elif key is not None:
                axis = str(key).upper()
            else:
                axis = f"0x{int(board_id):02X}:{int(motor)}"
        rail = rail_24v if isinstance(rail_24v, dict) else self.motor_query_24v_sensor()
        probe = {
            "axis": str(axis),
            "step_cmd": int(steps),
            "switch_before": test_out.get("pre_switches", {}) if isinstance(test_out, dict) else {},
            "switch_after": test_out.get("post_switches", {}) if isinstance(test_out, dict) else {},
            "test": test_out if isinstance(test_out, dict) else {},
        }
        val = self._validate_axis_micro_motion_probe(probe, rail_24v=rail)
        return {"axis": str(axis), "probe": probe, "validation": val, "rail_24v": rail}

    def motor_validate_oneway_move(
        self,
        *,
        axis,
        step_cmd,
        pre_switches,
        post_switches,
        move_ack,
        wait_row,
        delta,
        rail_24v=None,
    ):
        errors = []
        axis_label = str(axis) if axis is not None else None
        fwd_ok = self._tmcl_success(move_ack)
        if not fwd_ok:
            errors.append(self._motion_error_entry("ACK_FWD_FAIL", axis=axis_label))
        wait_ok = bool(isinstance(wait_row, dict) and wait_row.get("stopped") is True)
        if not wait_ok:
            errors.append(self._motion_error_entry("WAIT_FWD_TIMEOUT", axis=axis_label))
        if fwd_ok and isinstance(delta, int) and int(delta) == 0:
            errors.append(self._motion_error_entry("ACK_SUCCESS_NO_DELTA_FWD", axis=axis_label))

        pre_left = None if not isinstance(pre_switches, dict) else pre_switches.get("left_state")
        pre_right = None if not isinstance(pre_switches, dict) else pre_switches.get("right_state")
        post_left = None if not isinstance(post_switches, dict) else post_switches.get("left_state")
        post_right = None if not isinstance(post_switches, dict) else post_switches.get("right_state")
        if isinstance(step_cmd, int):
            if pre_left == 0 and pre_right == 1 and int(step_cmd) > 0 and post_left == 0:
                if isinstance(delta, int) and int(delta) != 0:
                    errors.append(self._motion_error_entry("REPORTED_MOVE_SWITCH_MISMATCH", axis=axis_label))
                else:
                    errors.append(self._motion_error_entry("ESCAPE_SWITCH_NOT_CLEARED", axis=axis_label))
            if pre_right == 0 and pre_left == 1 and int(step_cmd) < 0 and post_right == 0:
                if isinstance(delta, int) and int(delta) != 0:
                    errors.append(self._motion_error_entry("REPORTED_MOVE_SWITCH_MISMATCH", axis=axis_label))
                else:
                    errors.append(self._motion_error_entry("ESCAPE_SWITCH_NOT_CLEARED", axis=axis_label))

        rail = rail_24v if isinstance(rail_24v, dict) else self.motor_query_24v_sensor()
        if isinstance(rail, dict) and rail.get("no24v") is True:
            errors.append(self._motion_error_entry("NO_24V_DURING_PROBE", axis=axis_label, detail=f"raw={rail.get('raw')}"))

        return {
            "ok": len(errors) == 0,
            "error_code": None if len(errors) == 0 else int(errors[0]["code"]),
            "error_count": len(errors),
            "errors": errors,
            "rail_24v": rail,
        }

    def _axis_micro_motion_probe(self, axis_key, step_mag=600, speed=260, acc=120):
        p = self.motor_function_preset(axis_key)
        if not p:
            return {"error": f"unknown axis '{axis_key}'"}
        board = int(p["board"])
        motor = int(p["motor"])
        sw0 = self.motor_get_switch_activity(board, motor=motor)
        left_active = sw0.get("left_active")
        right_active = sw0.get("right_active")
        mag = max(100, abs(int(step_mag)))
        if left_active is True and right_active is not True:
            signed = +mag
        elif right_active is True and left_active is not True:
            signed = -mag
        else:
            signed = +mag
        prep = self.motor_prepare_axis(
            board,
            motor=motor,
            run_current=int(p.get("run_current", 31)),
            standby_current=int(p.get("standby_current", 10)),
            speed=int(speed),
            acc=int(acc),
            stall_guard=p.get("stall_guard"),
            ramp_mode=p.get("ramp_mode"),
            disable_right=bool(p.get("disable_right", False)),
            disable_left=bool(p.get("disable_left", False)),
            rdiv=p.get("rdiv"),
            pdiv=p.get("pdiv"),
            warm_enable=False,
        )
        test = self.motor_step_test(board, steps=int(signed), motor=motor, wait_timeout_s=5.0)
        sw1 = self.motor_get_switch_activity(board, motor=motor)
        out = {
            "axis": str(p.get("label", axis_key)).upper(),
            "board": board,
            "motor": motor,
            "switch_before": sw0,
            "switch_after": sw1,
            "step_cmd": int(signed),
            "prep": prep,
            "test": test,
        }
        out["validation"] = self._validate_axis_micro_motion_probe(out)
        return out

    def head_lock_actuator_probe(self, y_steps=1000, z_steps=500, speed=260, acc=120, settle_s=0.25, fail_fast=True):
        """
        Determine whether deck latch/solenoid state gates practical head motion.
        Sequence: lock -> unlock -> lock, with Y/Z micro motion checks at each state.
        """
        t0 = time.time()
        self.reconnect()
        board_status = self.activate_boards(expect_reply=True)
        io0 = self.io_snapshot(self.BOARD_DECK)
        rail0 = self.motor_query_24v_sensor()
        phases = []
        aborted = False
        for locked in (1, 0, 1):
            wr = self.deck_io_set_type(2, int(locked))
            time.sleep(max(0.05, float(settle_s)))
            snap = self.io_snapshot(self.BOARD_DECK)
            rail = self.motor_query_24v_sensor()
            y_probe = self._axis_micro_motion_probe("y", step_mag=int(y_steps), speed=int(speed), acc=int(acc))
            z_probe = self._axis_micro_motion_probe("z", step_mag=int(z_steps), speed=int(speed), acc=int(acc))
            y_probe["validation"] = self._validate_axis_micro_motion_probe(y_probe, rail_24v=rail)
            z_probe["validation"] = self._validate_axis_micro_motion_probe(z_probe, rail_24v=rail)

            phase_errors = []
            for axis_label, probe in (("Y", y_probe), ("Z", z_probe)):
                val = probe.get("validation", {}) if isinstance(probe, dict) else {}
                if isinstance(val, dict) and val.get("ok") is not True:
                    phase_errors.append(
                        {
                            "axis": axis_label,
                            "error_code": val.get("error_code"),
                            "errors": list(val.get("errors", [])),
                        }
                    )
            phase_ok = len(phase_errors) == 0
            phases.append(
                {
                    "state": "LOCK" if int(locked) == 1 else "UNLOCK",
                    "write": wr,
                    "snapshot": snap,
                    "rail_24v": rail,
                    "y_probe": y_probe,
                    "z_probe": z_probe,
                    "ok": phase_ok,
                    "error_code": None if phase_ok else phase_errors[0].get("error_code"),
                    "errors": phase_errors,
                }
            )
            if bool(fail_fast) and not phase_ok:
                aborted = True
                break
        io1 = self.io_snapshot(self.BOARD_DECK)
        rail1 = self.motor_query_24v_sensor()
        all_phase_errors = []
        for ph in phases:
            all_phase_errors.extend(list(ph.get("errors", [])))
        ok = bool(phases and all(bool(ph.get("ok")) for ph in phases))
        top_error_code = None if ok else (None if len(all_phase_errors) == 0 else all_phase_errors[0].get("error_code"))
        return {
            "board_status": board_status,
            "io_before": io0,
            "io_after": io1,
            "rail_before": rail0,
            "rail_after": rail1,
            "phases": phases,
            "ok": ok,
            "aborted": bool(aborted),
            "error_code": top_error_code,
            "errors": all_phase_errors,
            "elapsed_ms": int((time.time() - t0) * 1000),
        }

    def bus_health(self, cycles=20, interval_s=0.30):
        stats = {bid: {"ok": 0, "noresp": 0, "last": "n/a"} for bid in self.BOARDS}
        all_dead_streak = 0
        for i in range(cycles):
            line = [f"{i+1:02d}/{cycles}"]
            cycle_ok = 0
            for bid in self.BOARDS:
                r = self.send_tmcl_retry(
                    bid,
                    64,
                    0,
                    0,
                    1,
                    attempts=1,
                    wait_reply=True,
                    write_timeout_ms=35,
                    read_timeout_ms=30,
                    max_reads=10,
                    strict_match=True,
                )
                if r is None:
                    stats[bid]["noresp"] += 1
                    stats[bid]["last"] = "no resp"
                    line.append(f"0x{bid:02X}:NR")
                else:
                    stats[bid]["ok"] += 1
                    stats[bid]["last"] = r["status_str"]
                    line.append(f"0x{bid:02X}:{r['status_str'][:3]}")
                    cycle_ok += 1
                time.sleep(0.003)
            print("  " + " ".join(line))
            if cycle_ok == 0:
                all_dead_streak += 1
            else:
                all_dead_streak = 0
            if all_dead_streak >= 2:
                print("  [auto] all boards no-response streak; reconnecting USB interface...")
                self.reconnect()
                all_dead_streak = 0
            time.sleep(interval_s)
        return stats

    def latch_oem(self, locked):
        state = 1 if locked else 0
        t0 = time.time()
        self.activate_boards(expect_reply=False)

        # OEM-equivalent write: board 0x05, cmd14, type2, motor0, value {0|1}
        ack = self.send_tmcl_retry(
            self.BOARD_DECK,
            14,
            2,
            0,
            state,
            attempts=3,
            wait_reply=True,
            write_timeout_ms=65,
            read_timeout_ms=85,
            max_reads=20,
            strict_match=True,
        )
        # Reinforce with two non-blocking writes.
        for _ in range(2):
            self.send_tmcl(
                self.BOARD_DECK,
                14,
                2,
                0,
                state,
                wait_reply=False,
                write_timeout_ms=20,
            )
            time.sleep(0.12)

        snap = self.io_snapshot(self.BOARD_DECK)
        return {
            "ack": ack,
            "snapshot": snap,
            "elapsed_ms": int((time.time() - t0) * 1000),
        }

    def latch_compat(self, locked):
        state = 1 if locked else 0
        t0 = time.time()
        sent = 0
        got = 0

        # Compatibility mode: long timeouts + broad SIO path.
        self.activate_boards(expect_reply=False)

        primary = self.send_tmcl(
            self.BOARD_DECK,
            14,
            2,
            0,
            state,
            wait_reply=True,
            write_timeout_ms=120,
            read_timeout_ms=120,
            max_reads=24,
            strict_match=False,
        )
        sent += 1
        if primary is not None:
            got += 1

        for bid in self.BOARDS:
            self.send_tmcl(bid, 14, 2, 0, state, wait_reply=False, write_timeout_ms=40)
            sent += 1
            time.sleep(0.01)

        for bid in self.BOARDS:
            for port in range(8):
                r = self.send_tmcl(
                    bid,
                    14,
                    port,
                    0,
                    state,
                    wait_reply=True,
                    write_timeout_ms=120,
                    read_timeout_ms=100,
                    max_reads=20,
                    strict_match=False,
                )
                sent += 1
                if r is not None:
                    got += 1
                time.sleep(0.03)

        snap = self.io_snapshot(self.BOARD_DECK)
        return {
            "primary": primary,
            "snapshot": snap,
            "sent": sent,
            "got": got,
            "elapsed_ms": int((time.time() - t0) * 1000),
        }

    def latch_oneshot(self, locked, broad=False):
        """
        Write-only latch actuation path with forced reconnect.
        Use this when response handling is unreliable but bus TX still works.
        """
        state = 1 if locked else 0
        t0 = time.time()
        sent = 0

        self.reconnect()
        self.activate_boards(expect_reply=False)
        sent += len(self.BOARDS)

        for _ in range(3):
            self.send_tmcl(
                self.BOARD_DECK,
                14,
                2,
                0,
                state,
                wait_reply=False,
                write_timeout_ms=40,
            )
            sent += 1
            time.sleep(0.08)

        if broad:
            for bid in self.BOARDS:
                self.send_tmcl(bid, 14, 2, 0, state, wait_reply=False, write_timeout_ms=40)
                sent += 1
                time.sleep(0.006)
            for bid in self.BOARDS:
                for port in range(8):
                    self.send_tmcl(bid, 14, port, 0, state, wait_reply=False, write_timeout_ms=30)
                    sent += 1
                    time.sleep(0.008)

        return {"sent": sent, "elapsed_ms": int((time.time() - t0) * 1000), "broad": broad}

    @staticmethod
    def _clamp_u8(value):
        return max(0, min(255, int(value)))

    @staticmethod
    def _led_scale_to_tmcl(intensity):
        # OEM setLED: tmcl_value = (1024 * intensity) / 255
        return (1024 * BioXpTester._clamp_u8(intensity)) // 255

    def led_write(self, mask, value, broad=False, retries=2, prep=True, recover=True, reinforce=True):
        """
        OEM LED write from decompiled ClassIOControl.setLED:
          cmd=50, type=0, motor(mask), value(int32)
        """
        t0 = time.time()
        sent = 0
        boards = self.BOARDS if broad else [self.BOARD_DECK]
        acks = {}

        if prep:
            self.activate_boards(expect_reply=False)
            sent += len(self.BOARDS)

        for bid in boards:
            ack = self.send_tmcl_retry(
                bid,
                50,
                0,
                int(mask),
                int(value),
                attempts=max(1, retries),
                wait_reply=True,
                write_timeout_ms=55,
                read_timeout_ms=65,
                max_reads=16,
                strict_match=True,
            )
            sent += 1
            if ack is None and recover:
                # If bus RX is stale, recover once and retry exact same write.
                self.reconnect()
                self.activate_boards(expect_reply=False)
                sent += len(self.BOARDS)
                ack = self.send_tmcl(
                    bid,
                    50,
                    0,
                    int(mask),
                    int(value),
                    wait_reply=True,
                    write_timeout_ms=65,
                    read_timeout_ms=85,
                    max_reads=20,
                    strict_match=True,
                )
                sent += 1
            acks[bid] = ack

            # Reinforce with one non-blocking write after the acked command.
            if reinforce:
                self.send_tmcl(
                    bid,
                    50,
                    0,
                    int(mask),
                    int(value),
                    wait_reply=False,
                    write_timeout_ms=25,
                )
                sent += 1
            time.sleep(0.03)

        return {
            "board_mode": "broad" if broad else f"0x{self.BOARD_DECK:02X}",
            "mask": int(mask),
            "value": int(value),
            "acks": acks,
            "sent": sent,
            "elapsed_ms": int((time.time() - t0) * 1000),
        }

    def led_mask_scaled(self, mask, intensity, broad=False):
        val = self._led_scale_to_tmcl(intensity)
        out = self.led_write(mask=int(mask), value=val, broad=broad, retries=2)
        out["intensity"] = self._clamp_u8(intensity)
        out["tmcl_value"] = val
        return out

    def led_logo_pwm_raw(self, pwm_raw, broad=False):
        # setLogoPWM forwards user integer directly to setLED(mask=3, intensity=pwm_raw)
        out = self.led_write(mask=3, value=int(pwm_raw), broad=broad, retries=2)
        out["pwm_raw"] = int(pwm_raw)
        return out

    def led_rgb_scaled(self, r, g, b, broad=False):
        t0 = time.time()
        rr = self.led_mask_scaled(0, r, broad=broad)
        gg = self.led_mask_scaled(1, g, broad=broad)
        bb = self.led_mask_scaled(2, b, broad=broad)
        sent = rr["sent"] + gg["sent"] + bb["sent"]
        acks = {}
        for bid in (self.BOARDS if broad else [self.BOARD_DECK]):
            acks[bid] = {
                "r": rr["acks"].get(bid),
                "g": gg["acks"].get(bid),
                "b": bb["acks"].get(bid),
            }
        return {
            "rgb": (self._clamp_u8(r), self._clamp_u8(g), self._clamp_u8(b)),
            "board_mode": "broad" if broad else f"0x{self.BOARD_DECK:02X}",
            "acks": acks,
            "sent": sent,
            "elapsed_ms": int((time.time() - t0) * 1000),
        }

    def led_firstpath_restart(self, leave_on=True):
        """
        Deterministic CAN-only LED test for the OEM path.
        Uses one reconnect + one board-status check, then a fixed command sequence.
        """
        self.reconnect()
        st = self.activate_boards(expect_reply=True)
        steps = [
            ("TOP raw 0", 3, 0),
            ("RGB off raw 0", 0, 0),
            ("RGB off raw 0", 1, 0),
            ("RGB off raw 0", 2, 0),
            ("RED on", 0, self._led_scale_to_tmcl(255)),
            ("GREEN on", 1, self._led_scale_to_tmcl(255)),
            ("BLUE on", 2, self._led_scale_to_tmcl(255)),
        ]
        if not leave_on:
            steps.extend(
                [
                    ("RGB off raw 0", 0, 0),
                    ("RGB off raw 0", 1, 0),
                    ("RGB off raw 0", 2, 0),
                    ("TOP raw 0", 3, 0),
                ]
            )
        out = []
        for label, mask, value in steps:
            r = self.led_write(
                mask=mask,
                value=value,
                broad=False,
                retries=1,
                prep=False,
                recover=False,
                reinforce=False,
            )
            out.append(
                {
                    "label": label,
                    "mask": mask,
                    "value": value,
                    "ack": r["acks"].get(self.BOARD_DECK),
                    "elapsed_ms": r["elapsed_ms"],
                }
            )
            time.sleep(0.35)
        return {"status": st, "steps": out}

    def led_firstpath_matrix(self):
        """
        Exhaustive but bounded probe for OEM cmd50 across boards/masks/values.
        """
        self.reconnect()
        st = self.activate_boards(expect_reply=True)
        values = [0, 64, 255, 1024, 4095]
        masks = [3, 0, 1, 2]
        rows = []
        for bid in self.BOARDS:
            for mask in masks:
                for value in values:
                    ack = self.send_tmcl_retry(
                        bid,
                        50,
                        0,
                        mask,
                        value,
                        attempts=2,
                        wait_reply=True,
                        write_timeout_ms=55,
                        read_timeout_ms=65,
                        max_reads=16,
                        strict_match=True,
                    )
                    rows.append({"board": bid, "mask": mask, "value": value, "ack": ack})
                    time.sleep(0.03)
        return {"status": st, "rows": rows}

    def strip_set_rgb(self, r, g, b, reconnect_first=True):
        """
        Main strip control using the proven visible path (RGB channels 0/1/2).
        """
        r = self._clamp_u8(r)
        g = self._clamp_u8(g)
        b = self._clamp_u8(b)
        t0 = time.time()
        sent = 0

        if reconnect_first:
            self.reconnect()
        self.activate_boards(expect_reply=False)
        sent += len(self.BOARDS)

        acks = {}
        for mask, level, key in ((0, r, "r"), (1, g, "g"), (2, b, "b")):
            sval = self._led_scale_to_tmcl(level)
            ack = self.send_tmcl_retry(
                self.BOARD_DECK,
                50,
                0,
                mask,
                sval,
                attempts=2,
                wait_reply=True,
                write_timeout_ms=55,
                read_timeout_ms=65,
                max_reads=16,
                strict_match=True,
            )
            sent += 1
            if ack is None:
                # Fallback write when ack path is flaky.
                self.send_tmcl(self.BOARD_DECK, 50, 0, mask, sval, wait_reply=False, write_timeout_ms=25)
                sent += 1
            acks[key] = ack
            time.sleep(0.04)

        return {
            "rgb": (r, g, b),
            "tmcl": (
                self._led_scale_to_tmcl(r),
                self._led_scale_to_tmcl(g),
                self._led_scale_to_tmcl(b),
            ),
            "acks": acks,
            "sent": sent,
            "elapsed_ms": int((time.time() - t0) * 1000),
        }

    def strip_set_pct(self, pct):
        pct = max(0, min(100, int(pct)))
        level = int(round((pct / 100.0) * 255))
        out = self.strip_set_rgb(level, level, level, reconnect_first=True)
        out["pct"] = pct
        return out

    def strip_on(self):
        return self.strip_set_rgb(255, 255, 255, reconnect_first=True)

    def strip_off(self):
        return self.strip_set_rgb(0, 0, 0, reconnect_first=True)

    @staticmethod
    def _wheel_rgb(pos):
        p = int(pos) % 256
        if p < 85:
            return 255 - (p * 3), p * 3, 0
        if p < 170:
            p -= 85
            return 0, 255 - (p * 3), p * 3
        p -= 170
        return p * 3, 0, 255 - (p * 3)

    def strip_rainbow_cycle(self, seconds=10.0, step_delay_s=0.08, brightness=255):
        seconds = max(0.5, float(seconds))
        step_delay_s = max(0.01, float(step_delay_s))
        brightness = self._clamp_u8(brightness)
        t0 = time.time()
        sent = 0
        frames = 0
        interrupted = False
        last = None

        self.reconnect()

        try:
            while (time.time() - t0) < seconds:
                r, g, b = self._wheel_rgb(frames)
                if brightness < 255:
                    r = (r * brightness) // 255
                    g = (g * brightness) // 255
                    b = (b * brightness) // 255
                last = self.strip_set_rgb(r, g, b, reconnect_first=False)
                sent += int(last.get("sent", 0))
                frames += 1
                time.sleep(step_delay_s)
        except KeyboardInterrupt:
            interrupted = True

        elapsed_ms = int((time.time() - t0) * 1000)
        return {
            "seconds": seconds,
            "step_delay_s": step_delay_s,
            "brightness": brightness,
            "frames": frames,
            "sent": sent,
            "last": last,
            "interrupted": interrupted,
            "elapsed_ms": elapsed_ms,
        }

    def status_with_recovery(self):
        st = self.activate_boards(expect_reply=True)
        snap = self.io_snapshot(self.BOARD_DECK)
        if all(st.get(bid) is None for bid in self.BOARDS):
            self.reconnect()
            st = self.activate_boards(expect_reply=True)
            snap = self.io_snapshot(self.BOARD_DECK)
        return st, snap

    def _chiller_pace(self, min_gap_s=None):
        gap = self.CHILLER_MIN_CMD_GAP_S if min_gap_s is None else float(min_gap_s)
        now = time.monotonic()
        dt = now - float(self._chiller_last_tx_ts)
        if dt < gap:
            time.sleep(gap - dt)
        self._chiller_last_tx_ts = time.monotonic()

    def _send_chiller(
        self,
        command,
        cmd_type,
        motor,
        value,
        *,
        attempts=2,
        wait_reply=True,
        write_timeout_ms=55,
        read_timeout_ms=70,
        max_reads=20,
        strict_match=True,
        post_delay_s=None,
        allow_recover=True,
    ):
        self._chiller_pace()
        ack = self.send_tmcl_retry(
            self.BOARD_CHILLER,
            int(command),
            int(cmd_type),
            int(motor),
            int(value),
            attempts=max(1, int(attempts)),
            wait_reply=bool(wait_reply),
            write_timeout_ms=int(write_timeout_ms),
            read_timeout_ms=int(read_timeout_ms),
            max_reads=int(max_reads),
            strict_match=bool(strict_match),
        )
        if wait_reply:
            if ack is None:
                self._chiller_noresp_streak += 1
            else:
                self._chiller_noresp_streak = 0
            if ack is None and allow_recover and self._chiller_noresp_streak >= 2:
                self.reconnect()
                self._chiller_pace(min_gap_s=0.10)
                ack = self.send_tmcl(
                    self.BOARD_CHILLER,
                    int(command),
                    int(cmd_type),
                    int(motor),
                    int(value),
                    wait_reply=True,
                    write_timeout_ms=max(60, int(write_timeout_ms)),
                    read_timeout_ms=max(80, int(read_timeout_ms)),
                    max_reads=max(24, int(max_reads)),
                    strict_match=bool(strict_match),
                )
                if ack is None:
                    self._chiller_noresp_streak += 1
                else:
                    self._chiller_noresp_streak = 0
        if post_delay_s is not None and float(post_delay_s) > 0:
            time.sleep(float(post_delay_s))
        return ack

    def _thermal_pace(self, min_gap_s=None):
        gap = self.THERMAL_MIN_CMD_GAP_S if min_gap_s is None else float(min_gap_s)
        now = time.monotonic()
        dt = now - float(self._thermal_last_tx_ts)
        if dt < gap:
            time.sleep(gap - dt)
        self._thermal_last_tx_ts = time.monotonic()

    def _send_thermal(
        self,
        command,
        cmd_type,
        motor,
        value,
        *,
        attempts=2,
        wait_reply=True,
        write_timeout_ms=55,
        read_timeout_ms=70,
        max_reads=20,
        strict_match=True,
        post_delay_s=None,
        allow_recover=True,
    ):
        self._thermal_pace()
        ack = self.send_tmcl_retry(
            self.BOARD_THERMAL,
            int(command),
            int(cmd_type),
            int(motor),
            int(value),
            attempts=max(1, int(attempts)),
            wait_reply=bool(wait_reply),
            write_timeout_ms=int(write_timeout_ms),
            read_timeout_ms=int(read_timeout_ms),
            max_reads=int(max_reads),
            strict_match=bool(strict_match),
        )
        if wait_reply:
            if ack is None:
                self._thermal_noresp_streak += 1
            else:
                self._thermal_noresp_streak = 0
            if ack is None and allow_recover and self._thermal_noresp_streak >= 2:
                self.reconnect()
                self._thermal_pace(min_gap_s=0.10)
                ack = self.send_tmcl(
                    self.BOARD_THERMAL,
                    int(command),
                    int(cmd_type),
                    int(motor),
                    int(value),
                    wait_reply=True,
                    write_timeout_ms=max(60, int(write_timeout_ms)),
                    read_timeout_ms=max(80, int(read_timeout_ms)),
                    max_reads=max(24, int(max_reads)),
                    strict_match=bool(strict_match),
                )
                if ack is None:
                    self._thermal_noresp_streak += 1
                else:
                    self._thermal_noresp_streak = 0
        if post_delay_s is not None and float(post_delay_s) > 0:
            time.sleep(float(post_delay_s))
        return ack

    def _motor_pace(self, board_id, min_gap_s=None):
        bid = int(board_id)
        if bid not in self._motor_last_tx_ts:
            self._motor_last_tx_ts[bid] = 0.0
        gap = self.MOTOR_MIN_CMD_GAP_S if min_gap_s is None else float(min_gap_s)
        now = time.monotonic()
        dt = now - float(self._motor_last_tx_ts.get(bid, 0.0))
        if dt < gap:
            time.sleep(gap - dt)
        self._motor_last_tx_ts[bid] = time.monotonic()

    def _send_motor(
        self,
        board_id,
        command,
        cmd_type,
        motor,
        value,
        *,
        attempts=2,
        wait_reply=True,
        write_timeout_ms=55,
        read_timeout_ms=70,
        max_reads=20,
        strict_match=True,
        post_delay_s=None,
        allow_recover=True,
    ):
        board_id = int(board_id)
        self._motor_pace(board_id)
        ack = self.send_tmcl_retry(
            board_id,
            int(command),
            int(cmd_type),
            int(motor),
            int(value),
            attempts=max(1, int(attempts)),
            wait_reply=bool(wait_reply),
            write_timeout_ms=int(write_timeout_ms),
            read_timeout_ms=int(read_timeout_ms),
            max_reads=int(max_reads),
            strict_match=bool(strict_match),
        )
        if wait_reply:
            if board_id not in self._motor_noresp_streak:
                self._motor_noresp_streak[board_id] = 0
            if ack is None:
                self._motor_noresp_streak[board_id] += 1
            else:
                self._motor_noresp_streak[board_id] = 0
            if (
                ack is None
                and allow_recover
                and board_id in self.MOTOR_RECOVERY_BOARDS
                and self._motor_noresp_streak.get(board_id, 0) >= 2
            ):
                self.reconnect()
                self._motor_pace(board_id, min_gap_s=self.MOTOR_POST_RECOVER_GAP_S)
                ack = self.send_tmcl(
                    board_id,
                    int(command),
                    int(cmd_type),
                    int(motor),
                    int(value),
                    wait_reply=True,
                    write_timeout_ms=max(60, int(write_timeout_ms)),
                    read_timeout_ms=max(80, int(read_timeout_ms)),
                    max_reads=max(24, int(max_reads)),
                    strict_match=bool(strict_match),
                )
                if ack is None:
                    self._motor_noresp_streak[board_id] = self._motor_noresp_streak.get(board_id, 0) + 1
                else:
                    self._motor_noresp_streak[board_id] = 0
        if post_delay_s is not None and float(post_delay_s) > 0:
            time.sleep(float(post_delay_s))
        return ack

    def _motor_send_noreply_burst(self, board_id, command, cmd_type, motor, value, repeats=3, delay_s=0.05):
        attempted = 0
        sent = 0
        for _ in range(max(1, int(repeats))):
            attempted += 1
            self._motor_pace(board_id, min_gap_s=max(0.02, float(delay_s)))
            r = self.send_tmcl(
                int(board_id),
                int(command),
                int(cmd_type),
                int(motor),
                int(value),
                wait_reply=False,
                write_timeout_ms=35,
            )
            if r is not None:
                sent += 1
        return {"attempted": attempted, "sent": sent}

    @staticmethod
    def board_label(board_id):
        labels = {
            BioXpTester.BOARD_HEAD: "HEAD",
            BioXpTester.BOARD_DECK: "DECK",
            BioXpTester.BOARD_THERMAL: "THERMAL",
            BioXpTester.BOARD_CHILLER: "CHILLER",
        }
        return labels.get(int(board_id), f"0x{int(board_id):02X}")

    def motor_get_axis_param(self, board_id, param, motor=0):
        ack = self._send_motor(
            int(board_id),
            6,      # GAP
            int(param),
            int(motor),
            0,
            attempts=2,
            wait_reply=True,
            write_timeout_ms=55,
            read_timeout_ms=70,
            max_reads=18,
            strict_match=True,
        )
        val = ack.get("value") if ack else None
        return {"board": int(board_id), "param": int(param), "motor": int(motor), "ack": ack, "value": val}

    def motor_set_axis_param(self, board_id, param, value, motor=0):
        ack = self._send_motor(
            int(board_id),
            5,      # SAP
            int(param),
            int(motor),
            int(value),
            attempts=2,
            wait_reply=True,
            write_timeout_ms=55,
            read_timeout_ms=70,
            max_reads=18,
            strict_match=True,
        )
        rb = self.motor_get_axis_param(board_id, param, motor=motor)
        return {
            "board": int(board_id),
            "param": int(param),
            "motor": int(motor),
            "set_value": int(value),
            "ack": ack,
            "readback": rb,
            "ok": self._tmcl_success(ack),
        }

    def motor_get_position(self, board_id, motor=0):
        # GAP param 1 = actual position (from prior direct experiments).
        row = self.motor_get_axis_param(board_id, 1, motor=motor)
        return {
            "board": int(board_id),
            "motor": int(motor),
            "ack": row.get("ack"),
            "position": row.get("value"),
            "ok": self._tmcl_success(row.get("ack")),
        }

    def motor_get_speed(self, board_id, motor=0):
        # GAP param 3 = actual speed (per OEM ClassMotor.queryMotorSpeed).
        row = self.motor_get_axis_param(board_id, 3, motor=motor)
        return {
            "board": int(board_id),
            "motor": int(motor),
            "ack": row.get("ack"),
            "speed": row.get("value"),
            "ok": self._tmcl_success(row.get("ack")),
        }

    def motor_get_switches(self, board_id, motor=0):
        # GAP param 9/10 = left/right switch status in OEM path.
        left = self.motor_get_axis_param(board_id, 9, motor=motor)
        right = self.motor_get_axis_param(board_id, 10, motor=motor)
        return {
            "board": int(board_id),
            "motor": int(motor),
            "left": left,
            "right": right,
            "left_state": left.get("value"),
            "right_state": right.get("value"),
        }

    def motor_get_switch_activity(self, board_id, motor=0):
        """
        OEM switch polarity is active-low:
        - left/home active when GAP9 == 0
        - right/open active when GAP10 == 0
        """
        sw = self.motor_get_switches(board_id, motor=motor)
        left = sw.get("left_state")
        right = sw.get("right_state")
        return {
            "board": int(board_id),
            "motor": int(motor),
            "left_state": left,
            "right_state": right,
            "left_active": None if left is None else (int(left) == 0),
            "right_active": None if right is None else (int(right) == 0),
            "switches": sw,
        }

    def motor_query_24v_sensor(self):
        # OEM query24VSensor: cmd 15 type 0 on deck board IO.
        ack = self._send_motor(
            self.BOARD_DECK,
            15,
            0,
            0,
            0,
            attempts=2,
            wait_reply=True,
            write_timeout_ms=55,
            read_timeout_ms=70,
            max_reads=18,
            strict_match=True,
        )
        raw = None if ack is None else ack.get("value")
        # In OEM ClassDeckBoard.queryVoltage: non-zero means No24V.
        no24v = None if raw is None else bool(int(raw) != 0)
        return {"ack": ack, "raw": raw, "no24v": no24v}

    @staticmethod
    def _safe_int(value):
        try:
            return int(value)
        except Exception:
            return None

    def motion_arm_state(self):
        return dict(self._motion_arm) if isinstance(self._motion_arm, dict) else {"armed": False}

    def motion_latch_override_state(self):
        if isinstance(self._motion_latch_override, dict):
            return dict(self._motion_latch_override)
        return {"enabled": False, "reason": "unset", "note": None, "updated_ms": None, "seq": 0}

    def motion_latch_override_set(self, enabled, reason="manual"):
        target_enabled = bool(enabled)
        prev_seq = 0
        if isinstance(self._motion_latch_override, dict):
            prev_seq = int(self._motion_latch_override.get("seq", 0) or 0)
        # Enabled = diagnostic unlock mode; disabled = default lock mode.
        wr = self.latch_oem(False if target_enabled else True)
        note = "diagnostic_unlocked" if target_enabled else "auto_lock"
        self._motion_latch_override = {
            "enabled": bool(target_enabled),
            "reason": str(reason),
            "note": note,
            "updated_ms": int(time.time() * 1000),
            "seq": prev_seq + 1,
        }
        live = self.motion_gate_live_snapshot()
        return {"state": dict(self._motion_latch_override), "write": wr, "gate": live}

    def motion_last_strict_init_report(self):
        return self._motion_last_strict_init

    def motion_disarm(self, reason="manual", note=None):
        prev_seq = 0
        if isinstance(self._motion_arm, dict):
            prev_seq = int(self._motion_arm.get("arm_seq", 0) or 0)
        self._motion_arm = {
            "armed": False,
            "reason": str(reason),
            "note": None if note is None else str(note),
            "updated_ms": int(time.time() * 1000),
            "arm_seq": prev_seq + 1,
        }
        return dict(self._motion_arm)

    def motion_gate_live_snapshot(self):
        io = self.io_snapshot(self.BOARD_DECK)
        rail = self.motor_query_24v_sensor()
        door = self._safe_int(None if not isinstance(io, dict) else io.get(1))
        solenoid = self._safe_int(None if not isinstance(io, dict) else io.get(2))
        latch = self._safe_int(None if not isinstance(io, dict) else io.get(3))
        ov = self.motion_latch_override_state()
        ov_enabled = bool(ov.get("enabled"))
        io_ok = (door is not None) and (solenoid is not None) and (latch is not None)
        checks = {"rail_ok": rail.get("no24v") is False, "io_read_ok": io_ok}
        error_keys = [k for k, v in checks.items() if v is not True]
        if not io_ok:
            checks.update({"door_closed": None, "latch_closed": None, "solenoid_locked": None})
            error_keys.append("io_unavailable")
        else:
            checks.update(
                {
                    "door_closed": door == 1,
                    "latch_closed": True if ov_enabled else (latch == 1),
                    "solenoid_locked": True if ov_enabled else (solenoid == 1),
                    "latch_override_active": ov_enabled,
                }
            )
            error_keys.extend(
                [
                    k
                    for k in ("door_closed", "latch_closed", "solenoid_locked")
                    if checks.get(k) is not True
                ]
            )
        return {
            "io": io,
            "rail_24v": rail,
            "door_sensor": door,
            "solenoid_state": solenoid,
            "latch_sensor": latch,
            "latch_override": ov,
            "checks": checks,
            "error_keys": error_keys,
            "ok": len(error_keys) == 0,
        }

    def motion_gate_assert_live(self, auto_disarm=True):
        snap = self.motion_gate_live_snapshot()
        if (
            bool(auto_disarm)
            and bool(isinstance(self._motion_arm, dict) and self._motion_arm.get("armed"))
            and not bool(snap.get("ok"))
        ):
            self.motion_disarm(reason="sensor_gate_failed", note=",".join(snap.get("error_keys", [])))
        return snap

    def motion_arm_strict_startup(self, run_homing=True):
        """
        Strict startup gate for gantry motion:
        - reconnect + board activate
        - enforce deck lock path and verify deck IO/24V sensors
        - interlock wake sequence
        - full OEM-like startup homing
        """
        t0 = time.time()
        checks = []
        fail_names = []

        def _ack_text(ack):
            if ack is None:
                return "no response"
            try:
                status = int(ack.get("status", -1))
            except Exception:
                status = -1
            return TMCL_STATUS.get(status, f"?({status})")

        def add_check(name, ok, detail=None, critical=True):
            nonlocal fail_names
            row = {"name": str(name), "ok": bool(ok), "critical": bool(critical)}
            if detail is not None:
                row["detail"] = detail
            checks.append(row)
            if not bool(ok) and bool(critical):
                fail_names.append(str(name))

        self.reconnect()
        board_status = self.activate_boards(expect_reply=True)
        board_expect = (self.BOARD_HEAD, self.BOARD_DECK, self.BOARD_THERMAL)
        board_ok = all(self._tmcl_success(board_status.get(bid)) for bid in board_expect)
        add_check("board_activate", board_ok, detail={bid: _ack_text(board_status.get(bid)) for bid in board_expect})

        pre_gate = self.motion_gate_live_snapshot()
        add_check("pre_gate_live", pre_gate.get("ok"), detail=pre_gate.get("error_keys"))

        lock = self.latch_oem(True)
        add_check("lock_cmd_ack", self._tmcl_success(lock.get("ack")), detail=_ack_text(lock.get("ack")))
        time.sleep(0.12)

        post_lock_gate = self.motion_gate_live_snapshot()
        add_check("post_lock_gate_live", post_lock_gate.get("ok"), detail=post_lock_gate.get("error_keys"))

        interlock = self.motor_prepare_motion_interlock(force_lock=True)
        inter_rail = interlock.get("rail_24v", {}) if isinstance(interlock, dict) else {}
        add_check("interlock_24v", inter_rail.get("no24v") is False, detail=f"raw={inter_rail.get('raw')}")
        latch_ack = None
        if isinstance(interlock, dict) and isinstance(interlock.get("latch"), dict):
            latch_ack = interlock["latch"].get("ack")
        add_check("interlock_latch_ack", self._tmcl_success(latch_ack), detail=_ack_text(latch_ack))

        homing = None
        if bool(run_homing):
            homing = self.motor_startup_homing_mimic()

            def _home_check(label, row):
                if not isinstance(row, dict):
                    add_check(f"home_{label}", False, detail="missing")
                    return
                move_ok = self._tmcl_success(row.get("move_left", {}).get("ack"))
                wait_ok = bool(row.get("wait", {}).get("stopped") is True)
                stop_ok = self._tmcl_success(row.get("stop", {}).get("ack"))
                set_ok = self._tmcl_success(row.get("sethome_final", {}).get("ack"))
                hs = row.get("home_after", {}).get("value")
                hs_ok = hs == 0
                ok = bool(move_ok and wait_ok and stop_ok and set_ok and hs_ok)
                add_check(
                    f"home_{label}",
                    ok,
                    detail={
                        "move": _ack_text(row.get("move_left", {}).get("ack")),
                        "wait": row.get("wait", {}).get("stopped"),
                        "stop": _ack_text(row.get("stop", {}).get("ack")),
                        "set_home": _ack_text(row.get("sethome_final", {}).get("ack")),
                        "home_after": hs,
                    },
                )

            _home_check("Z", homing.get("z_home"))
            _home_check("G", homing.get("g_home"))
            _home_check("X", homing.get("x_home"))
            _home_check("Y", homing.get("y_home"))
            door_row = homing.get("door_home")
            if not isinstance(door_row, dict):
                add_check("home_DOOR", False, detail="missing", critical=False)
            else:
                move_ok = self._tmcl_success(door_row.get("move_left", {}).get("ack"))
                wait_ok = bool(door_row.get("wait", {}).get("stopped") is True)
                stop_ok = self._tmcl_success(door_row.get("stop", {}).get("ack"))
                set_ok = self._tmcl_success(door_row.get("sethome_final", {}).get("ack"))
                hs = door_row.get("home_after", {}).get("value")
                hs_ok = hs == 0
                add_check(
                    "home_DOOR",
                    bool(move_ok and wait_ok and stop_ok and set_ok and hs_ok),
                    detail={
                        "move": _ack_text(door_row.get("move_left", {}).get("ack")),
                        "wait": door_row.get("wait", {}).get("stopped"),
                        "stop": _ack_text(door_row.get("stop", {}).get("ack")),
                        "set_home": _ack_text(door_row.get("sethome_final", {}).get("ack")),
                        "home_after": hs,
                        "advisory_only": True,
                    },
                    critical=False,
                )
            add_check(
                "x_after_home_move6000",
                bool(
                    self._tmcl_success(homing.get("x_move_6000", {}).get("ack"))
                    and homing.get("x_wait", {}).get("stopped") is True
                ),
                detail={
                    "move": _ack_text(homing.get("x_move_6000", {}).get("ack")),
                    "wait": homing.get("x_wait", {}).get("stopped"),
                },
            )

        final_gate = self.motion_gate_live_snapshot()
        add_check("final_gate_live", final_gate.get("ok"), detail=final_gate.get("error_keys"))

        ok = all(bool(c.get("ok")) for c in checks if bool(c.get("critical", True)))
        error_code = None
        if not ok:
            if any(name.startswith("home_") or name.startswith("x_after_home") for name in fail_names):
                error_code = int(self.MOTION_ERROR_CODES.get("STRICT_HOMING_FAILED"))
            else:
                error_code = int(self.MOTION_ERROR_CODES.get("STRICT_INIT_FAILED"))

        prev_seq = int(self._motion_arm.get("arm_seq", 0) or 0) if isinstance(self._motion_arm, dict) else 0
        self._motion_arm = {
            "armed": bool(ok),
            "reason": "strict_init_pass" if ok else "strict_init_fail",
            "note": None if ok else ",".join(fail_names[:4]),
            "updated_ms": int(time.time() * 1000),
            "arm_seq": prev_seq + 1,
        }

        report = {
            "ok": bool(ok),
            "error_code": error_code,
            "board_status": board_status,
            "pre_gate": pre_gate,
            "lock": lock,
            "post_lock_gate": post_lock_gate,
            "interlock": interlock,
            "homing": homing,
            "final_gate": final_gate,
            "checks": checks,
            "arm_state": dict(self._motion_arm),
            "elapsed_ms": int((time.time() - t0) * 1000),
        }
        self._motion_last_strict_init = report
        return report

    def motor_query_motor_stop(self, board_id, motor=0):
        """
        OEM pre-move call (ClassMotor.queryMotorStop): cmd 138 type 0.
        """
        ack = self._send_motor(
            int(board_id),
            138,
            0,
            int(motor),
            0,
            attempts=2,
            wait_reply=True,
            write_timeout_ms=55,
            read_timeout_ms=70,
            max_reads=18,
            strict_match=True,
        )
        return {"board": int(board_id), "motor": int(motor), "ack": ack, "ok": self._tmcl_success(ack)}

    @classmethod
    def motor_axis_preset(cls, key):
        if key is None:
            return None
        return cls.MOTOR_AXIS_PRESETS.get(str(key).strip().lower())

    @classmethod
    def motor_function_preset(cls, key):
        if key is None:
            return None
        k = str(key).strip().lower()
        aliases = {
            "thermaldoor": "door",
            "thermal_door": "door",
            "d": "door",
            "gripper": "g",
        }
        k = aliases.get(k, k)
        return cls.MOTOR_FUNCTION_PRESETS.get(k)

    @classmethod
    def motor_axis_key_for_channel(cls, board_id, motor=0):
        bid = int(board_id)
        mid = int(motor)
        for key, preset in cls.MOTOR_FUNCTION_PRESETS.items():
            if int(preset.get("board", -1)) == bid and int(preset.get("motor", -1)) == mid:
                return str(key)
        return None

    @classmethod
    def motor_speed_acc_norms_for_channel(cls, board_id, motor=0):
        key = cls.motor_axis_key_for_channel(board_id, motor=motor)
        if key is None:
            return None
        return cls.MOTOR_SPEED_ACC_NORMS.get(key)

    def motor_normalize_speed_acc(self, board_id, motor=0, speed=None, acc=None):
        axis_key = self.motor_axis_key_for_channel(board_id, motor=motor)
        norms = self.motor_speed_acc_norms_for_channel(board_id, motor=motor)
        out_speed = None if speed is None else int(speed)
        out_acc = None if acc is None else int(acc)
        changes = []
        if isinstance(norms, dict):
            if out_speed is not None:
                lo = int(norms["speed_min"])
                hi = int(norms["speed_max"])
                clamped = max(lo, min(hi, out_speed))
                if clamped != out_speed:
                    changes.append(
                        {
                            "field": "speed",
                            "requested": int(out_speed),
                            "value": int(clamped),
                            "min": lo,
                            "max": hi,
                        }
                    )
                out_speed = int(clamped)
            if out_acc is not None:
                lo = int(norms["acc_min"])
                hi = int(norms["acc_max"])
                clamped = max(lo, min(hi, out_acc))
                if clamped != out_acc:
                    changes.append(
                        {
                            "field": "acc",
                            "requested": int(out_acc),
                            "value": int(clamped),
                            "min": lo,
                            "max": hi,
                        }
                    )
                out_acc = int(clamped)
        return {
            "axis_key": axis_key,
            "norms": norms,
            "speed": out_speed,
            "acc": out_acc,
            "changes": changes,
        }

    def motor_set_speed_acc(self, board_id, motor=0, speed=None, acc=None):
        norm = self.motor_normalize_speed_acc(board_id, motor=motor, speed=speed, acc=acc)
        speed = norm.get("speed")
        acc = norm.get("acc")
        rows = []
        for row in norm.get("changes", []):
            rows.append(
                {
                    "op": f"norm-{row.get('field')}-clamp",
                    "ack": None,
                    "rb": {
                        "axis": norm.get("axis_key"),
                        "requested": row.get("requested"),
                        "min": row.get("min"),
                        "max": row.get("max"),
                    },
                    "set": row.get("value"),
                }
            )
        if speed is not None:
            rs = self.motor_set_axis_param(board_id, 4, int(speed), motor=motor)  # max speed
            rows.append({"op": "sap4-max_speed", "ack": rs.get("ack"), "rb": rs.get("readback"), "set": int(speed)})
        if acc is not None:
            ra = self.motor_set_axis_param(board_id, 5, int(acc), motor=motor)  # max acc
            rows.append({"op": "sap5-max_acc", "ack": ra.get("ack"), "rb": ra.get("readback"), "set": int(acc)})
        return rows

    def motor_prepare_axis(
        self,
        board_id,
        motor=0,
        run_current=31,
        standby_current=10,
        speed=None,
        acc=None,
        stall_guard=None,
        ramp_mode=None,
        disable_right=None,
        disable_left=None,
        rdiv=None,
        pdiv=None,
        warm_enable=False,
        warm_current=10,
        warm_delay_s=0.50,
    ):
        """
        OEM-like prep for one axis:
        - SAP6 max current (clamped)
        - SAP7 standby current
        - optional SAP4/SAP5 speed/acc
        """
        ops = []
        rb_cur = self.motor_get_axis_param(board_id, 6, motor=motor)
        rb_cur_val = rb_cur.get("value")
        ops.append({"op": "gap6-current_before", "ack": rb_cur.get("ack"), "rb": rb_cur, "set": rb_cur_val})
        target_run = max(0, min(31, int(run_current)))
        target_standby = max(0, min(target_run, int(standby_current)))

        if bool(warm_enable):
            warm_target = max(0, min(31, int(warm_current)))
            rw = self.motor_set_axis_param(board_id, 6, warm_target, motor=motor)
            ops.append({"op": "sap6-warm_current", "ack": rw.get("ack"), "rb": rw.get("readback"), "set": warm_target})
            time.sleep(max(0.05, float(warm_delay_s)))

        r6 = self.motor_set_axis_param(board_id, 6, target_run, motor=motor)
        r7 = self.motor_set_axis_param(board_id, 7, target_standby, motor=motor)
        ops.append({"op": "sap6-run_current", "ack": r6.get("ack"), "rb": r6.get("readback"), "set": target_run})
        ops.append({"op": "sap7-standby_current", "ack": r7.get("ack"), "rb": r7.get("readback"), "set": target_standby})
        ops.extend(self.motor_set_speed_acc(board_id, motor=motor, speed=speed, acc=acc))
        if stall_guard is not None:
            rsg = self.motor_set_axis_param(board_id, 205, int(stall_guard), motor=motor)
            ops.append({"op": "sap205-stall_guard", "ack": rsg.get("ack"), "rb": rsg.get("readback"), "set": int(stall_guard)})
        if ramp_mode is not None:
            rrm = self.motor_set_axis_param(board_id, 138, int(ramp_mode), motor=motor)
            ops.append({"op": "sap138-ramp_mode", "ack": rrm.get("ack"), "rb": rrm.get("readback"), "set": int(ramp_mode)})
        if disable_right is not None:
            dr = 1 if bool(disable_right) else 0
            rr = self.motor_set_axis_param(board_id, 12, dr, motor=motor)
            ops.append({"op": "sap12-disable_right", "ack": rr.get("ack"), "rb": rr.get("readback"), "set": dr})
        if disable_left is not None:
            dl = 1 if bool(disable_left) else 0
            rl = self.motor_set_axis_param(board_id, 13, dl, motor=motor)
            ops.append({"op": "sap13-disable_left", "ack": rl.get("ack"), "rb": rl.get("readback"), "set": dl})
        if rdiv is not None:
            rrdiv = self.motor_set_axis_param(board_id, 153, int(rdiv), motor=motor)
            ops.append({"op": "sap153-rdiv", "ack": rrdiv.get("ack"), "rb": rrdiv.get("readback"), "set": int(rdiv)})
        if pdiv is not None:
            rpdiv = self.motor_set_axis_param(board_id, 154, int(pdiv), motor=motor)
            ops.append({"op": "sap154-pdiv", "ack": rpdiv.get("ack"), "rb": rpdiv.get("readback"), "set": int(pdiv)})
        return {"board": int(board_id), "motor": int(motor), "ops": ops}

    def motor_enable_sequence(self, conservative=True):
        """
        OEM-like axis prep for motion.
        """
        t0 = time.time()
        rows = []
        self.reconnect()
        self.activate_boards(expect_reply=False)
        for key in ("x", "y", "z"):
            p = self.motor_axis_preset(key)
            if p is None:
                continue
            prep = self.motor_prepare_axis(
                p["board"],
                motor=p["motor"],
                run_current=p["run_current"] if conservative else 31,
                standby_current=p["standby_current"] if conservative else 10,
                speed=p["speed"],
                acc=p["acc"],
                stall_guard=p.get("stall_guard"),
                ramp_mode=p.get("ramp_mode"),
                disable_right=bool(p.get("disable_right", False)),
                disable_left=bool(p.get("disable_left", False)),
                rdiv=p.get("rdiv"),
                pdiv=p.get("pdiv"),
                warm_enable=bool(p.get("warm_enable", False)),
            )
            for op in prep.get("ops", []):
                rows.append(
                    {
                        "axis": p["label"],
                        "board": prep["board"],
                        "motor": prep["motor"],
                        "op": op.get("op"),
                        "ack": op.get("ack"),
                        "rb": op.get("rb"),
                        "set": op.get("set"),
                    }
                )
        rail = self.motor_query_24v_sensor()
        rows.append(
            {
                "axis": "IO",
                "board": self.BOARD_DECK,
                "motor": 0,
                "op": "query24v",
                "ack": rail.get("ack"),
                "rb": {"value": rail.get("raw")},
                "set": None,
            }
        )
        return {"rows": rows, "elapsed_ms": int((time.time() - t0) * 1000)}

    def motor_prepare_motion_interlock(self, force_lock=True):
        """
        OEM-like wake path before gantry motion:
        - Optionally force latch lock on deck IO (cmd14/type2=1).
        - Pulse X/Y current 10 -> 31 with settle delays (as in enableXYZ).
        - Ensure Z run current is set.
        """
        t0 = time.time()
        self.reconnect()
        self.activate_boards(expect_reply=True)

        snap_before = self.io_snapshot(self.BOARD_DECK)
        latch = None
        if bool(force_lock):
            latch = self.latch_oem(True)
            time.sleep(0.08)
        snap_after = self.io_snapshot(self.BOARD_DECK)

        px = self.motor_function_preset("x")
        py = self.motor_function_preset("y")
        pz = self.motor_function_preset("z")

        ops = []
        for p in (px, py):
            r = self.motor_set_axis_param(p["board"], 6, 10, motor=p["motor"])
            ops.append(
                {
                    "axis": p["label"],
                    "op": "sap6-wake-10",
                    "board": p["board"],
                    "motor": p["motor"],
                    "ack": r.get("ack"),
                    "rb": r.get("readback"),
                    "set": 10,
                }
            )
        time.sleep(0.50)
        for p in (px, py):
            r = self.motor_set_axis_param(p["board"], 6, int(p["run_current"]), motor=p["motor"])
            ops.append(
                {
                    "axis": p["label"],
                    "op": "sap6-run",
                    "board": p["board"],
                    "motor": p["motor"],
                    "ack": r.get("ack"),
                    "rb": r.get("readback"),
                    "set": int(p["run_current"]),
                }
            )
        time.sleep(0.50)
        rz = self.motor_set_axis_param(pz["board"], 6, int(pz["run_current"]), motor=pz["motor"])
        ops.append(
            {
                "axis": pz["label"],
                "op": "sap6-run",
                "board": pz["board"],
                "motor": pz["motor"],
                "ack": rz.get("ack"),
                "rb": rz.get("readback"),
                "set": int(pz["run_current"]),
            }
        )

        rail = self.motor_query_24v_sensor()
        return {
            "snap_before": snap_before,
            "snap_after": snap_after,
            "latch": latch,
            "ops": ops,
            "rail_24v": rail,
            "elapsed_ms": int((time.time() - t0) * 1000),
        }

    def motor_move_relative(self, board_id, steps, motor=0):
        # cmd 4 type 1 = move to relative position (OEM MovetoRelPosition).
        pre_stop = self.motor_query_motor_stop(board_id, motor=motor)
        time.sleep(0.001)
        ack = self._send_motor(
            int(board_id),
            4,
            1,
            int(motor),
            int(steps),
            attempts=2,
            wait_reply=True,
            write_timeout_ms=55,
            read_timeout_ms=70,
            max_reads=18,
            strict_match=True,
        )
        return {
            "board": int(board_id),
            "motor": int(motor),
            "steps": int(steps),
            "pre_stop": pre_stop,
            "ack": ack,
            "ok": self._tmcl_success(ack),
        }

    def motor_move_absolute(self, board_id, position, motor=0):
        # cmd 4 type 0 = move to absolute position (OEM moveToAbs).
        pre_stop = self.motor_query_motor_stop(board_id, motor=motor)
        time.sleep(0.001)
        ack = self._send_motor(
            int(board_id),
            4,
            0,
            int(motor),
            int(position),
            attempts=2,
            wait_reply=True,
            write_timeout_ms=55,
            read_timeout_ms=70,
            max_reads=18,
            strict_match=True,
        )
        return {
            "board": int(board_id),
            "motor": int(motor),
            "position": int(position),
            "pre_stop": pre_stop,
            "ack": ack,
            "ok": self._tmcl_success(ack),
        }

    def motor_move_left(self, board_id, speed=250, motor=0):
        # OEM ClassMotor.MoveLeft uses cmd=2 with positive speed payload.
        vel = max(1, abs(int(speed)))
        ack = self._send_motor(
            int(board_id),
            2,
            0,
            int(motor),
            vel,
            attempts=2,
            wait_reply=True,
            write_timeout_ms=55,
            read_timeout_ms=70,
            max_reads=18,
            strict_match=True,
        )
        return {"board": int(board_id), "motor": int(motor), "speed": vel, "ack": ack, "ok": self._tmcl_success(ack)}

    def motor_query_home_switch(self, board_id, motor=0):
        row = self.motor_get_axis_param(board_id, 9, motor=motor)
        return {
            "board": int(board_id),
            "motor": int(motor),
            "ack": row.get("ack"),
            "value": row.get("value"),
            "ok": self._tmcl_success(row.get("ack")),
        }

    def motor_set_home(self, board_id, motor=0):
        # OEM ClassMotor.setHome => SAP param 1, value 0.
        row = self.motor_set_axis_param(board_id, 1, 0, motor=motor)
        return {
            "board": int(board_id),
            "motor": int(motor),
            "ack": row.get("ack"),
            "readback": row.get("readback"),
            "ok": self._tmcl_success(row.get("ack")),
        }

    def motor_axis_search_home(
        self,
        board_id,
        motor=0,
        speed=250,
        preclear_abs=10000,
        timeout_s=15.0,
        home_active_value=0,
        poll_s=0.05,
    ):
        """
        Approximate OEM axisSearchHome/goHome:
        - If already at home switch, move away first.
        - Drive left at fixed speed until home switch triggers.
        - Stop and set home.
        """
        t0 = time.time()
        log = []
        hs0 = self.motor_query_home_switch(board_id, motor=motor)
        log.append({"step": "home_before", **hs0})

        preclear = None
        prewait = None
        if hs0.get("value") == int(home_active_value):
            preclear = self.motor_move_absolute(board_id, int(preclear_abs), motor=motor)
            prewait = self.motor_wait_stopped(board_id, motor=motor, timeout_s=8.0)
            time.sleep(0.50)

        sethome_init = self.motor_set_home(board_id, motor=motor)
        move = self.motor_move_left(board_id, speed=int(speed), motor=motor)

        home_hit = None
        wait = None
        polls = 0
        while time.time() - t0 < max(2.0, float(timeout_s)):
            hs = self.motor_query_home_switch(board_id, motor=motor)
            sp = self.motor_get_speed(board_id, motor=motor)
            polls += 1
            log.append({"step": "poll", "home": hs.get("value"), "speed": sp.get("speed")})
            if hs.get("value") == int(home_active_value):
                home_hit = hs
                break
            # If motor appears stopped, still keep polling briefly for switch update.
            if isinstance(sp.get("speed"), int) and int(sp.get("speed")) == 0 and polls > 10:
                time.sleep(0.10)
            time.sleep(max(0.02, float(poll_s)))

        stop = self.motor_stop(board_id, motor=motor)
        wait = self.motor_wait_stopped(board_id, motor=motor, timeout_s=3.0)
        sethome_final = self.motor_set_home(board_id, motor=motor)
        hs1 = self.motor_query_home_switch(board_id, motor=motor)
        pos = self.motor_get_position(board_id, motor=motor)
        return {
            "board": int(board_id),
            "motor": int(motor),
            "speed": int(speed),
            "home_active_value": int(home_active_value),
            "preclear": preclear,
            "preclear_wait": prewait,
            "sethome_init": sethome_init,
            "move_left": move,
            "home_hit": home_hit,
            "stop": stop,
            "wait": wait,
            "sethome_final": sethome_final,
            "home_after": hs1,
            "position_after": pos,
            "elapsed_ms": int((time.time() - t0) * 1000),
            "log_tail": log[-20:],
        }

    def motor_startup_homing_mimic(self):
        """
        Approximate ControlInterface.initializeMotors startup homing:
        Z home -> gripper prep/home -> X home+setHome+move 6000 -> Y home+setHome -> door home.
        """
        t0 = time.time()
        self.reconnect()
        board_status = self.activate_boards(expect_reply=True)

        px = self.motor_function_preset("x")
        py = self.motor_function_preset("y")
        pz = self.motor_function_preset("z")
        pg = self.motor_function_preset("g")
        pd = self.motor_function_preset("door")

        prep = {
            "x": self.motor_prepare_axis(
                px["board"],
                motor=px["motor"],
                run_current=px["run_current"],
                standby_current=px["standby_current"],
                speed=px["speed"],
                acc=px["acc"],
                stall_guard=px.get("stall_guard"),
                ramp_mode=px.get("ramp_mode"),
                disable_right=bool(px.get("disable_right", False)),
                disable_left=bool(px.get("disable_left", False)),
                rdiv=px.get("rdiv"),
                pdiv=px.get("pdiv"),
                warm_enable=bool(px.get("warm_enable", False)),
            ),
            "y": self.motor_prepare_axis(
                py["board"],
                motor=py["motor"],
                run_current=py["run_current"],
                standby_current=py["standby_current"],
                speed=py["speed"],
                acc=py["acc"],
                stall_guard=py.get("stall_guard"),
                ramp_mode=py.get("ramp_mode"),
                disable_right=bool(py.get("disable_right", False)),
                disable_left=bool(py.get("disable_left", False)),
                rdiv=py.get("rdiv"),
                pdiv=py.get("pdiv"),
                warm_enable=bool(py.get("warm_enable", False)),
            ),
            "z": self.motor_prepare_axis(
                pz["board"],
                motor=pz["motor"],
                run_current=pz["run_current"],
                standby_current=pz["standby_current"],
                speed=pz["speed"],
                acc=pz["acc"],
                stall_guard=pz.get("stall_guard"),
                ramp_mode=pz.get("ramp_mode"),
                disable_right=bool(pz.get("disable_right", False)),
                disable_left=bool(pz.get("disable_left", False)),
                rdiv=pz.get("rdiv"),
                pdiv=pz.get("pdiv"),
                warm_enable=bool(pz.get("warm_enable", False)),
            ),
            "g": self.motor_prepare_axis(
                pg["board"],
                motor=pg["motor"],
                run_current=pg["run_current"],
                standby_current=pg["standby_current"],
                speed=pg["speed"],
                acc=pg["acc"],
                stall_guard=pg.get("stall_guard"),
                ramp_mode=pg.get("ramp_mode"),
                disable_right=bool(pg.get("disable_right", False)),
                disable_left=bool(pg.get("disable_left", False)),
                rdiv=pg.get("rdiv"),
                pdiv=pg.get("pdiv"),
                warm_enable=bool(pg.get("warm_enable", False)),
            ),
            "door": self.motor_prepare_axis(
                pd["board"],
                motor=pd["motor"],
                run_current=pd["run_current"],
                standby_current=pd["standby_current"],
                speed=pd["speed"],
                acc=pd["acc"],
                stall_guard=pd.get("stall_guard"),
                ramp_mode=pd.get("ramp_mode"),
                disable_right=bool(pd.get("disable_right", False)),
                disable_left=bool(pd.get("disable_left", False)),
                rdiv=pd.get("rdiv"),
                pdiv=pd.get("pdiv"),
                warm_enable=bool(pd.get("warm_enable", False)),
            ),
        }

        # OEM initializeMotors bumps gripper positive before homing.
        g_pre_move = self.motor_move_relative(pg["board"], 10000, motor=pg["motor"])
        g_pre_wait = self.motor_wait_stopped(pg["board"], motor=pg["motor"], timeout_s=10.0)

        z_home = self.motor_axis_search_home(
            pz["board"],
            motor=pz["motor"],
            speed=1791,
            timeout_s=12.0,
            home_active_value=0,
        )
        g_home = self.motor_axis_search_home(
            pg["board"],
            motor=pg["motor"],
            speed=600,
            timeout_s=10.0,
            home_active_value=0,
        )
        x_home = self.motor_axis_search_home(
            px["board"],
            motor=px["motor"],
            speed=250,
            timeout_s=12.0,
            home_active_value=0,
        )
        x_set_home = self.motor_set_home(px["board"], motor=px["motor"])
        x_speed = self.motor_set_axis_param(px["board"], 4, 1700, motor=px["motor"])
        x_move_6000 = self.motor_move_absolute(px["board"], 6000, motor=px["motor"])
        x_wait = self.motor_wait_stopped(px["board"], motor=px["motor"], timeout_s=8.0)
        y_home = self.motor_axis_search_home(
            py["board"],
            motor=py["motor"],
            speed=250,
            timeout_s=12.0,
            home_active_value=0,
        )
        y_set_home = self.motor_set_home(py["board"], motor=py["motor"])
        door_home = self.motor_axis_search_home(
            pd["board"],
            motor=pd["motor"],
            speed=int(pd["speed"]),
            timeout_s=10.0,
            home_active_value=0,
        )

        rail = self.motor_query_24v_sensor()
        return {
            "board_status": board_status,
            "rail_24v": rail,
            "prep": prep,
            "g_pre_move": g_pre_move,
            "g_pre_wait": g_pre_wait,
            "z_home": z_home,
            "g_home": g_home,
            "x_home": x_home,
            "x_set_home": x_set_home,
            "x_speed": x_speed,
            "x_move_6000": x_move_6000,
            "x_wait": x_wait,
            "y_home": y_home,
            "y_set_home": y_set_home,
            "door_home": door_home,
            "elapsed_ms": int((time.time() - t0) * 1000),
        }

    def motor_wait_stopped(self, board_id, motor=0, timeout_s=4.0, poll_s=0.06):
        t0 = time.monotonic()
        deadline = t0 + max(0.3, float(timeout_s))
        polls = 0
        last = None
        last_speed = None
        while time.monotonic() < deadline:
            row = self.motor_get_speed(board_id, motor=motor)
            polls += 1
            last = row.get("ack")
            last_speed = row.get("speed")
            if isinstance(last_speed, int) and last_speed == 0:
                return {
                    "stopped": True,
                    "elapsed_ms": int((time.monotonic() - t0) * 1000),
                    "polls": polls,
                    "last_speed": last_speed,
                    "last_ack": last,
                }
            time.sleep(max(0.02, float(poll_s)))
        return {
            "stopped": False,
            "elapsed_ms": int((time.monotonic() - t0) * 1000),
            "polls": polls,
            "last_speed": last_speed,
            "last_ack": last,
        }

    def motor_axis_status(self, board_id, motor=0):
        pos = self.motor_get_position(board_id, motor=motor)
        spd = self.motor_get_speed(board_id, motor=motor)
        cur = self.motor_get_axis_param(board_id, 6, motor=motor)
        sw = self.motor_get_switches(board_id, motor=motor)
        return {
            "board": int(board_id),
            "motor": int(motor),
            "position": pos,
            "speed": spd,
            "max_current": cur,
            "switches": sw,
        }

    def motor_step_test(self, board_id, steps=2000, motor=0, wait_timeout_s=4.0):
        pre = self.motor_get_position(board_id, motor=motor)
        pre_sw = self.motor_get_switches(board_id, motor=motor)
        fwd = self.motor_move_relative(board_id, int(steps), motor=motor)
        wait1 = self.motor_wait_stopped(board_id, motor=motor, timeout_s=wait_timeout_s)
        mid = self.motor_get_position(board_id, motor=motor)
        mid_sw = self.motor_get_switches(board_id, motor=motor)

        back = self.motor_move_relative(board_id, -int(steps), motor=motor)
        wait2 = self.motor_wait_stopped(board_id, motor=motor, timeout_s=wait_timeout_s)
        post = self.motor_get_position(board_id, motor=motor)
        post_sw = self.motor_get_switches(board_id, motor=motor)

        d1 = None if pre.get("position") is None or mid.get("position") is None else int(mid["position"]) - int(pre["position"])
        d2 = None if mid.get("position") is None or post.get("position") is None else int(post["position"]) - int(mid["position"])
        dn = None if pre.get("position") is None or post.get("position") is None else int(post["position"]) - int(pre["position"])
        return {
            "board": int(board_id),
            "motor": int(motor),
            "steps": int(steps),
            "pre": pre,
            "fwd": fwd,
            "wait_fwd": wait1,
            "mid": mid,
            "pre_switches": pre_sw,
            "mid_switches": mid_sw,
            "back": back,
            "wait_back": wait2,
            "post": post,
            "post_switches": post_sw,
            "delta_fwd": d1,
            "delta_back": d2,
            "delta_net": dn,
        }

    def motor_xz_power_diagnostic(self, steps_x=3200, steps_z=1800):
        t0 = time.time()
        self.reconnect()
        board_status = self.activate_boards(expect_reply=True)
        rail = self.motor_query_24v_sensor()

        px = self.motor_axis_preset("x")
        pz = self.motor_axis_preset("z")
        prep_x = self.motor_prepare_axis(
            px["board"],
            motor=px["motor"],
            run_current=px["run_current"],
            standby_current=px["standby_current"],
            speed=px["speed"],
            acc=px["acc"],
            stall_guard=px.get("stall_guard"),
            ramp_mode=px.get("ramp_mode"),
            disable_right=bool(px.get("disable_right", False)),
            disable_left=bool(px.get("disable_left", False)),
            rdiv=px.get("rdiv"),
            pdiv=px.get("pdiv"),
            warm_enable=bool(px.get("warm_enable", False)),
        )
        prep_z = self.motor_prepare_axis(
            pz["board"],
            motor=pz["motor"],
            run_current=pz["run_current"],
            standby_current=pz["standby_current"],
            speed=pz["speed"],
            acc=pz["acc"],
            stall_guard=pz.get("stall_guard"),
            ramp_mode=pz.get("ramp_mode"),
            disable_right=bool(pz.get("disable_right", False)),
            disable_left=bool(pz.get("disable_left", False)),
            rdiv=pz.get("rdiv"),
            pdiv=pz.get("pdiv"),
            warm_enable=bool(pz.get("warm_enable", False)),
        )
        test_x = self.motor_step_test(px["board"], steps=int(steps_x), motor=px["motor"], wait_timeout_s=4.0)
        test_z = self.motor_step_test(pz["board"], steps=int(steps_z), motor=pz["motor"], wait_timeout_s=4.0)
        return {
            "board_status": board_status,
            "rail_24v": rail,
            "prep_x": prep_x,
            "prep_z": prep_z,
            "test_x": test_x,
            "test_z": test_z,
            "elapsed_ms": int((time.time() - t0) * 1000),
        }

    def motor_hard_reset(self, rounds=3):
        rounds = max(1, int(rounds))
        rows = []
        recovered = False
        for idx in range(rounds):
            self.reconnect()
            time.sleep(0.10)
            self.drain(max_reads=20, timeout_ms=4)

            ops = []
            for board_id, label in ((self.BOARD_DECK, "DECK"), (self.BOARD_HEAD, "HEAD")):
                ops.append(
                    {
                        "name": f"{label} deactivate",
                        **self._motor_send_noreply_burst(board_id, 64, 0, 0, 0, repeats=2),
                    }
                )
                ops.append(
                    {
                        "name": f"{label} activate",
                        **self._motor_send_noreply_burst(board_id, 64, 0, 0, 1, repeats=2),
                    }
                )

            for board_id, motor, label in (
                (self.BOARD_DECK, 0, "X"),
                (self.BOARD_HEAD, 0, "Y"),
                (self.BOARD_HEAD, 1, "Z"),
                (self.BOARD_HEAD, 2, "GRIPPER"),
            ):
                ops.append(
                    {
                        "name": f"{label} stop (cmd3/MST)",
                        **self._motor_send_noreply_burst(board_id, 3, 0, motor, 0, repeats=2),
                    }
                )

            time.sleep(0.10)
            fw_head = self.board_query_firmware(self.BOARD_HEAD)
            fw_deck = self.board_query_firmware(self.BOARD_DECK)
            rail = self.motor_query_24v_sensor()
            head_ok = self._tmcl_success(fw_head.get("ack"))
            deck_ok = self._tmcl_success(fw_deck.get("ack"))
            alive = bool(head_ok and deck_ok)
            rows.append(
                {
                    "round": idx + 1,
                    "ops": ops,
                    "firmware_head": fw_head,
                    "firmware_deck": fw_deck,
                    "rail_24v": rail,
                    "alive": alive,
                }
            )
            if alive:
                recovered = True
                break
        return {"ok": recovered, "rounds": rows}

    def motor_quick_reliability_smoke(self, loops=2, step_scale=1.0):
        """
        Fast movement confidence check for gantry axes (X/Y/Z).
        Intended runtime: roughly 1-2 minutes depending on loop count.
        """
        t0 = time.time()
        loops = max(1, min(5, int(loops)))
        scale = max(0.2, min(2.0, float(step_scale)))
        default_steps = {"x": 2200, "y": 1800, "z": 900}

        self.reconnect()
        board_status = self.activate_boards(expect_reply=True)
        interlock = self.motor_prepare_motion_interlock(force_lock=True)
        rail = self.motor_query_24v_sensor()

        axes = {}
        for key in ("x", "y", "z"):
            p = self.motor_function_preset(key)
            board = int(p["board"])
            motor = int(p["motor"])
            label = str(p.get("label", key)).upper()
            step_mag = max(300, int(round(float(default_steps[key]) * scale)))
            runs = []
            for idx in range(loops):
                sw = self.motor_get_switch_activity(board, motor=motor)
                left_active = sw.get("left_active")
                right_active = sw.get("right_active")
                if left_active is True and right_active is not True:
                    step = +step_mag
                elif right_active is True and left_active is not True:
                    step = -step_mag
                else:
                    step = step_mag if (idx % 2 == 0) else -step_mag
                out = self.motor_step_test(board, steps=step, motor=motor, wait_timeout_s=5.0)
                fwd_ack = out.get("fwd", {}).get("ack")
                back_ack = out.get("back", {}).get("ack")
                ack_ok = bool(self._tmcl_success(fwd_ack) and self._tmcl_success(back_ack))
                wait_ok = bool(out.get("wait_fwd", {}).get("stopped") and out.get("wait_back", {}).get("stopped"))
                d1 = out.get("delta_fwd")
                d2 = out.get("delta_back")
                motion_ok = bool(isinstance(d1, int) and isinstance(d2, int) and int(d1) != 0 and int(d2) != 0)
                run_ok = bool(ack_ok and wait_ok and motion_ok)
                runs.append(
                    {
                        "idx": idx + 1,
                        "step": int(step),
                        "switches_before": sw,
                        "result": out,
                        "ack_ok": ack_ok,
                        "wait_ok": wait_ok,
                        "motion_ok": motion_ok,
                        "ok": run_ok,
                    }
                )
            axes[label] = {
                "board": board,
                "motor": motor,
                "step_mag": step_mag,
                "loops": loops,
                "runs": runs,
                "ok": bool(runs and all(r.get("ok") for r in runs)),
            }

        return {
            "board_status": board_status,
            "interlock": interlock,
            "rail_24v": rail,
            "loops": loops,
            "step_scale": scale,
            "axes": axes,
            "ok": bool(axes and all(row.get("ok") for row in axes.values())),
            "elapsed_ms": int((time.time() - t0) * 1000),
        }

    def motor_rotate(self, board_id, velocity, motor=0):
        vel = int(abs(int(velocity)))
        if vel == 0:
            return self.motor_stop(board_id, motor=motor)
        cmd = 1 if int(velocity) > 0 else 2  # ROR / ROL
        ack = self._send_motor(
            int(board_id),
            cmd,
            0,
            int(motor),
            vel,
            attempts=2,
            wait_reply=True,
            write_timeout_ms=55,
            read_timeout_ms=70,
            max_reads=18,
            strict_match=True,
        )
        return {
            "board": int(board_id),
            "motor": int(motor),
            "cmd": cmd,
            "velocity": int(velocity),
            "ack": ack,
            "ok": self._tmcl_success(ack),
        }

    def motor_stop(self, board_id, motor=0):
        ack = self._send_motor(
            int(board_id),
            3,  # MST
            0,
            int(motor),
            0,
            attempts=2,
            wait_reply=True,
            write_timeout_ms=55,
            read_timeout_ms=70,
            max_reads=18,
            strict_match=True,
        )
        return {"board": int(board_id), "motor": int(motor), "ack": ack, "ok": self._tmcl_success(ack)}

    def motor_spin_test(self, board_id, velocity, seconds=2.0, motor=0):
        dur = max(0.2, min(8.0, float(seconds)))
        pre = self.motor_get_position(board_id, motor=motor)
        start = self.motor_rotate(board_id, velocity, motor=motor)
        time.sleep(dur)
        stop = self.motor_stop(board_id, motor=motor)
        post = self.motor_get_position(board_id, motor=motor)
        return {
            "board": int(board_id),
            "motor": int(motor),
            "velocity": int(velocity),
            "seconds": dur,
            "pre": pre,
            "start": start,
            "stop": stop,
            "post": post,
            "delta": None
            if pre.get("position") is None or post.get("position") is None
            else int(post["position"]) - int(pre["position"]),
        }

    @staticmethod
    def _probe_switch_direction_guard(pre_left, pre_right, steps):
        sign = 1 if int(steps) >= 0 else -1
        suggested_dir = None
        blocked = False
        reason = None
        if pre_right == 0 and pre_left == 1:
            suggested_dir = "-"
        elif pre_left == 0 and pre_right == 1:
            suggested_dir = "+"
        if pre_left == 0 and pre_right == 0:
            blocked = True
            reason = "both limit switches report active"
        elif pre_left == 0 and pre_right == 1 and sign < 0:
            blocked = True
            reason = "left/home switch active and requested direction drives further into it"
        elif pre_right == 0 and pre_left == 1 and sign > 0:
            blocked = True
            reason = "right/end switch active and requested direction drives further into it"
        return {
            "blocked": bool(blocked),
            "reason": reason,
            "suggested_dir": suggested_dir,
            "sign": int(sign),
        }

    def motor_visible_oneway_probe(self, board_id, steps, motor=0, timeout_s=20.0, enforce_switch_guard=True):
        """
        One-way move probe (no auto-return) to validate physical motion.
        """
        self.drain(max_reads=10, timeout_ms=4)
        pre = self.motor_get_position(board_id, motor=motor)
        pre_sw = self.motor_get_switches(board_id, motor=motor)
        pre_left = pre_sw.get("left_state")
        pre_right = pre_sw.get("right_state")
        guard = self._probe_switch_direction_guard(pre_left, pre_right, int(steps))
        if bool(enforce_switch_guard) and bool(guard.get("blocked")):
            return {
                "board": int(board_id),
                "motor": int(motor),
                "steps": int(steps),
                "pre": pre,
                "pre_switches": pre_sw,
                "move": None,
                "wait": None,
                "post": pre,
                "post_switches": pre_sw,
                "delta": 0,
                "left_cleared": False,
                "right_cleared": False,
                "any_switch_edge": False,
                "suggested_dir": guard.get("suggested_dir"),
                "events": [],
                "stall_events": [],
                "aborted": True,
                "abort_reason": guard.get("reason"),
                "guard": guard,
            }

        mv = self.motor_move_relative(board_id, int(steps), motor=motor)
        wt = self.motor_wait_stopped(board_id, motor=motor, timeout_s=max(4.0, float(timeout_s)))
        events = self.collect_bus_events(duration_s=0.50, timeout_ms=18, max_events=120)
        post = self.motor_get_position(board_id, motor=motor)
        post_sw = self.motor_get_switches(board_id, motor=motor)
        delta = None
        if pre.get("position") is not None and post.get("position") is not None:
            delta = int(post["position"]) - int(pre["position"])
        post_left = post_sw.get("left_state")
        post_right = post_sw.get("right_state")
        # OEM switch logic is active-low: 0=active/hit, 1=inactive/clear.
        left_cleared = (pre_left == 0 and post_left == 1)
        right_cleared = (pre_right == 0 and post_right == 1)
        any_switch_edge = (pre_left != post_left) or (pre_right != post_right)
        suggested_dir = guard.get("suggested_dir")
        return {
            "board": int(board_id),
            "motor": int(motor),
            "steps": int(steps),
            "pre": pre,
            "pre_switches": pre_sw,
            "move": mv,
            "wait": wt,
            "post": post,
            "post_switches": post_sw,
            "delta": delta,
            "left_cleared": left_cleared,
            "right_cleared": right_cleared,
            "any_switch_edge": any_switch_edge,
            "suggested_dir": suggested_dir,
            "events": events,
            "stall_events": [e for e in events if int(e.get("status", -1)) == 130],
            "aborted": False,
            "abort_reason": None,
            "guard": guard,
        }

    def motor_visible_stepwise_probe(
        self,
        board_id,
        steps,
        motor=0,
        chunk_steps=2000,
        timeout_s=10.0,
        settle_s=0.06,
    ):
        """
        Low-noise probe: break a larger move into small segments and stop if
        the axis reports no delta.
        """
        total = int(steps)
        sign = 1 if total >= 0 else -1
        remain = abs(int(total))
        chunk = max(200, abs(int(chunk_steps)))
        tmo = max(3.0, float(timeout_s))
        segs = []

        pre = self.motor_get_position(board_id, motor=motor)
        pre_sw = self.motor_get_switches(board_id, motor=motor)
        prev_pos = pre.get("position")

        idx = 0
        while remain > 0:
            idx += 1
            step = sign * min(chunk, remain)
            mv = self.motor_move_relative(board_id, step, motor=motor)
            wt = self.motor_wait_stopped(board_id, motor=motor, timeout_s=tmo, poll_s=0.06)
            pos = self.motor_get_position(board_id, motor=motor)
            sw = self.motor_get_switches(board_id, motor=motor)
            now_pos = pos.get("position")
            delta = None if prev_pos is None or now_pos is None else int(now_pos) - int(prev_pos)
            seg = {
                "idx": idx,
                "step": int(step),
                "move": mv,
                "wait": wt,
                "position": now_pos,
                "delta": delta,
                "left_state": sw.get("left_state"),
                "right_state": sw.get("right_state"),
            }
            segs.append(seg)
            if not self._tmcl_success(mv.get("ack")):
                break
            if not bool(wt.get("stopped")):
                break
            if delta is None or int(delta) == 0:
                break
            prev_pos = now_pos
            remain -= abs(int(step))
            time.sleep(max(0.01, float(settle_s)))

        post = self.motor_get_position(board_id, motor=motor)
        post_sw = self.motor_get_switches(board_id, motor=motor)
        delta_total = None
        if pre.get("position") is not None and post.get("position") is not None:
            delta_total = int(post["position"]) - int(pre["position"])
        return {
            "board": int(board_id),
            "motor": int(motor),
            "requested_steps": int(total),
            "chunk_steps": int(chunk),
            "segments": segs,
            "pre": pre,
            "pre_switches": pre_sw,
            "post": post,
            "post_switches": post_sw,
            "delta": delta_total,
        }

    def motor_read_param_set(self, board_id, motor=0, params=()):
        out = {}
        for param in params:
            gp = self.motor_get_axis_param(board_id, int(param), motor=motor)
            out[int(param)] = {"ack": gp.get("ack"), "value": gp.get("value")}
        return out

    def motor_gentle_creep_probe(self, axis_key, steps=8000):
        """
        Low-speed/low-accel probe to reduce banging and distinguish
        true axis translation from vibration.
        """
        p = self.motor_function_preset(axis_key)
        if not p:
            return {"error": f"unknown axis '{axis_key}'"}
        label = str(p.get("label", axis_key)).upper()
        board = int(p["board"])
        motor = int(p["motor"])

        is_gantry = label in ("X", "Y", "Z")
        inter = None
        if is_gantry:
            inter = self.motor_prepare_motion_interlock(force_lock=True)

        plan = [
            {"run_current": 10, "standby": 6, "speed": 120, "acc": 60},
            {"run_current": 14, "standby": 8, "speed": 180, "acc": 90},
            {"run_current": 18, "standby": 10, "speed": 260, "acc": 120},
            {"run_current": 24, "standby": 10, "speed": 340, "acc": 170},
        ]
        sign = -1 if label in ("X", "Y", "Z") else 1
        step_mag = max(1000, int(abs(int(steps))))
        trials = []
        for idx, cfg in enumerate(plan, start=1):
            prep = self.motor_prepare_axis(
                board,
                motor=motor,
                run_current=int(cfg["run_current"]),
                standby_current=int(cfg["standby"]),
                speed=int(cfg["speed"]),
                acc=int(cfg["acc"]),
                stall_guard=p.get("stall_guard"),
                ramp_mode=p.get("ramp_mode"),
                disable_right=bool(p.get("disable_right", False)),
                disable_left=bool(p.get("disable_left", False)),
                rdiv=p.get("rdiv"),
                pdiv=p.get("pdiv"),
                warm_enable=False,
            )
            pre = self.motor_get_position(board, motor=motor)
            mv_f = self.motor_move_relative(board, sign * step_mag, motor=motor)
            wt_f = self.motor_wait_stopped(board, motor=motor, timeout_s=18.0, poll_s=0.08)
            mid = self.motor_get_position(board, motor=motor)
            sw_mid = self.motor_get_switches(board, motor=motor)
            mv_b = self.motor_move_relative(board, -sign * step_mag, motor=motor)
            wt_b = self.motor_wait_stopped(board, motor=motor, timeout_s=18.0, poll_s=0.08)
            post = self.motor_get_position(board, motor=motor)
            sw_post = self.motor_get_switches(board, motor=motor)
            d_f = None if pre.get("position") is None or mid.get("position") is None else int(mid["position"]) - int(pre["position"])
            d_b = None if mid.get("position") is None or post.get("position") is None else int(post["position"]) - int(mid["position"])
            d_n = None if pre.get("position") is None or post.get("position") is None else int(post["position"]) - int(pre["position"])
            trials.append(
                {
                    "idx": idx,
                    "cfg": cfg,
                    "prep": prep,
                    "pre": pre,
                    "mid": mid,
                    "post": post,
                    "switch_mid": sw_mid,
                    "switch_post": sw_post,
                    "move_fwd": mv_f,
                    "move_back": mv_b,
                    "wait_fwd": wt_f,
                    "wait_back": wt_b,
                    "delta_fwd": d_f,
                    "delta_back": d_b,
                    "delta_net": d_n,
                }
            )
            time.sleep(0.12)

        return {
            "axis": label,
            "board": board,
            "motor": motor,
            "steps": step_mag,
            "interlock": inter,
            "trials": trials,
        }

    def motor_probe_axis_params(self, board_id, motor=0, params=None):
        """
        Fast read-only GAP sweep for axis diagnostics.
        """
        plist = list(self.MOTOR_DRIVER_DIAG_PARAMS if params is None else params)
        rows = []
        for param in plist:
            ack = self._send_motor(
                int(board_id),
                6,  # GAP
                int(param),
                int(motor),
                0,
                attempts=1,
                wait_reply=True,
                write_timeout_ms=45,
                read_timeout_ms=55,
                max_reads=14,
                strict_match=True,
            )
            rows.append(
                {
                    "param": int(param),
                    "ack": ack,
                    "status": None if ack is None else ack.get("status"),
                    "status_str": None if ack is None else TMCL_STATUS.get(int(ack.get("status", -1)), f"code {ack.get('status')}"),
                    "value": None if ack is None else ack.get("value"),
                }
            )
        return {"board": int(board_id), "motor": int(motor), "rows": rows}

    def motor_driver_power_diag(self, axis_keys=("x", "y", "z", "g", "door"), params=None):
        """
        One-click motion power diagnostic:
        - reconnect + board activation + interlock wake
        - axis status snapshot
        - low-level GAP param probe for each axis
        """
        t0 = time.time()
        self.reconnect()
        board_status = self.activate_boards(expect_reply=True)
        interlock = self.motor_prepare_motion_interlock(force_lock=True)
        rail = self.motor_query_24v_sensor()

        probes = {}
        statuses = {}
        matrix = {}
        for key in axis_keys:
            p = self.motor_function_preset(key)
            if not p:
                continue
            label = str(p.get("label", str(key))).upper()
            board = int(p["board"])
            motor = int(p["motor"])
            statuses[label] = self.motor_axis_status(board, motor=motor)
            probe = self.motor_probe_axis_params(board, motor=motor, params=params)
            probes[label] = probe
            for row in probe.get("rows", []):
                pv = matrix.setdefault(int(row["param"]), {})
                pv[label] = row.get("value")
        return {
            "board_status": board_status,
            "interlock": interlock,
            "rail_24v": rail,
            "statuses": statuses,
            "probes": probes,
            "matrix": matrix,
            "elapsed_ms": int((time.time() - t0) * 1000),
        }

    def motor_clone_driver_profile(self, source_key="g", dest_keys=("x", "y", "z"), params=None):
        """
        Experimental helper:
        copy selected low-level motor params from a known-working axis
        to target axes, then read back.
        """
        src = self.motor_function_preset(source_key)
        if not src:
            return {"error": f"unknown source axis '{source_key}'"}
        src_board = int(src["board"])
        src_motor = int(src["motor"])
        plist = list(self.MOTOR_DRIVER_COPY_PARAMS if params is None else params)

        source_rows = []
        source_values = {}
        for param in plist:
            row = self.motor_get_axis_param(src_board, int(param), motor=src_motor)
            ack = row.get("ack")
            ok = self._tmcl_success(ack)
            val = row.get("value") if ok else None
            source_rows.append({"param": int(param), "ack": ack, "value": val, "ok": bool(ok)})
            if ok and isinstance(val, int):
                source_values[int(param)] = int(val)

        writes = {}
        for key in dest_keys:
            dst = self.motor_function_preset(key)
            if not dst:
                continue
            label = str(dst.get("label", str(key))).upper()
            board = int(dst["board"])
            motor = int(dst["motor"])
            ops = []
            for param in plist:
                if int(param) not in source_values:
                    continue
                setv = int(source_values[int(param)])
                wr = self.motor_set_axis_param(board, int(param), setv, motor=motor)
                ops.append(
                    {
                        "param": int(param),
                        "set": setv,
                        "ack": wr.get("ack"),
                        "readback": wr.get("readback"),
                    }
                )
            writes[label] = {"board": board, "motor": motor, "ops": ops}

        return {
            "source": {"key": str(source_key), "board": src_board, "motor": src_motor},
            "source_rows": source_rows,
            "source_values": source_values,
            "writes": writes,
        }

    def motor_esm_cycle_diag(self, cycles=1):
        """
        Emulate board-test ESM toggling for deck/head, then run quick motion probes.
        """
        t0 = time.time()
        self.reconnect()
        fw_before = {bid: self.board_query_firmware(bid) for bid in self.BOARDS}
        toggles = []
        n = max(1, int(cycles))
        for _ in range(n):
            toggles.append({"phase": "deck_off", **self.board_deactivate(self.BOARD_DECK)})
            toggles.append({"phase": "head_off", **self.board_deactivate(self.BOARD_HEAD)})
            time.sleep(0.12)
            toggles.append({"phase": "deck_on", **self.board_activate(self.BOARD_DECK)})
            toggles.append({"phase": "head_on", **self.board_activate(self.BOARD_HEAD)})
            time.sleep(0.12)

        board_status = self.activate_boards(expect_reply=True)
        interlock = self.motor_prepare_motion_interlock(force_lock=True)
        rail = self.motor_query_24v_sensor()

        out_axes = {}
        steps_map = {"x": 50000, "y": 30000, "z": 20000}
        for key in ("x", "y", "z"):
            p = self.motor_function_preset(key)
            prep = self.motor_prepare_axis(
                p["board"],
                motor=p["motor"],
                run_current=p["run_current"],
                standby_current=p["standby_current"],
                speed=p["speed"],
                acc=p["acc"],
                stall_guard=p.get("stall_guard"),
                ramp_mode=p.get("ramp_mode"),
                disable_right=bool(p.get("disable_right", False)),
                disable_left=bool(p.get("disable_left", False)),
                rdiv=p.get("rdiv"),
                pdiv=p.get("pdiv"),
                warm_enable=bool(p.get("warm_enable", False)),
            )
            probe = self.motor_visible_oneway_probe(
                p["board"],
                steps=int(steps_map.get(key, 20000)),
                motor=p["motor"],
                timeout_s=16.0,
            )
            out_axes[p["label"]] = {"prep": prep, "probe": probe}

        fw_after = {bid: self.board_query_firmware(bid) for bid in self.BOARDS}
        return {
            "fw_before": fw_before,
            "fw_after": fw_after,
            "toggles": toggles,
            "board_status": board_status,
            "interlock": interlock,
            "rail_24v": rail,
            "axes": out_axes,
            "elapsed_ms": int((time.time() - t0) * 1000),
        }

    def motor_restore_oem_xyz_profile(self, run_probe=True):
        """
        Re-apply the OEM-style X/Y/Z profile and run a short directional probe.
        This is the recovery path after experimental tuning/tests.
        """
        t0 = time.time()
        self.reconnect()
        board_status = self.activate_boards(expect_reply=True)
        interlock = self.motor_prepare_motion_interlock(force_lock=True)

        verify_params = (4, 5, 6, 7, 12, 13, 138, 153, 154, 205)
        probe_steps = {"x": 6000, "y": 5000, "z": 4000}
        probe_speed = {"x": 110, "y": 110, "z": 90}
        probe_acc = {"x": 40, "y": 40, "z": 35}
        probe_chunk = {"x": 700, "y": 600, "z": 500}
        probe_current = {"x": 31, "y": 31, "z": 31}
        axes = {}

        for key in ("x", "y", "z"):
            p = self.motor_function_preset(key)
            if not p:
                continue
            board = int(p["board"])
            motor = int(p["motor"])
            prep = self.motor_prepare_axis(
                board,
                motor=motor,
                run_current=int(p["run_current"]),
                standby_current=int(p["standby_current"]),
                speed=int(p["speed"]),
                acc=int(p["acc"]),
                stall_guard=p.get("stall_guard"),
                ramp_mode=p.get("ramp_mode"),
                disable_right=bool(p.get("disable_right", False)),
                disable_left=bool(p.get("disable_left", False)),
                rdiv=p.get("rdiv"),
                pdiv=p.get("pdiv"),
                warm_enable=False,
            )
            verify = self.motor_read_param_set(board, motor=motor, params=verify_params)

            probe_prep = None
            probe = None
            probe_retry = None
            if bool(run_probe):
                sw = self.motor_get_switches(board, motor=motor)
                left = sw.get("left_state")
                right = sw.get("right_state")
                mag = int(probe_steps.get(key, 20000))
                if left == 0 and right == 1:
                    steps = +mag
                elif right == 0 and left == 1:
                    steps = -mag
                else:
                    steps = +mag
                probe_prep = self.motor_prepare_axis(
                    board,
                    motor=motor,
                    run_current=min(int(p.get("run_current", 31)), int(probe_current.get(key, 14))),
                    standby_current=min(int(p.get("run_current", 31)), int(probe_current.get(key, 14))),
                    speed=int(probe_speed.get(key, 220)),
                    acc=int(probe_acc.get(key, 90)),
                    stall_guard=p.get("stall_guard"),
                    ramp_mode=p.get("ramp_mode"),
                    disable_right=bool(p.get("disable_right", False)),
                    disable_left=bool(p.get("disable_left", False)),
                    rdiv=p.get("rdiv"),
                    pdiv=p.get("pdiv"),
                    warm_enable=False,
                )
                probe = self.motor_visible_stepwise_probe(
                    board,
                    steps=int(steps),
                    motor=motor,
                    chunk_steps=int(probe_chunk.get(key, 1500)),
                    timeout_s=10.0,
                    settle_s=0.05,
                )
                moved = bool(isinstance(probe, dict) and int(probe.get("delta") or 0) != 0)
                if not moved:
                    time.sleep(0.05)
                    probe_retry = self.motor_visible_stepwise_probe(
                        board,
                        steps=int(-steps),
                        motor=motor,
                        chunk_steps=int(probe_chunk.get(key, 1500)),
                        timeout_s=10.0,
                        settle_s=0.05,
                    )
                # Leave axis in OEM profile after quiet probing.
                prep = self.motor_prepare_axis(
                    board,
                    motor=motor,
                    run_current=int(p["run_current"]),
                    standby_current=int(p["standby_current"]),
                    speed=int(p["speed"]),
                    acc=int(p["acc"]),
                    stall_guard=p.get("stall_guard"),
                    ramp_mode=p.get("ramp_mode"),
                    disable_right=bool(p.get("disable_right", False)),
                    disable_left=bool(p.get("disable_left", False)),
                    rdiv=p.get("rdiv"),
                    pdiv=p.get("pdiv"),
                    warm_enable=False,
                )
            verify_final = self.motor_read_param_set(board, motor=motor, params=verify_params)

            axes[str(p.get("label", key)).upper()] = {
                "board": board,
                "motor": motor,
                "prep": prep,
                "verify": verify,
                "probe_prep": probe_prep,
                "probe": probe,
                "probe_retry": probe_retry,
                "verify_final": verify_final,
            }

        return {
            "board_status": board_status,
            "interlock": interlock,
            "axes": axes,
            "elapsed_ms": int((time.time() - t0) * 1000),
        }

    @staticmethod
    def _tmcl_success(resp):
        return bool(resp is not None and int(resp.get("status", -1)) == 100)

    @staticmethod
    def _tmcl_success_or_invalid(resp):
        # OEM activateBoard treats status=2 as acceptable on CHILLER boards.
        return bool(resp is not None and int(resp.get("status", -1)) in (100, 2))

    def chiller_activate(self):
        ack = self._send_chiller(
            64,
            0,
            0,
            1,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=60,
            read_timeout_ms=80,
            max_reads=24,
            strict_match=True,
            post_delay_s=self.CHILLER_WRITE_SETTLE_S,
        )
        return {"ack": ack, "ok": self._tmcl_success_or_invalid(ack)}

    def chiller_query_firmware(self):
        ack = self._send_chiller(
            173,
            0,
            0,
            0,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=60,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
        )
        fw_hex = None
        if ack is not None and isinstance(ack.get("raw"), list) and len(ack["raw"]) >= 13:
            fw_hex = "-".join(f"{b:02X}" for b in ack["raw"][9:13])
        return {"ack": ack, "ok": self._tmcl_success(ack), "fw_hex": fw_hex}

    def chiller_read_temp_axis(self, axis):
        ack = self._send_chiller(
            143,
            0,
            int(axis),
            0,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=60,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
        )
        temp_c = (float(ack["value"]) / 1000.0) if self._tmcl_success(ack) else None
        return {"axis": int(axis), "ack": ack, "temp_c": temp_c, "ok": self._tmcl_success(ack)}

    def chiller_read_temps(self):
        out = {}
        for label, axis in self.CHILLER_TEMP_AXES:
            out[label] = self.chiller_read_temp_axis(axis)
        return out

    def chiller_gp_read(self, param, bank):
        param = int(param)
        bank = int(bank)
        if bank not in (self.CHILLER_BANK_RC, self.CHILLER_BANK_OC):
            return {
                "param": param,
                "bank": bank,
                "ack": None,
                "value": None,
                "scaled": None,
                "scale": self.CHILLER_GP_SCALE.get(param),
                "ok": False,
                "error": f"invalid bank {bank} (expected 0=RC or 1=OC)",
            }
        ack = self._send_chiller(
            10,
            param,
            bank,
            0,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=60,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
        )
        value = ack["value"] if self._tmcl_success(ack) else None
        scale = self.CHILLER_GP_SCALE.get(param)
        scaled = (float(value) / float(scale)) if (value is not None and scale) else None
        return {
            "param": param,
            "bank": bank,
            "ack": ack,
            "value": value,
            "scaled": scaled,
            "scale": scale,
            "ok": self._tmcl_success(ack),
        }

    def chiller_gp_write(self, param, bank, value, verify=False):
        param = int(param)
        bank = int(bank)
        value = int(value)
        if bank not in (self.CHILLER_BANK_RC, self.CHILLER_BANK_OC):
            return {"ok": False, "error": f"invalid bank {bank} (expected 0=RC or 1=OC)"}
        ack = self._send_chiller(
            9,
            param,
            bank,
            value,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=65,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
            post_delay_s=self.CHILLER_WRITE_SETTLE_S,
        )
        rb = None
        verified = None
        if verify and self._tmcl_success(ack):
            time.sleep(self.CHILLER_VERIFY_SETTLE_S)
            rb = self.chiller_gp_read(param, bank)
            verified = bool(rb.get("ok") and int(rb.get("value")) == value)
        return {
            "param": param,
            "bank": bank,
            "set_value": value,
            "ack": ack,
            "readback": rb,
            "verified": verified,
            "ok": self._tmcl_success(ack),
        }

    def chiller_set_target_temp(self, bank, temp_c, verify=False):
        bank = int(bank)
        if bank not in self.CHILLER_SET_AXIS:
            return {"ok": False, "error": f"invalid bank {bank} (expected 0=RC or 1=OC)"}
        temp_c = float(temp_c)
        if temp_c < 0.0 or temp_c > 60.0:
            return {"ok": False, "error": f"temp out of guarded range: {temp_c:.3f}C (allowed 0..60C)"}
        axis = self.CHILLER_SET_AXIS[bank]
        raw = int(round(temp_c * 1000.0))
        ack = self._send_chiller(
            140,
            0,
            axis,
            raw,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=65,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
            post_delay_s=self.CHILLER_WRITE_SETTLE_S,
        )
        ref = None
        verified = None
        if verify and self._tmcl_success(ack):
            time.sleep(self.CHILLER_VERIFY_SETTLE_S)
            ref = self.chiller_gp_read(22, bank)
            verified = bool(ref.get("ok") and int(ref.get("value")) == raw)
        return {
            "bank": bank,
            "axis": axis,
            "target_c": temp_c,
            "target_raw": raw,
            "ack": ack,
            "reference": ref,
            "verified": verified,
            "ok": self._tmcl_success(ack),
        }

    def chiller_set_fan(self, bank, speed, verify=False):
        bank = int(bank)
        if bank not in self.CHILLER_SET_AXIS:
            return {"ok": False, "error": f"invalid bank {bank} (expected 0=RC or 1=OC)"}
        speed = int(speed)
        if speed < 0 or speed > 255:
            return {"ok": False, "error": f"fan speed out of range: {speed} (allowed 0..255)"}
        axis = self.CHILLER_SET_AXIS[bank]
        ack = self._send_chiller(
            141,
            0,
            axis,
            speed,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=65,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
            post_delay_s=self.CHILLER_WRITE_SETTLE_S,
        )
        rb = None
        verified = None
        if verify and self._tmcl_success(ack):
            time.sleep(self.CHILLER_VERIFY_SETTLE_S)
            rb = self.chiller_gp_read(21, bank)
            verified = bool(rb.get("ok") and int(rb.get("value")) == speed)
        return {
            "bank": bank,
            "axis": axis,
            "speed": speed,
            "ack": ack,
            "readback": rb,
            "verified": verified,
            "ok": self._tmcl_success(ack),
        }

    def chiller_set_pwm(self, bank, pwm, verify=False):
        bank = int(bank)
        if bank not in self.CHILLER_SET_AXIS:
            return {"ok": False, "error": f"invalid bank {bank} (expected 0=RC or 1=OC)"}
        pwm = int(pwm)
        if pwm < 0 or pwm > 100:
            return {"ok": False, "error": f"pwm out of range: {pwm} (allowed 0..100)"}
        axis = self.CHILLER_SET_AXIS[bank]
        ack = self._send_chiller(
            144,
            0,
            axis,
            pwm,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=65,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
            post_delay_s=self.CHILLER_WRITE_SETTLE_S,
        )
        rb = None
        verified = None
        if verify and self._tmcl_success(ack):
            time.sleep(self.CHILLER_VERIFY_SETTLE_S)
            rb = self.chiller_gp_read(23, bank)
            verified = bool(rb.get("ok") and int(rb.get("value")) == pwm)
        return {
            "bank": bank,
            "axis": axis,
            "pwm": pwm,
            "ack": ack,
            "readback": rb,
            "verified": verified,
            "ok": self._tmcl_success(ack),
        }

    def chiller_set_rates(self, bank, cool_rate_c_s, heat_rate_c_s, verify=False):
        bank = int(bank)
        if bank not in self.CHILLER_SET_AXIS:
            return {"ok": False, "error": f"invalid bank {bank} (expected 0=RC or 1=OC)"}
        cool_rate_c_s = float(cool_rate_c_s)
        heat_rate_c_s = float(heat_rate_c_s)
        if cool_rate_c_s > 0:
            return {"ok": False, "error": f"cool rate must be <= 0 C/s (got {cool_rate_c_s})"}
        if heat_rate_c_s < 0:
            return {"ok": False, "error": f"heat rate must be >= 0 C/s (got {heat_rate_c_s})"}
        if abs(cool_rate_c_s) > 1.0 or abs(heat_rate_c_s) > 1.0:
            return {"ok": False, "error": "rate out of guarded range (allowed magnitude <= 1.0 C/s)"}
        cool_raw = int(round(cool_rate_c_s * 1000.0))
        heat_raw = int(round(heat_rate_c_s * 1000.0))
        cool = self.chiller_gp_write(8, bank, cool_raw, verify=verify)
        heat = self.chiller_gp_write(7, bank, heat_raw, verify=verify)
        verified = None
        if verify:
            verified = bool(cool.get("verified") and heat.get("verified"))
        return {
            "bank": bank,
            "cool_rate_c_s": cool_rate_c_s,
            "heat_rate_c_s": heat_rate_c_s,
            "cool_raw": cool_raw,
            "heat_raw": heat_raw,
            "cool": cool,
            "heat": heat,
            "verified": verified,
            "ok": bool(cool.get("ok") and heat.get("ok")),
        }

    def chiller_apply_vendor_baseline(self, fan_speed=250, cool_rate_c_s=-0.125, heat_rate_c_s=0.125, verify=False):
        out = {"activate": self.chiller_activate(), "banks": {}}
        for bank in (self.CHILLER_BANK_RC, self.CHILLER_BANK_OC):
            rates = self.chiller_set_rates(
                bank,
                cool_rate_c_s=cool_rate_c_s,
                heat_rate_c_s=heat_rate_c_s,
                verify=verify,
            )
            fan = self.chiller_set_fan(bank, fan_speed, verify=verify)
            out["banks"][bank] = {"rates": rates, "fan": fan}
        out["activate_ok"] = bool(out["activate"].get("ok"))
        out["ok"] = bool(all(v["rates"].get("ok") and v["fan"].get("ok") for v in out["banks"].values()))
        return out

    def chiller_snapshot(self, gp_params=None):
        act = self.chiller_activate()
        fw = self.chiller_query_firmware()
        temps = self.chiller_read_temps()
        alive = bool(fw.get("ok") or any(v.get("ok") for v in temps.values()))
        gp = {}
        if gp_params is None:
            gp_params = [2, 3, 7, 8, 13, 21, 22, 23]
        if not alive:
            for bank in (self.CHILLER_BANK_RC, self.CHILLER_BANK_OC):
                gp[bank] = []
            return {"activate": act, "firmware": fw, "temps": temps, "gp": gp, "alive": False}
        for bank in (self.CHILLER_BANK_RC, self.CHILLER_BANK_OC):
            rows = []
            for p in gp_params:
                rows.append(self.chiller_gp_read(p, bank))
            gp[bank] = rows
        return {"activate": act, "firmware": fw, "temps": temps, "gp": gp, "alive": True}

    def _chiller_send_noreply_burst(self, command, cmd_type, motor, value, repeats=3, delay_s=0.08):
        attempted = 0
        sent = 0
        for _ in range(max(1, int(repeats))):
            attempted += 1
            self._chiller_pace(min_gap_s=max(0.02, float(delay_s)))
            r = self.send_tmcl(
                self.BOARD_CHILLER,
                int(command),
                int(cmd_type),
                int(motor),
                int(value),
                wait_reply=False,
                write_timeout_ms=35,
            )
            if r is not None:
                sent += 1
        return {"attempted": attempted, "sent": sent}

    def chiller_hard_reset(self, rounds=3):
        rounds = max(1, int(rounds))
        rows = []
        recovered = False
        for idx in range(rounds):
            self.reconnect()
            time.sleep(0.10)
            self.drain(max_reads=20, timeout_ms=4)

            ops = []
            # Reset board state first.
            ops.append({"name": "deactivate", **self._chiller_send_noreply_burst(64, 0, 0, 0, repeats=2)})
            ops.append({"name": "activate", **self._chiller_send_noreply_burst(64, 0, 0, 1, repeats=2)})

            # Panic-off sequence: fan + PWM + GP mirrors for both banks.
            for axis, bank, label in ((0, 0, "RC"), (1, 1, "OC")):
                ops.append({"name": f"{label} fan off (cmd141)", **self._chiller_send_noreply_burst(141, 0, axis, 0, repeats=2)})
                ops.append({"name": f"{label} pwm off (cmd144)", **self._chiller_send_noreply_burst(144, 0, axis, 0, repeats=2)})
                ops.append({"name": f"{label} gp21=0", **self._chiller_send_noreply_burst(9, 21, bank, 0, repeats=2)})
                ops.append({"name": f"{label} gp23=0", **self._chiller_send_noreply_burst(9, 23, bank, 0, repeats=2)})

            time.sleep(0.10)
            fw = self.chiller_query_firmware()
            temps = self.chiller_read_temps()
            temps_ok = any(v.get("ok") for v in temps.values())
            alive = bool(fw.get("ok") or temps_ok)
            rows.append(
                {
                    "round": idx + 1,
                    "ops": ops,
                    "firmware": fw,
                    "temps": temps,
                    "temps_ok": temps_ok,
                    "alive": alive,
                }
            )
            if alive:
                recovered = True
                break

        return {"ok": recovered, "rounds": rows}

    def thermal_activate(self):
        ack = self._send_thermal(
            64,
            0,
            0,
            1,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=60,
            read_timeout_ms=80,
            max_reads=24,
            strict_match=True,
            post_delay_s=self.THERMAL_WRITE_SETTLE_S,
        )
        return {"ack": ack, "ok": self._tmcl_success_or_invalid(ack)}

    def thermal_query_firmware(self):
        ack = self._send_thermal(
            173,
            0,
            0,
            0,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=60,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
        )
        fw_hex = None
        if ack is not None and isinstance(ack.get("raw"), list) and len(ack["raw"]) >= 13:
            fw_hex = "-".join(f"{b:02X}" for b in ack["raw"][9:13])
        return {"ack": ack, "ok": self._tmcl_success(ack), "fw_hex": fw_hex}

    def thermal_read_temp_axis(self, axis):
        ack = self._send_thermal(
            143,
            0,
            int(axis),
            0,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=60,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
        )
        temp_c = (float(ack["value"]) / 1000.0) if self._tmcl_success(ack) else None
        return {"axis": int(axis), "ack": ack, "temp_c": temp_c, "ok": self._tmcl_success(ack)}

    def thermal_read_temp_gp(self, bank):
        bank = int(bank)
        ack = self._send_thermal(
            10,
            4,
            bank,
            0,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=60,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
        )
        temp_c = (float(ack["value"]) / 1000.0) if self._tmcl_success(ack) else None
        return {"bank": bank, "ack": ack, "temp_c": temp_c, "ok": self._tmcl_success(ack)}

    def thermal_read_temps(self):
        tc = self.thermal_read_temp_gp(self.THERMAL_BANK_NEST)
        lid = self.thermal_read_temp_gp(self.THERMAL_BANK_LID)
        ped = self.thermal_read_temp_axis(2)
        return {"tc_temp_c": tc, "lid_temp_c": lid, "ped_temp_c": ped}

    def thermal_gp_read(self, param, bank):
        param = int(param)
        bank = int(bank)
        if bank not in (self.THERMAL_BANK_NEST, self.THERMAL_BANK_LID):
            return {
                "param": param,
                "bank": bank,
                "ack": None,
                "value": None,
                "scaled": None,
                "scale": self.THERMAL_GP_SCALE.get(param),
                "ok": False,
                "error": f"invalid bank {bank} (expected 0=NEST or 1=LID)",
            }
        ack = self._send_thermal(
            10,
            param,
            bank,
            0,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=60,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
        )
        value = ack["value"] if self._tmcl_success(ack) else None
        scale = self.THERMAL_GP_SCALE.get(param)
        scaled = (float(value) / float(scale)) if (value is not None and scale) else None
        return {
            "param": param,
            "bank": bank,
            "ack": ack,
            "value": value,
            "scaled": scaled,
            "scale": scale,
            "ok": self._tmcl_success(ack),
        }

    def thermal_gp_write(self, param, bank, value, verify=False):
        param = int(param)
        bank = int(bank)
        value = int(value)
        if bank not in (self.THERMAL_BANK_NEST, self.THERMAL_BANK_LID):
            return {"ok": False, "error": f"invalid bank {bank} (expected 0=NEST or 1=LID)"}
        ack = self._send_thermal(
            9,
            param,
            bank,
            value,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=65,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
            post_delay_s=self.THERMAL_WRITE_SETTLE_S,
        )
        rb = None
        verified = None
        if verify and self._tmcl_success(ack):
            time.sleep(self.THERMAL_VERIFY_SETTLE_S)
            rb = self.thermal_gp_read(param, bank)
            verified = bool(rb.get("ok") and int(rb.get("value")) == value)
        return {
            "param": param,
            "bank": bank,
            "set_value": value,
            "ack": ack,
            "readback": rb,
            "verified": verified,
            "ok": self._tmcl_success(ack),
        }

    def thermal_set_target_temp(self, bank, temp_c, verify=False):
        bank = int(bank)
        if bank not in (self.THERMAL_BANK_NEST, self.THERMAL_BANK_LID):
            return {"ok": False, "error": f"invalid bank {bank} (expected 0=NEST or 1=LID)"}
        temp_c = float(temp_c)
        if temp_c < 0.0 or temp_c > 120.0:
            return {"ok": False, "error": f"temp out of guarded range: {temp_c:.3f}C (allowed 0..120C)"}
        raw = int(round(temp_c * 1000.0))
        ack = self._send_thermal(
            140,
            0,
            bank,
            raw,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=65,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
            post_delay_s=self.THERMAL_WRITE_SETTLE_S,
        )
        ref = None
        verified = None
        if verify and self._tmcl_success(ack):
            time.sleep(self.THERMAL_VERIFY_SETTLE_S)
            ref = self.thermal_gp_read(22, bank)
            verified = bool(ref.get("ok") and int(ref.get("value")) == raw)
        return {
            "bank": bank,
            "target_c": temp_c,
            "target_raw": raw,
            "ack": ack,
            "reference": ref,
            "verified": verified,
            "ok": self._tmcl_success(ack),
        }

    def thermal_set_ped_temp(self, temp_c, verify=False):
        temp_c = float(temp_c)
        if temp_c < 0.0 or temp_c > 120.0:
            return {"ok": False, "error": f"pedestal temp out of guarded range: {temp_c:.3f}C (allowed 0..120C)"}
        raw = int(round(temp_c * 1000.0))
        ack = self._send_thermal(
            140,
            0,
            2,
            raw,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=65,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
            post_delay_s=self.THERMAL_WRITE_SETTLE_S,
        )
        rb = None
        verified = None
        if verify and self._tmcl_success(ack):
            time.sleep(self.THERMAL_VERIFY_SETTLE_S)
            rb = self.thermal_read_temp_axis(2)
            if rb.get("temp_c") is not None:
                verified = bool(abs(float(rb["temp_c"]) - temp_c) <= 1.0)
            else:
                verified = False
        return {
            "target_c": temp_c,
            "target_raw": raw,
            "ack": ack,
            "readback": rb,
            "verified": verified,
            "ok": self._tmcl_success(ack),
        }

    def thermal_set_fan(self, speed, verify=False):
        speed = int(speed)
        if speed < 0 or speed > 255:
            return {"ok": False, "error": f"fan speed out of range: {speed} (allowed 0..255)"}
        ack = self._send_thermal(
            141,
            0,
            0,
            speed,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=65,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
            post_delay_s=self.THERMAL_WRITE_SETTLE_S,
        )
        rb = None
        verified = None
        if verify and self._tmcl_success(ack):
            time.sleep(self.THERMAL_VERIFY_SETTLE_S)
            rb = self.thermal_gp_read(21, self.THERMAL_BANK_NEST)
            verified = bool(rb.get("ok") and int(rb.get("value")) == speed)
        return {"speed": speed, "ack": ack, "readback": rb, "verified": verified, "ok": self._tmcl_success(ack)}

    def thermal_set_pwm(self, bank, pwm, verify=False):
        bank = int(bank)
        if bank not in (self.THERMAL_BANK_NEST, self.THERMAL_BANK_LID):
            return {"ok": False, "error": f"invalid bank {bank} (expected 0=NEST or 1=LID)"}
        pwm = int(pwm)
        if pwm < 0 or pwm > 100:
            return {"ok": False, "error": f"pwm out of range: {pwm} (allowed 0..100)"}
        ack = self._send_thermal(
            144,
            0,
            bank,
            pwm,
            attempts=1,
            wait_reply=True,
            write_timeout_ms=65,
            read_timeout_ms=90,
            max_reads=28,
            strict_match=True,
            post_delay_s=self.THERMAL_WRITE_SETTLE_S,
        )
        rb = None
        verified = None
        if verify and self._tmcl_success(ack):
            time.sleep(self.THERMAL_VERIFY_SETTLE_S)
            rb = self.thermal_gp_read(23, bank)
            verified = bool(rb.get("ok") and int(rb.get("value")) == pwm)
        return {"bank": bank, "pwm": pwm, "ack": ack, "readback": rb, "verified": verified, "ok": self._tmcl_success(ack)}

    def thermal_set_rates(self, bank, cool_rate_c_s, heat_rate_c_s, verify=False):
        bank = int(bank)
        if bank not in (self.THERMAL_BANK_NEST, self.THERMAL_BANK_LID):
            return {"ok": False, "error": f"invalid bank {bank} (expected 0=NEST or 1=LID)"}
        cool_rate_c_s = float(cool_rate_c_s)
        heat_rate_c_s = float(heat_rate_c_s)
        if cool_rate_c_s > 0:
            return {"ok": False, "error": f"cool rate must be <= 0 C/s (got {cool_rate_c_s})"}
        if heat_rate_c_s < 0:
            return {"ok": False, "error": f"heat rate must be >= 0 C/s (got {heat_rate_c_s})"}
        if abs(cool_rate_c_s) > 2.0 or abs(heat_rate_c_s) > 2.0:
            return {"ok": False, "error": "rate out of guarded range (allowed magnitude <= 2.0 C/s)"}
        cool_raw = int(round(cool_rate_c_s * 1000.0))
        heat_raw = int(round(heat_rate_c_s * 1000.0))
        cool = self.thermal_gp_write(8, bank, cool_raw, verify=verify)
        heat = self.thermal_gp_write(7, bank, heat_raw, verify=verify)
        verified = None
        if verify:
            verified = bool(cool.get("verified") and heat.get("verified"))
        return {
            "bank": bank,
            "cool_rate_c_s": cool_rate_c_s,
            "heat_rate_c_s": heat_rate_c_s,
            "cool_raw": cool_raw,
            "heat_raw": heat_raw,
            "cool": cool,
            "heat": heat,
            "verified": verified,
            "ok": bool(cool.get("ok") and heat.get("ok")),
        }

    def thermal_apply_vendor_baseline(self, fan_speed=190, cool_rate_c_s=-0.050, heat_rate_c_s=0.050, verify=False):
        out = {"activate": self.thermal_activate(), "banks": {}}
        for bank in (self.THERMAL_BANK_NEST, self.THERMAL_BANK_LID):
            rates = self.thermal_set_rates(
                bank,
                cool_rate_c_s=cool_rate_c_s,
                heat_rate_c_s=heat_rate_c_s,
                verify=verify,
            )
            out["banks"][bank] = {"rates": rates}
        fan = self.thermal_set_fan(fan_speed, verify=verify)
        out["fan"] = fan
        out["activate_ok"] = bool(out["activate"].get("ok"))
        out["ok"] = bool(fan.get("ok") and all(v["rates"].get("ok") for v in out["banks"].values()))
        return out

    def thermal_apply_fast_profile(self, fan_speed=225, cool_rate_c_s=-0.125, heat_rate_c_s=0.125, verify=False):
        return self.thermal_apply_vendor_baseline(
            fan_speed=fan_speed,
            cool_rate_c_s=cool_rate_c_s,
            heat_rate_c_s=heat_rate_c_s,
            verify=verify,
        )

    def thermal_snapshot(self, gp_params=None):
        act = self.thermal_activate()
        fw = self.thermal_query_firmware()
        temps = self.thermal_read_temps()
        alive = bool(fw.get("ok") or any(v.get("ok") for v in temps.values()))
        gp = {}
        if gp_params is None:
            gp_params = [7, 8, 13, 21, 22, 23]
        if not alive:
            for bank in (self.THERMAL_BANK_NEST, self.THERMAL_BANK_LID):
                gp[bank] = []
            return {"activate": act, "firmware": fw, "temps": temps, "gp": gp, "alive": False}
        for bank in (self.THERMAL_BANK_NEST, self.THERMAL_BANK_LID):
            rows = []
            for p in gp_params:
                rows.append(self.thermal_gp_read(p, bank))
            gp[bank] = rows
        return {"activate": act, "firmware": fw, "temps": temps, "gp": gp, "alive": True}

    def _thermal_send_noreply_burst(self, command, cmd_type, motor, value, repeats=3, delay_s=0.08):
        attempted = 0
        sent = 0
        for _ in range(max(1, int(repeats))):
            attempted += 1
            self._thermal_pace(min_gap_s=max(0.02, float(delay_s)))
            r = self.send_tmcl(
                self.BOARD_THERMAL,
                int(command),
                int(cmd_type),
                int(motor),
                int(value),
                wait_reply=False,
                write_timeout_ms=35,
            )
            if r is not None:
                sent += 1
        return {"attempted": attempted, "sent": sent}

    def thermal_hard_reset(self, rounds=3):
        rounds = max(1, int(rounds))
        rows = []
        recovered = False
        for idx in range(rounds):
            self.reconnect()
            time.sleep(0.10)
            self.drain(max_reads=20, timeout_ms=4)

            ops = []
            ops.append({"name": "deactivate", **self._thermal_send_noreply_burst(64, 0, 0, 0, repeats=2)})
            ops.append({"name": "activate", **self._thermal_send_noreply_burst(64, 0, 0, 1, repeats=2)})
            ops.append({"name": "fan off (cmd141 axis0)", **self._thermal_send_noreply_burst(141, 0, 0, 0, repeats=2)})
            for bank, label in ((self.THERMAL_BANK_NEST, "NEST"), (self.THERMAL_BANK_LID, "LID")):
                ops.append({"name": f"{label} pwm off (cmd144)", **self._thermal_send_noreply_burst(144, 0, bank, 0, repeats=2)})
                ops.append({"name": f"{label} gp23=0", **self._thermal_send_noreply_burst(9, 23, bank, 0, repeats=2)})

            time.sleep(0.10)
            fw = self.thermal_query_firmware()
            temps = self.thermal_read_temps()
            temps_ok = any(v.get("ok") for v in temps.values())
            alive = bool(fw.get("ok") or temps_ok)
            rows.append(
                {
                    "round": idx + 1,
                    "ops": ops,
                    "firmware": fw,
                    "temps": temps,
                    "temps_ok": temps_ok,
                    "alive": alive,
                }
            )
            if alive:
                recovered = True
                break

        return {"ok": recovered, "rounds": rows}

    def v4l2_get_ctrl(self, cid, device="/dev/video0"):
        fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        try:
            req = struct.pack("Ii", int(cid), 0)
            out = fcntl.ioctl(fd, self.VIDIOC_G_CTRL, req)
            _, val = struct.unpack("Ii", out)
            return {"ok": True, "cid": int(cid), "value": int(val), "device": device}
        except OSError as e:
            return {"ok": False, "cid": int(cid), "value": None, "device": device, "error": str(e)}
        finally:
            os.close(fd)

    def v4l2_set_ctrl(self, cid, value, device="/dev/video0"):
        fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        try:
            req = struct.pack("Ii", int(cid), int(value))
            fcntl.ioctl(fd, self.VIDIOC_S_CTRL, req)
            got = self.v4l2_get_ctrl(cid, device=device)
            return {
                "ok": True,
                "cid": int(cid),
                "set_value": int(value),
                "readback": got.get("value"),
                "device": device,
            }
        except OSError as e:
            return {
                "ok": False,
                "cid": int(cid),
                "set_value": int(value),
                "readback": None,
                "device": device,
                "error": str(e),
            }
        finally:
            os.close(fd)

    def v4l2_query_ctrl(self, cid, device="/dev/video0"):
        fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        try:
            req = struct.pack("<II32siiiiIII", int(cid), 0, b"\0" * 32, 0, 0, 0, 0, 0, 0, 0)
            out = fcntl.ioctl(fd, self.VIDIOC_QUERYCTRL, req)
            (
                _id,
                ctype,
                name_raw,
                minimum,
                maximum,
                step,
                default,
                flags,
                _r0,
                _r1,
            ) = struct.unpack("<II32siiiiIII", out)
            name = name_raw.split(b"\0", 1)[0].decode("ascii", errors="ignore")
            return {
                "ok": True,
                "cid": int(_id),
                "type": int(ctype),
                "name": name,
                "minimum": int(minimum),
                "maximum": int(maximum),
                "step": int(step),
                "default": int(default),
                "flags": int(flags),
                "device": device,
            }
        except OSError as e:
            return {
                "ok": False,
                "cid": int(cid),
                "device": device,
                "error": str(e),
            }
        finally:
            os.close(fd)

    def camera_devices(self):
        out = []
        for i in range(16):
            dev = f"/dev/video{i}"
            if os.path.exists(dev):
                out.append(dev)
        return out

    def camera_device_rows(self):
        rows = []
        for dev in self.camera_devices():
            base = os.path.basename(dev)
            name_path = f"/sys/class/video4linux/{base}/name"
            name = ""
            try:
                with open(name_path, "r", encoding="utf-8", errors="ignore") as f:
                    name = f.read().strip()
            except OSError:
                name = ""
            rows.append({"device": dev, "name": name})
        return rows

    def camera_pick_device(self, preferred="/dev/video0", require_controls=True):
        rows = self.camera_device_rows()
        if not rows:
            return {"ok": False, "error": "no /dev/video* devices", "rows": []}

        def _score(row):
            dev = row["device"]
            name = row.get("name", "")
            idx = int(dev.replace("/dev/video", "")) if dev.startswith("/dev/video") else 999
            # Prefer the known camera model, then preferred node, then lower index.
            return (
                0 if "SMI: IZONE UVC 5M CAMERA" in name else 1,
                0 if dev == preferred else 1,
                idx,
            )

        ordered = sorted(rows, key=_score)
        if not require_controls:
            best = ordered[0]
            return {"ok": True, "device": best["device"], "name": best.get("name", ""), "rows": ordered}

        for row in ordered:
            dev = row["device"]
            q = self.v4l2_query_ctrl(self.V4L2_CID_BRIGHTNESS, device=dev)
            if q.get("ok"):
                return {"ok": True, "device": dev, "name": row.get("name", ""), "rows": ordered}
            err = str(q.get("error", "")).lower()
            if "device or resource busy" in err or "errno 16" in err:
                # Busy is still potentially the correct node.
                return {"ok": True, "device": dev, "name": row.get("name", ""), "rows": ordered}

        best = ordered[0]
        return {"ok": True, "device": best["device"], "name": best.get("name", ""), "rows": ordered}

    def camera_wait_pick_device(self, preferred="/dev/video0", timeout_s=8.0, poll_s=0.25, require_controls=False):
        deadline = time.monotonic() + max(0.2, float(timeout_s))
        last = {"ok": False, "error": "timeout waiting for camera device"}
        while time.monotonic() < deadline:
            pick = self.camera_pick_device(preferred=preferred, require_controls=require_controls)
            if pick.get("ok"):
                return pick
            last = pick
            time.sleep(max(0.05, float(poll_s)))
        return last

    def camera_probe_controls(self, device="/dev/video0"):
        controls = [
            ("brightness", self.V4L2_CID_BRIGHTNESS),
            ("contrast", self.V4L2_CID_CONTRAST),
            ("saturation", self.V4L2_CID_SATURATION),
            ("gain", self.V4L2_CID_GAIN),
            ("backlight", self.V4L2_CID_BACKLIGHT_COMPENSATION),
            ("exposure_auto", self.V4L2_CID_EXPOSURE_AUTO),
            ("exposure_abs", self.V4L2_CID_EXPOSURE_ABSOLUTE),
            ("focus_auto", self.V4L2_CID_FOCUS_AUTO),
            ("focus_abs", self.V4L2_CID_FOCUS_ABSOLUTE),
        ]
        rows = []
        for name, cid in controls:
            q = self.v4l2_query_ctrl(cid, device=device)
            g = self.v4l2_get_ctrl(cid, device=device)
            rows.append(
                {
                    "name": name,
                    "cid": cid,
                    "query": q,
                    "get": g,
                }
            )
        return rows

    def camera_enumerate_controls(self, device="/dev/video0", include_disabled=False, max_items=256):
        fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        rows = []
        seen = set()
        try:
            qid = self.V4L2_CTRL_FLAG_NEXT_CTRL
            while len(rows) < max_items:
                req = struct.pack("<II32siiiiIII", int(qid), 0, b"\0" * 32, 0, 0, 0, 0, 0, 0, 0)
                try:
                    out = fcntl.ioctl(fd, self.VIDIOC_QUERYCTRL, req)
                except OSError as e:
                    if e.errno in (22, 25):
                        break
                    return {"ok": False, "device": device, "rows": rows, "error": str(e)}

                (
                    cid,
                    ctype,
                    name_raw,
                    minimum,
                    maximum,
                    step,
                    default,
                    flags,
                    _r0,
                    _r1,
                ) = struct.unpack("<II32siiiiIII", out)
                if cid in seen:
                    break
                seen.add(cid)
                qid = int(cid) | self.V4L2_CTRL_FLAG_NEXT_CTRL
                if (flags & self.V4L2_CTRL_FLAG_DISABLED) and not include_disabled:
                    continue
                name = name_raw.split(b"\0", 1)[0].decode("ascii", errors="ignore")
                g = self.v4l2_get_ctrl(cid, device=device)
                rows.append(
                    {
                        "cid": int(cid),
                        "type": int(ctype),
                        "name": name,
                        "minimum": int(minimum),
                        "maximum": int(maximum),
                        "step": int(step),
                        "default": int(default),
                        "flags": int(flags),
                        "get": g,
                    }
                )
            return {"ok": True, "device": device, "rows": rows}
        finally:
            os.close(fd)

    def uvc_xu_query(self, unit, selector, query, size, device="/dev/video0", data=None):
        if int(size) <= 0:
            return {
                "ok": False,
                "unit": int(unit),
                "selector": int(selector),
                "query": int(query),
                "size": int(size),
                "device": device,
                "error": "size must be > 0",
            }

        try:
            fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        except OSError as e:
            return {
                "ok": False,
                "unit": int(unit),
                "selector": int(selector),
                "query": int(query),
                "size": int(size),
                "device": device,
                "error": str(e),
            }
        try:
            n = int(size)
            buf = (ctypes.c_uint8 * n)()
            if data is not None:
                raw = bytes(data)[:n]
                for i, b in enumerate(raw):
                    buf[i] = int(b)

            q = UvcXuControlQuery(
                unit=ctypes.c_uint8(int(unit)),
                selector=ctypes.c_uint8(int(selector)),
                query=ctypes.c_uint8(int(query)),
                size=ctypes.c_uint16(n),
                data=ctypes.cast(buf, ctypes.c_void_p),
            )

            ctypes.set_errno(0)
            ret = self._libc.ioctl(fd, self.UVCIOC_CTRL_QUERY, ctypes.byref(q))
            if ret != 0:
                err = ctypes.get_errno()
                return {
                    "ok": False,
                    "unit": int(unit),
                    "selector": int(selector),
                    "query": int(query),
                    "size": n,
                    "device": device,
                    "error": f"[Errno {err}] {os.strerror(err)}",
                }

            out = bytes(buf[:n])
            return {
                "ok": True,
                "unit": int(unit),
                "selector": int(selector),
                "query": int(query),
                "size": n,
                "device": device,
                "data": list(out),
                "data_hex": out.hex(),
            }
        finally:
            os.close(fd)

    def camera_xu_sweep(self, device="/dev/video0", unit_min=1, unit_max=16, sel_min=1, sel_max=32):
        rows = []
        for unit in range(int(unit_min), int(unit_max) + 1):
            for selector in range(int(sel_min), int(sel_max) + 1):
                q_len = self.uvc_xu_query(unit, selector, self.UVC_GET_LEN, 2, device=device)
                if not q_len["ok"]:
                    continue
                b = q_len.get("data", [])
                if len(b) != 2:
                    continue
                payload_len = int(b[0]) | (int(b[1]) << 8)
                if payload_len <= 0 or payload_len > 1024:
                    continue

                q_info = self.uvc_xu_query(unit, selector, self.UVC_GET_INFO, 1, device=device)
                info_val = q_info["data"][0] if q_info["ok"] and q_info.get("data") else None
                q_cur = self.uvc_xu_query(
                    unit,
                    selector,
                    self.UVC_GET_CUR,
                    min(payload_len, 64),
                    device=device,
                )
                rows.append(
                    {
                        "unit": unit,
                        "selector": selector,
                        "len": payload_len,
                        "info": info_val,
                        "cur": q_cur,
                        "can_set": bool(info_val is not None and (info_val & 0x01)),
                        "can_get": bool(info_val is not None and (info_val & 0x02)),
                    }
                )
        return {"ok": True, "device": device, "rows": rows}

    def uvc_xu_get_len(self, unit, selector, device="/dev/video0"):
        q_len = self.uvc_xu_query(unit, selector, self.UVC_GET_LEN, 2, device=device)
        if not q_len["ok"]:
            return {"ok": False, "error": q_len.get("error"), "len": None}
        b = q_len.get("data", [])
        if len(b) != 2:
            return {"ok": False, "error": f"bad GET_LEN payload: {b}", "len": None}
        payload_len = int(b[0]) | (int(b[1]) << 8)
        if payload_len <= 0:
            return {"ok": False, "error": f"invalid len: {payload_len}", "len": None}
        return {"ok": True, "len": payload_len}

    @staticmethod
    def _is_timeout_error(resp):
        if not resp or resp.get("ok"):
            return False
        e = str(resp.get("error", "")).lower()
        return ("timed out" in e) or ("errno 110" in e)

    def camera_xu_auto_safe_sweep(self, device="/dev/video0", settle_s=0.05, max_len=16, controls=None):
        if controls is None:
            sweep = self.camera_xu_sweep(device=device)
            all_controls = [r for r in sweep.get("rows", []) if r.get("can_set")]
        else:
            all_controls = [r for r in controls if r.get("can_set")]
        controls = [r for r in all_controls if int(r.get("len", 0)) <= int(max_len)]
        skipped = [r for r in all_controls if int(r.get("len", 0)) > int(max_len)]
        rows = []
        for ctrl in controls:
            unit = int(ctrl["unit"])
            selector = int(ctrl["selector"])
            plen = int(ctrl["len"])

            base = self.uvc_xu_query(unit, selector, self.UVC_GET_CUR, plen, device=device)

            if plen == 1:
                patterns = ([0x00], [0x01])
            elif plen == 2:
                # Conservative 2-byte toggles first; this camera reports len=2.
                patterns = ([0x00, 0x00], [0x01, 0x00], [0x02, 0x00], [0x00, 0x01], [0x01, 0x01])
            else:
                z = [0x00] * plen
                o = [0x00] * plen
                o[0] = 0x01
                patterns = (z, o)

            if not base.get("ok"):
                # If baseline read already times out, keep tests minimal to avoid long waits.
                patterns = patterns[:1]

            tests = []
            for payload in patterns:
                set_r = self.uvc_xu_query(
                    unit,
                    selector,
                    self.UVC_SET_CUR,
                    plen,
                    device=device,
                    data=payload,
                )
                time.sleep(settle_s)
                get_r = self.uvc_xu_query(
                    unit,
                    selector,
                    self.UVC_GET_CUR,
                    plen,
                    device=device,
                )
                tests.append(
                    {
                        "payload": list(payload),
                        "payload_hex": bytes(payload).hex(),
                        "set": set_r,
                        "get": get_r,
                    }
                )
                if self._is_timeout_error(set_r) and self._is_timeout_error(get_r):
                    break

            restore_set = None
            restore_get = None
            if base.get("ok"):
                restore_set = self.uvc_xu_query(
                    unit,
                    selector,
                    self.UVC_SET_CUR,
                    plen,
                    device=device,
                    data=base.get("data", []),
                )
                time.sleep(settle_s)
                restore_get = self.uvc_xu_query(
                    unit,
                    selector,
                    self.UVC_GET_CUR,
                    plen,
                    device=device,
                )

            rows.append(
                {
                    "unit": unit,
                    "selector": selector,
                    "len": plen,
                    "baseline": base,
                    "tests": tests,
                    "restore_set": restore_set,
                    "restore_get": restore_get,
                }
            )

        return {"ok": True, "device": device, "rows": rows, "skipped": skipped}

    @staticmethod
    def _ffmpeg_available():
        return shutil.which("ffmpeg") is not None

    @staticmethod
    def _start_ffmpeg_keepalive(device="/dev/video0"):
        if shutil.which("ffmpeg") is None:
            return None
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "v4l2",
            "-i",
            device,
            "-f",
            "null",
            "-",
        ]
        try:
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(0.20)
            return proc
        except Exception:
            return None

    @staticmethod
    def _stop_process(proc):
        if proc is None:
            return
        try:
            proc.terminate()
            proc.wait(timeout=1.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass

    @staticmethod
    def camera_usb_sysfs_paths(device="/dev/video0"):
        dev_name = os.path.basename(device)
        class_path = f"/sys/class/video4linux/{dev_name}/device"
        if not os.path.exists(class_path):
            return {"ok": False, "error": f"missing {class_path}"}
        intf_path = os.path.realpath(class_path)
        base_name = os.path.basename(intf_path)
        if ":" in base_name:
            usb_dev_path = os.path.join(os.path.dirname(intf_path), base_name.split(":", 1)[0])
        else:
            usb_dev_path = intf_path
        auth_path = os.path.join(usb_dev_path, "authorized")
        return {
            "ok": True,
            "device": device,
            "interface_path": intf_path,
            "usb_device_path": usb_dev_path,
            "authorized_path": auth_path,
        }

    @staticmethod
    def camera_usb_candidates_by_product(product_substr="IZONE UVC 5M CAMERA"):
        roots = []
        base = "/sys/bus/usb/devices"
        if not os.path.exists(base):
            return roots
        try:
            names = os.listdir(base)
        except OSError:
            return roots
        for name in names:
            p = os.path.join(base, name)
            prod = os.path.join(p, "product")
            auth = os.path.join(p, "authorized")
            if not os.path.exists(prod) or not os.path.exists(auth):
                continue
            try:
                with open(prod, "r", encoding="utf-8", errors="ignore") as f:
                    txt = f.read().strip()
            except OSError:
                continue
            if product_substr.lower() in txt.lower():
                roots.append({"usb_device_path": p, "authorized_path": auth, "product": txt})
        return roots

    def camera_usb_soft_reset_by_product(self, product_substr="IZONE UVC 5M CAMERA", wait_s=2.0):
        cands = self.camera_usb_candidates_by_product(product_substr=product_substr)
        if not cands:
            return {"ok": False, "error": f"no USB candidates matching '{product_substr}'", "candidates": []}
        touched = []
        for c in cands:
            auth = c["authorized_path"]
            try:
                with open(auth, "w", encoding="ascii") as f:
                    f.write("0\n")
                touched.append(c)
            except OSError:
                pass
        time.sleep(0.35)
        for c in touched:
            auth = c["authorized_path"]
            try:
                with open(auth, "w", encoding="ascii") as f:
                    f.write("1\n")
            except OSError:
                pass
        time.sleep(max(0.2, float(wait_s)))
        return {"ok": bool(touched), "candidates": cands, "touched": touched}

    @staticmethod
    def _pid_alive(pid):
        try:
            os.kill(int(pid), 0)
            return True
        except OSError:
            return False

    @staticmethod
    def camera_release_busy_owners(device="/dev/video0"):
        if shutil.which("fuser") is None:
            return {"ok": False, "error": "fuser not found", "pids": [], "killed": []}
        try:
            p = subprocess.run(
                ["fuser", device],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=1.5,
                check=False,
            )
        except Exception as e:
            return {"ok": False, "error": str(e), "pids": [], "killed": []}

        text = f"{p.stdout}\n{p.stderr}"
        pids = sorted({int(x) for x in re.findall(r"\b\d+\b", text)})
        this_pid = os.getpid()
        pids = [pid for pid in pids if pid != this_pid]
        killed = []
        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
                killed.append(pid)
            except OSError:
                pass
        time.sleep(0.25)
        for pid in list(killed):
            if BioXpTester._pid_alive(pid):
                try:
                    os.kill(pid, signal.SIGKILL)
                except OSError:
                    pass
        alive = [pid for pid in pids if BioXpTester._pid_alive(pid)]
        return {
            "ok": True,
            "pids": pids,
            "killed": killed,
            "alive": alive,
        }

    def camera_usb_soft_reset(self, device="/dev/video0", wait_s=2.0):
        info = self.camera_usb_sysfs_paths(device=device)
        if not info.get("ok"):
            return info
        auth = info["authorized_path"]
        if not os.path.exists(auth):
            return {"ok": False, "error": f"missing {auth}", **info}
        try:
            with open(auth, "w", encoding="ascii") as f:
                f.write("0\n")
            time.sleep(0.35)
            with open(auth, "w", encoding="ascii") as f:
                f.write("1\n")
        except OSError as e:
            return {"ok": False, "error": str(e), **info}

        deadline = time.monotonic() + max(0.2, float(wait_s))
        while time.monotonic() < deadline:
            if os.path.exists(device):
                return {"ok": True, **info}
            time.sleep(0.10)
        return {"ok": False, "error": f"{device} did not reappear after reset", **info}

    @staticmethod
    def camera_measure_signalstats(device="/dev/video0", timeout_s=2.0):
        if shutil.which("ffmpeg") is None:
            return {"ok": False, "error": "ffmpeg not found"}
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "info",
            "-f",
            "v4l2",
            "-i",
            device,
            "-frames:v",
            "1",
            "-vf",
            "signalstats,metadata=mode=print",
            "-f",
            "null",
            "-",
        ]
        try:
            p = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=timeout_s,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return {"ok": False, "error": "ffmpeg timeout"}
        text = p.stdout or ""
        y = re.search(r"lavfi\.signalstats\.YAVG=([0-9.]+)", text)
        sat = re.search(r"lavfi\.signalstats\.SATAVG=([0-9.]+)", text)
        out = {
            "ok": y is not None,
            "yavg": float(y.group(1)) if y else None,
            "satavg": float(sat.group(1)) if sat else None,
            "rc": int(p.returncode),
        }
        if not out["ok"]:
            out["error"] = "signalstats parse failed"
        return out

    def camera_capture_snapshot(self, preferred="/dev/video0", out_dir="/home/molbiofreak/BioXP-Work/output/camera"):
        if shutil.which("ffmpeg") is None:
            return {"ok": False, "error": "ffmpeg not found"}
        pick = self.camera_wait_pick_device(
            preferred=preferred,
            timeout_s=8.0,
            poll_s=0.25,
            require_controls=False,
        )
        if not pick.get("ok"):
            return {"ok": False, "error": pick.get("error"), "device": None, "path": None}
        device = pick["device"]
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(out_dir, f"snapshot_{ts}.jpg")
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "v4l2",
            "-i",
            device,
            "-frames:v",
            "1",
            "-update",
            "1",
            out_path,
        ]
        try:
            p = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=8.0,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return {"ok": False, "error": "ffmpeg snapshot timeout", "device": device, "path": out_path}
        exists = os.path.exists(out_path)
        size = os.path.getsize(out_path) if exists else 0
        ok = (p.returncode == 0) and exists and size > 0
        return {
            "ok": ok,
            "device": device,
            "path": out_path,
            "size": size,
            "rc": int(p.returncode),
            "output": p.stdout.strip() if p.stdout else "",
            "error": None if ok else "snapshot failed",
        }

    def camera_stream_health(self, preferred="/dev/video0", seconds=5):
        if shutil.which("ffmpeg") is None:
            return {"ok": False, "error": "ffmpeg not found", "device": None}
        pick = self.camera_wait_pick_device(
            preferred=preferred,
            timeout_s=8.0,
            poll_s=0.25,
            require_controls=False,
        )
        if not pick.get("ok"):
            return {"ok": False, "error": pick.get("error"), "device": None}
        device = pick["device"]
        dur = max(1, int(seconds))
        cmd = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "info",
            "-f",
            "v4l2",
            "-i",
            device,
            "-t",
            str(dur),
            "-f",
            "null",
            "-",
        ]
        try:
            p = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=float(dur) + 6.0,
                check=False,
            )
        except subprocess.TimeoutExpired:
            return {"ok": False, "error": "ffmpeg stream timeout", "device": device}
        txt = p.stdout or ""
        frames = [int(x) for x in re.findall(r"frame=\s*([0-9]+)", txt)]
        fps_vals = [float(x) for x in re.findall(r"fps=\s*([0-9.]+)", txt)]
        frame_max = max(frames) if frames else 0
        fps_last = fps_vals[-1] if fps_vals else None
        ok = (p.returncode == 0) and frame_max > 0
        return {
            "ok": ok,
            "device": device,
            "seconds": dur,
            "frames": frame_max,
            "fps_last": fps_last,
            "rc": int(p.returncode),
            "error": None if ok else "stream health failed",
            "output_tail": "\n".join((txt.strip().splitlines()[-8:] if txt.strip() else [])),
        }

    def camera_prepare_measurement_lock(self, device="/dev/video0"):
        # Keep image statistics stable while sweeping vendor controls.
        prev = {}
        for name, cid in (
            ("exposure_auto", self.V4L2_CID_EXPOSURE_AUTO),
            ("exposure_abs", self.V4L2_CID_EXPOSURE_ABSOLUTE),
            ("gain", self.V4L2_CID_GAIN),
        ):
            g = self.v4l2_get_ctrl(cid, device=device)
            prev[name] = g.get("value") if g.get("ok") else None

        ops = []
        ops.append(("set exposure_auto=manual", self.v4l2_set_ctrl(self.V4L2_CID_EXPOSURE_AUTO, 1, device=device)))
        # Use known-stable defaults for this camera.
        ops.append(("set exposure_abs=1250", self.v4l2_set_ctrl(self.V4L2_CID_EXPOSURE_ABSOLUTE, 1250, device=device)))
        ops.append(("set gain=3", self.v4l2_set_ctrl(self.V4L2_CID_GAIN, 3, device=device)))

        ok = all(r.get("ok") for _, r in ops)
        return {"ok": ok, "prev": prev, "ops": ops}

    def camera_restore_measurement_lock(self, prev, device="/dev/video0"):
        ops = []
        if not isinstance(prev, dict):
            return {"ok": False, "ops": [], "error": "bad prev state"}
        if prev.get("exposure_abs") is not None:
            ops.append(("restore exposure_abs", self.v4l2_set_ctrl(self.V4L2_CID_EXPOSURE_ABSOLUTE, int(prev["exposure_abs"]), device=device)))
        if prev.get("gain") is not None:
            ops.append(("restore gain", self.v4l2_set_ctrl(self.V4L2_CID_GAIN, int(prev["gain"]), device=device)))
        if prev.get("exposure_auto") is not None:
            ops.append(("restore exposure_auto", self.v4l2_set_ctrl(self.V4L2_CID_EXPOSURE_AUTO, int(prev["exposure_auto"]), device=device)))
        ok = all(r.get("ok") for _, r in ops) if ops else True
        return {"ok": ok, "ops": ops}

    def camera_xu_selector_profile(
        self,
        unit=2,
        selector=1,
        device="/dev/video0",
        settle_s=0.06,
        with_signalstats=True,
    ):
        len_info = self.uvc_xu_get_len(unit, selector, device=device)
        if not len_info["ok"]:
            return {"ok": False, "error": f"GET_LEN failed: {len_info.get('error')}"}
        plen = int(len_info["len"])
        if plen <= 0 or plen > 512:
            return {"ok": False, "error": f"unsupported len from GET_LEN: {plen}"}
        if plen > 16:
            return {
                "ok": False,
                "error": (
                    f"selector currently reports len={plen} (unexpected high). "
                    "Likely vendor command mode; run camera USB reset + XU rescan first."
                ),
            }

        def _mk_payload(v0, v1):
            p = [0x00] * plen
            p[0] = int(v0) & 0xFF
            if plen > 1:
                p[1] = int(v1) & 0xFF
            return p

        raw_pairs = [
            (0x00, 0x00),
            (0x01, 0x00),
            (0x02, 0x00),
            (0x03, 0x00),
            (0x04, 0x00),
            (0x05, 0x00),
            (0x06, 0x00),
            (0x07, 0x00),
            (0x08, 0x00),
            (0x10, 0x00),
            (0x20, 0x00),
            (0x40, 0x00),
            (0x80, 0x00),
            (0xFF, 0x00),
            (0x00, 0x01),
            (0x01, 0x01),
        ]
        patterns = []
        seen = set()
        for v0, v1 in raw_pairs:
            p = _mk_payload(v0, v1)
            k = tuple(p)
            if k in seen:
                continue
            seen.add(k)
            patterns.append(p)

        base = self.uvc_xu_query(unit, selector, self.UVC_GET_CUR, plen, device=device)
        rows = []
        timeout_get_streak = 0
        for payload in patterns:
            set_r = self.uvc_xu_query(
                unit,
                selector,
                self.UVC_SET_CUR,
                plen,
                device=device,
                data=payload,
            )
            time.sleep(settle_s)
            get_r = self.uvc_xu_query(unit, selector, self.UVC_GET_CUR, plen, device=device)
            should_stats = with_signalstats and set_r.get("ok") and get_r.get("ok")
            stats = self.camera_measure_signalstats(device=device) if should_stats else None
            rows.append(
                {
                    "payload": payload,
                    "payload_hex": bytes(payload).hex(),
                    "set": set_r,
                    "get": get_r,
                    "stats": stats,
                }
            )
            if self._is_timeout_error(get_r):
                timeout_get_streak += 1
            else:
                timeout_get_streak = 0
            if self._is_timeout_error(set_r) and self._is_timeout_error(get_r):
                break
            if (not base.get("ok")) and timeout_get_streak >= 2:
                break

        restore_set = None
        restore_get = None
        if base.get("ok"):
            restore_set = self.uvc_xu_query(
                unit,
                selector,
                self.UVC_SET_CUR,
                plen,
                device=device,
                data=base.get("data", []),
            )
            time.sleep(settle_s)
            restore_get = self.uvc_xu_query(unit, selector, self.UVC_GET_CUR, plen, device=device)

        return {
            "ok": True,
            "device": device,
            "unit": int(unit),
            "selector": int(selector),
            "len": plen,
            "baseline": base,
            "rows": rows,
            "restore_set": restore_set,
            "restore_get": restore_get,
            "with_signalstats": bool(with_signalstats),
            "len_note": "dynamic" if plen != 2 else "classic",
        }

    def camera_auto_oneclick(self, device="/dev/video0", max_resets=2):
        log = []
        pick = self.camera_wait_pick_device(preferred=device, timeout_s=8.0, poll_s=0.25, require_controls=False)
        if not pick.get("ok"):
            log.append(f"initial pick failed: {pick.get('error')}")
            rr = self.camera_usb_soft_reset_by_product("IZONE UVC 5M CAMERA", wait_s=1.0)
            if rr.get("ok"):
                log.append(
                    "fallback usb reset by product: "
                    + ", ".join(c.get("usb_device_path", "?") for c in rr.get("touched", []))
                )
            else:
                log.append(f"fallback usb reset failed: {rr.get('error')}")
            pick = self.camera_wait_pick_device(
                preferred=device,
                timeout_s=12.0,
                poll_s=0.25,
                require_controls=False,
            )
            if not pick.get("ok"):
                rows = self.camera_device_rows()
                rowtxt = ", ".join(f"{r['device']}:{r.get('name','')}" for r in rows) if rows else "(none)"
                return {
                    "ok": False,
                    "error": f"{device} not found after wait+fallback",
                    "log": log + [f"visible video nodes: {rowtxt}"],
                }
        device = pick["device"]
        if pick.get("name"):
            log.append(f"resolved camera device: {device} ({pick.get('name')})")
        else:
            log.append(f"resolved camera device: {device}")

        chosen = None
        sweep_rows = []
        busy_seen = False
        for i in range(int(max_resets) + 1):
            probe = self.uvc_xu_query(2, 1, self.UVC_GET_LEN, 2, device=device)
            probe_err = str(probe.get("error", "")).lower() if not probe.get("ok") else ""
            if "device or resource busy" in probe_err or "errno 16" in probe_err:
                busy_seen = True
                log.append(f"sweep#{i+1}: camera busy on {device}")
                rel = self.camera_release_busy_owners(device=device)
                if rel.get("ok"):
                    log.append(
                        f"sweep#{i+1}: released owners pids={rel.get('pids', [])} "
                        f"alive_after={rel.get('alive', [])}"
                    )
                else:
                    log.append(f"sweep#{i+1}: release owners failed: {rel.get('error')}")
                if i < int(max_resets):
                    rr = self.camera_usb_soft_reset(device=device)
                    if rr.get("ok"):
                        log.append(f"reset#{i+1}: ok usb={rr.get('usb_device_path')}")
                        pick = self.camera_wait_pick_device(
                            preferred=device,
                            timeout_s=8.0,
                            poll_s=0.25,
                            require_controls=False,
                        )
                        if pick.get("ok"):
                            device = pick["device"]
                            log.append(f"reset#{i+1}: re-enumerated as {device}")
                        else:
                            log.append(f"reset#{i+1}: device wait failed ({pick.get('error')})")
                            return {
                                "ok": False,
                                "error": f"{device} missing after reset",
                                "log": log,
                                "sweep_rows": sweep_rows,
                            }
                        continue
                    log.append(f"reset#{i+1}: fail {rr.get('error')}")
                    return {
                        "ok": False,
                        "error": "camera busy recovery reset failed",
                        "log": log,
                        "sweep_rows": sweep_rows,
                    }
            sweep = self.camera_xu_sweep(device=device)
            rows = sweep.get("rows", [])
            sweep_rows = rows
            small = [
                r
                for r in rows
                if r.get("can_set") and r.get("can_get") and int(r.get("len", 0)) <= 16
            ]
            high = [r for r in rows if int(r.get("len", 0)) > 16]
            probed = []
            for r in small:
                u = int(r["unit"])
                s = int(r["selector"])
                l = int(r["len"])
                g = self.uvc_xu_query(u, s, self.UVC_GET_CUR, min(l, 64), device=device)
                rp = dict(r)
                rp["probe_cur"] = g
                rp["probe_ok"] = bool(g.get("ok"))
                probed.append(rp)
            readable = [r for r in probed if r.get("probe_ok")]
            log.append(
                f"sweep#{i+1}: total={len(rows)} small_writable={len(small)} "
                f"readable={len(readable)} high_len={len(high)} "
                f"lens={','.join(str(int(r.get('len', 0))) for r in rows[:8])}"
            )
            if readable:
                chosen = None
                for r in readable:
                    if int(r["unit"]) == 2 and int(r["selector"]) == 1:
                        chosen = r
                        break
                if chosen is None:
                    chosen = sorted(readable, key=lambda r: (int(r["unit"]), int(r["selector"])))[0]
                break

            if i < int(max_resets):
                # High-length mode (e.g., len=79) usually means protocol state drift; reset and retry.
                rr = self.camera_usb_soft_reset(device=device)
                if rr.get("ok"):
                    log.append(f"reset#{i+1}: ok usb={rr.get('usb_device_path')}")
                    pick = self.camera_wait_pick_device(
                        preferred=device,
                        timeout_s=8.0,
                        poll_s=0.25,
                        require_controls=False,
                    )
                    if pick.get("ok"):
                        device = pick["device"]
                        log.append(f"reset#{i+1}: re-enumerated as {device}")
                    else:
                        log.append(f"reset#{i+1}: device wait failed ({pick.get('error')})")
                        return {
                            "ok": False,
                            "error": f"{device} missing after reset",
                            "log": log,
                            "sweep_rows": sweep_rows,
                        }
                else:
                    log.append(f"reset#{i+1}: fail {rr.get('error')}")
                    return {
                        "ok": False,
                        "error": "camera reset failed during auto flow",
                        "log": log,
                        "sweep_rows": sweep_rows,
                    }

        if chosen is None:
            return {
                "ok": False,
                "error": (
                    f"{device} remained busy (another process owns camera)"
                    if busy_seen
                    else "no readable writable XU controls found after auto recovery"
                ),
                "log": log,
                "sweep_rows": sweep_rows,
            }

        log.append(
            f"chosen: unit={int(chosen['unit'])} sel={int(chosen['selector'])} "
            f"len={int(chosen['len'])} cur={short_hex(chosen.get('probe_cur', {}).get('data_hex', 'n/a'))}"
        )
        safe = self.camera_xu_auto_safe_sweep(
            device=device,
            settle_s=0.05,
            max_len=16,
            controls=[chosen],
        )
        lock = self.camera_prepare_measurement_lock(device=device)
        log.append(
            "measurement lock: "
            + ("ok" if lock.get("ok") else "partial/fail")
        )
        profile = self.camera_xu_selector_profile(
            unit=int(chosen["unit"]),
            selector=int(chosen["selector"]),
            device=device,
            with_signalstats=self._ffmpeg_available(),
        )
        restore = self.camera_restore_measurement_lock(lock.get("prev"), device=device)
        log.append("measurement restore: " + ("ok" if restore.get("ok") else "partial/fail"))
        if not profile.get("ok"):
            log.append(f"profile#1 failed: {profile.get('error')}")
            rr = self.camera_usb_soft_reset(device=device)
            if rr.get("ok"):
                log.append("profile recovery reset: ok")
                profile = self.camera_xu_selector_profile(
                    unit=int(chosen["unit"]),
                    selector=int(chosen["selector"]),
                    device=device,
                    with_signalstats=self._ffmpeg_available(),
                )
            else:
                log.append(f"profile recovery reset: fail {rr.get('error')}")
        return {
            "ok": True,
            "device": device,
            "chosen": chosen,
            "safe": safe,
            "profile": profile,
            "lock": lock,
            "restore": restore,
            "log": log,
        }

    @staticmethod
    def camera_smi_status():
        base = "/home/molbiofreak/BioXP_Original_Backup/BioXP_SSD_Backup/.deploy/Application Files/GenBotApp_6_3_0_1"
        files = [
            "CVisionLib.dll.deploy",
            "SMIUtility.dll.deploy",
            "SGI_Util.dll.deploy",
        ]
        rows = []
        for f in files:
            p = f"{base}/{f}"
            rows.append({"file": p, "exists": os.path.exists(p)})
        return rows


def fmt_resp(r):
    if r is None:
        return "no resp"
    return r.get("status_str", "resp")


def parse_hex_bytes(text):
    raw = text.strip().lower().replace(",", " ")
    if raw.startswith("0x") and " " not in raw:
        raw = raw[2:]
    parts = [p for p in raw.replace("0x", "").split() if p]
    if len(parts) == 1 and all(c in "0123456789abcdef" for c in parts[0]) and len(parts[0]) % 2 == 0:
        parts = [parts[0][i : i + 2] for i in range(0, len(parts[0]), 2)]
    out = []
    for p in parts:
        if not p or any(c not in "0123456789abcdef" for c in p):
            return None
        v = int(p, 16)
        if v < 0 or v > 255:
            return None
        out.append(v)
    return out if out else None


def parse_rgb_triplet(text):
    parts = [p for p in str(text).strip().lower().replace(",", " ").split() if p]
    if len(parts) != 3:
        return None
    vals = []
    for p in parts:
        try:
            v = int(p, 0)
        except ValueError:
            return None
        if v < 0 or v > 255:
            return None
        vals.append(v)
    return tuple(vals)


LED_COLOR_PRESETS = [
    ("OFF", (0, 0, 0)),
    ("WHITE", (255, 255, 255)),
    ("WARM WHITE", (255, 214, 170)),
    ("COOL WHITE", (220, 235, 255)),
    ("RED", (255, 0, 0)),
    ("ORANGE", (255, 128, 0)),
    ("AMBER", (255, 191, 0)),
    ("YELLOW", (255, 255, 0)),
    ("LIME", (128, 255, 0)),
    ("GREEN", (0, 255, 0)),
    ("MINT", (0, 255, 128)),
    ("CYAN", (0, 255, 255)),
    ("SKY BLUE", (102, 178, 255)),
    ("BLUE", (0, 0, 255)),
    ("INDIGO", (75, 0, 130)),
    ("VIOLET", (143, 0, 255)),
    ("PURPLE", (180, 0, 255)),
    ("MAGENTA", (255, 0, 255)),
    ("PINK", (255, 105, 180)),
    ("ROSE", (255, 0, 127)),
]


def short_hex(hex_text, limit=40):
    s = str(hex_text)
    if len(s) <= int(limit):
        return s
    tail = max(8, int(limit) // 4)
    head = max(8, int(limit) - tail - 3)
    return s[:head] + "..." + s[-tail:]


def print_status(tester):
    st, snap = tester.status_with_recovery()
    print(f"Board status (alt={tester.alt}):")
    for bid in tester.BOARDS:
        print(f"  0x{bid:02X}: {fmt_resp(st.get(bid))}")
    print(f"Deck IO snapshot (0x05): {snap}")


def fmt_ack_map(acks):
    parts = []
    for bid in sorted(acks):
        parts.append(f"0x{bid:02X}:{fmt_resp(acks[bid])}")
    return " ".join(parts)


def fmt_rgb_acks(acks):
    return f"R={fmt_resp(acks.get('r'))} G={fmt_resp(acks.get('g'))} B={fmt_resp(acks.get('b'))}"


def _persist_led_choice(tester, rgb, preset=None):
    saved = tester.save_led_state(rgb[0], rgb[1], rgb[2], preset=preset)
    if saved.get("ok"):
        st = saved.get("state") or {}
        name = st.get("preset") or "custom"
        print(f"Persisted default: preset={name} rgb={st.get('rgb')}")
    else:
        print(f"Persist warning: {saved.get('error')}")


def _print_deck_io_matrix(matrix, label="deck io"):
    rows = matrix.get("rows", []) if isinstance(matrix, dict) else []
    print(f"{label}:")
    if not rows:
        print("  no rows")
        return
    for row in rows:
        print(
            f"  type={row.get('type')}: ack={fmt_resp(row.get('ack'))} "
            f"value={row.get('value')}"
        )


def _print_head_lock_probe(out):
    if not isinstance(out, dict):
        print("No probe output.")
        return
    print(f"head lock probe elapsed={out.get('elapsed_ms')}ms")
    probe_ok = out.get("ok")
    if probe_ok is True:
        print("  result=PASS")
    elif probe_ok is False:
        print(
            f"  result=FAIL error_code={out.get('error_code')} "
            f"aborted={out.get('aborted')}"
        )
    rb = out.get("rail_before", {})
    ra = out.get("rail_after", {})
    print(
        f"  24V before={rb.get('raw')}({fmt_resp(rb.get('ack'))}) "
        f"after={ra.get('raw')}({fmt_resp(ra.get('ack'))})"
    )
    print(f"  io before={out.get('io_before')} after={out.get('io_after')}")
    for ph in out.get("phases", []):
        st = ph.get("state")
        wr = ph.get("write", {})
        snap = ph.get("snapshot", {})
        rail = ph.get("rail_24v", {})
        pstat = "OK" if ph.get("ok") else f"FAIL(code={ph.get('error_code')})"
        print(
            f"[{st}] {pstat} write ack={fmt_resp(wr.get('ack'))} "
            f"set(type={wr.get('type')},value={wr.get('value_set')}) "
            f"io={snap} 24V={rail.get('raw')}"
        )
        if ph.get("ok") is not True:
            for perr in ph.get("errors", []):
                print(
                    f"    phase-error axis={perr.get('axis')} "
                    f"code={perr.get('error_code')}"
                )
        for key in ("y_probe", "z_probe"):
            pr = ph.get(key, {})
            ax = pr.get("axis")
            ts = pr.get("test", {})
            d1 = ts.get("delta_fwd")
            d2 = ts.get("delta_back")
            dn = ts.get("delta_net")
            swb = pr.get("switch_before", {})
            swm = ts.get("mid_switches", {}) if isinstance(ts.get("mid_switches"), dict) else {}
            swa = pr.get("switch_after", {})
            v = pr.get("validation", {}) if isinstance(pr.get("validation"), dict) else {}
            print(
                f"  {ax}: step={pr.get('step_cmd')} "
                f"fwd={d1} back={d2} net={dn} "
                f"ack={fmt_resp(ts.get('fwd', {}).get('ack'))}/{fmt_resp(ts.get('back', {}).get('ack'))} "
                f"sw_before(L={swb.get('left_state')} R={swb.get('right_state')}) "
                f"sw_mid(L={swm.get('left_state')} R={swm.get('right_state')}) "
                f"sw_after(L={swa.get('left_state')} R={swa.get('right_state')})"
            )
            if v.get("ok") is not True:
                first = v.get("errors", [{}])[0]
                print(
                    f"    axis-error code={v.get('error_code')} "
                    f"key={first.get('key')} detail={first.get('detail')}"
                )


def run_latch_menu(tester):
    while True:
        print("\n----------------------------------------")
        print(" LATCH CONTROL")
        print("----------------------------------------")
        print("  1. LOCK")
        print("  2. UNLOCK")
        print("  3. PULSE (lock -> unlock)")
        print("  4. DECK IO QUERY SWEEP (cmd15 types 0..7)")
        print("  5. SOLENOID/HEAD LOCK PROBE (lock<->unlock vs Y/Z)")
        print("  6. DECK SIO PROBE (cmd14 type2 with verify)")
        print("  b. Back")
        print("  q. Quit")

        ans = input("\n[LATCH] > ").strip().lower()
        if ans == "":
            continue
        if ans == "b":
            return
        if ans == "q":
            sys.exit(0)
        if ans == "1":
            r = tester.latch_oneshot(True, broad=False)
            print(f"LOCK: sent={r['sent']} broad={r['broad']} time={r['elapsed_ms']}ms")
        elif ans == "2":
            r = tester.latch_oneshot(False, broad=False)
            print(f"UNLOCK: sent={r['sent']} broad={r['broad']} time={r['elapsed_ms']}ms")
        elif ans == "3":
            r1 = tester.latch_oneshot(True, broad=False)
            time.sleep(0.20)
            r2 = tester.latch_oneshot(False, broad=False)
            print(f"PULSE LOCK: sent={r1['sent']} broad={r1['broad']} time={r1['elapsed_ms']}ms")
            print(f"PULSE UNLOCK: sent={r2['sent']} broad={r2['broad']} time={r2['elapsed_ms']}ms")
        elif ans == "4":
            out = tester.deck_io_query_matrix(io_types=(0, 1, 2, 3, 4, 5, 6, 7))
            _print_deck_io_matrix(out, label="deck io query sweep")
        elif ans == "5":
            yin = input("Y micro-step magnitude (blank=1000): ").strip()
            zin = input("Z micro-step magnitude (blank=500): ").strip()
            sin = input("probe speed (blank=260): ").strip()
            ain = input("probe acc (blank=120): ").strip()
            try:
                ymag = int(yin) if yin != "" else 1000
                zmag = int(zin) if zin != "" else 500
                spd = int(sin) if sin != "" else 260
                acc = int(ain) if ain != "" else 120
            except ValueError:
                print("Invalid numeric input.")
                continue
            out = tester.head_lock_actuator_probe(
                y_steps=max(100, abs(ymag)),
                z_steps=max(100, abs(zmag)),
                speed=max(50, abs(spd)),
                acc=max(20, abs(acc)),
                fail_fast=True,
            )
            _print_head_lock_probe(out)
        elif ans == "6":
            out = tester.deck_io_sio_probe(
                set_types=(2,),
                values=(0, 1),
                verify_types=(0, 1, 2, 3),
                settle_s=0.20,
            )
            print(f"deck SIO probe elapsed={out.get('elapsed_ms')}ms")
            _print_deck_io_matrix(out.get("baseline", {}), label="baseline")
            rb = out.get("baseline_24v", {})
            print(f"  baseline 24V: raw={rb.get('raw')} ack={fmt_resp(rb.get('ack'))}")
            for row in out.get("rows", []):
                wr = row.get("write", {})
                print(
                    f"set type={row.get('set_type')} value={row.get('set_value')}: "
                    f"ack={fmt_resp(wr.get('ack'))}"
                )
                _print_deck_io_matrix(row.get("verify", {}), label="  verify")
                rv = row.get("rail_24v", {})
                print(f"  verify 24V: raw={rv.get('raw')} ack={fmt_resp(rv.get('ack'))}")
            _print_deck_io_matrix(out.get("final", {}), label="final")
            rf = out.get("final_24v", {})
            print(f"  final 24V: raw={rf.get('raw')} ack={fmt_resp(rf.get('ack'))}")
        else:
            print("Invalid.")


def _motor_steps_default(label):
    lab = str(label).upper()
    if lab == "X":
        return 50000
    if lab == "Z":
        # Z is head vertical (up/down); keep default nudge conservative.
        return 3000
    if lab == "Y":
        return 30000
    if lab == "GRIPPER":
        return 700
    if lab == "THERMAL_DOOR":
        return 500
    return 1200


def _motor_visible_steps_default(label):
    lab = str(label).upper()
    if lab == "X":
        return 350000
    if lab == "Y":
        return 280000
    if lab == "Z":
        # Z is head vertical (up/down); large visible runs are risky near deck.
        return 25000
    if lab == "GRIPPER":
        return 4000
    if lab == "THERMAL_DOOR":
        return 8000
    return 50000


def _print_motor_axis_status(tester, label, board, motor):
    st = tester.motor_axis_status(board, motor=motor)
    pos = st["position"].get("position")
    spd = st["speed"].get("speed")
    cur = st["max_current"].get("value")
    left = st["switches"].get("left_state")
    right = st["switches"].get("right_state")
    left_active = None if left is None else (int(left) == 0)
    right_active = None if right is None else (int(right) == 0)
    print(
        f"  {label}: board=0x{int(board):02X} motor={int(motor)} "
        f"pos={pos} speed={spd} cur={cur} "
        f"left={left}(active={left_active}) right={right}(active={right_active})"
    )


def _print_motor_step_result(label, out):
    print(
        f"{label}: board=0x{int(out['board']):02X} motor={int(out['motor'])} "
        f"steps={int(out['steps'])}"
    )
    print(
        f"  fwd={fmt_resp(out['fwd'].get('ack'))} "
        f"wait={out['wait_fwd'].get('stopped')}({out['wait_fwd'].get('elapsed_ms')}ms) "
        f"delta={out.get('delta_fwd')}"
    )
    print(
        f"  back={fmt_resp(out['back'].get('ack'))} "
        f"wait={out['wait_back'].get('stopped')}({out['wait_back'].get('elapsed_ms')}ms) "
        f"delta={out.get('delta_back')} net={out.get('delta_net')}"
    )
    print(
        f"  pos: pre={out['pre'].get('position')} mid={out['mid'].get('position')} "
        f"post={out['post'].get('position')}"
    )
    pre_sw = out.get("pre_switches", {})
    mid_sw = out.get("mid_switches", {})
    post_sw = out.get("post_switches", {})
    print(
        "  switches:"
        f" pre(L={pre_sw.get('left_state')} R={pre_sw.get('right_state')})"
        f" mid(L={mid_sw.get('left_state')} R={mid_sw.get('right_state')})"
        f" post(L={post_sw.get('left_state')} R={post_sw.get('right_state')})"
    )
    val = out.get("validation", {}) if isinstance(out.get("validation"), dict) else None
    if isinstance(val, dict):
        if val.get("ok") is True:
            print("  validation: PASS")
        else:
            first = val.get("errors", [{}])[0] if isinstance(val.get("errors"), list) else {}
            print(
                f"  validation: FAIL code={val.get('error_code')} "
                f"key={first.get('key')} detail={first.get('detail')}"
            )


def _print_motor_prep_ops(prep):
    for op in prep.get("ops", []):
        setv = op.get("set")
        rb = op.get("rb")
        if str(op.get("op", "")).startswith("norm-"):
            axis = None if not isinstance(rb, dict) else rb.get("axis")
            req = None if not isinstance(rb, dict) else rb.get("requested")
            lo = None if not isinstance(rb, dict) else rb.get("min")
            hi = None if not isinstance(rb, dict) else rb.get("max")
            print(
                f"  {op.get('op')}: axis={axis} requested={req} clamped={setv} "
                f"range=[{lo}..{hi}]"
            )
            continue
        if rb:
            print(
                f"  {op.get('op')}: ack={fmt_resp(op.get('ack'))} "
                f"set={setv} readback={rb.get('value')}"
            )
        else:
            print(f"  {op.get('op')}: ack={fmt_resp(op.get('ack'))}")


def _print_motion_interlock(inter):
    if not isinstance(inter, dict):
        return
    rail = inter.get("rail_24v", {})
    no24 = rail.get("no24v")
    if no24 is None:
        rail_state = "unknown"
    else:
        rail_state = "NO_24V" if no24 else "OK"
    print(
        f"  interlock: 24V={rail_state} raw={rail.get('raw')} "
        f"ack={fmt_resp(rail.get('ack'))} time={inter.get('elapsed_ms')}ms"
    )
    sb = inter.get("snap_before", {})
    sa = inter.get("snap_after", {})
    if sb or sa:
        print(f"  io before={sb} after={sa}")
    latch = inter.get("latch")
    if isinstance(latch, dict):
        ack = latch.get("ack")
        snap = latch.get("snapshot")
        print(f"  latch lock: ack={fmt_resp(ack)} snapshot={snap}")
    for op in inter.get("ops", []):
        rb = op.get("rb")
        print(
            f"  {op.get('axis')} {op.get('op')}: "
            f"ack={fmt_resp(op.get('ack'))} set={op.get('set')} "
            f"readback={None if rb is None else rb.get('value')}"
        )


def _print_motion_gate_status(tester, refresh=True, snapshot=None):
    st = tester.motion_arm_state()
    ov = tester.motion_latch_override_state()
    print(
        f"motion arm: armed={st.get('armed')} reason={st.get('reason')} "
        f"note={st.get('note')} seq={st.get('arm_seq')}"
    )
    print(
        f"latch override: enabled={ov.get('enabled')} "
        f"reason={ov.get('reason')} note={ov.get('note')} seq={ov.get('seq')}"
    )
    live = snapshot if isinstance(snapshot, dict) else None
    if live is None and bool(refresh):
        live = tester.motion_gate_assert_live(auto_disarm=bool(st.get("armed")))
    if not isinstance(live, dict):
        return
    rail = live.get("rail_24v", {})
    checks = live.get("checks", {})
    print(
        f"  live gate: ok={live.get('ok')} errors={live.get('error_keys')} "
        f"24V_raw={rail.get('raw')} no24v={rail.get('no24v')}"
    )
    print(f"  live io: {live.get('io')}")
    print(
        "  checks:"
        f" rail_ok={checks.get('rail_ok')}"
        f" io_read_ok={checks.get('io_read_ok')}"
        f" door_closed={checks.get('door_closed')}"
        f" latch_closed={checks.get('latch_closed')}"
        f" solenoid_locked={checks.get('solenoid_locked')}"
        f" latch_override_active={checks.get('latch_override_active')}"
    )


def _print_strict_startup_report(out):
    if not isinstance(out, dict):
        print("No strict startup report.")
        return
    print(
        f"strict startup init: ok={out.get('ok')} "
        f"error_code={out.get('error_code')} "
        f"time={out.get('elapsed_ms')}ms"
    )
    st = out.get("board_status", {})
    print(
        "board status:"
        f" HEAD={fmt_resp(st.get(BioXpTester.BOARD_HEAD))}"
        f" DECK={fmt_resp(st.get(BioXpTester.BOARD_DECK))}"
        f" THERMAL={fmt_resp(st.get(BioXpTester.BOARD_THERMAL))}"
    )
    for key in ("pre_gate", "post_lock_gate", "final_gate"):
        g = out.get(key, {})
        rail = g.get("rail_24v", {})
        print(
            f"  {key}: ok={g.get('ok')} errors={g.get('error_keys')} "
            f"io={g.get('io')} 24V_raw={rail.get('raw')} no24v={rail.get('no24v')}"
        )
    print("checks:")
    for row in out.get("checks", []):
        if row.get("ok"):
            tag = "PASS"
        elif bool(row.get("critical", True)):
            tag = "FAIL"
        else:
            tag = "WARN"
        print(f"  {tag} {row.get('name')}: {row.get('detail')}")
    h = out.get("homing")
    if isinstance(h, dict):
        for label, key in (("Z", "z_home"), ("G", "g_home"), ("X", "x_home"), ("Y", "y_home"), ("DOOR", "door_home")):
            hr = h.get(key, {})
            print(
                f"  home {label}: move={fmt_resp(hr.get('move_left', {}).get('ack'))} "
                f"wait={hr.get('wait', {}).get('stopped')} "
                f"set={fmt_resp(hr.get('sethome_final', {}).get('ack'))} "
                f"home_after={hr.get('home_after', {}).get('value')}"
            )
    arm = out.get("arm_state", {})
    print(
        f"arm state: armed={arm.get('armed')} reason={arm.get('reason')} "
        f"note={arm.get('note')} seq={arm.get('arm_seq')}"
    )


def _print_driver_diag_summary(out):
    if not isinstance(out, dict):
        print("No driver diagnostic output.")
        return
    print(f"driver diag done in {out.get('elapsed_ms')}ms")
    print("board status:")
    for bid in BioXpTester.BOARDS:
        print(f"  0x{bid:02X}: {fmt_resp(out.get('board_status', {}).get(bid))}")
    _print_motion_interlock(out.get("interlock"))
    rail = out.get("rail_24v", {})
    no24 = rail.get("no24v")
    rail_state = "unknown" if no24 is None else ("NO_24V" if no24 else "OK")
    print(f"24V sensor (deck cmd15/type0): raw={rail.get('raw')} state={rail_state} ack={fmt_resp(rail.get('ack'))}")

    print("axis status snapshot:")
    for label, st in out.get("statuses", {}).items():
        pos = st.get("position", {}).get("position")
        spd = st.get("speed", {}).get("speed")
        cur = st.get("max_current", {}).get("value")
        sw = st.get("switches", {})
        print(
            f"  {label}: pos={pos} speed={spd} cur={cur} "
            f"L={sw.get('left_state')} R={sw.get('right_state')}"
        )

    keys = ["X", "Y", "Z", "GRIPPER", "THERMAL_DOOR"]
    focus_params = [6, 7, 12, 13, 138, 153, 154, 170, 178, 204, 205]
    print("param compare (GAP):")
    for param in focus_params:
        vals = out.get("matrix", {}).get(int(param), {})
        parts = []
        for k in keys:
            if k in vals:
                parts.append(f"{k}={vals.get(k)}")
        if parts:
            print(f"  p{param}: " + " | ".join(parts))
    print("tip: if X/Y/Z differ strongly from GRIPPER on p170/p204/p138, run profile copy test.")


def _print_driver_clone_result(out):
    if not isinstance(out, dict):
        print("No clone output.")
        return
    if out.get("error"):
        print(f"clone error: {out.get('error')}")
        return
    src = out.get("source", {})
    print(
        f"clone source: key={src.get('key')} board=0x{int(src.get('board', 0)):02X} "
        f"motor={int(src.get('motor', 0))}"
    )
    have = sorted(out.get("source_values", {}).keys())
    print(f"  captured params: {have if have else '(none)'}")
    for label, row in out.get("writes", {}).items():
        print(f"  target {label} board=0x{int(row.get('board', 0)):02X} motor={int(row.get('motor', 0))}")
        ops = row.get("ops", [])
        if not ops:
            print("    no params written")
            continue
        ok = 0
        for op in ops:
            if fmt_resp(op.get("ack")) == "Success":
                ok += 1
        print(f"    writes ok={ok}/{len(ops)}")
        for op in ops[:8]:
            rb = op.get("readback")
            print(
                f"    p{op.get('param')} set={op.get('set')} "
                f"ack={fmt_resp(op.get('ack'))} rb={None if rb is None else rb.get('value')}"
            )
        if len(ops) > 8:
            print(f"    ... {len(ops) - 8} more")


def _print_esm_cycle_diag(out):
    if not isinstance(out, dict):
        print("No ESM diagnostic output.")
        return
    print(f"esm cycle diag done in {out.get('elapsed_ms')}ms")
    print("firmware before:")
    for bid in BioXpTester.BOARDS:
        row = out.get("fw_before", {}).get(bid, {})
        print(f"  0x{bid:02X}: ack={fmt_resp(row.get('ack'))} fw={row.get('fw_hex')}")
    print("toggle acks:")
    for row in out.get("toggles", []):
        print(f"  {row.get('phase')}: 0x{int(row.get('board', 0)):02X} {fmt_resp(row.get('ack'))}")
    print("board status:")
    for bid in BioXpTester.BOARDS:
        print(f"  0x{bid:02X}: {fmt_resp(out.get('board_status', {}).get(bid))}")
    _print_motion_interlock(out.get("interlock"))
    rail = out.get("rail_24v", {})
    no24 = rail.get("no24v")
    rail_state = "unknown" if no24 is None else ("NO_24V" if no24 else "OK")
    print(f"24V sensor (deck cmd15/type0): raw={rail.get('raw')} state={rail_state} ack={fmt_resp(rail.get('ack'))}")
    for label, row in out.get("axes", {}).items():
        probe = row.get("probe", {})
        evs = probe.get("events", [])
        stalls = probe.get("stall_events", [])
        print(
            f"{label} probe: ack={fmt_resp(probe.get('move', {}).get('ack'))} "
            f"delta={probe.get('delta')} stopped={probe.get('wait', {}).get('stopped')} "
            f"time={probe.get('wait', {}).get('elapsed_ms')}ms "
            f"L={probe.get('post_switches', {}).get('left_state')} "
            f"R={probe.get('post_switches', {}).get('right_state')} "
            f"events={len(evs)} stall={len(stalls)}"
        )
    print("firmware after:")
    for bid in BioXpTester.BOARDS:
        row = out.get("fw_after", {}).get(bid, {})
        print(f"  0x{bid:02X}: ack={fmt_resp(row.get('ack'))} fw={row.get('fw_hex')}")


def _print_gentle_creep_result(out):
    if not isinstance(out, dict):
        print("No creep output.")
        return
    if out.get("error"):
        print(f"creep error: {out.get('error')}")
        return
    print(
        f"{out.get('axis')} gentle creep: board=0x{int(out.get('board', 0)):02X} "
        f"motor={int(out.get('motor', 0))} steps={int(out.get('steps', 0))}"
    )
    inter = out.get("interlock")
    if isinstance(inter, dict):
        _print_motion_interlock(inter)
    for tr in out.get("trials", []):
        cfg = tr.get("cfg", {})
        sm = tr.get("switch_mid", {})
        sp = tr.get("switch_post", {})
        print(
            f"  T{int(tr.get('idx', 0))}: cur={cfg.get('run_current')} "
            f"spd={cfg.get('speed')} acc={cfg.get('acc')} "
            f"fwd={fmt_resp(tr.get('move_fwd', {}).get('ack'))}/"
            f"{tr.get('wait_fwd', {}).get('elapsed_ms')}ms "
            f"back={fmt_resp(tr.get('move_back', {}).get('ack'))}/"
            f"{tr.get('wait_back', {}).get('elapsed_ms')}ms "
            f"delta_f={tr.get('delta_fwd')} delta_b={tr.get('delta_back')} net={tr.get('delta_net')} "
            f"sw_mid(L={sm.get('left_state')} R={sm.get('right_state')}) "
            f"sw_post(L={sp.get('left_state')} R={sp.get('right_state')})"
        )


def _print_oem_restore_result(out):
    if not isinstance(out, dict):
        print("No OEM restore output.")
        return
    print(f"oem xyz restore done in {out.get('elapsed_ms')}ms")
    print("board status:")
    for bid in BioXpTester.BOARDS:
        print(f"  0x{bid:02X}: {fmt_resp(out.get('board_status', {}).get(bid))}")
    _print_motion_interlock(out.get("interlock"))
    for label, row in out.get("axes", {}).items():
        print(f"{label}: board=0x{int(row.get('board', 0)):02X} motor={int(row.get('motor', 0))}")
        verify = row.get("verify", {})
        part = []
        for param in (6, 7, 12, 13, 138, 153, 154, 205):
            v = verify.get(param, {})
            part.append(f"p{param}={v.get('value')}")
        if part:
            print("  verify(oem): " + " ".join(part))
        vfin = row.get("verify_final", {})
        part_fin = []
        for param in (6, 7, 12, 13, 138, 153, 154, 205):
            v = vfin.get(param, {})
            part_fin.append(f"p{param}={v.get('value')}")
        if part_fin:
            print("  verify(final): " + " ".join(part_fin))
        probe = row.get("probe")
        if isinstance(probe, dict):
            segs = probe.get("segments", [])
            seg_desc = []
            for seg in segs[:4]:
                seg_desc.append(
                    f"{seg.get('idx')}:{seg.get('step')}=>{seg.get('delta')}"
                )
            if len(segs) > 4:
                seg_desc.append(f"...{len(segs)-4} more")
            print(
                f"  probe: req={probe.get('requested_steps')} "
                f"delta={probe.get('delta')} "
                f"segs={len(segs)} "
                f"sw(L={probe.get('post_switches', {}).get('left_state')} "
                f"R={probe.get('post_switches', {}).get('right_state')}) "
                f"path=[{' '.join(seg_desc)}]"
            )
        retry = row.get("probe_retry")
        if isinstance(retry, dict):
            segs = retry.get("segments", [])
            seg_desc = []
            for seg in segs[:4]:
                seg_desc.append(
                    f"{seg.get('idx')}:{seg.get('step')}=>{seg.get('delta')}"
                )
            if len(segs) > 4:
                seg_desc.append(f"...{len(segs)-4} more")
            print(
                f"  retry: req={retry.get('requested_steps')} "
                f"delta={retry.get('delta')} segs={len(segs)} "
                f"sw(L={retry.get('post_switches', {}).get('left_state')} "
                f"R={retry.get('post_switches', {}).get('right_state')}) "
                f"path=[{' '.join(seg_desc)}]"
            )


def _motor_home_speed_default(label):
    lab = str(label).upper()
    if lab in ("X", "Y"):
        return 250
    if lab == "Z":
        return 1791
    if lab == "GRIPPER":
        return 600
    if lab == "THERMAL_DOOR":
        return 600
    return 250


def _print_motor_home_result(label, out):
    if not isinstance(out, dict) or not out:
        print(f"{label} home: no data")
        return
    board = out.get("board")
    motor = out.get("motor")
    speed = out.get("speed")
    btxt = f"0x{int(board):02X}" if isinstance(board, int) else "?"
    mtxt = str(int(motor)) if isinstance(motor, int) else "?"
    stxt = str(int(speed)) if isinstance(speed, int) else "?"
    print(
        f"{label} home: board={btxt} motor={mtxt} "
        f"speed={stxt} time={out.get('elapsed_ms')}ms"
    )
    print(
        f"  move_left={fmt_resp(out.get('move_left', {}).get('ack'))} "
        f"stop={fmt_resp(out.get('stop', {}).get('ack'))} "
        f"set_home={fmt_resp(out.get('sethome_final', {}).get('ack'))}"
    )
    hb = out.get("home_after", {}).get("value")
    hh = out.get("home_hit", {}).get("value") if isinstance(out.get("home_hit"), dict) else None
    pa = out.get("position_after", {}).get("position")
    print(f"  home_hit={hh} home_after={hb} pos_after={pa}")


def run_motor_channel_menu(tester, label, board, motor, preset=None):
    speed = int(preset.get("speed", 800)) if preset else 800
    acc = int(preset.get("acc", 300)) if preset else 300
    run_current = int(preset.get("run_current", 20)) if preset else 20
    standby_current = int(preset.get("standby_current", 10)) if preset else 10
    stall_guard = preset.get("stall_guard") if preset else None
    disable_right = preset.get("disable_right") if preset and ("disable_right" in preset) else None
    disable_left = preset.get("disable_left") if preset and ("disable_left" in preset) else None
    ramp_mode = preset.get("ramp_mode") if preset and ("ramp_mode" in preset) else None
    rdiv = preset.get("rdiv") if preset else None
    pdiv = preset.get("pdiv") if preset else None
    warm_enable = bool(preset.get("warm_enable", False)) if preset else False
    nudge_steps = _motor_steps_default(label)
    visible_steps = _motor_visible_steps_default(label)
    home_speed = _motor_home_speed_default(label)
    axis_name = str(label).upper()
    is_gantry_axis = (
        axis_name in ("X", "Y", "Z")
        or (int(board), int(motor)) in (
            (int(tester.BOARD_DECK), 0),
            (int(tester.BOARD_HEAD), 0),
            (int(tester.BOARD_HEAD), 1),
        )
    )
    norms = tester.motor_speed_acc_norms_for_channel(board, motor=motor)
    interlock_ready = False
    if is_gantry_axis:
        # Start manual probing in a conservative envelope to reduce chatter/vibration.
        speed = min(int(speed), 320)
        acc = min(int(acc), 140)

    def _prepare_for_motion(force_interlock=False):
        nonlocal interlock_ready
        if is_gantry_axis:
            ov = tester.motion_latch_override_state()
            ov_enabled = bool(ov.get("enabled"))
            if force_interlock or not interlock_ready:
                inter = tester.motor_prepare_motion_interlock(force_lock=not ov_enabled)
                _print_motion_interlock(inter)
                if ov_enabled:
                    unl = tester.latch_oem(False)
                    print(
                        f"  latch override unlock: ack={fmt_resp(unl.get('ack'))} "
                        f"snapshot={unl.get('snapshot')}"
                    )
                interlock_ready = True
            else:
                rail = tester.motor_query_24v_sensor()
                no24 = rail.get("no24v")
                if no24 is None:
                    rail_state = "unknown"
                else:
                    rail_state = "NO_24V" if no24 else "OK"
                print(
                    f"  interlock: reuse wake/lock, 24V={rail_state} raw={rail.get('raw')} "
                    f"ack={fmt_resp(rail.get('ack'))}"
                )
        prep = tester.motor_prepare_axis(
            board,
            motor=motor,
            run_current=run_current,
            standby_current=standby_current,
            speed=speed,
            acc=acc,
            stall_guard=stall_guard,
            ramp_mode=ramp_mode,
            disable_right=disable_right,
            disable_left=disable_left,
            rdiv=rdiv,
            pdiv=pdiv,
            warm_enable=warm_enable,
        )
        _print_motor_prep_ops(prep)
        return prep

    def _require_gantry_motion_gate(op_label):
        if not is_gantry_axis:
            return True
        st = tester.motion_arm_state()
        if not bool(st.get("armed")):
            code = int(tester.MOTION_ERROR_CODES.get("MOTION_NOT_ARMED", 9203))
            print(
                f"{op_label}: blocked error_code={code} "
                f"reason={st.get('reason')} note={st.get('note')}"
            )
            _print_motion_gate_status(tester, refresh=True)
            return False
        live = tester.motion_gate_assert_live(auto_disarm=True)
        if not bool(live.get("ok")):
            code_key = "MOTION_GATE_IO_UNAVAILABLE" if "io_unavailable" in set(live.get("error_keys", [])) else "MOTION_SENSOR_GATE_FAILED"
            code = int(tester.MOTION_ERROR_CODES.get(code_key, 9204))
            print(
                f"{op_label}: blocked error_code={code} "
                f"sensor_errors={','.join(live.get('error_keys', []))}"
            )
            _print_motion_gate_status(tester, refresh=False, snapshot=live)
            return False
        return True

    def _handle_runtime_motion_validation(op_label, validation):
        if not isinstance(validation, dict):
            return True
        if validation.get("ok") is True:
            return True
        first = validation.get("errors", [{}])[0] if isinstance(validation.get("errors"), list) else {}
        print(
            f"{op_label}: FAIL error_code={validation.get('error_code')} "
            f"key={first.get('key')} detail={first.get('detail')}"
        )
        if is_gantry_axis:
            st = tester.motion_disarm(
                reason="runtime_motion_fault",
                note=f"{axis_name}:{first.get('key')}:{validation.get('error_code')}",
            )
            print(
                f"motion arm disarmed: armed={st.get('armed')} "
                f"reason={st.get('reason')} note={st.get('note')} seq={st.get('arm_seq')}"
            )
        return False

    while True:
        print("\n----------------------------------------")
        print(" MOTOR CHANNEL")
        print("----------------------------------------")
        print(
            f"  target: {label} board=0x{int(board):02X} "
            f"({tester.board_label(board)}), motor={int(motor)}"
        )
        print(
            f"  defaults: speed={speed} acc={acc} run_current={run_current} "
            f"standby={standby_current} nudge={nudge_steps} visible={visible_steps} "
            f"home_speed={home_speed} stall_guard={stall_guard} "
            f"ramp={ramp_mode} sw_dis(R={disable_right} L={disable_left})"
        )
        if isinstance(norms, dict):
            print(
                f"  speed/acc norms: speed={int(norms['speed_min'])}..{int(norms['speed_max'])} "
                f"(OEM {int(norms['speed_oem'])}), "
                f"acc={int(norms['acc_min'])}..{int(norms['acc_max'])} "
                f"(OEM {int(norms['acc_oem'])})"
            )
        print("  1. Status")
        print("  2. Prepare axis (power/current/speed/acc)")
        print("  3. Nudge (+steps then -steps)")
        print("  4. Move absolute")
        print("  5. Move relative")
        print("  6. Edit defaults")
        print("  7. Stop")
        print("  8. Home search (OEM-like)")
        print("  9. Jog one-way (hold)")
        print("  10. Visible travel probe (quiet + switch-guarded)")
        print("  11. Force limit-bypass + visible probe")
        print("  12. Driver param quick probe")
        print("  b. Back")
        print("  q. Quit")

        ans = input("\n[CHAN] > ").strip().lower()
        if ans == "":
            continue
        if ans == "b":
            return
        if ans == "q":
            sys.exit(0)
        if ans == "1":
            rail = tester.motor_query_24v_sensor()
            no24 = rail.get("no24v")
            if no24 is None:
                rail_state = "unknown"
            else:
                rail_state = "NO_24V" if no24 else "OK"
            print(
                f"24V sensor (deck cmd15/type0): raw={rail.get('raw')} "
                f"state={rail_state} ack={fmt_resp(rail.get('ack'))}"
            )
            _print_motor_axis_status(tester, label, board, motor)
            _print_motion_gate_status(tester, refresh=True)
        elif ans == "2":
            _prepare_for_motion()
        elif ans == "3":
            if not _require_gantry_motion_gate("nudge"):
                continue
            _prepare_for_motion()
            out = tester.motor_step_test(board, steps=nudge_steps, motor=motor, wait_timeout_s=4.0)
            vrow = tester.motor_validate_step_test(
                board,
                steps=nudge_steps,
                test_out=out,
                motor=motor,
                axis_label=axis_name,
            )
            out["validation"] = vrow.get("validation")
            _print_motor_step_result(f"{label} nudge", out)
            _handle_runtime_motion_validation(f"{label} nudge", out.get("validation"))
        elif ans == "4":
            pin = input("absolute position: ").strip()
            try:
                pos = int(pin)
            except ValueError:
                print("Invalid position.")
                continue
            if not _require_gantry_motion_gate("move abs"):
                continue
            _prepare_for_motion()
            mv = tester.motor_move_absolute(board, pos, motor=motor)
            wt = tester.motor_wait_stopped(board, motor=motor, timeout_s=6.0)
            ps = tester.motor_get_position(board, motor=motor)
            print(
                f"move abs: ack={fmt_resp(mv.get('ack'))} target={pos} "
                f"stopped={wt.get('stopped')} time={wt.get('elapsed_ms')}ms pos={ps.get('position')}"
            )
        elif ans == "5":
            sin = input("relative steps (+/- int): ").strip()
            try:
                steps = int(sin)
            except ValueError:
                print("Invalid steps.")
                continue
            if not _require_gantry_motion_gate("move rel"):
                continue
            _prepare_for_motion()
            mv = tester.motor_move_relative(board, steps, motor=motor)
            wt = tester.motor_wait_stopped(board, motor=motor, timeout_s=6.0)
            ps = tester.motor_get_position(board, motor=motor)
            print(
                f"move rel: ack={fmt_resp(mv.get('ack'))} steps={steps} "
                f"stopped={wt.get('stopped')} time={wt.get('elapsed_ms')}ms pos={ps.get('position')}"
            )
        elif ans == "6":
            sin = input(f"speed (blank={speed}): ").strip()
            ain = input(f"acceleration (blank={acc}): ").strip()
            rin = input(f"run current (0..31, blank={run_current}): ").strip()
            tin = input(f"standby current (0..31, blank={standby_current}): ").strip()
            gin = input(f"stall guard (blank={stall_guard}): ").strip()
            minp = input(f"ramp mode SAP138 (blank={ramp_mode}): ").strip()
            nin = input(f"nudge steps (blank={nudge_steps}): ").strip()
            vin = input(f"visible one-way steps (blank={visible_steps}): ").strip()
            hin = input(f"home speed (blank={home_speed}): ").strip()
            try:
                if sin != "":
                    speed = int(sin)
                if ain != "":
                    acc = int(ain)
                if rin != "":
                    run_current = int(rin)
                if tin != "":
                    standby_current = int(tin)
                if gin != "":
                    stall_guard = int(gin)
                if minp != "":
                    ramp_mode = int(minp)
                if nin != "":
                    nudge_steps = int(nin)
                if vin != "":
                    visible_steps = int(vin)
                if hin != "":
                    home_speed = int(hin)
            except ValueError:
                print("Invalid numeric input.")
                continue
            n = tester.motor_normalize_speed_acc(board, motor=motor, speed=speed, acc=acc)
            speed = int(n.get("speed")) if n.get("speed") is not None else speed
            acc = int(n.get("acc")) if n.get("acc") is not None else acc
            for row in n.get("changes", []):
                print(
                    f"  clamp: {row.get('field')} {row.get('requested')} -> {row.get('value')} "
                    f"(range {row.get('min')}..{row.get('max')})"
                )
            print("Defaults updated.")
        elif ans == "7":
            r = tester.motor_stop(board, motor=motor)
            print(f"STOP: ack={fmt_resp(r.get('ack'))}")
        elif ans == "8":
            if not _require_gantry_motion_gate("home search"):
                continue
            _prepare_for_motion()
            timeout_s = 12.0
            if str(label).upper() in ("GRIPPER", "THERMAL_DOOR"):
                timeout_s = 10.0
            out = tester.motor_axis_search_home(
                board,
                motor=motor,
                speed=home_speed,
                timeout_s=timeout_s,
                home_active_value=0,
            )
            _print_motor_home_result(label, out)
        elif ans == "9":
            din = input("direction (+/- , blank=+): ").strip()
            sin = input(f"steps (blank={nudge_steps}): ").strip()
            sign = -1 if din == "-" else 1
            try:
                steps = int(sin) if sin != "" else int(nudge_steps)
            except ValueError:
                print("Invalid steps.")
                continue
            steps = abs(int(steps)) * sign
            if not _require_gantry_motion_gate("jog hold"):
                continue
            _prepare_for_motion()
            pre = tester.motor_get_position(board, motor=motor)
            pre_sw = tester.motor_get_switches(board, motor=motor)
            mv = tester.motor_move_relative(board, steps, motor=motor)
            wt = tester.motor_wait_stopped(board, motor=motor, timeout_s=8.0)
            post = tester.motor_get_position(board, motor=motor)
            sw = tester.motor_get_switches(board, motor=motor)
            delta = None
            if pre.get("position") is not None and post.get("position") is not None:
                delta = int(post["position"]) - int(pre["position"])
            print(
                f"jog hold: ack={fmt_resp(mv.get('ack'))} steps={steps} "
                f"stopped={wt.get('stopped')} time={wt.get('elapsed_ms')}ms "
                f"pre={pre.get('position')} post={post.get('position')} delta={delta} "
                f"switches(L={sw.get('left_state')} R={sw.get('right_state')})"
            )
            v = tester.motor_validate_oneway_move(
                axis=axis_name,
                step_cmd=int(steps),
                pre_switches=pre_sw,
                post_switches=sw,
                move_ack=mv.get("ack"),
                wait_row=wt,
                delta=delta,
            )
            _handle_runtime_motion_validation("jog hold", v)
        elif ans == "10":
            din = input("direction (+/- , blank=+): ").strip()
            sin = input(f"steps (blank={visible_steps}): ").strip()
            sign = -1 if din == "-" else 1
            try:
                steps = int(sin) if sin != "" else int(visible_steps)
            except ValueError:
                print("Invalid steps.")
                continue
            steps = abs(int(steps)) * sign
            if not _require_gantry_motion_gate("visible probe"):
                continue
            _prepare_for_motion()
            if is_gantry_axis:
                quiet_speed = min(int(speed), 320)
                quiet_acc = min(int(acc), 140)
                print(f"  quiet probe profile: speed={quiet_speed} acc={quiet_acc}")
                qops = tester.motor_set_speed_acc(board, motor=motor, speed=quiet_speed, acc=quiet_acc)
                _print_motor_prep_ops({"ops": qops})
            out = tester.motor_visible_oneway_probe(
                board,
                steps=steps,
                motor=motor,
                timeout_s=20.0,
                enforce_switch_guard=True,
            )
            if out.get("aborted"):
                print(f"visible probe: blocked ({out.get('abort_reason')})")
                if out.get("suggested_dir") in ("+", "-"):
                    print(f"  hint: try direction '{out.get('suggested_dir')}'")
                continue
            pre_sw = out.get("pre_switches", {})
            post_sw = out.get("post_switches", {})
            print(
                f"visible probe: ack={fmt_resp(out.get('move', {}).get('ack'))} steps={steps} "
                f"stopped={out.get('wait', {}).get('stopped')} "
                f"time={out.get('wait', {}).get('elapsed_ms')}ms"
            )
            print(
                f"  pos: pre={out.get('pre', {}).get('position')} "
                f"post={out.get('post', {}).get('position')} delta={out.get('delta')}"
            )
            print(
                f"  switches: pre(L={pre_sw.get('left_state')} R={pre_sw.get('right_state')}) "
                f"post(L={post_sw.get('left_state')} R={post_sw.get('right_state')}) "
                f"left_cleared={out.get('left_cleared')} "
                f"right_cleared={out.get('right_cleared')} "
                f"edge={out.get('any_switch_edge')}"
            )
            evs = out.get("events", [])
            stalls = out.get("stall_events", [])
            print(f"  events: total={len(evs)} stall={len(stalls)}")
            if len(stalls) > 0:
                print("  stall detail:")
                for ev in stalls[:6]:
                    print(
                        f"    board=0x{int(ev.get('board', 0)):02X} "
                        f"status={ev.get('status')}({ev.get('status_str')}) "
                        f"cmd={ev.get('cmd')} value={ev.get('value')}"
                    )
            if out.get("suggested_dir") in ("+", "-"):
                print(f"  hint: active switch suggests trying direction '{out.get('suggested_dir')}'")
            v = tester.motor_validate_oneway_move(
                axis=axis_name,
                step_cmd=int(steps),
                pre_switches=out.get("pre_switches", {}),
                post_switches=out.get("post_switches", {}),
                move_ack=out.get("move", {}).get("ack") if isinstance(out.get("move"), dict) else None,
                wait_row=out.get("wait", {}),
                delta=out.get("delta"),
            )
            _handle_runtime_motion_validation("visible probe", v)
        elif ans == "11":
            din = input("direction (+/- , blank=+): ").strip()
            sin = input(f"steps (blank={visible_steps}): ").strip()
            sign = -1 if din == "-" else 1
            try:
                steps = int(sin) if sin != "" else int(visible_steps)
            except ValueError:
                print("Invalid steps.")
                continue
            steps = abs(int(steps)) * sign
            if not _require_gantry_motion_gate("force probe"):
                continue
            _prepare_for_motion()
            dr = tester.motor_set_axis_param(board, 12, 1, motor=motor)
            dl = tester.motor_set_axis_param(board, 13, 1, motor=motor)
            print(
                f"  force limit bypass: "
                f"SAP12={fmt_resp(dr.get('ack'))} rb={dr.get('readback', {}).get('value')} "
                f"SAP13={fmt_resp(dl.get('ack'))} rb={dl.get('readback', {}).get('value')}"
            )
            out = tester.motor_visible_oneway_probe(
                board,
                steps=steps,
                motor=motor,
                timeout_s=20.0,
                enforce_switch_guard=False,
            )
            pre_sw = out.get("pre_switches", {})
            post_sw = out.get("post_switches", {})
            print(
                f"force-probe: ack={fmt_resp(out.get('move', {}).get('ack'))} steps={steps} "
                f"stopped={out.get('wait', {}).get('stopped')} "
                f"time={out.get('wait', {}).get('elapsed_ms')}ms"
            )
            print(
                f"  pos: pre={out.get('pre', {}).get('position')} "
                f"post={out.get('post', {}).get('position')} delta={out.get('delta')}"
            )
            print(
                f"  switches: pre(L={pre_sw.get('left_state')} R={pre_sw.get('right_state')}) "
                f"post(L={post_sw.get('left_state')} R={post_sw.get('right_state')}) "
                f"left_cleared={out.get('left_cleared')} "
                f"right_cleared={out.get('right_cleared')} "
                f"edge={out.get('any_switch_edge')}"
            )
            evs = out.get("events", [])
            stalls = out.get("stall_events", [])
            print(f"  events: total={len(evs)} stall={len(stalls)}")
            if len(stalls) > 0:
                print("  stall detail:")
                for ev in stalls[:6]:
                    print(
                        f"    board=0x{int(ev.get('board', 0)):02X} "
                        f"status={ev.get('status')}({ev.get('status_str')}) "
                        f"cmd={ev.get('cmd')} value={ev.get('value')}"
                    )
            v = tester.motor_validate_oneway_move(
                axis=axis_name,
                step_cmd=int(steps),
                pre_switches=out.get("pre_switches", {}),
                post_switches=out.get("post_switches", {}),
                move_ack=out.get("move", {}).get("ack") if isinstance(out.get("move"), dict) else None,
                wait_row=out.get("wait", {}),
                delta=out.get("delta"),
            )
            _handle_runtime_motion_validation("force probe", v)
        elif ans == "12":
            rows = tester.motor_probe_axis_params(
                board,
                motor=motor,
                params=[6, 7, 12, 13, 138, 153, 154, 170, 178, 204, 205],
            )
            print(f"driver quick probe: board=0x{int(board):02X} motor={int(motor)}")
            for row in rows.get("rows", []):
                print(
                    f"  p{int(row.get('param'))}: "
                    f"ack={fmt_resp(row.get('ack'))} "
                    f"value={row.get('value')}"
                )
        else:
            print("Invalid.")


def run_motor_function_menu(tester):
    while True:
        print("\n----------------------------------------")
        print(" MOTOR BY FUNCTION")
        print("----------------------------------------")
        print("  1. X axis (0x05:0, deck travel)")
        print("  2. Y axis (0x04:0, head gantry travel)")
        print("  3. Z axis (0x04:1, head UP/DOWN)")
        print("  4. Gripper (0x04:2)")
        print("  5. Thermal Door (0x06:0)")
        print("  s. Status all")
        print("  b. Back")
        print("  q. Quit")

        ans = input("\n[FUNC] > ").strip().lower()
        if ans == "":
            continue
        if ans == "b":
            return
        if ans == "q":
            sys.exit(0)

        key = None
        if ans == "1":
            key = "x"
        elif ans == "2":
            key = "y"
        elif ans == "3":
            key = "z"
        elif ans == "4":
            key = "g"
        elif ans == "5":
            key = "door"
        elif ans == "s":
            rail = tester.motor_query_24v_sensor()
            no24 = rail.get("no24v")
            if no24 is None:
                rail_state = "unknown"
            else:
                rail_state = "NO_24V" if no24 else "OK"
            print(
                f"24V sensor (deck cmd15/type0): raw={rail.get('raw')} "
                f"state={rail_state} ack={fmt_resp(rail.get('ack'))}"
            )
            for k in ("x", "y", "z", "door", "g"):
                p = tester.motor_function_preset(k)
                _print_motor_axis_status(tester, p["label"], p["board"], p["motor"])
            _print_motion_gate_status(tester, refresh=True)
            continue
        else:
            print("Invalid.")
            continue

        p = tester.motor_function_preset(key)
        run_motor_channel_menu(tester, p["label"], p["board"], p["motor"], preset=p)


def run_motor_board_menu(tester):
    while True:
        print("\n----------------------------------------")
        print(" MOTOR BY BOARD")
        print("----------------------------------------")
        print("  1. 0x04 HEAD (motors: 0=Y travel, 1=Z up/down, 2=GRIPPER)")
        print("  2. 0x05 DECK (motor: 0=X)")
        print("  3. 0x06 THERMAL (motor: 0=THERMAL_DOOR)")
        print("  4. Custom board/motor")
        print("  b. Back")
        print("  q. Quit")

        ans = input("\n[BOARD] > ").strip().lower()
        if ans == "":
            continue
        if ans == "b":
            return
        if ans == "q":
            sys.exit(0)

        if ans == "1":
            idx = input("motor on HEAD [0/1/2] (blank=1): ").strip()
            if idx == "":
                mid = 1
            else:
                try:
                    mid = int(idx)
                except ValueError:
                    print("Invalid motor.")
                    continue
            labels = {0: "Y", 1: "Z", 2: "GRIPPER"}
            label = labels.get(mid, "HEAD_CUSTOM")
            p = tester.motor_function_preset("y") if mid == 0 else tester.motor_function_preset("z") if mid == 1 else tester.motor_function_preset("g") if mid == 2 else None
            run_motor_channel_menu(tester, label, tester.BOARD_HEAD, mid, preset=p)
        elif ans == "2":
            p = tester.motor_function_preset("x")
            run_motor_channel_menu(tester, p["label"], p["board"], p["motor"], preset=p)
        elif ans == "3":
            p = tester.motor_function_preset("door")
            run_motor_channel_menu(tester, p["label"], p["board"], p["motor"], preset=p)
        elif ans == "4":
            print("Boards: 0x04=HEAD, 0x05=DECK, 0x06=THERMAL, 0x07=CHILLER")
            s = input("board id (hex like 0x04 or decimal): ").strip().lower()
            base = 16 if s.startswith("0x") else 10
            try:
                bid = int(s, base)
            except ValueError:
                print("Invalid board id.")
                continue
            if bid not in tester.BOARDS:
                print("Unknown board.")
                continue
            mid_in = input("motor index (default 0): ").strip()
            try:
                mid = int(mid_in) if mid_in != "" else 0
            except ValueError:
                print("Invalid motor index.")
                continue
            run_motor_channel_menu(tester, "CUSTOM", bid, mid, preset=None)
        else:
            print("Invalid.")


def run_thermal_door_menu(tester):
    p = tester.motor_function_preset("door")
    board = int(p["board"])
    motor = int(p["motor"])
    speed = 100
    acc = 50
    run_current = 31
    standby_current = int(p["standby_current"])
    stall_guard = 8
    open_pos = 10750
    close_pos = -7000
    nudge_steps = 2000
    seek_step = 300
    seek_max_moves = 90

    def _prepare_door():
        prep = tester.motor_prepare_axis(
            board,
            motor=motor,
            run_current=run_current,
            standby_current=standby_current,
            speed=speed,
            acc=acc,
            stall_guard=stall_guard,
            ramp_mode=p.get("ramp_mode"),
            disable_right=bool(p.get("disable_right", True)),
            disable_left=bool(p.get("disable_left", True)),
        )
        return prep

    def _move_abs_with_wait(pos):
        mv = tester.motor_move_absolute(board, int(pos), motor=motor)
        wt = tester.motor_wait_stopped(board, motor=motor, timeout_s=8.0)
        ps = tester.motor_get_position(board, motor=motor)
        sw = tester.motor_get_switch_activity(board, motor=motor)
        print(
            f"move abs: ack={fmt_resp(mv.get('ack'))} target={int(pos)} "
            f"stopped={wt.get('stopped')} time={wt.get('elapsed_ms')}ms pos={ps.get('position')} "
            f"switches(L={sw.get('left_state')} R={sw.get('right_state')}) "
            f"active(closed={sw.get('left_active')} opened={sw.get('right_active')})"
        )

    def _door_sensor_status():
        sw = tester.motor_get_switch_activity(board, motor=motor)
        pos = tester.motor_get_position(board, motor=motor)
        return {
            "position": pos.get("position"),
            "left_state": sw.get("left_state"),
            "right_state": sw.get("right_state"),
            "closed": sw.get("left_active"),
            "opened": sw.get("right_active"),
        }

    def _door_seek(target):
        target = str(target).strip().lower()
        if target not in ("open", "close"):
            return {"error": f"invalid target '{target}'"}
        prep = _prepare_door()
        start = _door_sensor_status()
        if target == "close":
            is_done = bool(start.get("closed"))
            direction = -1
            target_name = "tcDoorClosed"
        else:
            is_done = bool(start.get("opened"))
            direction = +1
            target_name = "tcDoorOpened"
        if is_done:
            return {
                "target": target_name,
                "already": True,
                "prep": prep,
                "start": start,
                "end": start,
                "moves": [],
            }

        moves = []
        last_pos = start.get("position")
        for i in range(max(1, int(seek_max_moves))):
            step = int(direction * abs(int(seek_step)))
            mv = tester.motor_move_relative(board, step, motor=motor)
            wt = tester.motor_wait_stopped(board, motor=motor, timeout_s=3.0, poll_s=0.06)
            st = _door_sensor_status()
            now_pos = st.get("position")
            delta = None if last_pos is None or now_pos is None else int(now_pos) - int(last_pos)
            moves.append(
                {
                    "idx": i + 1,
                    "step": step,
                    "ack": mv.get("ack"),
                    "stopped": wt.get("stopped"),
                    "elapsed_ms": wt.get("elapsed_ms"),
                    "pos": now_pos,
                    "delta": delta,
                    "closed": st.get("closed"),
                    "opened": st.get("opened"),
                    "left_state": st.get("left_state"),
                    "right_state": st.get("right_state"),
                }
            )
            last_pos = now_pos

            if target == "close" and bool(st.get("closed")):
                break
            if target == "open" and bool(st.get("opened")):
                break
            if delta is None or int(delta) == 0:
                break
            time.sleep(0.03)

        tester.motor_stop(board, motor=motor)
        end = _door_sensor_status()
        reached = bool(end.get("closed")) if target == "close" else bool(end.get("opened"))
        return {
            "target": target_name,
            "already": False,
            "prep": prep,
            "start": start,
            "end": end,
            "moves": moves,
            "reached": reached,
        }

    def _print_door_seek(out):
        if not isinstance(out, dict):
            print("door seek: no data")
            return
        if out.get("error"):
            print(f"door seek error: {out.get('error')}")
            return
        _print_motor_prep_ops(out.get("prep"))
        start = out.get("start", {})
        end = out.get("end", {})
        if out.get("already"):
            print(
                f"{out.get('target')}: already true "
                f"(pos={start.get('position')} L={start.get('left_state')} R={start.get('right_state')})"
            )
            return
        moves = out.get("moves", [])
        print(
            f"{out.get('target')}: reached={out.get('reached')} "
            f"moves={len(moves)} pos {start.get('position')} -> {end.get('position')} "
            f"sensors L {start.get('left_state')}->{end.get('left_state')} "
            f"R {start.get('right_state')}->{end.get('right_state')}"
        )
        tail = moves[-4:] if len(moves) > 4 else moves
        for row in tail:
            print(
                f"  #{row.get('idx')}: step={row.get('step')} ack={fmt_resp(row.get('ack'))} "
                f"dt={row.get('elapsed_ms')}ms delta={row.get('delta')} "
                f"L={row.get('left_state')} R={row.get('right_state')}"
            )

    while True:
        print("\n----------------------------------------")
        print(" THERMAL DOOR CONTROL (0x06:0)")
        print("----------------------------------------")
        print(
            f"  defaults: open={open_pos} close={close_pos} speed={speed} acc={acc} "
            f"run_current={run_current} standby={standby_current} stall_guard={stall_guard} "
            f"seek_step={seek_step} seek_max={seek_max_moves}"
        )
        print("  1. Status")
        print("  2. Prepare door axis")
        print("  3. OPEN (sensor-guided tcDoorOpened)")
        print("  4. CLOSE (sensor-guided tcDoorClosed)")
        print("  5. PULSE (close -> open, sensor-guided)")
        print("  6. Nudge (+steps then -steps)")
        print("  7. Move absolute (custom)")
        print("  8. Edit defaults")
        print("  9. Stop")
        print("  b. Back")
        print("  q. Quit")

        ans = input("\n[DOOR] > ").strip().lower()
        if ans == "":
            continue
        if ans == "b":
            return
        if ans == "q":
            sys.exit(0)

        if ans == "1":
            rail = tester.motor_query_24v_sensor()
            no24 = rail.get("no24v")
            if no24 is None:
                rail_state = "unknown"
            else:
                rail_state = "NO_24V" if no24 else "OK"
            print(
                f"24V sensor (deck cmd15/type0): raw={rail.get('raw')} "
                f"state={rail_state} ack={fmt_resp(rail.get('ack'))}"
            )
            _print_motor_axis_status(tester, "THERMAL_DOOR", board, motor)
            st = _door_sensor_status()
            print(
                f"  confirmAxis mapping: tcDoorClosed={st.get('closed')} "
                f"tcDoorOpened={st.get('opened')} "
                f"(L={st.get('left_state')} R={st.get('right_state')})"
            )
        elif ans == "2":
            prep = _prepare_door()
            _print_motor_prep_ops(prep)
        elif ans == "3":
            out = _door_seek("open")
            _print_door_seek(out)
        elif ans == "4":
            out = _door_seek("close")
            _print_door_seek(out)
        elif ans == "5":
            print("PULSE close -> open")
            out1 = _door_seek("close")
            _print_door_seek(out1)
            time.sleep(0.15)
            out2 = _door_seek("open")
            _print_door_seek(out2)
        elif ans == "6":
            out = tester.motor_step_test(board, steps=nudge_steps, motor=motor, wait_timeout_s=5.0)
            _print_motor_step_result("THERMAL_DOOR nudge", out)
        elif ans == "7":
            pin = input("absolute position: ").strip()
            try:
                pos = int(pin)
            except ValueError:
                print("Invalid position.")
                continue
            _move_abs_with_wait(pos)
        elif ans == "8":
            oin = input(f"open position (blank={open_pos}): ").strip()
            cin = input(f"close position (blank={close_pos}): ").strip()
            sin = input(f"speed (blank={speed}): ").strip()
            ain = input(f"acceleration (blank={acc}): ").strip()
            rin = input(f"run current 0..31 (blank={run_current}): ").strip()
            tin = input(f"standby current 0..31 (blank={standby_current}): ").strip()
            gin = input(f"stall guard SAP205 (blank={stall_guard}): ").strip()
            nin = input(f"nudge steps (blank={nudge_steps}): ").strip()
            qin = input(f"sensor-seek step (blank={seek_step}): ").strip()
            xin = input(f"sensor-seek max moves (blank={seek_max_moves}): ").strip()
            try:
                if oin != "":
                    open_pos = int(oin)
                if cin != "":
                    close_pos = int(cin)
                if sin != "":
                    speed = int(sin)
                if ain != "":
                    acc = int(ain)
                if rin != "":
                    run_current = int(rin)
                if tin != "":
                    standby_current = int(tin)
                if gin != "":
                    stall_guard = int(gin)
                if nin != "":
                    nudge_steps = int(nin)
                if qin != "":
                    seek_step = int(qin)
                if xin != "":
                    seek_max_moves = int(xin)
            except ValueError:
                print("Invalid numeric input.")
                continue
            print("Thermal door defaults updated.")
        elif ans == "9":
            r = tester.motor_stop(board, motor=motor)
            print(f"STOP: ack={fmt_resp(r.get('ack'))}")
        else:
            print("Invalid.")


def run_motor_experimental_menu(tester):
    while True:
        print("\n----------------------------------------")
        print(" EXPERIMENTAL CHANGES")
        print("----------------------------------------")
        print("  1. STRICT STARTUP MOTION INIT (sensor-gated + full homing)")
        print("  2. MOTION ARM STATUS (live sensors)")
        print("  3. DISARM XYZ MOTION GATE")
        print("  4. SHOW LAST STRICT INIT REPORT")
        print("  5. DECK LATCH OVERRIDE TOGGLE (diagnostic unlock mode)")
        print("  b. Back")
        print("  q. Quit")

        ans = input("\n[EXPERIMENTAL] > ").strip().lower()
        if ans == "":
            continue
        if ans == "b":
            return
        if ans == "q":
            sys.exit(0)

        if ans == "1":
            print("Running strict startup motion init...")
            out = tester.motion_arm_strict_startup(run_homing=True)
            _print_strict_startup_report(out)
        elif ans == "2":
            _print_motion_gate_status(tester, refresh=True)
        elif ans == "3":
            st = tester.motion_disarm(reason="manual")
            print(
                f"motion arm disarmed: armed={st.get('armed')} "
                f"reason={st.get('reason')} note={st.get('note')} seq={st.get('arm_seq')}"
            )
        elif ans == "4":
            out = tester.motion_last_strict_init_report()
            _print_strict_startup_report(out)
        elif ans == "5":
            cur = tester.motion_latch_override_state()
            enable = not bool(cur.get("enabled"))
            out = tester.motion_latch_override_set(enable, reason="experimental_toggle")
            st = out.get("state", {})
            wr = out.get("write", {})
            mode = "ENABLED (diagnostic unlock)" if st.get("enabled") else "DISABLED (auto-lock)"
            print(
                f"deck latch override {mode}: ack={fmt_resp(wr.get('ack'))} "
                f"snapshot={wr.get('snapshot')}"
            )
            _print_motion_gate_status(tester, refresh=False, snapshot=out.get("gate"))
        else:
            print("Invalid.")


def run_motor_menu(tester):
    while True:
        print("\n----------------------------------------")
        print(" MOTOR CONTROL")
        print("----------------------------------------")
        print("  1. BY FUNCTION")
        print("  2. BY BOARD")
        print("  3. THERMAL DOOR CONTROL (focus)")
        print("  4. QUICK STATUS (X/HEAD-Z/DOOR + 24V)")
        print("  5. ENABLE MOTION (X/Y/Z + DOOR)")
        print("  6. ONE-CLICK X/HEAD-Z POWER + MOTION DIAG")
        print("  7. ONE-CLICK OEM STARTUP HOMING")
        print("  8. MOTION INTERLOCK PREP (LOCK + XYZ WAKE)")
        print("  9. DRIVER POWER DIAG (X/Y/Z vs GRIPPER/DOOR)")
        print("  10. EXPERIMENTAL PROFILE COPY + QUICK PROBE")
        print("  11. ESM CYCLE (HEAD/DECK) + XYZ PROBE")
        print("  12. GENTLE CREEP TEST (X/Y/Z anti-bang)")
        print("  13. RESTORE OEM XYZ PROFILE + QUIET PROBE")
        print("  14. HARD RESET / PANIC OFF (HEAD+DECK)")
        print("  15. QUICK RELIABILITY SMOKE (X/Y/Z)")
        print("  16. EXPERIMENTAL CHANGES")
        print("  b. Back")
        print("  q. Quit")

        ans = input("\n[MOTOR] > ").strip().lower()
        if ans == "":
            continue
        if ans == "b":
            return
        if ans == "q":
            sys.exit(0)

        if ans == "1":
            run_motor_function_menu(tester)
        elif ans == "2":
            run_motor_board_menu(tester)
        elif ans == "3":
            run_thermal_door_menu(tester)
        elif ans == "4":
            rail = tester.motor_query_24v_sensor()
            no24 = rail.get("no24v")
            if no24 is None:
                rail_state = "unknown"
            else:
                rail_state = "NO_24V" if no24 else "OK"
            print(
                f"24V sensor (deck cmd15/type0): raw={rail.get('raw')} "
                f"state={rail_state} ack={fmt_resp(rail.get('ack'))}"
            )
            for key in ("x", "z", "door"):
                p = tester.motor_function_preset(key)
                _print_motor_axis_status(tester, p["label"], p["board"], p["motor"])
            _print_motion_gate_status(tester, refresh=True)
        elif ans == "5":
            print("Running enable for X/Y/Z...")
            out = tester.motor_enable_sequence(conservative=True)
            print(f"x/y/z done in {out.get('elapsed_ms')}ms")
            for row in out.get("rows", []):
                axis = row.get("axis", "?")
                op = row.get("op", "?")
                setv = row.get("set")
                rb = row.get("rb")
                if rb:
                    print(
                        f"  {axis} 0x{int(row['board']):02X}:{int(row.get('motor', 0))} {op}: "
                        f"ack={fmt_resp(row.get('ack'))} set={setv} readback={rb.get('value')}"
                    )
                else:
                    print(
                        f"  {axis} 0x{int(row['board']):02X}:{int(row.get('motor', 0))} {op}: "
                        f"ack={fmt_resp(row.get('ack'))}"
                    )
            pd = tester.motor_function_preset("door")
            print("Preparing thermal door...")
            door = tester.motor_prepare_axis(
                pd["board"],
                motor=pd["motor"],
                run_current=pd["run_current"],
                standby_current=pd["standby_current"],
                speed=pd["speed"],
                acc=pd["acc"],
                stall_guard=pd.get("stall_guard"),
                ramp_mode=pd.get("ramp_mode"),
                disable_right=bool(pd.get("disable_right", False)),
                disable_left=bool(pd.get("disable_left", False)),
                rdiv=pd.get("rdiv"),
                pdiv=pd.get("pdiv"),
                warm_enable=bool(pd.get("warm_enable", False)),
            )
            _print_motor_prep_ops(door)
        elif ans == "6":
            print("Running one-click X/Z diagnostic...")
            out = tester.motor_xz_power_diagnostic(
                steps_x=_motor_steps_default("X"),
                steps_z=_motor_steps_default("Z"),
            )
            print(f"done in {out.get('elapsed_ms')}ms")
            print("board status:")
            for bid in tester.BOARDS:
                print(f"  0x{bid:02X}: {fmt_resp(out['board_status'].get(bid))}")
            rail = out.get("rail_24v", {})
            no24 = rail.get("no24v")
            if no24 is None:
                rail_state = "unknown"
            else:
                rail_state = "NO_24V" if no24 else "OK"
            print(
                f"24V sensor (deck cmd15/type0): raw={rail.get('raw')} "
                f"state={rail_state} ack={fmt_resp(rail.get('ack'))}"
            )
            _print_motor_step_result("X diag", out["test_x"])
            _print_motor_step_result("Z diag", out["test_z"])
        elif ans == "7":
            print("Running OEM startup homing mimic (Z/G/X/Y/DOOR)...")
            out = tester.motor_startup_homing_mimic()
            print(f"done in {out.get('elapsed_ms')}ms")
            print("board status:")
            for bid in tester.BOARDS:
                print(f"  0x{bid:02X}: {fmt_resp(out['board_status'].get(bid))}")
            rail = out.get("rail_24v", {})
            no24 = rail.get("no24v")
            if no24 is None:
                rail_state = "unknown"
            else:
                rail_state = "NO_24V" if no24 else "OK"
            print(
                f"24V sensor (deck cmd15/type0): raw={rail.get('raw')} "
                f"state={rail_state} ack={fmt_resp(rail.get('ack'))}"
            )
            print("prep summary:")
            for key in ("x", "y", "z", "g", "door"):
                row = out.get("prep", {}).get(key, {})
                bid = row.get("board")
                mid = row.get("motor")
                btxt = f"0x{int(bid):02X}" if isinstance(bid, int) else "?"
                mtxt = str(int(mid)) if isinstance(mid, int) else "?"
                print(f"  {key.upper()} board={btxt} motor={mtxt}")
                _print_motor_prep_ops(row)
            print(
                f"gripper pre-move: rel+10000={fmt_resp(out.get('g_pre_move', {}).get('ack'))} "
                f"wait={out.get('g_pre_wait', {}).get('stopped')}({out.get('g_pre_wait', {}).get('elapsed_ms')}ms)"
            )
            _print_motor_home_result("Z", out.get("z_home", {}))
            _print_motor_home_result("GRIPPER", out.get("g_home", {}))
            _print_motor_home_result("X", out.get("x_home", {}))
            print(
                f"X post-home: set_home={fmt_resp(out.get('x_set_home', {}).get('ack'))} "
                f"speed={fmt_resp(out.get('x_speed', {}).get('ack'))} "
                f"move6000={fmt_resp(out.get('x_move_6000', {}).get('ack'))} "
                f"wait={out.get('x_wait', {}).get('stopped')}({out.get('x_wait', {}).get('elapsed_ms')}ms)"
            )
            _print_motor_home_result("Y", out.get("y_home", {}))
            print(f"Y post-home: set_home={fmt_resp(out.get('y_set_home', {}).get('ack'))}")
            _print_motor_home_result("THERMAL_DOOR", out.get("door_home", {}))
        elif ans == "8":
            print("Running motion interlock prep...")
            out = tester.motor_prepare_motion_interlock(force_lock=True)
            _print_motion_interlock(out)
        elif ans == "9":
            print("Running one-click driver power diag...")
            out = tester.motor_driver_power_diag(axis_keys=("x", "y", "z", "g", "door"))
            _print_driver_diag_summary(out)
        elif ans == "10":
            print("Running experimental profile copy from GRIPPER to X/Y/Z...")
            out = tester.motor_clone_driver_profile(source_key="g", dest_keys=("x", "y", "z"))
            _print_driver_clone_result(out)
            print("Preparing interlock + quick probes...")
            inter = tester.motor_prepare_motion_interlock(force_lock=True)
            _print_motion_interlock(inter)
            steps_map = {"x": 50000, "y": 30000, "z": 20000}
            for key in ("x", "y", "z"):
                p = tester.motor_function_preset(key)
                prep = tester.motor_prepare_axis(
                    p["board"],
                    motor=p["motor"],
                    run_current=p["run_current"],
                    standby_current=p["standby_current"],
                    speed=p["speed"],
                    acc=p["acc"],
                    stall_guard=p.get("stall_guard"),
                    ramp_mode=p.get("ramp_mode"),
                    disable_right=bool(p.get("disable_right", False)),
                    disable_left=bool(p.get("disable_left", False)),
                    rdiv=p.get("rdiv"),
                    pdiv=p.get("pdiv"),
                    warm_enable=bool(p.get("warm_enable", False)),
                )
                _print_motor_prep_ops(prep)
                steps = int(steps_map.get(key, 20000))
                probe = tester.motor_visible_oneway_probe(
                    p["board"],
                    steps=steps,
                    motor=p["motor"],
                    timeout_s=16.0,
                )
                evs = probe.get("events", [])
                stalls = probe.get("stall_events", [])
                print(
                    f"{p['label']} quick-probe: ack={fmt_resp(probe.get('move', {}).get('ack'))} "
                    f"steps={steps} delta={probe.get('delta')} "
                    f"stopped={probe.get('wait', {}).get('stopped')} "
                    f"time={probe.get('wait', {}).get('elapsed_ms')}ms "
                    f"switches L={probe.get('post_switches', {}).get('left_state')} "
                    f"R={probe.get('post_switches', {}).get('right_state')} "
                    f"events={len(evs)} stall={len(stalls)}"
                )
        elif ans == "11":
            cin = input("ESM cycles (blank=1): ").strip()
            try:
                cycles = int(cin) if cin != "" else 1
            except ValueError:
                print("Invalid cycles.")
                continue
            print("Running ESM cycle + XYZ probe...")
            out = tester.motor_esm_cycle_diag(cycles=cycles)
            _print_esm_cycle_diag(out)
        elif ans == "12":
            sin = input("step magnitude per trial (blank=8000): ").strip()
            try:
                step_mag = int(sin) if sin != "" else 8000
            except ValueError:
                print("Invalid step count.")
                continue
            step_mag = max(1000, abs(int(step_mag)))
            print("Running gentle creep tests (watch/listen for smooth travel)...")
            try:
                for key in ("x", "y", "z"):
                    out = tester.motor_gentle_creep_probe(key, steps=step_mag)
                    _print_gentle_creep_result(out)
            except KeyboardInterrupt:
                print("\nGentle creep interrupted.")
        elif ans == "13":
            print("Running OEM XYZ profile restore + quiet probe...")
            out = tester.motor_restore_oem_xyz_profile(run_probe=True)
            _print_oem_restore_result(out)
        elif ans == "14":
            rin = input("hard reset rounds (blank=3): ").strip()
            try:
                rounds = int(rin) if rin != "" else 3
            except ValueError:
                print("Invalid rounds.")
                continue
            out = tester.motor_hard_reset(rounds=max(1, rounds))
            for row in out.get("rounds", []):
                print(f"round {row.get('round')}:")
                for op in row.get("ops", []):
                    print(f"  {op.get('name')}: sent={op.get('sent')}/{op.get('attempted')}")
                print(f"  head fw: {fmt_resp(row.get('firmware_head', {}).get('ack'))}")
                print(f"  deck fw: {fmt_resp(row.get('firmware_deck', {}).get('ack'))}")
                rail = row.get("rail_24v", {})
                no24 = rail.get("no24v")
                if no24 is None:
                    rail_state = "unknown"
                else:
                    rail_state = "NO_24V" if no24 else "OK"
                print(
                    f"  24V sensor: raw={rail.get('raw')} state={rail_state} "
                    f"ack={fmt_resp(rail.get('ack'))}"
                )
            if out.get("ok"):
                print("Hard reset recovered deck/head motor comms.")
            else:
                print("Hard reset did not recover deck/head comms. Try main menu 's', then power-cycle instrument/CAN adapter.")
        elif ans == "15":
            lin = input("loops per axis (blank=2, range 1..5): ").strip()
            sin = input("step scale (blank=1.0, range 0.2..2.0): ").strip()
            try:
                loops = int(lin) if lin != "" else 2
                scale = float(sin) if sin != "" else 1.0
            except ValueError:
                print("Invalid quick-test inputs.")
                continue
            print("Axis note: Z is head vertical (UP/DOWN).")
            out = tester.motor_quick_reliability_smoke(loops=loops, step_scale=scale)
            print(f"quick smoke elapsed={out.get('elapsed_ms')}ms")
            rail = out.get("rail_24v", {})
            no24 = rail.get("no24v")
            if no24 is None:
                rail_state = "unknown"
            else:
                rail_state = "NO_24V" if no24 else "OK"
            print(
                f"24V sensor: raw={rail.get('raw')} state={rail_state} "
                f"ack={fmt_resp(rail.get('ack'))}"
            )
            for label, row in out.get("axes", {}).items():
                print(
                    f"{label} board=0x{int(row.get('board', 0)):02X} motor={int(row.get('motor', 0))} "
                    f"step={row.get('step_mag')} loops={row.get('loops')} axis_ok={row.get('ok')}"
                )
                for run in row.get("runs", []):
                    res = run.get("result", {})
                    print(
                        f"  run#{run.get('idx')}: step={run.get('step')} "
                        f"fwd={res.get('delta_fwd')} back={res.get('delta_back')} net={res.get('delta_net')} "
                        f"wait={res.get('wait_fwd', {}).get('stopped')}/{res.get('wait_back', {}).get('stopped')} "
                        f"ack={fmt_resp(res.get('fwd', {}).get('ack'))}/{fmt_resp(res.get('back', {}).get('ack'))} "
                        f"ok={run.get('ok')}"
                    )
            if out.get("ok"):
                print("Quick reliability smoke: PASS")
            else:
                print("Quick reliability smoke: FAIL (run hard reset 14, then retest)")
        elif ans == "16":
            run_motor_experimental_menu(tester)
        else:
            print("Invalid.")


def run_camera_menu_advanced(tester):
    def _print_probe(device):
        rows = tester.camera_probe_controls(device)
        print(f"UVC probe {device}:")
        for row in rows:
            q = row["query"]
            g = row["get"]
            if not q["ok"]:
                print(f"  {row['name']}: unsupported/query-error ({q.get('error')})")
                continue
            rng = f"[{q['minimum']}..{q['maximum']} step={q['step']} def={q['default']}]"
            if g["ok"]:
                print(
                    f"  {row['name']}: {g['value']} {rng} "
                    f"(cid=0x{row['cid']:08X} flags=0x{q['flags']:08X})"
                )
            else:
                print(
                    f"  {row['name']}: read-error ({g.get('error')}) {rng} "
                    f"(cid=0x{row['cid']:08X} flags=0x{q['flags']:08X})"
                )

    while True:
        print("\n----------------------------------------")
        print(" CAMERA SYSTEM (ADVANCED)")
        print("----------------------------------------")
        print("  1. List /dev/video devices")
        print("  2. Probe UVC controls (/dev/video0)")
        print("  3. Probe UVC controls (/dev/video1)")
        print("  4. Set backlight (/dev/video0)")
        print("  5. Set gain (/dev/video0)")
        print("  6. Set exposure abs (/dev/video0)")
        print("  7. Set exposure mode (/dev/video0)")
        print("  8. Show SMI LED path status")
        print("  9. Enumerate all UVC ctrls (/dev/video0)")
        print("  10. Set raw UVC ctrl by CID (/dev/video0)")
        print("  11. Sweep XU controls (vendor, /dev/video0)")
        print("  12. Set/Get raw XU payload")
        print("  13. AUTO XU SAFE SWEEP (no typing)")
        print("  14. AUTO PROFILE unit2/sel1 (+frame YAVG)")
        print("  15. CAMERA USB RESET + XU RESCAN")
        print("  16. AUTO CAMERA ONE-CLICK (recover + profile)")
        print("  b. Back")
        print("  q. Quit")

        ans = input("\n[CAM] > ").strip().lower()
        if ans == "":
            continue
        if ans == "b":
            return
        if ans == "q":
            sys.exit(0)
        if ans == "1":
            devs = tester.camera_devices()
            if not devs:
                print("No /dev/video* devices found.")
            else:
                print("Camera devices:")
                for d in devs:
                    print(f"  {d}")
        elif ans == "2":
            _print_probe("/dev/video0")
        elif ans == "3":
            _print_probe("/dev/video1")
            print("Note: /dev/video1 often appears as a metadata stream with no V4L2 controls.")
        elif ans == "4":
            val = input("backlight value: ").strip()
            try:
                iv = int(val)
            except ValueError:
                print("Invalid number.")
                continue
            r = tester.v4l2_set_ctrl(tester.V4L2_CID_BACKLIGHT_COMPENSATION, iv, device="/dev/video0")
            if r["ok"]:
                print(
                    f"set backlight: set={r['set_value']} readback={r['readback']} "
                    f"(cid=0x{r['cid']:08X}, {r['device']})"
                )
            else:
                print(f"set backlight failed: {r['error']}")
        elif ans == "5":
            val = input("gain value: ").strip()
            try:
                iv = int(val)
            except ValueError:
                print("Invalid number.")
                continue
            r = tester.v4l2_set_ctrl(tester.V4L2_CID_GAIN, iv, device="/dev/video0")
            if r["ok"]:
                print(
                    f"set gain: set={r['set_value']} readback={r['readback']} "
                    f"(cid=0x{r['cid']:08X}, {r['device']})"
                )
            else:
                print(f"set gain failed: {r['error']}")
        elif ans == "6":
            val = input("exposure_abs value: ").strip()
            try:
                iv = int(val)
            except ValueError:
                print("Invalid number.")
                continue
            r = tester.v4l2_set_ctrl(tester.V4L2_CID_EXPOSURE_ABSOLUTE, iv, device="/dev/video0")
            if r["ok"]:
                print(
                    f"set exposure_abs: set={r['set_value']} readback={r['readback']} "
                    f"(cid=0x{r['cid']:08X}, {r['device']})"
                )
            else:
                print(f"set exposure_abs failed: {r['error']}")
        elif ans == "7":
            print("Exposure mode values: 0=auto, 1=manual, 2=shutter-priority, 3=aperture-priority")
            val = input("exposure_auto mode (0..3): ").strip()
            try:
                iv = int(val)
            except ValueError:
                print("Invalid number.")
                continue
            r = tester.v4l2_set_ctrl(tester.V4L2_CID_EXPOSURE_AUTO, iv, device="/dev/video0")
            if r["ok"]:
                print(
                    f"set exposure_auto: set={r['set_value']} readback={r['readback']} "
                    f"(cid=0x{r['cid']:08X}, {r['device']})"
                )
            else:
                print(f"set exposure_auto failed: {r['error']}")
        elif ans == "8":
            rows = tester.camera_smi_status()
            print("SMI LED path files:")
            for row in rows:
                print(f"  {'OK' if row['exists'] else 'MISS'}  {row['file']}")
            print("Note: SMI LED calls are from Windows CVisionLib and are not directly callable from this Linux Python path.")
        elif ans == "9":
            out = tester.camera_enumerate_controls("/dev/video0")
            if not out["ok"]:
                print(f"enumerate failed: {out.get('error')}")
                continue
            rows = out["rows"]
            if not rows:
                print("No UVC controls discovered.")
                continue
            print(f"UVC controls on {out['device']} ({len(rows)}):")
            for row in rows:
                rng = f"[{row['minimum']}..{row['maximum']} step={row['step']} def={row['default']}]"
                g = row["get"]
                gtxt = str(g.get("value")) if g.get("ok") else f"read-error({g.get('error')})"
                print(
                    f"  cid=0x{row['cid']:08X} type={row['type']} flags=0x{row['flags']:08X} "
                    f"{row['name']}={gtxt} {rng}"
                )
            print("Tip: try option 10 with candidate CIDs that mention led/light/lamp/backlight.")
        elif ans == "10":
            cid_in = input("CID (hex like 0x0098091C or decimal): ").strip().lower()
            if cid_in.startswith("0x"):
                base = 16
            else:
                base = 10
            try:
                cid = int(cid_in, base)
            except ValueError:
                print("Invalid CID.")
                continue
            q = tester.v4l2_query_ctrl(cid, device="/dev/video0")
            if q["ok"]:
                print(
                    f"ctrl {q['name']} cid=0x{q['cid']:08X} "
                    f"range=[{q['minimum']}..{q['maximum']} step={q['step']} def={q['default']}] "
                    f"flags=0x{q['flags']:08X}"
                )
            else:
                print(f"query failed: {q.get('error')}")
            val_in = input("value to set: ").strip()
            try:
                val = int(val_in)
            except ValueError:
                print("Invalid number.")
                continue
            r = tester.v4l2_set_ctrl(cid, val, device="/dev/video0")
            if r["ok"]:
                print(
                    f"set raw: cid=0x{r['cid']:08X} set={r['set_value']} readback={r['readback']} "
                    f"({r['device']})"
                )
            else:
                print(f"set raw failed: {r['error']}")
        elif ans == "11":
            out = tester.camera_xu_sweep("/dev/video0")
            rows = out["rows"]
            if not rows:
                print("No XU controls discovered by GET_LEN sweep.")
                continue
            print(f"XU controls on {out['device']} ({len(rows)}):")
            for row in rows:
                info = row["info"]
                info_txt = f"0x{info:02X}" if info is not None else "NA"
                cur = row["cur"]
                cur_txt = cur["data_hex"] if cur["ok"] else f"err({cur.get('error')})"
                print(
                    f"  unit={row['unit']:02d} sel={row['selector']:02d} len={row['len']:03d} "
                    f"info={info_txt} set={row['can_set']} get={row['can_get']} cur={cur_txt}"
                )
            print("Tip: use option 12 on writable controls; this camera reports 2-byte payloads.")
            print("     Start with payloads: 0000, 0100, 0001, 0101")
        elif ans == "12":
            unit_in = input("XU unit (decimal): ").strip()
            sel_in = input("XU selector (decimal): ").strip()
            payload_in = input("payload bytes hex (e.g. 01 or 01 00): ").strip()
            try:
                unit = int(unit_in)
                selector = int(sel_in)
            except ValueError:
                print("Invalid unit/selector.")
                continue
            len_info = tester.uvc_xu_get_len(unit, selector, device="/dev/video0")
            expected_len = len_info["len"] if len_info["ok"] else None
            if expected_len is None:
                print(f"GET_LEN failed: {len_info.get('error')}")
                continue
            payload = parse_hex_bytes(payload_in) if payload_in else None
            do_set = payload is not None
            if payload_in and payload is None:
                print("Invalid payload hex.")
                continue
            if do_set and len(payload) != expected_len:
                original = bytes(payload).hex()
                if len(payload) < expected_len:
                    payload = payload + [0x00] * (expected_len - len(payload))
                else:
                    payload = payload[:expected_len]
                print(
                    f"payload adjusted to len={expected_len}: {original} -> {bytes(payload).hex()}"
                )
            s = tester.uvc_xu_query(
                unit,
                selector,
                tester.UVC_SET_CUR,
                expected_len,
                device="/dev/video0",
                data=payload,
            ) if do_set else None
            g = tester.uvc_xu_query(
                unit,
                selector,
                tester.UVC_GET_CUR,
                expected_len,
                device="/dev/video0",
            )
            if do_set and s["ok"]:
                print(
                    f"XU SET_CUR ok: unit={unit} sel={selector} payload={bytes(payload).hex()} "
                    f"size={expected_len}"
                )
            elif do_set:
                print(f"XU SET_CUR failed: {s.get('error')}")
            else:
                print(f"XU SET_CUR skipped (read-only probe). expected_len={expected_len}")
            if g["ok"]:
                print(f"XU GET_CUR ok: unit={unit} sel={selector} data={g['data_hex']}")
            else:
                print(f"XU GET_CUR failed: {g.get('error')}")
        elif ans == "13":
            print("Running auto XU safe sweep...")
            out = tester.camera_xu_auto_safe_sweep("/dev/video0")
            rows = out.get("rows", [])
            if not rows:
                print("No writable XU controls discovered.")
                continue
            print(f"Auto XU results on {out['device']} ({len(rows)} controls):")
            for row in rows:
                base = row["baseline"]
                base_txt = base["data_hex"] if base.get("ok") else f"err({base.get('error')})"
                print(
                    f"  unit={row['unit']:02d} sel={row['selector']:02d} len={row['len']:03d} "
                    f"baseline={base_txt}"
                )
                for t in row["tests"]:
                    s_txt = "ok" if t["set"].get("ok") else f"err({t['set'].get('error')})"
                    g_txt = t["get"]["data_hex"] if t["get"].get("ok") else f"err({t['get'].get('error')})"
                    delta = ""
                    if base.get("ok") and t["get"].get("ok"):
                        delta = " changed" if t["get"]["data_hex"] != base["data_hex"] else " same"
                    print(f"    payload={t['payload_hex']} set={s_txt} get={g_txt}{delta}")
                rs = row.get("restore_set")
                rg = row.get("restore_get")
                if rs is not None:
                    rs_txt = "ok" if rs.get("ok") else f"err({rs.get('error')})"
                    if rg is not None:
                        rg_txt = rg["data_hex"] if rg.get("ok") else f"err({rg.get('error')})"
                    else:
                        rg_txt = "n/a"
                    print(f"    restore: set={rs_txt} get={rg_txt}")
            skipped = out.get("skipped", [])
            if skipped:
                print(
                    f"  skipped high-len controls: "
                    + ", ".join(
                        f"u{int(r['unit'])}/s{int(r['selector'])}/len{int(r['len'])}"
                        for r in skipped
                    )
                )
        elif ans == "14":
            with_stats = tester._ffmpeg_available()
            if with_stats:
                print("Running selector profile with ffmpeg signalstats...")
            else:
                print("Running selector profile (ffmpeg missing, register-only mode)...")
            out = tester.camera_xu_selector_profile(
                unit=2,
                selector=1,
                device="/dev/video0",
                with_signalstats=with_stats,
            )
            if not out.get("ok"):
                print(f"profile failed: {out.get('error')}")
                continue
            base = out["baseline"]
            base_txt = short_hex(base["data_hex"]) if base.get("ok") else f"err({base.get('error')})"
            print(
                f"profile unit={out['unit']} sel={out['selector']} len={out['len']} "
                f"mode={out.get('len_note')} baseline={base_txt}"
            )
            for row in out["rows"]:
                s_txt = "ok" if row["set"].get("ok") else f"err({row['set'].get('error')})"
                g_txt = short_hex(row["get"]["data_hex"]) if row["get"].get("ok") else f"err({row['get'].get('error')})"
                change = ""
                if base.get("ok") and row["get"].get("ok"):
                    change = " changed" if row["get"]["data_hex"] != base["data_hex"] else " same"
                stats = row.get("stats")
                stats_txt = ""
                if stats is not None:
                    if stats.get("ok"):
                        stats_txt = f" yavg={stats.get('yavg')} satavg={stats.get('satavg')}"
                    else:
                        stats_txt = f" stats_err={stats.get('error')}"
                print(f"  payload={short_hex(row['payload_hex'])} set={s_txt} get={g_txt}{change}{stats_txt}")
            rs = out.get("restore_set")
            rg = out.get("restore_get")
            if rs is not None:
                rs_txt = "ok" if rs.get("ok") else f"err({rs.get('error')})"
                rg_txt = (
                    short_hex(rg["data_hex"])
                    if (rg is not None and rg.get("ok"))
                    else f"err({rg.get('error') if rg else 'n/a'})"
                )
                print(f"  restore: set={rs_txt} get={rg_txt}")
        elif ans == "15":
            print("Resetting camera USB device...")
            pick = tester.camera_wait_pick_device(
                preferred="/dev/video0",
                timeout_s=8.0,
                poll_s=0.25,
                require_controls=False,
            )
            if not pick.get("ok"):
                print(f"camera reset failed: {pick.get('error')}")
                continue
            cam_dev = pick["device"]
            r = tester.camera_usb_soft_reset(cam_dev)
            if not r.get("ok"):
                print(f"camera reset failed: {r.get('error')}")
                continue
            pick2 = tester.camera_wait_pick_device(
                preferred=cam_dev,
                timeout_s=8.0,
                poll_s=0.25,
                require_controls=False,
            )
            if pick2.get("ok"):
                cam_dev = pick2["device"]
                print(f"camera re-enumerated as: {cam_dev}")
            else:
                print(f"camera re-enumeration wait failed: {pick2.get('error')}")
                continue
            print(
                "camera reset ok: "
                f"usb={r.get('usb_device_path')} "
                f"iface={r.get('interface_path')}"
            )
            out = tester.camera_xu_sweep(cam_dev)
            rows = out.get("rows", [])
            if not rows:
                print("XU rescan: no controls found.")
            else:
                print(f"XU rescan on {cam_dev} ({len(rows)} controls):")
                for row in rows:
                    info = row["info"]
                    info_txt = f"0x{info:02X}" if info is not None else "NA"
                    cur = row["cur"]
                    cur_txt = cur["data_hex"] if cur["ok"] else f"err({cur.get('error')})"
                    print(
                        f"  unit={row['unit']:02d} sel={row['selector']:02d} len={row['len']:03d} "
                        f"info={info_txt} set={row['can_set']} get={row['can_get']} cur={short_hex(cur_txt)}"
                    )
        elif ans == "16":
            print("Running one-click camera automation...")
            out = tester.camera_auto_oneclick("/dev/video0", max_resets=2)
            for line in out.get("log", []):
                print(f"  {line}")
            if not out.get("ok"):
                print(f"auto failed: {out.get('error')}")
                rows = out.get("sweep_rows", [])
                if rows:
                    print("last sweep snapshot:")
                    for row in rows[:8]:
                        info = row["info"]
                        info_txt = f"0x{info:02X}" if info is not None else "NA"
                        cur = row["cur"]
                        cur_txt = cur["data_hex"] if cur["ok"] else f"err({cur.get('error')})"
                        print(
                            f"  unit={int(row['unit']):02d} sel={int(row['selector']):02d} "
                            f"len={int(row['len']):03d} info={info_txt} cur={short_hex(cur_txt)}"
                        )
                continue
            chosen = out["chosen"]
            print(
                f"auto chosen control: unit={int(chosen['unit'])} "
                f"sel={int(chosen['selector'])} len={int(chosen['len'])}"
            )
            safe = out.get("safe", {})
            for row in safe.get("rows", []):
                base = row["baseline"]
                base_txt = short_hex(base["data_hex"]) if base.get("ok") else f"err({base.get('error')})"
                print(
                    f"  safe baseline unit={int(row['unit'])} sel={int(row['selector'])} "
                    f"len={int(row['len'])} base={base_txt}"
                )
                for t in row.get("tests", []):
                    s_txt = "ok" if t["set"].get("ok") else f"err({t['set'].get('error')})"
                    g_txt = short_hex(t["get"]["data_hex"]) if t["get"].get("ok") else f"err({t['get'].get('error')})"
                    print(f"    payload={short_hex(t['payload_hex'])} set={s_txt} get={g_txt}")
            profile = out.get("profile", {})
            if not profile.get("ok"):
                print(f"profile result: fail {profile.get('error')}")
            else:
                base = profile["baseline"]
                base_txt = short_hex(base["data_hex"]) if base.get("ok") else f"err({base.get('error')})"
                print(
                    f"profile result: unit={profile['unit']} sel={profile['selector']} "
                    f"len={profile['len']} baseline={base_txt}"
                )
                y_rows = []
                for row in profile.get("rows", []):
                    s_txt = "ok" if row["set"].get("ok") else f"err({row['set'].get('error')})"
                    g_txt = short_hex(row["get"]["data_hex"]) if row["get"].get("ok") else f"err({row['get'].get('error')})"
                    stats = row.get("stats")
                    if stats and stats.get("ok"):
                        st = f" yavg={stats.get('yavg')} satavg={stats.get('satavg')}"
                        y_rows.append((row["payload_hex"], float(stats.get("yavg"))))
                    elif stats:
                        st = f" stats_err={stats.get('error')}"
                    else:
                        st = ""
                    print(f"    payload={short_hex(row['payload_hex'])} set={s_txt} get={g_txt}{st}")
                if y_rows:
                    y_sorted = sorted(y_rows, key=lambda x: x[1])
                    min_p, min_y = y_sorted[0]
                    max_p, max_y = y_sorted[-1]
                    dy = max_y - min_y
                    print(
                        f"  yavg summary: min={min_y:.3f} ({short_hex(min_p)}) "
                        f"max={max_y:.3f} ({short_hex(max_p)}) delta={dy:.3f}"
                    )
        else:
            print("Invalid.")


def run_thermal_cycler_menu(tester):
    def _bank_label(bank):
        return "NEST" if int(bank) == tester.THERMAL_BANK_NEST else "LID"

    def _fmt_gp(row):
        if row is None:
            return "n/a"
        if not row.get("ok"):
            return f"err({fmt_resp(row.get('ack'))})"
        if row.get("scaled") is None:
            return str(row.get("value"))
        return f"{row['scaled']:.3f} (raw={row['value']})"

    while True:
        print("\n----------------------------------------")
        print(" THERMAL CYCLER (0x06 OEM COMMAND MAP)")
        print("----------------------------------------")
        print("  1. Reachability check (activate + firmware + temps)")
        print("  2. Telemetry snapshot (temps + GP params)")
        print("  3. Apply vendor baseline (fan=190, cool=-0.050, heat=0.050)")
        print(" 15. Apply FAST profile (fan=225, cool=-0.125, heat=0.125)")
        print("  4. Set NEST target temp (C)")
        print("  5. Set LID target temp (C)")
        print("  6. Set pedestal target temp (C)")
        print("  7. Set TC fan speed (0..255)")
        print("  8. Set NEST rates (cool<=0, heat>=0)")
        print("  9. Set LID rates (cool<=0, heat>=0)")
        print(" 10. Set NEST PWM override (0..100)")
        print(" 11. Set LID PWM override (0..100)")
        print(" 12. GP read (param, bank)")
        print(" 13. GP write safe (param, bank, value)")
        print(" 14. HARD RESET / PANIC OFF")
        print("  b. Back")
        print("  q. Quit")

        ans = input("\n[THERM] > ").strip().lower()
        if ans == "":
            continue
        if ans == "b":
            return
        if ans == "q":
            sys.exit(0)

        if ans == "1":
            a = tester.thermal_activate()
            fw = tester.thermal_query_firmware()
            temps = tester.thermal_read_temps()
            print(f"activate 0x06: {fmt_resp(a.get('ack'))} ok={a.get('ok')}")
            fw_txt = fw.get("fw_hex") or "n/a"
            print(f"firmware cmd173: {fmt_resp(fw.get('ack'))} fw={fw_txt}")
            for label in ("tc_temp_c", "lid_temp_c", "ped_temp_c"):
                row = temps.get(label, {})
                if row.get("temp_c") is None:
                    print(f"  {label}: err({fmt_resp(row.get('ack'))})")
                else:
                    print(f"  {label}: {row['temp_c']:.3f} C ({fmt_resp(row.get('ack'))})")
        elif ans == "2":
            snap = tester.thermal_snapshot()
            print(
                f"activate: {fmt_resp(snap['activate'].get('ack'))} "
                f"fw: {snap['firmware'].get('fw_hex') or 'n/a'} ({fmt_resp(snap['firmware'].get('ack'))})"
            )
            print("temperatures:")
            for label in ("tc_temp_c", "lid_temp_c", "ped_temp_c"):
                row = snap["temps"].get(label, {})
                if row.get("temp_c") is None:
                    print(f"  {label}: err({fmt_resp(row.get('ack'))})")
                else:
                    print(f"  {label}: {row['temp_c']:.3f} C")
            if not snap.get("alive", False):
                print("thermal board not alive for GP polling; skipped GP reads to avoid bus spam.")
                continue
            for bank in (tester.THERMAL_BANK_NEST, tester.THERMAL_BANK_LID):
                print(f"{_bank_label(bank)} GP:")
                for row in snap["gp"].get(bank, []):
                    print(
                        f"  p{int(row['param']):02d}="
                        f"{_fmt_gp(row)} ({fmt_resp(row.get('ack'))})"
                    )
        elif ans == "3":
            out = tester.thermal_apply_vendor_baseline(
                fan_speed=190,
                cool_rate_c_s=-0.050,
                heat_rate_c_s=0.050,
            )
            print(
                f"baseline: ok={out.get('ok')} activate={fmt_resp(out['activate'].get('ack'))} "
                f"activate_ok={out.get('activate_ok')} fan_ack={fmt_resp(out.get('fan', {}).get('ack'))}"
            )
            for bank, row in out.get("banks", {}).items():
                rates = row.get("rates", {})
                print(
                    f"  {_bank_label(bank)} rates_ok={rates.get('ok')} "
                    f"cool={rates.get('cool_rate_c_s')} heat={rates.get('heat_rate_c_s')}"
                )
        elif ans == "15":
            out = tester.thermal_apply_fast_profile(
                fan_speed=225,
                cool_rate_c_s=-0.125,
                heat_rate_c_s=0.125,
            )
            print(
                f"fast profile: ok={out.get('ok')} activate={fmt_resp(out['activate'].get('ack'))} "
                f"activate_ok={out.get('activate_ok')} fan_ack={fmt_resp(out.get('fan', {}).get('ack'))}"
            )
            for bank, row in out.get("banks", {}).items():
                rates = row.get("rates", {})
                print(
                    f"  {_bank_label(bank)} rates_ok={rates.get('ok')} "
                    f"cool={rates.get('cool_rate_c_s')} heat={rates.get('heat_rate_c_s')}"
                )
        elif ans in ("4", "5"):
            bank = tester.THERMAL_BANK_NEST if ans == "4" else tester.THERMAL_BANK_LID
            temp_in = input(f"{_bank_label(bank)} target temp C (0..120): ").strip()
            try:
                temp_c = float(temp_in)
            except ValueError:
                print("Invalid number.")
                continue
            out = tester.thermal_set_target_temp(bank, temp_c)
            if not out.get("ok") and out.get("error"):
                print(f"set temp failed: {out.get('error')}")
                continue
            ref = out.get("reference", {})
            ref_txt = _fmt_gp(ref) if isinstance(ref, dict) else "n/a"
            print(
                f"{_bank_label(bank)} set temp {out['target_c']:.3f} C: "
                f"ack={fmt_resp(out.get('ack'))} ref(p22)={ref_txt}"
            )
        elif ans == "6":
            temp_in = input("pedestal target temp C (0..120): ").strip()
            try:
                temp_c = float(temp_in)
            except ValueError:
                print("Invalid number.")
                continue
            out = tester.thermal_set_ped_temp(temp_c)
            if not out.get("ok") and out.get("error"):
                print(f"set pedestal temp failed: {out.get('error')}")
                continue
            print(f"pedestal set temp {out['target_c']:.3f} C: ack={fmt_resp(out.get('ack'))}")
        elif ans == "7":
            speed_in = input("TC fan speed (0..255): ").strip()
            try:
                speed = int(speed_in)
            except ValueError:
                print("Invalid number.")
                continue
            out = tester.thermal_set_fan(speed, verify=True)
            if not out.get("ok") and out.get("error"):
                print(f"set fan failed: {out.get('error')}")
                continue
            rb = out.get("readback", {})
            print(f"fan={speed}: ack={fmt_resp(out.get('ack'))} gp21={_fmt_gp(rb)}")
        elif ans in ("8", "9"):
            bank = tester.THERMAL_BANK_NEST if ans == "8" else tester.THERMAL_BANK_LID
            cool_in = input(f"{_bank_label(bank)} cool rate C/s (<=0, e.g. -0.050): ").strip()
            heat_in = input(f"{_bank_label(bank)} heat rate C/s (>=0, e.g. 0.050): ").strip()
            try:
                cool = float(cool_in)
                heat = float(heat_in)
            except ValueError:
                print("Invalid number.")
                continue
            out = tester.thermal_set_rates(bank, cool_rate_c_s=cool, heat_rate_c_s=heat)
            if not out.get("ok") and out.get("error"):
                print(f"set rates failed: {out.get('error')}")
                continue
            cool_rb = out.get("cool", {}).get("readback", {})
            heat_rb = out.get("heat", {}).get("readback", {})
            print(
                f"{_bank_label(bank)} rates: cool={cool} heat={heat} "
                f"cool_rb={_fmt_gp(cool_rb)} heat_rb={_fmt_gp(heat_rb)}"
            )
        elif ans in ("10", "11"):
            bank = tester.THERMAL_BANK_NEST if ans == "10" else tester.THERMAL_BANK_LID
            pwm_in = input(f"{_bank_label(bank)} PWM override (0..100): ").strip()
            try:
                pwm = int(pwm_in)
            except ValueError:
                print("Invalid number.")
                continue
            out = tester.thermal_set_pwm(bank, pwm)
            if not out.get("ok") and out.get("error"):
                print(f"set pwm failed: {out.get('error')}")
                continue
            rb = out.get("readback", {})
            print(
                f"{_bank_label(bank)} pwm={pwm}: ack={fmt_resp(out.get('ack'))} "
                f"gp23={_fmt_gp(rb)}"
            )
        elif ans == "12":
            p_in = input("GP param (decimal): ").strip()
            b_in = input("bank (0=NEST,1=LID): ").strip()
            try:
                param = int(p_in)
                bank = int(b_in)
            except ValueError:
                print("Invalid param/bank.")
                continue
            out = tester.thermal_gp_read(param, bank)
            print(
                f"GP read p{param} bank{bank}: {_fmt_gp(out)} "
                f"ack={fmt_resp(out.get('ack'))}"
            )
        elif ans == "13":
            p_in = input("GP param (decimal): ").strip()
            b_in = input("bank (0=NEST,1=LID): ").strip()
            v_in = input("value (int32): ").strip()
            try:
                param = int(p_in)
                bank = int(b_in)
                value = int(v_in)
            except ValueError:
                print("Invalid param/bank/value.")
                continue
            if param not in tester.THERMAL_SAFE_WRITE_PARAMS:
                print(
                    "Blocked: param not in safe set "
                    f"{sorted(tester.THERMAL_SAFE_WRITE_PARAMS)}"
                )
                continue
            out = tester.thermal_gp_write(param, bank, value, verify=True)
            rb = out.get("readback", {})
            print(
                f"GP write p{param} bank{bank}={value}: "
                f"ack={fmt_resp(out.get('ack'))} rb={_fmt_gp(rb)}"
            )
        elif ans == "14":
            print("Running hard thermal reset (USB reconnect + panic off sequence)...")
            out = tester.thermal_hard_reset(rounds=3)
            for row in out.get("rounds", []):
                fw = row.get("firmware", {})
                fw_txt = fw.get("fw_hex") or "n/a"
                print(
                    f"  round#{row.get('round')}: "
                    f"fw={fmt_resp(fw.get('ack'))}({fw_txt}) "
                    f"temps_ok={row.get('temps_ok')} alive={row.get('alive')}"
                )
                for op in row.get("ops", []):
                    print(f"    {op.get('name')}: sent={op.get('sent')}/{op.get('attempted')}")
            if out.get("ok"):
                print("Hard reset recovered thermal board comms.")
            else:
                print("Hard reset did not recover comms. Try main menu 's', then power-cycle instrument/CAN adapter.")
        else:
            print("Invalid.")


def run_chiller_menu(tester):
    def _bank_label(bank):
        return "RC" if int(bank) == tester.CHILLER_BANK_RC else "OC"

    def _fmt_gp(row):
        if row is None:
            return "n/a"
        if not row.get("ok"):
            return f"err({fmt_resp(row.get('ack'))})"
        if row.get("scaled") is None:
            return str(row.get("value"))
        return f"{row['scaled']:.3f} (raw={row['value']})"

    while True:
        print("\n----------------------------------------")
        print(" CHILLER SYSTEM (OEM COMMAND MAP)")
        print("----------------------------------------")
        print("  1. Reachability check (activate + firmware + temps)")
        print("  2. Telemetry snapshot (temps + GP params)")
        print("  3. Apply vendor baseline (fan=250, cool=-0.125, heat=0.125)")
        print("  4. Set RC target temp (C)")
        print("  5. Set OC target temp (C)")
        print("  6. Set RC fan speed (0..255)")
        print("  7. Set OC fan speed (0..255)")
        print("  8. Set RC rates (cool<=0, heat>=0)")
        print("  9. Set OC rates (cool<=0, heat>=0)")
        print(" 10. Set RC PWM override (0..100)")
        print(" 11. Set OC PWM override (0..100)")
        print(" 12. GP read (param, bank)")
        print(" 13. GP write safe (param, bank, value)")
        print(" 14. HARD RESET / PANIC OFF")
        print("  b. Back")
        print("  q. Quit")

        ans = input("\n[CHILLER] > ").strip().lower()
        if ans == "":
            continue
        if ans == "b":
            return
        if ans == "q":
            sys.exit(0)

        if ans == "1":
            a = tester.chiller_activate()
            fw = tester.chiller_query_firmware()
            temps = tester.chiller_read_temps()
            print(f"activate 0x07: {fmt_resp(a.get('ack'))} ok={a.get('ok')}")
            fw_txt = fw.get("fw_hex") or "n/a"
            print(f"firmware cmd173: {fmt_resp(fw.get('ack'))} fw={fw_txt}")
            for label, _axis in tester.CHILLER_TEMP_AXES:
                row = temps.get(label, {})
                if row.get("temp_c") is None:
                    print(f"  {label}: err({fmt_resp(row.get('ack'))})")
                else:
                    print(f"  {label}: {row['temp_c']:.3f} C ({fmt_resp(row.get('ack'))})")
        elif ans == "2":
            snap = tester.chiller_snapshot()
            print(
                f"activate: {fmt_resp(snap['activate'].get('ack'))} "
                f"fw: {snap['firmware'].get('fw_hex') or 'n/a'} ({fmt_resp(snap['firmware'].get('ack'))})"
            )
            print("temperatures:")
            for label, _axis in tester.CHILLER_TEMP_AXES:
                row = snap["temps"].get(label, {})
                if row.get("temp_c") is None:
                    print(f"  {label}: err({fmt_resp(row.get('ack'))})")
                else:
                    print(f"  {label}: {row['temp_c']:.3f} C")
            if not snap.get("alive", False):
                print("chiller not alive for GP polling; skipped GP reads to avoid bus spam.")
                continue
            for bank in (tester.CHILLER_BANK_RC, tester.CHILLER_BANK_OC):
                print(f"{_bank_label(bank)} GP:")
                for row in snap["gp"].get(bank, []):
                    print(
                        f"  p{int(row['param']):02d}="
                        f"{_fmt_gp(row)} ({fmt_resp(row.get('ack'))})"
                    )
        elif ans == "3":
            out = tester.chiller_apply_vendor_baseline(
                fan_speed=250,
                cool_rate_c_s=-0.125,
                heat_rate_c_s=0.125,
            )
            print(
                f"baseline: ok={out.get('ok')} activate={fmt_resp(out['activate'].get('ack'))} "
                f"activate_ok={out.get('activate_ok')}"
            )
            for bank, row in out.get("banks", {}).items():
                rates = row.get("rates", {})
                fan = row.get("fan", {})
                print(
                    f"  {_bank_label(bank)} rates_ok={rates.get('ok')} fan_ok={fan.get('ok')} "
                    f"fan_ack={fmt_resp(fan.get('ack'))}"
                )
        elif ans in ("4", "5"):
            bank = tester.CHILLER_BANK_RC if ans == "4" else tester.CHILLER_BANK_OC
            temp_in = input(f"{_bank_label(bank)} target temp C (0..60): ").strip()
            try:
                temp_c = float(temp_in)
            except ValueError:
                print("Invalid number.")
                continue
            out = tester.chiller_set_target_temp(bank, temp_c)
            if not out.get("ok") and out.get("error"):
                print(f"set temp failed: {out.get('error')}")
                continue
            ref = out.get("reference", {})
            ref_txt = _fmt_gp(ref) if isinstance(ref, dict) else "n/a"
            print(
                f"{_bank_label(bank)} set temp {out['target_c']:.3f} C: "
                f"ack={fmt_resp(out.get('ack'))} ref(p22)={ref_txt}"
            )
        elif ans in ("6", "7"):
            bank = tester.CHILLER_BANK_RC if ans == "6" else tester.CHILLER_BANK_OC
            speed_in = input(f"{_bank_label(bank)} fan speed (0..255): ").strip()
            try:
                speed = int(speed_in)
            except ValueError:
                print("Invalid number.")
                continue
            out = tester.chiller_set_fan(bank, speed)
            if not out.get("ok") and out.get("error"):
                print(f"set fan failed: {out.get('error')}")
                continue
            rb = out.get("readback", {})
            print(
                f"{_bank_label(bank)} fan={speed}: ack={fmt_resp(out.get('ack'))} "
                f"gp21={_fmt_gp(rb)}"
            )
        elif ans in ("8", "9"):
            bank = tester.CHILLER_BANK_RC if ans == "8" else tester.CHILLER_BANK_OC
            cool_in = input(f"{_bank_label(bank)} cool rate C/s (<=0, e.g. -0.125): ").strip()
            heat_in = input(f"{_bank_label(bank)} heat rate C/s (>=0, e.g. 0.125): ").strip()
            try:
                cool = float(cool_in)
                heat = float(heat_in)
            except ValueError:
                print("Invalid number.")
                continue
            out = tester.chiller_set_rates(bank, cool_rate_c_s=cool, heat_rate_c_s=heat)
            if not out.get("ok") and out.get("error"):
                print(f"set rates failed: {out.get('error')}")
                continue
            cool_rb = out.get("cool", {}).get("readback", {})
            heat_rb = out.get("heat", {}).get("readback", {})
            print(
                f"{_bank_label(bank)} rates: cool={cool} heat={heat} "
                f"cool_rb={_fmt_gp(cool_rb)} heat_rb={_fmt_gp(heat_rb)}"
            )
        elif ans in ("10", "11"):
            bank = tester.CHILLER_BANK_RC if ans == "10" else tester.CHILLER_BANK_OC
            pwm_in = input(f"{_bank_label(bank)} PWM override (0..100): ").strip()
            try:
                pwm = int(pwm_in)
            except ValueError:
                print("Invalid number.")
                continue
            out = tester.chiller_set_pwm(bank, pwm)
            if not out.get("ok") and out.get("error"):
                print(f"set pwm failed: {out.get('error')}")
                continue
            rb = out.get("readback", {})
            print(
                f"{_bank_label(bank)} pwm={pwm}: ack={fmt_resp(out.get('ack'))} "
                f"gp23={_fmt_gp(rb)}"
            )
        elif ans == "12":
            p_in = input("GP param (decimal): ").strip()
            b_in = input("bank (0=RC,1=OC): ").strip()
            try:
                param = int(p_in)
                bank = int(b_in)
            except ValueError:
                print("Invalid param/bank.")
                continue
            out = tester.chiller_gp_read(param, bank)
            print(
                f"GP read p{param} bank{bank}: {_fmt_gp(out)} "
                f"ack={fmt_resp(out.get('ack'))}"
            )
        elif ans == "13":
            p_in = input("GP param (decimal): ").strip()
            b_in = input("bank (0=RC,1=OC): ").strip()
            v_in = input("value (int32): ").strip()
            try:
                param = int(p_in)
                bank = int(b_in)
                value = int(v_in)
            except ValueError:
                print("Invalid param/bank/value.")
                continue
            if param not in tester.CHILLER_SAFE_WRITE_PARAMS:
                print(
                    "Blocked: param not in safe set "
                    f"{sorted(tester.CHILLER_SAFE_WRITE_PARAMS)}"
                )
                continue
            out = tester.chiller_gp_write(param, bank, value, verify=True)
            rb = out.get("readback", {})
            print(
                f"GP write p{param} bank{bank}={value}: "
                f"ack={fmt_resp(out.get('ack'))} rb={_fmt_gp(rb)}"
            )
        elif ans == "14":
            print("Running hard chiller reset (USB reconnect + panic off sequence)...")
            out = tester.chiller_hard_reset(rounds=3)
            for row in out.get("rounds", []):
                fw = row.get("firmware", {})
                fw_txt = fw.get("fw_hex") or "n/a"
                print(
                    f"  round#{row.get('round')}: "
                    f"fw={fmt_resp(fw.get('ack'))}({fw_txt}) "
                    f"temps_ok={row.get('temps_ok')} alive={row.get('alive')}"
                )
                for op in row.get("ops", []):
                    print(f"    {op.get('name')}: sent={op.get('sent')}/{op.get('attempted')}")
            if out.get("ok"):
                print("Hard reset recovered chiller comms.")
            else:
                print("Hard reset did not recover comms. Try main menu 's', then power-cycle instrument/CAN adapter.")
        else:
            print("Invalid.")


def run_camera_menu(tester):
    last_snapshot = None
    while True:
        print("\n----------------------------------------")
        print(" CAMERA SYSTEM")
        print("----------------------------------------")
        print("  1. AUTO CAMERA CHECK (recover + control)")
        print("  2. CAPTURE SNAPSHOT (save JPG)")
        print("  3. STREAM HEALTH TEST (5s)")
        print("  4. LIST CAMERA DEVICES")
        print("  a. ADVANCED CAMERA MENU")
        print("  b. Back")
        print("  q. Quit")
        if last_snapshot:
            print(f"  Last snapshot: {last_snapshot}")

        ans = input("\n[CAM] > ").strip().lower()
        if ans == "":
            continue
        if ans == "b":
            return
        if ans == "q":
            sys.exit(0)
        if ans == "1":
            print("Running one-click camera automation...")
            out = tester.camera_auto_oneclick("/dev/video0", max_resets=2)
            for line in out.get("log", []):
                print(f"  {line}")
            if not out.get("ok"):
                print(f"auto failed: {out.get('error')}")
                continue
            chosen = out.get("chosen", {})
            print(
                f"auto chosen control: unit={int(chosen.get('unit', 0))} "
                f"sel={int(chosen.get('selector', 0))} len={int(chosen.get('len', 0))}"
            )
            profile = out.get("profile", {})
            if not profile.get("ok"):
                print(f"profile result: fail {profile.get('error')}")
                continue
            y_rows = []
            for row in profile.get("rows", []):
                stats = row.get("stats")
                if stats and stats.get("ok"):
                    y_rows.append((row.get("payload_hex", ""), float(stats.get("yavg"))))
            if y_rows:
                y_sorted = sorted(y_rows, key=lambda x: x[1])
                min_p, min_y = y_sorted[0]
                max_p, max_y = y_sorted[-1]
                print(
                    f"yavg summary: min={min_y:.3f} ({short_hex(min_p)}) "
                    f"max={max_y:.3f} ({short_hex(max_p)}) delta={max_y - min_y:.3f}"
                )
            else:
                print("yavg summary: no usable frame stats in this run.")
            print("Use 'a' for detailed per-payload diagnostics.")
        elif ans == "2":
            print("Capturing snapshot...")
            r = tester.camera_capture_snapshot("/dev/video0")
            if r.get("ok"):
                last_snapshot = r.get("path")
                print(
                    f"snapshot saved: {r.get('path')} "
                    f"(size={r.get('size')} bytes, device={r.get('device')})"
                )
            else:
                print(f"snapshot failed: {r.get('error')}")
                out = str(r.get("output", "")).strip()
                if out:
                    print(short_hex(out, limit=160))
        elif ans == "3":
            print("Running stream health test (5s)...")
            r = tester.camera_stream_health("/dev/video0", seconds=5)
            if r.get("ok"):
                fps_txt = f"{r.get('fps_last')}" if r.get("fps_last") is not None else "n/a"
                print(
                    f"stream ok: device={r.get('device')} frames={r.get('frames')} "
                    f"duration={r.get('seconds')}s fps(last)={fps_txt}"
                )
            else:
                print(f"stream failed: {r.get('error')}")
                tail = r.get("output_tail")
                if tail:
                    print(tail)
        elif ans == "4":
            rows = tester.camera_device_rows()
            if not rows:
                print("No /dev/video* devices found.")
            else:
                print("Camera devices:")
                for row in rows:
                    name = row.get("name", "")
                    if name:
                        print(f"  {row['device']}: {name}")
                    else:
                        print(f"  {row['device']}")
        elif ans == "a":
            run_camera_menu_advanced(tester)
        else:
            print("Invalid.")


def run_led_preset_menu(tester):
    while True:
        print("\n----------------------------------------")
        print(" LED PRESET COLORS")
        print("----------------------------------------")
        persisted = tester.led_state_cached(refresh=True)
        if persisted is None:
            print("  Persisted default: none")
        else:
            pname = persisted.get("preset") or "custom"
            print(f"  Persisted default: {pname} rgb={persisted['rgb']}")
        for idx, row in enumerate(LED_COLOR_PRESETS, start=1):
            name, rgb = row
            print(f"  {idx:>2}. {name:<12} rgb={rgb}")
        print("   b. Back")
        print("   q. Quit")

        ans = input("\n[LED/PRESET] > ").strip().lower()
        if ans == "":
            continue
        if ans == "b":
            return
        if ans == "q":
            sys.exit(0)
        try:
            pick = int(ans)
        except ValueError:
            print("Invalid.")
            continue
        if pick < 1 or pick > len(LED_COLOR_PRESETS):
            print("Invalid.")
            continue
        name, rgb = LED_COLOR_PRESETS[pick - 1]
        r = tester.strip_set_rgb(rgb[0], rgb[1], rgb[2], reconnect_first=True)
        print(
            f"PRESET {name}: rgb={r['rgb']} tmcl={r['tmcl']} ack=[{fmt_rgb_acks(r['acks'])}] "
            f"sent={r['sent']} time={r['elapsed_ms']}ms"
        )
        _persist_led_choice(tester, r["rgb"], preset=name)


def run_led_menu(tester):
    while True:
        print("\n----------------------------------------")
        print(" LED CONTROL (MAIN STRIP)")
        print("----------------------------------------")
        persisted = tester.led_state_cached(refresh=True)
        if persisted is None:
            print("  Persisted default: none")
        else:
            pname = persisted.get("preset") or "custom"
            print(f"  Persisted default: {pname} rgb={persisted['rgb']}")
        print("  1. ON")
        print("  2. OFF")
        print("  3. LOW (25%)")
        print("  4. MED (50%)")
        print("  5. HIGH (75%)")
        print("  6. MAX (100%)")
        print("  7. CUSTOM LEVEL (0..100%)")
        print("  8. CUSTOM RGB (R,G,B 0..255)")
        print("  9. RE-PRIME (leave ON)")
        print("  a. RAINBOW CYCLE")
        print("  p. PRESET COLOR LIST")
        print("  r. RE-APPLY PERSISTED DEFAULT")
        print("  b. Back")
        print("  q. Quit")

        ans = input("\n[LED] > ").strip().lower()
        if ans == "":
            continue
        if ans == "b":
            return
        if ans == "q":
            sys.exit(0)
        if ans == "1":
            r = tester.strip_on()
            print(
                f"STRIP ON: rgb={r['rgb']} tmcl={r['tmcl']} ack=[{fmt_rgb_acks(r['acks'])}] "
                f"sent={r['sent']} time={r['elapsed_ms']}ms"
            )
            _persist_led_choice(tester, r["rgb"], preset="WHITE")
        elif ans == "2":
            r = tester.strip_off()
            print(
                f"STRIP OFF: rgb={r['rgb']} tmcl={r['tmcl']} ack=[{fmt_rgb_acks(r['acks'])}] "
                f"sent={r['sent']} time={r['elapsed_ms']}ms"
            )
            _persist_led_choice(tester, r["rgb"], preset="OFF")
        elif ans == "3":
            r = tester.strip_set_pct(25)
            print(
                f"STRIP LOW: pct={r['pct']} rgb={r['rgb']} tmcl={r['tmcl']} ack=[{fmt_rgb_acks(r['acks'])}] "
                f"sent={r['sent']} time={r['elapsed_ms']}ms"
            )
            _persist_led_choice(tester, r["rgb"], preset="LOW_25")
        elif ans == "4":
            r = tester.strip_set_pct(50)
            print(
                f"STRIP MED: pct={r['pct']} rgb={r['rgb']} tmcl={r['tmcl']} ack=[{fmt_rgb_acks(r['acks'])}] "
                f"sent={r['sent']} time={r['elapsed_ms']}ms"
            )
            _persist_led_choice(tester, r["rgb"], preset="MED_50")
        elif ans == "5":
            r = tester.strip_set_pct(75)
            print(
                f"STRIP HIGH: pct={r['pct']} rgb={r['rgb']} tmcl={r['tmcl']} ack=[{fmt_rgb_acks(r['acks'])}] "
                f"sent={r['sent']} time={r['elapsed_ms']}ms"
            )
            _persist_led_choice(tester, r["rgb"], preset="HIGH_75")
        elif ans == "6":
            r = tester.strip_set_pct(100)
            print(
                f"STRIP MAX: pct={r['pct']} rgb={r['rgb']} tmcl={r['tmcl']} ack=[{fmt_rgb_acks(r['acks'])}] "
                f"sent={r['sent']} time={r['elapsed_ms']}ms"
            )
            _persist_led_choice(tester, r["rgb"], preset="MAX_100")
        elif ans == "7":
            pct_in = input("Strip level percent (0..100): ").strip()
            try:
                pct_val = int(pct_in)
            except ValueError:
                print("Invalid number.")
                continue
            r = tester.strip_set_pct(pct_val)
            print(
                f"STRIP CUSTOM: pct={r['pct']} rgb={r['rgb']} tmcl={r['tmcl']} ack=[{fmt_rgb_acks(r['acks'])}] "
                f"sent={r['sent']} time={r['elapsed_ms']}ms"
            )
            _persist_led_choice(tester, r["rgb"], preset=None)
        elif ans == "8":
            rgb_in = input("RGB values (e.g. 255 128 0 or 255,128,0): ").strip()
            rgb = parse_rgb_triplet(rgb_in)
            if rgb is None:
                print("Invalid RGB. Enter exactly 3 integers in range 0..255.")
                continue
            r = tester.strip_set_rgb(rgb[0], rgb[1], rgb[2], reconnect_first=True)
            print(
                f"STRIP RGB: rgb={r['rgb']} tmcl={r['tmcl']} ack=[{fmt_rgb_acks(r['acks'])}] "
                f"sent={r['sent']} time={r['elapsed_ms']}ms"
            )
            _persist_led_choice(tester, r["rgb"], preset=None)
        elif ans == "9":
            run = tester.led_firstpath_restart(leave_on=True)
            print("STRIP RE-PRIME complete.")
            for i, row in enumerate(run["steps"], start=1):
                print(
                    f"  {i:02d}. {row['label']}: mask={row['mask']} value={row['value']} "
                    f"ack={fmt_resp(row['ack'])} time={row['elapsed_ms']}ms"
                )
        elif ans == "p":
            run_led_preset_menu(tester)
        elif ans == "r":
            out = tester.apply_saved_led_state(reconnect_first=True)
            if not out.get("ok"):
                print(f"No persisted default to apply ({out.get('error')}).")
                continue
            persisted = out.get("persisted", {})
            pname = persisted.get("preset") or "custom"
            print(
                f"PERSISTED APPLY ({pname}): rgb={out['rgb']} tmcl={out['tmcl']} "
                f"ack=[{fmt_rgb_acks(out['acks'])}] sent={out['sent']} time={out['elapsed_ms']}ms"
            )
        elif ans == "a":
            sec_in = input("Duration seconds (blank=10): ").strip()
            bright_in = input("Brightness 0..255 (blank=255): ").strip()
            speed_in = input(
                "Speed preset [1=slow 2=normal 3=fast 4=turbo c=custom] (blank=2): "
            ).strip().lower()
            speed_presets = {
                "1": ("slow", 0.16),
                "2": ("normal", 0.08),
                "3": ("fast", 0.04),
                "4": ("turbo", 0.02),
            }
            try:
                seconds = float(sec_in) if sec_in else 10.0
                brightness = int(bright_in, 0) if bright_in else 255
                if speed_in in ("", "2"):
                    speed_label, step_delay_s = speed_presets["2"]
                elif speed_in in speed_presets:
                    speed_label, step_delay_s = speed_presets[speed_in]
                elif speed_in == "c":
                    delay_in = input("Custom frame delay seconds (blank=0.08): ").strip()
                    step_delay_s = float(delay_in) if delay_in else 0.08
                    speed_label = "custom"
                else:
                    print("Invalid speed preset.")
                    continue
            except ValueError:
                print("Invalid rainbow parameters.")
                continue
            print(
                f"Running rainbow cycle ({speed_label}, frame_delay={step_delay_s:.3f}s). "
                "Press Ctrl+C to stop early."
            )
            out = tester.strip_rainbow_cycle(
                seconds=seconds,
                step_delay_s=step_delay_s,
                brightness=brightness,
            )
            last = out.get("last") or {}
            rgb = last.get("rgb")
            tmcl = last.get("tmcl")
            acks = last.get("acks")
            ack_txt = fmt_rgb_acks(acks) if isinstance(acks, dict) else "n/a"
            stop_txt = " (stopped early)" if out.get("interrupted") else ""
            print(
                f"RAINBOW done{stop_txt}: frames={out['frames']} "
                f"time={out['elapsed_ms']}ms sent={out['sent']} "
                f"speed={speed_label} frame_delay={out['step_delay_s']:.3f}s "
                f"last_rgb={rgb} last_tmcl={tmcl} ack=[{ack_txt}]"
            )
        else:
            print("Invalid.")


if __name__ == "__main__":
    tester = BioXpTester(alt=1)
    print_status(tester)
    persisted_boot = tester.apply_saved_led_state(reconnect_first=False)
    if persisted_boot.get("ok"):
        pst = persisted_boot.get("persisted", {})
        pname = pst.get("preset") or "custom"
        print(
            f"Applied persisted LED default ({pname}): rgb={persisted_boot['rgb']} "
            f"tmcl={persisted_boot['tmcl']} ack=[{fmt_rgb_acks(persisted_boot['acks'])}] "
            f"sent={persisted_boot['sent']} time={persisted_boot['elapsed_ms']}ms"
        )
    elif persisted_boot.get("error") != "no persisted LED state":
        print(f"Persisted LED apply skipped: {persisted_boot.get('error')}")

    while True:
        print("\n========================================")
        print(" BIOXP 3200 - CONTROL MENU")
        print("========================================")
        print("  1. LATCH CONTROL")
        print("  2. LED CONTROL")
        print("  3. CAMERA SYSTEM")
        print("  4. MOTOR CONTROL")
        print("  5. CHILLER SYSTEM")
        print("  6. THERMAL CYCLER")
        print("  s. RECONNECT USB + STATUS")
        print("  q. Quit")

        ans = input("\n> ").strip().lower()
        if ans == "":
            continue
        if ans == "q":
            sys.exit(0)
        if ans in ("1", "11"):
            run_latch_menu(tester)
        elif ans in ("2", "22"):
            run_led_menu(tester)
        elif ans in ("3", "33"):
            run_camera_menu(tester)
        elif ans in ("4", "44"):
            run_motor_menu(tester)
        elif ans in ("5", "55"):
            run_chiller_menu(tester)
        elif ans in ("6", "66"):
            run_thermal_cycler_menu(tester)
        elif ans == "s":
            tester.reconnect()
            print_status(tester)
        else:
            print("Invalid.")
