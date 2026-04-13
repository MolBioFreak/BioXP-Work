import can
import struct
import time
from enum import IntEnum

class BoardAssy(IntEnum):
    THERMAL_CONTROLLER = 0x05  # Example ID based on CAN topology
    CHILLER_BOARD = 0x07       # Derived from ClassChillerBoard init
    MOTOR_CONTROLLER = 0x03    # Example ID for Gantry
    PIPETTE_CONTROLLER = 0x09  # Derived placeholder for the pipette CAN node

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
    def __init__(self, channel='can0', bitrate=1000000):
        # The BioXP USB-to-CAN adapter should map to can0 in Linux
        self.bus = can.interface.Bus(bustype='socketcan', channel=channel, bitrate=bitrate)
        
    def _send_packet(self, board_id: int, command: list):
        """
        Base wrapper for broadcasting a packet.
        The BioXP DLLs use a segmented multipart 8-byte structure.
        """
        payload = (command + [0] * (8 - len(command)))[:8]
        msg = can.Message(
            arbitration_id=board_id,
            data=payload,
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
            print(f"[CAN TX] ID: {hex(board_id)} | Data: {[hex(b) for b in payload]}")
            return {
                "ok": True,
                "board_id": int(board_id),
                "payload": payload,
            }
        except can.CanError as e:
            print(f"CAN Bus Error: {e}")
            return {
                "ok": False,
                "board_id": int(board_id),
                "payload": payload,
                "error": str(e),
            }

    def _send_ascii_packet(self, board_id: int, ascii_command: str):
        encoded = str(ascii_command).encode('ascii')
        if len(encoded) > 8:
            raise ValueError(f"ASCII pipette command exceeds 8-byte frame budget: {ascii_command!r}")
        return {
            **self._send_packet(board_id, list(encoded)),
            "ascii_command": str(ascii_command),
            "length": len(encoded),
        }

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
        
        self._send_packet(BoardAssy.THERMAL_CONTROLLER, packet)

    # ==========================================
    # PIPETTE SYSTEM (ClassPipette)
    # ==========================================
    @staticmethod
    def _format_pipette_volume(volume_ul: float) -> str:
        volume_ul = float(volume_ul)
        if volume_ul >= 100.0:
            return f"{round(volume_ul):.0f}"
        return f"{volume_ul:.1f}"

    def pipette_initialize(self, pressure_profile='1R'):
        return self._send_ascii_packet(BoardAssy.PIPETTE_CONTROLLER, f"INIT,{str(pressure_profile).upper()}")

    def pipette_load_tip(self):
        return self._send_ascii_packet(BoardAssy.PIPETTE_CONTROLLER, "TIP,LD")

    def pipette_eject_tip(self):
        return self._send_ascii_packet(BoardAssy.PIPETTE_CONTROLLER, "TIP,EJ")

    def aspirate(self, volume_ul: float, tip_pressure_profile='1R'):
        """
        Reverse Engineered from `ClassPipette.Aspirate(double volume)`.
        The DLL bypasses stepper math and pushes an ASCII formatted string
        over the CAN bus to the pipette firmware.
        """
        formatted_vol = self._format_pipette_volume(volume_ul)
        ascii_command = f"P{formatted_vol},{str(tip_pressure_profile).upper()}"
        return self._send_ascii_packet(BoardAssy.PIPETTE_CONTROLLER, ascii_command)

    def dispense(self, volume_ul: float, tip_pressure_profile='1R', blow_out=False):
        formatted_vol = self._format_pipette_volume(volume_ul)
        ascii_command = f"D{formatted_vol},{str(tip_pressure_profile).upper()}"
        result = self._send_ascii_packet(BoardAssy.PIPETTE_CONTROLLER, ascii_command)
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
        
        self._send_packet(BoardAssy.MOTOR_CONTROLLER, packet)


if __name__ == "__main__":
    print("BIOXP 3200 PYTHON-CAN DRIVER INITIALIZED")
    print("----------------------------------------")
    
    # NOTE: The physical unit must be attached to test actual CAN broadcasting.
    # The following commands will just demonstrate the packing algorithm natively.
    
    import warnings
    warnings.filterwarnings('ignore') # Ignore 'can0' interface missing for offline demo
    
    try:
        # We wrap in a try-except to allow testing without physical hardware
        robot = BioXpCanDriver(channel='vcan0') # Use virtual CAN for safety
        
        robot.set_thermal_temperature(95.0)  # Sets TC block to 95C
        robot.move_axis(MotorAxis.Z, 80000)  # Move Z axis down 80000 steps
        robot.aspirate(0.5)                  # Aspirate 0.5uL
        
    except OSError:
        print("Hardware SocketCAN interface not found. Cannot broadcast packets.")
