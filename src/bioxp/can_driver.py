import can
import struct
import time
from enum import IntEnum

class BoardAssy(IntEnum):
    THERMAL_CONTROLLER = 0x05  # Example ID based on CAN topology
    CHILLER_BOARD = 0x07       # Derived from ClassChillerBoard init
    MOTOR_CONTROLLER = 0x03    # Example ID for Gantry

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
        # Truncate/Pad to 8 bytes standard CAN frame
        payload = (command + [0]*(8-len(command)))[:8]
        msg = can.Message(
            arbitration_id=board_id,
            data=payload,
            is_extended_id=False
        )
        try:
            self.bus.send(msg)
            print(f"[CAN TX] ID: {hex(board_id)} | Data: {[hex(b) for b in payload]}")
        except can.CanError as e:
            print(f"CAN Bus Error: {e}")

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
    def aspirate(self, volume_ul: float, tip_pressure_profile='1R'):
        """
        Reverse Engineered from `ClassPipette.Aspirate(double volume)`.
        The DLL completely bypassing stepper math and pushes 
        an ASCII formatted string over the CAN bus to the pipette firmware.
        """
        # Applying the exact rounding logic found in the DLL 
        # (Though we can enhance it since we operate in Python!)
        if volume_ul >= 100.0:
            volume_ul = round(volume_ul)
            formatted_vol = f"{volume_ul:.0f}"
        else:
            # The hardcoded 0.1uL limit from ToString("F1")
            formatted_vol = f"{volume_ul:.1f}"
            
        # The firmware expects commands like "P50.0,1R"
        ascii_command = f"P{formatted_vol},{tip_pressure_profile}"
        
        # Convert string to bytes and send (Implementation of segmentation needed for >8 byte strings)
        # Assuming the first byte is an action header (e.g. 0x01 for Aspirate String)
        cmd_bytes = ascii_command.encode('ascii')
        
        print(f"Pipette Command String format: {ascii_command} | Bytes: {cmd_bytes}")
        # self._send_string(BoardAssy.PIPETTE_CONTROLLER, cmd_bytes)

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
