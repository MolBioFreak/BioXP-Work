import usb.core
import usb.util
import struct
import time


class BioXpUsbDriver:
    KNOWN_HEARTBEATS = {
        (0x7e, 0x0, 0x0, 0x4, 0x82, 0x0, 0x86, 0x7e),
        (0x7e, 0x0, 0x0, 0x4, 0x8a, 0x0, 0x8e, 0x7e),
        (0x7e, 0x0, 0x0, 0x4, 0x92, 0x0, 0x96, 0x7e),
        (0x7e, 0x0, 0x0, 0x4, 0x9a, 0x0, 0x9e, 0x7e),
        (0x7e, 0x0, 0x0, 0x0, 0x0, 0x8, 0x4, 0x81, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8d, 0x7e),
        (0x7e, 0x0, 0x0, 0x0, 0x0, 0x8, 0x5, 0x81, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8e, 0x7e),
        (0x7e, 0x0, 0x0, 0x0, 0x0, 0x8, 0x6, 0x81, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x8f, 0x7e),
        (0x7e, 0x0, 0x0, 0x0, 0x0, 0x8, 0x7, 0x81, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x90, 0x7e),
    }
    TMCL_STATUS = {100: "Success", 1: "Wrong checksum", 2: "Invalid command",
                   3: "Wrong type", 4: "Invalid value", 5: "EEPROM locked",
                   6: "Command not available", 7: "Busy"}

    def __init__(self):
        self.dev = usb.core.find(idVendor=0x03eb, idProduct=0x2423)
        if self.dev is None:
            raise ValueError("BioXP USB Device not found.")
        if self.dev.is_kernel_driver_active(0):
            self.dev.detach_kernel_driver(0)
        self.dev.set_configuration()
        usb.util.claim_interface(self.dev, 0)
        self.dev.set_interface_altsetting(interface=0, alternate_setting=1)
        cfg = self.dev.get_active_configuration()
        intf = cfg[(0, 1)]
        self.ep_out = usb.util.find_descriptor(
            intf, custom_match=lambda e:
            usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT)
        self.ep_in = usb.util.find_descriptor(
            intf, custom_match=lambda e:
            usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)

    def drain(self):
        for _ in range(50):
            try:
                self.ep_in.read(64, timeout=30)
            except usb.core.USBTimeoutError:
                break

    def send_tmcl(self, board_id, command, cmd_type, motor, value, retries=3):
        val_bytes = struct.pack('>i', value)
        inner = bytearray([0x00, 0x00, 0x00, board_id, 0x08,
                           command, cmd_type, motor,
                           val_bytes[0], val_bytes[1], val_bytes[2], val_bytes[3], 0x00])
        chksum = sum(inner) & 0xFF
        frame = bytearray([0x7E]) + inner + bytearray([chksum, 0x7E])
        for attempt in range(retries):
            self.drain()
            try:
                self.ep_out.write(frame, timeout=1000)
            except usb.core.USBTimeoutError:
                continue
            for _ in range(40):
                try:
                    resp = list(self.ep_in.read(64, timeout=100))
                    if tuple(resp) not in self.KNOWN_HEARTBEATS and len(resp) >= 14:
                        return {"status": resp[7],
                                "status_str": self.TMCL_STATUS.get(resp[7], f"?({resp[7]})"),
                                "cmd": resp[8],
                                "value": struct.unpack('>i', bytes(resp[9:13]))[0]}
                except usb.core.USBTimeoutError:
                    break
        return None

    def get_pos(self, board, motor=0):
        r = self.send_tmcl(board, 6, 1, motor, 0)
        return r['value'] if r and r['status'] == 100 else None

    def enable_power(self):
        """
        Attempts to enable the 24V motor power rail.
        From DLL strings, there are commands like ACTIVATE_BOARD and IO_QUERY_24V.
        We will try setting various IO pins and custom TMCL commands that might trigger power.
        """
        # Method A: Try setting all possible IO ports on all boards to HIGH (1)
        # SIO (cmd 14), Bank 0 (digital output) and Bank 1 (digital output)
        print("  [Power] Setting all digital outputs HIGH...")
        for bid in [0x04, 0x05, 0x06, 0x07]:
            for port in range(16):
                self.send_tmcl(bid, 14, port, 0, 1, retries=1)
                self.send_tmcl(bid, 14, port, 1, 1, retries=1)

        # Method B: Send custom initialization commands found in the DLL
        # Action IDs like SET_INIT, ACTIVATE_BOARD, ENABLE_FAN
        print("  [Power] Sending broad initialization commands...")
        for bid in [0x04, 0x05, 0x06, 0x07]:
            # Setting Bank 0 (General Purpose Output)
            for port in range(8):
                self.send_tmcl(bid, 14, port, 0, 1, retries=1)
                
            # Bank 2 is often used for "pull-up" or global enable in some TMCL dialects
            self.send_tmcl(bid, 14, 0, 2, 1, retries=1)
            
            # Send TMCL Command 10 (Set Initialization / Motor Enable in some firmwares)
            # Or TMCL Command 9 (Set Motor Power)
            self.send_tmcl(bid, 9, 0, 0, 1, retries=1)
            
            # Try to restore default running current (SAP 6) and standby current (SAP 7)
            # From our earlier dump, board 0x04 defaulted to 31, board 0x06 defaulted to 16
            # 255 is the absolute hardware maximum and might trigger over-current protection
            # Let's set a safe, conservative middle-ground of 50
            self.send_tmcl(bid, 5, 6, 0, 50, retries=1)
            self.send_tmcl(bid, 5, 7, 0, 20, retries=1)


if __name__ == "__main__":
    print("=" * 60)
    print("BIOXP 3200 - 24V POWER DIAGNOSTICS")
    print("=" * 60)

    robot = BioXpUsbDriver()

    print("\n--- Step 1: Querying Initial IO states ---")
    initial_io = {}
    for bid in [0x04, 0x05, 0x06, 0x07]:
        for port in range(8):
            r = robot.send_tmcl(bid, 15, port, 0, 0) # GIO
            if r and r['status'] == 100:
                initial_io[(bid, port)] = r['value']
                print(f"  Board 0x{bid:02X} port {port} = {r['value']}")

    print("\n--- Step 2: Firing global enable sequence... ---")
    robot.enable_power()
    time.sleep(2)

    print("\n--- Step 3: Checking if 24V state changed in IO ---")
    for bid in [0x04, 0x05, 0x06, 0x07]:
        for port in range(8):
            r = robot.send_tmcl(bid, 15, port, 0, 0)
            if r and r['status'] == 100:
                old = initial_io.get((bid, port))
                if old != r['value']:
                    print(f"  *** Board 0x{bid:02X} port {port} CHANGED: {old} -> {r['value']} ***")

    # Set max speed
    for bid in [0x04, 0x05, 0x06]:
        robot.send_tmcl(bid, 5, 4, 0, 2000)
        robot.send_tmcl(bid, 5, 5, 0, 2000)

    for bid in [0x04]:
        print(f"\n{'='*50}")
        print(f"  BOARD 0x{bid:02X} - MOTOR POWER TEST")
        print(f"{'='*50}")

        print(f"  >>> ROR vel=1900 for 5 seconds <<<")
        robot.drain()
        robot.send_tmcl(bid, 1, 0, 0, 1900)
        
        for i in range(5):
            time.sleep(1)
            p = robot.get_pos(bid)
            print(f"    t={i+1}s  pos={p}")
            
        robot.send_tmcl(bid, 3, 0, 0, 0)
        print(f"  >>> STOPPED <<<")
