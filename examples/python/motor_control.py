"""
RealMan WHJ Motor Control
Complete motor control with initialization sequence
"""

import sys
import time
from zlgcan_driver import ZlgCanDriver, ZCANDeviceType
from whj_protocol import WHJProtocol, Register, WorkMode, ErrorCode


def parse_32bit_value(low, high):
    """Parse two 16-bit values to signed 32-bit"""
    val = (high << 16) | low
    if val & 0x80000000:
        val -= 0x100000000
    return val


class MotorController:
    """WHJ Motor Controller with proper initialization"""
    
    def __init__(self, driver, motor_id):
        self.driver = driver
        self.motor_id = motor_id
        self.response_id = motor_id + 0x100
    
    def send_command(self, data, timeout_ms=500):
        """Send command and wait for response"""
        # Clear buffer using driver's clear_buffer method
        self.driver.clear_buffer()
        
        # Send
        if not self.driver.send(can_id=self.motor_id, data=data):
            return None, "Send failed"
        
        # Wait for response
        start = time.time()
        while (time.time() - start) * 1000 < timeout_ms:
            frame = self.driver.receive(timeout_ms=50)
            if frame and frame.can_id == self.response_id:
                return frame.data, None
            time.sleep(0.001)
        
        return None, "Timeout"
    
    def initialize(self):
        """
        Initialize motor communication
        Try reading firmware version to check if motor is online
        """
        print(f"[Init] Initializing motor {self.motor_id}...")
        
        # Try to ping motor by reading firmware version
        print("[Init] Pinging motor...")
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.SYS_FW_VERSION, 1)
        resp, err = self.send_command(cmd, timeout_ms=500)
        
        if resp:
            print(f"[Init] Motor is online! Response: {resp.hex()}")
            return True
        else:
            print(f"[Init] Ping failed: {err}")
            return False
    
    def get_system_info(self):
        """Get system information"""
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.SYS_MODEL_TYPE, 6)
        resp, err = self.send_command(cmd)
        
        if not resp or len(resp) < 14:
            return None
        
        try:
            model = resp[2] | (resp[3] << 8)
            fw_ver = resp[4] | (resp[5] << 8)
            voltage_raw = resp[6] | (resp[7] << 8)
            temp_raw = resp[8] | (resp[9] << 8)
            redu_ratio = resp[10] | (resp[11] << 8)
            
            model_names = {
                0x02: "J14 (Joint 10)",
                0x03: "J17 (Joint 30)",
                0x04: "J20 (Joint 60)",
                0x05: "J25 (Joint 120)",
                0x06: "Gripper",
                0x07: "J3 (Joint 03)",
            }
            
            return {
                'model': model_names.get(model, f"Unknown (0x{model:02X})"),
                'firmware': f"v{fw_ver >> 8}.{fw_ver & 0xFF}",
                'voltage': voltage_raw * 0.01,
                'temperature': temp_raw * 0.1,
                'reduction_ratio': redu_ratio
            }
        except:
            return None
    
    def get_error_status(self):
        """Get error status"""
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.SYS_ERROR, 1)
        resp, err = self.send_command(cmd)
        
        if not resp or len(resp) < 4:
            return None
        
        error_code = resp[2] | (resp[3] << 8)
        errors = ErrorCode.parse(error_code)
        return error_code, errors
    
    def is_enabled(self):
        """Check if driver is enabled"""
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.SYS_ENABLE_DRIVER, 1)
        resp, err = self.send_command(cmd)
        
        if not resp or len(resp) < 4:
            return None
        
        return resp[2] | (resp[3] << 8) == 1
    
    def get_work_mode(self):
        """Get current work mode"""
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.TAG_WORK_MODE, 1)
        resp, err = self.send_command(cmd)
        
        if not resp or len(resp) < 4:
            return None
        
        mode = resp[2] | (resp[3] << 8)
        modes = {
            0: "OPEN_LOOP",
            1: "CURRENT_MODE",
            2: "SPEED_MODE",
            3: "POSITION_MODE"
        }
        return modes.get(mode, f"UNKNOWN ({mode})")
    
    def get_position(self):
        """Get current position"""
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.CUR_POSITION_L, 2)
        resp, err = self.send_command(cmd)
        
        if not resp or len(resp) < 6:
            return None
        
        low = resp[2] | (resp[3] << 8)
        high = resp[4] | (resp[5] << 8)
        raw = parse_32bit_value(low, high)
        return raw * 0.0001
    
    def enable(self, enable=True):
        """Enable or disable motor driver"""
        value = 1 if enable else 0
        cmd = WHJProtocol.build_write_frame(self.motor_id, Register.SYS_ENABLE_DRIVER, value)
        resp, err = self.send_command(cmd)
        
        if resp and len(resp) >= 3:
            success = resp[2] == 0x01
            return success
        return False
    
    def clear_error(self):
        """Clear error flags"""
        cmd = WHJProtocol.build_write_frame(self.motor_id, Register.SYS_CLEAR_ERROR, 1)
        resp, err = self.send_command(cmd)
        return resp is not None
    
    def set_zero_position(self):
        """Set current position as zero (for encoder multi-turn lost recovery)"""
        cmd = WHJProtocol.build_write_frame(self.motor_id, Register.SYS_SET_ZERO_POS, 1)
        resp, err = self.send_command(cmd)
        return resp is not None
    
    def save_to_flash(self):
        """Save current configuration to flash"""
        cmd = WHJProtocol.build_write_frame(self.motor_id, Register.SYS_SAVE_TO_FLASH, 1)
        resp, err = self.send_command(cmd)
        return resp is not None
    
    def set_work_mode(self, mode):
        """Set work mode"""
        cmd = WHJProtocol.build_write_frame(self.motor_id, Register.TAG_WORK_MODE, mode)
        resp, err = self.send_command(cmd)
        return resp is not None
    
    def set_target_position(self, position_deg):
        """Set target position"""
        raw = int(position_deg / 0.0001)
        
        # Send low 16 bits
        cmd1 = WHJProtocol.build_write_frame(self.motor_id, Register.TAG_POSITION_L, raw & 0xFFFF)
        resp1, _ = self.send_command(cmd1)
        
        # Send high 16 bits
        cmd2 = WHJProtocol.build_write_frame(self.motor_id, Register.TAG_POSITION_H, (raw >> 16) & 0xFFFF)
        resp2, _ = self.send_command(cmd2)
        
        return resp1 is not None and resp2 is not None


def main():
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    
    print("=" * 60)
    print("RealMan WHJ Motor Control")
    print("=" * 60)
    print(f"Motor ID: {motor_id}")
    print()
    
    driver = None
    motor = None
    
    # Initialize CAN
    try:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
    except RuntimeError as e:
        print(f"[Error] Failed to open CAN device: {e}")
        print("  Solutions:")
        print("  1. Close ZCANPro or other programs using the device")
        print("  2. Unplug and replug the USB cable")
        print("  3. Restart your computer")
        return
    
    # Create controller
    motor = MotorController(driver, motor_id)
    
    # Initialize (wake-up sequence)
    if not motor.initialize():
        print("\n[Error] Failed to initialize motor. Please check:")
        print("  - Motor is powered on")
        print("  - CAN cable is connected")
        print("  - Motor ID is correct")
        driver.close()
        return
    
    print()
    print("-" * 60)
    print("Motor Status")
    print("-" * 60)
    
    # Get system info
    info = motor.get_system_info()
    if info:
        print(f"Model:           {info['model']}")
        print(f"Firmware:        {info['firmware']}")
        print(f"Voltage:         {info['voltage']:.2f} V")
        print(f"Temperature:     {info['temperature']:.1f} °C")
    
    # Get error status
    error = motor.get_error_status()
    if error:
        code, errors = error
        print(f"Error Code:      0x{code:04X}")
        if code == 0:
            print("Status:          OK (No errors)")
        else:
            print("Errors:")
            for e in errors:
                print(f"  - {e}")
    
    # Get enabled status
    enabled = motor.is_enabled()
    if enabled is not None:
        print(f"Driver Enabled:  {'Yes' if enabled else 'No'}")
    
    # Get work mode
    mode = motor.get_work_mode()
    if mode:
        print(f"Work Mode:       {mode}")
    
    # Get position
    pos = motor.get_position()
    if pos is not None:
        print(f"Current Position: {pos:.4f}°")
    
    print("-" * 60)
    print()
    
    # Interactive control
    print("Commands:")
    print("  e  - Enable driver")
    print("  d  - Disable driver")
    print("  c  - Clear errors")
    print("  p  - Go to position (e.g., p 90)")
    print("  r  - Read current position")
    print("  s  - Show status")
    print("  q  - Quit")
    print()
    
    while True:
        try:
            cmd = input("> ").strip().lower()
            
            if cmd == 'q':
                break
            
            elif cmd == 'e':
                if motor.enable(True):
                    print("Driver enabled")
                else:
                    print("Failed to enable")
            
            elif cmd == 'd':
                if motor.enable(False):
                    print("Driver disabled")
                else:
                    print("Failed to disable")
            
            elif cmd == 'c':
                if motor.clear_error():
                    print("Errors cleared")
                else:
                    print("Failed to clear errors")
            
            elif cmd.startswith('p '):
                try:
                    target = float(cmd.split()[1])
                    if motor.set_target_position(target):
                        print(f"Target position set to {target}°")
                    else:
                        print("Failed to set position")
                except:
                    print("Usage: p <position_in_degrees>")
            
            elif cmd == 'r':
                pos = motor.get_position()
                if pos is not None:
                    print(f"Current position: {pos:.4f}°")
                else:
                    print("Failed to read position")
            
            elif cmd == 's':
                error = motor.get_error_status()
                if error:
                    code, errors = error
                    print(f"Error Code: 0x{code:04X}")
                enabled = motor.is_enabled()
                if enabled is not None:
                    print(f"Enabled: {'Yes' if enabled else 'No'}")
                pos = motor.get_position()
                if pos is not None:
                    print(f"Position: {pos:.4f}°")
            
            else:
                print("Unknown command")
        
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    # Disable motor before exit
    if motor:
        print("\nDisabling motor...")
        try:
            motor.enable(False)
        except:
            pass
    
    if driver:
        driver.close()
    print("Done!")


if __name__ == "__main__":
    main()
