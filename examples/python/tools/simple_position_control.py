"""
RealMan WHJ Simple Position Control
Direct position commands without interpolation
Uses motor's internal trajectory planning
"""

import sys
import time
from zlgcan_driver import ZlgCanDriver, ZCANDeviceType
from whj_protocol import WHJProtocol, Register, WorkMode
from motor_control import MotorController


class SimplePositionController(MotorController):
    """
    Simple position controller using motor's internal trajectory
    
    No software interpolation - let the motor handle smooth motion
    with its built-in acceleration limits.
    """
    
    def move_to_position(self, target_pos: float, timeout: float = 15.0) -> bool:
        """
        Move to target position using motor's internal trajectory
        
        Args:
            target_pos: Target position in degrees
            timeout: Maximum wait time
        
        Returns:
            True if reached target
        """
        # Get current position
        current_pos = self.get_position()
        if current_pos is None:
            print("[Error] Failed to get current position")
            return False
        
        distance = abs(target_pos - current_pos)
        if distance < 0.5:
            print(f"[Motion] Already at target {target_pos:.2f}°")
            return True
        
        print(f"[Motion] Moving {current_pos:.2f}° -> {target_pos:.2f}° ({distance:.1f}°)")
        
        # Ensure motor is enabled and in position mode
        if not self.is_enabled():
            print("[Motion] Enabling motor...")
            if not self.enable(True):
                print("[Error] Failed to enable")
                return False
            time.sleep(0.1)
        
        mode = self.get_work_mode()
        if mode != "POSITION_MODE (Position)":
            print("[Motion] Setting position mode...")
            if not self.set_work_mode(WorkMode.POSITION_MODE):
                print("[Error] Failed to set position mode")
                return False
            time.sleep(0.05)
        
        # Send target position directly (motor handles trajectory internally)
        if not self.set_target_position(target_pos):
            print("[Error] Failed to send position command")
            return False
        
        # Wait for motion to complete
        print("[Motion] Moving...")
        start_time = time.time()
        last_print = start_time
        
        try:
            while True:
                now = time.time()
                elapsed = now - start_time
                
                if elapsed > timeout:
                    print(f"[Motion] Timeout after {timeout}s")
                    return False
                
                # Read current position
                pos = self.get_position()
                if pos is not None:
                    error = abs(pos - target_pos)
                    
                    # Print progress every 0.3s
                    if now - last_print >= 0.3:
                        print(f"  Position: {pos:8.2f}° | Target: {target_pos:8.2f}° | Error: {error:6.3f}°")
                        last_print = now
                    
                    # Check if reached target
                    if error < 1.0:  # Within 1 degree
                        print(f"[Motion] Reached target! Final: {pos:.2f}° (error: {error:.3f}°)")
                        return True
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            print("\n[Motion] Interrupted")
            return False
    
    def set_motion_limits(self, max_speed: float = None, max_accel: float = None):
        """
        Set motor motion limits (writes to flash)
        
        Args:
            max_speed: Maximum speed in RPM
            max_accel: Maximum acceleration in 0.1 rpm/s
        """
        if max_speed is not None:
            # Register LIT_MAX_SPEED (0x41) - units: RPM
            print(f"[Config] Setting max speed to {max_speed} RPM")
            # Note: Need to implement register write
        
        if max_accel is not None:
            # Register LIT_MAX_ACC (0x42) - units: 0.1 rpm/s
            print(f"[Config] Setting max accel to {max_accel * 0.1} rpm/s")


def main():
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    
    print("=" * 60)
    print("RealMan WHJ Simple Position Control")
    print("=" * 60)
    print(f"Motor ID: {motor_id}")
    print()
    print("Uses motor's internal trajectory planning")
    print("Set motor acceleration limits with ZCANPro for smoother motion")
    print()
    
    driver = None
    motor = None
    
    try:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
    except RuntimeError as e:
        print(f"[Error] {e}")
        return
    
    motor = SimplePositionController(driver, motor_id)
    
    if not motor.initialize():
        print("[Error] Failed to initialize motor")
        driver.close()
        return
    
    print()
    print("Commands:")
    print("  p <pos>  - Move to position (uses motor internal trajectory)")
    print("  e        - Enable motor")
    print("  d        - Disable motor")
    print("  c        - Clear errors")
    print("  r        - Read position")
    print("  q        - Quit")
    print()
    
    while True:
        try:
            cmd_input = input("> ").strip().lower()
            parts = cmd_input.split()
            cmd = parts[0] if parts else ""
            
            if cmd == 'q':
                break
            
            elif cmd == 'e':
                if motor.enable(True):
                    print("Motor enabled")
                else:
                    print("Failed to enable")
            
            elif cmd == 'd':
                if motor.enable(False):
                    print("Motor disabled")
                else:
                    print("Failed to disable")
            
            elif cmd == 'c':
                if motor.clear_error():
                    print("Errors cleared")
                else:
                    print("Failed to clear errors")
            
            elif cmd == 'p' and len(parts) >= 2:
                try:
                    target = float(parts[1])
                    motor.move_to_position(target)
                except ValueError:
                    print("Usage: p <position_in_degrees>")
            
            elif cmd == 'r':
                pos = motor.get_position()
                if pos is not None:
                    print(f"Current position: {pos:.4f}°")
                else:
                    print("Failed to read position")
            
            else:
                print("Unknown command")
        
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    # Cleanup
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
