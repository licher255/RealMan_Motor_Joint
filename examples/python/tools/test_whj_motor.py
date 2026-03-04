"""
RealMan WHJ Motor Test
Tests basic communication with WHJ joint motors using ZLG CAN FD
"""

import sys
import time
from typing import Optional

from zlgcan_driver import ZlgCanDriver, ZCANDeviceType, CANFDFrame
from whj_protocol import WHJProtocol, Register, WorkMode, JointState, ErrorCode


class WHJMotorController:
    """
    RealMan WHJ Motor Controller
    
    High-level interface for controlling WHJ joint motors
    """
    
    def __init__(self, can_driver: ZlgCanDriver):
        self.can = can_driver
        self.protocol = WHJProtocol()
        
    def _send_and_receive(self, motor_id: int, data: bytes, 
                          timeout_ms: int = 100) -> Optional[CANFDFrame]:
        """
        Send frame and wait for response
        
        Args:
            motor_id: Target motor ID
            data: Frame data to send
            timeout_ms: Timeout in milliseconds
        
        Returns:
            Response frame or None if timeout
        """
        # Clear old frames
        self.can.clear_buffer()
        
        # Send command
        tx_success = self.can.send(can_id=motor_id, data=data)
        if not tx_success:
            print(f"[Error] Failed to send command to motor {motor_id}")
            return None
        
        # Wait for response (response ID = motor_id + 0x100)
        response_id = motor_id + 0x100
        start_time = time.time()
        
        while (time.time() - start_time) * 1000 < timeout_ms:
            rx_frame = self.can.receive(timeout_ms=10)
            if rx_frame and rx_frame.can_id == response_id:
                return rx_frame
            time.sleep(0.001)
        
        return None
    
    def ping(self, motor_id: int, timeout_ms: int = 200) -> bool:
        """
        Check if motor is online
        
        Args:
            motor_id: Motor ID to ping
            timeout_ms: Timeout in milliseconds
        
        Returns:
            True if motor responded
        """
        # Read model type register
        cmd = WHJProtocol.build_read_frame(motor_id, Register.SYS_MODEL_TYPE, 1)
        response = self._send_and_receive(motor_id, cmd, timeout_ms)
        return response is not None
    
    def scan_motors(self, max_id: int = 30) -> list:
        """
        Scan for all connected motors
        
        Args:
            max_id: Maximum motor ID to scan
        
        Returns:
            List of motor IDs found
        """
        print(f"[Scan] Scanning for motors (ID 1-{max_id})...")
        found = []
        
        for motor_id in range(1, max_id + 1):
            if self.ping(motor_id, timeout_ms=100):
                print(f"[Scan] Found motor ID: {motor_id}")
                found.append(motor_id)
        
        print(f"[Scan] Found {len(found)} motor(s)")
        return found
    
    def get_motor_info(self, motor_id: int) -> Optional[dict]:
        """
        Get motor information
        
        Args:
            motor_id: Motor ID
        
        Returns:
            Dict with motor info or None if failed
        """
        cmd = WHJProtocol.build_read_system_info(motor_id)
        response = self._send_and_receive(motor_id, cmd)
        
        if not response:
            return None
        
        try:
            model, fw_version, redu_ratio = WHJProtocol.parse_system_info(
                motor_id, response.data
            )
            return {
                'motor_id': motor_id,
                'model': model,
                'model_name': model.name if hasattr(model, 'name') else str(model),
                'firmware_version': fw_version,
                'reduction_ratio': redu_ratio
            }
        except Exception as e:
            print(f"[Error] Failed to parse motor info: {e}")
            return None
    
    def get_joint_state(self, motor_id: int) -> Optional[JointState]:
        """
        Get current joint state
        
        Args:
            motor_id: Motor ID
        
        Returns:
            JointState or None if failed
        """
        # Read all feedback registers
        cmd = WHJProtocol.build_read_state(motor_id)
        response = self._send_and_receive(motor_id, cmd)
        
        if not response:
            return None
        
        try:
            return WHJProtocol.parse_state_response(motor_id, response.data)
        except Exception as e:
            print(f"[Error] Failed to parse state: {e}")
            return None
    
    def get_system_state(self, motor_id: int) -> Optional[dict]:
        """
        Get system state (voltage, temperature, error)
        
        Args:
            motor_id: Motor ID
        
        Returns:
            Dict with system state or None if failed
        """
        # Read voltage, temp, error, enable status
        cmd = WHJProtocol.build_read_frame(motor_id, Register.SYS_VOLTAGE, 6)
        response = self._send_and_receive(motor_id, cmd)
        
        if not response:
            return None
        
        try:
            values = WHJProtocol.parse_read_response(response.data, Register.SYS_VOLTAGE)
            if len(values) < 6:
                return None
            
            return {
                'voltage_v': values[0] * 0.01,
                'temperature_c': values[1] * 0.1,
                'reduction_ratio': values[2],
                'enable_on_power': values[3],
                'is_enabled': values[4] == 1,
                'error_code': values[5]
            }
        except Exception as e:
            print(f"[Error] Failed to parse system state: {e}")
            return None
    
    def enable_motor(self, motor_id: int, enable: bool = True) -> bool:
        """
        Enable or disable motor driver
        
        Args:
            motor_id: Motor ID
            enable: True to enable, False to disable
        
        Returns:
            True if successful
        """
        cmd = WHJProtocol.build_enable_motor(motor_id, enable)
        response = self._send_and_receive(motor_id, cmd)
        
        if not response:
            return False
        
        return WHJProtocol.parse_write_response(response.data)
    
    def clear_error(self, motor_id: int) -> bool:
        """
        Clear motor errors
        
        Args:
            motor_id: Motor ID
        
        Returns:
            True if successful
        """
        cmd = WHJProtocol.build_clear_error(motor_id)
        response = self._send_and_receive(motor_id, cmd)
        
        if not response:
            return False
        
        return WHJProtocol.parse_write_response(response.data)
    
    def set_work_mode(self, motor_id: int, mode: WorkMode) -> bool:
        """
        Set motor work mode
        
        Args:
            motor_id: Motor ID
            mode: WorkMode (OPEN_LOOP, CURRENT_MODE, SPEED_MODE, POSITION_MODE)
        
        Returns:
            True if successful
        """
        cmd = WHJProtocol.build_set_work_mode(motor_id, mode)
        response = self._send_and_receive(motor_id, cmd)
        
        if not response:
            return False
        
        return WHJProtocol.parse_write_response(response.data)
    
    def set_target_position(self, motor_id: int, position_deg: float) -> bool:
        """
        Set target position (position mode)
        
        Args:
            motor_id: Motor ID
            position_deg: Target position in degrees
        
        Returns:
            True if successful
        """
        cmds = WHJProtocol.build_set_target_position(motor_id, position_deg)
        
        # Send low word
        response = self._send_and_receive(motor_id, cmds[0])
        if not response or not WHJProtocol.parse_write_response(response.data):
            return False
        
        # Send high word
        response = self._send_and_receive(motor_id, cmds[1])
        if not response:
            return False
        
        return WHJProtocol.parse_write_response(response.data)
    
    def set_target_speed(self, motor_id: int, speed_rpm: float) -> bool:
        """
        Set target speed (speed mode)
        
        Args:
            motor_id: Motor ID
            speed_rpm: Target speed in RPM
        
        Returns:
            True if successful
        """
        cmds = WHJProtocol.build_set_target_speed(motor_id, speed_rpm)
        
        response = self._send_and_receive(motor_id, cmds[0])
        if not response or not WHJProtocol.parse_write_response(response.data):
            return False
        
        response = self._send_and_receive(motor_id, cmds[1])
        if not response:
            return False
        
        return WHJProtocol.parse_write_response(response.data)
    
    def print_state(self, motor_id: int):
        """Print motor state in readable format"""
        # Get joint state
        state = self.get_joint_state(motor_id)
        sys_state = self.get_system_state(motor_id)
        
        if not state or not sys_state:
            print(f"[Motor {motor_id}] Failed to read state")
            return
        
        print(f"\n[Motor {motor_id}] State:")
        print(f"  Position:     {state.position_deg:.4f}°")
        print(f"  Speed:        {state.speed_rpm:.2f} RPM")
        print(f"  Current:      {state.current_ma} mA")
        print(f"  Voltage:      {sys_state['voltage_v']:.2f} V")
        print(f"  Temperature:  {sys_state['temperature_c']:.1f}°C")
        print(f"  Enabled:      {'Yes' if sys_state['is_enabled'] else 'No'}")
        print(f"  Work Mode:    {state.work_mode.name if hasattr(state.work_mode, 'name') else state.work_mode}")
        
        if sys_state['error_code'] != 0:
            errors = ErrorCode.parse(sys_state['error_code'])
            print(f"  Errors:       {', '.join(errors)}")


def main():
    """Main test function"""
    print("=" * 70)
    print("RealMan WHJ Motor Driver Test")
    print("=" * 70)
    print()
    print("This test will:")
    print("  1. Connect to ZLG USBCANFD-100U-mini")
    print("  2. Scan for motors")
    print("  3. Read motor state")
    print("  4. Test enable/disable")
    print()
    
    # Default motor ID
    motor_id = 1
    if len(sys.argv) > 1:
        motor_id = int(sys.argv[1])
    
    print(f"Target motor ID: {motor_id}")
    print(f"Hint: If you know the motor ID, run: python test_whj_motor.py <motor_id>")
    print()
    
    driver = None
    controller = None
    
    try:
        # Step 1: Connect to CAN device
        print("[Step 1] Connecting to ZLG CAN device...")
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, device_index=0, channel=0)
        
        # Initialize CAN FD: 1Mbps arbitration, 5Mbps data
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        print("[Step 1] CAN FD initialized successfully!")
        print()
        
        # Create controller
        controller = WHJMotorController(driver)
        
        # Step 2: Scan for motors
        print("[Step 2] Scanning for motors...")
        motors = controller.scan_motors(max_id=10)
        
        if not motors:
            print("[Warning] No motors found in scan!")
            print("  Possible reasons:")
            print("    - Motor is still booting (wait 2-3 seconds after power on)")
            print("    - High latency on CAN bus")
            print("    - Motor ID > 10")
            print()
            
            # Try to ping the specified motor ID directly
            print(f"[Step 2] Trying to ping motor {motor_id} directly...")
            if controller.ping(motor_id, timeout_ms=200):
                print(f"[Step 2] Motor {motor_id} responded! (Direct ping successful)")
                motors = [motor_id]
            else:
                print(f"[Error] Motor {motor_id} did not respond to direct ping")
                print("Please check:")
                print("  - Motor is powered on (LED should be on)")
                print("  - CAN cable is connected correctly")
                print("  - Termination resistor is enabled")
                print("  - Motor ID is correct")
                print()
                print("You can specify a different motor ID:")
                print(f"  python test_whj_motor.py <motor_id>")
                return
        
        if motor_id not in motors:
            print(f"[Warning] Motor {motor_id} not found in scan, but motor {motors} responded")
            print(f"[Info] Will try to use motor {motor_id} anyway...")
        
        print()
        
        # Step 3: Get motor info
        print(f"[Step 3] Reading motor {motor_id} information...")
        info = controller.get_motor_info(motor_id)
        if info:
            print(f"  Model:           {info['model_name']}")
            print(f"  Firmware:        v{info['firmware_version']}")
            print(f"  Reduction Ratio: {info['reduction_ratio']}")
        else:
            print("  Failed to read motor info")
        print()
        
        # Step 4: Read current state
        print(f"[Step 4] Reading motor {motor_id} state...")
        controller.print_state(motor_id)
        print()
        
        # Step 5: Test enable/disable
        print(f"[Step 5] Testing motor control...")
        
        # Clear any errors
        print("  Clearing errors...")
        if controller.clear_error(motor_id):
            print("  Errors cleared successfully")
        
        # Enable motor
        print("  Enabling motor...")
        if controller.enable_motor(motor_id, True):
            print("  Motor enabled successfully")
            time.sleep(0.5)
            
            # Read state again
            controller.print_state(motor_id)
            
            # Set position mode
            print("\n  Setting position mode...")
            if controller.set_work_mode(motor_id, WorkMode.POSITION_MODE):
                print("  Position mode set successfully")
                time.sleep(0.1)
                
                # Get current position
                state = controller.get_joint_state(motor_id)
                if state:
                    print(f"\n  Current position: {state.position_deg:.2f}°")
                    
                    # Move to current position + 10 degrees
                    target = state.position_deg + 10.0
                    print(f"  Moving to: {target:.2f}°...")
                    
                    if controller.set_target_position(motor_id, target):
                        print("  Position command sent")
                        
                        # Wait for movement
                        print("  Waiting for movement...")
                        for i in range(10):
                            time.sleep(0.2)
                            new_state = controller.get_joint_state(motor_id)
                            if new_state:
                                print(f"    Position: {new_state.position_deg:.2f}°")
                    else:
                        print("  Failed to set position")
            else:
                print("  Failed to set work mode")
            
            # Disable motor
            print("\n  Disabling motor...")
            if controller.enable_motor(motor_id, False):
                print("  Motor disabled successfully")
        else:
            print("  Failed to enable motor")
        
        print()
        print("=" * 70)
        print("Test completed successfully!")
        print("=" * 70)
        
    except KeyboardInterrupt:
        print("\n[Interrupted] Stopped by user")
    except Exception as e:
        print(f"\n[Error] {type(e).__name__}: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if controller:
            try:
                # Ensure motor is disabled
                controller.enable_motor(motor_id, False)
            except:
                pass
        if driver:
            driver.close()
        print("\n[Cleanup] Resources released")


if __name__ == "__main__":
    main()
