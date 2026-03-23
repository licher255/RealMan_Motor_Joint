"""
Basic Kinco Motor Control Example

Demonstrates basic Kinco servo motor control:
- CAN interface initialization
- Motor initialization (NMT + enable)
- Position control
- Origin setting
"""

import sys
import os

# Add SDK to path (in production, would be installed via pip)
sdk_path = os.path.join(os.path.dirname(__file__), '..', 'src')
sys.path.insert(0, sdk_path)

from src.zlg_can import ZlgCanDriver, ZCANDeviceType
from src.kinco_motor import KincoMotor


def main():
    print("=" * 60)
    print("Kinco FD1X5 Motor Control - Basic Example")
    print("=" * 60)
    
    # Initialize CAN interface
    print("\n[1] Initializing CAN interface...")
    can = ZlgCanDriver()
    
    try:
        can.open(
            device_type=ZCANDeviceType.USBCANFD_MINI,
            device_index=0,
            channel=0
        )
        can.init_canfd(
            arbitration_bps=1_000_000,
            internal_resistance=True
        )
        print("    CAN interface initialized successfully")
        print("    Note: Kinco uses standard CAN frames on CAN FD bus")
        
    except Exception as e:
        print(f"    Failed to initialize CAN: {e}")
        return 1
    
    # Create motor controller
    print("\n[2] Creating Kinco motor controller...")
    motor = KincoMotor(
        can_interface=can,
        node_id=1,
        name="Kinco_Servo"
    )
    
    try:
        # Initialize motor
        print("\n[3] Initializing motor (NMT start + enable)...")
        if not motor.initialize():
            print("    Failed to initialize motor")
            return 1
        print("    Motor initialized successfully")
        
        # Read current position
        print("\n[4] Reading current position...")
        if motor.poll_state(duration=0.5):
            pos = motor.get_position()
            print(f"    Current position: {pos:.2f}°")
        else:
            print("    No position feedback received (check TPDO mapping)")
        
        # Move to positions
        print("\n[5] Moving to target positions...")
        positions = [45.0, 90.0, 45.0, 0.0]
        
        for target in positions:
            print(f"\n    Moving to {target}°...")
            if motor.move_to_degree(target, velocity_rpm=50.0):
                print(f"    Motion completed")
                pos = motor.get_position()
                if pos is not None:
                    print(f"    Final position: {pos:.2f}°")
            else:
                print("    Motion command failed")
        
        print("\n[6] All motions completed!")
        
        # Set origin
        print("\n[7] Setting current position as origin...")
        if motor.set_origin():
            print("    Origin set successfully")
        else:
            print("    Failed to set origin")
        
        # Go home
        print("\n[8] Moving to home position...")
        if motor.home(velocity_rpm=30.0):
            print("    Homing completed")
        else:
            print("    Homing failed")
        
    except KeyboardInterrupt:
        print("\n    Interrupted by user")
        
    finally:
        # Cleanup
        print("\n[9] Cleaning up...")
        motor.shutdown()
        can.close()
        print("    Cleanup completed")
    
    print("\n" + "=" * 60)
    print("Example completed!")
    print("=" * 60)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
