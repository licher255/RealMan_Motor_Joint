"""
Basic WHJ Motor Control Example

Demonstrates basic motor control operations including:
- CAN interface initialization
- Motor initialization and enable
- Position control
- Error handling
"""

import sys
import os

# Add SDK to path (in production, would be installed via pip)
sdk_path = os.path.join(os.path.dirname(__file__), '..', 'src')
sys.path.insert(0, sdk_path)

from src.realman_whj import (
    ZlgCanDriver,
    ZCANDeviceType,
    WHJMotor,
    MotionController,
    MotionProfile,
    set_log_level,
)


def main():
    # Set logging level
    set_log_level('INFO')
    
    print("=" * 60)
    print("RealMan WHJ Motor Control - Basic Example")
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
            data_bps=5_000_000,
            internal_resistance=True
        )
        print("    CAN interface initialized successfully")
        
    except Exception as e:
        print(f"    Failed to initialize CAN: {e}")
        return 1
    
    # Create motor controller
    print("\n[2] Creating motor controller...")
    
    # Custom motion profile
    profile = MotionProfile(
        max_velocity=500.0,      # deg/s
        max_acceleration=1000.0,  # deg/s^2
        max_deceleration=1000.0,  # deg/s^2
    )
    
    motor = WHJMotor(
        can_interface=can,
        motor_id=7,
        name="Joint_7"
    )
    motor.profile = profile
    
    # Set up callbacks
    def on_error(code, msg):
        print(f"    [Motor Error] Code={code}: {msg}")
    
    def on_pos_reached(pos):
        print(f"    [Position Reached] {pos:.2f}°")
    
    motor.set_error_callback(on_error)
    motor.set_position_reached_callback(on_pos_reached)
    
    try:
        # Initialize motor
        print("\n[3] Initializing motor...")
        if not motor.initialize():
            print("    Failed to initialize motor")
            return 1
        print("    Motor initialized successfully")
        
        # Read current position
        print("\n[4] Reading current position...")
        pos = motor.get_position()
        if pos is not None:
            print(f"    Current position: {pos:.2f}°")
        else:
            print("    Failed to read position")
        
        # Create motion controller with trajectory planning
        print("\n[5] Creating motion controller...")
        controller = MotionController(motor, profile)
        
        # Move to positions
        positions = [30.0, 60.0, 90.0, 60.0, 30.0, 0.0]
        
        for target_pos in positions:
            print(f"\n[6] Moving to {target_pos}°...")
            try:
                controller.move_to(target_pos)
                pos = motor.get_position()
                print(f"    Final position: {pos:.2f}°")
            except Exception as e:
                print(f"    Motion failed: {e}")
        
        print("\n[7] All motions completed successfully!")
        
    except KeyboardInterrupt:
        print("\n    Interrupted by user")
        
    finally:
        # Cleanup
        print("\n[8] Cleaning up...")
        motor.close()
        can.close()
        print("    Cleanup completed")
    
    print("\n" + "=" * 60)
    print("Example completed!")
    print("=" * 60)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
