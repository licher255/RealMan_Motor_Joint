"""
Hybrid Control Example - WHJ + Kinco Motors

Demonstrates simultaneous control of both motor types on the same CAN FD bus:
- RealMan WHJ joint motors (CAN FD)
- Kinco FD1X5 servo motors (Standard CAN)

Hardware Setup:
- ZLG USBCANFD-100U-mini (or compatible)
- WHJ motor connected to CAN FD bus
- Kinco motor connected to the same bus (uses standard CAN frames)
"""

import sys
import os

# Add SDK to path (in production, would be installed via pip)
sdk_path = os.path.join(os.path.dirname(__file__), '..', 'src')
sys.path.insert(0, sdk_path)

from src.zlg_can import ZlgCanDriver, ZCANDeviceType
from src.realman_whj import WHJMotor, MotionProfile
from src.kinco_motor import KincoMotor


def main():
    print("=" * 60)
    print("Hybrid Control - WHJ + Kinco Motors")
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
        print("    Mixed mode: CAN FD (WHJ) + Standard CAN (Kinco)")
        
    except Exception as e:
        print(f"    Failed to initialize CAN: {e}")
        return 1
    
    # Create motor controllers
    print("\n[2] Creating motor controllers...")
    
    # WHJ Motor (CAN FD)
    whj_profile = MotionProfile(
        max_velocity=500.0,
        max_acceleration=1000.0,
        max_deceleration=1000.0,
    )
    
    whj_motor = WHJMotor(
        can_interface=can,
        motor_id=7,
        name="WHJ_Joint_7"
    )
    whj_motor.profile = whj_profile
    
    # Kinco Motor (Standard CAN)
    kinco_motor = KincoMotor(
        can_interface=can,
        node_id=1,
        name="Kinco_Servo_1"
    )
    
    try:
        # Initialize both motors
        print("\n[3] Initializing WHJ motor...")
        if not whj_motor.initialize():
            print("    Failed to initialize WHJ motor")
            return 1
        print("    WHJ motor ready")
        
        print("\n[4] Initializing Kinco motor...")
        if not kinco_motor.initialize():
            print("    Failed to initialize Kinco motor")
            return 1
        print("    Kinco motor ready")
        
        # Read initial positions
        print("\n[5] Reading initial positions...")
        whj_pos = whj_motor.get_position()
        kinco_pos = kinco_motor.get_position()
        print(f"    WHJ position: {whj_pos:.2f}°" if whj_pos else "    WHJ: position unknown")
        print(f"    Kinco position: {kinco_pos:.2f}°" if kinco_pos else "    Kinco: position unknown")
        
        # Synchronized motion demo
        print("\n[6] Synchronized motion demo...")
        
        targets = [
            (30.0, 45.0),   # WHJ: 30°, Kinco: 45°
            (60.0, 90.0),   # WHJ: 60°, Kinco: 90°
            (90.0, 135.0),  # WHJ: 90°, Kinco: 135°
            (0.0, 0.0),     # Both home
        ]
        
        for whj_target, kinco_target in targets:
            print(f"\n    Moving to WHJ={whj_target}°, Kinco={kinco_target}°...")
            
            # Send commands to both motors (non-blocking)
            whj_motor.move_to(whj_target, blocking=False)
            kinco_motor.move_to_degree(kinco_target, velocity_rpm=50.0)
            
            # Wait for completion (simple approach)
            import time
            time.sleep(2.0)
            
            # Read positions
            whj_pos = whj_motor.get_position()
            kinco_pos = kinco_motor.get_position()
            print(f"    WHJ: {whj_pos:.2f}°, Kinco: {kinco_pos:.2f}°")
        
        print("\n[7] All motions completed!")
        
        # Set Kinco origin
        print("\n[8] Setting Kinco origin...")
        kinco_motor.set_origin()
        print("    Origin set")
        
    except KeyboardInterrupt:
        print("\n    Interrupted by user")
        
    except Exception as e:
        print(f"\n    Error: {e}")
        
    finally:
        # Cleanup
        print("\n[9] Cleaning up...")
        whj_motor.close()
        kinco_motor.close()
        can.close()
        print("    Cleanup completed")
    
    print("\n" + "=" * 60)
    print("Example completed!")
    print("=" * 60)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
