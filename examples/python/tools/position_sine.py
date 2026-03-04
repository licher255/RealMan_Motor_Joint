#!/usr/bin/env python3
"""
Sine wave position control example using ZLG CAN driver.

Usage:
    python position_sine.py [motor_id]
"""

import sys
import time
import math
from zlgcan_driver import ZlgCanDriver, ZCANDeviceType
from motion_controller import SmoothMotorController, MotionProfile


def main():
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    
    print("RealMan WHJ Driver - Sine Wave Position Control")
    print("=" * 60)
    print(f"Motor ID: {motor_id}")
    print()
    
    # Initialize CAN
    try:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
    except RuntimeError as e:
        print(f"[Error] {e}")
        return 1
    
    # Create motion controller
    profile = MotionProfile(
        max_velocity=180.0,      # 180°/s - conservative for sine tracking
        max_acceleration=360.0,
        max_deceleration=360.0
    )
    
    motor = SmoothMotorController(driver, motor_id, profile)
    
    # Initialize
    if not motor.initialize():
        print("[Error] Failed to initialize motor")
        driver.close()
        return 1
    
    # Get current position as center
    center_pos = motor.get_position()
    if center_pos is None:
        print("[Error] Failed to read position")
        driver.close()
        return 1
    
    print(f"Center position: {center_pos:.2f}°")
    print()
    
    # Enable motor
    if not motor.is_enabled():
        print("Enabling motor...")
        motor.enable(True)
        time.sleep(0.2)
    
    # Set position mode
    motor.set_work_mode(3)  # POSITION_MODE
    time.sleep(0.1)
    
    # Sine wave parameters
    amplitude = 10.0   # degrees
    frequency = 0.5    # Hz
    
    print(f"Sine wave: amplitude={amplitude}°, frequency={frequency}Hz")
    print("Press Ctrl+C to stop...")
    print()
    
    start_time = time.time()
    last_print = start_time
    
    try:
        while True:
            elapsed = time.time() - start_time
            
            # Calculate sine target
            target = center_pos + amplitude * math.sin(2 * math.pi * frequency * elapsed)
            
            # Send position command
            motor.set_target_position(target)
            
            # Print status every 0.5s
            now = time.time()
            if now - last_print >= 0.5:
                actual = motor.get_position()
                if actual is not None:
                    error = abs(actual - target)
                    print(f"Target: {target:7.2f}° | Actual: {actual:7.2f}° | Error: {error:5.2f}°")
                last_print = now
            
            time.sleep(0.02)  # 50Hz
    
    except KeyboardInterrupt:
        print("\n\nStopping...")
    
    # Return to center
    print(f"Returning to center: {center_pos:.2f}°")
    motor.move_to_position(center_pos, wait=True, timeout=5.0)
    
    # Disable
    motor.enable(False)
    driver.close()
    print("Done!")
    return 0


if __name__ == "__main__":
    sys.exit(main())
