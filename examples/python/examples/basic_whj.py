"""
WHJ Motor Basic Control Example

WHJ关节电机基础控制示例，使用平滑轨迹规划。

直接使用 motion_controller.py 中的 SmoothMotorController。

Hardware:
    - ZLG USBCANFD-100U-mini
    - RealMan WHJ60 motor (ID=7)

Usage:
    python -m examples.basic_whj
    python -m examples.basic_whj [motor_id]
"""

import sys
import os

# 添加父目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from core import ZlgCanDriver, ZCANDeviceType
from drivers import SmoothMotorController, MotionProfile


def main():
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    
    print("=" * 70)
    print("WHJ Motor Basic Control Example")
    print("(With Trajectory Planning)")
    print("=" * 70)
    print(f"Motor ID: {motor_id}")
    
    # 创建CAN驱动
    driver = None
    motor = None
    
    try:
        # 1. 打开设备
        print("\n[1] Opening CAN device...")
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        
        # 2. 创建运动控制器，使用保守的运动参数
        print("[2] Creating motion controller...")
        
        profile = MotionProfile(
            max_velocity=1800.0,      # 180°/s (3 RPM) - 保守
            max_acceleration=720.0,   # 360°/s^2
            max_deceleration=720.0
        )
        
        motor = SmoothMotorController(driver, motor_id, profile)
        
        # 3. 初始化电机
        print("[3] Initializing motor...")
        if not motor.initialize():
            print("[Error] Failed to initialize motor")
            driver.close()
            return
        
        print("\n" + "-" * 70)
        print("Motion Controller Ready")
        print("-" * 70)
        print(f"Motion Profile:")
        print(f"  Max Velocity: {profile.max_velocity}°/s ({profile.max_velocity/6:.1f} RPM)")
        print(f"  Max Acceleration: {profile.max_acceleration}°/s²")
        print()
        print("Commands:")
        print("  m <pos>  - Move to position with smooth trajectory")
        print("  e        - Enable motor")
        print("  d        - Disable motor")
        print("  c        - Clear errors")
        print("  r        - Read current position")
        print("  s        - Show status")
        print("  q        - Quit")
        print()
        
        # 4. 交互式控制
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
                    motor.stop()
                    if motor.enable(False):
                        print("Motor disabled")
                    else:
                        print("Failed to disable")
                
                elif cmd == 'c':
                    if motor.clear_error():
                        print("Errors cleared")
                    else:
                        print("Failed to clear errors")
                
                elif cmd == 'm' and len(parts) >= 2:
                    try:
                        target = float(parts[1])
                        motor.move_to_position(target, wait=True)
                    except ValueError:
                        print("Usage: m <position_in_degrees>")
                
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
                        if code != 0:
                            for e in errors:
                                print(f"  - {e}")
                    enabled = motor.is_enabled()
                    if enabled is not None:
                        print(f"Enabled: {'Yes' if enabled else 'No'}")
                    pos = motor.get_position()
                    if pos is not None:
                        print(f"Position: {pos:.4f}°")
                
                else:
                    print("Unknown command or missing argument")
            
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
        
        print("\n[Done] Example completed!")
        
    except Exception as e:
        print(f"\n[Error] {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理
        if motor:
            print("\n[Cleanup] Stopping motor...")
            motor.stop()
            try:
                motor.enable(False)
            except:
                pass
        if driver:
            driver.close()


if __name__ == "__main__":
    main()
