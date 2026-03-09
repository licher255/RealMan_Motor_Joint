"""
Kinco Motor Basic Control Example

Kinco旋转舵盘电机基础控制示例。

Hardware:
    - ZLG USBCANFD-100U-mini
    - Kinco servo motor (ID=1)

Usage:
    python -m examples.basic_kinco
"""

import sys
import os
import time

# 添加父目录到路径 (支持从任何位置运行)
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from core import ZlgCanDriver, ZCANDeviceType
from drivers import KincoDriver


def main():
    print("=" * 60)
    print("Kinco Motor Basic Control Example")
    print("=" * 60)
    
    # 创建CAN驱动
    can_driver = ZlgCanDriver()
    
    try:
        # 1. 打开设备
        print("\n[1] Opening CAN device...")
        can_driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
        
        # 2. 初始化CAN FD (Kinco使用标准CAN, 但ZLG设备兼容)
        print("[2] Initializing CAN...")
        can_driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        
        # 3. 创建Kinco驱动
        print("[3] Creating Kinco driver (ID=1)...")
        motor = KincoDriver(can_driver, motor_id=1)
        
        # 4. 完整初始化
        print("[4] Initializing motor...")
        motor.initialize()
        
        # 5. 位置控制演示
        print("\n[5] Position control demo:")
        positions = [90.0, 0.0, 90.0, 0.0]
        
        for pos in positions:
            print(f"\n  Moving to {pos}°...")
            motor.set_position(pos, velocity=50)
            time.sleep(2.0)  # Kinco没有实时反馈，使用固定延时
            
            print(f"  Motion completed")
        
        # 6. 演示序列
        print("\n[6] Running demo sequence (3 cycles)...")
        motor.demo_sequence(cycles=3, delay=2.0)
        
        # 7. 停止节点
        print("\n[7] Stopping node...")
        motor.stop_node()
        
        print("\n[Done] Example completed successfully!")
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        motor.emergency_stop() if 'motor' in locals() else None
    except Exception as e:
        print(f"\n[Error] {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理
        if 'motor' in locals():
            motor.stop_node()
        can_driver.close()


if __name__ == "__main__":
    main()
