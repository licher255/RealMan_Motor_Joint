"""
Dual Motor Basic Control Example

WHJ60 + Kinco 双电机基础控制示例。

Hardware:
    - ZLG USBCANFD-100U-mini
    - RealMan WHJ60 motor (ID=7)
    - Kinco servo motor (ID=1)

Usage:
    python -m examples.dual_motor_basic
"""

import sys
import os
import time

# 添加父目录到路径 (支持从任何位置运行)
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from utils import DualMotorManager


def main():
    print("=" * 60)
    print("Dual Motor Basic Control Example")
    print("=" * 60)
    
    try:
        # 使用上下文管理器
        with DualMotorManager() as manager:
            # 1. 初始化设备
            print("\n[1] Initializing CAN device...")
            if not manager.init_device():
                print("Failed to initialize device!")
                return
            
            # 2. 添加电机
            print("[2] Adding motors...")
            manager.add_whj(motor_id=7)
            manager.add_kinco(motor_id=1)
            
            # 3. 启动所有电机
            print("[3] Starting all motors...")
            manager.start_all()
            
            # 4. 同步控制演示
            print("\n[4] Synchronized control demo:")
            
            # 同步移动到不同位置
            print("\n  Move 1: WHJ->45°, Kinco->90°")
            manager.sync_move(whj_pos=45.0, kinco_pos=90.0)
            time.sleep(2.0)
            
            print("\n  Move 2: WHJ->0°, Kinco->0°")
            manager.sync_move(whj_pos=0.0, kinco_pos=0.0)
            time.sleep(2.0)
            
            # 5. 运动序列
            print("\n[5] Running motion sequence:")
            sequence = [
                {'whj': 30.0, 'kinco': 45.0},
                {'whj': 60.0, 'kinco': 90.0},
                {'whj': 30.0, 'kinco': 45.0},
                {'whj': 0.0, 'kinco': 0.0},
            ]
            manager.sync_sequence(sequence, delay=2.0)
            
            # 6. 打印状态
            print("\n[6] Final status:")
            manager.print_status()
        
        print("\n[Done] Example completed successfully!")
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n[Error] {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
