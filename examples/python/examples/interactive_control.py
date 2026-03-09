"""
Interactive Dual Motor Control

交互式双电机控制示例。

Commands:
    w <pos>     - Set WHJ position
    k <pos>     - Set Kinco position
    s <w_pos> <k_pos>  - Sync move
    status      - Print status
    home        - Home all motors
    demo        - Run demo sequence
    quit        - Exit

Usage:
    python -m examples.interactive_control
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


def print_help():
    """打印帮助信息"""
    print("\nCommands:")
    print("  w <pos>            - Set WHJ position (degrees)")
    print("  k <pos>            - Set Kinco position (degrees)")
    print("  s <w_pos> <k_pos>  - Sync move both motors")
    print("  status             - Print current status")
    print("  home               - Home all motors (0°)")
    print("  demo               - Run demo sequence")
    print("  help               - Show this help")
    print("  quit               - Exit program")
    print()


def main():
    print("=" * 60)
    print("Interactive Dual Motor Control")
    print("=" * 60)
    print_help()
    
    try:
        with DualMotorManager() as manager:
            # 初始化
            print("[Init] Initializing device...")
            if not manager.init_device():
                print("Failed!")
                return
            
            print("[Init] Adding motors...")
            manager.add_whj(motor_id=7)
            manager.add_kinco(motor_id=1)
            
            print("[Init] Starting motors...")
            manager.start_all()
            
            print("\n[Ready] Enter commands (type 'help' for help)\n")
            
            # 交互循环
            while True:
                try:
                    cmd = input("> ").strip().lower()
                    
                    if not cmd:
                        continue
                    
                    parts = cmd.split()
                    action = parts[0]
                    
                    if action == 'quit' or action == 'q':
                        break
                    
                    elif action == 'help' or action == 'h':
                        print_help()
                    
                    elif action == 'w':
                        # WHJ控制
                        if len(parts) < 2:
                            print("Usage: w <position>")
                            continue
                        pos = float(parts[1])
                        whj = manager.whj_motors.get(7)
                        if whj:
                            whj.set_position(pos)
                            print(f"WHJ moving to {pos}°")
                    
                    elif action == 'k':
                        # Kinco控制
                        if len(parts) < 2:
                            print("Usage: k <position>")
                            continue
                        pos = float(parts[1])
                        kinco = manager.kinco_motors.get(1)
                        if kinco:
                            kinco.set_position(pos)
                            print(f"Kinco moving to {pos}°")
                    
                    elif action == 's':
                        # 同步控制
                        if len(parts) < 3:
                            print("Usage: s <whj_pos> <kinco_pos>")
                            continue
                        w_pos = float(parts[1])
                        k_pos = float(parts[2])
                        manager.sync_move(whj_pos=w_pos, kinco_pos=k_pos)
                    
                    elif action == 'status':
                        manager.update()
                        manager.print_status()
                    
                    elif action == 'home':
                        print("Homing all motors...")
                        manager.sync_move(whj_pos=0.0, kinco_pos=0.0)
                    
                    elif action == 'demo':
                        print("Running demo sequence...")
                        sequence = [
                            {'whj': 45.0, 'kinco': 90.0},
                            {'whj': 0.0, 'kinco': 0.0},
                        ]
                        manager.sync_sequence(sequence)
                    
                    else:
                        print(f"Unknown command: {action}")
                        print_help()
                
                except ValueError as e:
                    print(f"Invalid value: {e}")
                except KeyboardInterrupt:
                    print("\n")
                    break
        
        print("\n[Exit] Goodbye!")
        
    except Exception as e:
        print(f"\n[Error] {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
