"""
WHJ60 + Kinco 基础双电机控制示例

这是最简单的双电机控制示例,演示如何同时控制WHJ60和Kinco电机。

硬件配置:
- WHJ60: CAN ID = 7
- Kinco: CAN ID = 1
- CAN波特率: 1Mbps (仲裁段)
- CAN FD数据段: 5Mbps (仅WHJ使用)

使用方法:
1. 连接ZLG USBCANFD-100U-mini设备
2. 连接WHJ60电机 (ID=7) 和 Kinco电机 (ID=1) 到CAN总线
3. 运行: python example_dual_motor_basic.py
"""

import time
import sys

# 导入驱动模块
from zlgcan_driver import ZlgCanDriver, ZCANDeviceType
from whj_protocol import WHJProtocol, Register, WorkMode
from kinco_driver import KincoDriver


def main():
    print("=" * 60)
    print("WHJ60 + Kinco Basic Dual Motor Control")
    print("=" * 60)
    
    # 创建CAN驱动
    can_driver = ZlgCanDriver()
    
    try:
        # 打开ZLG设备
        print("\n[1] Opening ZLG CANFD device...")
        can_driver.open(ZCANDeviceType.USBCANFD_MINI, device_index=0, channel=0)
        
        # 初始化CAN FD (1M/5M)
        # 注意: 虽然Kinco使用标准CAN,但ZLG设备可以混合处理
        print("[2] Initializing CAN FD (1Mbps/5Mbps)...")
        can_driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        
        # 创建Kinco驱动 (ID=1)
        print("[3] Initializing Kinco motor (ID=1)...")
        kinco = KincoDriver(can_driver, motor_id=1)
        
        # 启动Kinco节点
        print("[4] Starting Kinco node (NMT)...")
        kinco.start_node()
        time.sleep(0.3)
        
        # 设置Kinco为绝对位置模式
        print("[5] Setting Kinco to absolute position mode...")
        kinco.set_absolute_position_mode()
        time.sleep(0.2)
        
        # 初始化WHJ60 (ID=7)
        print("[6] Initializing WHJ60 motor (ID=7)...")
        whj_id = 7
        
        # 使能WHJ60
        print("    Enabling WHJ60...")
        cmd = WHJProtocol.build_enable_motor(whj_id, True)
        can_driver.send(can_id=whj_id, data=cmd)
        time.sleep(0.1)
        
        # 设置为位置模式
        print("    Setting WHJ60 to position mode...")
        cmd = WHJProtocol.build_set_work_mode(whj_id, WorkMode.POSITION_MODE)
        can_driver.send(can_id=whj_id, data=cmd)
        time.sleep(0.1)
        
        # 清空接收缓冲区
        can_driver.clear_buffer()
        
        print("\n" + "=" * 60)
        print("All motors initialized successfully!")
        print("=" * 60)
        
        # 主控制循环
        print("\n[7] Starting control loop...")
        print("Commands:")
        print("  1 - Move WHJ to 45°, Kinco to 90°")
        print("  2 - Move WHJ to 0°, Kinco to 0°")
        print("  3 - Sync demo (90° -> 0°)")
        print("  s - Query WHJ state")
        print("  q - Quit")
        print("-" * 60)
        
        while True:
            try:
                cmd = input("\nEnter command: ").strip().lower()
                
                if cmd == 'q':
                    break
                
                elif cmd == '1':
                    print("\n[Action] Moving to position 1...")
                    
                    # WHJ -> 45°
                    cmds = WHJProtocol.build_set_target_position(whj_id, 45.0)
                    for c in cmds:
                        can_driver.send(can_id=whj_id, data=c)
                        time.sleep(0.01)
                    
                    # Kinco -> 90°
                    kinco.move_to_position(90.0, speed_rpm=50)
                    
                    print("  Commands sent!")
                
                elif cmd == '2':
                    print("\n[Action] Moving to home position...")
                    
                    # WHJ -> 0°
                    cmds = WHJProtocol.build_set_target_position(whj_id, 0.0)
                    for c in cmds:
                        can_driver.send(can_id=whj_id, data=c)
                        time.sleep(0.01)
                    
                    # Kinco -> 0°
                    kinco.move_to_position(0.0, speed_rpm=50)
                    
                    print("  Commands sent!")
                
                elif cmd == '3':
                    print("\n[Action] Running sync demo...")
                    
                    # 转到90度
                    print("  Moving to 90°...")
                    cmds = WHJProtocol.build_set_target_position(whj_id, 90.0)
                    for c in cmds:
                        can_driver.send(can_id=whj_id, data=c)
                    kinco.move_to_position(90.0, speed_rpm=50)
                    time.sleep(2.0)
                    
                    # 转回0度
                    print("  Moving to 0°...")
                    cmds = WHJProtocol.build_set_target_position(whj_id, 0.0)
                    for c in cmds:
                        can_driver.send(can_id=whj_id, data=c)
                    kinco.move_to_position(0.0, speed_rpm=50)
                    time.sleep(2.0)
                    
                    print("  Demo completed!")
                
                elif cmd == 's':
                    print("\n[Action] Querying WHJ state...")
                    
                    # 查询WHJ状态
                    cmd = WHJProtocol.build_read_state(whj_id)
                    can_driver.send(can_id=whj_id, data=cmd)
                    
                    # 等待响应 (最多等待10帧)
                    found = False
                    for _ in range(10):
                        frame = can_driver.receive_frame(timeout_ms=50)
                        if frame and frame.can_id == whj_id + 0x100:
                            # 解析响应
                            try:
                                state = WHJProtocol.parse_state_response(whj_id, frame.data)
                                print(f"  WHJ-{whj_id} State:")
                                print(f"    Position: {state.position_deg:.4f}°")
                                print(f"    Speed: {state.speed_rpm:.2f} RPM")
                                print(f"    Current: {state.current_ma} mA")
                                found = True
                                break
                            except Exception as e:
                                print(f"    Parse error: {e}")
                        elif frame:
                            # 打印其他帧 (可能是Kinco的)
                            print(f"  Other frame: ID=0x{frame.can_id:03X}, Data={frame.data.hex()}")
                    
                    if not found:
                        print("  No WHJ response received")
                
                else:
                    print("Unknown command")
                    
            except KeyboardInterrupt:
                print("\nInterrupted by user")
                break
            except Exception as e:
                print(f"Error: {e}")
        
        # 停止所有电机
        print("\n[8] Stopping all motors...")
        kinco.quick_stop()
        cmd = WHJProtocol.build_enable_motor(whj_id, False)
        can_driver.send(can_id=whj_id, data=cmd)
        
        print("[Done] All motors stopped")
        
    except Exception as e:
        print(f"\n[Error] {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 关闭设备
        print("\n[9] Closing device...")
        can_driver.close()
        print("[Done] Device closed")


if __name__ == "__main__":
    main()
