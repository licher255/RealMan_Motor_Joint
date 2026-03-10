"""
Kinco PDO Driver Test - CAN FD Mixed Mode Support

Kinco 通过 CAN FD 通道的 CAN 向下兼容模式通信，与 WHJ 共享同一 CAN FD 通道。

操作指南指令序列:
1) 0x000, 01 00               - 启动节点
2) 0x201, 01 3F 10 00 00 00 00 00 - 使能 + 绝对位置模式
3) 0x301, 00 80 16 00 40 A7 0D 00 - 移动到 90度 @ 50rpm (90*16384=1474560=0x168000)
4) 0x301, 00 00 00 00 40 A7 0D 00 - 移动到 0度 @ 50rpm

转速计算公式:
    1 RPM = 65536 * 512 / 1875 ≈ 17895.7 units
    转速值(uint32) = RPM * 65536 * 512 / 1875 (低位在前)

CAN FD 混合模式:
    - WHJ 使用 CAN FD 帧 (ID=7)
    - Kinco 使用标准 CAN 帧 (ID=0x201, 0x301, 0x181+node_id)
    - 两者共享同一 CAN FD 通道 (1Mbps仲裁 + 5Mbps数据)
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import time
from core import ZlgCanDriver, ZCANDeviceType
from drivers import KincoPDODriver

# ============================================================================
# 可配置的转速参数 - 修改这里来调整默认转速
# ============================================================================
DEFAULT_RPM = 50.0  # 默认转速 (RPM)

# 转速转换公式: 1 RPM = 65536 * 512 / 1875 ≈ 17895.7 units
# 低位在前 (little-endian)
RPM_TO_UNITS = (65536 * 512) / 1875

def rpm_to_units(rpm: float) -> int:
    """将 RPM 转换为 Kinco 速度单位 (低位在前)"""
    return int(rpm * RPM_TO_UNITS)

def print_velocity_info():
    """打印当前转速配置信息"""
    print(f"\n[Velocity Config]")
    print(f"  Default RPM: {DEFAULT_RPM}")
    print(f"  RPM to Units: {RPM_TO_UNITS:.2f}")
    print(f"  {DEFAULT_RPM} RPM = {rpm_to_units(DEFAULT_RPM)} units (0x{rpm_to_units(DEFAULT_RPM):08X})")


def main():
    global DEFAULT_RPM
    
    print("=" * 70)
    print("Kinco PDO Driver Test - CAN FD Mixed Mode")
    print("=" * 70)
    print("\n操作指南指令:")
    print("  1) 0x000, 01 00               - NMT 启动")
    print("  2) 0x201, 01 3F 10 00 00 00 00 00 - 使能 + 绝对位置")
    print("  3) 0x301, XX XX XX XX XX XX XX XX - 位置 + 速度")
    print("\nCAN FD 混合模式:")
    print("  - Kinco 使用标准 CAN 帧 (通过 CAN FD 通道发送)")
    print("  - WHJ 使用 CAN FD 帧 (可在同一通道共存)")
    print("  - 仲裁位率: 1 Mbps, 数据位率: 5 Mbps")
    print()
    
    driver = None
    motor = None
    
    # 初始化 CAN FD 混合模式
    try:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        # 初始化 CAN FD 混合模式（支持标准 CAN 向下兼容）
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        print("[CAN] Initialized in CAN FD Mixed Mode (1M/5M)\n")
    except RuntimeError as e:
        print(f"[Error] Failed to open CAN device: {e}")
        return
    
    # 创建 PDO 驱动
    motor = KincoPDODriver(driver, node_id=1)
    
    # 注册退出处理（确保资源释放）
    import atexit
    def emergency_cleanup():
        """紧急清理函数 - 在异常退出时调用"""
        nonlocal motor, driver
        print("\n[Emergency Cleanup] Cleaning up resources...")
        if motor:
            try:
                motor.shutdown()
            except:
                pass
        if driver:
            try:
                driver.close()
                print("[Emergency Cleanup] CAN device closed")
            except:
                pass
    atexit.register(emergency_cleanup)
    
    try:
        # 初始化
        if not motor.initialize():
            print("[Error] Initialization failed!")
            return
        
        print_velocity_info()
        
        print("\n" + "-" * 70)
        print("Interactive Commands:")
        print("-" * 70)
        print("  p              - Poll and print state (轮询状态)")
        print("  m <degree>     - Move to position (e.g., 'm 90')")
        print("  h              - Home (move to 0)")
        print("  o              - Set origin (设置原点)")
        print("  e              - Enable (使能)")
        print("  d              - Disable (禁用)")
        print("  c              - Clear fault (清除错误)")
        print("  test           - Run test sequence (测试序列)")
        print("  rpm <value>    - Set default RPM (e.g., 'rpm 30')")
        print("  v              - Show velocity config")
        print("  q              - Quit")
        print("-" * 70)
        
        while True:
            try:
                cmd_input = input("\n> ").strip().lower()
                parts = cmd_input.split()
                cmd = parts[0] if parts else ""
                
                if cmd == 'q':
                    break
                
                elif cmd == 'p':
                    # 轮询状态
                    print("Polling state...")
                    if motor.poll_state(duration=0.3):
                        motor.print_state()
                    else:
                        print("No TPDO received")
                
                elif cmd == 'm' and len(parts) >= 2:
                    try:
                        target = float(parts[1])
                        print(f"\nMoving to {target} degrees @ {DEFAULT_RPM} RPM...")
                        
                        # 计算目标位置
                        position_inc = int(target * motor.GEAR_RATIO)
                        velocity_units = rpm_to_units(DEFAULT_RPM)
                        
                        print(f"  Position: {position_inc} inc (0x{position_inc:08X})")
                        print(f"  Velocity: {velocity_units} units ({DEFAULT_RPM} rpm, 0x{velocity_units:08X})")
                        
                        motor.move_to_degree(target, velocity_rpm=DEFAULT_RPM)
                        
                        # 等待到达
                        print("  Waiting for target...")
                        time.sleep(2.0)
                        motor.poll_state(duration=0.5)
                        motor.print_state()
                        
                    except ValueError:
                        print("Usage: m <degree>")
                
                elif cmd == 'h':
                    print(f"\nHoming (moving to 0 @ {DEFAULT_RPM} RPM)...")
                    motor.home(velocity_rpm=DEFAULT_RPM)
                    time.sleep(1.0)
                    motor.poll_state(duration=0.5)
                    motor.print_state()
                
                elif cmd == 'o':
                    motor.set_origin()
                
                elif cmd == 'e':
                    motor.enable_absolute_mode()
                
                elif cmd == 'd':
                    motor.disable()
                
                elif cmd == 'c':
                    motor.clear_fault()
                
                elif cmd == 'rpm' and len(parts) >= 2:
                    try:
                        new_rpm = float(parts[1])
                        DEFAULT_RPM = new_rpm
                        print(f"\n[Config] Default RPM set to {DEFAULT_RPM}")
                        print(f"         {DEFAULT_RPM} RPM = {rpm_to_units(DEFAULT_RPM)} units")
                    except ValueError:
                        print("Usage: rpm <value>")
                
                elif cmd == 'v':
                    print_velocity_info()
                
                elif cmd == 'test':
                    print("\n" + "=" * 70)
                    print("Test Sequence (按照操作指南)")
                    print(f"  Default RPM: {DEFAULT_RPM}")
                    print("=" * 70)
                    
                    # 步骤1: NMT 启动
                    print("\n[1] NMT Start...")
                    motor.start_node()
                    time.sleep(0.2)
                    
                    # 步骤2: 使能 + 绝对位置
                    print("\n[2] Enable + Absolute Position Mode...")
                    motor.enable_absolute_mode()
                    time.sleep(0.2)
                    
                    # 步骤3: 移动到 90度
                    print(f"\n[3] Move to 90 degrees @ {DEFAULT_RPM} rpm...")
                    pos_inc = int(90.0 * motor.GEAR_RATIO)
                    vel_units = rpm_to_units(DEFAULT_RPM)
                    print(f"    Expected: 0x301, [{pos_inc:08X}] [{vel_units:08X}]")
                    motor.move_to_degree(90.0, velocity_rpm=DEFAULT_RPM)
                    time.sleep(3.0)
                    motor.poll_state(duration=0.5)
                    motor.print_state()
                    
                    # 步骤4: 移动到 0度
                    print(f"\n[4] Move to 0 degrees @ {DEFAULT_RPM} rpm...")
                    vel_units = rpm_to_units(DEFAULT_RPM)
                    print(f"    Expected: 0x301, [00 00 00 00] [{vel_units:08X}]")
                    motor.home(velocity_rpm=DEFAULT_RPM)
                    time.sleep(3.0)
                    motor.poll_state(duration=0.5)
                    motor.print_state()
                    
                    print("\n" + "=" * 70)
                    print("Test sequence completed!")
                    print("=" * 70)
                
                else:
                    print("Unknown command")
            
            except KeyboardInterrupt:
                print("\nInterrupted")
                break
            except Exception as e:
                print(f"Error: {e}")
    
    finally:
        print("\n" + "=" * 70)
        print("Shutting down...")
        print("=" * 70)
        # 使用 shutdown 方法安全关闭（禁用 + NMT停止节点）
        if motor:
            try:
                motor.shutdown()
            except Exception as e:
                print(f"[Warning] Error during motor shutdown: {e}")
        
        # 关闭 CAN FD 设备（释放 ZLG CAN FD 占用）
        if driver:
            try:
                driver.close()
                print("[CAN] Device closed - ZLG CAN FD released")
            except Exception as e:
                print(f"[Warning] Error during CAN close: {e}")
        
        print("Done!")


if __name__ == "__main__":
    main()
