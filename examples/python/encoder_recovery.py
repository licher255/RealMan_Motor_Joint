"""
Encoder Recovery for RealMan WHJ Motor
专门处理断电后手动转动电机导致的编码器错误
包括：Encoder error (Bit 5) 和 Multi-turn counter lost (Bit 15)
"""

import sys
import time
from zlgcan_driver import ZlgCanDriver, ZCANDeviceType
from whj_protocol import WHJProtocol, Register, WorkMode, ErrorCode
from motor_control import MotorController


def main():
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    
    print("=" * 60)
    print("编码器故障恢复工具 (Encoder Recovery)")
    print("=" * 60)
    print(f"Motor ID: {motor_id}")
    print()
    print("⚠️  警告: 本工具用于解决断电后手动转动电机导致的编码器错误")
    print("    包括: Encoder error, Multi-turn counter lost")
    print()
    print("⚠️  请确保:")
    print("    1. 电机已上电")
    print("    2. 电机处于机械零点位置 (如果需要重新标定)")
    print()
    
    driver = None
    
    try:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        print("✓ CAN设备已连接")
    except RuntimeError as e:
        print(f"✗ [错误] {e}")
        return
    
    motor = MotorController(driver, motor_id)
    
    # Step 1: 检查当前状态
    print("\n" + "-" * 60)
    print("[1] 检查电机状态...")
    print("-" * 60)
    
    if not motor.initialize():
        print("    ⚠️  电机无响应，但将继续尝试恢复...")
    else:
        print("    ✓ 电机在线")
    
    # 获取当前位置
    pos = motor.get_position()
    if pos is not None:
        print(f"    当前位置: {pos:.2f}°")
    
    # 检查错误
    error = motor.get_error_status()
    has_encoder_error = False
    is_multi_turn_lost = False
    
    if error:
        code, errors = error
        print(f"    错误码: 0x{code:04X}")
        if code != 0:
            print("    错误列表:")
            for e in errors:
                print(f"      - {e}")
            
            # 检查是否包含编码器相关错误
            if code & ErrorCode.ENCODER_ERROR:
                has_encoder_error = True
                print("\n    ⚠️  检测到: Encoder error (编码器错误)")
            if code & ErrorCode.MULTI_TURN_LOST:
                is_multi_turn_lost = True
                print("\n    ⚠️  检测到: Multi-turn counter lost (多圈计数器丢失)")
            if code & ErrorCode.POSITION_STEP_ERROR:
                has_encoder_error = True
                print("\n    ⚠️  检测到: Position step too large (位置跳变过大)")
    
    if not has_encoder_error and not is_multi_turn_lost and (error is None or error[0] == 0):
        print("\n    ✓ 没有检测到编码器错误，无需恢复")
        driver.close()
        return
    
    # Step 2: 禁用电机
    print("\n" + "-" * 60)
    print("[2] 禁用电机 (安全)...")
    print("-" * 60)
    motor.enable(False)
    time.sleep(0.3)
    print("    ✓ 电机已禁用")
    
    # Step 3: 清除错误
    print("\n" + "-" * 60)
    print("[3] 清除错误标志...")
    print("-" * 60)
    
    if motor.clear_error():
        print("    ✓ 清除错误命令已发送")
    else:
        print("    ⚠️  清除错误命令无响应，继续...")
    
    time.sleep(0.3)
    
    # 重新检查错误
    error = motor.get_error_status()
    if error:
        code, errors = error
        if code == 0:
            print("    ✓ 所有错误已清除!")
            has_encoder_error = False
            is_multi_turn_lost = False
        else:
            print(f"    ⚠️  仍有错误: 0x{code:04X}")
            for e in errors:
                print(f"      - {e}")
    
    # Step 4: 如果是多圈丢失或持续编码器错误，需要设置零点
    if is_multi_turn_lost or (has_encoder_error and error and error[0] != 0):
        print("\n" + "-" * 60)
        print("[4] 编码器错误无法清除，需要重新标定零点")
        print("-" * 60)
        print()
        print("⚠️  请将电机转动到机械零点位置，然后按 Enter 继续")
        print("    (机械零点通常是电机出厂时的0°位置)")
        input("    准备好后按 Enter...")
        
        # 获取当前位置
        pos = motor.get_position()
        if pos is not None:
            print(f"\n    当前位置将被设为 0° (当前读数: {pos:.2f}°)")
        
        print("\n    正在设置零点...")
        if motor.set_zero_position():
            print("    ✓ 零点设置成功!")
            
            # 等待并验证
            time.sleep(0.3)
            
            # 重新检查错误
            error = motor.get_error_status()
            if error and error[0] == 0:
                print("    ✓ 错误已清除!")
            
            # 询问是否保存到 Flash
            print("\n" + "-" * 60)
            print("[5] 保存配置到 Flash (永久保存零点)")
            print("-" * 60)
            save = input("    是否保存到 Flash? (y/n): ").strip().lower()
            if save == 'y':
                print("    禁用电机...")
                motor.enable(False)
                time.sleep(0.1)
                
                print("    保存到 Flash...")
                if motor.save_to_flash():
                    print("    ✓ 保存命令已发送")
                    print("    等待 50ms 完成写入...")
                    time.sleep(0.05)  # 必须等待 50ms
                else:
                    print("    ⚠️  保存命令发送失败")
            else:
                print("    跳过保存 (零点仅在本次上电有效)")
        else:
            print("    ✗ 零点设置失败")
    
    # Step 6: 设置为位置模式并重新启用
    print("\n" + "-" * 60)
    print("[6] 设置为位置模式并启用电机...")
    print("-" * 60)
    
    motor.set_work_mode(WorkMode.POSITION_MODE)
    time.sleep(0.1)
    
    # 设置目标位置为当前位置（保持不动）
    pos = motor.get_position()
    if pos is not None:
        motor.set_target_position(pos)
        print(f"    保持位置: {pos:.2f}°")
    
    time.sleep(0.1)
    
    if motor.enable(True):
        print("    ✓ 电机已启用!")
    else:
        print("    ⚠️  电机启用可能失败")
    
    time.sleep(0.3)
    
    # Step 7: 最终验证
    print("\n" + "-" * 60)
    print("[7] 恢复结果验证...")
    print("-" * 60)
    
    error = motor.get_error_status()
    final_code = 0xFFFF
    if error:
        final_code, errors = error
        if final_code == 0:
            print("    ✓ 所有错误已清除!")
        else:
            print(f"    ✗ 仍有错误: 0x{final_code:04X}")
            for e in errors:
                print(f"      - {e}")
    
    enabled = motor.is_enabled()
    if enabled:
        print("    ✓ 电机已启用")
    else:
        print("    ✗ 电机未启用")
    
    pos = motor.get_position()
    if pos is not None:
        print(f"    ✓ 位置反馈正常: {pos:.2f}°")
    else:
        print("    ✗ 位置反馈异常")
    
    # 最终结果
    print("\n" + "=" * 60)
    if enabled and final_code == 0:
        print("🎉 恢复成功!")
        print("=" * 60)
        print()
        print("电机现在可以正常使用。建议:")
        print("  1. 使用 simple_position_control.py 测试位置控制")
        print("  2. 检查位置零点是否正确")
        print("  3. 如有问题，重新运行本工具并确保在机械零点执行")
    else:
        print("⚠️  恢复部分成功或失败")
        print("=" * 60)
        print()
        print("可能的解决方案:")
        print("  1. 重新上电后再次尝试")
        print("  2. 检查电源功率是否足够")
        print("  3. 使用 ZCANPro 检查编码器状态")
        print("  4. 如果驱动板亮蓝灯，可能需要返厂维修")
    
    print()
    driver.close()


if __name__ == "__main__":
    main()
