"""
IAP Handshake Test

测试WHJ电机的IAP握手功能。

Usage:
    python test_iap.py [motor_id]

IAP Protocol:
    Send:    ID 0x007, CANFD, Data=[0x02, 0x49, 0x00]
    Receive: ID 0x107, CANFD, Data=[0x02, 0x49, 0x01] (success)
"""

import sys
import time

from core import ZlgCanDriver, ZCANDeviceType
from drivers import MotorController


def test_iap_handshake(motor_id: int = 7):
    """测试IAP握手"""
    print("=" * 60)
    print("IAP Handshake Test")
    print("=" * 60)
    print(f"Motor ID: {motor_id}")
    print()
    
    driver = None
    motor = None
    
    try:
        # 打开CAN设备
        print("[1] Opening CAN device...")
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        print("[OK] CAN device opened")
        
        # 创建电机控制器
        print("[2] Creating motor controller...")
        motor = MotorController(driver, motor_id)
        
        # 测试IAP握手
        print("[3] Testing IAP handshake...")
        print("-" * 60)
        
        success = motor.iap_handshake(timeout_ms=3000)
        
        print("-" * 60)
        if success:
            print("[OK] IAP handshake successful!")
        else:
            print("[FAIL] IAP handshake failed!")
        
        # 如果IAP成功，继续测试完整初始化
        if success:
            print("\n[4] Testing full initialization...")
            if motor.initialize():
                print("[OK] Full initialization successful!")
            else:
                print("[FAIL] Full initialization failed!")
        
    except Exception as e:
        print(f"\n[Error] {e}")
        import traceback
        traceback.print_exc()
    finally:
        if driver:
            print("\n[Cleanup] Closing device...")
            driver.close()
    
    print("\n" + "=" * 60)
    print("Test completed")
    print("=" * 60)


if __name__ == "__main__":
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    test_iap_handshake(motor_id)
