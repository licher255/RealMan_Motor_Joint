"""
WHJ Motor Test Suite

WHJ电机测试套件。

Tests:
    - Connection test
    - Enable/Disable test
    - Position control test
    - State query test
    - Error handling test

Usage:
    python -m tests.test_whj
"""

import sys
import os
import time

# 添加父目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from core import ZlgCanDriver, ZCANDeviceType
from drivers import WHJDriver
from core.protocol import WorkMode


class WHJTestSuite:
    """WHJ测试套件"""
    
    def __init__(self, motor_id: int = 7):
        self.motor_id = motor_id
        self.can_driver = None
        self.motor = None
        self.passed = 0
        self.failed = 0
    
    def setup(self):
        """测试前准备"""
        print("\n[Setup] Initializing CAN device...")
        self.can_driver = ZlgCanDriver()
        self.can_driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
        self.can_driver.init_canfd()
        
        self.motor = WHJDriver(self.can_driver, self.motor_id)
        print("[Setup] Done")
    
    def teardown(self):
        """测试后清理"""
        print("\n[Teardown] Cleaning up...")
        if self.motor:
            self.motor.disable()
        if self.can_driver:
            self.can_driver.close()
        print("[Teardown] Done")
    
    def test_connection(self):
        """测试连接"""
        print("\n[Test] Connection test...")
        
        if self.motor.ping():
            print("  [PASS] Motor is responsive")
            self.passed += 1
        else:
            print("  [FAIL] Motor not responding")
            self.failed += 1
    
    def test_enable_disable(self):
        """测试使能/禁用"""
        print("\n[Test] Enable/Disable test...")
        
        # 测试使能
        if self.motor.enable():
            print("  [PASS] Enable command sent")
            time.sleep(0.2)
        else:
            print("  [FAIL] Enable failed")
            self.failed += 1
            return
        
        # 测试禁用
        if self.motor.disable():
            print("  [PASS] Disable command sent")
            self.passed += 1
        else:
            print("  [FAIL] Disable failed")
            self.failed += 1
    
    def test_position_control(self):
        """测试位置控制"""
        print("\n[Test] Position control test...")
        
        # 先使能
        self.motor.enable()
        time.sleep(0.2)
        
        # 测试移动到45度
        target = 45.0
        print(f"  Moving to {target}°...")
        
        if self.motor.set_position(target):
            # 等待到达
            if self.motor.wait_for_position(target, tolerance=2.0, timeout=5.0):
                state = self.motor.get_state()
                if state:
                    error = abs(state.position - target)
                    if error <= 2.0:
                        print(f"  [PASS] Position reached (error={error:.2f}°)")
                        self.passed += 1
                    else:
                        print(f"  [FAIL] Position error too large ({error:.2f}°)")
                        self.failed += 1
            else:
                print("  [FAIL] Timeout waiting for position")
                self.failed += 1
        else:
            print("  [FAIL] Failed to send position command")
            self.failed += 1
        
        # 回到0度
        print("  Returning to 0°...")
        self.motor.set_position(0.0)
        time.sleep(2.0)
    
    def test_state_query(self):
        """测试状态查询"""
        print("\n[Test] State query test...")
        
        state = self.motor.get_state(query=True)
        if state:
            print(f"  Position: {state.position:.2f}°")
            print(f"  Velocity: {state.velocity:.2f} RPM")
            print(f"  Current: {state.current:.0f} mA")
            print("  [PASS] State query successful")
            self.passed += 1
        else:
            print("  [FAIL] Failed to get state")
            self.failed += 1
    
    def test_error_handling(self):
        """测试错误处理"""
        print("\n[Test] Error handling test...")
        
        # 获取错误码
        state = self.motor.get_state()
        if state:
            if state.error_code == 0:
                print(f"  [PASS] No errors (code=0x{state.error_code:04X})")
                self.passed += 1
            else:
                print(f"  [WARN] Error detected: 0x{state.error_code:04X}")
                # 尝试清除错误
                if self.motor.clear_error():
                    print("  [PASS] Error cleared")
                    self.passed += 1
                else:
                    print("  [FAIL] Failed to clear error")
                    self.failed += 1
    
    def run_all(self):
        """运行所有测试"""
        print("=" * 60)
        print("WHJ Motor Test Suite")
        print("=" * 60)
        
        try:
            self.setup()
            
            # 运行测试
            self.test_connection()
            self.test_enable_disable()
            self.test_position_control()
            self.test_state_query()
            self.test_error_handling()
            
        except Exception as e:
            print(f"\n[Error] Test interrupted: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.teardown()
        
        # 打印结果
        print("\n" + "=" * 60)
        print("Test Results")
        print("=" * 60)
        print(f"Passed: {self.passed}")
        print(f"Failed: {self.failed}")
        print(f"Total:  {self.passed + self.failed}")
        
        if self.failed == 0:
            print("\nAll tests passed! ✓")
        else:
            print(f"\n{self.failed} test(s) failed ✗")
        print("=" * 60)


def main():
    suite = WHJTestSuite(motor_id=7)
    suite.run_all()


if __name__ == "__main__":
    main()
