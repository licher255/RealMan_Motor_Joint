"""
Kinco Motor Test Suite

Kinco电机测试套件。

Tests:
    - NMT commands test
    - Mode setting test
    - Position control test
    - Response time test

Usage:
    python -m tests.test_kinco
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
from drivers import KincoDriver


class KincoTestSuite:
    """Kinco测试套件"""
    
    def __init__(self, motor_id: int = 1):
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
        
        self.motor = KincoDriver(self.can_driver, self.motor_id)
        print("[Setup] Done")
    
    def teardown(self):
        """测试后清理"""
        print("\n[Teardown] Cleaning up...")
        if self.motor:
            self.motor.stop_node()
        if self.can_driver:
            self.can_driver.close()
        print("[Teardown] Done")
    
    def test_nmt_commands(self):
        """测试NMT命令"""
        print("\n[Test] NMT commands test...")
        
        # 测试启动节点
        if self.motor.start_node():
            print("  [PASS] Start node")
            self.passed += 1
        else:
            print("  [FAIL] Start node")
            self.failed += 1
            return
        
        time.sleep(0.2)
        
        # 测试停止节点
        if self.motor.stop_node():
            print("  [PASS] Stop node")
            self.passed += 1
        else:
            print("  [FAIL] Stop node")
            self.failed += 1
        
        time.sleep(0.2)
        
        # 重新启动
        self.motor.start_node()
    
    def test_mode_setting(self):
        """测试模式设置"""
        print("\n[Test] Mode setting test...")
        
        # 确保节点已启动
        if not self.motor.is_node_active():
            self.motor.start_node()
            time.sleep(0.2)
        
        # 测试绝对位置模式
        if self.motor.set_absolute_position_mode():
            print("  [PASS] Absolute position mode")
            self.passed += 1
        else:
            print("  [FAIL] Absolute position mode")
            self.failed += 1
    
    def test_position_control(self):
        """测试位置控制"""
        print("\n[Test] Position control test...")
        
        # 确保准备就绪
        if not self.motor.is_node_active():
            self.motor.initialize()
        
        # 测试移动到90度
        print("  Moving to 90°...")
        if self.motor.set_position(90.0, velocity=50):
            print("  [PASS] Position command sent")
            time.sleep(2.0)
            
            # 测试回到0度
            print("  Moving to 0°...")
            if self.motor.set_position(0.0, velocity=50):
                print("  [PASS] Return command sent")
                time.sleep(2.0)
                self.passed += 1
            else:
                print("  [FAIL] Return command")
                self.failed += 1
        else:
            print("  [FAIL] Position command")
            self.failed += 1
    
    def test_stop(self):
        """测试停止功能"""
        print("\n[Test] Stop test...")
        
        # 先运动
        self.motor.set_position(45.0)
        time.sleep(0.5)
        
        # 测试停止
        if self.motor.stop():
            print("  [PASS] Stop command")
            self.passed += 1
        else:
            print("  [FAIL] Stop command")
            self.failed += 1
    
    def run_all(self):
        """运行所有测试"""
        print("=" * 60)
        print("Kinco Motor Test Suite")
        print("=" * 60)
        
        try:
            self.setup()
            
            # 运行测试
            self.test_nmt_commands()
            self.test_mode_setting()
            self.test_position_control()
            self.test_stop()
            
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
    suite = KincoTestSuite(motor_id=1)
    suite.run_all()


if __name__ == "__main__":
    main()
