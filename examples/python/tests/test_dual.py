"""
Dual Motor Test Suite

双电机测试套件。

Tests:
    - Initialization test
    - Sync control test
    - Communication conflict test
    - State monitoring test

Usage:
    python -m tests.test_dual
"""

import sys
import os
import time

# 添加父目录到路径
script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from utils import DualMotorManager


class DualMotorTestSuite:
    """双电机测试套件"""
    
    def __init__(self, whj_id: int = 7, kinco_id: int = 1):
        self.whj_id = whj_id
        self.kinco_id = kinco_id
        self.manager = None
        self.passed = 0
        self.failed = 0
    
    def setup(self):
        """测试前准备"""
        print("\n[Setup] Initializing...")
        self.manager = DualMotorManager()
        
        if not self.manager.init_device():
            raise RuntimeError("Failed to initialize device")
        
        self.manager.add_whj(self.whj_id)
        self.manager.add_kinco(self.kinco_id)
        print("[Setup] Done")
    
    def teardown(self):
        """测试后清理"""
        print("\n[Teardown] Cleaning up...")
        if self.manager:
            self.manager.close()
        print("[Teardown] Done")
    
    def test_initialization(self):
        """测试初始化"""
        print("\n[Test] Initialization test...")
        
        try:
            self.manager.start_all()
            print("  [PASS] All motors initialized")
            self.passed += 1
        except Exception as e:
            print(f"  [FAIL] Initialization error: {e}")
            self.failed += 1
    
    def test_sync_control(self):
        """测试同步控制"""
        print("\n[Test] Sync control test...")
        
        try:
            # 测试同步移动
            print("  Moving to different positions...")
            self.manager.sync_move(whj_pos=30.0, kinco_pos=60.0)
            time.sleep(2.0)
            
            # 更新状态
            self.manager.update()
            
            # 检查状态
            whj_state = self.manager.get_whj_state(self.whj_id)
            if whj_state:
                print(f"  WHJ position: {whj_state.position:.2f}°")
            
            # 回到0度
            print("  Returning to 0°...")
            self.manager.sync_move(whj_pos=0.0, kinco_pos=0.0)
            time.sleep(2.0)
            
            print("  [PASS] Sync control")
            self.passed += 1
            
        except Exception as e:
            print(f"  [FAIL] Sync control error: {e}")
            self.failed += 1
    
    def test_sequence(self):
        """测试运动序列"""
        print("\n[Test] Motion sequence test...")
        
        try:
            sequence = [
                {'whj': 20.0, 'kinco': 40.0},
                {'whj': 40.0, 'kinco': 80.0},
                {'whj': 0.0, 'kinco': 0.0},
            ]
            
            self.manager.sync_sequence(sequence, delay=1.5)
            print("  [PASS] Motion sequence")
            self.passed += 1
            
        except Exception as e:
            print(f"  [FAIL] Sequence error: {e}")
            self.failed += 1
    
    def test_communication(self):
        """测试通信冲突解决"""
        print("\n[Test] Communication test...")
        
        try:
            # 连续发送多个命令
            print("  Sending rapid commands...")
            for i in range(5):
                self.manager.sync_move(whj_pos=i*10.0, kinco_pos=i*20.0)
                time.sleep(0.1)
            
            time.sleep(1.0)
            
            # 检查是否都能收到响应
            self.manager.update()
            whj_state = self.manager.get_whj_state(self.whj_id)
            
            if whj_state:
                print(f"  WHJ responsive, position={whj_state.position:.2f}°")
                print("  [PASS] Communication OK")
                self.passed += 1
            else:
                print("  [FAIL] WHJ not responding")
                self.failed += 1
                
        except Exception as e:
            print(f"  [FAIL] Communication error: {e}")
            self.failed += 1
    
    def test_status(self):
        """测试状态监控"""
        print("\n[Test] Status monitoring test...")
        
        try:
            self.manager.print_status()
            print("  [PASS] Status retrieved")
            self.passed += 1
        except Exception as e:
            print(f"  [FAIL] Status error: {e}")
            self.failed += 1
    
    def run_all(self):
        """运行所有测试"""
        print("=" * 60)
        print("Dual Motor Test Suite")
        print("=" * 60)
        
        try:
            self.setup()
            
            # 运行测试
            self.test_initialization()
            self.test_sync_control()
            self.test_sequence()
            self.test_communication()
            self.test_status()
            
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
    suite = DualMotorTestSuite(whj_id=7, kinco_id=1)
    suite.run_all()


if __name__ == "__main__":
    main()
