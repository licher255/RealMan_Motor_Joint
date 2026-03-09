"""
方案 C 实现：动态 CAN 滤波器切换
控制 WHJ 时屏蔽 Kinco，控制 Kinco 时恢复
"""

import sys
import time
from typing import Optional

from core import ZlgCanDriver, ZCANDeviceType
from core.protocol import WHJProtocol, Register, WorkMode
from drivers.motor_control import MotorController, parse_32bit_value


class SwitchableFilterMotorController(MotorController):
    """
    带滤波器切换的 WHJ 电机控制器
    在发送命令前自动切换到 WHJ 滤波模式
    """
    
    def __init__(self, driver, motor_id):
        super().__init__(driver, motor_id)
        self.filter_active = False
    
    def _enable_whj_filter(self):
        """启用 WHJ 专用滤波器"""
        if hasattr(self.driver, 'set_filter_for_motor_response_only'):
            self.driver.set_filter_for_motor_response_only(self.motor_id)
            self.filter_active = True
            # 滤波器切换后短暂延时，确保生效
            time.sleep(0.01)
    
    def _restore_filter(self):
        """恢复到接收所有帧（控制完 WHJ 后调用）"""
        if self.filter_active and hasattr(self.driver, 'set_filter_accept_all'):
            self.driver.set_filter_accept_all()
            self.filter_active = False
            time.sleep(0.01)
    
    def send_command(self, data, timeout_ms=1500, retry_count=5):
        """
        发送命令，自动启用 WHJ 滤波器
        """
        # 发送前启用 WHJ 专用滤波器
        self._enable_whj_filter()
        
        try:
            result = super().send_command(data, timeout_ms, retry_count)
            return result
        finally:
            # 注意：这里可以选择是否恢复滤波器
            # 如果紧接着还要通信，保持 WHJ 滤波器更高效
            pass
    
    def get_position_fast(self, timeout_ms=500):
        """
        快速获取位置，使用优化的超时和重试
        """
        self._enable_whj_filter()
        
        try:
            cmd = WHJProtocol.build_read_frame(self.motor_id, Register.CUR_POSITION_L, 2)
            
            # 清空缓冲区
            self.driver.clear_buffer()
            
            # 发送
            if not self.driver.send(can_id=self.motor_id, data=cmd, bitrate_switch=self.USE_BRS):
                return None
            
            # 快速等待响应
            start = time.time()
            while (time.time() - start) * 1000 < timeout_ms:
                frame = self.driver.receive_frame(timeout_ms=0)
                if frame and frame.can_id == self.response_id:
                    if len(frame.data) >= 6:
                        low = frame.data[2] | (frame.data[3] << 8)
                        high = frame.data[4] | (frame.data[5] << 8)
                        raw = parse_32bit_value(low, high)
                        return raw * 0.0001
                time.sleep(0.0005)
            
            return None
        finally:
            pass  # 保持 WHJ 滤波器，下次通信更快
    
    def restore_filter(self):
        """外部调用：恢复到接收所有帧"""
        self._restore_filter()


class DualMotorManager:
    """
    双电机管理器：协调 WHJ 和 Kinco 的控制
    在切换控制目标时自动切换滤波器
    """
    
    def __init__(self, driver: ZlgCanDriver, whj_id: int, kinco_id: int):
        self.driver = driver
        self.whj_id = whj_id
        self.kinco_id = kinco_id
        
        # 创建 WHJ 控制器（带滤波切换）
        self.whj = SwitchableFilterMotorController(driver, whj_id)
        
        # 当前激活的电机
        self.active_motor = None
    
    def control_whj(self, operation, *args, **kwargs):
        """
        执行 WHJ 控制操作，自动切换到 WHJ 滤波模式
        
        Args:
            operation: 操作名称字符串或 callable
            *args, **kwargs: 操作参数
        
        Returns:
            操作结果
        """
        # 切换到 WHJ 滤波模式
        self.whj._enable_whj_filter()
        self.active_motor = 'whj'
        
        try:
            if callable(operation):
                return operation(*args, **kwargs)
            else:
                # 如果是字符串方法名
                method = getattr(self.whj, operation)
                return method(*args, **kwargs)
        finally:
            # 控制完成后可以选择恢复，或保持 WHJ 模式
            pass
    
    def control_kinco(self, operation):
        """
        执行 Kinco 控制操作，恢复接收所有帧
        
        Args:
            operation: callable 操作
        """
        # 恢复接收所有帧，否则收不到 Kinco 响应
        self.whj.restore_filter()
        self.active_motor = 'kinco'
        
        try:
            return operation()
        finally:
            pass
    
    def whj_move_to_position(self, target_pos: float, timeout: float = None) -> bool:
        """
        WHJ 移动到指定位置（简化版，不带轨迹规划）
        """
        self.whj._enable_whj_filter()
        
        # 获取当前位置
        current_pos = self.whj.get_position_fast()
        if current_pos is None:
            print("[Error] 无法获取当前位置")
            return False
        
        print(f"[WHJ] 当前位置: {current_pos:.2f}°, 目标: {target_pos:.2f}°")
        
        # 确保电机使能并处于位置模式
        if not self.whj.is_enabled():
            print("[WHJ] 使能电机...")
            self.whj.enable(True)
            time.sleep(0.1)
        
        current_mode = self.whj.get_work_mode()
        if current_mode != "POSITION_MODE (Position)":
            print("[WHJ] 设置位置模式...")
            self.whj.set_work_mode(WorkMode.POSITION_MODE)
            time.sleep(0.05)
        
        # 发送目标位置
        if self.whj.set_target_position(target_pos):
            print(f"[WHJ] 目标位置已设置: {target_pos:.2f}°")
            
            # 等待完成（简单轮询）
            start = time.time()
            auto_timeout = timeout or max(abs(target_pos - current_pos) / 30.0, 3.0)
            
            while (time.time() - start) < auto_timeout:
                time.sleep(0.1)
                actual = self.whj.get_position_fast(timeout_ms=200)
                if actual is not None:
                    error = abs(actual - target_pos)
                    if error < 1.0:
                        print(f"[WHJ] 已到达目标位置: {actual:.2f}° (误差: {error:.3f}°)")
                        return True
                    if int(time.time() - start) % 2 == 0:
                        print(f"  目标: {target_pos:.2f}° | 实际: {actual:.2f}° | 误差: {error:.2f}°")
            
            print(f"[WHJ] 移动超时")
            return False
        else:
            print("[WHJ] 设置位置失败")
            return False
    
    def close(self):
        """清理资源"""
        try:
            self.whj.enable(False)
        except:
            pass
        self.driver.close()


def main():
    """测试方案 C"""
    whj_id = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    
    print("=" * 70)
    print("方案 C 测试：动态 CAN 滤波器切换")
    print("=" * 70)
    print(f"WHJ Motor ID: {whj_id}")
    print()
    
    # 初始化 CAN
    try:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
    except RuntimeError as e:
        print(f"[Error] 无法打开 CAN 设备: {e}")
        return
    
    # 创建管理器
    manager = DualMotorManager(driver, whj_id, kinco_id=0x601)  # 假设 Kinco ID
    
    # 初始化 WHJ
    print("[Init] 初始化 WHJ 电机...")
    if not manager.whj.initialize():
        print("[Error] WHJ 初始化失败")
        driver.close()
        return
    
    print("[Init] WHJ 初始化成功!")
    print()
    
    # 测试位置查询速度
    print("[Test] 测试位置查询速度（已启用 WHJ 滤波器）...")
    for i in range(3):
        start = time.time()
        pos = manager.whj.get_position_fast(timeout_ms=500)
        elapsed = (time.time() - start) * 1000
        if pos is not None:
            print(f"  查询 {i+1}: {elapsed:.1f}ms, 位置: {pos:.4f}°")
        else:
            print(f"  查询 {i+1}: {elapsed:.1f}ms, 超时")
    
    print()
    print("-" * 70)
    print("命令:")
    print("  m <pos>  - 移动 WHJ 到指定位置")
    print("  r        - 读取 WHJ 当前位置")
    print("  e        - 使能 WHJ")
    print("  d        - 禁用 WHJ")
    print("  f        - 手动切换到 WHJ 滤波器")
    print("  a        - 恢复接收所有帧")
    print("  q        - 退出")
    print()
    
    while True:
        try:
            cmd_input = input("> ").strip().lower()
            parts = cmd_input.split()
            cmd = parts[0] if parts else ""
            
            if cmd == 'q':
                break
            
            elif cmd == 'e':
                if manager.whj.enable(True):
                    print("WHJ 已使能")
                else:
                    print("WHJ 使能失败")
            
            elif cmd == 'd':
                if manager.whj.enable(False):
                    print("WHJ 已禁用")
                else:
                    print("WHJ 禁用失败")
            
            elif cmd == 'm' and len(parts) >= 2:
                try:
                    target = float(parts[1])
                    manager.whj_move_to_position(target)
                except ValueError:
                    print("用法: m <位置度数>")
            
            elif cmd == 'r':
                start = time.time()
                pos = manager.whj.get_position_fast(timeout_ms=500)
                elapsed = (time.time() - start) * 1000
                if pos is not None:
                    print(f"当前位置: {pos:.4f}° (查询耗时: {elapsed:.1f}ms)")
                else:
                    print(f"查询失败 (耗时: {elapsed:.1f}ms)")
            
            elif cmd == 'f':
                manager.whj._enable_whj_filter()
                print("已切换到 WHJ 滤波模式")
            
            elif cmd == 'a':
                manager.whj.restore_filter()
                print("已恢复接收所有帧")
            
            else:
                print("未知命令")
        
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"错误: {e}")
    
    print("\n清理资源...")
    manager.close()
    print("完成!")


if __name__ == "__main__":
    main()
