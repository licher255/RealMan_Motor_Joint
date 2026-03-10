"""
Kinco FD1X5 PDO Driver (蓝莓项目专用)

完全按照操作指南实现，使用 RPDO 方式通信。

操作指南指令:
1) 0x000, 01 00               - NMT 启动节点
2) 0x201, 01 3F 10 00 00 00 00 00 - 控制字+模式 (RPDO1)
3) 0x301, XX XX XX XX XX XX XX XX - 位置+速度 (RPDO2)
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import time
import struct
from typing import Optional
from dataclasses import dataclass

from core import ZlgCanDriver


@dataclass
class KincoPDOState:
    """Kinco PDO 状态缓存"""
    position_inc: int = 0
    velocity_rpm: int = 0
    current_ap: int = 0
    is_enabled: bool = False
    target_reached: bool = False


class KincoPDODriver:
    """
    Kinco FD1X5 PDO 驱动器
    
    使用 RPDO 方式直接发送过程数据，符合操作指南。
    """
    
    # CAN ID 定义
    NMT_ID = 0x000          # NMT 命令
    RPDO1_ID = 0x201        # 控制字 + 工作模式
    RPDO2_ID = 0x301        # 目标位置 + 目标速度
    TPDO1_ID_BASE = 0x181   # 实际位置 + 实际速度 (0x181 + node_id)
    
    # 机械参数
    # 根据操作指南: 90度 = 0x00168000 = 1474560 inc
    # 减速比 = 1474560 / 90 = 16384 = 2^14 (每转脉冲数)
    GEAR_RATIO = 16384
    RPM_TO_UNITS = (65536 * 512) / 1875  # ≈ 17895.7
    
    def __init__(self, can_driver: ZlgCanDriver, node_id: int = 1):
        """
        初始化驱动器
        
        Args:
            can_driver: ZLG CAN 驱动实例
            node_id: 节点 ID (默认 1)
        """
        self.can_driver = can_driver
        self.node_id = node_id
        self.state = KincoPDOState()
        self._tpdo1_id = self.TPDO1_ID_BASE + node_id
        
        print(f"[Kinco-PDO-{node_id}] Driver initialized")
    
    # ========================================================================
    # NMT Commands
    # ========================================================================
    
    def start_node(self) -> bool:
        """NMT 启动节点: 0x000, [01, node_id]"""
        data = bytes([0x01, self.node_id])
        # 强制使用标准 CAN 帧（兼容 CAN FD 混合模式）
        success = self.can_driver.send(can_id=self.NMT_ID, data=data, is_extended=False, frame_type="CAN")
        if success:
            print(f"[Kinco-{self.node_id}] NMT Start sent")
            time.sleep(0.1)
        return success
    
    def stop_node(self) -> bool:
        """NMT 停止节点: 0x000, [02, node_id]
        
        安全退出时使用，释放 CANopen 节点
        """
        data = bytes([0x02, self.node_id])
        # 强制使用标准 CAN 帧（兼容 CAN FD 混合模式）
        success = self.can_driver.send(can_id=self.NMT_ID, data=data, is_extended=False, frame_type="CAN")
        if success:
            print(f"[Kinco-{self.node_id}] NMT Stop sent (node released)")
        return success
    
    # ========================================================================
    # RPDO1: Control Word + Mode (0x201)
    # ========================================================================
    
    def send_rpdo1(self, control_low: int, control_high: int, mode: int) -> bool:
        """
        发送 RPDO1 (控制字 + 模式)
        
        格式: [control_low, control_high, mode, 0, 0, 0, 0, 0]
        
        Args:
            control_low: 控制字低位
            control_high: 控制字高位
            mode: 工作模式 (0x10=绝对位置, 0x0F=相对位置)
        """
        data = bytes([control_low, control_high, mode, 0x00, 0x00, 0x00, 0x00, 0x00])
        # 强制使用标准 CAN 帧（兼容 CAN FD 混合模式）
        success = self.can_driver.send(can_id=self.RPDO1_ID, data=data, is_extended=False, frame_type="CAN")
        if success:
            print(f"[Kinco-{self.node_id}] RPDO1: 0x{control_low:02X} 0x{control_high:02X} 0x{mode:02X}")
        return success
    
    def enable_absolute_mode(self) -> bool:
        """
        使能并设置为绝对位置模式
        操作指南: 0x201, [01 3F 10 00 00 00 00 00]
        """
        success = self.send_rpdo1(0x01, 0x3F, 0x10)
        if success:
            self.state.is_enabled = True
            time.sleep(0.05)
        return success
    
    def enable_relative_mode(self) -> bool:
        """
        使能并设置为相对位置模式
        格式: [01, 3F, 0F, 00, 00, 00, 00, 00]
        """
        success = self.send_rpdo1(0x01, 0x3F, 0x0F)
        if success:
            self.state.is_enabled = True
            time.sleep(0.05)
        return success
    
    def disable(self) -> bool:
        """
        禁用驱动器 (松轴)
        格式: [01, 06, 10, 00, 00, 00, 00, 00]
        """
        success = self.send_rpdo1(0x01, 0x06, 0x10)
        if success:
            self.state.is_enabled = False
            print(f"[Kinco-{self.node_id}] Disabled")
        return success
    
    def clear_fault(self) -> bool:
        """
        清除错误
        格式: [01, 86, 10, 00, 00, 00, 00, 00]
        """
        success = self.send_rpdo1(0x01, 0x86, 0x10)
        if success:
            print(f"[Kinco-{self.node_id}] Fault clear sent")
            time.sleep(0.1)
        return success
    
    def set_homing_mode(self) -> bool:
        """
        设置为原点模式
        格式: [01, 3F, 06, 00, 00, 00, 00, 00]
        """
        return self.send_rpdo1(0x01, 0x3F, 0x06)
    
    # ========================================================================
    # Homing (Set Origin)
    # ========================================================================
    
    def set_origin(self) -> bool:
        """
        设置当前位置为原点
        
        按照操作指南:
        1) 0x201, [06 0F 00 00 00 00 00 00] - 模式6, 控制字0F
        2) 0x201, [06 1F 00 00 00 00 00 00] - 控制字1F
        3) 0x201, [01 3F 10 00 00 00 00 00] - 回绝对位置模式
        """
        print(f"\n[Kinco-{self.node_id}] Setting origin...")
        
        # Step 1: 模式6, 控制字0F
        if not self.send_rpdo1(0x06, 0x0F, 0x00):
            return False
        time.sleep(0.1)
        
        # Step 2: 控制字1F
        if not self.send_rpdo1(0x06, 0x1F, 0x00):
            return False
        time.sleep(0.1)
        
        # Step 3: 回绝对位置模式
        if not self.enable_absolute_mode():
            return False
        
        print(f"[Kinco-{self.node_id}] Origin set successfully!")
        return True
    
    # ========================================================================
    # RPDO2: Position + Velocity (0x301)
    # ========================================================================
    
    def send_rpdo2(self, position_inc: int, velocity_units: int) -> bool:
        """
        发送 RPDO2 (目标位置 + 目标速度)
        
        操作指南格式:
        - Byte 0-3: 目标位置 (int32, little-endian)
        - Byte 4-7: 目标速度 (uint32, little-endian)
        
        示例: 0x301, [00 80 16 00, 41 A7 0D 00]
        - 位置: 0x00168000 = 1474560 inc (90 deg)
        - 速度: 0x0DA741 ≈ 894785 (50 rpm)
        """
        # 构建数据
        data = struct.pack('<i', position_inc)      # 4 bytes position (signed)
        data += struct.pack('<I', velocity_units)   # 4 bytes velocity (unsigned)
        
        # 强制使用标准 CAN 帧（兼容 CAN FD 混合模式）
        success = self.can_driver.send(can_id=self.RPDO2_ID, data=data, is_extended=False, frame_type="CAN")
        if success:
            vel_rpm = velocity_units / self.RPM_TO_UNITS
            print(f"[Kinco-{self.node_id}] RPDO2: pos={position_inc} inc ({position_inc/self.GEAR_RATIO:.2f} deg), vel={vel_rpm:.1f} rpm")
        return success
    
    def move_to_position(self, position_inc: int, velocity_rpm: float = 50.0) -> bool:
        """
        移动到目标位置
        
        Args:
            position_inc: 目标位置 (脉冲)
            velocity_rpm: 速度 (rpm)
        """
        velocity_units = int(velocity_rpm * self.RPM_TO_UNITS)
        return self.send_rpdo2(position_inc, velocity_units)
    
    def move_to_degree(self, degree: float, velocity_rpm: float = 50.0) -> bool:
        """
        移动到目标角度
        
        Args:
            degree: 目标角度 (度)
            velocity_rpm: 速度 (rpm)
        """
        position_inc = int(degree * self.GEAR_RATIO)
        return self.move_to_position(position_inc, velocity_rpm)
    
    def home(self, velocity_rpm: float = 30.0) -> bool:
        """回零位 (0度)"""
        return self.move_to_degree(0.0, velocity_rpm)
    
    def stop(self) -> bool:
        """
        停止运动 (设置当前位置为目标)
        """
        # 读取当前位置
        self.read_state()
        return self.move_to_position(self.state.position_inc, 0)
    
    # ========================================================================
    # TPDO1: Read State (0x181 + node_id)
    # ========================================================================
    
    def read_state(self, timeout_ms: int = 50, frame_type: str = "any") -> bool:
        """
        读取 TPDO1 状态 (实际位置 + 实际速度)
        
        TPDO1 格式:
        - Byte 0-3: 实际位置 (int32, little-endian, 0x6064)
        - Byte 4-7: 实际速度 (int32, little-endian, 0x606C)
        
        Args:
            timeout_ms: 超时时间（毫秒）
            frame_type: "any" - 接收任何类型, "CAN" - 只接收标准 CAN, "CANFD" - 只接收 CAN FD
        """
        # 尝试接收 TPDO1（支持 CAN FD 混合模式）
        frame = self.can_driver.receive_frame(timeout_ms=timeout_ms, frame_type=frame_type)
        if frame is None:
            return False
        
        if frame.can_id == self._tpdo1_id:
            data = frame.data
            if len(data) >= 8:
                self.state.position_inc = struct.unpack('<i', data[0:4])[0]
                # 实际速度单位与目标速度相同，需要除以 RPM_TO_UNITS 得到 RPM
                velocity_raw = struct.unpack('<i', data[4:8])[0]
                self.state.velocity_rpm = velocity_raw / self.RPM_TO_UNITS
                return True
        
        return False
    
    def poll_state(self, duration: float = 0.5) -> bool:
        """
        轮询状态一段时间
        
        Args:
            duration: 轮询时间 (秒)
        """
        start = time.time()
        while time.time() - start < duration:
            if self.read_state(timeout_ms=10):
                return True
            time.sleep(0.01)
        return False
    
    # ========================================================================
    # High Level Control
    # ========================================================================
    
    def initialize(self) -> bool:
        """
        完整初始化流程 (按照操作指南)
        
        1) 0x000, 01 00 - 启动节点
        2) 0x201, 01 3F 10 00 00 00 00 00 - 使能 + 绝对位置模式
        
        Note: 支持 CAN FD 混合模式，可以在与 WHJ 共享的 CAN FD 通道上工作
        """
        print(f"\n[Kinco-{self.node_id}] Initializing...")
        print(f"[Kinco-{self.node_id}] Mode: CAN FD Mixed (Standard CAN frames)")
        
        # 步骤1: NMT 启动
        if not self.start_node():
            print(f"[Kinco-{self.node_id}] Warning: NMT start failed")
        
        # 步骤2: 使能 + 绝对位置模式
        if not self.enable_absolute_mode():
            print(f"[Kinco-{self.node_id}] Error: Enable failed")
            return False
        
        print(f"[Kinco-{self.node_id}] Initialization complete")
        print(f"[Kinco-{self.node_id}] Ready for position control")
        return True
    
    def shutdown(self) -> bool:
        """
        安全关闭驱动器
        
        执行顺序:
        1. 禁用驱动器（松轴）
        2. 发送 NMT 停止节点命令（释放 CANopen 节点）
        
        在安全退出时调用，确保 CAN FD 资源正确释放
        """
        print(f"\n[Kinco-{self.node_id}] Shutting down...")
        
        # 步骤1: 禁用驱动器
        try:
            self.disable()
            time.sleep(0.1)
        except Exception as e:
            print(f"[Kinco-{self.node_id}] Warning during disable: {e}")
        
        # 步骤2: NMT 停止节点（释放 CANopen 节点）
        try:
            self.stop_node()
            time.sleep(0.05)
        except Exception as e:
            print(f"[Kinco-{self.node_id}] Warning during stop node: {e}")
        
        print(f"[Kinco-{self.node_id}] Shutdown complete")
        return True
    
    def print_state(self):
        """打印当前状态"""
        print(f"\n[Kinco-{self.node_id}] State:")
        print(f"  Position: {self.state.position_inc} inc ({self.state.position_inc/self.GEAR_RATIO:.2f} deg)")
        print(f"  Velocity: {self.state.velocity_rpm} rpm")
        print(f"  Current: {self.state.current_ap} Ap")
        print(f"  Enabled: {self.state.is_enabled}")


# ============================================================================
# Test
# ============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("Kinco PDO Driver Test")
    print("=" * 60)
    print("\nThis is a hardware test script.")
    print("Usage:")
    print("  from drivers.kinco_pdo_driver import KincoPDODriver")
    print("  from core import ZlgCanDriver, ZCANDeviceType")
    print("")
    print("  driver = ZlgCanDriver()")
    print("  driver.open(ZCANDeviceType.USBCANFD_MINI)")
    print("  driver.init_canfd(arbitration_bps=1000000)")
    print("")
    print("  motor = KincoPDODriver(driver, node_id=1)")
    print("  motor.initialize()")
    print("  motor.move_to_degree(90.0)")
