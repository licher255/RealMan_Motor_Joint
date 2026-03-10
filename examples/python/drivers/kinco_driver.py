"""
Kinco Rotary Servo Motor Driver

Kinco旋转舵盘电机驱动实现，支持标准CAN通信。

Features:
- NMT节点管理
- 绝对/相对位置控制
- 速度控制
- 实时状态监控

Example:
    from core import ZlgCanDriver, ZCANDeviceType
    from drivers import KincoDriver
    
    can_driver = ZlgCanDriver()
    can_driver.open(ZCANDeviceType.USBCANFD_MINI)
    can_driver.init_canfd()
    
    motor = KincoDriver(can_driver, motor_id=1)
    motor.start_node()
    motor.set_absolute_position_mode()
    motor.set_position(90.0, velocity=50)
    
    can_driver.close()
"""

import sys
import os
# 添加父目录到路径，以便导入 core 模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import time
from typing import Optional

from core import ZlgCanDriver
from core.protocol import (
    KincoProtocol, KincoMode, KincoState,
    KincoNMTCommand
)
from drivers.base_driver import BaseMotorDriver, MotorState


class KincoDriver(BaseMotorDriver):
    """
    Kinco旋转舵盘电机驱动
    
    通过ZLG CAN设备发送标准CAN帧控制电机。
    """
    
    def __init__(self, can_driver: ZlgCanDriver, motor_id: int = 1):
        """
        初始化Kinco驱动
        
        Args:
            can_driver: ZLG CAN驱动实例
            motor_id: 电机节点ID (默认1)
        """
        super().__init__(motor_id, can_driver)
        self._kinco_state = KincoState(motor_id=motor_id)
        self._current_mode: Optional[KincoMode] = None
        self._is_node_active = False
    
    # ========================================================================
    # NMT (Network Management) Commands
    # ========================================================================
    
    def start_node(self) -> bool:
        """
        启动节点 (NMT命令)
        
        Returns:
            True if successful
        """
        data = KincoProtocol.build_start_node(self.motor_id)
        success = self._send_frame(KincoProtocol.NMT_ID, data)
        
        if success:
            self._is_node_active = True
            self._state.is_enabled = True
            print(f"[Kinco-{self.motor_id}] Node started")
            time.sleep(0.1)  # 等待启动完成
        
        return success
    
    def stop_node(self) -> bool:
        """
        停止节点 (NMT命令)
        
        Returns:
            True if successful
        """
        data = KincoProtocol.build_stop_node(self.motor_id)
        success = self._send_frame(KincoProtocol.NMT_ID, data)
        
        if success:
            self._is_node_active = False
            self._state.is_enabled = False
            print(f"[Kinco-{self.motor_id}] Node stopped")
        
        return success
    
    def reset_node(self) -> bool:
        """
        复位节点 (NMT命令)
        
        Returns:
            True if successful
        """
        data = KincoProtocol.build_reset_node(self.motor_id)
        success = self._send_frame(KincoProtocol.NMT_ID, data)
        
        if success:
            print(f"[Kinco-{self.motor_id}] Node reset")
            time.sleep(0.5)  # 等待复位完成
        
        return success
    
    def reset_communication(self) -> bool:
        """
        复位通信 (NMT命令)
        
        Returns:
            True if successful
        """
        data = KincoProtocol.build_nmt_frame(
            self.motor_id, KincoNMTCommand.RESET_COMM
        )
        success = self._send_frame(KincoProtocol.NMT_ID, data)
        
        if success:
            print(f"[Kinco-{self.motor_id}] Communication reset")
            time.sleep(0.3)
        
        return success
    
    # ========================================================================
    # Mode Settings
    # ========================================================================
    
    def set_absolute_position_mode(self) -> bool:
        """
        设置为绝对位置模式 (0x01 0x3F 0x10)
        
        Returns:
            True if successful
        """
        data = KincoProtocol.build_set_absolute_mode()
        success = self._send_frame(KincoProtocol.CONTROL_ID, data)
        
        if success:
            self._current_mode = KincoMode.ABSOLUTE_POSITION
            print(f"[Kinco-{self.motor_id}] Absolute position mode")
            time.sleep(0.05)
        
        return success
    
    def set_relative_position_mode(self) -> bool:
        """
        设置为相对位置模式
        
        Returns:
            True if successful
        """
        data = KincoProtocol.build_set_relative_mode()
        success = self._send_frame(KincoProtocol.CONTROL_ID, data)
        
        if success:
            self._current_mode = KincoMode.RELATIVE_POSITION
            print(f"[Kinco-{self.motor_id}] Relative position mode")
            time.sleep(0.05)
        
        return success
    
    def set_homing_mode(self) -> bool:
        """
        设置为原点设置模式 (模式6)
        
        Returns:
            True if successful
        """
        data = KincoProtocol.build_control_word_frame(
            0x06, KincoProtocol.CONTROL_ENABLE, KincoMode.HOMING_MODE
        )
        success = self._send_frame(KincoProtocol.CONTROL_ID, data)
        
        if success:
            self._current_mode = KincoMode.HOMING_MODE
            print(f"[Kinco-{self.motor_id}] Homing mode")
            time.sleep(0.05)
        
        return success
    
    # ========================================================================
    # Motion Control
    # ========================================================================
    
    def set_position(self, position: float, velocity: Optional[float] = 50.0) -> bool:
        """
        设置目标位置
        
        Args:
            position: 目标位置 (度)
            velocity: 目标速度 (RPM), 默认50
        
        Returns:
            True if successful
        """
        if not self._is_node_active:
            print(f"[Kinco-{self.motor_id}] Warning: Node not active")
        
        data = KincoProtocol.build_position_frame(position, velocity)
        success = self._send_frame(KincoProtocol.POSITION_CTRL_ID, data)
        
        if success:
            self._state.position = position
            self._state.velocity = velocity if velocity else 0
            self._kinco_state.target_position_deg = position
            self._kinco_state.target_speed_rpm = velocity if velocity else 0
            self._kinco_state.is_moving = True
            print(f"[Kinco-{self.motor_id}] Move to {position:.2f}° @ {velocity} RPM")
        
        return success
    
    def move_relative(self, delta: float, velocity: Optional[float] = 50.0) -> bool:
        """
        相对移动 (需要先设置为相对位置模式)
        
        Args:
            delta: 相对移动距离 (度)
            velocity: 目标速度 (RPM)
        
        Returns:
            True if successful
        """
        if self._current_mode != KincoMode.RELATIVE_POSITION:
            print(f"[Kinco-{self.motor_id}] Warning: Not in relative mode")
        
        data = KincoProtocol.build_relative_position_frame(delta, velocity)
        success = self._send_frame(KincoProtocol.POSITION_CTRL_ID, data)
        
        if success:
            print(f"[Kinco-{self.motor_id}] Move relative {delta:+.2f}°")
        
        return success
    
    def set_velocity(self, velocity: float) -> bool:
        """
        设置速度 (在速度模式下使用)
        
        Args:
            velocity: 目标速度 (RPM)
        
        Returns:
            True if successful
        """
        # Kinco的速度控制也是通过位置控制ID发送
        # 这需要电机处于速度模式
        print(f"[Kinco-{self.motor_id}] Speed control not implemented yet")
        return False
    
    def stop(self) -> bool:
        """
        停止运动
        
        Returns:
            True if successful
        """
        # 发送当前位置作为目标来停止
        data = KincoProtocol.build_stop_frame(self._state.position)
        success = self._send_frame(KincoProtocol.POSITION_CTRL_ID, data)
        
        if success:
            self._kinco_state.is_moving = False
            print(f"[Kinco-{self.motor_id}] Motion stopped")
        
        return success
    
    def emergency_stop(self) -> bool:
        """
        紧急停止 (停止节点)
        
        Returns:
            True if successful
        """
        return self.stop_node()
    
    def home(self) -> bool:
        """
        回零位 (转到0度)
        
        Returns:
            True if successful
        """
        return self.set_position(0.0, velocity=30.0)
    
    def set_origin(self) -> bool:
        """
        设置当前位置为原点 (按照操作指南流程)
        
        流程:
        1) 0x201, [06 0F 00 00 00 00 00 00] - 控制模式6, 控制字0F
        2) 0x201, [06 1F 00 00 00 00 00 00] - 控制字1F
        3) 0x201, [01 3F 10 00 00 00 00 00] - 切换回绝对位置模式
        
        Returns:
            True if successful
        """
        print(f"\n[Kinco-{self.motor_id}] Setting origin...")
        
        # Step 1: 控制模式6, 控制字0F
        step1 = KincoProtocol.build_homing_step1()
        if not self._send_frame(KincoProtocol.CONTROL_ID, step1):
            print(f"[Kinco-{self.motor_id}] Homing step 1 failed")
            return False
        print(f"[Kinco-{self.motor_id}] Homing step 1: Mode 6, Control 0x0F")
        time.sleep(0.1)
        
        # Step 2: 控制字1F
        step2 = KincoProtocol.build_homing_step2()
        if not self._send_frame(KincoProtocol.CONTROL_ID, step2):
            print(f"[Kinco-{self.motor_id}] Homing step 2 failed")
            return False
        print(f"[Kinco-{self.motor_id}] Homing step 2: Control 0x1F")
        time.sleep(0.1)
        
        # Step 3: 切换回绝对位置模式
        if not self.set_absolute_position_mode():
            print(f"[Kinco-{self.motor_id}] Homing step 3 failed")
            return False
        print(f"[Kinco-{self.motor_id}] Homing step 3: Back to absolute mode")
        
        print(f"[Kinco-{self.motor_id}] Origin set successfully!")
        return True
    
    # ========================================================================
    # State & Status
    # ========================================================================
    
    def get_state(self, query: bool = True) -> Optional[MotorState]:
        """
        获取电机状态
        
        注意: Kinco的状态主要通过TPDO获取，需要设置好PDO映射。
        如果没有TPDO反馈，则返回缓存值。
        
        Args:
            query: 是否查询硬件 (对Kinco通常无效，依赖TPDO)
        
        Returns:
            MotorState或None
        """
        # 尝试从接收缓冲区获取TPDO数据
        if query and self.can_driver:
            self._process_received_frames()
        
        return self._state
    
    def get_kinco_state(self) -> KincoState:
        """获取Kinco专用状态"""
        return self._kinco_state
    
    def is_moving(self) -> bool:
        """检查是否正在运动"""
        return self._kinco_state.is_moving
    
    def is_node_active(self) -> bool:
        """检查节点是否激活"""
        return self._is_node_active
    
    # ========================================================================
    # Enable/Disable (Mapped to NMT)
    # ========================================================================
    
    def enable(self) -> bool:
        """使能电机 (启动节点)"""
        return self.start_node()
    
    def disable(self) -> bool:
        """禁用电机 (停止节点)"""
        return self.stop_node()
    
    # ========================================================================
    # Sequences
    # ========================================================================
    
    def demo_sequence(self, cycles: int = 1, delay: float = 2.0):
        """
        演示序列: 90度来回运动
        
        Args:
            cycles: 循环次数
            delay: 每次运动后的等待时间
        """
        print(f"\n[Kinco-{self.motor_id}] Demo sequence: {cycles} cycles")
        
        # 确保节点已启动
        if not self._is_node_active:
            self.start_node()
        
        # 确保是绝对位置模式
        if self._current_mode != KincoMode.ABSOLUTE_POSITION:
            self.set_absolute_position_mode()
        
        try:
            for i in range(cycles):
                print(f"\n  Cycle {i+1}/{cycles}")
                
                # 转到90度
                print("    Moving to 90°...")
                self.set_position(90.0, velocity=50.0)
                time.sleep(delay)
                
                # 转回0度
                print("    Moving to 0°...")
                self.set_position(0.0, velocity=50.0)
                time.sleep(delay)
            
            print(f"\n[Kinco-{self.motor_id}] Demo completed")
            
        except KeyboardInterrupt:
            print(f"\n[Kinco-{self.motor_id}] Demo interrupted")
            self.stop()
    
    def initialize(self) -> bool:
        """
        完整初始化流程 (按照操作指南)
        
        流程:
        1) 0x000, [01 00] - 启动节点
        2) 0x201, [01 3F 10 00 00 00 00 00] - 上使能, 绝对位置模式
        
        Returns:
            True if successful
        """
        print(f"\n[Kinco-{self.motor_id}] Initializing...")
        
        # 步骤1: 启动节点
        if not self.start_node():
            return False
        
        # 步骤2: 上使能, 设置为绝对位置模式
        if not self.set_absolute_position_mode():
            return False
        
        print(f"[Kinco-{self.motor_id}] Initialization complete")
        print(f"[Kinco-{self.motor_id}] Ready for position control (0x301)")
        return True
    
    # ========================================================================
    # Internal Methods
    # ========================================================================
    
    def _send_frame(self, can_id: int, data: bytes) -> bool:
        """
        发送CAN帧
        
        Args:
            can_id: CAN ID
            data: 帧数据
        
        Returns:
            True if successful
        """
        if not self.can_driver:
            return False
        
        return self.can_driver.send(
            can_id=can_id,
            data=data,
            is_extended=False
        )
    
    def _process_received_frames(self):
        """处理接收到的帧 (TPDO)"""
        # 检查所有接收到的帧
        max_frames = 10
        for _ in range(max_frames):
            frame = self.can_driver.receive_frame(timeout_ms=0)
            if frame is None:
                break
            
            # 检查是否是本电机的TPDO
            expected_tpdo1 = KincoProtocol.TPDO1_BASE + self.motor_id
            expected_tpdo2 = KincoProtocol.TPDO2_BASE + self.motor_id
            
            if frame.can_id in [expected_tpdo1, expected_tpdo2]:
                self._parse_tpdo(frame.can_id, frame.data)
    
    def _parse_tpdo(self, can_id: int, data: bytes):
        """解析TPDO帧"""
        # 解析位置和速度
        parsed = KincoProtocol.parse_tpdo1_frame(data)
        if parsed:
            self._state.position = parsed['position_deg']
            self._state.velocity = parsed['speed_rpm']
            self._kinco_state.position_deg = parsed['position_deg']
            self._kinco_state.speed_rpm = parsed['speed_rpm']
            self._state.timestamp = time.time()
    
    def __repr__(self):
        return f"KincoDriver(id={self.motor_id}, mode={self._current_mode})"
