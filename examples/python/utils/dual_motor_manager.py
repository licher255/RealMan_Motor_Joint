"""
Dual Motor Manager

WHJ60 + Kinco 双电机管理器

提供统一的双电机控制接口，解决CAN总线上的通信冲突。

Example:
    from utils import DualMotorManager
    
    with DualMotorManager() as manager:
        manager.init_device()
        manager.add_whj(motor_id=7)
        manager.add_kinco(motor_id=1)
        manager.start_all()
        
        # 同步控制
        manager.sync_move(whj_pos=45.0, kinco_pos=90.0)
"""

import time
import threading
from typing import Optional, Dict, List, Callable
from dataclasses import dataclass

from core import ZlgCanDriver, ZCANDeviceType
from drivers import SmoothMotorController, KincoDriver, MotorState


@dataclass
class DualMotorState:
    """双电机状态"""
    whj_states: Dict[int, MotorState]
    kinco_states: Dict[int, MotorState]
    timestamp: float


class DualMotorManager:
    """
    双电机管理器
    
    管理WHJ60和Kinco电机的同步通信和控制。
    
    Features:
    - 统一初始化
    - 同步/异步控制
    - 状态监控
    - 通信冲突解决
    """
    
    def __init__(self, dll_path: Optional[str] = None):
        """
        初始化管理器
        
        Args:
            dll_path: zlgcan.dll路径, None则自动检测
        """
        self.dll_path = dll_path
        self.can_driver: Optional[ZlgCanDriver] = None
        self.is_initialized = False
        
        # 电机实例
        self.whj_motors: Dict[int, SmoothMotorController] = {}
        self.kinco_motors: Dict[int, KincoDriver] = {}
        

        
        # 统计
        self._stats = {
            'commands_sent': 0,
            'queries_sent': 0,
            'responses_received': 0,
            'errors': 0,
        }
    
    def init_device(self, device_type=ZCANDeviceType.USBCANFD_MINI,
                    channel: int = 0,
                    arbitration_bps: int = 1000000,
                    data_bps: int = 5000000) -> bool:
        """
        初始化CAN设备
        
        Args:
            device_type: ZLG设备类型
            channel: CAN通道
            arbitration_bps: 仲裁段波特率
            data_bps: 数据段波特率
        
        Returns:
            True if successful
        """
        try:
            self.can_driver = ZlgCanDriver(self.dll_path)
            self.can_driver.open(device_type, device_index=0, channel=channel)
            self.can_driver.init_canfd(arbitration_bps, data_bps)
            self.is_initialized = True
            
            print(f"[DualMotor] Device initialized: {device_type.name}")
            return True
            
        except Exception as e:
            print(f"[DualMotor] Failed to initialize: {e}")
            return False
    
    def add_whj(self, motor_id: int) -> SmoothMotorController:
        """
        添加WHJ电机
        
        Args:
            motor_id: 电机ID
        
        Returns:
            SmoothMotorController实例
        """
        if not self.is_initialized:
            raise RuntimeError("Device not initialized")
        
        from drivers import MotionProfile
        profile = MotionProfile(
            max_velocity=1800.0,
            max_acceleration=720.0,
            max_deceleration=720.0
        )
        driver = SmoothMotorController(self.can_driver, motor_id, profile)
        self.whj_motors[motor_id] = driver
        
        print(f"[DualMotor] Added WHJ motor ID={motor_id}")
        return driver
    
    def add_kinco(self, motor_id: int) -> KincoDriver:
        """
        添加Kinco电机
        
        Args:
            motor_id: 电机ID
        
        Returns:
            KincoDriver实例
        """
        if not self.is_initialized:
            raise RuntimeError("Device not initialized")
        
        driver = KincoDriver(self.can_driver, motor_id)
        self.kinco_motors[motor_id] = driver
        
        print(f"[DualMotor] Added Kinco motor ID={motor_id}")
        return driver
    
    def start_all(self):
        """启动所有电机"""
        print("\n[DualMotor] Starting all motors...")
        
        # 启动Kinco电机
        for motor_id, motor in self.kinco_motors.items():
            print(f"  Starting Kinco-{motor_id}...")
            motor.initialize()
            time.sleep(0.2)
        
        # 启动WHJ电机
        for motor_id, motor in self.whj_motors.items():
            print(f"  Starting WHJ-{motor_id}...")
            motor.enable()
            motor.set_work_mode(motor.work_mode)
            time.sleep(0.1)
        
        print("[DualMotor] All motors started")
    
    def stop_all(self):
        """停止所有电机"""
        print("\n[DualMotor] Stopping all motors...")
        
        for motor in self.kinco_motors.values():
            motor.emergency_stop()
        
        for motor in self.whj_motors.values():
            motor.emergency_stop()
        
        print("[DualMotor] All motors stopped")
    
    def sync_move(self, whj_id: Optional[int] = None, whj_pos: Optional[float] = None,
                  kinco_id: Optional[int] = None, kinco_pos: Optional[float] = None,
                  whj_vel: Optional[float] = None, kinco_vel: Optional[float] = None):
        """
        同步移动电机
        
        Args:
            whj_id: WHJ电机ID (None则使用第一个)
            whj_pos: WHJ目标位置
            kinco_id: Kinco电机ID (None则使用第一个)
            kinco_pos: Kinco目标位置
            whj_vel: WHJ速度
            kinco_vel: Kinco速度
        """
        # 使用默认ID
        if whj_id is None and self.whj_motors:
            whj_id = list(self.whj_motors.keys())[0]
        if kinco_id is None and self.kinco_motors:
            kinco_id = list(self.kinco_motors.keys())[0]
        
        print(f"\n[DualMotor] Sync move:")
        
        # 发送WHJ命令
        if whj_id is not None and whj_pos is not None:
            whj = self.whj_motors.get(whj_id)
            if whj:
                print(f"  WHJ-{whj_id} -> {whj_pos:.2f}°")
                whj.set_position(whj_pos, velocity=whj_vel)
                self._stats['commands_sent'] += 1
        
        # 发送Kinco命令
        if kinco_id is not None and kinco_pos is not None:
            kinco = self.kinco_motors.get(kinco_id)
            if kinco:
                print(f"  Kinco-{kinco_id} -> {kinco_pos:.2f}°")
                velocity = kinco_vel if kinco_vel is not None else 50.0
                kinco.set_position(kinco_pos, velocity=velocity)
                self._stats['commands_sent'] += 1
    
    def sync_sequence(self, sequence: List[Dict], delay: float = 2.0):
        """
        执行同步运动序列
        
        Args:
            sequence: 运动序列
                [
                    {'whj': 45.0, 'kinco': 90.0},
                    {'whj': 0.0, 'kinco': 0.0},
                ]
            delay: 每步等待时间
        """
        whj_id = list(self.whj_motors.keys())[0] if self.whj_motors else None
        kinco_id = list(self.kinco_motors.keys())[0] if self.kinco_motors else None
        
        print(f"\n[DualMotor] Starting sequence ({len(sequence)} points)")
        
        for i, point in enumerate(sequence):
            whj_pos = point.get('whj')
            kinco_pos = point.get('kinco')
            
            print(f"\n  Step {i+1}/{len(sequence)}")
            self.sync_move(whj_id, whj_pos, kinco_id, kinco_pos)
            time.sleep(delay)
        
        print("\n[DualMotor] Sequence completed")
    
    def get_whj_state(self, motor_id: Optional[int] = None) -> Optional[MotorState]:
        """
        获取WHJ状态
        
        Args:
            motor_id: 电机ID (None则使用第一个)
        
        Returns:
            MotorState或None
        """
        if motor_id is None:
            motor_id = list(self.whj_motors.keys())[0] if self.whj_motors else None
        
        motor = self.whj_motors.get(motor_id)
        if motor:
            return motor.get_state(query=True)
        return None
    
    def get_kinco_state(self, motor_id: Optional[int] = None) -> Optional[MotorState]:
        """
        获取Kinco状态
        
        Args:
            motor_id: 电机ID (None则使用第一个)
        
        Returns:
            MotorState或None
        """
        if motor_id is None:
            motor_id = list(self.kinco_motors.keys())[0] if self.kinco_motors else None
        
        motor = self.kinco_motors.get(motor_id)
        if motor:
            return motor.get_state(query=False)  # Kinco依赖TPDO
        return None
    
    def update(self, timeout_ms: int = 10):
        """
        更新状态 - 处理接收缓冲区
        
        Args:
            timeout_ms: 接收超时
        """
        if not self.can_driver:
            return
        
        # 处理所有可用帧
        max_frames = 50
        for _ in range(max_frames):
            frame = self.can_driver.receive_frame(timeout_ms=0)
            if frame is None:
                break
            
            # 分发给相应的驱动
            self._dispatch_frame(frame)
    
    def _dispatch_frame(self, frame):
        """分发接收到的帧"""
        can_id = frame.can_id
        
        # 检查是否是WHJ响应 (ID = motor_id + 0x100)
        for motor_id, motor in self.whj_motors.items():
            if can_id == motor_id + 0x100:
                # 让WHJ驱动处理
                motor.get_state(query=False)
                self._stats['responses_received'] += 1
                return
        
        # 检查是否是Kinco TPDO
        for motor_id, motor in self.kinco_motors.items():
            if can_id in [0x180 + motor_id, 0x280 + motor_id]:
                motor.get_state(query=False)
                return
    

    
    def print_status(self):
        """打印状态"""
        print("\n" + "=" * 60)
        print("Dual Motor System Status")
        print("=" * 60)
        
        # WHJ状态
        print("\nWHJ Motors:")
        for motor_id, motor in self.whj_motors.items():
            state = motor.get_state(query=False)
            if state:
                print(f"  ID={motor_id}: {state.position:7.2f}° | "
                      f"{state.velocity:6.2f} RPM | "
                      f"{state.current:5.0f} mA")
            else:
                print(f"  ID={motor_id}: No data")
        
        # Kinco状态
        print("\nKinco Motors:")
        for motor_id, motor in self.kinco_motors.items():
            state = motor.get_state(query=False)
            if state:
                print(f"  ID={motor_id}: {state.position:7.2f}° | "
                      f"Target={motor._kinco_state.target_position_deg:.2f}°")
            else:
                print(f"  ID={motor_id}: No data")
        
        # 统计
        print("\nStatistics:")
        for key, value in self._stats.items():
            print(f"  {key}: {value}")
        
        print("=" * 60)
    
    def close(self):
        """关闭管理器"""
        print("\n[DualMotor] Closing...")
        
        self.stop_all()
        
        if self.can_driver:
            self.can_driver.close()
            self.can_driver = None
        
        self.is_initialized = False
        print("[DualMotor] Closed")
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False
