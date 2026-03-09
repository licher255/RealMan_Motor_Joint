"""
Base Motor Driver

所有电机驱动的抽象基类，定义统一的接口。
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Callable, Any
import time


@dataclass
class MotorState:
    """通用电机状态"""
    motor_id: int
    position: float = 0.0        # 位置 (度)
    velocity: float = 0.0        # 速度 (RPM)
    current: float = 0.0         # 电流 (mA)
    is_enabled: bool = False     # 是否使能
    is_moving: bool = False      # 是否正在运动
    error_code: int = 0          # 错误码
    timestamp: float = 0.0       # 时间戳
    
    def to_dict(self) -> dict:
        """转换为字典"""
        return {
            'motor_id': self.motor_id,
            'position': self.position,
            'velocity': self.velocity,
            'current': self.current,
            'is_enabled': self.is_enabled,
            'is_moving': self.is_moving,
            'error_code': self.error_code,
            'timestamp': self.timestamp,
        }


class BaseMotorDriver(ABC):
    """
    电机驱动抽象基类
    
    所有具体电机驱动都应继承此类并实现抽象方法。
    
    Example:
        class MyMotorDriver(BaseMotorDriver):
            def enable(self):
                # 实现使能逻辑
                pass
            
            def disable(self):
                # 实现禁用逻辑
                pass
            
            def set_position(self, position, **kwargs):
                # 实现位置控制
                pass
            
            def get_state(self):
                # 实现状态查询
                pass
    """
    
    def __init__(self, motor_id: int, can_driver=None):
        """
        初始化驱动
        
        Args:
            motor_id: 电机CAN ID
            can_driver: CAN驱动实例
        """
        self.motor_id = motor_id
        self.can_driver = can_driver
        self.is_initialized = False
        self._state = MotorState(motor_id=motor_id)
        self._state_callbacks: list[Callable] = []
        self._error_callbacks: list[Callable] = []
    
    @abstractmethod
    def enable(self) -> bool:
        """
        使能电机
        
        Returns:
            True if successful
        """
        pass
    
    @abstractmethod
    def disable(self) -> bool:
        """
        禁用电机
        
        Returns:
            True if successful
        """
        pass
    
    @abstractmethod
    def set_position(self, position: float, **kwargs) -> bool:
        """
        设置目标位置
        
        Args:
            position: 目标位置 (度)
            **kwargs: 额外参数 (如速度、加速度等)
        
        Returns:
            True if successful
        """
        pass
    
    @abstractmethod
    def get_state(self, query: bool = True) -> Optional[MotorState]:
        """
        获取电机状态
        
        Args:
            query: 是否查询硬件 (False则返回缓存值)
        
        Returns:
            MotorState或None
        """
        pass
    
    def stop(self) -> bool:
        """
        停止电机 (默认实现为禁用)
        
        Returns:
            True if successful
        """
        return self.disable()
    
    def reset(self) -> bool:
        """
        复位电机 (可选实现)
        
        Returns:
            True if successful
        """
        return True
    
    def set_velocity(self, velocity: float) -> bool:
        """
        设置速度 (可选实现)
        
        Args:
            velocity: 目标速度 (RPM)
        
        Returns:
            True if successful
        """
        raise NotImplementedError("Velocity control not supported")
    
    def set_current(self, current: float) -> bool:
        """
        设置电流 (可选实现)
        
        Args:
            current: 目标电流 (mA)
        
        Returns:
            True if successful
        """
        raise NotImplementedError("Current control not supported")
    
    def home(self) -> bool:
        """
        回零位 (可选实现)
        
        Returns:
            True if successful
        """
        return self.set_position(0.0)
    
    def wait_for_position(self, target_position: float, 
                          tolerance: float = 1.0,
                          timeout: float = 10.0,
                          poll_interval: float = 0.05) -> bool:
        """
        等待到达目标位置
        
        Args:
            target_position: 目标位置 (度)
            tolerance: 误差容忍范围 (度)
            timeout: 超时时间 (秒)
            poll_interval: 轮询间隔 (秒)
        
        Returns:
            True if position reached
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            state = self.get_state(query=True)
            if state is None:
                time.sleep(poll_interval)
                continue
            
            error = abs(state.position - target_position)
            if error <= tolerance:
                return True
            
            time.sleep(poll_interval)
        
        return False
    
    def register_state_callback(self, callback: Callable[[MotorState], None]):
        """注册状态更新回调"""
        self._state_callbacks.append(callback)
    
    def unregister_state_callback(self, callback: Callable[[MotorState], None]):
        """注销状态更新回调"""
        if callback in self._state_callbacks:
            self._state_callbacks.remove(callback)
    
    def register_error_callback(self, callback: Callable[[int, str], None]):
        """注册错误回调"""
        self._error_callbacks.append(callback)
    
    def _notify_state_update(self):
        """通知状态更新"""
        for callback in self._state_callbacks:
            try:
                callback(self._state)
            except Exception as e:
                print(f"State callback error: {e}")
    
    def _notify_error(self, error_code: int, error_msg: str):
        """通知错误"""
        for callback in self._error_callbacks:
            try:
                callback(error_code, error_msg)
            except Exception as e:
                print(f"Error callback error: {e}")
    
    @property
    def state(self) -> MotorState:
        """获取当前状态缓存"""
        return self._state
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disable()
        return False
