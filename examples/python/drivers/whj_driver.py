"""
RealMan WHJ Joint Motor Driver

WHJ关节电机驱动实现，支持平滑轨迹规划。

基于 motion_controller.py 中的 SmoothMotorController 实现，
使用梯形速度规划来避免电机过热。

Features:
- 位置/速度/电流控制
- 平滑轨迹规划 (梯形速度曲线)
- 实时状态查询
- 错误处理
- 多电机支持

Example:
    from core import ZlgCanDriver, ZCANDeviceType
    from drivers import WHJDriver
    
    can_driver = ZlgCanDriver()
    can_driver.open(ZCANDeviceType.USBCANFD_MINI)
    can_driver.init_canfd()
    
    motor = WHJDriver(can_driver, motor_id=7)
    motor.initialize()
    motor.enable()
    motor.move_to_position(90.0)  # 平滑移动到90度
    
    can_driver.close()
"""

import time
import math
from typing import Optional, List
from enum import Enum
from dataclasses import dataclass

from core import ZlgCanDriver
from core.protocol import (
    WHJProtocol, Register, WorkMode,
    JointState, ErrorCode
)
from .base_driver import BaseMotorDriver, MotorState


class TrajectoryState(Enum):
    """轨迹状态"""
    IDLE = "idle"
    ACCELERATING = "accelerating"
    CONSTANT_VELOCITY = "constant_velocity"
    DECELERATING = "decelerating"
    FINISHED = "finished"


@dataclass
class MotionProfile:
    """运动规划参数"""
    max_velocity: float = 14400.0      # degrees/s
    max_acceleration: float = 72000.0  # degrees/s^2
    max_deceleration: float = 72000.0  # degrees/s^2
    jerk: float = 0.0                  # Jerk for S-curve (0 = trapezoidal)


class TrapezoidalPlanner:
    """
    梯形速度轨迹规划器
    
    生成平滑的位置轨迹，限制速度和加速度。
    """
    
    def __init__(self, profile: MotionProfile):
        self.profile = profile
        self.reset()
    
    def reset(self):
        self.state = TrajectoryState.IDLE
        self.start_pos = 0.0
        self.target_pos = 0.0
        self.current_pos = 0.0
        self.current_vel = 0.0
        self.direction = 1.0
        
        self.accel_distance = 0.0
        self.decel_distance = 0.0
        self.const_vel_distance = 0.0
        self.total_distance = 0.0
        
        self.start_time = 0.0
        self.phase_start_time = 0.0
        self.accel_time = 0.0
        self.const_vel_time = 0.0
        self.decel_time = 0.0
        self.total_time = 0.0
    
    def plan(self, start_pos: float, target_pos: float):
        """规划梯形轨迹"""
        self.reset()
        
        self.start_pos = start_pos
        self.target_pos = target_pos
        self.current_pos = start_pos
        
        self.total_distance = abs(target_pos - start_pos)
        self.direction = 1.0 if target_pos > start_pos else -1.0
        
        if self.total_distance < 0.001:
            self.state = TrajectoryState.FINISHED
            return
        
        v_max = self.profile.max_velocity
        a_max = self.profile.max_acceleration
        d_max = self.profile.max_deceleration
        
        # 计算加速到最大速度所需时间和距离
        t_accel = v_max / a_max
        d_accel = 0.5 * a_max * t_accel * t_accel
        
        t_decel = v_max / d_max
        d_decel = 0.5 * d_max * t_decel * t_decel
        
        # 判断是否能达到最大速度 (三角形 vs 梯形)
        if d_accel + d_decel >= self.total_distance:
            # 三角形轨迹 - 无法达到最大速度
            v_peak = math.sqrt(2 * self.total_distance / (1/a_max + 1/d_max))
            self.accel_time = v_peak / a_max
            self.decel_time = v_peak / d_max
            self.const_vel_time = 0.0
            self.accel_distance = 0.5 * a_max * self.accel_time ** 2
            self.decel_distance = 0.5 * d_max * self.decel_time ** 2
            self.const_vel_distance = 0.0
        else:
            # 梯形轨迹 - 有匀速阶段
            self.accel_time = t_accel
            self.decel_time = t_decel
            self.const_vel_distance = self.total_distance - d_accel - d_decel
            self.const_vel_time = self.const_vel_distance / v_max
            self.accel_distance = d_accel
            self.decel_distance = d_decel
        
        self.total_time = self.accel_time + self.const_vel_time + self.decel_time
        self.state = TrajectoryState.ACCELERATING
        self.start_time = time.time()
        self.phase_start_time = self.start_time
        
        print(f"[Trajectory] Planned: {start_pos:.2f}° -> {target_pos:.2f}°")
        print(f"             Distance: {self.total_distance:.2f}°")
        print(f"             Accel: {self.accel_time:.3f}s, Const: {self.const_vel_time:.3f}s, Decel: {self.decel_time:.3f}s")
        print(f"             Total time: {self.total_time:.3f}s")
    
    def update(self, dt: Optional[float] = None) -> tuple:
        """
        更新轨迹并获取下一个位置设定点
        
        Returns:
            (position, velocity, finished)
        """
        if self.state == TrajectoryState.IDLE:
            return self.current_pos, 0.0, False
        
        if self.state == TrajectoryState.FINISHED:
            return self.target_pos, 0.0, True
        
        now = time.time()
        elapsed = now - self.start_time
        phase_elapsed = now - self.phase_start_time
        
        if self.state == TrajectoryState.ACCELERATING:
            if phase_elapsed >= self.accel_time:
                if self.const_vel_time > 0.001:
                    self.state = TrajectoryState.CONSTANT_VELOCITY
                    self.phase_start_time = now
                    self.current_vel = self.profile.max_velocity
                    self.current_pos = self.start_pos + self.direction * self.accel_distance
                else:
                    self.state = TrajectoryState.DECELERATING
                    self.phase_start_time = now
                    self.current_vel = self.profile.max_velocity
                    self.current_pos = self.start_pos + self.direction * self.accel_distance
            else:
                t = phase_elapsed
                self.current_vel = self.profile.max_acceleration * t
                self.current_pos = self.start_pos + self.direction * (0.5 * self.profile.max_acceleration * t * t)
        
        elif self.state == TrajectoryState.CONSTANT_VELOCITY:
            if phase_elapsed >= self.const_vel_time:
                self.state = TrajectoryState.DECELERATING
                self.phase_start_time = now
                self.current_pos = self.start_pos + self.direction * (self.accel_distance + self.const_vel_distance)
            else:
                t = phase_elapsed
                self.current_vel = self.profile.max_velocity
                self.current_pos = self.start_pos + self.direction * (self.accel_distance + self.current_vel * t)
        
        elif self.state == TrajectoryState.DECELERATING:
            if phase_elapsed >= self.decel_time:
                self.state = TrajectoryState.FINISHED
                self.current_pos = self.target_pos
                self.current_vel = 0.0
                print(f"[Trajectory] Finished at {self.target_pos:.2f}°")
            else:
                t = phase_elapsed
                v_start = self.profile.max_velocity
                self.current_vel = max(0, v_start - self.profile.max_deceleration * t)
                d_decel_now = v_start * t - 0.5 * self.profile.max_deceleration * t * t
                self.current_pos = (self.start_pos + 
                                   self.direction * (self.accel_distance + self.const_vel_distance + d_decel_now))
        
        finished = (self.state == TrajectoryState.FINISHED)
        return self.current_pos, self.current_vel, finished
    
    def is_finished(self) -> bool:
        return self.state == TrajectoryState.FINISHED


class WHJDriver(BaseMotorDriver):
    """
    WHJ关节电机驱动 (带平滑轨迹规划)
    
    适用于WHJ03, WHJ10, WHJ30, WHJ60, WHJ120等型号。
    
    使用梯形速度规划来平滑运动，防止电机过热。
    """
    
    def __init__(self, can_driver: ZlgCanDriver, motor_id: int,
                 profile: Optional[MotionProfile] = None):
        """
        初始化WHJ驱动
        
        Args:
            can_driver: ZLG CAN驱动实例
            motor_id: 电机ID (1-30)
            profile: 运动规划参数，None使用默认保守参数
        """
        super().__init__(motor_id, can_driver)
        
        # 默认使用保守的运动参数
        self.profile = profile or MotionProfile(
            max_velocity=720.0,      # 180°/s (3 RPM) - 保守
            max_acceleration=360.0,   # 360°/s^2
            max_deceleration=360.0
        )
        
        self.planner = TrapezoidalPlanner(self.profile)
        self.running = False
        self.update_interval = 0.002  # 500Hz更新率
        
        self._response_timeout = 0.5
        self._last_response_time = 0
        self._error_count = 0
        self._joint_state: Optional[JointState] = None
        
        # 响应ID
        self.response_id = motor_id + 0x100
    
    def send_command(self, data: bytes, timeout_ms: int = 500) -> tuple:
        """
        发送命令并等待响应
        
        Returns:
            (response_data, error)
        """
        # 清空缓冲区
        self.can_driver.clear_buffer()
        
        # 发送
        if not self.can_driver.send(can_id=self.motor_id, data=data):
            return None, "Send failed"
        
        # 等待响应
        start = time.time()
        while (time.time() - start) * 1000 < timeout_ms:
            frame = self.can_driver.receive(timeout_ms=50)
            if frame and frame.can_id == self.response_id:
                return frame.data, None
            time.sleep(0.001)
        
        return None, "Timeout"
    
    def initialize(self) -> bool:
        """
        初始化电机通信
        
        Returns:
            True if successful
        """
        print(f"[WHJ-{self.motor_id}] Initializing...")
        
        # Ping电机
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.SYS_FW_VERSION, 1)
        resp, err = self.send_command(cmd, timeout_ms=500)
        
        if resp:
            print(f"[WHJ-{self.motor_id}] Motor is online!")
            self.is_initialized = True
            return True
        else:
            print(f"[WHJ-{self.motor_id}] Ping failed: {err}")
            return False
    
    def enable(self) -> bool:
        """使能电机"""
        cmd = WHJProtocol.build_enable_motor(self.motor_id, True)
        resp, err = self.send_command(cmd)
        
        if resp:
            self._state.is_enabled = True
            print(f"[WHJ-{self.motor_id}] Enabled")
            return True
        else:
            print(f"[WHJ-{self.motor_id}] Failed to enable: {err}")
            return False
    
    def disable(self) -> bool:
        """禁用电机"""
        self.stop()
        cmd = WHJProtocol.build_enable_motor(self.motor_id, False)
        resp, err = self.send_command(cmd)
        
        if resp:
            self._state.is_enabled = False
            print(f"[WHJ-{self.motor_id}] Disabled")
            return True
        return False
    
    def get_position(self) -> Optional[float]:
        """获取当前位置"""
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.CUR_POSITION_L, 2)
        resp, err = self.send_command(cmd)
        
        if not resp or len(resp) < 6:
            return None
        
        low = resp[2] | (resp[3] << 8)
        high = resp[4] | (resp[5] << 8)
        
        # 转换为有符号32位
        val = (high << 16) | low
        if val & 0x80000000:
            val -= 0x100000000
        
        return val * 0.0001
    
    def set_target_position(self, position: float) -> bool:
        """设置目标位置 (直接发送)"""
        cmds = WHJProtocol.build_set_target_position(self.motor_id, position)
        for cmd in cmds:
            if not self.can_driver.send(can_id=self.motor_id, data=cmd):
                return False
            time.sleep(0.01)
        return True
    
    def set_position(self, position: float, **kwargs) -> bool:
        """
        设置目标位置 (带平滑轨迹规划)
        
        Args:
            position: 目标位置 (度)
            **kwargs: 可选参数
                - wait: 是否等待完成 (默认True)
                - timeout: 超时时间 (秒)
        
        Returns:
            True if successful
        """
        wait = kwargs.get('wait', True)
        timeout = kwargs.get('timeout', None)
        
        # 获取当前位置
        current_pos = self.get_position()
        if current_pos is None:
            print(f"[WHJ-{self.motor_id}] Error: Failed to get current position")
            return False
        
        # 检查是否已经在目标位置
        distance = abs(position - current_pos)
        if distance < 0.1:
            print(f"[WHJ-{self.motor_id}] Already at target position {position:.2f}°")
            return True
        
        # 自动计算超时时间
        if timeout is None:
            v_max = self.profile.max_velocity
            a_max = self.profile.max_acceleration
            
            t_accel = v_max / a_max
            d_accel = 0.5 * a_max * t_accel * t_accel
            
            if 2 * d_accel >= distance:
                t_total = 2 * math.sqrt(distance / a_max)
            else:
                t_const = (distance - 2 * d_accel) / v_max
                t_total = 2 * t_accel + t_const
            
            timeout = max(t_total * 1.5, 5.0)
            print(f"[WHJ-{self.motor_id}] Auto timeout: {timeout:.1f}s for {distance:.1f}° move")
        
        # 规划轨迹
        self.planner.plan(current_pos, position)
        
        # 确保电机使能并处于位置模式
        if not self._state.is_enabled:
            print(f"[WHJ-{self.motor_id}] Enabling motor...")
            if not self.enable():
                return False
            time.sleep(0.1)
        
        # 执行轨迹
        print(f"[WHJ-{self.motor_id}] Executing trajectory...")
        self.running = True
        
        start_time = time.time()
        last_update = start_time
        
        try:
            while self.running:
                now = time.time()
                
                # 检查超时
                if now - start_time > timeout:
                    print(f"[WHJ-{self.motor_id}] Timeout after {timeout:.1f}s")
                    return False
                
                # 固定频率更新轨迹
                if now - last_update >= self.update_interval:
                    pos, vel, finished = self.planner.update()
                    
                    # 发送位置命令
                    if not self.set_target_position(pos):
                        print(f"[WHJ-{self.motor_id}] Error: Failed to send position command")
                        return False
                    
                    # 每0.5秒打印进度
                    elapsed = now - start_time
                    if int(elapsed * 2) != int((elapsed - self.update_interval) * 2):
                        actual_pos = self.get_position()
                        if actual_pos is not None:
                            print(f"  Target: {pos:7.2f}° | Actual: {actual_pos:7.2f}° | Vel: {vel:6.2f}°/s")
                    
                    last_update = now
                    
                    if finished:
                        break
                
                time.sleep(0.001)
            
            # 最终位置设定
            self.set_target_position(position)
            time.sleep(0.1)
            
            final_pos = self.get_position()
            if final_pos is not None:
                error = abs(final_pos - position)
                print(f"[WHJ-{self.motor_id}] Reached {final_pos:.2f}° (error: {error:.3f}°)")
                self._state.position = final_pos
                return error < 1.0
            
            return True
            
        except KeyboardInterrupt:
            print(f"\n[WHJ-{self.motor_id}] Interrupted by user")
            current = self.get_position()
            if current is not None:
                self.set_target_position(current)
            return False
        finally:
            self.running = False
    
    def stop(self) -> bool:
        """停止当前运动"""
        self.running = False
        self.planner.reset()
        current = self.get_position()
        if current is not None:
            return self.set_target_position(current)
        return True
    
    def get_state(self, query: bool = True) -> Optional[MotorState]:
        """获取电机状态"""
        if not query:
            return self._state
        
        pos = self.get_position()
        if pos is not None:
            self._state.position = pos
            self._state.timestamp = time.time()
            return self._state
        return None
    
    def get_error_status(self):
        """获取错误状态"""
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.SYS_ERROR, 1)
        resp, err = self.send_command(cmd)
        
        if not resp or len(resp) < 4:
            return None
        
        error_code = resp[2] | (resp[3] << 8)
        errors = ErrorCode.parse(error_code)
        return error_code, errors
    
    def clear_error(self) -> bool:
        """清除错误"""
        cmd = WHJProtocol.build_clear_error(self.motor_id)
        resp, err = self.send_command(cmd)
        return resp is not None
    
    def set_work_mode(self, mode: WorkMode) -> bool:
        """设置工作模式"""
        cmd = WHJProtocol.build_set_work_mode(self.motor_id, mode)
        resp, err = self.send_command(cmd)
        return resp is not None
    
    def is_enabled(self) -> Optional[bool]:
        """检查是否使能"""
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.SYS_ENABLE_DRIVER, 1)
        resp, err = self.send_command(cmd)
        
        if not resp or len(resp) < 4:
            return None
        
        return resp[2] | (resp[3] << 8) == 1
    
    def ping(self) -> bool:
        """检查电机是否在线"""
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.SYS_FW_VERSION, 1)
        resp, err = self.send_command(cmd, timeout_ms=500)
        return resp is not None
    
    def __repr__(self):
        return f"WHJDriver(id={self.motor_id}, profile={self.profile.max_velocity}°/s)"
