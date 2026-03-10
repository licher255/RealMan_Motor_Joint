"""
================================================================================
RealMan WHJ Motion Controller - ORIGINAL VERSION (原始版本)
================================================================================

文件名: motion_controller.py
版本: 原始版本（标准轨迹控制）
用途: WHJ60 电机梯形轨迹规划控制

适用场景:
- 单电机或多电机控制（无干扰环境）
- 需要平滑的梯形速度轨迹规划
- CAN 总线上无大量干扰帧

功能:
- 梯形速度轨迹规划（加速-匀速-减速）
- 实时位置反馈
- 500Hz 位置指令更新率
- 自动超时计算

依赖:
- core/zlgcan_driver.py
- core/protocol/whj_protocol.py
- drivers/motor_control.py

运行方式:
  cd examples/python
  python drivers/motion_controller.py <motor_id>
  
  或直接运行（从drivers目录）:
  cd examples/python/drivers
  python motion_controller.py <motor_id>

使用方法:
    python motion_controller.py <motor_id>

命令:
    m <pos>  - 移动到指定位置（带轨迹规划）
    r        - 读取当前位置
    e        - 使能电机
    d        - 禁用电机
    c        - 清除错误
    s        - 显示状态
    q        - 退出

日期: 2026-03-09
================================================================================
"""

import sys
import time
import math
import atexit
from typing import Optional, Callable
import sys
import os

# 添加项目根目录到路径（支持从drivers目录或parent目录运行）
# 获取当前文件所在目录，然后找到项目根目录
_current_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.join(_current_dir, '..')
sys.path.insert(0, os.path.abspath(_project_root))

from dataclasses import dataclass
from enum import Enum

from core import ZlgCanDriver, ZCANDeviceType
from core.protocol import WHJProtocol, Register, WorkMode
from drivers.whj_motor_control import WHJMotorController, parse_32bit_value


# Global variables for cleanup
global_driver = None
global_motor = None

def cleanup_resources():
    """Ensure CAN device is properly closed on exit"""
    global global_motor, global_driver
    
    print("\n[Cleanup] Cleaning up resources...")
    
    if global_motor:
        try:
            print("[Cleanup] Stopping motor...")
            global_motor.stop()
            global_motor.enable(False)
        except:
            pass
    
    if global_driver:
        try:
            print("[Cleanup] Closing CAN device...")
            global_driver.close()
            print("[Cleanup] CAN device closed")
        except Exception as e:
            print(f"[Cleanup] Error closing device: {e}")

# Register cleanup function
atexit.register(cleanup_resources)


@dataclass
class MotionProfile:
    """Motion profile parameters"""
    max_velocity: float = 14400.0      # degrees/s
    max_acceleration: float = 72000.0  # degrees/s^2
    max_deceleration: float = 72000.0  # degrees/s^2
    
    # Jerk for S-curve (0 = trapezoidal, >0 = S-curve)
    jerk: float = 0.0


class TrajectoryState(Enum):
    IDLE = "idle"
    ACCELERATING = "accelerating"
    CONSTANT_VELOCITY = "constant_velocity"
    DECELERATING = "decelerating"
    FINISHED = "finished"


class TrapezoidalPlanner:
    """
    Trapezoidal velocity trajectory planner
    
    Generates smooth position trajectories with limited velocity and acceleration
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
        
        # Phase distances
        self.accel_distance = 0.0
        self.decel_distance = 0.0
        self.const_vel_distance = 0.0
        self.total_distance = 0.0
        
        # Time tracking
        self.start_time = 0.0
        self.phase_start_time = 0.0
        self.accel_time = 0.0
        self.const_vel_time = 0.0
        self.decel_time = 0.0
        self.total_time = 0.0
    
    def plan(self, start_pos: float, target_pos: float):
        """
        Plan a trapezoidal trajectory
        
        Args:
            start_pos: Starting position in degrees
            target_pos: Target position in degrees
        """
        self.reset()
        
        self.start_pos = start_pos
        self.target_pos = target_pos
        self.current_pos = start_pos
        
        self.total_distance = abs(target_pos - start_pos)
        self.direction = 1.0 if target_pos > start_pos else -1.0
        
        if self.total_distance < 0.001:  # Already there
            self.state = TrajectoryState.FINISHED
            return
        
        # Calculate trajectory parameters
        v_max = self.profile.max_velocity
        a_max = self.profile.max_acceleration
        d_max = self.profile.max_deceleration
        
        # Time to accelerate to max velocity
        t_accel = v_max / a_max
        # Distance during acceleration (d = 0.5 * a * t^2)
        d_accel = 0.5 * a_max * t_accel * t_accel
        
        # Time to decelerate from max velocity
        t_decel = v_max / d_max
        d_decel = 0.5 * d_max * t_decel * t_decel
        
        # Check if we can reach max velocity (triangular vs trapezoidal)
        if d_accel + d_decel >= self.total_distance:
            # Triangular profile - never reach max velocity
            # d_total = d_accel + d_decel = 0.5*a*t_accel^2 + 0.5*d*t_decel^2
            # v_peak = a*t_accel = d*t_decel
            # Solve for v_peak given d_total
            
            v_peak = math.sqrt(2 * self.total_distance / (1/a_max + 1/d_max))
            
            self.accel_time = v_peak / a_max
            self.decel_time = v_peak / d_max
            self.const_vel_time = 0.0
            
            self.accel_distance = 0.5 * a_max * self.accel_time ** 2
            self.decel_distance = 0.5 * d_max * self.decel_time ** 2
            self.const_vel_distance = 0.0
        else:
            # Trapezoidal profile - constant velocity phase
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
        Update trajectory and get next position setpoint
        
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
                # Transition to constant velocity or deceleration
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
                # Accelerating phase
                t = phase_elapsed
                self.current_vel = self.profile.max_acceleration * t
                self.current_pos = self.start_pos + self.direction * (0.5 * self.profile.max_acceleration * t * t)
        
        elif self.state == TrajectoryState.CONSTANT_VELOCITY:
            if phase_elapsed >= self.const_vel_time:
                # Transition to deceleration
                self.state = TrajectoryState.DECELERATING
                self.phase_start_time = now
                self.current_pos = self.start_pos + self.direction * (self.accel_distance + self.const_vel_distance)
            else:
                # Constant velocity phase
                t = phase_elapsed
                self.current_vel = self.profile.max_velocity
                self.current_pos = self.start_pos + self.direction * (self.accel_distance + self.current_vel * t)
        
        elif self.state == TrajectoryState.DECELERATING:
            if phase_elapsed >= self.decel_time:
                # Finished
                self.state = TrajectoryState.FINISHED
                self.current_pos = self.target_pos
                self.current_vel = 0.0
                print(f"[Trajectory] Finished at {self.target_pos:.2f}°")
            else:
                # Decelerating phase
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


class WHJMotionController(WHJMotorController):
    """
    Motor controller with smooth trajectory planning
    
    Wraps the base WHJMotorController with trapezoidal velocity profiles
    to prevent motor overheat from sudden large position changes.
    """
    
    def __init__(self, driver, motor_id: int, profile: Optional[MotionProfile] = None):
        super().__init__(driver, motor_id)
        self.profile = profile or MotionProfile()
        self.planner = TrapezoidalPlanner(self.profile)
        self.running = False
        self.update_interval = 0.002  # 500Hz update rate
    
    def move_to_position(self, target_pos: float, wait: bool = True, timeout: float = None) -> bool:
        """
        Move to target position with smooth trajectory
        
        Args:
            target_pos: Target position in degrees
            wait: Whether to wait for motion to complete
            timeout: Maximum time to wait (seconds)
        
        Returns:
            True if successful
        """
        # Get current position
        current_pos = self.get_position()
        if current_pos is None:
            print("[Error] Failed to get current position")
            return False
        
        # Check if already there
        distance = abs(target_pos - current_pos)
        if distance < 0.1:
            print(f"[Motion] Already at target position {target_pos:.2f}°")
            return True
        
        # Auto-calculate timeout based on distance and profile
        if timeout is None:
            v_max = self.profile.max_velocity
            a_max = self.profile.max_acceleration
            
            # Calculate theoretical motion time
            t_accel = v_max / a_max
            d_accel = 0.5 * a_max * t_accel * t_accel
            
            if 2 * d_accel >= distance:
                # Triangular profile
                t_total = 2 * math.sqrt(distance / a_max)
            else:
                # Trapezoidal profile
                t_const = (distance - 2 * d_accel) / v_max
                t_total = 2 * t_accel + t_const
            
            # Add 50% margin, minimum 5 seconds
            timeout = max(t_total * 1.5, 5.0)
            print(f"[Motion] Auto timeout: {timeout:.1f}s for {distance:.1f}° move")
        
        # Plan trajectory
        self.planner.plan(current_pos, target_pos)
        
        # Ensure motor is enabled and in position mode
        if not self.is_enabled():
            print("[Motion] Enabling motor...")
            if not self.enable(True):
                print("[Error] Failed to enable motor")
                return False
            time.sleep(0.1)
        
        current_mode = self.get_work_mode()
        if current_mode != "POSITION_MODE (Position)":
            print("[Motion] Setting position mode...")
            if not self.set_work_mode(WorkMode.POSITION_MODE):
                print("[Error] Failed to set position mode")
                return False
            time.sleep(0.05)
        
        # Execute trajectory
        print(f"[Motion] Executing trajectory...")
        self.running = True
        
        start_time = time.time()
        last_update = start_time
        
        try:
            while self.running:
                now = time.time()
                
                # Check timeout
                if now - start_time > timeout:
                    print(f"[Motion] Timeout after {timeout:.1f}s")
                    return False
                
                # Update trajectory at fixed rate
                if now - last_update >= self.update_interval:
                    pos, vel, finished = self.planner.update()
                    
                    # Send position command
                    if not self.set_target_position(pos):
                        print(f"[Error] Failed to send position command")
                        return False
                    
                    # Print progress every 0.5s
                    elapsed = now - start_time
                    if int(elapsed * 2) != int((elapsed - self.update_interval) * 2):
                        actual_pos = self.get_position()
                        if actual_pos is not None:
                            print(f"  Target: {pos:7.2f}° | Actual: {actual_pos:7.2f}° | Vel: {vel:6.2f}°/s")
                    
                    last_update = now
                    
                    if finished:
                        break
                
                time.sleep(0.001)
            
            # Final position set
            self.set_target_position(target_pos)
            time.sleep(0.1)
            
            final_pos = self.get_position()
            if final_pos is not None:
                error = abs(final_pos - target_pos)
                print(f"[Motion] Reached {final_pos:.2f}° (error: {error:.3f}°)")
                return error < 1.0  # Within 1 degree
            
            return True
            
        except KeyboardInterrupt:
            print("\n[Motion] Interrupted by user")
            # Stop motor
            current = self.get_position()
            if current is not None:
                self.set_target_position(current)
            return False
        finally:
            self.running = False
    
    def stop(self):
        """Stop current motion"""
        self.running = False
        self.planner.reset()


def main():
    global global_driver, global_motor
    
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    
    print("=" * 70)
    print("RealMan WHJ Motion Controller (WHJMotionController)")
    print("=" * 70)
    print(f"Motor ID: {motor_id}")
    print()
    
    # Initialize CAN
    driver = None
    motor = None
    
    try:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        global_driver = driver
    except RuntimeError as e:
        print(f"[Error] Failed to open CAN device: {e}")
        return
    
    # Create motion controller with conservative profile
    profile = MotionProfile(
        max_velocity=1800.0,      # 180°/s (3 RPM) - conservative
        max_acceleration=720.0,  # 360°/s^2
        max_deceleration=720.0
    )
    
    motor = WHJMotionController(driver, motor_id, profile)
    global_motor = motor
    
    # Initialize
    if not motor.initialize():
        print("\n[Error] Failed to initialize motor")
        driver.close()
        return
    
    print()
    print("-" * 70)
    print("Motion Controller Ready")
    print("-" * 70)
    print(f"Motion Profile:")
    print(f"  Max Velocity: {profile.max_velocity}°/s ({profile.max_velocity/6:.1f} RPM)")
    print(f"  Max Acceleration: {profile.max_acceleration}°/s²")
    print()
    print("Commands:")
    print("  m <pos>  - Move to position with smooth trajectory")
    print("  e        - Enable motor")
    print("  d        - Disable motor")
    print("  c        - Clear errors")
    print("  r        - Read current position")
    print("  s        - Show status")
    print("  q        - Quit")
    print()
    
    while True:
        try:
            cmd_input = input("> ").strip().lower()
            parts = cmd_input.split()
            cmd = parts[0] if parts else ""
            
            if cmd == 'q':
                break
            
            elif cmd == 'e':
                if motor.enable(True):
                    print("Motor enabled")
                else:
                    print("Failed to enable")
            
            elif cmd == 'd':
                motor.stop()
                if motor.enable(False):
                    print("Motor disabled")
                else:
                    print("Failed to disable")
            
            elif cmd == 'c':
                if motor.clear_error():
                    print("Errors cleared")
                else:
                    print("Failed to clear errors")
            
            elif cmd == 'm' and len(parts) >= 2:
                try:
                    target = float(parts[1])
                    motor.move_to_position(target, wait=True)
                except ValueError:
                    print("Usage: m <position_in_degrees>")
            
            elif cmd == 'r':
                pos = motor.get_position()
                if pos is not None:
                    print(f"Current position: {pos:.4f}°")
                else:
                    print("Failed to read position")
            
            elif cmd == 's':
                error = motor.get_error_status()
                if error:
                    code, errors = error
                    print(f"Error Code: 0x{code:04X}")
                    if code != 0:
                        for e in errors:
                            print(f"  - {e}")
                enabled = motor.is_enabled()
                if enabled is not None:
                    print(f"Enabled: {'Yes' if enabled else 'No'}")
                pos = motor.get_position()
                if pos is not None:
                    print(f"Position: {pos:.4f}°")            
            else:
                print("Unknown command or missing argument")
        
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    # Cleanup
    if motor:
        print("\nStopping motion and disabling motor...")
        motor.stop()
        try:
            motor.enable(False)
        except:
            pass
    
    if driver:
        driver.close()
    print("Done!")


if __name__ == "__main__":
    main()