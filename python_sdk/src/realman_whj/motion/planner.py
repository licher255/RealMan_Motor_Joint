"""
Trajectory planning algorithms.

Implements trapezoidal velocity profile for smooth motion.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple
from enum import Enum, auto


class MotionState(Enum):
    """Motion execution state."""
    IDLE = auto()
    ACCELERATING = auto()
    CONSTANT_VELOCITY = auto()
    DECELERATING = auto()
    COMPLETED = auto()


@dataclass
class MotionProfile:
    """
    Motion profile parameters.
    
    Attributes:
        max_velocity: Maximum velocity (degrees/second)
        max_acceleration: Maximum acceleration (degrees/second^2)
        max_deceleration: Maximum deceleration (degrees/second^2)
        position_tolerance: Position error tolerance (degrees)
    """
    max_velocity: float = 1000.0       # deg/s
    max_acceleration: float = 2000.0   # deg/s^2
    max_deceleration: float = 2000.0   # deg/s^2
    position_tolerance: float = 0.1    # degrees
    
    def validate(self):
        """Validate profile parameters."""
        if self.max_velocity <= 0:
            raise ValueError("max_velocity must be positive")
        if self.max_acceleration <= 0:
            raise ValueError("max_acceleration must be positive")
        if self.max_deceleration <= 0:
            raise ValueError("max_deceleration must be positive")


@dataclass
class TrapezoidalTrajectory:
    """
    Computed trapezoidal trajectory.
    
    Attributes:
        total_distance: Total movement distance
        t_acc: Acceleration phase duration
        t_const: Constant velocity phase duration
        t_dec: Deceleration phase duration
        t_total: Total motion duration
        d_acc: Distance during acceleration
        d_const: Distance during constant velocity
        d_dec: Distance during deceleration
        max_vel_reached: Maximum velocity actually reached
    """
    total_distance: float
    t_acc: float
    t_const: float
    t_dec: float
    t_total: float
    d_acc: float
    d_const: float
    d_dec: float
    max_vel_reached: float


class TrapezoidalPlanner:
    """
    Trapezoidal velocity profile planner.
    
    Generates smooth motion trajectories with acceleration,
    constant velocity, and deceleration phases.
    
    Example:
        profile = MotionProfile(max_velocity=1000, max_acceleration=2000)
        planner = TrapezoidalPlanner(profile)
        
        traj = planner.plan(0, 90)  # Plan motion from 0 to 90 degrees
        
        for t in np.linspace(0, traj.t_total, 100):
            pos, vel, acc = planner.evaluate(t)
            print(f"t={t:.3f}: pos={pos:.2f}, vel={vel:.2f}, acc={acc:.2f}")
    """
    
    def __init__(self, profile: Optional[MotionProfile] = None):
        """
        Initialize planner.
        
        Args:
            profile: Motion profile parameters
        """
        self._profile = profile or MotionProfile()
        self._profile.validate()
        
        self._start_pos = 0.0
        self._end_pos = 0.0
        self._direction = 1
        self._trajectory: Optional[TrapezoidalTrajectory] = None
    
    @property
    def profile(self) -> MotionProfile:
        """Get current motion profile."""
        return self._profile
    
    @profile.setter
    def profile(self, profile: MotionProfile):
        """Set motion profile."""
        profile.validate()
        self._profile = profile
    
    def plan(self, start_position: float, end_position: float) -> TrapezoidalTrajectory:
        """
        Plan trajectory from start to end position.
        
        Args:
            start_position: Starting position (degrees)
            end_position: Target position (degrees)
            
        Returns:
            Computed trajectory parameters
        """
        self._start_pos = start_position
        self._end_pos = end_position
        
        distance = end_position - start_position
        self._direction = 1 if distance >= 0 else -1
        abs_distance = abs(distance)
        
        v_max = self._profile.max_velocity
        a_max = self._profile.max_acceleration
        d_max = self._profile.max_deceleration
        
        # Calculate minimum distance needed for triangular profile
        # (accelerate then immediately decelerate)
        t_acc_min = v_max / a_max
        t_dec_min = v_max / d_max
        d_acc_min = 0.5 * a_max * t_acc_min ** 2
        d_dec_min = 0.5 * d_max * t_dec_min ** 2
        d_min = d_acc_min + d_dec_min
        
        if abs_distance >= d_min:
            # Trapezoidal profile: accelerate, constant velocity, decelerate
            d_acc = d_acc_min
            d_dec = d_dec_min
            d_const = abs_distance - d_acc - d_dec
            
            t_acc = t_acc_min
            t_dec = t_dec_min
            t_const = d_const / v_max
            v_reached = v_max
        else:
            # Triangular profile: accelerate then decelerate (no constant velocity)
            # d = 0.5 * a * t_acc^2 + 0.5 * d * t_dec^2
            # With a*t_acc = d*t_dec = v_peak
            # Solve for t_acc and t_dec
            
            # For symmetric case (a = d):
            # d = a * t_acc^2 -> t_acc = sqrt(d/a)
            # v_peak = a * t_acc = sqrt(d*a)
            
            t_acc = math.sqrt(abs_distance / a_max)
            t_dec = math.sqrt(abs_distance / d_max)
            d_acc = 0.5 * a_max * t_acc ** 2
            d_dec = 0.5 * d_max * t_dec ** 2
            d_const = 0
            t_const = 0
            v_reached = a_max * t_acc
        
        t_total = t_acc + t_const + t_dec
        
        self._trajectory = TrapezoidalTrajectory(
            total_distance=abs_distance,
            t_acc=t_acc,
            t_const=t_const,
            t_dec=t_dec,
            t_total=t_total,
            d_acc=d_acc,
            d_const=d_const,
            d_dec=d_dec,
            max_vel_reached=v_reached
        )
        
        return self._trajectory
    
    def evaluate(self, time_sec: float) -> Tuple[float, float, float]:
        """
        Evaluate trajectory at given time.
        
        Args:
            time_sec: Time from start of motion (seconds)
            
        Returns:
            Tuple of (position, velocity, acceleration)
            
        Raises:
            RuntimeError: If plan() has not been called
        """
        if self._trajectory is None:
            raise RuntimeError("Must call plan() before evaluate()")
        
        traj = self._trajectory
        t = max(0, min(time_sec, traj.t_total))
        
        a_max = self._profile.max_acceleration
        d_max = self._profile.max_deceleration
        v_max = traj.max_vel_reached
        dir_ = self._direction
        
        if t < traj.t_acc:
            # Acceleration phase
            s = 0.5 * a_max * t ** 2
            v = a_max * t
            a = a_max
            state = MotionState.ACCELERATING
        elif t < (traj.t_acc + traj.t_const):
            # Constant velocity phase
            t_const = t - traj.t_acc
            s = traj.d_acc + v_max * t_const
            v = v_max
            a = 0
            state = MotionState.CONSTANT_VELOCITY
        else:
            # Deceleration phase
            t_dec = t - traj.t_acc - traj.t_const
            
            # Distance remaining
            s = traj.d_acc + traj.d_const + v_max * t_dec - 0.5 * d_max * t_dec ** 2
            v = max(0, v_max - d_max * t_dec)
            a = -d_max
            state = MotionState.DECELERATING
        
        position = self._start_pos + dir_ * s
        velocity = dir_ * v
        acceleration = dir_ * a
        
        return position, velocity, acceleration
    
    def get_state_at(self, time_sec: float) -> MotionState:
        """
        Get motion state at given time.
        
        Args:
            time_sec: Time from start of motion (seconds)
            
        Returns:
            Motion state
        """
        if self._trajectory is None:
            return MotionState.IDLE
        
        if time_sec < 0:
            return MotionState.IDLE
        elif time_sec >= self._trajectory.t_total:
            return MotionState.COMPLETED
        elif time_sec < self._trajectory.t_acc:
            return MotionState.ACCELERATING
        elif time_sec < (self._trajectory.t_acc + self._trajectory.t_const):
            return MotionState.CONSTANT_VELOCITY
        else:
            return MotionState.DECELERATING
    
    def is_completed(self, time_sec: float) -> bool:
        """Check if motion is completed at given time."""
        if self._trajectory is None:
            return False
        return time_sec >= self._trajectory.t_total
    
    def get_interpolation_points(self, sample_rate_hz: float = 100) -> list:
        """
        Generate interpolation points for motion execution.
        
        Args:
            sample_rate_hz: Sampling rate in Hz
            
        Returns:
            List of (time, position) tuples
        """
        if self._trajectory is None:
            return []
        
        dt = 1.0 / sample_rate_hz
        points = []
        t = 0.0
        
        while t <= self._trajectory.t_total:
            pos, _, _ = self.evaluate(t)
            points.append((t, pos))
            t += dt
        
        # Ensure final point is included
        if not points or points[-1][0] < self._trajectory.t_total:
            pos, _, _ = self.evaluate(self._trajectory.t_total)
            points.append((self._trajectory.t_total, pos))
        
        return points
