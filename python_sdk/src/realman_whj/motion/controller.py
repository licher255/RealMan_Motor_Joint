"""
Motion controller with trajectory execution.

Combines trajectory planning with real-time motor control.
"""

import time
import threading
from typing import Optional, Callable

from .planner import TrapezoidalPlanner, MotionProfile, TrapezoidalTrajectory
from ..motor.base import BaseMotor
from ..exceptions import MotionError


class MotionController:
    """
    Motion controller with trajectory planning and execution.
    
    Provides smooth motion control with trapezoidal velocity profiles.
    Supports both blocking and non-blocking execution.
    
    Example:
        motor = WHJMotor(can_interface, motor_id=7)
        controller = MotionController(motor)
        
        # Blocking motion
        controller.move_to(90.0)
        
        # Non-blocking motion with callback
        controller.move_to_async(90.0, on_complete=lambda: print("Done!"))
    """
    
    def __init__(self, motor: BaseMotor, profile: Optional[MotionProfile] = None):
        """
        Initialize motion controller.
        
        Args:
            motor: Motor instance to control
            profile: Motion profile (uses motor's profile if not provided)
        """
        self._motor = motor
        self._profile = profile or motor.profile
        self._planner = TrapezoidalPlanner(self._profile)
        
        # Execution state
        self._lock = threading.RLock()
        self._motion_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._is_moving = False
        self._current_trajectory: Optional[TrapezoidalTrajectory] = None
        
        # Callbacks
        self._on_motion_start: Optional[Callable[[float], None]] = None
        self._on_motion_complete: Optional[Callable[[float], None]] = None
        self._on_motion_error: Optional[Callable[[str], None]] = None
    
    @property
    def profile(self) -> MotionProfile:
        """Get motion profile."""
        return self._profile
    
    @profile.setter
    def profile(self, profile: MotionProfile):
        """Set motion profile."""
        self._profile = profile
        self._planner.profile = profile
    
    @property
    def is_moving(self) -> bool:
        """Check if motion is in progress."""
        with self._lock:
            return self._is_moving
    
    def set_callbacks(self,
                      on_start: Optional[Callable[[float], None]] = None,
                      on_complete: Optional[Callable[[float], None]] = None,
                      on_error: Optional[Callable[[str], None]] = None):
        """
        Set motion callbacks.
        
        Args:
            on_start: Called when motion starts (receives target position)
            on_complete: Called when motion completes (receives final position)
            on_error: Called when motion error occurs (receives error message)
        """
        self._on_motion_start = on_start
        self._on_motion_complete = on_complete
        self._on_motion_error = on_error
    
    def move_to(self, target_position: float, timeout: float = None) -> bool:
        """
        Move to target position (blocking).
        
        Args:
            target_position: Target position in degrees
            timeout: Maximum time to wait (seconds), None for auto-calculate
            
        Returns:
            True if motion completed successfully
            
        Raises:
            MotionError: If motion fails
        """
        with self._lock:
            if self._is_moving:
                raise MotionError("Motion already in progress")
            
            # Get current position
            current_pos = self._motor.get_position()
            if current_pos is None:
                raise MotionError("Failed to read current position")
            
            # Plan trajectory
            traj = self._planner.plan(current_pos, target_position)
            self._current_trajectory = traj
            
            # Execute motion
            return self._execute_motion(target_position, traj, blocking=True)
    
    def move_to_async(self, target_position: float) -> bool:
        """
        Move to target position (non-blocking).
        
        Args:
            target_position: Target position in degrees
            
        Returns:
            True if motion started successfully
        """
        with self._lock:
            if self._is_moving:
                return False
            
            # Get current position
            current_pos = self._motor.get_position()
            if current_pos is None:
                if self._on_motion_error:
                    self._on_motion_error("Failed to read current position")
                return False
            
            # Plan trajectory
            traj = self._planner.plan(current_pos, target_position)
            self._current_trajectory = traj
            
            # Start motion thread
            self._stop_event.clear()
            self._is_moving = True
            
            self._motion_thread = threading.Thread(
                target=self._execute_motion,
                args=(target_position, traj, False),
                daemon=True
            )
            self._motion_thread.start()
            
            return True
    
    def _execute_motion(self, target_position: float, 
                        trajectory: TrapezoidalTrajectory,
                        blocking: bool) -> bool:
        """
        Execute planned motion.
        
        Args:
            target_position: Target position
            trajectory: Planned trajectory
            blocking: Whether this is a blocking call
            
        Returns:
            True if successful
        """
        if not blocking:
            with self._lock:
                self._is_moving = True
        
        try:
            if self._on_motion_start:
                self._on_motion_start(target_position)
            
            # Get interpolation points
            points = self._planner.get_interpolation_points(sample_rate_hz=100)
            
            start_time = time.time()
            
            for t_target, pos_target in points:
                # Check stop signal
                if self._stop_event.is_set():
                    raise MotionError("Motion stopped by user")
                
                # Wait until target time
                elapsed = time.time() - start_time
                wait_time = t_target - elapsed
                if wait_time > 0:
                    time.sleep(wait_time)
                
                # Send position command (no wait for response during motion)
                self._motor._do_move(pos_target) if hasattr(self._motor, '_do_move') else \
                    self._motor.move_to(pos_target, blocking=False)
            
            # Motion complete
            if self._on_motion_complete:
                self._on_motion_complete(target_position)
            
            return True
            
        except Exception as e:
            error_msg = str(e)
            if self._on_motion_error:
                self._on_motion_error(error_msg)
            if blocking:
                raise MotionError(error_msg)
            return False
            
        finally:
            if not blocking:
                with self._lock:
                    self._is_moving = False
                    self._current_trajectory = None
    
    def stop(self) -> bool:
        """
        Stop current motion.
        
        Returns:
            True if stopped successfully
        """
        with self._lock:
            self._stop_event.set()
            
            if self._motion_thread and self._motion_thread.is_alive():
                self._motion_thread.join(timeout=2.0)
            
            # Stop motor
            self._motor.stop()
            
            self._is_moving = False
            self._current_trajectory = None
            
            return True
    
    def wait_for_completion(self, timeout: float = None) -> bool:
        """
        Wait for motion to complete.
        
        Args:
            timeout: Maximum time to wait (seconds)
            
        Returns:
            True if motion completed, False if timeout
        """
        with self._lock:
            if not self._is_moving:
                return True
            
            thread = self._motion_thread
        
        if thread:
            thread.join(timeout=timeout)
            return not thread.is_alive()
        
        return True
    
    def get_progress(self) -> float:
        """
        Get current motion progress.
        
        Returns:
            Progress from 0.0 to 1.0
        """
        with self._lock:
            if not self._is_moving or self._current_trajectory is None:
                return 1.0 if not self._is_moving else 0.0
            
            # This would need timing info from the motion thread
            # For now, return estimated progress based on motor position
            current_pos = self._motor.get_position()
            if current_pos is None:
                return 0.0
            
            # Estimate progress (rough approximation)
            return 0.5  # Placeholder
    
    def get_remaining_time(self) -> Optional[float]:
        """
        Get estimated remaining time.
        
        Returns:
            Remaining time in seconds, or None if not moving
        """
        with self._lock:
            if not self._is_moving or self._current_trajectory is None:
                return None
            
            # Would need actual timing from motion thread
            return self._current_trajectory.t_total * (1 - self.get_progress())
