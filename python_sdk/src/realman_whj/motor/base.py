"""
Base motor interface.

Defines the abstract base class for all motor implementations.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional
from enum import Enum, auto


class MotorState(Enum):
    """Motor state."""
    IDLE = auto()
    INITIALIZING = auto()
    ENABLED = auto()
    MOVING = auto()
    ERROR = auto()
    DISABLED = auto()


@dataclass
class MotionProfile:
    """Motion profile configuration."""
    max_velocity: float = 1000.0       # deg/s
    max_acceleration: float = 2000.0   # deg/s^2
    max_deceleration: float = 2000.0   # deg/s^2
    position_tolerance: float = 0.1    # degrees


class BaseMotor(ABC):
    """
    Abstract base class for motor control.
    
    All motor implementations should inherit from this class.
    """
    
    def __init__(self, motor_id: int, name: str = ""):
        """
        Initialize motor.
        
        Args:
            motor_id: Motor identifier
            name: Human-readable name
        """
        self._motor_id = motor_id
        self._name = name or f"Motor_{motor_id}"
        self._state = MotorState.IDLE
        self._profile = MotionProfile()
    
    @property
    def motor_id(self) -> int:
        """Get motor ID."""
        return self._motor_id
    
    @property
    def name(self) -> str:
        """Get motor name."""
        return self._name
    
    @property
    def state(self) -> MotorState:
        """Get current state."""
        return self._state
    
    @property
    def profile(self) -> MotionProfile:
        """Get motion profile."""
        return self._profile
    
    @profile.setter
    def profile(self, profile: MotionProfile):
        """Set motion profile."""
        self._profile = profile
    
    @abstractmethod
    def initialize(self) -> bool:
        """Initialize the motor."""
        pass
    
    @abstractmethod
    def enable(self, enabled: bool = True) -> bool:
        """Enable or disable the motor."""
        pass
    
    @abstractmethod
    def get_position(self) -> Optional[float]:
        """Get current position in degrees."""
        pass
    
    @abstractmethod
    def move_to(self, position_deg: float, blocking: bool = True) -> bool:
        """Move to target position."""
        pass
    
    @abstractmethod
    def stop(self) -> bool:
        """Stop motion."""
        pass
    
    @abstractmethod
    def clear_error(self) -> bool:
        """Clear errors."""
        pass
    
    @abstractmethod
    def get_error_code(self) -> Optional[int]:
        """Get error code."""
        pass
    
    @abstractmethod
    def close(self):
        """Close motor and cleanup."""
        pass
    
    def is_idle(self) -> bool:
        """Check if motor is idle."""
        return self._state == MotorState.IDLE
    
    def is_moving(self) -> bool:
        """Check if motor is moving."""
        return self._state == MotorState.MOVING
    
    def has_error(self) -> bool:
        """Check if motor has error."""
        return self._state == MotorState.ERROR
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False
