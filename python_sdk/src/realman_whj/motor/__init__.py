"""
WHJ Motor control module.

Provides high-level control interfaces for RealMan WHJ series motors.
"""

from .base import BaseMotor, MotorState
from .whj import WHJMotor

__all__ = [
    'BaseMotor',
    'MotorState',
    'WHJMotor',
]
