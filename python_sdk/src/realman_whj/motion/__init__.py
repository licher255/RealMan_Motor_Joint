"""
Motion control module.

Provides trajectory planning and motion control functionality.
"""

from .planner import TrapezoidalPlanner, MotionProfile, MotionState
from .controller import MotionController

__all__ = [
    'TrapezoidalPlanner',
    'MotionProfile',
    'MotionState',
    'MotionController',
]
