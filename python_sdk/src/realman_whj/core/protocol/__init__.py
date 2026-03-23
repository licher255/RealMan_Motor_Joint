"""
Protocol implementations for WHJ motor communication.
"""

from .whj import (
    WHJProtocol,
    Register,
    WorkMode,
    JointModel,
    JointState,
    MotorInfo,
    ErrorCode,
)

__all__ = [
    # WHJ Protocol only
    'WHJProtocol',
    'Register',
    'WorkMode',
    'JointModel',
    'JointState',
    'MotorInfo',
    'ErrorCode',
]
