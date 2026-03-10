"""
Motor Communication Protocols

电机通信协议实现模块。

Protocols:
    WHJ: RealMan WHJ关节电机 (CAN FD)
    Kinco: Kinco旋转舵盘电机 (标准CAN, PDO方式)
"""

from .whj_protocol import (
    WHJProtocol,
    Register,
    WorkMode,
    JointModel,
    JointState,
    MotorInfo,
    ErrorCode,
    CMD_READ,
    CMD_WRITE,
    RESPONSE_ID_OFFSET,
)

from .kinco_protocol import (
    KincoProtocol,
    KincoMode,
    KincoState,
    KincoNMTCommand,
)

__all__ = [
    # WHJ Protocol (RealMan WHJ关节电机 - CAN FD)
    'WHJProtocol',
    'Register',
    'WorkMode',
    'JointModel',
    'JointState',
    'MotorInfo',
    'ErrorCode',
    'CMD_READ',
    'CMD_WRITE',
    'RESPONSE_ID_OFFSET',
    # Kinco Protocol (Kinco旋转舵盘电机 - 标准CAN)
    'KincoProtocol',
    'KincoMode',
    'KincoState',
    'KincoNMTCommand',
]
