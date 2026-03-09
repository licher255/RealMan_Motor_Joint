"""
RealMan Motor Joint - Core Module

核心模块包含底层的CAN通信和协议实现。

Modules:
    zlgcan_driver: ZLG CAN设备驱动
    protocol.whj_protocol: RealMan WHJ电机协议
    protocol.kinco_protocol: Kinco电机协议
"""

from .zlgcan_driver import (
    ZlgCanDriver,
    ZCANDeviceType,
    CANFDFrame,
    ZCAN_STATUS_OK,
    ZCAN_TYPE_CANFD,
)

__all__ = [
    'ZlgCanDriver',
    'ZCANDeviceType', 
    'CANFDFrame',
    'ZCAN_STATUS_OK',
    'ZCAN_TYPE_CANFD',
]
