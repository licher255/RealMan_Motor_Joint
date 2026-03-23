"""
ZLG CAN Driver - USB-CAN FD device driver for Python.

This package provides CAN communication support for ZLG devices:
- USBCANFD-100U-mini
- USBCANFD-100U/200U/400U/800U

Supports mixed mode: CAN FD + Standard CAN on the same bus.

Example:
    from zlg_can import ZlgCanDriver, ZCANDeviceType
    
    with ZlgCanDriver() as can:
        can.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
        can.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        
        # Send CAN FD frame
        can.send_canfd(can_id=0x07, data=b'...')
        
        # Send Standard CAN frame
        can.send_can(can_id=0x201, data=b'...')
"""

from .__version__ import __version__, __author__, __description__
from .interface import CANInterface, CANFrame, CANFDFrame, FrameType
from .zlg_driver import ZlgCanDriver, ZCANDeviceType
from .exceptions import CANError, TimeoutError, ProtocolError

__all__ = [
    # Version
    '__version__',
    '__author__',
    '__description__',
    
    # Interface
    'CANInterface',
    'CANFrame',
    'CANFDFrame',
    'FrameType',
    
    # Driver
    'ZlgCanDriver',
    'ZCANDeviceType',
    
    # Exceptions
    'CANError',
    'TimeoutError',
    'ProtocolError',
]
