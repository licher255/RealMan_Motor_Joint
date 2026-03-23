"""
Core module for protocol handling.

Note: CAN driver functionality is provided by 'zlg_can' package.
Use 'zlg_can' for CAN interface and driver imports.
"""

# Re-export from zlg_can for convenience
from zlg_can import (
    CANInterface,
    CANFrame,
    CANFDFrame,
    ZlgCanDriver,
    ZCANDeviceType,
)

__all__ = [
    'CANInterface',
    'CANFrame',
    'CANFDFrame',
    'ZlgCanDriver',
    'ZCANDeviceType',
]
