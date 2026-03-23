"""
Kinco Motor Python Driver

Professional Python SDK for controlling Kinco FD1X5 series servo motors
via CANopen PDO protocol on standard CAN.

Example:
    from zlg_can import ZlgCanDriver, ZCANDeviceType
    from kinco_motor import KincoMotor
    
    # Initialize CAN interface
    with ZlgCanDriver() as can:
        can.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
        can.init_canfd(arbitration_bps=1000000)
        
        # Control Kinco motor
        with KincoMotor(can, node_id=1) as motor:
            motor.initialize()
            motor.move_to_degree(90.0, velocity_rpm=50.0)
            
            # Set current position as origin
            motor.set_origin()
"""

from .__version__ import __version__, __author__, __description__

# Protocol
from .protocol import (
    KincoProtocol,
    KincoMode,
    KincoNMTCommand,
    KincoControlWord,
    KincoState,
    KincoConfig,
)

# Motor
from .motor import KincoMotor

# Config
from .config import KincoMotorConfig

# Exceptions
from .exceptions import KincoError

__all__ = [
    # Version
    '__version__',
    '__author__',
    '__description__',
    
    # Protocol
    'KincoProtocol',
    'KincoMode',
    'KincoNMTCommand',
    'KincoControlWord',
    'KincoState',
    'KincoConfig',
    
    # Motor
    'KincoMotor',
    
    # Config
    'KincoMotorConfig',
    
    # Exceptions
    'KincoError',
]
