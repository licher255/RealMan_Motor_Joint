"""
RealMan WHJ Joint Motor Python SDK

A professional SDK for controlling RealMan WHJ series joint motors.

Example:
    from zlg_can import ZlgCanDriver, ZCANDeviceType
    from realman_whj import WHJMotor, MotionController, MotionProfile
    
    # Initialize CAN interface
    can = ZlgCanDriver()
    can.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
    can.init_canfd(arbitration_bps=1000000, data_bps=5000000)
    
    # WHJ Motor (CAN FD)
    with WHJMotor(can, motor_id=7) as whj:
        whj.initialize()
        whj.move_to(90.0)

Note: For Kinco motor support, use the separate 'kinco_motor' package.
"""

from .__version__ import __version__, __author__, __description__

# Re-export from zlg_can for convenience
from zlg_can import (
    CANInterface,
    CANFrame,
    CANFDFrame,
    ZlgCanDriver,
    ZCANDeviceType,
    CANError,
    TimeoutError,
    ProtocolError,
)

# WHJ Protocol
from .core.protocol import (
    WHJProtocol,
    Register,
    WorkMode,
    JointModel,
    JointState,
    MotorInfo,
    ErrorCode,
)

# Motor components
from .motor import (
    BaseMotor,
    MotorState,
    WHJMotor,
)

# Motion components
from .motion import (
    TrapezoidalPlanner,
    MotionProfile,
    MotionState,
    MotionController,
)

# Configuration
from .config import (
    SDKConfig,
    CANConfig,
    WHJMotorConfig,
    get_config,
    set_config,
    load_config,
    save_config,
    # Linear motion conversion constants
    MM_PER_DEGREE,
    DEGREE_PER_MM,
)

# Exceptions
from .exceptions import (
    RealManError,
    MotorError,
    ConfigurationError,
    MotionError,
)

# Utilities
from .utils import get_logger, set_log_level

__all__ = [
    # Version
    '__version__',
    '__author__',
    '__description__',
    
    # Core (from zlg_can)
    'CANInterface',
    'CANFrame',
    'CANFDFrame',
    'ZlgCanDriver',
    'ZCANDeviceType',
    
    # WHJ Protocol
    'WHJProtocol',
    'Register',
    'WorkMode',
    'JointModel',
    'JointState',
    'MotorInfo',
    'ErrorCode',
    
    # Motor
    'BaseMotor',
    'MotorState',
    'WHJMotor',
    
    # Motion
    'TrapezoidalPlanner',
    'MotionProfile',
    'MotionState',
    'MotionController',
    
    # Config
    'SDKConfig',
    'CANConfig',
    'WHJMotorConfig',
    'get_config',
    'set_config',
    'load_config',
    'save_config',
    # Linear motion conversion constants
    'MM_PER_DEGREE',
    'DEGREE_PER_MM',
    
    # Exceptions
    'RealManError',
    'MotorError',
    'ConfigurationError',
    'MotionError',
    'CANError',
    'TimeoutError',
    'ProtocolError',
    
    # Utils
    'get_logger',
    'set_log_level',
]
