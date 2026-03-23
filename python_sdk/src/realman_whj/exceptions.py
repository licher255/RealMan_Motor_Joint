"""
Exception definitions for RealMan WHJ SDK.

All exceptions inherit from RealManError for easy catching.
"""


class RealManError(Exception):
    """Base exception for all RealMan SDK errors."""
    pass


class CANError(RealManError):
    """CAN communication error."""
    
    def __init__(self, message: str, error_code: int = None):
        super().__init__(message)
        self.error_code = error_code


class ProtocolError(RealManError):
    """Protocol parsing or validation error."""
    pass


class MotorError(RealManError):
    """Motor control error."""
    
    def __init__(self, message: str, motor_id: int = None, error_code: int = None):
        super().__init__(message)
        self.motor_id = motor_id
        self.error_code = error_code


class TimeoutError(RealManError):
    """Operation timeout."""
    pass


class ConfigurationError(RealManError):
    """Configuration error."""
    pass


class MotionError(RealManError):
    """Motion control error."""
    pass


class ResourceNotFoundError(RealManError):
    """Resource (e.g., DLL) not found."""
    pass
