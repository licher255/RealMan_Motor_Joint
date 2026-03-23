"""
Exception definitions for zlg_can package.
"""


class CANError(Exception):
    """CAN communication error."""
    
    def __init__(self, message: str, error_code: int = None):
        super().__init__(message)
        self.error_code = error_code


class TimeoutError(CANError):
    """Operation timeout error."""
    pass


class ProtocolError(CANError):
    """Protocol error."""
    pass


class ResourceNotFoundError(Exception):
    """Resource (e.g., DLL) not found error."""
    pass
