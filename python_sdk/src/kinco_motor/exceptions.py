"""
Exception definitions for kinco_motor package.
"""


class KincoError(Exception):
    """Kinco motor error."""
    
    def __init__(self, message: str, motor_id: int = None):
        super().__init__(message)
        self.motor_id = motor_id
