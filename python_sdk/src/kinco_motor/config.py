"""
Kinco motor configuration.
"""

from dataclasses import dataclass, asdict
from typing import Dict, Any


@dataclass
class KincoMotorConfig:
    """Kinco motor configuration."""
    node_id: int = 1
    gear_ratio: float = 16384.0           # Pulses per revolution
    default_velocity_rpm: float = 50.0    # Default velocity for moves (RPM)
    timeout_ms: int = 500                 # Communication timeout (ms)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary."""
        return asdict(self)
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "KincoMotorConfig":
        """Create from dictionary."""
        return cls(**{k: v for k, v in data.items() if k in cls.__dataclass_fields__})
