"""
Configuration management for RealMan WHJ SDK.

Simple JSON-based configuration.

Example:
    from realman_whj import load_config, get_config
    
    # Load from JSON file
    load_config("config.json")
    config = get_config()
    
    # Access motor config
    motor_cfg = config.get_motor_config(1)
"""

from dataclasses import dataclass, field, asdict
from typing import Optional, Dict, Any, Union
from pathlib import Path
import json


# =============================================================================
# Linear motion conversion constants (升降机构转换系数)
# Default: 1000° rotation = 18.0mm linear movement
# =============================================================================

MM_PER_DEGREE: float = 0.018  # 1000° = 18.0mm
DEGREE_PER_MM: float = 1000.0 / 18.0  # ≈55.5556°/mm


@dataclass
class CANConfig:
    """CAN bus configuration."""
    device_type: str = "USBCANFD_MINI"
    channel: int = 0
    arbitration_bps: int = 1_000_000
    data_bps: int = 5_000_000
    internal_resistance: bool = True
    dll_path: Optional[str] = None


@dataclass
class WHJMotorConfig:
    """WHJ motor configuration."""
    motor_id: int = 1
    max_velocity: float = 720.0           # deg/s
    max_acceleration: float = 360.0       # deg/s^2
    max_deceleration: float = 360.0       # deg/s^2
    position_tolerance: float = 0.1       # degrees
    motion_update_rate: int = 100         # Hz
    timeout_ms: int = 500
    retry_count: int = 3
    # Linear motion conversion (None = use global default)
    mm_per_degree: Optional[float] = None
    degree_per_mm: Optional[float] = None


@dataclass
class SDKConfig:
    """SDK global configuration."""
    can: CANConfig = field(default_factory=CANConfig)
    whj_motors: Dict[int, WHJMotorConfig] = field(default_factory=dict)
    log_level: str = "INFO"

    def get_motor_config(self, motor_id: int) -> WHJMotorConfig:
        """Get config for specific motor (returns default if not found)."""
        return self.whj_motors.get(motor_id, WHJMotorConfig(motor_id=motor_id))


# Global config instance
_global_config: Optional[SDKConfig] = None


def get_config() -> SDKConfig:
    """Get current global config."""
    global _global_config
    if _global_config is None:
        _global_config = SDKConfig()
    return _global_config


def set_config(config: SDKConfig):
    """Set global config."""
    global _global_config
    _global_config = config


def load_config(path: Union[str, Path]) -> SDKConfig:
    """
    Load configuration from JSON file.
    
    Args:
        path: Path to config.json file
        
    Returns:
        Loaded SDKConfig instance
        
    Example:
        from realman_whj import load_config
        load_config("config.json")
    """
    global _global_config
    path = Path(path)
    
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")
    
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    # Parse CAN config
    can_data = data.get("can", {})
    can_config = CANConfig(**{k: v for k, v in can_data.items() if k in CANConfig.__dataclass_fields__})
    
    # Parse motor configs
    motors_data = data.get("whj_motors", {})
    motors_config = {}
    for k, v in motors_data.items():
        motor_id = int(k)
        motor_cfg = WHJMotorConfig(**{k2: v2 for k2, v2 in v.items() if k2 in WHJMotorConfig.__dataclass_fields__})
        motors_config[motor_id] = motor_cfg
    
    _global_config = SDKConfig(
        can=can_config,
        whj_motors=motors_config,
        log_level=data.get("log_level", "INFO")
    )
    
    return _global_config


def save_config(path: Union[str, Path], config: Optional[SDKConfig] = None):
    """
    Save configuration to JSON file.
    
    Args:
        path: Path to save config.json
        config: Config to save (uses global if None)
        
    Example:
        from realman_whj import save_config
        save_config("config.json")
    """
    config = config or get_config()
    path = Path(path)
    
    data = {
        "can": {k: v for k, v in asdict(config.can).items() if v is not None},
        "whj_motors": {str(k): {k2: v2 for k2, v2 in asdict(v).items() if v2 is not None} 
                      for k, v in config.whj_motors.items()},
        "log_level": config.log_level,
    }
    
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2)
