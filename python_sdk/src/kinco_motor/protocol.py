"""
Kinco Rotary Servo Motor Protocol

Motor Model: FD135 Driver + Q6 Motor
Communication: CAN 2.0A Standard Frames
Baud Rate: 1Mbps

Protocol Specifications:
- CAN ID Format:
  - NMT Command: 0x000
  - Control Word/Mode: 0x201
  - Position Control: 0x301
  - TPDO1 Feedback: 0x181 + NodeID
  
- Data Format:
  - Position: angle × 16384 (gear ratio), signed 32-bit, little-endian
  - Speed: RPM × (65536×512/1875), unsigned 32-bit, little-endian
"""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, List


class KincoMode(IntEnum):
    """Kinco Work Modes."""
    RELATIVE_POSITION = 0x0F  # Relative position mode
    ABSOLUTE_POSITION = 0x10  # Absolute position mode
    SPEED_MODE = 0x03         # Speed mode
    HOMING_MODE = 0x06        # Homing mode (control mode 6)


class KincoNMTCommand(IntEnum):
    """NMT (Network Management) Commands."""
    START = 0x01              # Start node
    STOP = 0x02               # Stop node
    PRE_OPERATIONAL = 0x80    # Enter pre-operational state
    RESET_NODE = 0x81         # Reset node
    RESET_COMM = 0x82         # Reset communication


class KincoControlWord:
    """Kinco Control Words."""
    ENABLE = 0x3F             # Enable control word (high byte)
    HOME_1 = 0x0F             # Homing step 1
    HOME_2 = 0x1F             # Homing step 2
    DISABLE = 0x06            # Disable
    FAULT_RESET = 0x86        # Fault reset


@dataclass
class KincoState:
    """Kinco motor state."""
    motor_id: int
    position_deg: float = 0.0       # Current position (degrees)
    target_position_deg: float = 0.0  # Target position (degrees)
    speed_rpm: float = 0.0          # Current speed (RPM)
    target_speed_rpm: float = 0.0   # Target speed (RPM)
    is_enabled: bool = False        # Driver enabled
    is_moving: bool = False         # Is moving
    error_code: int = 0             # Error code
    
    def to_dict(self) -> dict:
        """Convert to dictionary."""
        return {
            'motor_id': self.motor_id,
            'position_deg': self.position_deg,
            'target_position_deg': self.target_position_deg,
            'speed_rpm': self.speed_rpm,
            'target_speed_rpm': self.target_speed_rpm,
            'is_enabled': self.is_enabled,
            'is_moving': self.is_moving,
            'error_code': self.error_code,
        }


@dataclass
class KincoConfig:
    """Kinco configuration parameters."""
    node_id: int = 1
    gear_ratio: float = 16384.0     # Pulses per revolution
    rpm_scale: float = (65536 * 512) / 1875  # RPM to protocol units
    max_position: float = 360.0     # Maximum position (degrees)
    min_position: float = -360.0    # Minimum position (degrees)
    max_speed: float = 3000.0       # Maximum speed (RPM)
    default_velocity_rpm: float = 50.0  # Default velocity for moves


class KincoProtocol:
    """
    Kinco Protocol Implementation.
    
    Provides frame building and parsing functionality.
    """
    
    # CAN ID Definitions
    NMT_ID = 0x000          # NMT command
    RPDO1_ID = 0x201        # Control word + work mode
    RPDO2_ID = 0x301        # Position control
    TPDO1_BASE = 0x181      # Status feedback
    
    # Mechanical Parameters
    # According to operation guide: 90 degrees = 0x00168000 = 1474560 inc
    # Gear ratio = 1474560 / 90 = 16384 = 2^14 (pulses per revolution)
    GEAR_RATIO = 16384
    
    # Speed conversion factor: 1 RPM = 65536 * 512 / 1875 ≈ 17895.7
    RPM_TO_UNITS = (65536 * 512) / 1875
    
    @staticmethod
    def build_nmt_frame(node_id: int, command: KincoNMTCommand) -> bytes:
        """
        Build NMT command frame.
        
        Args:
            node_id: Node ID
            command: NMT command
            
        Returns:
            2 bytes: [command, node_id]
        """
        return bytes([command, node_id])
    
    @staticmethod
    def build_start_node(node_id: int) -> bytes:
        """Build start node command."""
        return KincoProtocol.build_nmt_frame(node_id, KincoNMTCommand.START)
    
    @staticmethod
    def build_stop_node(node_id: int) -> bytes:
        """Build stop node command."""
        return KincoProtocol.build_nmt_frame(node_id, KincoNMTCommand.STOP)
    
    @staticmethod
    def build_reset_node(node_id: int) -> bytes:
        """Build reset node command."""
        return KincoProtocol.build_nmt_frame(node_id, KincoNMTCommand.RESET_NODE)
    
    @staticmethod
    def build_rpdo1_frame(control_low: int, control_high: int, mode: int) -> bytes:
        """
        Build RPDO1 frame (control word + mode).
        
        Format: [control_low, control_high, mode, 0, 0, 0, 0, 0]
        
        Args:
            control_low: Control word low byte
            control_high: Control word high byte
            mode: Work mode
            
        Returns:
            8 bytes
        """
        return bytes([control_low, control_high, mode, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    @staticmethod
    def build_enable_absolute_mode() -> bytes:
        """Build enable + absolute position mode frame."""
        return KincoProtocol.build_rpdo1_frame(0x01, KincoControlWord.ENABLE, KincoMode.ABSOLUTE_POSITION)
    
    @staticmethod
    def build_enable_relative_mode() -> bytes:
        """Build enable + relative position mode frame."""
        return KincoProtocol.build_rpdo1_frame(0x01, KincoControlWord.ENABLE, KincoMode.RELATIVE_POSITION)
    
    @staticmethod
    def build_disable_frame() -> bytes:
        """Build disable frame (release brake)."""
        return KincoProtocol.build_rpdo1_frame(0x01, KincoControlWord.DISABLE, KincoMode.ABSOLUTE_POSITION)
    
    @staticmethod
    def build_fault_reset_frame() -> bytes:
        """Build fault reset frame."""
        return KincoProtocol.build_rpdo1_frame(0x01, KincoControlWord.FAULT_RESET, KincoMode.ABSOLUTE_POSITION)
    
    @staticmethod
    def build_homing_step1_frame() -> bytes:
        """
        Build homing step 1 frame.
        
        Send: 0x201, [06, 0F, 00, 00, 00, 00, 00, 00]
        """
        return bytes([0x06, KincoControlWord.HOME_1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    @staticmethod
    def build_homing_step2_frame() -> bytes:
        """
        Build homing step 2 frame.
        
        Send: 0x201, [06, 1F, 00, 00, 00, 00, 00, 00]
        """
        return bytes([0x06, KincoControlWord.HOME_2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    @staticmethod
    def build_position_frame(position_deg: float, speed_rpm: float = 50.0) -> bytes:
        """
        Build position control frame (RPDO2, ID=0x301).
        
        Data Format (8 bytes, Little-endian):
        - Byte 0-3: Target position (int32) = angle × GEAR_RATIO
        - Byte 4-7: Target speed (uint32) = RPM × RPM_TO_UNITS
        
        Args:
            position_deg: Target position (degrees, positive=counter-clockwise)
            speed_rpm: Target speed (RPM)
            
        Returns:
            8 bytes
        """
        # Position conversion: degrees × gear ratio
        position_units = int(position_deg * KincoProtocol.GEAR_RATIO)
        
        # Speed conversion: RPM × conversion factor
        speed_units = int(speed_rpm * KincoProtocol.RPM_TO_UNITS)
        
        # Build data: int32 (position) + uint32 (speed)
        data = struct.pack('<i', position_units)   # 4 bytes position
        data += struct.pack('<I', speed_units)     # 4 bytes speed
        
        return data
    
    @staticmethod
    def parse_tpdo1_frame(data: bytes) -> Optional[dict]:
        """
        Parse TPDO1 frame (position and speed feedback).
        
        TPDO1 Format:
        - Byte 0-3: Actual position (int32, little-endian)
        - Byte 4-7: Actual speed (int32, little-endian)
        
        Args:
            data: Frame data
            
        Returns:
            Parsed result dict or None
        """
        if len(data) < 8:
            return None
        
        # Parse position and velocity
        position_units = struct.unpack('<i', data[0:4])[0]
        velocity_raw = struct.unpack('<i', data[4:8])[0]
        
        return {
            'position_inc': position_units,
            'position_deg': position_units / KincoProtocol.GEAR_RATIO,
            'velocity_raw': velocity_raw,
            'velocity_rpm': velocity_raw / KincoProtocol.RPM_TO_UNITS,
        }
    
    @staticmethod
    def degree_to_units(position_deg: float) -> int:
        """Convert degrees to protocol units."""
        return int(position_deg * KincoProtocol.GEAR_RATIO)
    
    @staticmethod
    def units_to_degree(position_units: int) -> float:
        """Convert protocol units to degrees."""
        return position_units / KincoProtocol.GEAR_RATIO
    
    @staticmethod
    def rpm_to_units(speed_rpm: float) -> int:
        """Convert RPM to protocol units."""
        return int(speed_rpm * KincoProtocol.RPM_TO_UNITS)
    
    @staticmethod
    def units_to_rpm(speed_units: int) -> float:
        """Convert protocol units to RPM."""
        return speed_units / KincoProtocol.RPM_TO_UNITS
