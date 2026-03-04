"""
RealMan WHJ Joint Motor Protocol
Implements CAN FD communication protocol for RealMan WHJ series motors
"""

from dataclasses import dataclass
from typing import Optional, List, Tuple
from enum import IntEnum
import time


# ============================================================================
# Protocol Constants
# ============================================================================
class Register(IntEnum):
    """Memory Control Table Register Addresses"""
    # System Information (0x00-0x0F)
    SYS_ID = 0x01
    SYS_MODEL_TYPE = 0x02
    SYS_FW_VERSION = 0x03
    SYS_ERROR = 0x04
    SYS_VOLTAGE = 0x05
    SYS_TEMP = 0x06
    SYS_REDU_RATIO = 0x07
    SYS_ENABLE_DRIVER = 0x0A
    SYS_ENABLE_ON_POWER = 0x0B
    SYS_SAVE_TO_FLASH = 0x0C
    SYS_SET_ZERO_POS = 0x0E
    SYS_CLEAR_ERROR = 0x0F
    
    # Current Feedback (0x10-0x11)
    CUR_CURRENT_L = 0x10
    CUR_CURRENT_H = 0x11
    
    # Speed Feedback (0x12-0x13)
    CUR_SPEED_L = 0x12
    CUR_SPEED_H = 0x13
    
    # Position Feedback (0x14-0x15)
    CUR_POSITION_L = 0x14
    CUR_POSITION_H = 0x15
    
    # Target Control (0x30-0x37)
    TAG_WORK_MODE = 0x30
    TAG_OPEN_PWM = 0x31
    TAG_CURRENT_L = 0x32
    TAG_CURRENT_H = 0x33
    TAG_SPEED_L = 0x34
    TAG_SPEED_H = 0x35
    TAG_POSITION_L = 0x36
    TAG_POSITION_H = 0x37
    
    # Limit Parameters (0x40-0x47)
    LIT_MAX_CURRENT = 0x40
    LIT_MAX_SPEED = 0x41
    LIT_MAX_ACC = 0x42
    LIT_MAX_DEC = 0x43


class WorkMode(IntEnum):
    """Joint Work Modes"""
    OPEN_LOOP = 0
    CURRENT_MODE = 1
    SPEED_MODE = 2
    POSITION_MODE = 3


class JointModel(IntEnum):
    """Joint Model Types"""
    J14 = 0x02  # Joint 10
    J17 = 0x03  # Joint 30
    J20 = 0x04  # Joint 60
    J25 = 0x05  # Joint 120
    GRIPPER = 0x06
    J3 = 0x07   # Joint 03


# Command Types
CMD_READ = 0x01
CMD_WRITE = 0x02
CMD_ERR = 0xFF

# Response ID offset
RESPONSE_ID_OFFSET = 0x100
WRITE_SUCCESS = 0x01


# ============================================================================
# Data Classes
# ============================================================================
@dataclass
class JointState:
    """Joint motor state"""
    motor_id: int
    position_deg: float      # Position in degrees
    speed_rpm: float         # Speed in RPM
    current_ma: int          # Current in mA
    voltage_v: float         # Voltage in volts
    temperature_c: float     # Temperature in Celsius
    error_code: int          # Error bitmap
    is_enabled: bool         # Driver enabled
    work_mode: WorkMode      # Current work mode
    
    @property
    def has_error(self) -> bool:
        return self.error_code != 0


@dataclass
class MotorInfo:
    """Motor information"""
    motor_id: int
    model: JointModel
    firmware_version: str
    reduction_ratio: int
    
    @property
    def model_name(self) -> str:
        names = {
            JointModel.J3: "J3 (Joint 03)",
            JointModel.J14: "J14 (Joint 10)",
            JointModel.J17: "J17 (Joint 30)",
            JointModel.J20: "J20 (Joint 60)",
            JointModel.J25: "J25 (Joint 120)",
            JointModel.GRIPPER: "GRIPPER",
        }
        return names.get(self.model, "UNKNOWN")
    
    @property
    def current_scale(self) -> int:
        """Get current scale for this motor model"""
        if self.model in [JointModel.J14, JointModel.J17, JointModel.J3]:
            return 1  # 1 mA/LSB
        elif self.model in [JointModel.J20, JointModel.J25]:
            return 2  # 2 mA/LSB
        return 1


# ============================================================================
# Protocol Frame Builder/Parser
# ============================================================================
class WHJProtocol:
    """
    RealMan WHJ Protocol Handler
    
    Builds and parses CAN FD frames according to WHJ communication protocol.
    """
    
    # Unit conversion factors
    POS_SCALE = 0.0001      # 0.0001 degrees per LSB
    TARGET_SPEED_SCALE = 0.002  # 0.002 RPM per LSB
    ACTUAL_SPEED_SCALE = 0.02   # 0.02 RPM per LSB
    VOLTAGE_SCALE = 0.01    # 0.01 V per LSB
    TEMP_SCALE = 0.1        # 0.1 degrees per LSB
    
    @staticmethod
    def build_read_frame(motor_id: int, reg: Register, count: int = 1) -> bytes:
        """
        Build read command frame
        
        Args:
            motor_id: Target motor ID (0x00-0x1E)
            reg: Starting register address
            count: Number of registers to read
        
        Returns:
            Frame data bytes
        """
        return bytes([CMD_READ, reg, count])
    
    @staticmethod
    def build_write_frame(motor_id: int, reg: Register, value: int) -> bytes:
        """
        Build write command frame (16-bit value)
        
        Args:
            motor_id: Target motor ID
            reg: Register address
            value: 16-bit value to write
        
        Returns:
            Frame data bytes
        """
        return bytes([CMD_WRITE, reg, value & 0xFF, (value >> 8) & 0xFF])
    
    @staticmethod
    def build_write_32bit(motor_id: int, low_reg: Register, value: int) -> List[bytes]:
        """
        Build write command for 32-bit value (returns two frames)
        
        Args:
            motor_id: Target motor ID
            low_reg: Low 16-bit register address
            value: 32-bit value
        
        Returns:
            List of two frame data bytes (low and high)
        """
        low = value & 0xFFFF
        high = (value >> 16) & 0xFFFF
        high_reg = low_reg + 1
        
        return [
            WHJProtocol.build_write_frame(motor_id, low_reg, low),
            WHJProtocol.build_write_frame(motor_id, high_reg, high)
        ]
    
    @staticmethod
    def parse_read_response(data: bytes, expected_reg: Register) -> List[int]:
        """
        Parse read response data
        
        Args:
            data: Response data bytes
            expected_reg: Expected register address
        
        Returns:
            List of register values
        """
        if len(data) < 3:
            raise ValueError("Response too short")
        
        if data[0] != CMD_READ:
            raise ValueError(f"Invalid response command: {data[0]:02X}")
        
        if data[1] != expected_reg:
            raise ValueError(f"Register mismatch: expected {expected_reg:02X}, got {data[1]:02X}")
        
        # Parse values (little-endian, 2 bytes each)
        values = []
        for i in range(2, len(data), 2):
            if i + 1 < len(data):
                val = data[i] | (data[i + 1] << 8)
                values.append(val)
        
        return values
    
    @staticmethod
    def parse_write_response(data: bytes) -> bool:
        """
        Parse write response
        
        Args:
            data: Response data bytes
        
        Returns:
            True if write successful
        """
        if len(data) < 3:
            return False
        
        if data[0] != CMD_WRITE:
            return False
        
        return data[2] == WRITE_SUCCESS
    
    # ========================================================================
    # High-level Commands
    # ========================================================================
    
    @staticmethod
    def build_enable_motor(motor_id: int, enable: bool) -> bytes:
        """Build enable/disable motor command"""
        return WHJProtocol.build_write_frame(motor_id, Register.SYS_ENABLE_DRIVER, 1 if enable else 0)
    
    @staticmethod
    def build_clear_error(motor_id: int) -> bytes:
        """Build clear error command"""
        return WHJProtocol.build_write_frame(motor_id, Register.SYS_CLEAR_ERROR, 1)
    
    @staticmethod
    def build_set_zero_position(motor_id: int) -> bytes:
        """Build set current position as zero command (for encoder recovery)"""
        return WHJProtocol.build_write_frame(motor_id, Register.SYS_SET_ZERO_POS, 1)
    
    @staticmethod
    def build_set_work_mode(motor_id: int, mode: WorkMode) -> bytes:
        """Build set work mode command"""
        return WHJProtocol.build_write_frame(motor_id, Register.TAG_WORK_MODE, mode)
    
    @staticmethod
    def build_set_target_position(motor_id: int, position_deg: float) -> List[bytes]:
        """
        Build set target position command
        
        Args:
            motor_id: Target motor ID
            position_deg: Target position in degrees
        
        Returns:
            List of two frame data bytes
        """
        # Convert degrees to protocol units (0.0001 degrees per LSB)
        pos_units = int(position_deg / WHJProtocol.POS_SCALE)
        return WHJProtocol.build_write_32bit(motor_id, Register.TAG_POSITION_L, pos_units)
    
    @staticmethod
    def build_set_target_speed(motor_id: int, speed_rpm: float) -> List[bytes]:
        """
        Build set target speed command
        
        Args:
            motor_id: Target motor ID
            speed_rpm: Target speed in RPM
        
        Returns:
            List of two frame data bytes
        """
        # Convert RPM to protocol units (0.002 RPM per LSB)
        speed_units = int(speed_rpm / WHJProtocol.TARGET_SPEED_SCALE)
        return WHJProtocol.build_write_32bit(motor_id, Register.TAG_SPEED_L, speed_units)
    
    @staticmethod
    def build_set_target_current(motor_id: int, current_ma: int) -> List[bytes]:
        """
        Build set target current command
        
        Args:
            motor_id: Target motor ID
            current_ma: Target current in mA
        
        Returns:
            List of two frame data bytes
        """
        return WHJProtocol.build_write_32bit(motor_id, Register.TAG_CURRENT_L, current_ma)
    
    @staticmethod
    def build_read_state(motor_id: int) -> bytes:
        """Build read current state command (reads all feedback registers)"""
        # Read: CUR_CURRENT_L to CUR_POSITION_H (6 registers)
        return WHJProtocol.build_read_frame(motor_id, Register.CUR_CURRENT_L, 6)
    
    @staticmethod
    def build_read_system_info(motor_id: int) -> bytes:
        """Build read system info command"""
        # Read: SYS_MODEL_TYPE to SYS_TEMP (5 registers)
        return WHJProtocol.build_read_frame(motor_id, Register.SYS_MODEL_TYPE, 6)
    
    # ========================================================================
    # State Parsing
    # ========================================================================
    
    @staticmethod
    def parse_state_response(motor_id: int, data: bytes) -> JointState:
        """
        Parse joint state from response
        
        Args:
            motor_id: Motor ID
            data: Response data bytes
        
        Returns:
            JointState object
        """
        values = WHJProtocol.parse_read_response(data, Register.CUR_CURRENT_L)
        
        if len(values) < 6:
            raise ValueError(f"Insufficient data: expected 6 registers, got {len(values)}")
        
        # Extract values (all signed 32-bit)
        current_raw = WHJProtocol._to_int32(values[0], values[1])
        speed_raw = WHJProtocol._to_int32(values[2], values[3])
        position_raw = WHJProtocol._to_int32(values[4], values[5])
        
        return JointState(
            motor_id=motor_id,
            position_deg=position_raw * WHJProtocol.POS_SCALE,
            speed_rpm=speed_raw * WHJProtocol.ACTUAL_SPEED_SCALE,
            current_ma=current_raw,
            voltage_v=0.0,  # Need separate read
            temperature_c=0.0,  # Need separate read
            error_code=0,  # Need separate read
            is_enabled=False,  # Need separate read
            work_mode=WorkMode.POSITION_MODE
        )
    
    @staticmethod
    def parse_system_info(motor_id: int, data: bytes) -> Tuple[JointModel, str, int]:
        """
        Parse system info from response
        
        Returns:
            Tuple of (model, firmware_version, reduction_ratio)
        """
        values = WHJProtocol.parse_read_response(data, Register.SYS_MODEL_TYPE)
        
        if len(values) < 6:
            raise ValueError(f"Insufficient data: expected 6 registers, got {len(values)}")
        
        model = JointModel(values[0]) if values[0] in [m.value for m in JointModel] else JointModel.J14
        fw_version = f"{values[1] >> 8}.{values[1] & 0xFF}"
        redu_ratio = values[4]
        
        return model, fw_version, redu_ratio
    
    @staticmethod
    def _to_int32(low: int, high: int) -> int:
        """Convert two 16-bit values to signed 32-bit"""
        val = (high << 16) | low
        if val & 0x80000000:
            val -= 0x100000000
        return val


# ============================================================================
# Error Code Parser
# ============================================================================
class ErrorCode:
    """Error code bit definitions"""
    FOC_FREQ_HIGH = 0x0001
    OVER_VOLTAGE = 0x0002
    UNDER_VOLTAGE = 0x0004
    OVER_TEMPERATURE = 0x0008
    STARTUP_FAILED = 0x0010
    ENCODER_ERROR = 0x0020
    OVER_CURRENT = 0x0040
    SOFTWARE_ERROR = 0x0080
    TEMP_SENSOR_ERROR = 0x0100
    POSITION_OUT_OF_RANGE = 0x0200
    INVALID_ID = 0x0400
    POSITION_TRACK_ERROR = 0x0800
    CURRENT_SENSOR_ERROR = 0x1000
    BRAKE_FAILED = 0x2000
    POSITION_STEP_ERROR = 0x4000
    MULTI_TURN_LOST = 0x8000
    
    @staticmethod
    def parse(error_code: int) -> List[str]:
        """Parse error code to list of error descriptions"""
        if error_code == 0:
            return ["No error"]
        
        errors = []
        error_map = {
            ErrorCode.FOC_FREQ_HIGH: "FOC frequency too high",
            ErrorCode.OVER_VOLTAGE: "Over-voltage",
            ErrorCode.UNDER_VOLTAGE: "Under-voltage",
            ErrorCode.OVER_TEMPERATURE: "Over-temperature",
            ErrorCode.STARTUP_FAILED: "Startup failed",
            ErrorCode.ENCODER_ERROR: "Encoder error",
            ErrorCode.OVER_CURRENT: "Over-current",
            ErrorCode.SOFTWARE_ERROR: "Software/hardware mismatch",
            ErrorCode.TEMP_SENSOR_ERROR: "Temperature sensor error",
            ErrorCode.POSITION_OUT_OF_RANGE: "Position out of range",
            ErrorCode.INVALID_ID: "Invalid motor ID",
            ErrorCode.POSITION_TRACK_ERROR: "Position tracking error",
            ErrorCode.CURRENT_SENSOR_ERROR: "Current sensor error",
            ErrorCode.BRAKE_FAILED: "Brake failed",
            ErrorCode.POSITION_STEP_ERROR: "Position step too large (>10°)",
            ErrorCode.MULTI_TURN_LOST: "Multi-turn counter lost",
        }
        
        for code, desc in error_map.items():
            if error_code & code:
                errors.append(desc)
        
        return errors


# ============================================================================
# Simple test
# ============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("WHJ Protocol Test")
    print("=" * 60)
    
    # Test frame building
    print("\n[Test] Building frames:")
    
    # Read command
    read_cmd = WHJProtocol.build_read_frame(1, Register.CUR_POSITION_L, 2)
    print(f"  Read position: {read_cmd.hex()}")
    
    # Write command
    write_cmd = WHJProtocol.build_write_frame(1, Register.SYS_ENABLE_DRIVER, 1)
    print(f"  Enable motor: {write_cmd.hex()}")
    
    # Position command
    pos_cmds = WHJProtocol.build_set_target_position(1, 90.0)
    print(f"  Set position 90°: {[cmd.hex() for cmd in pos_cmds]}")
    
    # Test parsing
    print("\n[Test] Parsing responses:")
    
    # Simulate state response
    # Response format: CMD(1B) + INDEX(1B) + DATA(N*2B)
    # Example: current=100mA, speed=500, position=900000 (90 degrees)
    test_response = bytes([
        CMD_READ, Register.CUR_CURRENT_L,
        0x64, 0x00,      # current = 100
        0x00, 0x00,      # current high = 0
        0xC4, 0x09,      # speed = 2500 (50 RPM * 0.02)
        0x00, 0x00,      # speed high = 0
        0xA0, 0xBB,      # position low = 48000
        0x0D, 0x00,      # position high = 13 (900000 total = 90 degrees)
    ])
    
    try:
        state = WHJProtocol.parse_state_response(1, test_response)
        print(f"  Parsed state:")
        print(f"    Position: {state.position_deg:.4f}°")
        print(f"    Speed: {state.speed_rpm:.2f} RPM")
        print(f"    Current: {state.current_ma} mA")
    except Exception as e:
        print(f"  Parse error: {e}")
    
    # Test error parsing
    print("\n[Test] Error codes:")
    test_errors = [
        0x0000,
        0x0002,  # Over-voltage
        0x0042,  # Over-current + Over-voltage
        0x8000,  # Multi-turn lost
    ]
    for err in test_errors:
        errors = ErrorCode.parse(err)
        print(f"  0x{err:04X}: {', '.join(errors)}")
    
    print("\n[Test] Done!")
