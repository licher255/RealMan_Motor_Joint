"""
Kinco Motor implementation.
"""

import time
import threading
from enum import Enum, auto
from typing import Optional, Callable

from zlg_can import CANInterface

from .protocol import (
    KincoProtocol, KincoMode, KincoNMTCommand,
    KincoState, KincoConfig
)
from .exceptions import KincoError


class MotorState(Enum):
    """Motor state enumeration."""
    IDLE = auto()
    INITIALIZING = auto()
    ENABLED = auto()
    MOVING = auto()
    ERROR = auto()
    DISABLED = auto()


class KincoMotor:
    """
    Kinco FD1X5 Motor Controller.
    
    Provides high-level control for Kinco rotary servo motors using PDO mode.
    Works in CAN FD mixed mode (standard CAN frames on CAN FD bus).
    
    Example:
        with KincoMotor(can_interface, node_id=1) as motor:
            motor.initialize()
            motor.move_to_degree(90.0, velocity_rpm=50.0)
            print(f"Position: {motor.get_position()}")
    """
    
    # CAN ID definitions
    NMT_ID = 0x000
    RPDO1_ID = 0x201
    RPDO2_ID = 0x301
    TPDO1_ID_BASE = 0x181
    
    def __init__(self,
                 can_interface: CANInterface,
                 node_id: int = 1,
                 name: str = "",
                 config: Optional[KincoConfig] = None):
        """
        Initialize Kinco motor.
        
        Args:
            can_interface: CAN interface instance
            node_id: Node ID (1-127)
            name: Motor name
            config: Motor configuration
        """
        self._motor_id = node_id
        self._name = name or f"Kinco_{node_id}"
        self._can = can_interface
        self._config = config or KincoConfig(node_id=node_id)
        self._tpdo1_id = self.TPDO1_ID_BASE + node_id
        
        # Thread safety
        self._lock = threading.RLock()
        
        # State
        self._state = MotorState.IDLE
        self._kinco_state = KincoState(motor_id=node_id)
        self._current_position: Optional[float] = None
        
        # Callbacks
        self._on_error: Optional[Callable[[int, str], None]] = None
        self._on_position_reached: Optional[Callable[[float], None]] = None
    
    @property
    def motor_id(self) -> int:
        """Get motor ID."""
        return self._motor_id
    
    @property
    def name(self) -> str:
        """Get motor name."""
        return self._name
    
    def set_error_callback(self, callback: Callable[[int, str], None]):
        """Set error callback function."""
        self._on_error = callback
    
    def set_position_reached_callback(self, callback: Callable[[float], None]):
        """Set position reached callback function."""
        self._on_position_reached = callback
    
    def _send_rpdo1(self, control_low: int, control_high: int, mode: int) -> bool:
        """Send RPDO1 frame (control word + mode)."""
        data = KincoProtocol.build_rpdo1_frame(control_low, control_high, mode)
        # Force standard CAN frame for Kinco compatibility
        return self._can.send_can(self.RPDO1_ID, data)
    
    def _send_rpdo2(self, position_inc: int, velocity_units: int) -> bool:
        """Send RPDO2 frame (position + velocity)."""
        import struct
        data = struct.pack('<i', position_inc) + struct.pack('<I', velocity_units)
        # Force standard CAN frame for Kinco compatibility
        return self._can.send_can(self.RPDO2_ID, data)
    
    def _send_nmt(self, command: KincoNMTCommand) -> bool:
        """Send NMT command."""
        data = KincoProtocol.build_nmt_frame(self._motor_id, command)
        return self._can.send_can(self.NMT_ID, data)
    
    def initialize(self) -> bool:
        """
        Initialize motor with full startup sequence.
        
        Sequence:
        1. NMT Start node
        2. Enable + Absolute position mode
        
        Returns:
            True if successful
        """
        with self._lock:
            self._state = MotorState.INITIALIZING
            
            try:
                # Step 1: NMT Start
                if not self._send_nmt(KincoNMTCommand.START):
                    raise KincoError("NMT start failed", self._motor_id)
                time.sleep(0.1)
                
                # Step 2: Enable + Absolute position mode
                if not self.enable(True):
                    raise KincoError("Enable failed", self._motor_id)
                
                self._state = MotorState.ENABLED
                return True
                
            except Exception as e:
                self._state = MotorState.ERROR
                raise KincoError(f"Initialization failed: {e}", self._motor_id)
    
    def enable(self, enabled: bool = True) -> bool:
        """Enable or disable motor."""
        with self._lock:
            if enabled:
                success = self._send_rpdo1(0x01, 0x3F, KincoMode.ABSOLUTE_POSITION)
                if success:
                    self._kinco_state.is_enabled = True
                    self._state = MotorState.ENABLED
                    time.sleep(0.05)
                return success
            else:
                success = self._send_rpdo1(0x01, 0x06, KincoMode.ABSOLUTE_POSITION)
                if success:
                    self._kinco_state.is_enabled = False
                    self._state = MotorState.DISABLED
                return success
    
    def get_position(self) -> Optional[float]:
        """Get current position in degrees."""
        with self._lock:
            # Try to read from TPDO1
            if self._read_state():
                self._current_position = self._kinco_state.position_deg
                return self._current_position
            return self._current_position
    
    def _read_state(self, timeout_ms: int = 50) -> bool:
        """Read state from TPDO1."""
        frame = self._can.receive(timeout_ms=timeout_ms, frame_type="CAN")
        if frame is None:
            return False
        
        if frame.can_id == self._tpdo1_id:
            parsed = KincoProtocol.parse_tpdo1_frame(frame.data)
            if parsed:
                self._kinco_state.position_inc = parsed['position_inc']
                self._kinco_state.position_deg = parsed['position_deg']
                self._kinco_state.speed_rpm = parsed['velocity_rpm']
                return True
        
        return False
    
    def poll_state(self, duration: float = 0.5) -> bool:
        """
        Poll state for a duration.
        
        Args:
            duration: Poll duration in seconds
            
        Returns:
            True if state received
        """
        start = time.time()
        while time.time() - start < duration:
            if self._read_state(timeout_ms=10):
                return True
            time.sleep(0.01)
        return False
    
    def move_to(self, position_deg: float, blocking: bool = True, 
                velocity_rpm: float = None) -> bool:
        """
        Move to target position.
        
        Args:
            position_deg: Target position in degrees
            blocking: If True, wait for completion (approximate)
            velocity_rpm: Velocity in RPM (uses default if None)
            
        Returns:
            True if command accepted
        """
        with self._lock:
            if self._state == MotorState.MOVING:
                return False
            
            vel = velocity_rpm or self._config.default_velocity_rpm
            position_inc = KincoProtocol.degree_to_units(position_deg)
            velocity_units = KincoProtocol.rpm_to_units(vel)
            
            success = self._send_rpdo2(position_inc, velocity_units)
            
            if success:
                self._kinco_state.target_position_deg = position_deg
                self._kinco_state.target_speed_rpm = vel
                self._state = MotorState.MOVING if blocking else MotorState.ENABLED
                
                if blocking:
                    # Approximate wait time
                    current_pos = self.get_position() or 0
                    distance = abs(position_deg - current_pos)
                    wait_time = (distance / vel) * 60 + 0.5  # Add margin
                    time.sleep(wait_time)
                    self._state = MotorState.ENABLED
                    
                    if self._on_position_reached:
                        self._on_position_reached(position_deg)
            
            return success
    
    def move_to_degree(self, degree: float, velocity_rpm: float = 50.0) -> bool:
        """
        Move to target angle.
        
        Args:
            degree: Target angle in degrees
            velocity_rpm: Speed in RPM
            
        Returns:
            True if successful
        """
        return self.move_to(degree, blocking=True, velocity_rpm=velocity_rpm)
    
    def move_relative(self, delta_deg: float, velocity_rpm: float = 50.0) -> bool:
        """
        Move relative to current position.
        
        Args:
            delta_deg: Relative distance in degrees
            velocity_rpm: Speed in RPM
            
        Returns:
            True if successful
        """
        with self._lock:
            # Switch to relative mode temporarily
            self._send_rpdo1(0x01, 0x3F, KincoMode.RELATIVE_POSITION)
            time.sleep(0.05)
            
            # Send relative move
            position_inc = KincoProtocol.degree_to_units(delta_deg)
            velocity_units = KincoProtocol.rpm_to_units(velocity_rpm)
            success = self._send_rpdo2(position_inc, velocity_units)
            
            # Switch back to absolute mode
            time.sleep(0.1)
            self._send_rpdo1(0x01, 0x3F, KincoMode.ABSOLUTE_POSITION)
            
            return success
    
    def home(self, velocity_rpm: float = 30.0) -> bool:
        """Move to home position (0 degrees)."""
        return self.move_to_degree(0.0, velocity_rpm)
    
    def set_origin(self) -> bool:
        """
        Set current position as origin.
        
        Sequence:
        1. Mode 6, control word 0F
        2. Control word 1F
        3. Back to absolute position mode
        """
        with self._lock:
            # Step 1: Mode 6, control word 0F
            if not self._send_rpdo1(0x06, 0x0F, 0x00):
                return False
            time.sleep(0.1)
            
            # Step 2: Control word 1F
            if not self._send_rpdo1(0x06, 0x1F, 0x00):
                return False
            time.sleep(0.1)
            
            # Step 3: Back to absolute position mode
            return self.enable(True)
    
    def stop(self) -> bool:
        """Stop motion."""
        with self._lock:
            # Read current position and set as target
            current_pos = self.get_position()
            if current_pos is not None:
                position_inc = KincoProtocol.degree_to_units(current_pos)
                return self._send_rpdo2(position_inc, 0)
            return False
    
    def clear_error(self) -> bool:
        """Clear errors."""
        with self._lock:
            success = self._send_rpdo1(0x01, 0x86, KincoMode.ABSOLUTE_POSITION)
            if success:
                time.sleep(0.1)
            return success
    
    def get_error_code(self) -> Optional[int]:
        """Get error code."""
        # Kinco PDO mode doesn't provide direct error reading
        # Would need SDO mode for full error handling
        return self._kinco_state.error_code if self._kinco_state.error_code != 0 else None
    
    def close(self):
        """Close motor and cleanup."""
        with self._lock:
            self.enable(False)
            self._send_nmt(KincoNMTCommand.STOP)
            time.sleep(0.05)
            self._state = MotorState.IDLE
    
    def shutdown(self) -> bool:
        """
        Safe shutdown sequence.
        
        1. Disable driver
        2. NMT stop node
        """
        with self._lock:
            self.enable(False)
            time.sleep(0.1)
            self._send_nmt(KincoNMTCommand.STOP)
            time.sleep(0.05)
            return True
    
    def get_state(self) -> KincoState:
        """Get current state."""
        return self._kinco_state
    
    def print_state(self):
        """Print current state."""
        print(f"\n[{self._name}] State:")
        print(f"  Position: {self._kinco_state.position_deg:.2f} deg")
        print(f"  Velocity: {self._kinco_state.speed_rpm:.1f} rpm")
        print(f"  Enabled: {self._kinco_state.is_enabled}")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.close()
        return False
