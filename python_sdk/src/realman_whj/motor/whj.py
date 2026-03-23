"""
RealMan WHJ Motor implementation.
"""

import time
import threading
from typing import Optional, Callable

from .base import BaseMotor, MotorState, MotionProfile
from zlg_can import CANInterface
from ..core.protocol.whj import (
    WHJProtocol, Register, WorkMode, ErrorCode,
    JointState as ProtocolJointState
)
from ..config import WHJMotorConfig, MM_PER_DEGREE, DEGREE_PER_MM, get_config
from ..exceptions import MotorError, TimeoutError


class WHJMotor(BaseMotor):
    """
    RealMan WHJ Motor Controller.
    
    Provides high-level control for WHJ series joint motors with:
    - Thread-safe operation
    - Automatic error handling
    - Motion state tracking
    - Configurable timeouts and retries
    
    Example:
        with WHJMotor(can_interface, motor_id=7) as motor:
            motor.initialize()
            motor.enable()
            motor.move_to(90.0)
            print(f"Position: {motor.get_position()}")
    """
    
    # Unit conversions
    POS_SCALE = 0.0001  # 0.0001 degrees per LSB
    
    def __init__(self, 
                 can_interface: CANInterface,
                 motor_id: int = 1,
                 name: str = "",
                 config: Optional[WHJMotorConfig] = None):
        """
        Initialize WHJ motor.
        
        Args:
            can_interface: CAN interface instance
            motor_id: Motor ID (1-30)
            name: Motor name
            config: Motor configuration
        """
        super().__init__(motor_id, name or f"WHJ_{motor_id}")
        self._can = can_interface
        self._config = config or WHJMotorConfig(motor_id=motor_id)
        self._response_id = motor_id + 0x100
        
        # Thread safety
        self._lock = threading.RLock()
        self._motion_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        
        # State
        self._current_position: Optional[float] = None
        self._target_position: Optional[float] = None
        self._error_code: int = 0
        
        # Callbacks
        self._on_error: Optional[Callable[[int, str], None]] = None
        self._on_position_reached: Optional[Callable[[float], None]] = None
    
    def set_error_callback(self, callback: Callable[[int, str], None]):
        """Set error callback function."""
        self._on_error = callback
    
    def set_position_reached_callback(self, callback: Callable[[float], None]):
        """Set position reached callback function."""
        self._on_position_reached = callback
    
    def _send_command(self, data: bytes, timeout_ms: int = None) -> Optional[bytes]:
        """Send command and wait for response."""
        timeout_ms = timeout_ms or self._config.timeout_ms
        
        # Clear buffer
        self._can.clear_buffer()
        
        # Send frame
        if not self._can.send_canfd(self._motor_id, data):
            return None
        
        # Wait for response
        start_time = time.time()
        while (time.time() - start_time) * 1000 < timeout_ms:
            frame = self._can.receive(10, frame_type="CANFD")
            if frame and frame.can_id == self._response_id:
                return frame.data
            time.sleep(0.001)
        
        return None
    
    def _retry_command(self, data: bytes, timeout_ms: int = None) -> Optional[bytes]:
        """Send command with retry."""
        for attempt in range(self._config.retry_count):
            resp = self._send_command(data, timeout_ms)
            if resp is not None:
                return resp
            time.sleep(0.01 * (attempt + 1))
        return None
    
    def initialize(self) -> bool:
        """
        Initialize motor with full自检流程.
        
        Returns:
            True if successful
        """
        with self._lock:
            self._state = MotorState.INITIALIZING
            
            try:
                # Step 1: IAP handshake
                self._iap_handshake()
                time.sleep(0.1)
                
                # Step 2: Clear buffer and test communication
                for _ in range(3):
                    pos = self.get_position()
                    if pos is not None:
                        break
                    time.sleep(0.05)
                
                # Step 3: Clear errors
                self.clear_error()
                time.sleep(0.05)
                
                # Step 4: Check error status
                error = self._read_error()
                if error is not None and error != 0:
                    errors = ErrorCode.parse(error)
                    if self._on_error:
                        self._on_error(error, f"Initialization warnings: {errors}")
                
                # Step 5: Enable driver
                if not self.enable(True):
                    raise MotorError("Failed to enable motor", self._motor_id)
                
                # Step 6: Set position mode
                self._set_work_mode(WorkMode.POSITION_MODE)
                time.sleep(0.1)
                
                self._state = MotorState.ENABLED
                return True
                
            except Exception as e:
                self._state = MotorState.ERROR
                raise MotorError(f"Initialization failed: {e}", self._motor_id)
    
    def _iap_handshake(self):
        """Perform IAP handshake."""
        iap_cmd = bytes([0x02, 0x49, 0x00])
        for _ in range(3):
            self._can.send_canfd(self._motor_id, iap_cmd)
            time.sleep(0.05)
    
    def enable(self, enabled: bool = True) -> bool:
        """Enable or disable motor."""
        with self._lock:
            cmd = WHJProtocol.build_enable_motor(self._motor_id, enabled)
            resp = self._retry_command(cmd)
            
            if resp and WHJProtocol.parse_write_response(resp):
                self._state = MotorState.ENABLED if enabled else MotorState.DISABLED
                return True
            return False
    
    def get_position(self) -> Optional[float]:
        """Get current position in degrees."""
        with self._lock:
            cmd = WHJProtocol.build_read_frame(self._motor_id, Register.CUR_POSITION_L, 2)
            resp = self._retry_command(cmd, timeout_ms=200)
            
            if resp and len(resp) >= 6:
                low = resp[2] | (resp[3] << 8)
                high = resp[4] | (resp[5] << 8)
                val = (high << 16) | low
                if val & 0x80000000:
                    val -= 0x100000000
                self._current_position = val * self.POS_SCALE
                return self._current_position
            return None
    
    def _get_mm_per_degree(self) -> float:
        """Get mm per degree conversion factor for this motor."""
        if self._config and self._config.mm_per_degree is not None:
            return self._config.mm_per_degree
        return MM_PER_DEGREE
    
    def _get_degree_per_mm(self) -> float:
        """Get degree per mm conversion factor for this motor."""
        if self._config and self._config.degree_per_mm is not None:
            return self._config.degree_per_mm
        return DEGREE_PER_MM
    
    def get_position_mm(self) -> Optional[float]:
        """
        Get current position in millimeters (for linear motion).
        
        Uses the mm_per_degree conversion factor. For WHJ motors with 
        lead screw mechanisms (升降机构), 1000° = 18.0mm by default.
        
        Returns:
            Position in millimeters, or None if failed
        """
        position_deg = self.get_position()
        if position_deg is None:
            return None
        return position_deg * self._get_mm_per_degree()
    
    def move_to_mm(self, position_mm: float, blocking: bool = True) -> bool:
        """
        Move to target position in millimeters (for linear motion).
        
        Converts mm to degrees using degree_per_mm conversion factor.
        For WHJ motors with lead screw mechanisms (升降机构), 
        1mm = 55.5556° by default.
        
        Args:
            position_mm: Target position in millimeters
            blocking: If True, wait for motion to complete
            
        Returns:
            True if command accepted
            
        Example:
            # Move to 10mm position
            motor.move_to_mm(10.0)
            
            # Move relative by 5mm
            current_mm = motor.get_position_mm()
            motor.move_to_mm(current_mm + 5.0)
        """
        position_deg = position_mm * self._get_degree_per_mm()
        return self.move_to(position_deg, blocking=blocking)
    
    def move_relative_mm(self, delta_mm: float, blocking: bool = True) -> bool:
        """
        Move relative distance in millimeters.
        
        Args:
            delta_mm: Relative distance in mm (positive or negative)
            blocking: If True, wait for motion to complete
            
        Returns:
            True if command accepted
            
        Example:
            # Move up by 5mm
            motor.move_relative_mm(5.0)
            
            # Move down by 3mm
            motor.move_relative_mm(-3.0)
        """
        current_mm = self.get_position_mm()
        if current_mm is None:
            return False
        return self.move_to_mm(current_mm + delta_mm, blocking=blocking)
    
    def _set_work_mode(self, mode: WorkMode) -> bool:
        """Set work mode."""
        cmd = WHJProtocol.build_set_work_mode(self._motor_id, mode)
        resp = self._retry_command(cmd)
        return resp is not None
    
    def _read_error(self) -> Optional[int]:
        """Read error code."""
        cmd = WHJProtocol.build_read_frame(self._motor_id, Register.SYS_ERROR, 1)
        resp = self._retry_command(cmd, timeout_ms=200)
        
        if resp and len(resp) >= 4:
            error_code = resp[2] | (resp[3] << 8)
            self._error_code = error_code
            return error_code
        return None
    
    def get_error_code(self) -> Optional[int]:
        """Get current error code."""
        return self._read_error()
    
    def clear_error(self) -> bool:
        """Clear errors."""
        with self._lock:
            cmd = WHJProtocol.build_clear_error(self._motor_id)
            # Send multiple times for reliability
            for _ in range(3):
                self._can.send_canfd(self._motor_id, cmd)
                time.sleep(0.02)
            return True
    
    def move_to(self, position_deg: float, blocking: bool = True) -> bool:
        """
        Move to target position.
        
        Args:
            position_deg: Target position in degrees
            blocking: If True, wait for motion to complete
            
        Returns:
            True if command accepted
        """
        with self._lock:
            if self._state == MotorState.MOVING:
                return False
            
            self._target_position = position_deg
            
            if blocking:
                return self._do_move(position_deg)
            else:
                self._start_motion_thread(position_deg)
                return True
    
    def _do_move(self, position_deg: float) -> bool:
        """Execute move command."""
        cmds = WHJProtocol.build_set_target_position(self._motor_id, position_deg)
        
        # Send low 16 bits
        resp = self._retry_command(cmds[0])
        if not resp:
            return False
        
        time.sleep(0.005)
        
        # Send high 16 bits
        resp = self._retry_command(cmds[1])
        return resp is not None
    
    def _start_motion_thread(self, target_deg: float):
        """Start motion in background thread."""
        self._stop_event.clear()
        self._state = MotorState.MOVING
        
        self._motion_thread = threading.Thread(
            target=self._motion_worker,
            args=(target_deg,),
            daemon=True
        )
        self._motion_thread.start()
    
    def _motion_worker(self, target_deg: float):
        """Motion worker thread."""
        try:
            self._do_move(target_deg)
            
            # Wait for motion to complete (simple polling)
            # In production, should use more sophisticated method
            time.sleep(0.1)
            
            if self._on_position_reached:
                self._on_position_reached(target_deg)
                
        except Exception as e:
            if self._on_error:
                self._on_error(-1, str(e))
        finally:
            self._state = MotorState.ENABLED
    
    def stop(self) -> bool:
        """Stop motion."""
        with self._lock:
            self._stop_event.set()
            
            if self._motion_thread and self._motion_thread.is_alive():
                self._motion_thread.join(timeout=1.0)
            
            self._state = MotorState.ENABLED
            return True
    
    def close(self):
        """Close motor and cleanup."""
        with self._lock:
            self.stop()
            self.enable(False)
            self._state = MotorState.IDLE
    
    def set_zero_position(self) -> bool:
        """Set current position as zero."""
        cmd = WHJProtocol.build_set_zero_position(self._motor_id)
        resp = self._retry_command(cmd)
        return resp is not None
    
    def get_full_state(self) -> Optional[ProtocolJointState]:
        """Get full motor state."""
        cmd = WHJProtocol.build_read_state(self._motor_id)
        resp = self._retry_command(cmd)
        
        if resp:
            try:
                return WHJProtocol.parse_state_response(self._motor_id, resp)
            except Exception:
                pass
        return None
