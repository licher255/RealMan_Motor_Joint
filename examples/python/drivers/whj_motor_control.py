"""
RealMan WHJ Motor Control
Complete motor control with initialization sequence
"""

import sys
import os
# 添加父目录到路径，以便导入 core 模块
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import time
from core import ZlgCanDriver, ZCANDeviceType
from core.protocol import WHJProtocol, Register, WorkMode, ErrorCode


# Unit conversion constants
MM_PER_DEGREE = 0.018  # 1000° = 18.0mm
DEGREE_PER_MM = 1000.0 / 18.0


def parse_32bit_value(low, high):
    """Parse two 16-bit values to signed 32-bit"""
    val = (high << 16) | low
    if val & 0x80000000:
        val -= 0x100000000
    return val


class WHJMotorController:
    """WHJ Motor Controller with proper initialization
    
    Unit conventions:
    - Position: [°] (degrees) or [mm] (millimeters)
    - Speed: [rpm] (revolutions per minute)
    - Current: [mA] (milliamperes)
    """
    
    # 类级别设置：是否启用 CAN FD 的 Bitrate Switching
    # 注意：BRS=False 会导致某些设备发送失败，必须保持 True
    USE_BRS = True
    
    def __init__(self, driver, motor_id, filter_canfd_only=False):
        """
        Initialize motor controller
        
        Args:
            driver: ZlgCanDriver instance
            motor_id: Motor CAN ID
            filter_canfd_only: If True, only accept CAN FD frames (filters out standard CAN/Kinco traffic)
        """
        self.driver = driver
        self.motor_id = motor_id
        self.response_id = motor_id + 0x100
        self.filter_canfd_only = filter_canfd_only
        
        if filter_canfd_only:
            print(f"[MotorController] CAN FD filter enabled - will ignore standard CAN frames")
    
    def send_command(self, data, timeout_ms=1500, retry_count=5):
        """Send command and wait for response with retry
        
        注意: 在多设备CAN总线上，Kinco会定期发送大量数据，需要：
        1. 更长的超时时间（1500ms）
        2. 更多重试次数（5次）
        3. 更快的轮询（0.5ms）
        4. 清空旧数据再发送
        
        如果 filter_canfd_only=True，将只接收CAN FD帧，自动过滤Kinco的标准CAN帧
        """
        # 根据过滤设置选择接收模式
        recv_type = "CANFD" if self.filter_canfd_only else "any"
        
        for attempt in range(retry_count):
            # 清空旧数据，避免处理之前的堆积帧
            # 注意：即使filter_canfd_only=True，清空时也清空所有类型，避免积压
            old_frames = []
            while True:
                frame = self.driver.receive_frame(timeout_ms=0, frame_type="any")
                if frame is None:
                    break
                old_frames.append(frame)
            
            # Send command
            if not self.driver.send(can_id=self.motor_id, data=data, bitrate_switch=self.USE_BRS):
                if attempt < retry_count - 1:
                    time.sleep(0.05 * (attempt + 1))
                    continue
                return None, "Send failed"
            
            # Wait for response with high-frequency polling
            start = time.time()
            checked_frames = 0
            
            while (time.time() - start) * 1000 < timeout_ms:
                # 直接接收，不先检查计数（更快）
                # 如果filter_canfd_only=True，只接收CAN FD帧，过滤掉Kinco的标准CAN帧
                frame = self.driver.receive_frame(timeout_ms=0, frame_type=recv_type)
                if frame:
                    checked_frames += 1
                    # 只接受来自目标电机的响应
                    if frame.can_id == self.response_id:
                        return frame.data, None
                    # 如果积累了太多无关帧，提前结束（总线太忙）
                    if checked_frames > 200:  # 收到200帧还没找到，重试
                        break
                else:
                    # 无数据时短暂休眠
                    time.sleep(0.0005)  # 0.5ms
            
            # 超时重试
            if attempt < retry_count - 1:
                wait_time = 0.05 + 0.05 * attempt
                time.sleep(wait_time)
        
        return None, "Timeout"
    
    def iap_handshake(self, timeout_ms: int = 1000, max_retries: int = 3) -> bool:
        """
        IAP握手 - 必须在使能电机前完成
        
        注意：即使握手超时报告失败，电机可能实际上已经使能成功。
        这是因为某些固件版本的响应格式可能不同。
        
        在多设备CAN总线上，需要处理Kinco的干扰数据
        如果 filter_canfd_only=True，将自动过滤Kinco的标准CAN帧
        """
        iap_cmd = bytes([0x02, 0x49, 0x00])
        expected_response_id = self.motor_id + 0x100
        
        # 根据过滤设置选择接收模式
        recv_type = "CANFD" if self.filter_canfd_only else "any"
        
        for attempt in range(max_retries):
            # 清空旧数据
            while self.driver.receive_frame(timeout_ms=0, frame_type="any"):
                pass
            
            # 发送
            if not self.driver.send(can_id=self.motor_id, data=iap_cmd, bitrate_switch=self.USE_BRS):
                time.sleep(0.05 * (attempt + 1))
                continue
            
            # 等待响应
            start = time.time()
            checked = 0
            
            while (time.time() - start) * 1000 < timeout_ms:
                frame = self.driver.receive_frame(timeout_ms=0, frame_type=recv_type)
                if frame:
                    checked += 1
                    if frame.can_id == expected_response_id:
                        # 放宽检查：只要CAN ID正确且数据以0x02开头即认为成功
                        if len(frame.data) >= 1 and frame.data[0] == 0x02:
                            return True
                    if checked > 100:
                        break
                else:
                    time.sleep(0.001)
            
            if attempt < max_retries - 1:
                time.sleep(0.05 * (attempt + 1))
        
        return False
    
    def initialize(self):
        """Initialize motor communication"""
        print(f"[Init] Initializing motor {self.motor_id}...")
        
        # IAP握手（某些固件版本可能不响应，但电机仍可使能）
        if not self.iap_handshake():
            print("[Init] IAP handshake timeout, but motor may still be enabled")
        
        # 查询固件版本确认通信
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.SYS_FW_VERSION, 1)
        resp, err = self.send_command(cmd)
        
        if resp:
            print(f"[Init] Motor online! FW: {resp.hex()}")
            return True
        else:
            # 即使ping失败，也返回True让用户可以尝试继续
            print(f"[Init] Cannot confirm motor status, but you can try to continue")
            return True
    
    def get_system_info(self):
        """Get system information"""
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.SYS_MODEL_TYPE, 6)
        resp, err = self.send_command(cmd)
        
        if not resp or len(resp) < 14:
            return None
        
        try:
            model = resp[2] | (resp[3] << 8)
            fw_ver = resp[4] | (resp[5] << 8)
            voltage_raw = resp[6] | (resp[7] << 8)
            temp_raw = resp[8] | (resp[9] << 8)
            redu_ratio = resp[10] | (resp[11] << 8)
            
            model_names = {
                0x02: "J14 (Joint 10)",
                0x03: "J17 (Joint 30)",
                0x04: "J20 (Joint 60)",
                0x05: "J25 (Joint 120)",
                0x06: "Gripper",
                0x07: "J3 (Joint 03)",
            }
            
            return {
                'model': model_names.get(model, f"Unknown (0x{model:02X})"),
                'firmware': f"v{fw_ver >> 8}.{fw_ver & 0xFF}",
                'voltage': voltage_raw * 0.01,
                'temperature': temp_raw * 0.1,
                'reduction_ratio': redu_ratio
            }
        except:
            return None
    
    def get_error_status(self):
        """Get error status"""
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.SYS_ERROR, 1)
        resp, err = self.send_command(cmd)
        
        if not resp or len(resp) < 4:
            return None
        
        error_code = resp[2] | (resp[3] << 8)
        errors = ErrorCode.parse(error_code)
        return error_code, errors
    
    def is_enabled(self):
        """Check if driver is enabled"""
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.SYS_ENABLE_DRIVER, 1)
        resp, err = self.send_command(cmd)
        
        if not resp or len(resp) < 4:
            return None
        
        return resp[2] | (resp[3] << 8) == 1
    
    def get_work_mode(self):
        """Get current work mode"""
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.TAG_WORK_MODE, 1)
        resp, err = self.send_command(cmd)
        
        if not resp or len(resp) < 4:
            return None
        
        mode = resp[2] | (resp[3] << 8)
        modes = {
            0: "OPEN_LOOP",
            1: "CURRENT_MODE",
            2: "SPEED_MODE",
            3: "POSITION_MODE"
        }
        return modes.get(mode, f"UNKNOWN ({mode})")
    
    def get_position(self):
        """Get current position [°]
        
        Returns:
            Current position in degrees [°], or None if failed
        """
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.CUR_POSITION_L, 2)
        resp, err = self.send_command(cmd)
        
        if not resp or len(resp) < 6:
            return None
        
        low = resp[2] | (resp[3] << 8)
        high = resp[4] | (resp[5] << 8)
        raw = parse_32bit_value(low, high)
        return raw * 0.0001
    
    def get_position_mm(self):
        """Get current position [mm]
        
        Returns:
            Current position in millimeters [mm], or None if failed
        """
        pos_deg = self.get_position()
        if pos_deg is None:
            return None
        return pos_deg * MM_PER_DEGREE
    
    def enable(self, enable=True):
        """Enable or disable motor driver"""
        value = 1 if enable else 0
        cmd = WHJProtocol.build_write_frame(self.motor_id, Register.SYS_ENABLE_DRIVER, value)
        resp, err = self.send_command(cmd)
        
        if resp and len(resp) >= 3:
            return resp[2] == 0x01
        return False
    
    def clear_error(self):
        """Clear error flags"""
        cmd = WHJProtocol.build_write_frame(self.motor_id, Register.SYS_CLEAR_ERROR, 1)
        resp, err = self.send_command(cmd)
        return resp is not None
    
    def set_zero_position(self):
        """Set current position as zero"""
        cmd = WHJProtocol.build_write_frame(self.motor_id, Register.SYS_SET_ZERO_POS, 1)
        resp, err = self.send_command(cmd)
        return resp is not None
    
    def save_to_flash(self):
        """Save current configuration to flash"""
        cmd = WHJProtocol.build_write_frame(self.motor_id, Register.SYS_SAVE_TO_FLASH, 1)
        resp, err = self.send_command(cmd)
        return resp is not None
    
    def set_work_mode(self, mode):
        """Set work mode"""
        cmd = WHJProtocol.build_write_frame(self.motor_id, Register.TAG_WORK_MODE, mode)
        resp, err = self.send_command(cmd)
        return resp is not None
    
    def set_target_position(self, position_deg):
        """Set target position [°]
        
        Args:
            position_deg: Target position in degrees [°]
        
        Returns:
            True if successful, False otherwise
        """
        raw = int(position_deg / 0.0001)
        
        # Send low 16 bits
        cmd1 = WHJProtocol.build_write_frame(self.motor_id, Register.TAG_POSITION_L, raw & 0xFFFF)
        resp1, _ = self.send_command(cmd1)
        
        # Send high 16 bits
        cmd2 = WHJProtocol.build_write_frame(self.motor_id, Register.TAG_POSITION_H, (raw >> 16) & 0xFFFF)
        resp2, _ = self.send_command(cmd2)
        
        return resp1 is not None and resp2 is not None
    
    def set_target_position_mm(self, position_mm):
        """Set target position [mm]
        
        Args:
            position_mm: Target position in millimeters [mm]
        
        Returns:
            True if successful, False otherwise
        """
        position_deg = position_mm * DEGREE_PER_MM
        return self.set_target_position(position_deg)


def main():
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    
    print("=" * 60)
    print("RealMan WHJ Motor Control (WHJMotorController)")
    print("=" * 60)
    print(f"Motor ID: {motor_id}")
    print()
    
    driver = None
    motor = None
    
    # Initialize CAN
    try:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
    except RuntimeError as e:
        print(f"[Error] Failed to open CAN device: {e}")
        return
    
    # Create controller
    motor = WHJMotorController(driver, motor_id)
    
    # Initialize
    if not motor.initialize():
        print("\n[Error] Failed to initialize motor")
        driver.close()
        return
    
    print()
    print("-" * 60)
    print("Motor Status")
    print("-" * 60)
    
    # Get system info
    info = motor.get_system_info()
    if info:
        print(f"Model:           {info['model']}")
        print(f"Firmware:        {info['firmware']}")
        print(f"Voltage:         {info['voltage']:.2f} V")
        print(f"Temperature:     {info['temperature']:.1f} °C")
    
    # Get error status
    error = motor.get_error_status()
    if error:
        code, errors = error
        print(f"Error Code:      0x{code:04X}")
        if code == 0:
            print("Status:          OK (No errors)")
        else:
            print("Errors:")
            for e in errors:
                print(f"  - {e}")
    
    # Get enabled status
    enabled = motor.is_enabled()
    if enabled is not None:
        print(f"Driver Enabled:  {'Yes' if enabled else 'No'}")
    
    # Get work mode
    mode = motor.get_work_mode()
    if mode:
        print(f"Work Mode:       {mode}")
    
    # Get position
    pos = motor.get_position()
    if pos is not None:
        print(f"Current Position: {pos:.4f}°")
    
    # Get position in mm
    pos_mm = motor.get_position_mm()
    if pos_mm is not None:
        print(f"Current Position: {pos_mm:.4f} mm")
    
    print("-" * 60)
    print()
    
    # Interactive control
    print("Commands:")
    print("  e    - Enable driver")
    print("  d    - Disable driver")
    print("  c    - Clear errors")
    print("  p    - Go to position [°] (e.g., p 90)")
    print("  m    - Go to position [mm] (e.g., m 10.0)")
    print("  r    - Read current position [°]")
    print("  mm   - Read current position [mm]")
    print("  s    - Show status")
    print("  q    - Quit")
    print()
    
    while True:
        try:
            cmd = input("> ").strip().lower()
            
            if cmd == 'q':
                break
            
            elif cmd == 'e':
                if motor.enable(True):
                    print("Driver enabled")
                else:
                    print("Failed to enable")
            
            elif cmd == 'd':
                if motor.enable(False):
                    print("Driver disabled")
                else:
                    print("Failed to disable")
            
            elif cmd == 'c':
                if motor.clear_error():
                    print("Errors cleared")
                else:
                    print("Failed to clear errors")
            
            elif cmd.startswith('p '):
                try:
                    target = float(cmd.split()[1])
                    if motor.set_target_position(target):
                        print(f"Target position set to {target}°")
                    else:
                        print("Failed to set position")
                except:
                    print("Usage: p <position_in_degrees>")
            
            elif cmd.startswith('m '):
                try:
                    target_mm = float(cmd.split()[1])
                    if motor.set_target_position_mm(target_mm):
                        print(f"Target position set to {target_mm} mm")
                    else:
                        print("Failed to set position")
                except:
                    print("Usage: m <position_in_mm>")
            
            elif cmd == 'r':
                pos = motor.get_position()
                if pos is not None:
                    print(f"Current position: {pos:.4f}°")
                else:
                    print("Failed to read position")
            
            elif cmd == 'mm':
                pos_mm = motor.get_position_mm()
                if pos_mm is not None:
                    print(f"Current position: {pos_mm:.4f} mm")
                else:
                    print("Failed to read position")
            
            elif cmd == 's':
                error = motor.get_error_status()
                if error:
                    code, errors = error
                    print(f"Error Code: 0x{code:04X}")
                enabled = motor.is_enabled()
                if enabled is not None:
                    print(f"Enabled: {'Yes' if enabled else 'No'}")
                pos = motor.get_position()
                if pos is not None:
                    print(f"Position: {pos:.4f}°")
                pos_mm = motor.get_position_mm()
                if pos_mm is not None:
                    print(f"Position: {pos_mm:.4f} mm")
            
            else:
                print("Unknown command")
        
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    # Disable motor before exit
    if motor:
        print("\nDisabling motor...")
        try:
            motor.enable(False)
        except:
            pass
    
    if driver:
        driver.close()
    print("Done!")


if __name__ == "__main__":
    main()
