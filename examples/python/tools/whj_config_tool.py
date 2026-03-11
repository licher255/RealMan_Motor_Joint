"""
RealMan WHJ Motor Configuration Tool
WHJ关节电机配置工具

功能:
- 调整原点 / Adjust origin
- 修改编码器最大/最小位置限制 / Modify encoder position limits
- 清除错误 / Clear errors
- 使能/禁用电机 / Enable/disable motor
- 保存当前位置为零点 / Save current position as zero
- 修改驱动器ID / Change motor ID
- 保存数据到Flash / Save configuration to flash
- 移动电机控制 / Move motor: m <position>, mr <delta>

【单位说明 / Unit Conventions】
- ° (度): 电机旋转角度，Flash存储的单位
- rpm: 转速，转/分钟
- mm: 直线位移，向上为正方向。换算关系: 1000° = 18.0mm

Usage:
    python whj_config_tool.py [motor_id]
    
Example:
    python whj_config_tool.py 7

Commands:
    # 角度控制 (°)
    m 23000        - 移动到绝对位置 23000°
    mr +100        - 相对当前位置移动 +100°
    mr -50         - 相对当前位置移动 -50°
    
    # 毫米控制 (mm)
    mm 50          - 移动到绝对位置 50mm
    mmr +10        - 相对当前位置向上移动 10mm
    mmr -5         - 相对当前位置向下移动 5mm
    
    # 其他
    z              - 设置当前位置为零点
    s              - 停止电机
    save           - 保存配置到Flash
"""

import sys
import os
# 添加父目录到路径
_current_dir = os.path.dirname(os.path.abspath(__file__))
_project_root = os.path.join(_current_dir, '..')
sys.path.insert(0, os.path.abspath(_project_root))

import time
import math
from typing import Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

from core import ZlgCanDriver, ZCANDeviceType
from core.protocol import WHJProtocol, Register, WorkMode, ErrorCode


class MoveMode(Enum):
    """移动模式"""
    IDLE = "idle"
    POSITION = "position"
    SPEED = "speed"


def parse_32bit_value(low: int, high: int) -> int:
    """将两个16位值转换为有符号32位整数"""
    val = (high << 16) | low
    if val & 0x80000000:
        val -= 0x100000000
    return val


class WHJConfigTool:
    """WHJ电机配置工具"""
    
    # 编码器位置限制寄存器 (根据WHJ协议: 0x44-0x47)
    LIT_MIN_POSITION_L = 0x44  # 最小位置低16位
    LIT_MIN_POSITION_H = 0x45  # 最小位置高16位
    LIT_MAX_POSITION_L = 0x46  # 最大位置低16位
    LIT_MAX_POSITION_H = 0x47  # 最大位置高16位
    
    # ========== 单位转换常量 / Unit Conversion Constants ==========
    # 内部存储和协议使用的单位
    POS_SCALE = 0.0001  # [°/LSB] 协议位置比例因子: 0.0001度/LSB
    
    # 速度单位
    TARGET_SPEED_SCALE = 0.002  # [rpm/LSB] 目标速度比例因子: 0.002 rpm/LSB
    ACTUAL_SPEED_SCALE = 0.02   # [rpm/LSB] 实际速度比例因子: 0.02 rpm/LSB
    
    # 默认移动速度
    DEFAULT_SPEED_RPM = 30.0  # [rpm] 默认移动速度: 30 RPM = 180°/s
    
    # ========== 直线位移转换 / Linear Displacement Conversion ==========
    # 实测: 1000° 旋转 = 18.0mm 向上移动
    # 用于将旋转角度(°)转换为直线位移(mm)
    MM_PER_DEGREE = 0.018  # [mm/°] 18.0mm / 1000° = 0.018 mm/°
    DEGREE_PER_MM = 1000.0 / 18.0  # [°/mm] ≈ 55.5556 °/mm
    
    def __init__(self, can_driver: ZlgCanDriver, motor_id: int = 7):
        self.can_driver = can_driver
        self.motor_id = motor_id
        self.response_id = motor_id + 0x100
        self.filter_canfd_only = True
        self.move_mode = MoveMode.IDLE
        self.current_speed = 0.0
    
    def _send_command(self, data: bytes, timeout_ms: int = 1500, retry_count: int = 5) -> Tuple[Optional[bytes], Optional[str]]:
        """发送命令并等待响应"""
        recv_type = "CANFD" if self.filter_canfd_only else "any"
        
        for attempt in range(retry_count):
            # 清空旧数据
            while True:
                frame = self.can_driver.receive_frame(timeout_ms=0, frame_type="any")
                if frame is None:
                    break
            
            # 发送命令
            if not self.can_driver.send(can_id=self.motor_id, data=data, bitrate_switch=True):
                if attempt < retry_count - 1:
                    time.sleep(0.05 * (attempt + 1))
                    continue
                return None, "Send failed"
            
            # 等待响应
            start = time.time()
            checked_frames = 0
            
            while (time.time() - start) * 1000 < timeout_ms:
                frame = self.can_driver.receive_frame(timeout_ms=0, frame_type=recv_type)
                if frame:
                    checked_frames += 1
                    if frame.can_id == self.response_id:
                        return frame.data, None
                    if checked_frames > 200:
                        break
                else:
                    time.sleep(0.0005)
            
            if attempt < retry_count - 1:
                wait_time = 0.05 + 0.05 * attempt
                time.sleep(wait_time)
        
        return None, "Timeout"
    
    def _read_register(self, reg: Register, count: int = 1) -> Tuple[Optional[List[int]], Optional[str]]:
        """读取寄存器值"""
        cmd = WHJProtocol.build_read_frame(self.motor_id, reg, count)
        resp, err = self._send_command(cmd)
        
        if not resp or len(resp) < 2:
            return None, err or "Invalid response"
        
        if resp[0] != 0x01:  # CMD_READ
            return None, f"Invalid response command: {resp[0]:02X}"
        
        values = []
        for i in range(2, len(resp), 2):
            if i + 1 < len(resp):
                val = resp[i] | (resp[i + 1] << 8)
                values.append(val)
        
        return values, None
    
    def _write_register(self, reg: Register, value: int) -> bool:
        """写入16位寄存器值"""
        cmd = WHJProtocol.build_write_frame(self.motor_id, reg, value)
        resp, err = self._send_command(cmd)
        
        if not resp or len(resp) < 3:
            return False
        
        return resp[0] == 0x02 and resp[2] == 0x01
    
    def _write_32bit_register(self, low_reg: Register, value: int) -> bool:
        """写入32位寄存器值"""
        low = value & 0xFFFF
        high = (value >> 16) & 0xFFFF
        high_reg = low_reg + 1
        
        if not self._write_register(low_reg, low):
            return False
        time.sleep(0.01)
        
        if not self._write_register(high_reg, high):
            return False
        time.sleep(0.01)
        
        return True
    
    def iap_handshake(self, timeout_ms: int = 1000, max_retries: int = 3) -> bool:
        """
        IAP握手 - 必须在使能电机前完成
        
        注意：即使握手超时报告失败，电机可能实际上已经使能成功。
        """
        iap_cmd = bytes([0x02, 0x49, 0x00])
        expected_response_id = self.motor_id + 0x100
        recv_type = "CANFD" if self.filter_canfd_only else "any"
        
        for attempt in range(max_retries):
            # 清空旧数据
            while self.can_driver.receive_frame(timeout_ms=0, frame_type="any"):
                pass
            
            # 发送IAP握手命令
            if not self.can_driver.send(can_id=self.motor_id, data=iap_cmd, bitrate_switch=True):
                time.sleep(0.05 * (attempt + 1))
                continue
            
            # 等待响应
            start = time.time()
            checked = 0
            
            while (time.time() - start) * 1000 < timeout_ms:
                frame = self.can_driver.receive_frame(timeout_ms=0, frame_type=recv_type)
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
    
    def ping(self) -> bool:
        """检查电机是否在线（先IAP握手，再读取版本）"""
        # 先进行IAP握手
        if not self.iap_handshake():
            print("[Warning] IAP握手失败，尝试继续...")
        else:
            print("[OK] IAP握手成功")
        
        # 读取固件版本确认通信
        cmd = WHJProtocol.build_read_frame(self.motor_id, Register.SYS_FW_VERSION, 1)
        resp, err = self._send_command(cmd, timeout_ms=500)
        return resp is not None
    
    def get_system_info(self) -> Optional[dict]:
        """获取系统信息"""
        values, err = self._read_register(Register.SYS_MODEL_TYPE, 6)
        if not values or len(values) < 6:
            return None
        
        model_names = {
            0x02: "J14 (Joint 10)",
            0x03: "J17 (Joint 30)",
            0x04: "J20 (Joint 60)",
            0x05: "J25 (Joint 120)",
            0x06: "Gripper",
            0x07: "J3 (Joint 03)",
        }
        
        return {
            'model': model_names.get(values[0], f"Unknown (0x{values[0]:02X})"),
            'firmware': f"v{values[1] >> 8}.{values[1] & 0xFF}",
            'voltage': values[2] * 0.01,
            'temperature': values[3] * 0.1,
            'reduction_ratio': values[4],
            'motor_id': self.motor_id
        }
    
    def get_error_status(self) -> Tuple[Optional[int], Optional[List[str]]]:
        """获取错误状态"""
        values, err = self._read_register(Register.SYS_ERROR, 1)
        if not values:
            return None, None
        
        error_code = values[0]
        errors = ErrorCode.parse(error_code)
        return error_code, errors
    
    def clear_error(self) -> bool:
        """清除错误"""
        return self._write_register(Register.SYS_CLEAR_ERROR, 1)
    
    def enable_motor(self, enable: bool = True) -> bool:
        """使能/禁用电机（使能前会自动进行IAP握手）"""
        if enable:
            # 使能前先进行IAP握手
            if not self.iap_handshake():
                print("[Warning] IAP握手失败，继续尝试使能...")
            time.sleep(0.05)
        
        return self._write_register(Register.SYS_ENABLE_DRIVER, 1 if enable else 0)
    
    def is_enabled(self) -> Optional[bool]:
        """检查电机是否使能"""
        values, err = self._read_register(Register.SYS_ENABLE_DRIVER, 1)
        if not values:
            return None
        return values[0] == 1
    
    def get_current_position(self) -> Optional[float]:
        """
        获取当前位置 [°]
        
        Returns:
            当前角度位置 [度]
        """
        values, err = self._read_register(Register.CUR_POSITION_L, 2)
        if not values or len(values) < 2:
            return None
        
        raw = parse_32bit_value(values[0], values[1])
        return raw * self.POS_SCALE  # [°]
    
    def get_current_position_mm(self) -> Optional[float]:
        """
        获取当前位置 [mm]
        
        Returns:
            当前直线位移 [mm]，向上为正方向
            基于换算: 1000° = 18.0mm
        """
        pos_deg = self.get_current_position()
        if pos_deg is None:
            return None
        return pos_deg * self.MM_PER_DEGREE  # [mm]
    
    def get_current_speed(self) -> Optional[float]:
        """
        获取当前速度 [rpm]
        
        Returns:
            当前转速 [转/分钟]
        """
        values, err = self._read_register(Register.CUR_SPEED_L, 2)
        if not values or len(values) < 2:
            return None
        
        raw = parse_32bit_value(values[0], values[1])
        return raw * self.ACTUAL_SPEED_SCALE  # [rpm]
    
    def set_zero_position(self) -> bool:
        """设置当前位置为零点"""
        return self._write_register(Register.SYS_SET_ZERO_POS, 1)
    
    def save_to_flash(self) -> bool:
        """保存配置到Flash"""
        return self._write_register(Register.SYS_SAVE_TO_FLASH, 1)
    
    def get_encoder_limits(self) -> Optional[dict]:
        """获取编码器位置限制"""
        min_values, err = self._read_register(self.LIT_MIN_POSITION_L, 2)
        if not min_values or len(min_values) < 2:
            return None
        min_raw = parse_32bit_value(min_values[0], min_values[1])
        
        max_values, err = self._read_register(self.LIT_MAX_POSITION_L, 2)
        if not max_values or len(max_values) < 2:
            return None
        max_raw = parse_32bit_value(max_values[0], max_values[1])
        
        return {
            'min_deg': min_raw * self.POS_SCALE,
            'max_deg': max_raw * self.POS_SCALE,
            'min_raw': min_raw,
            'max_raw': max_raw
        }
    
    def set_encoder_limits(self, min_deg: Optional[float] = None, max_deg: Optional[float] = None) -> bool:
        """设置编码器位置限制"""
        current = self.get_encoder_limits()
        if not current:
            print("[Error] 无法读取当前限制值")
            return False
        
        success = True
        
        if min_deg is not None:
            min_raw = int(min_deg / self.POS_SCALE)
            print(f"[Config] 设置最小位置: {min_deg}° (raw: {min_raw})")
            if not self._write_32bit_register(self.LIT_MIN_POSITION_L, min_raw):
                print("[Error] 设置最小位置失败")
                success = False
            time.sleep(0.05)
        
        if max_deg is not None:
            max_raw = int(max_deg / self.POS_SCALE)
            print(f"[Config] 设置最大位置: {max_deg}° (raw: {max_raw})")
            if not self._write_32bit_register(self.LIT_MAX_POSITION_L, max_raw):
                print("[Error] 设置最大位置失败")
                success = False
            time.sleep(0.05)
        
        return success
    
    def set_motor_id(self, new_id: int) -> bool:
        """修改电机ID"""
        if not 1 <= new_id <= 30:
            print(f"[Error] 电机ID必须在1-30范围内: {new_id}")
            return False
        
        print(f"[Config] 修改电机ID: {self.motor_id} -> {new_id}")
        
        if not self._write_register(Register.SYS_ID, new_id):
            print("[Error] 修改ID失败")
            return False
        
        self.motor_id = new_id
        self.response_id = new_id + 0x100
        
        print(f"[Config] ID修改成功，新ID: {new_id}")
        print("[Warning] 请保存到Flash以永久生效！")
        return True
    
    def adjust_origin(self, offset_deg: float) -> bool:
        """调整原点位置"""
        current_pos = self.get_current_position()
        if current_pos is None:
            print("[Error] 无法获取当前位置")
            return False
        
        print(f"[Origin] 当前位置: {current_pos:.4f}°")
        print(f"[Origin] 偏移量: {offset_deg:.4f}°")
        
        if not self.set_zero_position():
            print("[Error] 设置零点失败")
            return False
        
        print("[Origin] 零点设置成功！")
        print("[Warning] 请保存到Flash以永久生效！")
        return True
    
    def allow_movement(self) -> bool:
        """允许移动（IAP握手+清除错误并使能）"""
        print("[Move] 准备允许移动...")
        
        # 1. IAP握手（必须先完成）
        print("[Move] IAP握手...")
        if not self.iap_handshake():
            print("[Warning] IAP握手失败，继续尝试...")
        else:
            print("[OK] IAP握手成功")
        time.sleep(0.1)
        
        # 2. 清除错误
        if not self.clear_error():
            print("[Warning] 清除错误可能失败，继续尝试...")
        time.sleep(0.1)
        
        if not self.enable_motor(True):
            print("[Error] 使能失败")
            return False
        time.sleep(0.1)
        
        if not self._write_register(Register.TAG_WORK_MODE, WorkMode.POSITION_MODE):
            print("[Warning] 设置位置模式可能失败")
        time.sleep(0.1)
        
        enabled = self.is_enabled()
        error_code, errors = self.get_error_status()
        
        print(f"[Move] 电机状态: {'已使能' if enabled else '未使能'}")
        if error_code is not None and error_code != 0:
            print(f"[Move] 仍有错误: 0x{error_code:04X}")
            for e in errors:
                print(f"         - {e}")
            return False
        
        print("[Move] 电机已准备好移动！")
        return True
    
    # ========================================================================
    # 电机移动控制
    # ========================================================================
    
    def set_target_position(self, position_deg: float) -> bool:
        """设置目标位置"""
        pos_raw = int(position_deg / self.POS_SCALE)
        low = pos_raw & 0xFFFF
        high = (pos_raw >> 16) & 0xFFFF
        
        if not self._write_register(Register.TAG_POSITION_L, low):
            return False
        time.sleep(0.01)
        
        if not self._write_register(Register.TAG_POSITION_H, high):
            return False
        
        return True
    
    def set_target_speed(self, speed_rpm: float) -> bool:
        """设置目标速度"""
        speed_raw = int(speed_rpm / 0.002)
        low = speed_raw & 0xFFFF
        high = (speed_raw >> 16) & 0xFFFF
        
        if not self._write_register(Register.TAG_SPEED_L, low):
            return False
        time.sleep(0.01)
        
        if not self._write_register(Register.TAG_SPEED_H, high):
            return False
        
        return True
    
    def move_to_position(self, target_deg: float, speed_rpm: float = None, 
                         wait: bool = True, timeout: float = None) -> bool:
        """移动到指定位置"""
        current = self.get_current_position()
        if current is None:
            print("[Error] 无法获取当前位置")
            return False
        
        distance = abs(target_deg - current)
        if distance < 0.01:
            print(f"[Move] 已到达目标位置 {target_deg:.2f}°")
            return True
        
        # 确保电机使能（使能前会自动IAP握手）
        enabled = self.is_enabled()
        if not enabled:
            print("[Move] 电机未使能，正在IAP握手并使能...")
            if not self.enable_motor(True):
                print("[Error] 使能失败")
                return False
            time.sleep(0.1)
        
        # 设置速度限制
        if speed_rpm is not None:
            speed_raw = int(speed_rpm / 0.002)
            self._write_register(Register.LIT_MAX_SPEED, speed_raw & 0xFFFF)
            time.sleep(0.05)
        
        # 设置为位置模式
        if not self._write_register(Register.TAG_WORK_MODE, WorkMode.POSITION_MODE):
            print("[Error] 设置位置模式失败")
            return False
        time.sleep(0.05)
        
        print(f"[Move] {current:.2f}° -> {target_deg:.2f}° (距离: {distance:.2f}°)")
        
        if not self.set_target_position(target_deg):
            print("[Error] 设置目标位置失败")
            return False
        
        if not wait:
            return True
        
        # 等待到达
        if timeout is None:
            est_time = distance / 180.0 + 2.0
            timeout = max(est_time, 3.0)
        
        print(f"[Move] 等待到达... (超时: {timeout:.1f}s)")
        start = time.time()
        last_print = start
        
        while time.time() - start < timeout:
            pos = self.get_current_position()
            now = time.time()
            
            if pos is not None:
                error = abs(pos - target_deg)
                if now - last_print > 0.5:
                    print(f"  当前: {pos:7.2f}° | 目标: {target_deg:7.2f}° | 误差: {error:5.2f}°")
                    last_print = now
                
                if error < 0.1:
                    print(f"[OK] 已到达: {pos:.2f}° (误差: {error:.3f}°)")
                    return True
            
            time.sleep(0.05)
        
        print("[Warning] 移动超时")
        return False
    
    def move_relative(self, delta_deg: float, speed_rpm: float = None) -> bool:
        """
        相对当前位置移动 [°]
        
        Args:
            delta_deg: 相对移动量 [度]
            speed_rpm: 移动速度 [rpm]
        """
        current = self.get_current_position()
        if current is None:
            print("[Error] 无法获取当前位置")
            return False
        
        target = current + delta_deg
        print(f"[Move] 相对移动: {delta_deg:+.2f}° ({current:.2f}° -> {target:.2f}°)")
        return self.move_to_position(target, speed_rpm)
    
    # ========== 毫米单位控制接口 / mm Control Interface ==========
    
    def move_to_position_mm(self, target_mm: float, speed_rpm: float = None, 
                            wait: bool = True, timeout: float = None) -> bool:
        """
        移动到指定位置 [mm]
        
        Args:
            target_mm: 目标位置 [mm]，向上为正方向
            speed_rpm: 移动速度 [rpm]
            wait: 是否等待完成
            timeout: 超时时间 [s]
        
        Returns:
            True if successful
        """
        # 将 mm 转换为 °
        target_deg = target_mm * self.DEGREE_PER_MM  # [°]
        
        current_mm = self.get_current_position_mm()
        if current_mm is not None:
            print(f"[Move mm] {current_mm:.2f}mm -> {target_mm:.2f}mm ({target_deg:.2f}°)")
        else:
            print(f"[Move mm] -> {target_mm:.2f}mm ({target_deg:.2f}°)")
        
        # 调用角度控制方法
        return self.move_to_position(target_deg, speed_rpm, wait, timeout)
    
    def move_relative_mm(self, delta_mm: float, speed_rpm: float = None) -> bool:
        """
        相对当前位置移动 [mm]
        
        Args:
            delta_mm: 相对移动量 [mm]，正值向上，负值向下
            speed_rpm: 移动速度 [rpm]
        
        Returns:
            True if successful
        """
        current_mm = self.get_current_position_mm()
        if current_mm is None:
            print("[Error] 无法获取当前位置")
            return False
        
        target_mm = current_mm + delta_mm
        print(f"[Move mm] 相对移动: {delta_mm:+.2f}mm ({current_mm:.2f}mm -> {target_mm:.2f}mm)")
        
        # 调用绝对位置移动
        return self.move_to_position_mm(target_mm, speed_rpm)
    
    def stop(self):
        """停止电机（保持当前位置）"""
        current = self.get_current_position()
        if current is not None:
            self._write_register(Register.TAG_WORK_MODE, WorkMode.POSITION_MODE)
            time.sleep(0.05)
            self.set_target_position(current)
            print(f"[Stop] 电机已停止: {current:.2f}°")
        else:
            print("[Error] 无法获取当前位置")


def print_help():
    """打印帮助"""
    print("""
╔══════════════════════════════════════════════════════════════════════╗
║              RealMan WHJ Motor Configuration Tool                     ║
╠══════════════════════════════════════════════════════════════════════╣
║  配置命令                                                            ║
║    i, info          显示电机信息                                     ║
║    st, status       显示错误状态                                     ║
║    c, clear         清除错误                                         ║
║    e, enable        使能电机                                         ║
║    d, disable       禁用电机                                         ║
║    lim, limits      显示编码器位置限制 [°]                           ║
║    setlim           设置编码器位置限制 [°]                           ║
║    z, zero          保存当前位置为零点                               ║
║    id <new_id>      修改电机ID (1-30)                                ║
║    save             保存配置到Flash                                  ║
║    r, ready         准备移动(清除错误+使能)                          ║
╠══════════════════════════════════════════════════════════════════════╣
║  移动控制 (角度 / °)                                                  ║
║    m <角度>         移动到绝对位置 [°] (如: m 23000)                 ║
║    mr <偏移量>      相对移动 [°] (如: mr +100, mr -50)               ║
║    p, pos           读取当前位置 [°]                                 ║
╠══════════════════════════════════════════════════════════════════════╣
║  移动控制 (直线位移 / mm)  换算: 1000° = 18.0mm                      ║
║    mm <位置>        移动到绝对位置 [mm] (如: mm 50)                  ║
║    mmr <偏移量>     相对移动 [mm] (如: mmr +10, mmr -5)              ║
║    pmm              读取当前位置 [mm]                                ║
╠══════════════════════════════════════════════════════════════════════╣
║  其他                                                                ║
║    h, help          显示帮助                                         ║
║    s, stop          停止电机                                         ║
║    q, quit          退出                                             ║
╚══════════════════════════════════════════════════════════════════════╝
""")


def main():
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 7
    
    print("=" * 70)
    print("RealMan WHJ Motor Configuration Tool")
    print("=" * 70)
    print(f"Motor ID: {motor_id}")
    print("输入 'help' 或 'h' 查看命令列表")
    print()
    
    # 初始化CAN
    driver = None
    tool = None
    
    try:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        print("[OK] CAN设备已打开")
    except RuntimeError as e:
        print(f"[Error] 无法打开CAN设备: {e}")
        return
    
    # 创建配置工具
    tool = WHJConfigTool(driver, motor_id)
    
    # 检查电机是否在线
    print("[Info] 检查电机连接...")
    if not tool.ping():
        print("[Warning] 电机无响应，请检查连接")
        response = input("是否继续? (y/n): ").strip().lower()
        if response != 'y':
            driver.close()
            return
    else:
        print("[OK] 电机在线！")
    
    print_help()
    
    # 主循环
    while True:
        try:
            cmd_input = input("> ").strip()
            if not cmd_input:
                continue
            
            parts = cmd_input.split()
            cmd = parts[0].lower()
            args = parts[1:]
            
            # ==================== 退出 ====================
            if cmd in ['q', 'quit', 'exit']:
                break
            
            # ==================== 帮助 ====================
            elif cmd in ['h', 'help', '?']:
                print_help()
            
            # ==================== 信息显示 ====================
            elif cmd in ['i', 'info']:
                info = tool.get_system_info()
                if info:
                    print(f"\n[Motor Info]")
                    print(f"  Model:       {info['model']}")
                    print(f"  Firmware:    {info['firmware']}")
                    print(f"  Voltage:     {info['voltage']:.2f} V")
                    print(f"  Temperature: {info['temperature']:.1f} °C")
                    print(f"  Reduction:   {info['reduction_ratio']}:1")
                    print(f"  Motor ID:    {info['motor_id']}")
                else:
                    print("[Error] 无法获取信息")
            
            elif cmd in ['st', 'status']:
                error_code, errors = tool.get_error_status()
                if error_code is not None:
                    print(f"\n[Error Status] 0x{error_code:04X}")
                    if error_code == 0:
                        print("  Status: OK")
                    else:
                        for e in errors:
                            print(f"  - {e}")
                else:
                    print("[Error] 无法获取错误状态")
            
            elif cmd in ['p', 'pos']:
                # 读取位置 [°]
                pos = tool.get_current_position()
                speed = tool.get_current_speed()
                if pos is not None:
                    status = f"[Position] {pos:.4f}°"
                    if speed is not None:
                        status += f"  [Speed] {speed:.2f} RPM"
                    print(status)
                else:
                    print("[Error] 无法读取位置")
            
            elif cmd == 'pmm':
                # 读取位置 [mm]
                pos_mm = tool.get_current_position_mm()
                speed = tool.get_current_speed()
                if pos_mm is not None:
                    status = f"[Position] {pos_mm:.2f}mm"
                    if speed is not None:
                        status += f"  [Speed] {speed:.2f} RPM"
                    print(status)
                else:
                    print("[Error] 无法读取位置")
            
            elif cmd in ['lim', 'limits']:
                limits = tool.get_encoder_limits()
                if limits:
                    print(f"\n[Encoder Limits]")
                    print(f"  Min: {limits['min_deg']:.2f}° (raw: {limits['min_raw']})")
                    print(f"  Max: {limits['max_deg']:.2f}° (raw: {limits['max_raw']})")
                else:
                    print("[Error] 无法读取位置限制")
            
            # ==================== 控制命令 ====================
            elif cmd in ['c', 'clear']:
                if tool.clear_error():
                    print("[OK] 错误已清除")
                else:
                    print("[Error] 清除失败")
            
            elif cmd in ['e', 'enable']:
                if tool.enable_motor(True):
                    print("[OK] 电机已使能")
                else:
                    print("[Error] 使能失败")
            
            elif cmd in ['d', 'disable']:
                if tool.enable_motor(False):
                    print("[OK] 电机已禁用")
                else:
                    print("[Error] 禁用失败")
            
            elif cmd in ['r', 'ready']:
                tool.allow_movement()
            
            # ==================== 配置命令 ====================
            elif cmd == 'setlim':
                print("\n[Set Limits] 留空表示不修改")
                min_input = input("  最小位置(度): ").strip()
                max_input = input("  最大位置(度): ").strip()
                
                min_deg = float(min_input) if min_input else None
                max_deg = float(max_input) if max_input else None
                
                if min_deg is None and max_deg is None:
                    print("[Info] 无修改")
                    continue
                
                if tool.set_encoder_limits(min_deg, max_deg):
                    print("[OK] 已设置，请保存到Flash！")
                else:
                    print("[Error] 设置失败")
            
            elif cmd in ['z', 'zero']:
                pos = tool.get_current_position()
                if pos is not None:
                    print(f"当前位置: {pos:.4f}°")
                confirm = input("确认设为0点? (y/n): ").strip().lower()
                if confirm == 'y':
                    if tool.set_zero_position():
                        print("[OK] 零点已设置！请保存到Flash！")
                    else:
                        print("[Error] 设置失败")
            
            elif cmd == 'id':
                if len(args) < 1:
                    print(f"当前ID: {tool.motor_id}")
                    new_id = input("输入新ID (1-30): ").strip()
                else:
                    new_id = args[0]
                try:
                    new_id = int(new_id)
                    tool.set_motor_id(new_id)
                except ValueError:
                    print("[Error] 无效的ID")
            
            elif cmd == 'save':
                confirm = input("确认保存到Flash? (y/n): ").strip().lower()
                if confirm == 'y':
                    if tool.save_to_flash():
                        print("[OK] 已保存到Flash")
                    else:
                        print("[Error] 保存失败")
            
            # ==================== 移动控制 ====================
            elif cmd == 'm':
                # 绝对移动: m 23000
                if len(args) < 1:
                    print("[Error] 用法: m <角度>  (如: m 23000)")
                    continue
                try:
                    target = float(args[0])
                    tool.move_to_position(target)
                except ValueError:
                    print("[Error] 无效的角度值")
            
            elif cmd == 'mr':
                # 相对移动 [°]: mr +100 或 mr -50
                if len(args) < 1:
                    print("[Error] 用法: mr <偏移量>  (如: mr +100, mr -50)")
                    continue
                try:
                    delta = float(args[0])
                    tool.move_relative(delta)
                except ValueError:
                    print("[Error] 无效的偏移量")
            
            # ---------- 毫米单位控制 ----------
            elif cmd == 'mm':
                # 绝对移动 [mm]: mm 50
                if len(args) < 1:
                    print("[Error] 用法: mm <位置>  (如: mm 50)")
                    continue
                try:
                    target_mm = float(args[0])
                    tool.move_to_position_mm(target_mm)
                except ValueError:
                    print("[Error] 无效的位置值")
            
            elif cmd == 'mmr':
                # 相对移动 [mm]: mmr +10 或 mmr -5
                if len(args) < 1:
                    print("[Error] 用法: mmr <偏移量>  (如: mmr +10, mmr -5)")
                    continue
                try:
                    delta_mm = float(args[0])
                    tool.move_relative_mm(delta_mm)
                except ValueError:
                    print("[Error] 无效的偏移量")
            
            elif cmd in ['s', 'stop']:
                tool.stop()
            
            # ==================== 未知命令 ====================
            else:
                print(f"[Error] 未知命令: {cmd}")
                print("输入 'help' 查看命令列表")
        
        except KeyboardInterrupt:
            print("\n[Info] 被用户中断")
            break
        except Exception as e:
            print(f"[Error] {e}")
    
    # 清理
    print("\n[Cleanup] 关闭设备...")
    if driver:
        driver.close()
    print("[OK] 已退出")


if __name__ == "__main__":
    main()
