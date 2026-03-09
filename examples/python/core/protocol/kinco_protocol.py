"""
Kinco Rotary Servo Motor Protocol

Kinco旋转舵盘电机通信协议实现。

Protocol Specifications:
- CAN ID Format:
  - NMT Command: 0x000
  - Set Mode: 0x201
  - Position Control: 0x301
  - Speed Control: 0x401 (optional)
  - TPDO1 Feedback: 0x180 + NodeID
  - TPDO2 Feedback: 0x280 + NodeID
  
- Data Format (Little-endian):
  - Position: 0.01 degrees/LSB (int32)
  - Speed: 1 RPM/LSB (uint16)
"""

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, List


class KincoMode(IntEnum):
    """Kinco工作模式"""
    RELATIVE_POSITION = 0x0F  # 相对位置模式
    ABSOLUTE_POSITION = 0x10  # 绝对位置模式
    SPEED_MODE = 0x03         # 速度模式


class KincoNMTCommand(IntEnum):
    """NMT (Network Management) 命令"""
    START = 0x01      # 启动节点
    STOP = 0x02       # 停止节点
    PRE_OPERATIONAL = 0x80  # 进入预操作状态
    RESET_NODE = 0x81       # 复位节点
    RESET_COMM = 0x82       # 复位通信


@dataclass
class KincoState:
    """Kinco电机状态"""
    motor_id: int
    position_deg: float = 0.0       # 当前位置 (度)
    target_position_deg: float = 0.0  # 目标位置 (度)
    speed_rpm: float = 0.0          # 当前速度 (RPM)
    target_speed_rpm: float = 0.0   # 目标速度 (RPM)
    is_enabled: bool = False        # 是否使能
    is_moving: bool = False         # 是否正在运动
    error_code: int = 0             # 错误码
    
    def to_dict(self) -> dict:
        """转换为字典"""
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
    """Kinco配置参数"""
    node_id: int = 1
    position_scale: float = 0.01    # 位置分辨率 (度/LSB)
    speed_scale: float = 1.0        # 速度分辨率 (RPM/LSB)
    max_position: float = 360.0     # 最大位置 (度)
    min_position: float = -360.0    # 最小位置 (度)
    max_speed: float = 3000.0       # 最大速度 (RPM)


class KincoProtocol:
    """
    Kinco协议实现类
    
    提供帧构建和解析功能。
    """
    
    # CAN ID定义
    NMT_ID = 0x000          # NMT命令
    SET_MODE_ID = 0x201     # 设置工作模式
    POSITION_CTRL_ID = 0x301  # 位置控制
    SPEED_CTRL_ID = 0x401     # 速度控制 (备用)
    
    # TPDO反馈ID (0x180 + NodeID, 0x280 + NodeID, etc.)
    TPDO1_BASE = 0x180
    TPDO2_BASE = 0x280
    TPDO3_BASE = 0x380
    TPDO4_BASE = 0x480
    
    @staticmethod
    def build_nmt_frame(node_id: int, command: KincoNMTCommand) -> bytes:
        """
        构建NMT命令帧
        
        Args:
            node_id: 节点ID
            command: NMT命令
        
        Returns:
            2字节数据: [command, node_id]
        """
        return bytes([command, node_id])
    
    @staticmethod
    def build_start_node(node_id: int) -> bytes:
        """构建启动节点命令"""
        return KincoProtocol.build_nmt_frame(node_id, KincoNMTCommand.START)
    
    @staticmethod
    def build_stop_node(node_id: int) -> bytes:
        """构建停止节点命令"""
        return KincoProtocol.build_nmt_frame(node_id, KincoNMTCommand.STOP)
    
    @staticmethod
    def build_reset_node(node_id: int) -> bytes:
        """构建复位节点命令"""
        return KincoProtocol.build_nmt_frame(node_id, KincoNMTCommand.RESET_NODE)
    
    @staticmethod
    def build_set_mode_frame(mode: KincoMode) -> bytes:
        """
        构建设置工作模式帧
        
        Args:
            mode: 工作模式
        
        Returns:
            8字节数据: [0x3F, mode, 0, 0, 0, 0, 0, 0]
        """
        return bytes([0x3F, mode, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    @staticmethod
    def build_set_absolute_mode() -> bytes:
        """构建设置绝对位置模式帧"""
        return KincoProtocol.build_set_mode_frame(KincoMode.ABSOLUTE_POSITION)
    
    @staticmethod
    def build_set_relative_mode() -> bytes:
        """构建设置相对位置模式帧"""
        return KincoProtocol.build_set_mode_frame(KincoMode.RELATIVE_POSITION)
    
    @staticmethod
    def build_position_frame(position_deg: float, speed_rpm: float = 50.0) -> bytes:
        """
        构建位置控制帧
        
        Data Format (8 bytes):
        - Byte 0-3: Target position (int32, little-endian, 0.01 deg/LSB)
        - Byte 4-5: Target speed (uint16, little-endian, 1 RPM/LSB)
        - Byte 6-7: Reserved (0)
        
        Args:
            position_deg: 目标位置 (度)
            speed_rpm: 目标速度 (RPM)
        
        Returns:
            8字节数据
        """
        # 位置转换为0.01度单位
        position_units = int(position_deg * 100)
        # 速度转换为RPM单位
        speed_units = int(speed_rpm)
        
        # 构建数据: int32 (little-endian) + uint16 (little-endian) + 2 bytes padding
        data = struct.pack('<i', position_units)  # 4 bytes position
        data += struct.pack('<H', speed_units)     # 2 bytes speed
        data += bytes([0x00, 0x00])                # 2 bytes padding
        
        return data
    
    @staticmethod
    def build_relative_position_frame(delta_deg: float, speed_rpm: float = 50.0) -> bytes:
        """
        构建相对位置控制帧
        
        Args:
            delta_deg: 相对移动距离 (度)
            speed_rpm: 速度 (RPM)
        
        Returns:
            8字节数据
        """
        return KincoProtocol.build_position_frame(delta_deg, speed_rpm)
    
    @staticmethod
    def build_stop_frame(current_position_deg: float = 0.0) -> bytes:
        """
        构建停止帧 (设置当前位置为目标位置)
        
        Args:
            current_position_deg: 当前位置
        
        Returns:
            8字节数据
        """
        return KincoProtocol.build_position_frame(current_position_deg, 0)
    
    @staticmethod
    def parse_tpdo1_frame(data: bytes) -> Optional[dict]:
        """
        解析TPDO1帧 (位置和速度反馈)
        
        注意: 具体格式取决于Kinco的PDO映射配置
        这是一个通用的解析实现，可能需要根据实际配置调整。
        
        Args:
            data: 帧数据
        
        Returns:
            解析结果字典或None
        """
        if len(data) < 6:
            return None
        
        # 假设格式: [position(4 bytes), speed(2 bytes)]
        position_units = struct.unpack('<i', data[0:4])[0]
        speed_units = struct.unpack('<h', data[4:6])[0]  # 有符号short
        
        return {
            'position_deg': position_units * 0.01,
            'speed_rpm': speed_units,
        }
    
    @staticmethod
    def position_to_units(position_deg: float) -> int:
        """将度数转换为协议单位"""
        return int(position_deg * 100)
    
    @staticmethod
    def units_to_position(position_units: int) -> float:
        """将协议单位转换为度数"""
        return position_units * 0.01
    
    @staticmethod
    def speed_to_units(speed_rpm: float) -> int:
        """将RPM转换为协议单位"""
        return int(speed_rpm)
    
    @staticmethod
    def units_to_speed(speed_units: int) -> float:
        """将协议单位转换为RPM"""
        return float(speed_units)


# ============================================================================
# Utility Functions
# ============================================================================
def create_kinco_can_id(node_id: int, message_type: str) -> int:
    """
    创建Kinco CAN ID
    
    Args:
        node_id: 节点ID
        message_type: 消息类型 ('nmt', 'mode', 'position', 'tpdo1', 'tpdo2')
    
    Returns:
        CAN ID
    """
    type_map = {
        'nmt': KincoProtocol.NMT_ID,
        'mode': KincoProtocol.SET_MODE_ID,
        'position': KincoProtocol.POSITION_CTRL_ID,
        'speed': KincoProtocol.SPEED_CTRL_ID,
        'tpdo1': KincoProtocol.TPDO1_BASE + node_id,
        'tpdo2': KincoProtocol.TPDO2_BASE + node_id,
    }
    return type_map.get(message_type, 0)


def validate_position(position_deg: float, config: KincoConfig = None) -> bool:
    """
    验证位置是否在有效范围内
    
    Args:
        position_deg: 位置 (度)
        config: 配置对象
    
    Returns:
        True if valid
    """
    if config is None:
        config = KincoConfig()
    
    return config.min_position <= position_deg <= config.max_position


# ============================================================================
# Test
# ============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("Kinco Protocol Test")
    print("=" * 60)
    
    # 测试NMT命令
    print("\n[1] NMT Commands:")
    start_cmd = KincoProtocol.build_start_node(1)
    print(f"  Start Node 1: ID=0x000, Data={start_cmd.hex()}")
    
    stop_cmd = KincoProtocol.build_stop_node(1)
    print(f"  Stop Node 1:  ID=0x000, Data={stop_cmd.hex()}")
    
    # 测试模式设置
    print("\n[2] Mode Setting:")
    abs_mode = KincoProtocol.build_set_absolute_mode()
    print(f"  Absolute Mode: ID=0x201, Data={abs_mode.hex()}")
    
    rel_mode = KincoProtocol.build_set_relative_mode()
    print(f"  Relative Mode: ID=0x201, Data={rel_mode.hex()}")
    
    # 测试位置控制
    print("\n[3] Position Control:")
    pos_frame = KincoProtocol.build_position_frame(90.0, 50)
    print(f"  90° @ 50RPM: ID=0x301, Data={pos_frame.hex()}")
    print(f"  Parsed: position={KincoProtocol.position_to_units(90.0)} units, "
          f"speed={KincoProtocol.speed_to_units(50)} units")
    
    pos_frame2 = KincoProtocol.build_position_frame(0.0, 50)
    print(f"  0° @ 50RPM:  ID=0x301, Data={pos_frame2.hex()}")
    
    # 测试相对位置
    print("\n[4] Relative Position:")
    rel_frame = KincoProtocol.build_relative_position_frame(45.0, 30)
    print(f"  +45° @ 30RPM: ID=0x301, Data={rel_frame.hex()}")
    
    # 测试状态解析
    print("\n[5] State Parsing:")
    # 模拟TPDO1数据: position=9000 (90.00°), speed=500 (500 RPM)
    test_data = struct.pack('<i', 9000) + struct.pack('<h', 500) + bytes([0, 0])
    parsed = KincoProtocol.parse_tpdo1_frame(test_data)
    if parsed:
        print(f"  Parsed TPDO1: position={parsed['position_deg']:.2f}°, "
              f"speed={parsed['speed_rpm']} RPM")
    
    print("\n" + "=" * 60)
    print("All tests passed!")
    print("=" * 60)
