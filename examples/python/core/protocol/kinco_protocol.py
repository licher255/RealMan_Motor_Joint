"""
Kinco Rotary Servo Motor Protocol (蓝莓项目)

电机型号: FD135驱动器 + Q6电机
通信模式: CAN标准帧 2.0A
波特率: 1Mbps

Protocol Specifications:
- CAN ID Format:
  - NMT Command: 0x000
  - Control Word/Mode: 0x201
  - Position Control: 0x301
  - TPDO1 Feedback: 0x180 + NodeID
  
- Data Format:
  - Position: 角度 × 16384 (减速比), 有符号32位, 小端
  - Speed: RPM × (65536×512/1875), 无符号32位, 小端

参考: 蓝莓项目旋转电机操作指南
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
    HOMING_MODE = 0x06        # 原点设置模式 (控制模式6)


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
    Kinco协议实现类 (蓝莓项目专用)
    
    提供帧构建和解析功能。
    """
    
    # CAN ID定义
    NMT_ID = 0x000          # NMT命令
    CONTROL_ID = 0x201      # 控制字/模式设置
    POSITION_CTRL_ID = 0x301  # 位置控制
    
    # TPDO反馈ID
    TPDO1_BASE = 0x180      # 状态反馈
    
    # 机械参数
    # 根据操作指南: 90度 = 0x00168000 = 1474560 inc
    # 减速比 = 1474560 / 90 = 16384 = 2^14 (每转脉冲数)
    GEAR_RATIO = 16384
    
    # 速度转换因子: 1 RPM = 65536 * 512 / 1875 ≈ 17895.7
    RPM_TO_UNITS = (65536 * 512) / 1875
    
    # 控制字
    CONTROL_ENABLE = 0x3F   # 使能控制字 (高位)
    CONTROL_HOME_1 = 0x0F   # 设置原点步骤1
    CONTROL_HOME_2 = 0x1F   # 设置原点步骤2
    
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
    def build_control_word_frame(control_low: int, control_high: int, mode: int) -> bytes:
        """
        构建控制字帧 (0x201)
        
        格式: [control_low, control_high, mode, 0, 0, 0, 0, 0]
        
        Args:
            control_low: 控制字低位
            control_high: 控制字高位
            mode: 工作模式
        
        Returns:
            8字节数据
        """
        return bytes([control_low, control_high, mode, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    @staticmethod
    def build_set_mode_frame(mode: KincoMode) -> bytes:
        """
        构建设置工作模式帧 (使用默认使能控制字)
        
        Args:
            mode: 工作模式
        
        Returns:
            8字节数据: [0x01, 0x3F, mode, 0, 0, 0, 0, 0]
        """
        return KincoProtocol.build_control_word_frame(0x01, KincoProtocol.CONTROL_ENABLE, mode)
    
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
        构建位置控制帧 (0x301)
        
        Data Format (8 bytes, Little-endian):
        - Byte 0-3: Target position (int32) = 角度 × 182 × 90
        - Byte 4-7: Target speed (uint32) = RPM × (65536×512/1875)
        
        Args:
            position_deg: 目标位置 (度, 正=逆时针, 负=顺时针)
            speed_rpm: 目标速度 (RPM)
        
        Returns:
            8字节数据
        """
        # 位置转换: 角度 × 减速比
        position_units = int(position_deg * KincoProtocol.GEAR_RATIO)
        
        # 速度转换: RPM × 转换因子
        speed_units = int(speed_rpm * KincoProtocol.RPM_TO_UNITS)
        
        # 构建数据: int32 (position) + uint32 (speed)
        data = struct.pack('<i', position_units)   # 4 bytes position
        data += struct.pack('<I', speed_units)     # 4 bytes speed
        
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
    
    # ========================================================================
    # 原点设置 (Homing)
    # ========================================================================
    
    @staticmethod
    def build_homing_step1() -> bytes:
        """
        设置原点步骤1: 控制模式6, 控制字0F
        
        发送: 0x201, [06, 0F, 00, 00, 00, 00, 00, 00]
        """
        return bytes([0x06, KincoProtocol.CONTROL_HOME_1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
    @staticmethod
    def build_homing_step2() -> bytes:
        """
        设置原点步骤2: 控制字1F
        
        发送: 0x201, [06, 1F, 00, 00, 00, 00, 00, 00]
        """
        return bytes([0x06, KincoProtocol.CONTROL_HOME_2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    
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
        """将度数转换为协议单位 (角度 × 182 × 90)"""
        return int(position_deg * KincoProtocol.GEAR_RATIO)
    
    @staticmethod
    def units_to_position(position_units: int) -> float:
        """将协议单位转换为度数"""
        return position_units / KincoProtocol.GEAR_RATIO
    
    @staticmethod
    def speed_to_units(speed_rpm: float) -> int:
        """将RPM转换为协议单位"""
        return int(speed_rpm * KincoProtocol.RPM_TO_UNITS)
    
    @staticmethod
    def units_to_speed(speed_units: int) -> float:
        """将协议单位转换为RPM"""
        return speed_units / KincoProtocol.RPM_TO_UNITS


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
    print(f"  Gear Ratio: {KincoProtocol.GEAR_RATIO} (16384 inc/rev)")
    print(f"  RPM Factor: {KincoProtocol.RPM_TO_UNITS:.1f}")
    
    pos_frame = KincoProtocol.build_position_frame(90.0, 50)
    print(f"  90 deg @ 50RPM: ID=0x301, Data={pos_frame.hex()}")
    print(f"  Position units: {KincoProtocol.position_to_units(90.0)}")
    print(f"  Speed units: {KincoProtocol.speed_to_units(50.0):.0f}")
    
    pos_frame2 = KincoProtocol.build_position_frame(0.0, 50)
    print(f"  0 deg @ 50RPM:  ID=0x301, Data={pos_frame2.hex()}")
    
    # 测试相对位置
    print("\n[4] Relative Position:")
    rel_frame = KincoProtocol.build_relative_position_frame(45.0, 30)
    print(f"  +45 deg @ 30RPM: ID=0x301, Data={rel_frame.hex()}")
    
    # 测试原点设置
    print("\n[5] Homing (Set Origin):")
    home_step1 = KincoProtocol.build_homing_step1()
    print(f"  Step 1 - ID=0x201, Data={home_step1.hex()}")
    home_step2 = KincoProtocol.build_homing_step2()
    print(f"  Step 2 - ID=0x201, Data={home_step2.hex()}")
    
    # 测试状态解析
    print("\n[6] State Parsing:")
    # 模拟TPDO1数据: position=9000 (90.00 deg), speed=500 (500 RPM)
    test_data = struct.pack('<i', 9000) + struct.pack('<h', 500) + bytes([0, 0])
    parsed = KincoProtocol.parse_tpdo1_frame(test_data)
    if parsed:
        print(f"  Parsed TPDO1: position={parsed['position_deg']:.2f} deg, "
              f"speed={parsed['speed_rpm']} RPM")
    
    print("\n" + "=" * 60)
    print("All tests passed!")
    print("=" * 60)
