"""
WHJ60 + Kinco 双电机同时控制

本模块实现WHJ60关节电机与Kinco旋转舵盘电机的同步控制。
解决在CAN总线上同时存在两种电机时的通信冲突问题。

硬件配置:
- WHJ60: ID=7, CAN FD协议, 波特率1M/5M
- Kinco: ID=1, 标准CAN协议, 波特率1M

使用方法:
1. 单独控制WHJ60示例
2. 单独控制Kinco示例
3. 双电机同步控制示例
"""

import struct
import time
import threading
from typing import Optional, Dict, Callable, List
from dataclasses import dataclass
from enum import IntEnum

# 导入现有驱动
from zlgcan_driver import ZlgCanDriver, ZCANDeviceType, CANFDFrame
from whj_protocol import WHJProtocol, Register, WorkMode
from kinco_driver import KincoDriver, KincoMode


@dataclass
class WHJState:
    """WHJ电机状态"""
    motor_id: int
    position_deg: float
    speed_rpm: float
    current_ma: int
    voltage_v: float
    temperature_c: float
    error_code: int
    is_enabled: bool
    work_mode: int
    timestamp: float


@dataclass
class SystemState:
    """系统整体状态"""
    whj_states: Dict[int, WHJState]
    kinco_states: Dict[int, 'KincoState']  # forward reference
    is_running: bool
    error_count: int


class DualMotorManager:
    """
    双电机管理器
    
    管理WHJ60和Kinco电机的同步通信，解决CAN总线上的冲突问题。
    
    关键设计:
    1. 使用独立的发送/接收循环
    2. 基于CAN ID过滤接收到的帧
    3. WHJ (CAN FD) 和 Kinco (标准CAN) 分离处理
    4. 优先级调度: WHJ状态查询优先级更高
    
    Usage:
        manager = DualMotorManager()
        manager.init_device()  # 初始化ZLG设备
        
        # 添加电机
        manager.add_whj(motor_id=7)
        manager.add_kinco(motor_id=1)
        
        # 启动所有电机
        manager.start_all()
        
        # 读取状态
        whj_state = manager.get_whj_state(7)
        
        # 同步控制
        manager.sync_move(whj_pos=45.0, kinco_pos=90.0)
        
        manager.close()
    """
    
    def __init__(self, dll_path: Optional[str] = None):
        """
        初始化双电机管理器
        
        Args:
            dll_path: zlgcan.dll路径, None则自动检测
        """
        self.dll_path = dll_path
        self.can_driver: Optional[ZlgCanDriver] = None
        self.is_initialized = False
        
        # 电机实例
        self.whj_motors: Dict[int, Dict] = {}  # motor_id -> config dict
        self.kinco_motors: Dict[int, KincoDriver] = {}  # motor_id -> KincoDriver
        
        # 状态缓存
        self.whj_states: Dict[int, WHJState] = {}
        self.kinco_states: Dict[int, 'KincoState'] = {}
        
        # 接收处理线程
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        
        # 回调函数
        self._whj_state_callbacks: List[Callable] = []
        self._error_callbacks: List[Callable] = []
        
        # 发送队列 (用于异步发送)
        self._tx_queue: List[tuple] = []  # [(can_id, data, is_extended), ...]
        
        # 统计
        self._rx_count_whj = 0
        self._rx_count_kinco = 0
        self._rx_count_other = 0
        self._tx_count = 0
        self._error_count = 0
    
    def init_device(self, device_type=ZCANDeviceType.USBCANFD_MINI, 
                    channel: int = 0,
                    arbitration_bps: int = 1000000,
                    data_bps: int = 5000000) -> bool:
        """
        初始化CAN设备
        
        Args:
            device_type: ZLG设备类型
            channel: CAN通道
            arbitration_bps: 仲裁段波特率 (默认1M, 与Kinco兼容)
            data_bps: 数据段波特率 (默认5M, 用于CAN FD)
        
        Returns:
            True if successful
        """
        try:
            self.can_driver = ZlgCanDriver(self.dll_path)
            self.can_driver.open(device_type, device_index=0, channel=channel)
            self.can_driver.init_canfd(arbitration_bps, data_bps)
            self.is_initialized = True
            print(f"[DualMotor] Device initialized: {device_type.name}")
            print(f"[DualMotor] Bitrate: Arbitration={arbitration_bps/1000:.0f}kbps, Data={data_bps/1000:.0f}kbps")
            return True
        except Exception as e:
            print(f"[DualMotor] Failed to initialize device: {e}")
            return False
    
    def add_whj(self, motor_id: int, auto_query: bool = True) -> bool:
        """
        添加WHJ电机
        
        Args:
            motor_id: 电机ID
            auto_query: 是否自动查询状态
        
        Returns:
            True if successful
        """
        if not self.is_initialized:
            print("[DualMotor] Error: Device not initialized")
            return False
        
        self.whj_motors[motor_id] = {
            'auto_query': auto_query,
            'last_query_time': 0,
            'query_interval': 0.05  # 50ms
        }
        
        # 初始化状态
        self.whj_states[motor_id] = WHJState(
            motor_id=motor_id,
            position_deg=0.0,
            speed_rpm=0.0,
            current_ma=0,
            voltage_v=0.0,
            temperature_c=0.0,
            error_code=0,
            is_enabled=False,
            work_mode=WorkMode.POSITION_MODE,
            timestamp=0
        )
        
        print(f"[DualMotor] Added WHJ motor ID={motor_id}")
        return True
    
    def add_kinco(self, motor_id: int) -> KincoDriver:
        """
        添加Kinco电机
        
        Args:
            motor_id: 电机ID
        
        Returns:
            KincoDriver实例
        """
        if not self.is_initialized:
            raise RuntimeError("Device not initialized")
        
        kinco = KincoDriver(self.can_driver, motor_id)
        self.kinco_motors[motor_id] = kinco
        print(f"[DualMotor] Added Kinco motor ID={motor_id}")
        return kinco
    
    def start_all(self):
        """启动所有电机"""
        print("\n[DualMotor] Starting all motors...")
        
        # 启动Kinco电机 (使用NMT命令)
        for motor_id, kinco in self.kinco_motors.items():
            print(f"  Starting Kinco ID={motor_id}...")
            kinco.start_node()
            time.sleep(0.2)
            kinco.set_absolute_position_mode()
            time.sleep(0.1)
        
        # 初始化WHJ电机
        for motor_id in self.whj_motors:
            print(f"  Initializing WHJ ID={motor_id}...")
            self._init_whj(motor_id)
            time.sleep(0.1)
        
        print("[DualMotor] All motors started")
    
    def _init_whj(self, motor_id: int):
        """初始化WHJ电机"""
        # 使能电机
        cmd = WHJProtocol.build_enable_motor(motor_id, True)
        self._send_whj_frame(motor_id, cmd)
        time.sleep(0.05)
        
        # 设置为位置模式
        cmd = WHJProtocol.build_set_work_mode(motor_id, WorkMode.POSITION_MODE)
        self._send_whj_frame(motor_id, cmd)
        time.sleep(0.05)
        
        # 查询初始状态
        self._query_whj_state(motor_id)
    
    def _send_whj_frame(self, motor_id: int, data: bytes):
        """发送WHJ CAN FD帧"""
        if not self.can_driver:
            return False
        
        # WHJ使用CAN ID = motor_id
        success = self.can_driver.send(
            can_id=motor_id,
            data=data,
            is_extended=False
        )
        
        if success:
            self._tx_count += 1
        return success
    
    def _query_whj_state(self, motor_id: int):
        """查询WHJ状态"""
        cmd = WHJProtocol.build_read_state(motor_id)
        return self._send_whj_frame(motor_id, cmd)
    
    def _query_whj_system_info(self, motor_id: int):
        """查询WHJ系统信息"""
        cmd = WHJProtocol.build_read_system_info(motor_id)
        return self._send_whj_frame(motor_id, cmd)
    
    def _process_received_frame(self, frame: CANFDFrame):
        """
        处理接收到的CAN帧
        
        关键: 根据CAN ID区分WHJ和Kinco
        """
        can_id = frame.can_id
        data = frame.data
        
        # 检查是否是WHJ响应 (ID = motor_id + 0x100)
        for motor_id in self.whj_motors:
            if can_id == motor_id + 0x100:
                self._process_whj_response(motor_id, data)
                self._rx_count_whj += 1
                return
        
        # 检查是否是Kinco响应 (Kinco通常使用TPDO/RPDO)
        # Kinco的响应ID通常是0x180+NodeID, 0x280+NodeID, 0x380+NodeID, 0x480+NodeID
        for motor_id in self.kinco_motors:
            if can_id == 0x180 + motor_id or can_id == 0x280 + motor_id:
                # Kinco状态反馈 (如果有的话)
                self._process_kinco_response(motor_id, can_id, data)
                self._rx_count_kinco += 1
                return
        
        # 其他帧
        self._rx_count_other += 1
    
    def _process_whj_response(self, motor_id: int, data: bytes):
        """处理WHJ响应帧"""
        try:
            if len(data) < 3:
                return
            
            cmd = data[0]
            reg = data[1]
            
            # 状态响应 (从CUR_CURRENT_L开始)
            if cmd == 0x01 and reg == Register.CUR_CURRENT_L:
                state = WHJProtocol.parse_state_response(motor_id, data)
                
                with self._lock:
                    self.whj_states[motor_id] = WHJState(
                        motor_id=motor_id,
                        position_deg=state.position_deg,
                        speed_rpm=state.speed_rpm,
                        current_ma=state.current_ma,
                        voltage_v=0.0,  # 需要单独查询
                        temperature_c=0.0,
                        error_code=0,
                        is_enabled=True,
                        work_mode=state.work_mode,
                        timestamp=time.time()
                    )
                
                # 触发回调
                for callback in self._whj_state_callbacks:
                    try:
                        callback(motor_id, self.whj_states[motor_id])
                    except Exception as e:
                        print(f"[DualMotor] Callback error: {e}")
            
            # 系统信息响应
            elif cmd == 0x01 and reg == Register.SYS_MODEL_TYPE:
                # 解析电压和温度
                if len(data) >= 12:
                    values = WHJProtocol.parse_read_response(data, Register.SYS_MODEL_TYPE)
                    voltage_raw = values[3]  # SYS_VOLTAGE
                    temp_raw = values[4]     # SYS_TEMP
                    
                    with self._lock:
                        if motor_id in self.whj_states:
                            self.whj_states[motor_id].voltage_v = voltage_raw * 0.01
                            self.whj_states[motor_id].temperature_c = temp_raw * 0.1
            
            # 错误响应
            elif cmd == 0xFF:
                print(f"[WHJ-{motor_id}] Error response: {data.hex()}")
                self._error_count += 1
                
        except Exception as e:
            print(f"[DualMotor] Error processing WHJ response: {e}")
            self._error_count += 1
    
    def _process_kinco_response(self, motor_id: int, can_id: int, data: bytes):
        """处理Kinco响应帧 (TPDO)"""
        # Kinco的TPDO1 (0x180+NodeID)通常包含位置和速度
        # 具体格式取决于Kinco的PDO映射配置
        # 这里做一个通用处理
        pass
    
    def update(self, timeout_ms: int = 10):
        """
        手动更新 - 接收所有可用帧并处理
        
        在同步控制循环中调用此方法
        
        Args:
            timeout_ms: 接收超时时间
        """
        if not self.can_driver:
            return
        
        # 清空接收缓冲区中的所有帧
        max_frames = 50  # 防止无限循环
        for _ in range(max_frames):
            frame = self.can_driver.receive_frame(timeout_ms=0)
            if frame is None:
                break
            self._process_received_frame(frame)
    
    def query_all_whj_states(self):
        """查询所有WHJ电机状态"""
        for motor_id in self.whj_motors:
            self._query_whj_state(motor_id)
            time.sleep(0.01)  # 避免总线拥塞
    
    def get_whj_state(self, motor_id: int) -> Optional[WHJState]:
        """
        获取WHJ电机状态
        
        Args:
            motor_id: 电机ID
        
        Returns:
            WHJState或None
        """
        with self._lock:
            return self.whj_states.get(motor_id)
    
    def get_kinco(self, motor_id: int) -> Optional[KincoDriver]:
        """
        获取Kinco驱动实例
        
        Args:
            motor_id: 电机ID
        
        Returns:
            KincoDriver或None
        """
        return self.kinco_motors.get(motor_id)
    
    def set_whj_position(self, motor_id: int, position_deg: float, block: bool = False, timeout: float = 5.0) -> bool:
        """
        设置WHJ目标位置
        
        Args:
            motor_id: 电机ID
            position_deg: 目标位置 (度)
            block: 是否等待到达目标位置
            timeout: 阻塞超时时间
        
        Returns:
            True if successful
        """
        if motor_id not in self.whj_motors:
            print(f"[DualMotor] WHJ motor {motor_id} not found")
            return False
        
        # 发送位置命令
        cmds = WHJProtocol.build_set_target_position(motor_id, position_deg)
        for cmd in cmds:
            self._send_whj_frame(motor_id, cmd)
            time.sleep(0.01)
        
        if block:
            start_time = time.time()
            while time.time() - start_time < timeout:
                self.update()
                state = self.get_whj_state(motor_id)
                if state:
                    error = abs(state.position_deg - position_deg)
                    if error < 0.5:  # 0.5度误差容忍
                        return True
                time.sleep(0.05)
            print(f"[DualMotor] WHJ position timeout")
            return False
        
        return True
    
    def set_kinco_position(self, motor_id: int, position_deg: float, speed_rpm: float = 50.0):
        """
        设置Kinco目标位置
        
        Args:
            motor_id: 电机ID
            position_deg: 目标位置 (度)
            speed_rpm: 速度 (RPM)
        
        Returns:
            True if successful
        """
        kinco = self.kinco_motors.get(motor_id)
        if not kinco:
            print(f"[DualMotor] Kinco motor {motor_id} not found")
            return False
        
        return kinco.move_to_position(position_deg, speed_rpm)
    
    def sync_move(self, whj_id: int, whj_pos: float, 
                  kinco_id: int, kinco_pos: float,
                  kinco_speed: float = 50.0):
        """
        同步移动两个电机
        
        Args:
            whj_id: WHJ电机ID
            whj_pos: WHJ目标位置 (度)
            kinco_id: Kinco电机ID
            kinco_pos: Kinco目标位置 (度)
            kinco_speed: Kinco速度 (RPM)
        """
        print(f"\n[DualMotor] Sync move: WHJ->{whj_pos}°, Kinco->{kinco_pos}°")
        
        # 同时发送命令
        self.set_whj_position(whj_id, whj_pos)
        self.set_kinco_position(kinco_id, kinco_pos, kinco_speed)
    
    def sync_move_sequence(self, sequence: List[dict], delay: float = 2.0):
        """
        执行同步运动序列
        
        Args:
            sequence: 运动序列列表
                [
                    {'whj': 45.0, 'kinco': 90.0},
                    {'whj': 0.0, 'kinco': 0.0},
                ]
            delay: 每个位置之间的等待时间 (秒)
        """
        whj_ids = list(self.whj_motors.keys())
        kinco_ids = list(self.kinco_motors.keys())
        
        if not whj_ids or not kinco_ids:
            print("[DualMotor] No motors available")
            return
        
        whj_id = whj_ids[0]
        kinco_id = kinco_ids[0]
        
        print(f"\n[DualMotor] Starting sequence with {len(sequence)} points")
        
        for i, point in enumerate(sequence):
            whj_pos = point.get('whj', 0.0)
            kinco_pos = point.get('kinco', 0.0)
            
            print(f"\n[Sequence] Step {i+1}/{len(sequence)}")
            self.sync_move(whj_id, whj_pos, kinco_id, kinco_pos)
            
            # 等待运动完成
            time.sleep(delay)
            
            # 更新状态
            self.query_all_whj_states()
            self.update()
            
            # 打印状态
            state = self.get_whj_state(whj_id)
            if state:
                print(f"  WHJ-{whj_id}: {state.position_deg:.2f}°")
        
        print("\n[DualMotor] Sequence completed")
    
    def enable_whj(self, motor_id: int, enable: bool = True):
        """使能/禁用WHJ电机"""
        cmd = WHJProtocol.build_enable_motor(motor_id, enable)
        self._send_whj_frame(motor_id, cmd)
        print(f"[DualMotor] WHJ-{motor_id} {'enabled' if enable else 'disabled'}")
    
    def stop_all(self):
        """停止所有电机"""
        print("\n[DualMotor] Stopping all motors...")
        
        # 停止Kinco
        for motor_id, kinco in self.kinco_motors.items():
            kinco.quick_stop()
        
        # 停止WHJ (保持当前位置)
        for motor_id in self.whj_motors:
            state = self.get_whj_state(motor_id)
            if state:
                self.set_whj_position(motor_id, state.position_deg)
    
    def print_status(self):
        """打印当前状态"""
        print("\n" + "=" * 60)
        print("Dual Motor System Status")
        print("=" * 60)
        
        # WHJ状态
        print("\nWHJ Motors:")
        for motor_id in self.whj_motors:
            state = self.get_whj_state(motor_id)
            if state:
                print(f"  ID={motor_id}: {state.position_deg:7.2f}° | "
                      f"{state.speed_rpm:6.2f} RPM | "
                      f"{state.current_ma} mA | "
                      f"Err=0x{state.error_code:04X}")
            else:
                print(f"  ID={motor_id}: No data")
        
        # Kinco状态
        print("\nKinco Motors:")
        for motor_id, kinco in self.kinco_motors.items():
            print(f"  ID={motor_id}: Enabled={kinco.state.is_enabled}, "
                  f"Target={kinco.state.target_position_deg}°")
        
        # 通信统计
        print("\nCommunication Stats:")
        print(f"  TX: {self._tx_count} frames")
        print(f"  RX: WHJ={self._rx_count_whj}, Kinco={self._rx_count_kinco}, Other={self._rx_count_other}")
        print(f"  Errors: {self._error_count}")
        print("=" * 60)
    
    def clear_buffer(self):
        """清空接收缓冲区"""
        if self.can_driver:
            self.can_driver.clear_buffer()
    
    def close(self):
        """关闭所有资源"""
        print("\n[DualMotor] Closing...")
        
        self._running = False
        
        # 等待线程结束
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.0)
        
        # 关闭CAN设备
        if self.can_driver:
            self.can_driver.close()
            self.can_driver = None
        
        self.is_initialized = False
        print("[DualMotor] Closed")
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


# ============================================================================
# 示例使用
# ============================================================================
def demo_separate_control():
    """示例1: 单独控制WHJ和Kinco"""
    print("\n" + "=" * 60)
    print("Demo 1: Separate Control")
    print("=" * 60)
    
    with DualMotorManager() as manager:
        # 初始化设备
        if not manager.init_device():
            return
        
        # 添加电机
        manager.add_whj(motor_id=7)
        manager.add_kinco(motor_id=1)
        
        # 启动所有电机
        manager.start_all()
        
        # 先测试WHJ单独控制
        print("\n--- WHJ Test ---")
        manager.set_whj_position(7, 45.0)
        
        # 等待并更新状态
        for _ in range(20):
            manager.update()
            time.sleep(0.05)
        
        state = manager.get_whj_state(7)
        if state:
            print(f"WHJ Position: {state.position_deg:.2f}°")
        
        # 再测试Kinco单独控制
        print("\n--- Kinco Test ---")
        kinco = manager.get_kinco(1)
        kinco.move_to_position(90.0, speed_rpm=50)
        time.sleep(2.0)
        
        kinco.move_to_position(0.0, speed_rpm=50)
        time.sleep(2.0)
        
        # 打印状态
        manager.print_status()


def demo_sync_control():
    """示例2: 同步控制WHJ和Kinco"""
    print("\n" + "=" * 60)
    print("Demo 2: Synchronized Control")
    print("=" * 60)
    
    with DualMotorManager() as manager:
        # 初始化设备
        if not manager.init_device():
            return
        
        # 添加电机
        manager.add_whj(motor_id=7)
        manager.add_kinco(motor_id=1)
        
        # 启动
        manager.start_all()
        time.sleep(0.5)
        
        # 同步运动序列
        sequence = [
            {'whj': 30.0, 'kinco': 45.0},
            {'whj': 60.0, 'kinco': 90.0},
            {'whj': 30.0, 'kinco': 45.0},
            {'whj': 0.0, 'kinco': 0.0},
        ]
        
        try:
            manager.sync_move_sequence(sequence, delay=2.0)
        except KeyboardInterrupt:
            print("\nInterrupted by user")
        
        # 最终状态
        manager.print_status()


def demo_whj_only():
    """示例3: 仅WHJ测试 (用于调试)"""
    print("\n" + "=" * 60)
    print("Demo 3: WHJ Only (Debug)")
    print("=" * 60)
    
    with DualMotorManager() as manager:
        if not manager.init_device():
            return
        
        manager.add_whj(motor_id=7)
        manager.start_all()
        
        print("\nReading WHJ state (press Ctrl+C to stop)...")
        try:
            while True:
                manager.query_all_whj_states()
                manager.update(timeout_ms=50)
                
                state = manager.get_whj_state(7)
                if state:
                    print(f"\rWHJ-7: {state.position_deg:7.2f}° | "
                          f"{state.speed_rpm:6.2f} RPM | "
                          f"{state.current_ma:5d} mA", end='')
                
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n\nStopped")
        
        manager.print_status()


def demo_kinco_only():
    """示例4: 仅Kinco测试 (用于调试)"""
    print("\n" + "=" * 60)
    print("Demo 4: Kinco Only (Debug)")
    print("=" * 60)
    
    with DualMotorManager() as manager:
        if not manager.init_device():
            return
        
        manager.add_kinco(motor_id=1)
        manager.start_all()
        
        print("\nRunning Kinco demo...")
        kinco = manager.get_kinco(1)
        
        try:
            for _ in range(3):
                print("\nMoving to 90°...")
                kinco.move_to_position(90.0, speed_rpm=50)
                time.sleep(2.0)
                
                print("Moving to 0°...")
                kinco.move_to_position(0.0, speed_rpm=50)
                time.sleep(2.0)
        except KeyboardInterrupt:
            print("\nStopped")


def demo_interactive():
    """示例5: 交互式控制"""
    print("\n" + "=" * 60)
    print("Demo 5: Interactive Control")
    print("=" * 60)
    
    with DualMotorManager() as manager:
        if not manager.init_device():
            return
        
        manager.add_whj(motor_id=7)
        manager.add_kinco(motor_id=1)
        manager.start_all()
        
        print("\nCommands:")
        print("  w <pos>  - Set WHJ position")
        print("  k <pos>  - Set Kinco position")
        print("  s        - Sync move (both to same position)")
        print("  q        - Query and print status")
        print("  quit     - Exit")
        
        while True:
            try:
                cmd = input("\n> ").strip().lower()
                
                if cmd == 'quit':
                    break
                elif cmd == 'q':
                    manager.query_all_whj_states()
                    manager.update()
                    manager.print_status()
                elif cmd.startswith('w '):
                    pos = float(cmd.split()[1])
                    manager.set_whj_position(7, pos)
                    print(f"WHJ moving to {pos}°")
                elif cmd.startswith('k '):
                    pos = float(cmd.split()[1])
                    manager.set_kinco_position(1, pos)
                    print(f"Kinco moving to {pos}°")
                elif cmd == 's':
                    pos = float(input("Enter sync position: "))
                    manager.sync_move(7, pos, 1, pos)
                    print(f"Both motors moving to {pos}°")
                else:
                    print("Unknown command")
                    
            except (ValueError, IndexError) as e:
                print(f"Invalid input: {e}")
            except KeyboardInterrupt:
                break
        
        print("\nExiting...")


# ============================================================================
# Main
# ============================================================================
if __name__ == "__main__":
    import sys
    
    print("=" * 60)
    print("WHJ60 + Kinco Dual Motor Control")
    print("=" * 60)
    print("\nSelect demo:")
    print("  1. Separate control (test individually)")
    print("  2. Sync control (synchronized movement)")
    print("  3. WHJ only (debug)")
    print("  4. Kinco only (debug)")
    print("  5. Interactive control")
    
    choice = input("\nEnter choice (1-5): ").strip()
    
    demos = {
        '1': demo_separate_control,
        '2': demo_sync_control,
        '3': demo_whj_only,
        '4': demo_kinco_only,
        '5': demo_interactive,
    }
    
    demo_func = demos.get(choice)
    if demo_func:
        try:
            demo_func()
        except Exception as e:
            print(f"\n[Error] {e}")
            import traceback
            traceback.print_exc()
    else:
        print("Invalid choice")
    
    print("\nDone!")
