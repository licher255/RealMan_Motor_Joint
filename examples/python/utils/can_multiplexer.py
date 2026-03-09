"""
CAN总线多路复用管理器

解决WHJ60 (CAN FD) 和 Kinco (标准CAN) 同时在总线上时的通信冲突问题。

问题分析:
1. WHJ60使用CAN FD协议, 响应ID = motor_id + 0x100
2. Kinco使用标准CAN协议, 可能产生TPDO/RPDO帧
3. 当两者同时在总线上时, 接收缓冲区可能被Kinco的帧占满
4. 需要有效过滤和优先级管理

解决方案:
1. 接收过滤器: 只处理感兴趣的CAN ID
2. 分时发送: 避免同时查询多个设备
3. 接收缓冲区管理: 定期清空,优先处理高优先级帧
4. 协议识别: 根据ID范围区分WHJ和Kinco帧
"""

import time
import threading
import queue
from typing import Optional, Dict, List, Callable, Set, Tuple
from dataclasses import dataclass
from enum import IntEnum

from core import ZlgCanDriver, CANFDFrame, ZCANDeviceType


class ProtocolType(IntEnum):
    """协议类型"""
    WHJ = 1      # RealMan WHJ (CAN FD)
    KINCO = 2    # Kinco伺服 (标准CAN)
    UNKNOWN = 0


@dataclass
class FilterConfig:
    """接收过滤器配置"""
    protocol: ProtocolType
    min_id: int      # 最小CAN ID (包含)
    max_id: int      # 最大CAN ID (包含)
    is_extended: bool = False
    
    def matches(self, can_id: int, is_extended: bool = False) -> bool:
        """检查CAN ID是否匹配过滤器"""
        if self.is_extended != is_extended:
            return False
        return self.min_id <= can_id <= self.max_id


class CANMultiplexer:
    """
    CAN总线多路复用器
    
    管理多个设备共享同一CAN总线时的通信。
    
    Features:
    - 基于ID的接收过滤
    - 优先级发送队列
    - 协议自动识别
    - 接收缓冲区管理
    - 统计和诊断
    
    Usage:
        mux = CANMultiplexer(can_driver)
        
        # 注册设备
        mux.register_device(7, ProtocolType.WHJ)
        mux.register_device(1, ProtocolType.KINCO)
        
        # 设置回调
        mux.set_rx_callback(7, whj_callback, ProtocolType.WHJ)
        mux.set_rx_callback(1, kinco_callback, ProtocolType.KINCO)
        
        # 启动接收线程
        mux.start()
        
        # 发送数据
        mux.send_whj(7, data)
        mux.send_kinco(1, data)
        
        mux.stop()
    """
    
    def __init__(self, can_driver: ZlgCanDriver, 
                 rx_buffer_size: int = 1000,
                 tx_queue_size: int = 100):
        """
        初始化多路复用器
        
        Args:
            can_driver: ZlgCanDriver实例
            rx_buffer_size: 接收缓冲区大小
            tx_queue_size: 发送队列大小
        """
        self.can_driver = can_driver
        self.rx_buffer_size = rx_buffer_size
        self.tx_queue_size = tx_queue_size
        
        # 设备注册表
        self._devices: Dict[int, ProtocolType] = {}  # motor_id -> protocol
        
        # 回调函数
        self._rx_callbacks: Dict[int, Callable] = {}  # motor_id -> callback
        self._protocol_callbacks: Dict[ProtocolType, List[Callable]] = {
            ProtocolType.WHJ: [],
            ProtocolType.KINCO: [],
            ProtocolType.UNKNOWN: []
        }
        
        # 过滤器 (用于快速判断帧归属)
        self._filters: List[FilterConfig] = [
            FilterConfig(ProtocolType.WHJ, 0x101, 0x11E, False),  # WHJ响应: ID+0x100
            FilterConfig(ProtocolType.KINCO, 0x180, 0x1FF, False), # Kinco TPDO
            FilterConfig(ProtocolType.KINCO, 0x000, 0x07F, False), # Kinco NMT/SDO
        ]
        
        # 接收和发送队列
        self._rx_queue: queue.Queue = queue.Queue(maxsize=rx_buffer_size)
        self._tx_queue: queue.Queue = queue.Queue(maxsize=tx_queue_size)
        
        # 高优先级发送队列 (用于查询命令)
        self._tx_priority_queue: queue.Queue = queue.Queue(maxsize=50)
        
        # 线程控制
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None
        self._process_thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()
        
        # 统计
        self._stats = {
            'rx_total': 0,
            'rx_whj': 0,
            'rx_kinco': 0,
            'rx_dropped': 0,
            'tx_total': 0,
            'tx_whj': 0,
            'tx_kinco': 0,
            'errors': 0,
        }
        
        # 协议特定的处理
        self._whj_response_map: Dict[int, bytes] = {}  # motor_id -> accumulated response
        self._last_rx_time: Dict[int, float] = {}  # motor_id -> timestamp
    
    def register_device(self, motor_id: int, protocol: ProtocolType):
        """
        注册设备到多路复用器
        
        Args:
            motor_id: 设备CAN ID
            protocol: 协议类型
        """
        with self._lock:
            self._devices[motor_id] = protocol
        print(f"[CANMux] Registered device ID={motor_id}, Protocol={protocol.name}")
    
    def unregister_device(self, motor_id: int):
        """注销设备"""
        with self._lock:
            if motor_id in self._devices:
                del self._devices[motor_id]
            if motor_id in self._rx_callbacks:
                del self._rx_callbacks[motor_id]
    
    def set_rx_callback(self, motor_id: int, callback: Callable, 
                        protocol: ProtocolType = ProtocolType.UNKNOWN):
        """
        设置接收回调函数
        
        Args:
            motor_id: 设备ID (0表示该协议的所有设备)
            callback: 回调函数(frame: CANFDFrame, protocol: ProtocolType)
            protocol: 协议类型 (用于ID=0的全局回调)
        """
        if motor_id != 0:
            self._rx_callbacks[motor_id] = callback
        else:
            self._protocol_callbacks[protocol].append(callback)
    
    def _identify_protocol(self, frame: CANFDFrame) -> ProtocolType:
        """
        识别帧的协议类型
        
        Args:
            frame: CAN帧
        
        Returns:
            ProtocolType
        """
        can_id = frame.can_id
        
        # 检查所有过滤器
        for filt in self._filters:
            if filt.matches(can_id, frame.is_extended):
                return filt.protocol
        
        # 特殊检查: WHJ响应帧 (ID = motor_id + 0x100)
        for motor_id, protocol in self._devices.items():
            if protocol == ProtocolType.WHJ and can_id == motor_id + 0x100:
                return ProtocolType.WHJ
        
        return ProtocolType.UNKNOWN
    
    def _get_motor_id_from_response(self, frame: CANFDFrame, 
                                     protocol: ProtocolType) -> int:
        """
        从响应帧中解析出motor_id
        
        Args:
            frame: CAN帧
            protocol: 已识别的协议
        
        Returns:
            motor_id (如果无法识别返回0)
        """
        can_id = frame.can_id
        
        if protocol == ProtocolType.WHJ:
            # WHJ响应ID = motor_id + 0x100
            motor_id = can_id - 0x100
            if motor_id in self._devices and self._devices[motor_id] == ProtocolType.WHJ:
                return motor_id
        
        elif protocol == ProtocolType.KINCO:
            # Kinco TPDO ID = 0x180 + NodeID
            if 0x180 <= can_id <= 0x1FF:
                node_id = can_id - 0x180
                if node_id in self._devices and self._devices[node_id] == ProtocolType.KINCO:
                    return node_id
        
        return 0
    
    def start(self):
        """启动多路复用器线程"""
        if self._running:
            return
        
        self._running = True
        
        # 启动接收线程
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        
        # 启动处理线程
        self._process_thread = threading.Thread(target=self._process_loop, daemon=True)
        self._process_thread.start()
        
        # 启动发送线程
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._tx_thread.start()
        
        print("[CANMux] Multiplexer started")
    
    def stop(self):
        """停止多路复用器"""
        self._running = False
        
        # 等待线程结束
        for thread in [self._rx_thread, self._process_thread, self._tx_thread]:
            if thread and thread.is_alive():
                thread.join(timeout=1.0)
        
        print("[CANMux] Multiplexer stopped")
    
    def _rx_loop(self):
        """接收循环 - 从CAN设备读取帧"""
        while self._running:
            try:
                # 非阻塞接收
                frame = self.can_driver.receive_frame(timeout_ms=10)
                
                if frame is not None:
                    # 放入接收队列
                    try:
                        self._rx_queue.put_nowait(frame)
                        self._stats['rx_total'] += 1
                    except queue.Full:
                        # 缓冲区满,丢弃最旧的帧
                        try:
                            self._rx_queue.get_nowait()
                            self._rx_queue.put_nowait(frame)
                            self._stats['rx_dropped'] += 1
                        except queue.Empty:
                            pass
                else:
                    time.sleep(0.001)  # 短暂休眠避免CPU占用过高
                    
            except Exception as e:
                print(f"[CANMux] RX error: {e}")
                self._stats['errors'] += 1
                time.sleep(0.01)
    
    def _process_loop(self):
        """处理循环 - 分发接收到的帧"""
        while self._running:
            try:
                # 从接收队列取帧
                frame = self._rx_queue.get(timeout=0.1)
                
                # 识别协议
                protocol = self._identify_protocol(frame)
                
                # 更新统计
                if protocol == ProtocolType.WHJ:
                    self._stats['rx_whj'] += 1
                elif protocol == ProtocolType.KINCO:
                    self._stats['rx_kinco'] += 1
                
                # 获取motor_id
                motor_id = self._get_motor_id_from_response(frame, protocol)
                
                # 调用回调
                if motor_id != 0 and motor_id in self._rx_callbacks:
                    try:
                        self._rx_callbacks[motor_id](frame, protocol)
                    except Exception as e:
                        print(f"[CANMux] Callback error for ID={motor_id}: {e}")
                
                # 调用协议全局回调
                for callback in self._protocol_callbacks[protocol]:
                    try:
                        callback(frame, protocol)
                    except Exception as e:
                        print(f"[CANMux] Protocol callback error: {e}")
                        
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[CANMux] Process error: {e}")
                self._stats['errors'] += 1
    
    def _tx_loop(self):
        """发送循环 - 处理发送队列"""
        while self._running:
            try:
                # 优先处理高优先级队列
                try:
                    item = self._tx_priority_queue.get(timeout=0.001)
                except queue.Empty:
                    # 高优先级队列为空,处理普通队列
                    try:
                        item = self._tx_queue.get(timeout=0.1)
                    except queue.Empty:
                        continue
                
                can_id, data, is_extended, protocol = item
                
                # 发送帧
                success = self.can_driver.send(
                    can_id=can_id,
                    data=data,
                    is_extended=is_extended
                )
                
                if success:
                    self._stats['tx_total'] += 1
                    if protocol == ProtocolType.WHJ:
                        self._stats['tx_whj'] += 1
                    elif protocol == ProtocolType.KINCO:
                        self._stats['tx_kinco'] += 1
                
            except Exception as e:
                print(f"[CANMux] TX error: {e}")
                self._stats['errors'] += 1
                time.sleep(0.01)
    
    def send_whj(self, motor_id: int, data: bytes, priority: bool = False) -> bool:
        """
        发送WHJ帧
        
        Args:
            motor_id: 电机ID
            data: 帧数据
            priority: 是否高优先级
        
        Returns:
            True if queued successfully
        """
        item = (motor_id, data, False, ProtocolType.WHJ)
        
        try:
            if priority:
                self._tx_priority_queue.put_nowait(item)
            else:
                self._tx_queue.put_nowait(item)
            return True
        except queue.Full:
            print(f"[CANMux] TX queue full, dropping WHJ frame to ID={motor_id}")
            return False
    
    def send_kinco(self, can_id: int, data: bytes, priority: bool = False) -> bool:
        """
        发送Kinco帧
        
        Args:
            can_id: CAN ID
            data: 帧数据
            priority: 是否高优先级
        
        Returns:
            True if queued successfully
        """
        item = (can_id, data, False, ProtocolType.KINCO)
        
        try:
            if priority:
                self._tx_priority_queue.put_nowait(item)
            else:
                self._tx_queue.put_nowait(item)
            return True
        except queue.Full:
            print(f"[CANMux] TX queue full, dropping Kinco frame to ID=0x{can_id:03X}")
            return False
    
    def clear_buffer(self):
        """清空接收缓冲区"""
        # 清空队列
        while not self._rx_queue.empty():
            try:
                self._rx_queue.get_nowait()
            except queue.Empty:
                break
        
        # 清空硬件缓冲区
        self.can_driver.clear_buffer()
    
    def get_stats(self) -> Dict:
        """获取统计信息"""
        return self._stats.copy()
    
    def print_stats(self):
        """打印统计信息"""
        print("\n" + "=" * 50)
        print("CAN Multiplexer Statistics")
        print("=" * 50)
        print(f"RX Total:     {self._stats['rx_total']}")
        print(f"  WHJ:        {self._stats['rx_whj']}")
        print(f"  Kinco:      {self._stats['rx_kinco']}")
        print(f"  Dropped:    {self._stats['rx_dropped']}")
        print(f"TX Total:     {self._stats['tx_total']}")
        print(f"  WHJ:        {self._stats['tx_whj']}")
        print(f"  Kinco:      {self._stats['tx_kinco']}")
        print(f"Errors:       {self._stats['errors']}")
        print("=" * 50)


class WHJCommunicator:
    """
    WHJ通信器 (使用多路复用器)
    
    专门处理WHJ电机的通信,避免Kinco干扰。
    """
    
    def __init__(self, multiplexer: CANMultiplexer, motor_id: int):
        """
        初始化WHJ通信器
        
        Args:
            multiplexer: CANMultiplexer实例
            motor_id: WHJ电机ID
        """
        self.mux = multiplexer
        self.motor_id = motor_id
        self._last_response: Optional[bytes] = None
        self._response_event = threading.Event()
        
        # 注册回调
        self.mux.set_rx_callback(motor_id, self._on_response, ProtocolType.WHJ)
    
    def _on_response(self, frame: CANFDFrame, protocol: ProtocolType):
        """接收响应回调"""
        self._last_response = frame.data
        self._response_event.set()
    
    def send_command(self, data: bytes, wait_response: bool = False, 
                     timeout: float = 0.5) -> Optional[bytes]:
        """
        发送命令并可选等待响应
        
        Args:
            data: 命令数据
            wait_response: 是否等待响应
            timeout: 等待超时时间
        
        Returns:
            响应数据或None
        """
        self._response_event.clear()
        self._last_response = None
        
        # 发送 (高优先级)
        success = self.mux.send_whj(self.motor_id, data, priority=True)
        
        if not success or not wait_response:
            return None
        
        # 等待响应
        if self._response_event.wait(timeout):
            return self._last_response
        
        return None
    
    def query_state(self) -> Optional[bytes]:
        """查询状态"""
        from core.protocol import WHJProtocol, Register
        cmd = WHJProtocol.build_read_state(self.motor_id)
        return self.send_command(cmd, wait_response=True, timeout=0.2)


class KincoCommunicator:
    """
    Kinco通信器 (使用多路复用器)
    """
    
    def __init__(self, multiplexer: CANMultiplexer, motor_id: int):
        """
        初始化Kinco通信器
        
        Args:
            multiplexer: CANMultiplexer实例
            motor_id: Kinco电机ID
        """
        self.mux = multiplexer
        self.motor_id = motor_id
    
    def send_nmt(self, command: int):
        """
        发送NMT命令
        
        Args:
            command: NMT命令码 (0x01=启动, 0x02=停止, 0x80=复位)
        """
        data = bytes([command, self.motor_id])
        self.mux.send_kinco(0x000, data, priority=True)
    
    def send_position_command(self, position_deg: float, speed_rpm: float = 50.0):
        """
        发送位置命令
        
        Args:
            position_deg: 目标位置 (度)
            speed_rpm: 速度 (RPM)
        """
        import struct
        
        position_units = int(position_deg * 100)  # 0.01度单位
        speed_units = int(speed_rpm)
        
        data = struct.pack('<i', position_units) + struct.pack('<H', speed_units) + bytes([0x00, 0x00])
        
        # 0x301 = 位置控制ID
        self.mux.send_kinco(0x301, data, priority=False)
    
    def set_absolute_mode(self):
        """设置为绝对位置模式"""
        data = bytes([0x3F, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.mux.send_kinco(0x201, data, priority=True)


# ============================================================================
# 示例和测试
# ============================================================================
def demo_multiplexer():
    """多路复用器测试"""
    print("=" * 60)
    print("CAN Multiplexer Test")
    print("=" * 60)
    
    # 创建CAN驱动
    can_driver = ZlgCanDriver()
    can_driver.open(ZCANDeviceType.USBCANFD_MINI, device_index=0, channel=0)
    can_driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
    
    # 创建多路复用器
    mux = CANMultiplexer(can_driver)
    
    # 注册设备
    mux.register_device(7, ProtocolType.WHJ)
    mux.register_device(1, ProtocolType.KINCO)
    
    # 启动
    mux.start()
    
    # 创建通信器
    whj = WHJCommunicator(mux, 7)
    kinco = KincoCommunicator(mux, 1)
    
    try:
        print("\nTest 1: Start Kinco node")
        kinco.send_nmt(0x01)  # 启动节点
        time.sleep(0.5)
        
        print("Test 2: Set Kinco to absolute mode")
        kinco.set_absolute_mode()
        time.sleep(0.5)
        
        print("Test 3: Query WHJ state")
        for i in range(5):
            response = whj.query_state()
            if response:
                print(f"  WHJ response: {response.hex()}")
            else:
                print("  No WHJ response")
            time.sleep(0.2)
        
        print("\nTest 4: Kinco position command")
        kinco.send_position_command(90.0, 50.0)
        time.sleep(2.0)
        
        print("\nStatistics:")
        mux.print_stats()
        
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        mux.stop()
        can_driver.close()


if __name__ == "__main__":
    demo_multiplexer()
