"""
ZLG CAN FD Driver for Python - Mixed Mode Support
Supports USBCANFD-100U-mini and other ZLG CAN FD devices

Features:
- CAN FD mode with backward compatibility for standard CAN devices
- Mixed communication with WHJ (CAN FD) and Kinco (CAN) motors

Requires: zlgcan.dll (included with ZCANPro installation)
"""

import ctypes
from ctypes import *
import platform
import time
from typing import List, Optional, Callable
from dataclasses import dataclass
from enum import IntEnum

# ============================================================================
# Device Type Constants
# ============================================================================
class ZCANDeviceType(IntEnum):
    """ZLG CAN Device Types"""
    USBCAN1 = 3
    USBCAN2 = 4
    USBCAN_E_U = 20
    USBCAN_2E_U = 21
    USBCAN_4E_U = 31
    USBCANFD_200U = 41
    USBCANFD_100U = 42
    USBCANFD_MINI = 43  # <-- Your device!
    USBCANFD_800U = 59
    USBCANFD_400U = 76


# ============================================================================
# Status Constants
# ============================================================================
ZCAN_STATUS_ERR = 0
ZCAN_STATUS_OK = 1
ZCAN_STATUS_ONLINE = 2
ZCAN_STATUS_OFFLINE = 3

ZCAN_TYPE_CAN = 0
ZCAN_TYPE_CANFD = 1
ZCAN_TYPE_ALL_DATA = 2


# ============================================================================
# Data Structures
# ============================================================================
class ZCANDeviceInfo(Structure):
    _fields_ = [
        ("hw_Version", c_ushort),
        ("fw_Version", c_ushort),
        ("dr_Version", c_ushort),
        ("in_Version", c_ushort),
        ("irq_Num", c_ushort),
        ("can_Num", c_ubyte),
        ("str_Serial_Num", c_ubyte * 20),
        ("str_hw_Type", c_ubyte * 40),
        ("reserved", c_ushort * 4),
    ]

    @property
    def hw_version(self):
        v = self.hw_Version
        return f"V{v // 0xFF}.{v & 0xFF:02x}"

    @property
    def fw_version(self):
        v = self.fw_Version
        return f"V{v // 0xFF}.{v & 0xFF:02x}"

    @property
    def serial(self):
        return "".join(chr(c) for c in self.str_Serial_Num if c > 0)


class _ZCANChannelCANConfig(Structure):
    _fields_ = [
        ("acc_code", c_uint),
        ("acc_mask", c_uint),
        ("reserved", c_uint),
        ("filter", c_ubyte),
        ("timing0", c_ubyte),
        ("timing1", c_ubyte),
        ("mode", c_ubyte),
    ]


class _ZCANChannelCANFDConfig(Structure):
    _fields_ = [
        ("acc_code", c_uint),
        ("acc_mask", c_uint),
        ("abit_timing", c_uint),
        ("dbit_timing", c_uint),
        ("brp", c_uint),
        ("filter", c_ubyte),
        ("mode", c_ubyte),
        ("pad", c_ushort),
        ("reserved", c_uint),
    ]


class _ZCANChannelConfigUnion(Union):
    _fields_ = [("can", _ZCANChannelCANConfig), ("canfd", _ZCANChannelCANFDConfig)]


class ZCANChannelInitConfig(Structure):
    _fields_ = [("can_type", c_uint), ("config", _ZCANChannelConfigUnion)]


class ZCANCANFrame(Structure):
    """Standard CAN Frame Structure (for CAN 2.0 devices like Kinco)"""
    _fields_ = [
        ("can_id", c_uint, 29),
        ("err", c_uint, 1),
        ("rtr", c_uint, 1),
        ("eff", c_uint, 1),
        ("can_dlc", c_ubyte),
        ("__pad", c_ubyte),
        ("__res0", c_ubyte),
        ("__res1", c_ubyte),
        ("data", c_ubyte * 8),
    ]


class ZCANTransmitData(Structure):
    """Standard CAN Transmit Structure"""
    _fields_ = [("frame", ZCANCANFrame), ("transmit_type", c_uint)]


class ZCANReceiveData(Structure):
    """Standard CAN Receive Structure"""
    _fields_ = [("frame", ZCANCANFrame), ("timestamp", c_ulonglong)]


class ZCANCANFDFrame(Structure):
    """CAN FD Frame Structure (for CAN FD devices like WHJ)"""
    _fields_ = [
        ("can_id", c_uint, 29),
        ("err", c_uint, 1),
        ("rtr", c_uint, 1),
        ("eff", c_uint, 1),
        ("len", c_ubyte),
        ("brs", c_ubyte, 1),
        ("esi", c_ubyte, 1),
        ("pad", c_ubyte, 6),
        ("__res0", c_ubyte),
        ("__res1", c_ubyte),
        ("data", c_ubyte * 64),
    ]


class ZCANTransmitFDData(Structure):
    _fields_ = [("frame", ZCANCANFDFrame), ("transmit_type", c_uint)]


class ZCANReceiveFDData(Structure):
    _fields_ = [("frame", ZCANCANFDFrame), ("timestamp", c_ulonglong)]


# ============================================================================
# ZLG CAN Driver Class
# ============================================================================
@dataclass
class CANFrame:
    """Python-friendly Standard CAN Frame (for Kinco)"""
    can_id: int
    data: bytes
    is_extended: bool = False
    is_remote: bool = False
    len: int = 0
    frame_type: str = "CAN"  # "CAN" or "CANFD"

    def __post_init__(self):
        if self.len == 0:
            self.len = len(self.data)
        if len(self.data) > 8:
            raise ValueError("Standard CAN frame data max 8 bytes")


@dataclass
class CANFDFrame(CANFrame):
    """Python-friendly CAN FD Frame (for WHJ)"""
    bitrate_switch: bool = True
    frame_type: str = "CANFD"

    def __post_init__(self):
        if self.len == 0:
            self.len = len(self.data)
        if len(self.data) > 64:
            raise ValueError("CAN FD frame data max 64 bytes")


class ZlgCanDriver:
    """
    ZLG CAN FD Device Driver
    
    Usage:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        
        # Send frame
        frame = CANFDFrame(can_id=0x01, data=bytes([0x01, 0x02, 0x03]))
        driver.send_frame(frame)
        
        # Receive frame
        rx_frame = driver.receive_frame(timeout_ms=100)
        
        driver.close()
    """

    def __init__(self, dll_path: Optional[str] = None):
        """
        Initialize driver
        
        Args:
            dll_path: Path to zlgcan.dll. 
                     Auto-detects based on Python architecture if not provided.
        """
        if dll_path is None:
            dll_path = self._auto_detect_dll_path()
        
        self.dll_path = dll_path
        self._dll = None
        self._device_handle = 0
        self._channel_handle = 0
        self._device_type = None
        self._is_open = False
        
        self._load_dll()
    
    def _auto_detect_dll_path(self) -> str:
        """Auto-detect DLL path based on Python architecture"""
        import os
        import platform
        
        is_64bit = platform.architecture()[0] == "64bit"
        arch = "x64" if is_64bit else "x86"
        
        # Get project root (examples/python/core/../../..)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
        
        # Primary path: third_party/zlgcan
        dll_path = os.path.join(project_root, "third_party", "zlgcan", arch, "zlgcan.dll")
        
        if os.path.exists(dll_path):
            print(f"[ZLG] Auto-detected {arch} DLL: {dll_path}")
            return dll_path
        
        # Fallback paths
        fallback_paths = [
            # Old location (backward compatibility)
            os.path.join(project_root, "aa1c7-main", "aa1c7-main", 
                       "zlgcanDev(2023.07.28)", "zlgcan", f"zlgcan_{arch}", "zlgcan.dll"),
            # ZCANPro installation
            r"C:\Program Files (x86)\ZCANPRO\zlgcan.dll" if not is_64bit else "",
        ]
        
        for path in fallback_paths:
            if path and os.path.exists(path):
                print(f"[ZLG] Using fallback DLL: {path}")
                return path
        
        # Return primary path even if not exists, will show error later
        return dll_path

    def _load_dll(self):
        """Load zlgcan.dll"""
        try:
            self._dll = WinDLL(self.dll_path)
            print(f"[ZLG] Loaded DLL: {self.dll_path}")
        except OSError as e:
            if "not a valid Win32 application" in str(e) or "193" in str(e):
                raise RuntimeError(
                    f"Architecture mismatch: {e}\n\n"
                    f"Your Python is 64-bit, but zlgcan.dll is 32-bit.\n"
                    f"Solutions:\n"
                    f"1. Use ZCANPro's built-in Python (32-bit):\n"
                    f"   C:\\Program Files (x86)\\ZCANPRO\\python38\\python.exe zlgcan_driver.py\n\n"
                    f"2. Download 64-bit zlgcan.dll from ZLG website\n\n"
                    f"3. Install 32-bit Python and run with that"
                )
            raise RuntimeError(f"Failed to load zlgcan.dll: {e}\n"
                             f"Please ensure ZCANPro is installed.")

    def open(self, device_type: ZCANDeviceType, device_index: int = 0, 
             channel: int = 0, reset_device: bool = False) -> bool:
        """
        Open ZLG CAN device
        
        Args:
            device_type: Type of ZLG device (e.g., ZCANDeviceType.USBCANFD_MINI)
            device_index: Device index (0 for first device)
            channel: CAN channel index (0 = CAN1, 1 = CAN2)
            reset_device: Try to reset device if open fails
        
        Returns:
            True if successful
        """
        self._device_type = device_type
        
        # Open device
        self._device_handle = self._dll.ZCAN_OpenDevice(device_type, device_index, 0)
        
        if self._device_handle == 0 and reset_device:
            # Try to close and reopen
            print(f"[ZLG] Trying to reset device...")
            self._dll.ZCAN_CloseDevice(0)  # Close all
            time.sleep(0.5)
            self._device_handle = self._dll.ZCAN_OpenDevice(device_type, device_index, 0)
        
        if self._device_handle == 0:
            raise RuntimeError(f"Failed to open device {device_type.name}. "
                             f"The device may be in use by another program.")
        
        print(f"[ZLG] Device opened: {device_type.name}, handle={self._device_handle}")
        
        # Get device info
        info = ZCANDeviceInfo()
        ret = self._dll.ZCAN_GetDeviceInf(self._device_handle, byref(info))
        if ret == ZCAN_STATUS_OK:
            print(f"[ZLG] Device Info:")
            print(f"  - Hardware Version: {info.hw_version}")
            print(f"  - Firmware Version: {info.fw_version}")
            print(f"  - Serial: {info.serial}")
            print(f"  - CAN Channels: {info.can_Num}")
        
        self._channel = channel
        return True

    def init_mixed_mode(self, arbitration_bps: int = 1000000,
                        data_bps: int = 5000000,
                        internal_resistance: bool = True) -> bool:
        """
        Initialize CAN FD channel in mixed mode (backward compatible with CAN 2.0)
        
        This mode allows communication with:
        - CAN FD devices (WHJ motors) at full CAN FD speed
        - Standard CAN devices (Kinco motors) using arbitration bitrate
        
        Args:
            arbitration_bps: Arbitration phase bitrate (default 1Mbps)
                             Standard CAN devices use this bitrate
            data_bps: Data phase bitrate (default 5Mbps)
                      Only used by CAN FD devices
            internal_resistance: Enable 120Ohm internal termination
        
        Returns:
            True if successful
        """
        return self.init_canfd(arbitration_bps, data_bps, internal_resistance)

    def init_canfd(self, arbitration_bps: int = 1000000, 
                   data_bps: int = 5000000,
                   internal_resistance: bool = True) -> bool:
        """
        Initialize CAN FD channel (alias for init_mixed_mode)
        
        Args:
            arbitration_bps: Arbitration phase bitrate (default 1Mbps)
            data_bps: Data phase bitrate (default 5Mbps)
            internal_resistance: Enable 120Ohm internal termination
        
        Returns:
            True if successful
        """
        if self._device_handle == 0:
            raise RuntimeError("Device not opened")
        
        chn = self._channel
        
        # Set CAN FD standard (0 = ISO CAN FD)
        ret = self._dll.ZCAN_SetValue(self._device_handle, 
                                      f"{chn}/canfd_standard".encode(), 
                                      b"0")
        if ret != ZCAN_STATUS_OK:
            print(f"[ZLG] Warning: Failed to set CANFD standard")
        
        # Set internal resistance (1 = enable 120Ohm termination)
        if internal_resistance:
            res_val = b"1"
        else:
            res_val = b"0"
        ret = self._dll.ZCAN_SetValue(self._device_handle, 
                                      f"{chn}/initenal_resistance".encode(), 
                                      res_val)
        if ret != ZCAN_STATUS_OK:
            print(f"[ZLG] Warning: Failed to set termination resistance")
        else:
            print(f"[ZLG] Internal 120Ohm termination: {'ON' if internal_resistance else 'OFF'}")
        
        # Set arbitration phase bitrate
        ret = self._dll.ZCAN_SetValue(self._device_handle, 
                                      f"{chn}/canfd_abit_baud_rate".encode(), 
                                      str(arbitration_bps).encode())
        if ret != ZCAN_STATUS_OK:
            raise RuntimeError(f"Failed to set arbitration bitrate {arbitration_bps}")
        
        # Set data phase bitrate
        ret = self._dll.ZCAN_SetValue(self._device_handle, 
                                      f"{chn}/canfd_dbit_baud_rate".encode(), 
                                      str(data_bps).encode())
        if ret != ZCAN_STATUS_OK:
            raise RuntimeError(f"Failed to set data bitrate {data_bps}")
        
        print(f"[ZLG] Bitrate: Arbitration={arbitration_bps/1000:.0f}kbps, Data={data_bps/1000:.0f}kbps")
        
        # Initialize channel
        init_cfg = ZCANChannelInitConfig()
        init_cfg.can_type = ZCAN_TYPE_CANFD
        init_cfg.config.canfd.mode = 0  # Normal mode
        
        self._channel_handle = self._dll.ZCAN_InitCAN(self._device_handle, 
                                                       chn, 
                                                       byref(init_cfg))
        if self._channel_handle == 0:
            raise RuntimeError(f"Failed to initialize CAN channel {chn}")
        
        print(f"[ZLG] Channel initialized, handle={self._channel_handle}")
        
        # Set filter (accept all frames)
        self._setup_filter()
        
        # Start CAN
        ret = self._dll.ZCAN_StartCAN(self._channel_handle)
        if ret != ZCAN_STATUS_OK:
            raise RuntimeError("Failed to start CAN")
        
        print(f"[ZLG] CAN FD started successfully!")
        self._is_open = True
        return True

    def _setup_filter(self):
        """Setup CAN filter to accept all frames"""
        chn = self._channel
        
        # Clear filter
        ret = self._dll.ZCAN_SetValue(self._device_handle, 
                                f"{chn}/filter_clear".encode(), b"0")
        print(f"[ZLG] Filter clear: {'OK' if ret == ZCAN_STATUS_OK else 'FAILED'}")
        
        # Set standard frame filter (accept all standard frames 0x000 - 0x7FF)
        ret = self._dll.ZCAN_SetValue(self._device_handle, 
                                f"{chn}/filter_mode".encode(), b"0")
        print(f"[ZLG] Filter mode (std): {'OK' if ret == ZCAN_STATUS_OK else 'FAILED'}")
        
        ret = self._dll.ZCAN_SetValue(self._device_handle, 
                                f"{chn}/filter_start".encode(), b"0")
        print(f"[ZLG] Filter start: {'OK' if ret == ZCAN_STATUS_OK else 'FAILED'}")
        
        ret = self._dll.ZCAN_SetValue(self._device_handle, 
                                f"{chn}/filter_end".encode(), b"0x7FF")
        print(f"[ZLG] Filter end: {'OK' if ret == ZCAN_STATUS_OK else 'FAILED'}")
        
        # Apply filter
        ret = self._dll.ZCAN_SetValue(self._device_handle, 
                                f"{chn}/filter_ack".encode(), b"0")
        print(f"[ZLG] Filter apply: {'OK' if ret == ZCAN_STATUS_OK else 'FAILED'}")

    def send_frame_can(self, frame: CANFrame) -> bool:
        """
        Send Standard CAN frame (for Kinco and other CAN 2.0 devices)
        
        Args:
            frame: CANFrame to send (max 8 bytes data)
        
        Returns:
            True if successful
        """
        if not self._is_open:
            raise RuntimeError("CAN not initialized")
        
        if len(frame.data) > 8:
            raise ValueError("Standard CAN frame data max 8 bytes, use send_frame_canfd for larger frames")
        
        tx_data = ZCANTransmitData()
        tx_data.frame.can_id = frame.can_id
        tx_data.frame.eff = 1 if frame.is_extended else 0
        tx_data.frame.rtr = 1 if frame.is_remote else 0
        tx_data.frame.can_dlc = min(frame.len, 8)
        tx_data.transmit_type = 0  # Normal send
        
        # Copy data
        data_bytes = frame.data if isinstance(frame.data, bytes) else bytes(frame.data)
        for i in range(min(len(data_bytes), 8)):
            tx_data.frame.data[i] = data_bytes[i]
        
        ret = self._dll.ZCAN_Transmit(self._channel_handle, byref(tx_data), 1)
        return ret == 1

    def send_frame_canfd(self, frame: CANFDFrame) -> bool:
        """
        Send CAN FD frame (for WHJ and other CAN FD devices)
        
        Args:
            frame: CANFDFrame to send (max 64 bytes data)
        
        Returns:
            True if successful
        """
        if not self._is_open:
            raise RuntimeError("CAN not initialized")
        
        tx_data = ZCANTransmitFDData()
        tx_data.frame.can_id = frame.can_id
        tx_data.frame.eff = 1 if frame.is_extended else 0
        tx_data.frame.rtr = 1 if frame.is_remote else 0
        tx_data.frame.brs = 1 if frame.bitrate_switch else 0
        tx_data.frame.len = min(frame.len, 64)
        tx_data.transmit_type = 0  # Normal send
        
        # Copy data
        data_bytes = frame.data if isinstance(frame.data, bytes) else bytes(frame.data)
        for i in range(min(len(data_bytes), 64)):
            tx_data.frame.data[i] = data_bytes[i]
        
        ret = self._dll.ZCAN_TransmitFD(self._channel_handle, byref(tx_data), 1)
        return ret == 1

    def send_frame(self, frame: CANFDFrame) -> bool:
        """
        Send frame (auto-detect CAN or CAN FD based on frame type and data length)
        
        Args:
            frame: CANFrame or CANFDFrame to send
        
        Returns:
            True if successful
        """
        if isinstance(frame, CANFDFrame) and (frame.frame_type == "CANFD" or len(frame.data) > 8):
            return self.send_frame_canfd(frame)
        else:
            return self.send_frame_can(frame)

    def send_can(self, can_id: int, data: bytes, is_extended: bool = False) -> bool:
        """
        Send Standard CAN frame (for Kinco motors)
        
        Args:
            can_id: CAN ID
            data: Data bytes (max 8 bytes)
            is_extended: Use extended ID (29-bit)
        
        Returns:
            True if successful
        """
        frame = CANFrame(
            can_id=can_id,
            data=data,
            is_extended=is_extended,
            frame_type="CAN"
        )
        return self.send_frame_can(frame)

    def send_canfd(self, can_id: int, data: bytes, is_extended: bool = False, bitrate_switch: bool = True) -> bool:
        """
        Send CAN FD frame (for WHJ motors)
        
        Args:
            can_id: CAN ID
            data: Data bytes (max 64 bytes)
            is_extended: Use extended ID (29-bit)
            bitrate_switch: Enable bitrate switching for data phase (default: True)
        
        Returns:
            True if successful
        """
        frame = CANFDFrame(
            can_id=can_id,
            data=data,
            is_extended=is_extended,
            bitrate_switch=bitrate_switch
        )
        return self.send_frame_canfd(frame)

    def send(self, can_id: int, data: bytes, is_extended: bool = False, 
             bitrate_switch: bool = True, frame_type: str = "auto") -> bool:
        """
        Simple send method (auto-detect or manual select)
        
        Args:
            can_id: CAN ID
            data: Data bytes (max 64 bytes for CAN FD, 8 for CAN)
            is_extended: Use extended ID (29-bit)
            bitrate_switch: Enable bitrate switching for data phase (CAN FD only)
            frame_type: "auto" - auto detect by data length
                       "CAN" - force standard CAN
                       "CANFD" - force CAN FD
        
        Returns:
            True if successful
        """
        if frame_type == "auto":
            # Auto detect: > 8 bytes must use CAN FD
            if len(data) > 8:
                frame_type = "CANFD"
            else:
                frame_type = "CAN"
        
        if frame_type == "CANFD":
            return self.send_canfd(can_id, data, is_extended, bitrate_switch)
        else:
            return self.send_can(can_id, data, is_extended)

    def receive_frame_can(self, timeout_ms: int = 100) -> Optional[CANFrame]:
        """
        Receive single Standard CAN frame
        
        Args:
            timeout_ms: Timeout in milliseconds
        
        Returns:
            CANFrame or None if timeout
        """
        if not self._is_open:
            raise RuntimeError("CAN not initialized")
        
        # Check CAN receive buffer
        rx_num = self._dll.ZCAN_GetReceiveNum(self._channel_handle, ZCAN_TYPE_CAN)
        if rx_num == 0:
            return None
        
        # Receive frame
        rx_data = ZCANReceiveData()
        ret = self._dll.ZCAN_Receive(self._channel_handle, byref(rx_data), 1, timeout_ms)
        
        if ret <= 0:
            return None
        
        # Convert to Python-friendly format
        data_len = min(rx_data.frame.can_dlc, 8)
        data = bytes(rx_data.frame.data[:data_len])
        
        return CANFrame(
            can_id=rx_data.frame.can_id,
            data=data,
            is_extended=bool(rx_data.frame.eff),
            is_remote=bool(rx_data.frame.rtr),
            len=data_len,
            frame_type="CAN"
        )

    def receive_frame_canfd(self, timeout_ms: int = 100) -> Optional[CANFDFrame]:
        """
        Receive single CAN FD frame
        
        Args:
            timeout_ms: Timeout in milliseconds
        
        Returns:
            CANFDFrame or None if timeout
        """
        if not self._is_open:
            raise RuntimeError("CAN not initialized")
        
        # Check CAN FD receive buffer
        rx_num = self._dll.ZCAN_GetReceiveNum(self._channel_handle, ZCAN_TYPE_CANFD)
        if rx_num == 0:
            return None
        
        # Receive frame
        rx_data = ZCANReceiveFDData()
        ret = self._dll.ZCAN_ReceiveFD(self._channel_handle, byref(rx_data), 1, timeout_ms)
        
        if ret <= 0:
            return None
        
        # Convert to Python-friendly format
        data_len = min(rx_data.frame.len, 64)
        data = bytes(rx_data.frame.data[:data_len])
        
        return CANFDFrame(
            can_id=rx_data.frame.can_id,
            data=data,
            is_extended=bool(rx_data.frame.eff),
            is_remote=bool(rx_data.frame.rtr),
            bitrate_switch=bool(rx_data.frame.brs),
            len=data_len
        )

    def receive_frame(self, timeout_ms: int = 100, frame_type: str = "any") -> Optional[CANFrame]:
        """
        Receive single frame (CAN or CAN FD)
        
        Args:
            timeout_ms: Timeout in milliseconds
            frame_type: "any" - receive any type
                       "CAN" - receive only standard CAN
                       "CANFD" - receive only CAN FD
        
        Returns:
            CANFrame or CANFDFrame, or None if timeout
        """
        if not self._is_open:
            raise RuntimeError("CAN not initialized")
        
        if frame_type == "CAN":
            return self.receive_frame_can(timeout_ms)
        elif frame_type == "CANFD":
            return self.receive_frame_canfd(timeout_ms)
        
        # "any" - check both buffers
        # Try CAN FD first (usually higher priority in mixed setup)
        frame = self.receive_frame_canfd(0)
        if frame:
            return frame
        
        # Then try standard CAN
        frame = self.receive_frame_can(timeout_ms)
        if frame:
            return frame
        
        return None

    def receive(self, timeout_ms: int = 100, frame_type: str = "any") -> Optional[CANFrame]:
        """Alias for receive_frame"""
        return self.receive_frame(timeout_ms, frame_type)

    def receive_all(self, max_frames: int = 100, frame_type: str = "any") -> List[CANFrame]:
        """
        Receive all available frames (mixed CAN and CAN FD)
        
        Args:
            max_frames: Maximum number of frames to receive
            frame_type: "any" - receive all types
                       "CAN" - receive only standard CAN
                       "CANFD" - receive only CAN FD
        
        Returns:
            List of CANFrame or CANFDFrame
        """
        frames = []
        for _ in range(max_frames):
            frame = self.receive_frame(timeout_ms=0, frame_type=frame_type)
            if frame is None:
                break
            frames.append(frame)
        return frames

    def clear_buffer(self):
        """Clear receive buffer"""
        if self._channel_handle:
            self._dll.ZCAN_ClearBuffer(self._channel_handle)

    def get_receive_count(self, frame_type: str = "any") -> int:
        """
        Get number of frames in receive buffer
        
        Args:
            frame_type: "any" - total count of CAN + CAN FD
                       "CAN" - only standard CAN frames
                       "CANFD" - only CAN FD frames
        """
        if not self._is_open:
            return 0
        
        if frame_type == "CAN":
            return self._dll.ZCAN_GetReceiveNum(self._channel_handle, ZCAN_TYPE_CAN)
        elif frame_type == "CANFD":
            return self._dll.ZCAN_GetReceiveNum(self._channel_handle, ZCAN_TYPE_CANFD)
        else:  # "any"
            can_count = self._dll.ZCAN_GetReceiveNum(self._channel_handle, ZCAN_TYPE_CAN)
            canfd_count = self._dll.ZCAN_GetReceiveNum(self._channel_handle, ZCAN_TYPE_CANFD)
            return can_count + canfd_count

    def get_mixed_mode_status(self) -> dict:
        """
        Get mixed mode status including receive counts for both CAN and CAN FD
        
        Returns:
            Dictionary with counts and mode info
        """
        if not self._is_open:
            return {"status": "closed", "can": 0, "canfd": 0, "total": 0}
        
        can_count = self._dll.ZCAN_GetReceiveNum(self._channel_handle, ZCAN_TYPE_CAN)
        canfd_count = self._dll.ZCAN_GetReceiveNum(self._channel_handle, ZCAN_TYPE_CANFD)
        
        return {
            "status": "open",
            "mode": "CAN FD Mixed Mode",
            "can": can_count,
            "canfd": canfd_count,
            "total": can_count + canfd_count
        }

    def close(self):
        """Close device and cleanup - ensures CAN bus is properly reset"""
        if not self._is_open and not self._device_handle:
            return  # Already closed
        
        print("[ZLG] Closing device...")
        
        try:
            # Clear any pending transmissions/receptions
            if self._channel_handle:
                self._dll.ZCAN_ClearBuffer(self._channel_handle)
                
            # Stop CAN channel
            if self._channel_handle:
                ret = self._dll.ZCAN_ResetCAN(self._channel_handle)
                if ret != ZCAN_STATUS_OK:
                    print(f"[ZLG] Warning: ResetCAN returned {ret}")
                self._channel_handle = 0
            
            # Close device
            if self._device_handle:
                ret = self._dll.ZCAN_CloseDevice(self._device_handle)
                if ret != ZCAN_STATUS_OK:
                    print(f"[ZLG] Warning: CloseDevice returned {ret}")
                self._device_handle = 0
            
            self._is_open = False
            print("[ZLG] Device closed successfully")
            
        except Exception as e:
            print(f"[ZLG] Error during close: {e}")
            # Force cleanup even on error
            self._channel_handle = 0
            self._device_handle = 0
            self._is_open = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False


# ============================================================================
# Mixed Mode Test - CAN FD + Standard CAN
# ============================================================================
if __name__ == "__main__":
    print("=" * 70)
    print("ZLG CAN FD Driver Test - Mixed Mode (WHJ CAN FD + Kinco CAN)")
    print("=" * 70)
    print("\nThis test demonstrates communication with:")
    print("  - WHJ motors using CAN FD frames")
    print("  - Kinco motors using Standard CAN frames")
    print("\nBoth devices share the same CAN bus at 1Mbps arbitration bitrate.")
    print("=" * 70)
    
    try:
        # Create driver
        driver = ZlgCanDriver()
        
        # Open USBCANFD-100U-mini
        driver.open(ZCANDeviceType.USBCANFD_MINI, device_index=0, channel=0)
        
        # Initialize Mixed Mode (CAN FD with CAN backward compatibility)
        # Arbitration: 1Mbps (used by both CAN and CAN FD devices)
        # Data: 5Mbps (used only by CAN FD devices like WHJ)
        driver.init_mixed_mode(arbitration_bps=1000000, data_bps=5000000)
        
        print("\n" + "-" * 70)
        print("[Test 1] Send to Kinco motor (Standard CAN)")
        print("-" * 70)
        # Kinco typically uses CAN ID 0x601 + node_id for SDO communication
        kinco_cmd = bytes([0x23, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])  # Set target position
        success = driver.send_can(can_id=0x601, data=kinco_cmd)
        print(f"  Send CAN to Kinco (ID=0x601): {'OK' if success else 'FAILED'}")
        
        print("\n" + "-" * 70)
        print("[Test 2] Send to WHJ motor (CAN FD)")
        print("-" * 70)
        # WHJ uses CAN FD with ID = motor_id
        # Read current position: command=0x01, reg=0x14 (CUR_POSITION_L)
        whj_cmd = bytes([0x01, 0x14, 0x00, 0x02])  # Read position (4 bytes)
        success = driver.send_canfd(can_id=0x01, data=whj_cmd, bitrate_switch=True)
        print(f"  Send CAN FD to WHJ (ID=0x01): {'OK' if success else 'FAILED'}")
        
        print("\n" + "-" * 70)
        print("[Test 3] Auto-detect mode (by data length)")
        print("-" * 70)
        # Short data (<=8 bytes) -> auto send as CAN
        short_data = bytes([0x01, 0x02, 0x03])
        success = driver.send(can_id=0x02, data=short_data)  # auto -> CAN
        print(f"  Auto send short data (3 bytes): {'OK' if success else 'FAILED'} -> CAN")
        
        # Long data (>8 bytes) -> auto send as CAN FD
        long_data = bytes([0x01] * 16)
        success = driver.send(can_id=0x03, data=long_data)  # auto -> CAN FD
        print(f"  Auto send long data (16 bytes): {'OK' if success else 'FAILED'} -> CAN FD")
        
        print("\n" + "-" * 70)
        print("[Test 4] Force frame type")
        print("-" * 70)
        # Force 4 bytes as CAN FD (even though it's short)
        success = driver.send(can_id=0x04, data=bytes([0x01, 0x02, 0x03, 0x04]), 
                             frame_type="CANFD")
        print(f"  Force 4 bytes as CAN FD: {'OK' if success else 'FAILED'}")
        
        # Force as CAN (explicit)
        success = driver.send(can_id=0x05, data=bytes([0x01, 0x02]), 
                             frame_type="CAN")
        print(f"  Force as Standard CAN: {'OK' if success else 'FAILED'}")
        
        print("\n" + "-" * 70)
        print("[Test 5] Receive frames (Mixed Mode)")
        print("-" * 70)
        print("  Receiving for 3 seconds (press Ctrl+C to stop early)...")
        print(f"  {'Time':<10} {'Type':<10} {'ID':<8} {'Data'}")
        print("  " + "-" * 50)
        
        start_time = time.time()
        try:
            while time.time() - start_time < 3:
                # Check mixed mode status
                status = driver.get_mixed_mode_status()
                
                # Receive any frame type
                frame = driver.receive(timeout_ms=50)
                if frame:
                    elapsed = time.time() - start_time
                    frame_type_str = getattr(frame, 'frame_type', 'CAN')
                    id_str = f"0x{frame.can_id:03X}"
                    data_str = frame.data.hex()[:20]  # Limit display length
                    if len(frame.data) > 10:
                        data_str += "..."
                    print(f"  {elapsed:>6.2f}s   {frame_type_str:<10} {id_str:<8} {data_str}")
                
                time.sleep(0.001)
        except KeyboardInterrupt:
            print("\n  [Test] Stopped by user")
        
        # Final status
        print("\n" + "-" * 70)
        print("[Final Status]")
        print("-" * 70)
        status = driver.get_mixed_mode_status()
        print(f"  CAN frames received:  {status['can']}")
        print(f"  CAN FD frames received: {status['canfd']}")
        print(f"  Total frames received: {status['total']}")
        
    except Exception as e:
        print(f"\n[Error] {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'driver' in locals():
            driver.close()
    
    print("\n" + "=" * 70)
    print("[Test] Done!")
    print("=" * 70)
