"""
ZLG CAN FD Driver implementation.

Supports USBCANFD-100U-mini and other ZLG CAN FD devices.
"""

import ctypes
from ctypes import *
import platform
import time
import os
from typing import Optional, List
from enum import IntEnum
from pathlib import Path

from .interface import CANInterface, CANFrame, CANFDFrame, FrameType
from .exceptions import CANError


class ZCANDeviceType(IntEnum):
    """ZLG CAN Device Types."""
    USBCAN1 = 3
    USBCAN2 = 4
    USBCAN_E_U = 20
    USBCAN_2E_U = 21
    USBCAN_4E_U = 31
    USBCANFD_200U = 41
    USBCANFD_100U = 42
    USBCANFD_MINI = 43
    USBCANFD_800U = 59
    USBCANFD_400U = 76


# Status Constants
ZCAN_STATUS_ERR = 0
ZCAN_STATUS_OK = 1
ZCAN_STATUS_ONLINE = 2
ZCAN_STATUS_OFFLINE = 3
ZCAN_TYPE_CAN = 0
ZCAN_TYPE_CANFD = 1
ZCAN_TYPE_ALL_DATA = 2


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
    def hw_version(self) -> str:
        v = self.hw_Version
        return f"V{v // 0xFF}.{v & 0xFF:02x}"
    
    @property
    def fw_version(self) -> str:
        v = self.fw_Version
        return f"V{v // 0xFF}.{v & 0xFF:02x}"
    
    @property
    def serial(self) -> str:
        return "".join(chr(c) for c in self.str_Serial_Num if c > 0)


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
    _fields_ = [("canfd", _ZCANChannelCANFDConfig)]


class ZCANChannelInitConfig(Structure):
    _fields_ = [("can_type", c_uint), ("config", _ZCANChannelConfigUnion)]


class ZCANCANFDFrame(Structure):
    """CAN FD Frame Structure."""
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


class ZCANCANFrame(Structure):
    """Standard CAN Frame Structure."""
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
    _fields_ = [("frame", ZCANCANFrame), ("transmit_type", c_uint)]


class ZCANReceiveData(Structure):
    _fields_ = [("frame", ZCANCANFrame), ("timestamp", c_ulonglong)]


class ResourceNotFoundError(Exception):
    """Resource not found error."""
    pass


class ZlgCanDriver(CANInterface):
    """
    ZLG CAN FD Device Driver.
    
    Supports mixed mode communication with both CAN FD and standard CAN devices.
    
    Example:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        
        # Send CAN FD frame (WHJ)
        driver.send_canfd(can_id=0x07, data=bytes([0x01, 0x14, 0x00, 0x02]))
        
        # Send standard CAN frame (Kinco)
        driver.send_can(can_id=0x601, data=bytes([0x23, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00]))
        
        driver.close()
    """
    
    def __init__(self, dll_path: Optional[str] = None):
        """
        Initialize driver.
        
        Args:
            dll_path: Path to zlgcan.dll. Auto-detects if not provided.
        """
        super().__init__()
        self._dll = None
        self._device_handle = 0
        self._channel_handle = 0
        self._device_type: Optional[ZCANDeviceType] = None
        self._is_open_flag = False
        self._channel = 0
        
        if dll_path is None:
            dll_path = self._auto_detect_dll_path()
        
        self._dll_path = dll_path
        self._load_dll()
    
    def _auto_detect_dll_path(self) -> str:
        """Auto-detect DLL path based on Python architecture."""
        is_64bit = platform.architecture()[0] == "64bit"
        arch = "x64" if is_64bit else "x86"
        
        # DLL is located at: project_root/third_party/zlgcan/{arch}/zlgcan.dll
        # This file is at: project_root/python_sdk/src/zlg_can/zlg_driver.py
        current_file = Path(__file__).resolve()
        dll_path = current_file.parents[3] / "third_party" / "zlgcan" / arch / "zlgcan.dll"
        
        return str(dll_path)
    
    def _load_dll(self):
        """Load zlgcan.dll."""
        try:
            self._dll = WinDLL(self._dll_path)
        except OSError as e:
            if "not a valid Win32 application" in str(e) or "193" in str(e):
                raise ResourceNotFoundError(
                    f"Architecture mismatch: {e}. "
                    f"Ensure Python architecture matches DLL (x64 vs x86)."
                )
            raise ResourceNotFoundError(
                f"Failed to load zlgcan.dll: {e}. "
                f"Please ensure ZCANPro is installed or provide correct dll_path."
            )
    
    def open(self, device_type: ZCANDeviceType, device_index: int = 0,
             channel: int = 0, reset_device: bool = False) -> bool:
        """
        Open ZLG CAN device.
        
        Args:
            device_type: Type of ZLG device
            device_index: Device index (0 for first device)
            channel: CAN channel index
            reset_device: Try to reset device if open fails
            
        Returns:
            True if successful
            
        Raises:
            CANError: If device cannot be opened
        """
        self._device_type = device_type
        self._channel = channel
        
        self._device_handle = self._dll.ZCAN_OpenDevice(device_type, device_index, 0)
        
        if self._device_handle == 0 and reset_device:
            self._dll.ZCAN_CloseDevice(0)
            time.sleep(0.5)
            self._device_handle = self._dll.ZCAN_OpenDevice(device_type, device_index, 0)
        
        if self._device_handle == 0:
            raise CANError(
                f"Failed to open device {device_type.name}. "
                f"The device may be in use by another program."
            )
        
        return True
    
    def init_canfd(self, arbitration_bps: int = 1_000_000,
                   data_bps: int = 5_000_000,
                   internal_resistance: bool = True) -> bool:
        """
        Initialize CAN FD channel.
        
        Args:
            arbitration_bps: Arbitration phase bitrate
            data_bps: Data phase bitrate
            internal_resistance: Enable 120Ω internal termination
            
        Returns:
            True if successful
            
        Raises:
            CANError: If initialization fails
        """
        if self._device_handle == 0:
            raise CANError("Device not opened")
        
        chn = self._channel
        
        # Set CAN FD standard (0 = ISO CAN FD)
        self._dll.ZCAN_SetValue(
            self._device_handle,
            f"{chn}/canfd_standard".encode(),
            b"0"
        )
        
        # Set internal resistance
        res_val = b"1" if internal_resistance else b"0"
        self._dll.ZCAN_SetValue(
            self._device_handle,
            f"{chn}/initenal_resistance".encode(),
            res_val
        )
        
        # Set bitrates
        ret = self._dll.ZCAN_SetValue(
            self._device_handle,
            f"{chn}/canfd_abit_baud_rate".encode(),
            str(arbitration_bps).encode()
        )
        if ret != ZCAN_STATUS_OK:
            raise CANError(f"Failed to set arbitration bitrate {arbitration_bps}")
        
        ret = self._dll.ZCAN_SetValue(
            self._device_handle,
            f"{chn}/canfd_dbit_baud_rate".encode(),
            str(data_bps).encode()
        )
        if ret != ZCAN_STATUS_OK:
            raise CANError(f"Failed to set data bitrate {data_bps}")
        
        # Initialize channel
        init_cfg = ZCANChannelInitConfig()
        init_cfg.can_type = ZCAN_TYPE_CANFD
        init_cfg.config.canfd.mode = 0  # Normal mode
        
        self._channel_handle = self._dll.ZCAN_InitCAN(
            self._device_handle, chn, byref(init_cfg)
        )
        if self._channel_handle == 0:
            raise CANError(f"Failed to initialize CAN channel {chn}")
        
        # Setup filter (accept all)
        self._setup_filter()
        
        # Start CAN
        ret = self._dll.ZCAN_StartCAN(self._channel_handle)
        if ret != ZCAN_STATUS_OK:
            raise CANError("Failed to start CAN")
        
        self._is_open_flag = True
        return True
    
    def _setup_filter(self):
        """Setup CAN filter to accept all frames."""
        chn = self._channel
        self._dll.ZCAN_SetValue(self._device_handle, f"{chn}/filter_clear".encode(), b"0")
        self._dll.ZCAN_SetValue(self._device_handle, f"{chn}/filter_mode".encode(), b"0")
        self._dll.ZCAN_SetValue(self._device_handle, f"{chn}/filter_start".encode(), b"0")
        self._dll.ZCAN_SetValue(self._device_handle, f"{chn}/filter_end".encode(), b"0x7FF")
        self._dll.ZCAN_SetValue(self._device_handle, f"{chn}/filter_ack".encode(), b"0")
    
    def send(self, frame: CANFrame) -> bool:
        """Send a CAN frame."""
        if isinstance(frame, CANFDFrame) or frame.frame_type == FrameType.CANFD:
            return self._send_canfd(frame)
        return self._send_can(frame)
    
    def _send_can(self, frame: CANFrame) -> bool:
        """Send standard CAN frame."""
        if not self._is_open_flag:
            raise CANError("CAN not initialized")
        
        if len(frame.data) > 8:
            raise ValueError("Standard CAN frame data max 8 bytes")
        
        tx_data = ZCANTransmitData()
        tx_data.frame.can_id = frame.can_id
        tx_data.frame.eff = 1 if frame.is_extended else 0
        tx_data.frame.rtr = 1 if frame.is_remote else 0
        tx_data.frame.can_dlc = len(frame.data)
        tx_data.transmit_type = 0
        
        data_bytes = frame.data if isinstance(frame.data, bytes) else bytes(frame.data)
        for i in range(len(data_bytes)):
            tx_data.frame.data[i] = data_bytes[i]
        
        ret = self._dll.ZCAN_Transmit(self._channel_handle, byref(tx_data), 1)
        return ret == 1
    
    def _send_canfd(self, frame: CANFDFrame) -> bool:
        """Send CAN FD frame."""
        if not self._is_open_flag:
            raise CANError("CAN not initialized")
        
        tx_data = ZCANTransmitFDData()
        tx_data.frame.can_id = frame.can_id
        tx_data.frame.eff = 1 if frame.is_extended else 0
        tx_data.frame.rtr = 1 if frame.is_remote else 0
        tx_data.frame.brs = 1 if frame.bitrate_switch else 0
        tx_data.frame.len = len(frame.data)
        tx_data.transmit_type = 0
        
        data_bytes = frame.data if isinstance(frame.data, bytes) else bytes(frame.data)
        for i in range(len(data_bytes)):
            tx_data.frame.data[i] = data_bytes[i]
        
        ret = self._dll.ZCAN_TransmitFD(self._channel_handle, byref(tx_data), 1)
        return ret == 1
    
    def receive(self, timeout_ms: int = 100, frame_type: str = "any") -> Optional[CANFrame]:
        """
        Receive a CAN frame.
        
        Args:
            timeout_ms: Timeout in milliseconds
            frame_type: "any", "CAN", or "CANFD"
            
        Returns:
            Received frame or None if timeout
        """
        if not self._is_open_flag:
            raise CANError("CAN not initialized")
        
        if frame_type == "CAN":
            return self._receive_can(timeout_ms)
        elif frame_type == "CANFD":
            return self._receive_canfd(timeout_ms)
        
        # Try CAN FD first, then standard CAN
        frame = self._receive_canfd(0)
        if frame:
            return frame
        return self._receive_can(timeout_ms)
    
    def _receive_can(self, timeout_ms: int) -> Optional[CANFrame]:
        """Receive standard CAN frame."""
        rx_num = self._dll.ZCAN_GetReceiveNum(self._channel_handle, ZCAN_TYPE_CAN)
        if rx_num == 0:
            return None
        
        rx_data = ZCANReceiveData()
        ret = self._dll.ZCAN_Receive(self._channel_handle, byref(rx_data), 1, timeout_ms)
        
        if ret <= 0:
            return None
        
        data_len = min(rx_data.frame.can_dlc, 8)
        data = bytes(rx_data.frame.data[:data_len])
        
        return CANFrame(
            can_id=rx_data.frame.can_id,
            data=data,
            is_extended=bool(rx_data.frame.eff),
            is_remote=bool(rx_data.frame.rtr),
            frame_type=FrameType.CAN
        )
    
    def _receive_canfd(self, timeout_ms: int) -> Optional[CANFDFrame]:
        """Receive CAN FD frame."""
        rx_num = self._dll.ZCAN_GetReceiveNum(self._channel_handle, ZCAN_TYPE_CANFD)
        if rx_num == 0:
            return None
        
        rx_data = ZCANReceiveFDData()
        ret = self._dll.ZCAN_ReceiveFD(self._channel_handle, byref(rx_data), 1, timeout_ms)
        
        if ret <= 0:
            return None
        
        data_len = min(rx_data.frame.len, 64)
        data = bytes(rx_data.frame.data[:data_len])
        
        return CANFDFrame(
            can_id=rx_data.frame.can_id,
            data=data,
            is_extended=bool(rx_data.frame.eff),
            is_remote=bool(rx_data.frame.rtr),
            bitrate_switch=bool(rx_data.frame.brs),
            frame_type=FrameType.CANFD
        )
    
    def is_open(self) -> bool:
        """Check if the interface is open."""
        return self._is_open_flag
    
    def close(self):
        """Close the device and cleanup."""
        if not self._is_open_flag and not self._device_handle:
            return
        
        try:
            if self._channel_handle:
                self._dll.ZCAN_ClearBuffer(self._channel_handle)
                self._dll.ZCAN_ResetCAN(self._channel_handle)
                self._channel_handle = 0
            
            if self._device_handle:
                self._dll.ZCAN_CloseDevice(self._device_handle)
                self._device_handle = 0
            
            self._is_open_flag = False
            
        except Exception:
            # Force cleanup on error
            self._channel_handle = 0
            self._device_handle = 0
            self._is_open_flag = False
            raise
    
    def clear_buffer(self):
        """Clear receive buffer."""
        if self._channel_handle:
            self._dll.ZCAN_ClearBuffer(self._channel_handle)
    
    def get_receive_count(self, frame_type: str = "any") -> int:
        """Get number of frames in receive buffer."""
        if not self._is_open_flag:
            return 0
        
        if frame_type == "CAN":
            return self._dll.ZCAN_GetReceiveNum(self._channel_handle, ZCAN_TYPE_CAN)
        elif frame_type == "CANFD":
            return self._dll.ZCAN_GetReceiveNum(self._channel_handle, ZCAN_TYPE_CANFD)
        else:
            can = self._dll.ZCAN_GetReceiveNum(self._channel_handle, ZCAN_TYPE_CAN)
            canfd = self._dll.ZCAN_GetReceiveNum(self._channel_handle, ZCAN_TYPE_CANFD)
            return can + canfd
