"""
Abstract CAN interface definition.

This module defines the abstract base class for CAN interfaces,
allowing different CAN hardware implementations.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, List
from enum import Enum


class FrameType(Enum):
    """CAN frame type."""
    CAN = "CAN"
    CANFD = "CANFD"


@dataclass
class CANFrame:
    """Standard CAN frame (CAN 2.0)."""
    can_id: int
    data: bytes
    is_extended: bool = False
    is_remote: bool = False
    frame_type: FrameType = FrameType.CAN
    
    def __post_init__(self):
        if len(self.data) > 8:
            raise ValueError("Standard CAN frame data max 8 bytes")


@dataclass  
class CANFDFrame(CANFrame):
    """CAN FD frame."""
    bitrate_switch: bool = True
    frame_type: FrameType = FrameType.CANFD
    
    def __post_init__(self):
        if len(self.data) > 64:
            raise ValueError("CAN FD frame data max 64 bytes")


class CANInterface(ABC):
    """
    Abstract base class for CAN interfaces.
    
    All CAN hardware drivers should inherit from this class.
    """
    
    @abstractmethod
    def open(self, **kwargs) -> bool:
        """Open the CAN device."""
        pass
    
    @abstractmethod
    def close(self) -> None:
        """Close the CAN device."""
        pass
    
    @abstractmethod
    def send(self, frame: CANFrame) -> bool:
        """Send a CAN frame."""
        pass
    
    @abstractmethod
    def receive(self, timeout_ms: int = 100, frame_type: str = "any") -> Optional[CANFrame]:
        """Receive a CAN frame."""
        pass
    
    @abstractmethod
    def is_open(self) -> bool:
        """Check if the interface is open."""
        pass
    
    def send_can(self, can_id: int, data: bytes, is_extended: bool = False) -> bool:
        """Convenience method to send standard CAN frame."""
        frame = CANFrame(
            can_id=can_id,
            data=data,
            is_extended=is_extended,
            frame_type=FrameType.CAN
        )
        return self.send(frame)
    
    def send_canfd(self, can_id: int, data: bytes, is_extended: bool = False, 
                   bitrate_switch: bool = True) -> bool:
        """Convenience method to send CAN FD frame."""
        frame = CANFDFrame(
            can_id=can_id,
            data=data,
            is_extended=is_extended,
            bitrate_switch=bitrate_switch,
            frame_type=FrameType.CANFD
        )
        return self.send(frame)
    
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False
