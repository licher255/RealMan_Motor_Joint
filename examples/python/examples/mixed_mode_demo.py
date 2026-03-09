"""
Mixed Mode Demo - CAN FD + Standard CAN Compatibility

This example demonstrates how to use the ZLG CAN FD driver in mixed mode
to communicate with both CAN FD devices (WHJ motors) and standard CAN
devices (Kinco motors) on the same CAN bus.

Hardware Requirements:
- ZLG USBCANFD-100U-mini (or other CAN FD capable device)
- WHJ motor (CAN FD compatible)
- Kinco motor (Standard CAN)

Wiring:
- Connect CAN_H of all devices together
- Connect CAN_L of all devices together
- Ensure 120Ω termination at both ends of the bus

Usage:
    python mixed_mode_demo.py

Note:
    In mixed mode, both CAN and CAN FD devices share the same arbitration
    bitrate (e.g., 1Mbps), but CAN FD devices can use faster data bitrate
    (e.g., 5Mbps) for the data phase.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from core.zlgcan_driver import ZlgCanDriver, ZCANDeviceType, CANFrame, CANFDFrame
import time


def main():
    print("=" * 70)
    print("Mixed Mode Demo: WHJ (CAN FD) + Kinco (Standard CAN)")
    print("=" * 70)
    
    driver = None
    try:
        # Initialize driver
        driver = ZlgCanDriver()
        
        # Open device
        print("\n[1] Opening CAN device...")
        driver.open(ZCANDeviceType.USBCANFD_MINI, device_index=0, channel=0)
        
        # Initialize mixed mode
        print("\n[2] Initializing Mixed Mode (CAN FD + CAN compatibility)...")
        driver.init_mixed_mode(
            arbitration_bps=1000000,  # 1Mbps - shared by both CAN and CAN FD
            data_bps=5000000,         # 5Mbps - only used by CAN FD devices
            internal_resistance=True  # Enable 120Ω termination
        )
        print("    Mode: CAN FD Mixed Mode")
        print("    Arbitration Bitrate: 1 Mbps (CAN & CAN FD)")
        print("    Data Bitrate: 5 Mbps (CAN FD only)")
        
        # Example: Send to Kinco motor (Standard CAN)
        print("\n[3] Sending commands to Kinco motor (Standard CAN)...")
        print("    Kinco SDO protocol example:")
        
        # Kinco node ID (change according to your setup)
        kinco_node_id = 1
        kinco_cob_id = 0x600 + kinco_node_id  # SDO client-to-server
        
        # SDO read object 0x6041 (Status Word)
        # Format: [ccs=1, index_low, index_high, subindex, 0, 0, 0, 0]
        sdo_read_status = bytes([0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00])
        success = driver.send_can(can_id=kinco_cob_id, data=sdo_read_status)
        print(f"    SDO Read Status Word (0x6041): {'OK' if success else 'FAILED'}")
        time.sleep(0.1)
        
        # SDO write object 0x6040 (Control Word) to enable operation
        sdo_write_control = bytes([0x2B, 0x40, 0x60, 0x00, 0x0F, 0x00, 0x00, 0x00])
        success = driver.send_can(can_id=kinco_cob_id, data=sdo_write_control)
        print(f"    SDO Write Control Word (0x6040): {'OK' if success else 'FAILED'}")
        
        # Example: Send to WHJ motor (CAN FD)
        print("\n[4] Sending commands to WHJ motor (CAN FD)...")
        print("    WHJ Protocol example:")
        
        # WHJ motor ID
        whj_motor_id = 1
        
        # Read current position (register 0x14 - CUR_POSITION_L)
        # Format: [command=0x01, reg_low, reg_high, count]
        whj_read_pos = bytes([0x01, 0x14, 0x00, 0x02])  # Read 2 words (4 bytes)
        success = driver.send_canfd(
            can_id=whj_motor_id, 
            data=whj_read_pos,
            bitrate_switch=True
        )
        print(f"    Read Position (reg 0x14): {'OK' if success else 'FAILED'}")
        time.sleep(0.1)
        
        # Enable motor (register 0x0A - SYS_ENABLE_DRIVER)
        whj_enable = bytes([0x02, 0x0A, 0x00, 0x01, 0x01, 0x00])  # Write 1 to enable
        success = driver.send_canfd(
            can_id=whj_motor_id,
            data=whj_enable,
            bitrate_switch=True
        )
        print(f"    Enable Motor (reg 0x0A): {'OK' if success else 'FAILED'}")
        time.sleep(0.1)
        
        # Example: Mixed communication pattern
        print("\n[5] Mixed communication pattern (query both motors)...")
        
        for cycle in range(3):
            print(f"\n    --- Cycle {cycle + 1} ---")
            
            # Query Kinco
            driver.send_can(can_id=kinco_cob_id, data=sdo_read_status)
            print(f"    Sent CAN to Kinco (ID=0x{kinco_cob_id:03X})")
            
            time.sleep(0.01)
            
            # Query WHJ
            driver.send_canfd(can_id=whj_motor_id, data=whj_read_pos)
            print(f"    Sent CAN FD to WHJ (ID=0x{whj_motor_id:03X})")
            
            time.sleep(0.05)
            
            # Process responses
            status = driver.get_mixed_mode_status()
            print(f"    Buffer: CAN={status['can']}, CAN FD={status['canfd']}")
            
            # Receive and print all pending frames
            while True:
                frame = driver.receive(timeout_ms=10)
                if frame is None:
                    break
                
                frame_type = getattr(frame, 'frame_type', 'CAN')
                id_str = f"0x{frame.can_id:03X}"
                data_preview = frame.data[:8].hex()
                if len(frame.data) > 8:
                    data_preview += f"...({len(frame.data)}B)"
                
                print(f"      RX [{frame_type:5}] ID={id_str:<8} Data={data_preview}")
            
            time.sleep(0.2)
        
        # Final statistics
        print("\n[6] Final Statistics")
        print("-" * 70)
        status = driver.get_mixed_mode_status()
        print(f"    Total CAN frames:   {status['can']}")
        print(f"    Total CAN FD frames: {status['canfd']}")
        print(f"    Grand Total:        {status['total']}")
        
    except KeyboardInterrupt:
        print("\n\n[Interrupted by user]")
    except Exception as e:
        print(f"\n[Error] {e}")
        import traceback
        traceback.print_exc()
    finally:
        if driver:
            print("\n[Cleanup] Closing device...")
            driver.close()
    
    print("\n" + "=" * 70)
    print("Demo completed!")
    print("=" * 70)


if __name__ == "__main__":
    main()
