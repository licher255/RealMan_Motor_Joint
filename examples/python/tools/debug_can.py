"""
Simple CAN debug tool - Monitor and test raw CAN communication
"""

import sys
import time
from zlgcan_driver import ZlgCanDriver, ZCANDeviceType, CANFDFrame
from whj_protocol import WHJProtocol, Register


def main():
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    
    print("=" * 60)
    print("CAN Debug Tool")
    print("=" * 60)
    print(f"Target Motor ID: {motor_id}")
    print(f"Expected Response ID: 0x{motor_id + 0x100:03X}")
    print()
    
    driver = ZlgCanDriver()
    driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
    driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
    
    print("\n[1] Ping Test - Send read firmware version command")
    cmd = WHJProtocol.build_read_frame(motor_id, Register.SYS_FW_VERSION, 1)
    print(f"    TX: ID=0x{motor_id:03X}, Data={cmd.hex()}")
    driver.send(can_id=motor_id, data=cmd)
    
    print("    Waiting for response (2 seconds)...")
    time.sleep(0.5)
    
    response_count = 0
    for _ in range(20):  # Check for 2 seconds
        frame = driver.receive(timeout_ms=100)
        if frame:
            response_count += 1
            print(f"    RX: ID=0x{frame.can_id:03X}, Data={frame.data.hex()}, Len={frame.len}")
            
            # Check if it's our response
            if frame.can_id == motor_id + 0x100:
                print("    --> This is our motor's response!")
            elif frame.can_id > 0x100 and frame.can_id <= 0x11E:
                other_id = frame.can_id - 0x100
                print(f"    --> This is motor {other_id}'s response!")
        
        time.sleep(0.1)
    
    if response_count == 0:
        print("\n    [WARNING] No response received!")
        print("    Possible reasons:")
        print("      - Wrong motor ID")
        print("      - Motor not powered on")
        print("      - CAN wiring issue")
        print("      - Motor still initializing (wait 2-3s after power on)")
    else:
        print(f"\n    Received {response_count} frame(s)")
    
    print("\n[2] Continuous Monitor Mode (press Ctrl+C to stop)")
    print("    Listening for all CAN frames...")
    try:
        while True:
            frame = driver.receive(timeout_ms=100)
            if frame:
                # Determine frame type
                if frame.can_id < 0x100:
                    src = f"Request (Motor {frame.can_id})"
                elif frame.can_id < 0x200:
                    motor = frame.can_id - 0x100
                    src = f"Response (Motor {motor})"
                else:
                    src = f"Other (0x{frame.can_id:03X})"
                
                print(f"    [{src}] ID=0x{frame.can_id:03X}, Data={frame.data.hex()}")
            
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n    Stopped")
    
    driver.close()
    print("\nDone!")


if __name__ == "__main__":
    main()
