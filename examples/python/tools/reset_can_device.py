"""
Force reset ZLG CAN device
Use this when the device is stuck in use
"""

import sys
import time
from zlgcan_driver import ZlgCanDriver, ZCANDeviceType

def main():
    print("=" * 60)
    print("ZLG CAN Device Force Reset")
    print("=" * 60)
    print()
    
    try:
        driver = ZlgCanDriver()
        
        # Try to open with reset
        print("Attempting to open and reset device...")
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        
        # If opened, immediately close
        print("Device opened, closing...")
        driver.close()
        
        print()
        print("[SUCCESS] Device has been reset")
        print("The CAN LED should be off now")
        print()
        print("You can now run your control scripts")
        
    except RuntimeError as e:
        print(f"[ERROR] {e}")
        print()
        print("If the device is still stuck:")
        print("1. Unplug the USB cable")
        print("2. Wait 5 seconds")
        print("3. Plug it back in")
        print("4. Run this script again")

if __name__ == "__main__":
    main()
