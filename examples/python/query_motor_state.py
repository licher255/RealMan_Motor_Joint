"""
Query RealMan WHJ Motor State
Send query commands and display motor status
"""

import sys
import time
from zlgcan_driver import ZlgCanDriver, ZCANDeviceType
from whj_protocol import WHJProtocol, Register, ErrorCode


def send_wakeup_command(driver, motor_id, timeout_ms=100):
    """Send wake-up command (write 0x00 to IAP_FLAG 0x49)"""
    # Command format matching ZCANPro: CMD(0x02) + INDEX(0x49) + DATA(0x00)
    # ZCANPro sends: d 3 3 02 49 00 (DLC=3, data=02 49 00)
    cmd = bytes([0x02, 0x49, 0x00])  # Write 0 to IAP_FLAG (3 bytes)
    
    driver.clear_buffer()
    
    if not driver.send(can_id=motor_id, data=cmd):
        return False
    
    # Wait for response (optional, motor may not respond)
    response_id = motor_id + 0x100
    start_time = time.time()
    
    while (time.time() - start_time) * 1000 < timeout_ms:
        frame = driver.receive(timeout_ms=50)
        if frame and frame.can_id == response_id:
            return True  # Got response
        time.sleep(0.001)
    
    return True  # Even if no response, command was sent


def query_register(driver, motor_id, reg, count=1, timeout_ms=200):
    """Query a register and return response data"""
    cmd = WHJProtocol.build_read_frame(motor_id, reg, count)
    
    # Clear buffer
    driver.clear_buffer()
    
    # Send command
    tx_success = driver.send(can_id=motor_id, data=cmd)
    if not tx_success:
        return None, "Failed to send"
    
    # Wait for response
    response_id = motor_id + 0x100
    start_time = time.time()
    
    while (time.time() - start_time) * 1000 < timeout_ms:
        frame = driver.receive(timeout_ms=50)
        if frame and frame.can_id == response_id:
            return frame.data, None
        time.sleep(0.001)
    
    return None, "Timeout"


def parse_32bit_value(low, high):
    """Parse two 16-bit values to signed 32-bit"""
    val = (high << 16) | low
    if val & 0x80000000:
        val -= 0x100000000
    return val


def main():
    motor_id = int(sys.argv[1]) if len(sys.argv) > 1 else 1
    
    print("=" * 60)
    print("RealMan WHJ Motor State Query")
    print("=" * 60)
    print(f"Motor ID: {motor_id}")
    print()
    
    # Initialize CAN
    driver = ZlgCanDriver()
    driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0)
    driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
    
    print("Querying motor state...")
    print("-" * 60)
    
    # 0. Send wake-up command (required after device power-on)
    print("\n[0] Sending wake-up command (IAP_FLAG = 0)...")
    if send_wakeup_command(driver, motor_id):
        print("    Wake-up command sent")
        time.sleep(0.1)  # Wait for motor to initialize
    else:
        print("    Warning: Failed to send wake-up command")
    
    # 1. Query System Info (Model, Firmware, Voltage, Temp)
    print("\n[1] System Information")
    data, err = query_register(driver, motor_id, Register.SYS_MODEL_TYPE, 6)
    if data:
        try:
            # Parse: CMD(1) + INDEX(1) + DATA(12)
            if len(data) >= 14:
                model = data[2] | (data[3] << 8)
                fw_ver = data[4] | (data[5] << 8)
                voltage_raw = data[6] | (data[7] << 8)
                temp_raw = data[8] | (data[9] << 8)
                redu_ratio = data[10] | (data[11] << 12)
                
                model_names = {
                    0x02: "J14 (Joint 10)",
                    0x03: "J17 (Joint 30)",
                    0x04: "J20 (Joint 60)",
                    0x05: "J25 (Joint 120)",
                    0x06: "Gripper",
                    0x07: "J3 (Joint 03)",
                }
                model_name = model_names.get(model, f"Unknown (0x{model:02X})")
                
                print(f"    Model:           {model_name}")
                print(f"    Firmware:        v{fw_ver >> 8}.{fw_ver & 0xFF}")
                print(f"    Voltage:         {voltage_raw * 0.01:.2f} V")
                print(f"    Temperature:     {temp_raw * 0.1:.1f} °C")
                print(f"    Reduction Ratio: {redu_ratio}")
            else:
                print(f"    Data: {data.hex()} (length={len(data)})")
        except Exception as e:
            print(f"    Parse error: {e}, Data: {data.hex()}")
    else:
        print(f"    Failed: {err}")
    
    # 2. Query Error Code
    print("\n[2] Error Status")
    data, err = query_register(driver, motor_id, Register.SYS_ERROR, 1)
    if data:
        try:
            if len(data) >= 4:
                error_code = data[2] | (data[3] << 8)
                print(f"    Error Code: 0x{error_code:04X}")
                
                if error_code == 0:
                    print("    Status: No errors")
                else:
                    errors = ErrorCode.parse(error_code)
                    print("    Errors:")
                    for e in errors:
                        print(f"      - {e}")
            else:
                print(f"    Data: {data.hex()}")
        except Exception as e:
            print(f"    Parse error: {e}")
    else:
        print(f"    Failed: {err}")
    
    # 3. Query Enable Status
    print("\n[3] Driver Status")
    data, err = query_register(driver, motor_id, Register.SYS_ENABLE_DRIVER, 1)
    if data:
        try:
            if len(data) >= 4:
                enabled = data[2] | (data[3] << 8)
                print(f"    Driver Enabled: {'Yes' if enabled else 'No'}")
            else:
                print(f"    Data: {data.hex()}")
        except Exception as e:
            print(f"    Parse error: {e}")
    else:
        print(f"    Failed: {err}")
    
    # 4. Query Work Mode
    print("\n[4] Work Mode")
    data, err = query_register(driver, motor_id, Register.TAG_WORK_MODE, 1)
    if data:
        try:
            if len(data) >= 4:
                mode = data[2] | (data[3] << 8)
                mode_names = {
                    0: "OPEN_LOOP (Open Loop)",
                    1: "CURRENT_MODE (Current/Torque)",
                    2: "SPEED_MODE (Speed)",
                    3: "POSITION_MODE (Position)",
                }
                mode_name = mode_names.get(mode, f"Unknown ({mode})")
                print(f"    Work Mode: {mode_name}")
            else:
                print(f"    Data: {data.hex()}")
        except Exception as e:
            print(f"    Parse error: {e}")
    else:
        print(f"    Failed: {err}")
    
    # 5. Query Current Position
    print("\n[5] Position Feedback")
    data, err = query_register(driver, motor_id, Register.CUR_POSITION_L, 2)
    if data:
        try:
            if len(data) >= 6:
                pos_low = data[2] | (data[3] << 8)
                pos_high = data[4] | (data[5] << 8)
                pos_raw = parse_32bit_value(pos_low, pos_high)
                pos_deg = pos_raw * 0.0001
                print(f"    Raw Value:  {pos_raw}")
                print(f"    Position:   {pos_deg:.4f}°")
            else:
                print(f"    Data: {data.hex()}")
        except Exception as e:
            print(f"    Parse error: {e}")
    else:
        print(f"    Failed: {err}")
    
    # 6. Query Current Speed
    print("\n[6] Speed Feedback")
    data, err = query_register(driver, motor_id, Register.CUR_SPEED_L, 2)
    if data:
        try:
            if len(data) >= 6:
                spd_low = data[2] | (data[3] << 8)
                spd_high = data[4] | (data[5] << 8)
                spd_raw = parse_32bit_value(spd_low, spd_high)
                spd_rpm = spd_raw * 0.02
                print(f"    Raw Value:  {spd_raw}")
                print(f"    Speed:      {spd_rpm:.2f} RPM")
            else:
                print(f"    Data: {data.hex()}")
        except Exception as e:
            print(f"    Parse error: {e}")
    else:
        print(f"    Failed: {err}")
    
    # 7. Query Current
    print("\n[7] Current Feedback")
    data, err = query_register(driver, motor_id, Register.CUR_CURRENT_L, 2)
    if data:
        try:
            if len(data) >= 6:
                cur_low = data[2] | (data[3] << 8)
                cur_high = data[4] | (data[5] << 8)
                cur_raw = parse_32bit_value(cur_low, cur_high)
                print(f"    Raw Value:  {cur_raw}")
                print(f"    Current:    {cur_raw} mA")
            else:
                print(f"    Data: {data.hex()}")
        except Exception as e:
            print(f"    Parse error: {e}")
    else:
        print(f"    Failed: {err}")
    
    # 8. Query Target Position (if in position mode)
    print("\n[8] Target Position")
    data, err = query_register(driver, motor_id, Register.TAG_POSITION_L, 2)
    if data:
        try:
            if len(data) >= 6:
                pos_low = data[2] | (data[3] << 8)
                pos_high = data[4] | (data[5] << 8)
                pos_raw = parse_32bit_value(pos_low, pos_high)
                pos_deg = pos_raw * 0.0001
                print(f"    Raw Value:  {pos_raw}")
                print(f"    Target:     {pos_deg:.4f}°")
            else:
                print(f"    Data: {data.hex()}")
        except Exception as e:
            print(f"    Parse error: {e}")
    else:
        print(f"    Failed: {err}")
    
    print()
    print("-" * 60)
    print("Query complete!")
    
    driver.close()


if __name__ == "__main__":
    main()
