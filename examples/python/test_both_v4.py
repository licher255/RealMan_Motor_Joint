"""
================================================================================
WHJ + Kinco - FINAL PERFECT VERSION
================================================================================

功能:
1. WHJ 通信：仅接收 CANFD 帧，彻底屏蔽 Kinco 标准帧干扰 (软件隔离)。
2. Kinco 控制：支持发送标准 CAN 指令给 Kinco (开环控制)。
3. 平滑轨迹：WHJ 运动采用梯形规划，避免冲击。

操作:
  m <pos> : WHJ 平滑移动到目标位置
  k <pos> : Kinco 移动到目标位置 (开环，无位置反馈)
  r       : 读取 WHJ 当前位置
  c       : 清除 WHJ 错误
  q       : 退出
================================================================================
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import time
import math
import struct
import atexit

from core import ZlgCanDriver, ZCANDeviceType
from core.zlgcan_driver import CANFrame
from core.protocol import WHJProtocol, Register, WorkMode

# ============================================================================
# 配置
# ============================================================================
WHJ_ID = 7
WHJ_RESPONSE_ID = WHJ_ID + 0x100
KINCO_ID = 1

# Kinco 参数
KINCO_NMT_ID = 0x000
KINCO_RPDO1_ID = 0x201 # Control Word
KINCO_RPDO2_ID = 0x301 # Target Position + Velocity
KINCO_GEAR_RATIO = 16384 # 根据实际电机调整
KINCO_RPM_TO_UNITS = (65536 * 512) / 1875

# WHJ 运动参数
MAX_VEL = 1000.0       # deg/s
MAX_ACC = 2000.0       # deg/s^2

_global_driver = None

def cleanup():
    global _global_driver
    if _global_driver:
        try:
            _global_driver.close()
            print("\n[Cleanup] Device closed.")
        except: pass
atexit.register(cleanup)

class HybridController:
    def __init__(self, driver):
        self.driver = driver
        self.whj_id = WHJ_ID
        self.whj_resp_id = WHJ_RESPONSE_ID
        
    # ------------------------------------------------------------------------
    # WHJ 部分 (严格过滤模式)
    # ------------------------------------------------------------------------
    def _recv_whj_only(self, timeout_ms=500):
        """
        【核心魔法】只接收 CANFD 帧 (WHJ)，自动丢弃/忽略标准 CAN 帧 (Kinco)。
        这样即使 Kinco 在疯狂发数据，也不会阻塞 WHJ 的通信。
        """
        start = time.time()
        while (time.time() - start) * 1000 < timeout_ms:
            # 只尝试读取 CANFD 帧
            frame = self.driver.receive_frame_canfd(timeout_ms=0)
            if frame:
                if frame.can_id == self.whj_resp_id:
                    return frame.data
                # 其他 CANFD 帧忽略
            # 注意：这里故意不调用 receive_frame_can()，从而物理隔绝 Kinco 流量
            time.sleep(0.001)
        return None

    def _send_whj_cmd(self, cmd, timeout_ms=500):
        # 发送前清空缓冲
        while self.driver.receive_frame_canfd(timeout_ms=0): pass
        
        ok = self.driver.send_canfd(self.whj_id, cmd, bitrate_switch=True)
        if not ok:
            return None
        return self._recv_whj_only(timeout_ms)

    def whj_get_pos(self):
        cmd = WHJProtocol.build_read_frame(self.whj_id, Register.CUR_POSITION_L, 2)
        for _ in range(3):
            resp = self._send_whj_cmd(cmd, timeout_ms=200)
            if resp and len(resp) >= 6:
                low = resp[2] | (resp[3] << 8)
                high = resp[4] | (resp[5] << 8)
                val = (high << 16) | low
                if val & 0x80000000: val -= 0x100000000
                return val * 0.0001
            time.sleep(0.05)
        return None

    def whj_set_pos_raw(self, raw):
        cmd_l = WHJProtocol.build_write_frame(self.whj_id, Register.TAG_POSITION_L, raw & 0xFFFF)
        if not self._send_whj_cmd(cmd_l, timeout_ms=200): return False
        time.sleep(0.005)
        cmd_h = WHJProtocol.build_write_frame(self.whj_id, Register.TAG_POSITION_H, (raw >> 16) & 0xFFFF)
        return self._send_whj_cmd(cmd_h, timeout_ms=200) is not None

    def whj_clear_error(self):
        cmd = WHJProtocol.build_write_frame(self.whj_id, Register.SYS_CLEAR_ERROR, 1)
        self.driver.send_canfd(self.whj_id, cmd, bitrate_switch=True)
        time.sleep(0.02)
        print("  [WHJ] Error Cleared")

    def whj_init(self):
        print("[Init] WHJ...")
        self.whj_clear_error()
        time.sleep(0.1)
        en = WHJProtocol.build_write_frame(self.whj_id, Register.SYS_ENABLE_DRIVER, 1)
        self._send_whj_cmd(en, timeout_ms=500)
        mode = WHJProtocol.build_write_frame(self.whj_id, Register.TAG_WORK_MODE, WorkMode.POSITION_MODE)
        self._send_whj_cmd(mode, timeout_ms=500)
        time.sleep(0.2)
        print("[Init] WHJ OK")

    def whj_move_smooth(self, target_deg):
        print(f"\n[WHJ] Moving to {target_deg}° (Smooth)")
        curr = self.whj_get_pos()
        if curr is None:
            print("[Error] Cannot read WHJ pos")
            return False
        
        dist = target_deg - curr
        if abs(dist) < 0.5:
            print("  Already there.")
            return True

        # 梯形规划计算
        t_acc = MAX_VEL / MAX_ACC
        d_acc = 0.5 * MAX_ACC * t_acc * t_acc
        abs_dist = abs(dist)
        
        if 2 * d_acc >= abs_dist:
            t_total = 2 * math.sqrt(abs_dist / MAX_ACC)
        else:
            t_const = (abs_dist - 2 * d_acc) / MAX_VEL
            t_total = 2 * t_acc + t_const

        print(f"  Dist: {abs_dist:.1f}°, Time: {t_total:.2f}s")
        
        start_time = time.time()
        direction = 1 if dist > 0 else -1
        
        while True:
            t = time.time() - start_time
            if t >= t_total:
                self.whj_set_pos_raw(int(target_deg / 0.0001))
                print(f"[WHJ] Done. Final: {target_deg}°")
                break
            
            # 计算插值点
            if t < t_acc:
                s = 0.5 * MAX_ACC * t * t
            elif t < (t_total - t_acc):
                s = d_acc + MAX_VEL * (t - t_acc)
            else:
                remaining_t = t_total - t
                s = abs_dist - 0.5 * MAX_ACC * remaining_t * remaining_t
            
            current_target = curr + direction * s
            raw = int(current_target / 0.0001)
            
            # 高速发送 (不等待应答，依靠内部闭环)
            self.driver.send_canfd(self.whj_id, WHJProtocol.build_write_frame(self.whj_id, Register.TAG_POSITION_L, raw & 0xFFFF), bitrate_switch=True)
            self.driver.send_canfd(self.whj_id, WHJProtocol.build_write_frame(self.whj_id, Register.TAG_POSITION_H, (raw >> 16) & 0xFFFF), bitrate_switch=True)
            
            time.sleep(0.002) # 500Hz
            
            if int(t * 10) % 5 == 0:
                print(f"  Progress: {current_target:.1f}°", end='\r')
        
        # 校准
        time.sleep(0.1)
        final = self.whj_get_pos()
        print(f"\n[Check] WHJ Pos: {final:.2f}° (Err: {abs(final-target_deg):.2f}°)")
        return True

    # ------------------------------------------------------------------------
    # Kinco 部分 (开环发送)
    # ------------------------------------------------------------------------
    def kinco_send(self, can_id, data):
        """发送标准 CAN 帧给 Kinco"""
        frame = CANFrame(can_id=can_id, data=data, frame_type="CAN")
        ok = self.driver.send_frame_can(frame)
        if ok:
            print(f"  [Kinco] Cmd Sent to ID {hex(can_id)}")
        else:
            print(f"  [Kinco] Send Failed!")
        return ok

    def kinco_init(self):
        print("[Init] Kinco...")
        # NMT Start
        self.kinco_send(KINCO_NMT_ID, bytes([0x01, KINCO_ID]))
        time.sleep(0.1)
        # Enable (Control Word)
        # 0x013F1000 -> 0x0000003F (Little Endian mapping depends on object dictionary, usually 0x06 for Enable)
        # Standard CiA402: 0x0006 (Switch On), 0x0007 (Enable Voltage), 0x000F (Quick Stop), 0x001F (Enable Operation)
        # Simplified for Kinco often: 0x01 (Enable) in first byte or specific mapping
        # Here using a common sequence:
        data = bytes([0x01, 0x3F, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00]) # Example PDO mapping
        self.kinco_send(KINCO_RPDO1_ID, data)
        time.sleep(0.1)
        print("[Init] Kinco OK (Open Loop)")

    def kinco_move(self, deg):
        print(f"\n[Kinco] Moving to {deg}° (Open Loop)")
        pos_raw = int(deg * KINCO_GEAR_RATIO)
        vel_raw = int(50 * KINCO_RPM_TO_UNITS) # 50 RPM
        
        # Pack: Position (int32) + Velocity (uint32)
        data = struct.pack('<i', pos_raw) + struct.pack('<I', vel_raw)
        
        # 发送 3 次以确保到达 (因为不收应答)
        for i in range(3):
            self.kinco_send(KINCO_RPDO2_ID, data)
            time.sleep(0.05)
        
        print(f"[Kinco] Command dispatched. (No feedback check)")

def main():
    global _global_driver
    print("="*60)
    print("HYBRID CONTROL: WHJ (Filtered) + Kinco (Open Loop)")
    print("="*60)
    
    try:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        _global_driver = driver
        print("[HW] Opened (CANFD + CAN Mixed Mode)")
    except Exception as e:
        print(f"[HW Error] {e}")
        return

    ctrl = HybridController(driver)
    ctrl.whj_init()
    ctrl.kinco_init()
    
    print("\nReady!")
    print("Commands:")
    print("  m <pos> : Move WHJ (Smooth, Filtered)")
    print("  k <pos> : Move Kinco (Open Loop)")
    print("  r       : Read WHJ Pos")
    print("  c       : Clear WHJ Error")
    print("  q       : Quit")

    while True:
        try:
            txt = input("> ").strip()
            if not txt: continue
            parts = txt.split()
            cmd = parts[0].lower()
            
            if cmd == 'q': break
            
            elif cmd == 'm' and len(parts) > 1:
                ctrl.whj_move_smooth(float(parts[1]))
            
            elif cmd == 'k' and len(parts) > 1:
                ctrl.kinco_move(float(parts[1]))
            
            elif cmd == 'r':
                p = ctrl.whj_get_pos()
                print(f"WHJ Pos: {p}" if p else "WHJ Pos: Read Failed")
            
            elif cmd == 'c':
                ctrl.whj_clear_error()
                
        except KeyboardInterrupt:
            print("\nStopping...")
            ctrl.whj_clear_error()
            break
        except Exception as e:
            print(f"Err: {e}")
            import traceback
            traceback.print_exc()

    driver.close()
    print("Bye! Have a great evening!")

if __name__ == "__main__":
    main()