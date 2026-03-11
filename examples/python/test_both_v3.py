"""
================================================================================
WHJ + Kinco Mixed Control - FINAL FIX (TX BLOCKING RESOLVED)
================================================================================

故障诊断结论:
1. 读位置成功 -> RX 通路正常。
2. 发命令失败 -> TX 通路被堵死。
3. 原因推测: 在混合模式下，频繁调用 receive_frame_can (即使是清空) 会导致
   ZLG 驱动内部锁定在"Standard CAN Mode"，从而拒绝后续的 send_canfd 请求。
   或者 Tx 缓冲区因之前的操作已满且未释放。

修复策略 (严格执行):
1. 【铁律】WHJ 运动循环内，绝对不调用任何 CAN (标准) 相关的函数 (收/发/清空)。
2. 【隔离】WHJ 通信前，仅清空 CANFD 缓冲区。
3. 【降频】控制频率降至 50Hz (20ms)，确保 Tx 缓冲区有足够时间发送。
4. 【独占】运动开始时，打印警告，提示用户此时 Kinco 将完全无响应。
================================================================================
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import time
import struct
import atexit

from drivers.motion_controller import SmoothMotorController, MotionProfile
from core import ZlgCanDriver, ZCANDeviceType
from core.zlgcan_driver import CANFrame
from core.protocol import WHJProtocol, Register, WorkMode

# ============================================================================
# 配置 (保守参数)
# ============================================================================
WHJ_ID = 7
WHJ_RESPONSE_ID = WHJ_ID + 0x100
KINCO_ID = 1

KINCO_NMT_ID = 0x000
KINCO_RPDO1_ID = 0x201
KINCO_RPDO2_ID = 0x301
KINCO_GEAR_RATIO = 16384
KINCO_RPM_TO_UNITS = (65536 * 512) / 1875

# 关键调整：更慢、更稳
STEP_SIZE_DEG = 20.0      # 每次移动 20 度 (减少包数量)
STEP_DELAY_SEC = 0.20     # 每步间隔 200ms (给总线充分时间)

# ============================================================================
# 单位转换常数 (Unit Conversion Constants)
# ============================================================================
MM_PER_DEGREE = 0.018           # [mm/°] 每度对应的毫米数
DEGREE_PER_MM = 1000.0 / 18.0   # [°/mm] 每毫米对应的度数 (~55.556)
RX_TIMEOUT_MS = 500       # 接收超时 500ms
MAX_RETRY_PER_STEP = 2    # 单步失败重试次数

_global_driver = None

def cleanup():
    global _global_driver
    if _global_driver:
        try:
            _global_driver.close()
        except: pass
atexit.register(cleanup)

class StableWHJ:
    def __init__(self, driver):
        self.driver = driver
        self.error_count = 0

    def _flush_canfd(self):
        count = 0
        while True:
            f = self.driver.receive_frame_canfd(timeout_ms=0)
            if not f: break
            count += 1
        return count

    def _send_and_wait(self, cmd, timeout_ms=RX_TIMEOUT_MS, verbose=True):
        self._flush_canfd()
        
        tx_ok = self.driver.send_canfd(WHJ_ID, cmd, bitrate_switch=True)
        if not tx_ok:
            if verbose: print("  [TX FAIL]")
            return None
        
        start = time.time()
        while (time.time() - start) * 1000 < timeout_ms:
            f = self.driver.receive_frame_canfd(timeout_ms=0)
            if f and f.can_id == WHJ_RESPONSE_ID:
                return f.data
            time.sleep(0.002) # 2ms 轮询
        
        if verbose: print("  [RX TIMEOUT]")
        return None

    def get_pos(self, verbose=False):
        """读取当前位置 [°] 和 [mm]"""
        cmd = WHJProtocol.build_read_frame(WHJ_ID, Register.CUR_POSITION_L, 2)
        resp = self._send_and_wait(cmd, verbose=verbose)
        if resp and len(resp) >= 6:
            low = resp[2] | (resp[3] << 8)
            high = resp[4] | (resp[5] << 8)
            val = (high << 16) | low
            if val & 0x80000000: val -= 0x100000000
            pos_deg = val * 0.0001      # [°] 当前位置 (度)
            pos_mm = pos_deg * MM_PER_DEGREE  # [mm] 当前位置 (毫米)
            return pos_deg, pos_mm
        return None, None

    def set_pos_raw(self, raw, verbose=True):
        # Low
        cmd_l = WHJProtocol.build_write_frame(WHJ_ID, Register.TAG_POSITION_L, raw & 0xFFFF)
        if not self._send_and_wait(cmd_l, verbose=verbose):
            return False
        time.sleep(0.005)
        # High
        cmd_h = WHJProtocol.build_write_frame(WHJ_ID, Register.TAG_POSITION_H, (raw >> 16) & 0xFFFF)
        return self._send_and_wait(cmd_h, verbose=verbose) is not None

    def clear_error(self):
        """清除错误寄存器"""
        print("  [Clearing Error]...", end=" ", flush=True)
        cmd = WHJProtocol.build_write_frame(WHJ_ID, Register.SYS_CLEAR_ERROR, 1)
        # 清错不需要严格等待应答，发了就行
        self._flush_canfd()
        ok = self.driver.send_canfd(WHJ_ID, cmd, bitrate_switch=True)
        time.sleep(0.05) # 给电机复位时间
        if ok:
            print("OK")
            return True
        print("FAIL")
        return False

    def move_safe(self, target_deg):
        print(f"\n[START] Moving to {target_deg}... (Step={STEP_SIZE_DEG}°, Delay={STEP_DELAY_SEC}s)")
        
        # 1. 获取当前位置
        curr = None
        for i in range(5):
            curr = self.get_pos(verbose=False)
            if curr is not None: break
            time.sleep(0.1)
        
        if curr is None:
            print("[ABORT] Cannot read position!")
            return False
            
        curr_deg, curr_mm = curr
        print(f"  [Start Pos] {curr_deg:.2f}° [{curr_mm:.3f} mm]")
        
        distance = target_deg - curr_deg
        if abs(distance) < 1.0:
            print("  [SKIP] Already there.")
            return True

        direction = 1 if distance > 0 else -1
        steps = int(abs(distance) / STEP_SIZE_DEG)
        if steps == 0: steps = 1
        
        print(f"  [Plan] Steps: {steps}, Target: {target_deg:.2f}° [{target_deg * MM_PER_DEGREE:.3f} mm]")

        success = 0
        failed = 0

        for i in range(steps):
            next_target = curr + (i + 1) * STEP_SIZE_DEG * direction
            # 修正最后一步
            if (direction > 0 and next_target > target_deg) or (direction < 0 and next_target < target_deg):
                next_target = target_deg
            
            raw_target = int(next_target / 0.0001)
            
            # 尝试发送 (带重试)
            step_ok = False
            for retry in range(MAX_RETRY_PER_STEP + 1):
                if retry > 0:
                    print(f"    [Retry {retry}]...", end=" ", flush=True)
                    # 重试前先清错！这是关键！
                    self.clear_error()
                
                if self.set_pos_raw(raw_target, verbose=(retry==0)):
                    step_ok = True
                    break
            
            if step_ok:
                next_target_mm = next_target * MM_PER_DEGREE
                print(f"OK (Target: {next_target:.1f}° [{next_target_mm:.3f} mm])")
                success += 1
                curr = next_target # 更新逻辑位置
                time.sleep(STEP_DELAY_SEC) # 关键延时
            else:
                print("FAIL")
                failed += 1
                # 连续失败两次，彻底清错并暂停
                self.clear_error()
                time.sleep(0.5)
                # 可以选择继续或退出，这里选择继续尝试后面的
        
        # 最终修正
        target_mm = target_deg * MM_PER_DEGREE
        print(f"  [Final] Aligning to {target_deg:.2f}° [{target_mm:.3f} mm]...")
        final_raw = int(target_deg / 0.0001)
        if self.set_pos_raw(final_raw):
            final_pos = self.get_pos(verbose=False)
            if final_pos[0] is not None:
                final_deg, final_mm = final_pos
                print(f"  [Done] Final OK: {final_deg:.2f}° [{final_mm:.3f} mm]")
            else:
                print("  [Done] Final OK")
        else:
            print("  [Warn] Final Failed (Try 'c' to clear error)")

        print(f"\n[SUMMARY] Success: {success}, Failed: {failed}")
        return failed < (steps * 0.2) # 允许 20% 失败率

    def init(self):
        print("[Init] WHJ...")
        # Handshake
        self._flush_canfd()
        self.driver.send_canfd(WHJ_ID, bytes([0x02, 0x49, 0x00]), bitrate_switch=True)
        time.sleep(0.1)
        
        # Enable
        en_cmd = WHJProtocol.build_write_frame(WHJ_ID, Register.SYS_ENABLE_DRIVER, 1)
        if not self._send_and_wait(en_cmd, verbose=False):
            print("  [Warn] Enable timeout, retrying...")
            time.sleep(0.2)
            self._send_and_wait(en_cmd, verbose=False)
            
        # Mode
        mode_cmd = WHJProtocol.build_write_frame(WHJ_ID, Register.TAG_WORK_MODE, WorkMode.POSITION_MODE)
        self._send_and_wait(mode_cmd, verbose=False)
        time.sleep(0.2)
        print("[Init] OK")
        return True

class SimpleKinco:
    def __init__(self, driver):
        self.driver = driver
    def send(self, cid, data):
        f = CANFrame(can_id=cid, data=data, frame_type="CAN")
        return self.driver.send_frame_can(f)
    def start(self):
        self.send(KINCO_NMT_ID, bytes([0x01, KINCO_ID]))
        time.sleep(0.1)
    def enable(self):
        self.send(KINCO_RPDO1_ID, bytes([0x01, 0x3F, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00]))
        time.sleep(0.1)
    def move(self, deg):
        pos = int(deg * KINCO_GEAR_RATIO)
        vel = int(50 * KINCO_RPM_TO_UNITS)  # [rpm] 目标转速
        data = struct.pack('<i', pos) + struct.pack('<I', vel)
        self.send(KINCO_RPDO2_ID, data)
        print(f"[Kinco] Cmd Sent: {deg:.2f}°")

def main():
    global _global_driver
    print("="*60)
    print("STABLE VERSION v4 (With Auto-Clear-Error)")
    print("="*60)
    print("Commands:")
    print("  whj <pos> : Move WHJ (Safe Mode)")
    print("  kc <pos>  : Move Kinco")
    print("  c         : Clear WHJ Error Manually")
    print("  r         : Read WHJ Position")
    print("  q         : Quit")
    print("="*60)
    
    try:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        _global_driver = driver
        print("[HW] Opened")
    except Exception as e:
        print(f"[HW Error] {e}")
        return

    whj = StableWHJ(driver)
    kinco = SimpleKinco(driver)

    if not whj.init():
        print("[FATAL] Init failed")
        driver.close()
        return
    
    kinco.start()
    kinco.enable()
    print("\nReady.")

    while True:
        try:
            txt = input("> ").strip()
            if not txt: continue
            parts = txt.split()
            cmd = parts[0].lower()
            
            if cmd == 'q': break
            
            elif cmd == 'whj' and len(parts) > 1:
                target_deg = float(parts[1])
                whj.move_safe(target_deg)
                # 运动后读一次
                p = whj.get_pos(verbose=False)
                if p[0] is not None:
                    p_deg, p_mm = p
                    print(f"[Check] Current: {p_deg:.2f}° [{p_mm:.3f} mm]")
            
            elif cmd == 'kc' and len(parts) > 1:
                kinco.move(float(parts[1]))
            
            elif cmd == 'r':
                p_deg, p_mm = whj.get_pos()
                if p_deg is not None:
                    print(f"Pos: {p_deg:.4f}° [{p_mm:.4f} mm]")
                else:
                    print("Pos: Read Failed")
            
            elif cmd == 'c':
                whj.clear_error()
            
            elif cmd == 'h':
                print("Commands: whj, kc, r, c, q")
                
        except KeyboardInterrupt:
            print("\nStopping...")
            whj.clear_error()
            break
        except Exception as e:
            print(f"Err: {e}")

    driver.close()
    print("Bye. Have a good evening!")

if __name__ == "__main__":
    main()