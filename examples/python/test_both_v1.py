"""
================================================================================
WHJ + Kinco Mixed Control - Based on whj_motion_controller.py
================================================================================

基于 whj_motion_controller.py 的可靠实现，添加 Kinco 支持。
关键：WHJ 只操作 CANFD 缓冲区，Kinco 只操作 CAN 缓冲区

硬件:
    - WHJ Motor: ID = 7 (CAN FD)
    - Kinco Motor: ID = 1 (Standard CAN)
================================================================================
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import time
import struct
import atexit
import math

from core import ZlgCanDriver, ZCANDeviceType
from core.zlgcan_driver import CANFrame
from core.protocol import WHJProtocol, Register, WorkMode


# ============================================================================
# 配置
# ============================================================================
WHJ_ID = 7
KINCO_ID = 1
WHJ_RESPONSE_ID = WHJ_ID + 0x100

KINCO_TPDO1_ID = 0x181 + KINCO_ID
KINCO_RPDO1_ID = 0x201
KINCO_RPDO2_ID = 0x301
KINCO_NMT_ID = 0x000

KINCO_GEAR_RATIO = 16384
KINCO_RPM_TO_UNITS = (65536 * 512) / 1875

_global_driver = None


def cleanup():
    global _global_driver
    if _global_driver:
        try:
            _global_driver.close()
        except:
            pass


atexit.register(cleanup)


# ============================================================================
# WHJ 控制器（基于 whj_motion_controller.py）
# ============================================================================
class WHJMotionController:
    """WHJ 运动控制器 - 带梯形轨迹规划"""
    
    USE_BRS = True
    
    def __init__(self, driver):
        self.driver = driver
    
    def send_command(self, data, timeout_ms=100, clear_buffer=True, filter_can=False):
        """
        发送命令并等待响应
        
        Args:
            filter_can: 如果 True，清空 CAN 缓冲区（运动时避免 Kinco 干扰）
        """
        # 清空 CANFD 缓冲区
        if clear_buffer:
            while self.driver.receive_frame_canfd(timeout_ms=0):
                pass
            # 运动时清空 CAN 缓冲区
            if filter_can:
                while self.driver.receive_frame_can(timeout_ms=0):
                    pass
        
        # 发送 CAN FD 帧
        if not self.driver.send_canfd(WHJ_ID, data, bitrate_switch=self.USE_BRS):
            return None
        
        # 立即开始快速轮询（0.5ms 间隔）
        start = time.time()
        while (time.time() - start) * 1000 < timeout_ms:
            frame = self.driver.receive_frame_canfd(timeout_ms=0)
            if frame and frame.can_id == WHJ_RESPONSE_ID:
                return frame.data
            time.sleep(0.0005)  # 0.5ms 轮询间隔
        
        return None
    
    def get_position(self, filter_can=False):
        """读取位置 - filter_can: 运动时过滤 CAN 帧"""
        cmd = WHJProtocol.build_read_frame(WHJ_ID, Register.CUR_POSITION_L, 2)
        resp = self.send_command(cmd, filter_can=filter_can)
        if resp and len(resp) >= 6:
            low = resp[2] | (resp[3] << 8)
            high = resp[4] | (resp[5] << 8)
            val = (high << 16) | low
            if val & 0x80000000:
                val -= 0x100000000
            return val * 0.0001
        return None
    
    def is_enabled(self, filter_can=False):
        """查询使能状态 - filter_can: 运动时过滤 CAN 帧"""
        cmd = WHJProtocol.build_read_frame(WHJ_ID, Register.SYS_ENABLE_DRIVER, 1)
        resp = self.send_command(cmd, filter_can=filter_can)
        if resp and len(resp) >= 4:
            return (resp[2] | (resp[3] << 8)) == 1
        return None
    
    def iap_handshake(self):
        """IAP 握手 - 必须先完成"""
        print("[WHJ] IAP handshake...")
        cmd = bytes([0x02, 0x49, 0x00])
        resp = self.send_command(cmd, timeout_ms=1000)
        if resp and len(resp) >= 1 and resp[0] == 0x02:
            print("[WHJ] IAP OK")
            return True
        print("[WHJ] IAP timeout (may still work)")
        return False
    
    def enable(self, enable=True):
        value = 1 if enable else 0
        cmd = WHJProtocol.build_write_frame(WHJ_ID, Register.SYS_ENABLE_DRIVER, value)
        resp = self.send_command(cmd, timeout_ms=1000)
        success = resp is not None and len(resp) >= 3
        if success:
            print(f"[WHJ] {'Enabled' if enable else 'Disabled'}")
        return success
    
    def set_mode(self, mode):
        cmd = WHJProtocol.build_write_frame(WHJ_ID, Register.TAG_WORK_MODE, mode)
        return self.send_command(cmd) is not None
    
    def set_position(self, pos_deg, clear_buffer=False, filter_can=True):
        """设置目标位置（直接）"""
        raw = int(pos_deg / 0.0001)
        
        # 低 16 位
        cmd_l = WHJProtocol.build_write_frame(WHJ_ID, Register.TAG_POSITION_L, raw & 0xFFFF)
        resp_l = self.send_command(cmd_l, timeout_ms=300, clear_buffer=clear_buffer, filter_can=filter_can)
        if resp_l is None:
            return False
        
        time.sleep(0.005)  # 5ms 间隔
        
        # 高 16 位
        cmd_h = WHJProtocol.build_write_frame(WHJ_ID, Register.TAG_POSITION_H, (raw >> 16) & 0xFFFF)
        resp_h = self.send_command(cmd_h, timeout_ms=300, clear_buffer=False, filter_can=filter_can)
        return resp_h is not None
    
    def clear_error(self):
        cmd = WHJProtocol.build_write_frame(WHJ_ID, Register.SYS_CLEAR_ERROR, 1)
        return self.send_command(cmd) is not None
    
    def move_with_trajectory(self, target_pos, max_step=50.0):
        """
        带小步进的移动（避免 >10° 错误）
        
        策略：运动时 filter_can=True 过滤 CAN 帧，查询时 filter_can=False
        """
        # 获取当前位置（查询时不过滤）
        current = self.get_position(filter_can=False)
        if current is None:
            print("[WHJ] Cannot read position")
            return False
        
        distance = target_pos - current
        if abs(distance) < 0.5:
            print(f"[WHJ] Already at {target_pos:.1f} deg")
            return True
        
        # 确保使能（查询时不过滤）
        if not self.is_enabled(filter_can=False):
            print("[WHJ] Enabling...")
            if not self.enable(True):
                return False
            time.sleep(0.2)
        
        print(f"[WHJ] Moving {current:.1f} -> {target_pos:.1f} deg...")
        print("[WHJ] Filter CAN: ON during motion")
        
        # 步进参数
        step = max_step if distance > 0 else -max_step
        steps = int(abs(distance) / max_step)
        remainder = abs(distance) - steps * max_step
        
        # 运动时：filter_can=True，过滤 CAN 帧避免干扰
        success_count = 0
        for i in range(steps):
            next_pos = current + step * (i + 1)
            
            # 运动时过滤 CAN
            if self.set_position(next_pos, clear_buffer=True, filter_can=True):
                success_count += 1
            
            time.sleep(0.15)  # 150ms 间隔
        
        # 最终位置
        if remainder > 0.1:
            final_pos = current + distance
            self.set_position(final_pos, clear_buffer=True, filter_can=True)
        
        time.sleep(0.2)
        # 读取最终位置（查询时不过滤）
        final = self.get_position(filter_can=False)
        if final:
            error = abs(final - target_pos)
            print(f"[WHJ] Done: {final:.1f} deg (err: {error:.1f})")
        return success_count > 0


# ============================================================================
# Kinco 控制器
# ============================================================================
class KincoController:
    """Kinco 控制器 - 只操作 CAN 缓冲区"""
    
    def __init__(self, driver):
        self.driver = driver
    
    def send_can(self, can_id, data):
        """发送标准 CAN 帧"""
        frame = CANFrame(can_id=can_id, data=data, frame_type="CAN")
        return self.driver.send_frame_can(frame)
    
    def read_can(self, timeout_ms=100):
        """从 CAN 缓冲区读取"""
        return self.driver.receive_frame_can(timeout_ms=timeout_ms)
    
    def nmt_start(self):
        print("[Kinco] NMT start...")
        success = self.send_can(KINCO_NMT_ID, bytes([0x01, KINCO_ID]))
        if success:
            print("[Kinco] NMT OK")
            time.sleep(0.1)
        return success
    
    def nmt_stop(self):
        success = self.send_can(KINCO_NMT_ID, bytes([0x02, KINCO_ID]))
        if success:
            print("[Kinco] NMT stop")
        return success
    
    def enable(self):
        print("[Kinco] Enabling...")
        data = bytes([0x01, 0x3F, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00])
        success = self.send_can(KINCO_RPDO1_ID, data)
        if success:
            print("[Kinco] Enabled")
            time.sleep(0.1)
        return success
    
    def disable(self):
        data = bytes([0x01, 0x06, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00])
        success = self.send_can(KINCO_RPDO1_ID, data)
        if success:
            print("[Kinco] Disabled")
        return success
    
    def set_position(self, pos_deg, rpm=50.0):
        pos_inc = int(pos_deg * KINCO_GEAR_RATIO)
        vel_units = int(rpm * KINCO_RPM_TO_UNITS)
        data = struct.pack('<i', pos_inc) + struct.pack('<I', vel_units)
        success = self.send_can(KINCO_RPDO2_ID, data)
        if success:
            print(f"[Kinco] Move to {pos_deg:.1f} deg")
        return success
    
    def read_state(self, timeout_ms=200):
        """读取 TPDO1 - 只从 CAN 缓冲区"""
        start = time.time()
        while (time.time() - start) * 1000 < timeout_ms:
            frame = self.read_can(timeout_ms=10)
            if frame and frame.can_id == KINCO_TPDO1_ID:
                data = frame.data
                if len(data) >= 8:
                    pos_inc = struct.unpack('<i', data[0:4])[0]
                    return pos_inc / KINCO_GEAR_RATIO
            time.sleep(0.001)
        return None


# ============================================================================
# 主程序
# ============================================================================
def print_help():
    print("\n" + "=" * 60)
    print("Commands:")
    print("=" * 60)
    print("  whj <pos>   - Move WHJ")
    print("  kc <pos>    - Move Kinco")
    print("  rwhj        - Read WHJ position")
    print("  rkc         - Read Kinco position")
    print("  e           - Enable both")
    print("  dwhj        - Disable WHJ")
    print("  dkc         - Disable Kinco")
    print("  cwhj        - Clear WHJ errors")
    print("  q           - Quit")
    print("=" * 60)


def main():
    global _global_driver
    
    print("\n" + "=" * 60)
    print("WHJ + Kinco Mixed Control (Final)")
    print("=" * 60)
    print("WHJ: CANFD buffer only")
    print("Kinco: CAN buffer only")
    print("=" * 60)
    
    # 打开 CAN
    try:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        _global_driver = driver
        print("\n[CAN] Opened (CANFD Mixed Mode)")
    except Exception as e:
        print(f"[Error] {e}")
        return
    
    # 创建控制器
    whj = WHJMotionController(driver)
    kinco = KincoController(driver)
    
    # ========================================================================
    # 初始化 WHJ
    # ========================================================================
    print("\n" + "-" * 60)
    print("[Init] WHJ...")
    print("-" * 60)
    
    # IAP 握手（必须先完成）
    whj.iap_handshake()
    time.sleep(0.2)
    
    # 使能
    if not whj.enable(True):
        print("[WHJ] Enable retry...")
        time.sleep(0.3)
        whj.enable(True)
    
    time.sleep(0.1)
    whj.set_mode(WorkMode.POSITION_MODE)
    time.sleep(0.1)
    
    pos = whj.get_position(filter_can=False)
    if pos:
        print(f"[WHJ] Position: {pos:.2f} deg")
    else:
        print("[WHJ] Position read failed")
    
    # ========================================================================
    # 初始化 Kinco
    # ========================================================================
    print("\n" + "-" * 60)
    print("[Init] Kinco...")
    print("-" * 60)
    
    kinco.nmt_start()
    kinco.enable()
    
    # ========================================================================
    # 交互循环
    # ========================================================================
    print("\n" + "=" * 60)
    print("Ready!")
    print("=" * 60)
    print_help()
    
    while True:
        try:
            cmd_input = input("\n> ").strip()
            if not cmd_input:
                continue
            
            parts = cmd_input.split()
            cmd = parts[0].lower()
            
            if cmd == 'q':
                break
            
            elif cmd == 'h':
                print_help()
            
            elif cmd == 'whj' and len(parts) >= 2:
                try:
                    pos = float(parts[1])
                    whj.move_with_trajectory(pos)
                except ValueError:
                    print("Usage: whj <position>")
            
            elif cmd == 'kc' and len(parts) >= 2:
                try:
                    pos = float(parts[1])
                    kinco.set_position(pos)
                except ValueError:
                    print("Usage: kc <position>")
            
            elif cmd == 'rwhj':
                pos = whj.get_position(filter_can=False)
                en = whj.is_enabled(filter_can=False)
                print(f"[WHJ] Pos: {pos:.2f} deg" if pos else "[WHJ] Read failed")
                print(f"[WHJ] Enabled: {en}" if en is not None else "[WHJ] Enable check failed")
            
            elif cmd == 'rkc':
                pos = kinco.read_state(timeout_ms=300)
                print(f"[Kinco] Pos: {pos:.2f} deg" if pos else "[Kinco] No response")
            
            elif cmd == 'e':
                print("[Enable]...")
                whj.enable(True)
                kinco.enable()
            
            elif cmd == 'dwhj':
                whj.enable(False)
            
            elif cmd == 'dkc':
                kinco.disable()
            
            elif cmd == 'cwhj':
                if whj.clear_error():
                    print("[WHJ] Errors cleared")
                else:
                    print("[WHJ] Clear failed")
            
            else:
                print("Unknown command. Type 'h' for help.")
        
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[Error] {e}")
    
    # 退出
    print("\n[Exit]...")
    whj.enable(False)
    kinco.disable()
    kinco.nmt_stop()
    driver.close()
    print("[Done]")


if __name__ == "__main__":
    main()
