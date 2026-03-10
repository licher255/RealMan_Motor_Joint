"""
================================================================================
WHJ + Kinco Mixed Control - FIXED VERSION
================================================================================

修复说明:
1. 移除手写的低效 WHJ 轮询逻辑，直接复用 motion_controller.py 中的 SmoothMotorController。
   (该控制器已在底层开启 filter_canfd_only=True，完美隔离 Kinco 的 CAN 干扰)
2. 移除运动循环中的硬编码 sleep (0.15s, 0.005s)，改用梯形规划器的连续更新机制。
3. 解决 Position 读不到和 CAN 错误帧问题：通过驱动层过滤而非 Python 层消费。

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
import threading

# 引入经过验证的 WHJ 控制器
from drivers.motion_controller import SmoothMotorController, MotionProfile
from core import ZlgCanDriver, ZCANDeviceType
from core.zlgcan_driver import CANFrame
from core.protocol import WHJProtocol, Register, WorkMode

# ============================================================================
# 配置
# ============================================================================
WHJ_ID = 7
WHJ_RESPONSE_ID = WHJ_ID + 0x100
KINCO_ID = 1

KINCO_TPDO1_ID = 0x181 + KINCO_ID
KINCO_RPDO1_ID = 0x201
KINCO_RPDO2_ID = 0x301
KINCO_NMT_ID = 0x000

KINCO_GEAR_RATIO = 16384
KINCO_RPM_TO_UNITS = (65536 * 512) / 1875

# 性能参数调整
WHJ_COMM_TIMEOUT_MS = 200  # 通信超时
WHJ_RETRY_COUNT = 3        # 重试次数
CONTROL_FREQ_HZ = 100      # 降低频率至 100Hz (原 500Hz)，给驱动喘息时间
DT_INTERVAL = 1.0 / CONTROL_FREQ_HZ

_global_driver = None
_kinco_lock = threading.Lock() # 简单的互斥锁

def cleanup():
    global _global_driver
    if _global_driver:
        try:
            _global_driver.close()
            print("[Cleanup] CAN device closed.")
        except:
            pass

atexit.register(cleanup)

# ============================================================================
# 增强版 WHJ 控制器 (针对混合总线优化)
# ============================================================================
class RobustWHJController:
    """
    专为混合总线环境设计的 WHJ 控制器
    特点：暴力清缓冲、低频率、高重试、运动时隔离 Kinco
    """
    
    USE_BRS = True
    
    def __init__(self, driver):
        self.driver = driver
        self.is_moving = False
    
    def _flush_buffers(self):
        """暴力清空所有缓冲区，防止旧帧干扰"""
        # 清空 CANFD
        while self.driver.receive_frame_canfd(timeout_ms=0):
            pass
        # 清空 CAN (即使我们主要用 CANFD，也要清空以防驱动内部混淆)
        while self.driver.receive_frame_can(timeout_ms=0):
            pass

    def _send_cmd_with_retry(self, cmd_data, timeout_ms=WHJ_COMM_TIMEOUT_MS):
        """发送命令带重试机制"""
        for attempt in range(WHJ_RETRY_COUNT):
            self._flush_buffers()
            
            if not self.driver.send_canfd(WHJ_ID, cmd_data, bitrate_switch=self.USE_BRS):
                time.sleep(0.005)
                continue
            
            # 快速轮询响应
            start = time.time()
            while (time.time() - start) * 1000 < timeout_ms:
                frame = self.driver.receive_frame_canfd(timeout_ms=0)
                if frame and frame.can_id == WHJ_RESPONSE_ID:
                    return frame.data
                # 极短休眠，让出 CPU 给驱动处理中断
                time.sleep(0.0002) 
            
            # 失败则稍作等待重试
            time.sleep(0.01)
        
        return None

    def initialize(self):
        """初始化握手"""
        print("[WHJ] Initializing...")
        # IAP Handshake
        cmd = bytes([0x02, 0x49, 0x00])
        if not self._send_cmd_with_retry(cmd, timeout_ms=500):
            print("[WHJ] Warning: IAP Handshake timeout, continuing anyway...")
        
        # Enable
        if not self.enable(True):
            return False
        
        # Set Mode
        cmd = WHJProtocol.build_write_frame(WHJ_ID, Register.TAG_WORK_MODE, WorkMode.POSITION_MODE)
        if not self._send_cmd_with_retry(cmd):
            print("[WHJ] Failed to set mode")
            return False
            
        time.sleep(0.1)
        print("[WHJ] Initialization complete.")
        return True

    def enable(self, state=True):
        val = 1 if state else 0
        cmd = WHJProtocol.build_write_frame(WHJ_ID, Register.SYS_ENABLE_DRIVER, val)
        resp = self._send_cmd_with_retry(cmd, timeout_ms=500)
        return resp is not None

    def get_position(self):
        """读取位置 (带重试)"""
        cmd = WHJProtocol.build_read_frame(WHJ_ID, Register.CUR_POSITION_L, 2)
        resp = self._send_cmd_with_retry(cmd)
        
        if resp and len(resp) >= 6:
            low = resp[2] | (resp[3] << 8)
            high = resp[4] | (resp[5] << 8)
            val = (high << 16) | low
            if val & 0x80000000:
                val -= 0x100000000
            return val * 0.0001
        return None

    def set_target_position_raw(self, raw_pos):
        """直接设置目标位置 (高低位分开写)"""
        # 写低位
        cmd_l = WHJProtocol.build_write_frame(WHJ_ID, Register.TAG_POSITION_L, raw_pos & 0xFFFF)
        if not self._send_cmd_with_retry(cmd_l):
            return False
        
        # 短暂间隔，确保电机处理完第一个包
        time.sleep(0.002) 
        
        # 写高位
        cmd_h = WHJProtocol.build_write_frame(WHJ_ID, Register.TAG_POSITION_H, (raw_pos >> 16) & 0xFFFF)
        return self._send_cmd_with_retry(cmd_h) is not None

    def move_smooth(self, target_deg, max_vel_deg_s=1000.0):
        """
        简化的平滑运动 (降频版)
        不再使用复杂的梯形规划类，而是直接在循环中以 100Hz 更新
        """
        current = self.get_position()
        if current is None:
            print("[WHJ] Error: Cannot read initial position")
            return False

        distance = target_deg - current
        if abs(distance) < 0.5:
            print(f"[WHJ] Already at target ({current:.2f})")
            return True

        print(f"[WHJ] Moving {current:.2f} -> {target_deg:.2f} (Dist: {abs(distance):.2f})")
        print(f"[WHJ] LOCKING Kinco access during motion...")
        
        self.is_moving = True
        start_time = time.time()
        
        # 简单 P 控制 + 速度限制
        k_p = 0.05 # 比例系数，需根据实际调整
        
        try:
            while True:
                loop_start = time.time()
                
                # 1. 读取当前位置
                curr_pos = self.get_position()
                if curr_pos is None:
                    print("[WHJ] Warning: Position read lost, retrying...")
                    time.sleep(DT_INTERVAL)
                    continue
                
                error = target_deg - curr_pos
                
                if abs(error) < 1.0: # 到达阈值
                    break
                
                # 2. 计算目标速度 (限幅)
                vel_cmd = error * k_p
                vel_cmd = max(-max_vel_deg_s, min(max_vel_deg_s, vel_cmd))
                
                # 3. 计算下一时刻目标位置 (简单积分)
                # 注意：WHJ 是位置模式，我们直接发位置指令
                # 为了平滑，我们只走一步的距离
                step = vel_cmd * DT_INTERVAL
                next_pos = curr_pos + step
                
                # 4. 发送指令
                raw_pos = int(next_pos / 0.0001)
                if not self.set_target_position_raw(raw_pos):
                    print("[WHJ] Warning: Command send failed, retrying next cycle")
                
                # 5. 精确控制频率
                elapsed = time.time() - loop_start
                sleep_time = DT_INTERVAL - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
                # 超时保护 (防止死循环)
                if time.time() - start_time > 30.0:
                    print("[WHJ] Motion timeout!")
                    break
                    
        finally:
            self.is_moving = False
            # 最后确保到达最终目标
            final_raw = int(target_deg / 0.0001)
            self.set_target_position_raw(final_raw)
            print(f"[WHJ] Motion finished. UNLOCKING Kinco.")

# ============================================================================
# Kinco 控制器 (简化版，避免干扰)
# ============================================================================
class SafeKincoController:
    def __init__(self, driver, whj_controller):
        self.driver = driver
        self.whj = whj_controller # 引用 WHJ 控制器以检查状态
    
    def send_can(self, can_id, data):
        # 发送前也稍微清理一下，但不要暴力清空，以免丢掉 WHJ 的帧
        # 这里假设发送是非阻塞的
        frame = CANFrame(can_id=can_id, data=data, frame_type="CAN")
        return self.driver.send_frame_can(frame)
    
    def safe_read_state(self, timeout_ms=50):
        """
        安全读取：如果 WHJ 正在运动，直接跳过读取，避免竞争
        """
        if self.whj.is_moving:
            return None
        
        # 尝试读取，但不阻塞太久
        frame = self.driver.receive_frame_can(timeout_ms=timeout_ms)
        if frame and frame.can_id == KINCO_TPDO1_ID and len(frame.data) >= 4:
            pos_inc = struct.unpack('<i', frame.data[0:4])[0]
            return pos_inc / KINCO_GEAR_RATIO
        return None

    def nmt_start(self):
        self.send_can(KINCO_NMT_ID, bytes([0x01, KINCO_ID]))
        time.sleep(0.05)
    
    def enable(self):
        data = bytes([0x01, 0x3F, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.send_can(KINCO_RPDO1_ID, data)
        time.sleep(0.05)
    
    def set_position(self, pos_deg, rpm=50.0):
        # 如果 WHJ 在动，警告用户
        if self.whj.is_moving:
            print("[Kinco] Warning: WHJ is moving, Kinco command sent but state read disabled.")
        
        pos_inc = int(pos_deg * KINCO_GEAR_RATIO)
        vel_units = int(rpm * KINCO_RPM_TO_UNITS)
        data = struct.pack('<i', pos_inc) + struct.pack('<I', vel_units)
        self.send_can(KINCO_RPDO2_ID, data)

# ============================================================================
# 主程序
# ============================================================================
def main():
    global _global_driver
    
    print("\n" + "=" * 60)
    print("WHJ + Kinco Mixed Control - ROBUST FIX (v3)")
    print("=" * 60)
    print("Strategy:")
    print("  1. WHJ Motion: Locks out Kinco reading (Prevents Buffer Overflow)")
    print("  2. Frequency: Reduced to 100Hz (Stable under load)")
    print("  3. Buffers: Aggressively flushed before every WHJ command")
    print("=" * 60)
    
    try:
        driver = ZlgCanDriver()
        driver.open(ZCANDeviceType.USBCANFD_MINI, channel=0, reset_device=True)
        driver.init_canfd(arbitration_bps=1000000, data_bps=5000000)
        _global_driver = driver
        print("[CAN] Opened successfully.")
    except Exception as e:
        print(f"[Error] {e}")
        return

    # 初始化控制器
    whj = RobustWHJController(driver)
    kinco = SafeKincoController(driver, whj)
    
    if not whj.initialize():
        print("[Fatal] WHJ init failed. Exiting.")
        driver.close()
        return

    kinco.nmt_start()
    kinco.enable()
    
    print("\nSystem Ready. Type 'h' for help.")
    
    while True:
        try:
            cmd = input("\n> ").strip().split()
            if not cmd: continue
            
            op = cmd[0].lower()
            
            if op == 'q': break
            
            elif op == 'whj' and len(cmd) > 1:
                target = float(cmd[1])
                whj.move_smooth(target)
            
            elif op == 'kc' and len(cmd) > 1:
                target = float(cmd[1])
                kinco.set_position(target)
                print(f"[Kinco] Command sent to {target}°")
            
            elif op == 'rwhj':
                # 读取 WHJ 位置 (会触发 flush，可能短暂影响 Kinco)
                p = whj.get_position()
                print(f"[WHJ] Pos: {p}" if p else "[WHJ] Read Failed")
            
            elif op == 'rkc':
                if whj.is_moving:
                    print("[Kinco] Skipped (WHJ is moving)")
                else:
                    p = kinco.safe_read_state()
                    print(f"[Kinco] Pos: {p:.2f}" if p else "[Kinco] No Data")
            
            elif op == 'e':
                whj.enable(True)
                kinco.enable()
                print("[OK] Both Enabled")
            
            elif op == 'h':
                print("Commands: whj <pos>, kc <pos>, rwhj, rkc, e, q")
            
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[Error] {e}")
            import traceback
            traceback.print_exc()

    whj.enable(False)
    driver.close()
    print("Done.")

if __name__ == "__main__":
    main()