"""
================================================================================
WHJ + Kinco - V5: Non-blocking Motion + Disable Support
================================================================================

功能:
1. WHJ 通信：仅接收 CANFD 帧，彻底屏蔽 Kinco 标准帧干扰 (软件隔离)。
2. Kinco 控制：支持发送标准 CAN 指令给 Kinco (开环控制)。
3. 平滑轨迹：WHJ 运动采用梯形规划，避免冲击。
4. 【NEW】WHJ 运动非阻塞 - 运动过程中可以继续输入 Kinco 指令
5. 【NEW】支持 disable 关闭 WHJ 使能
6. 【NEW】退出时自动关闭 WHJ 使能

操作:
  m <pos> : WHJ 平滑移动到目标位置 (非阻塞)
  k <pos> : Kinco 移动到目标位置 (开环，无位置反馈)
  r       : 读取 WHJ 当前位置
  c       : 清除 WHJ 错误
  d       : 关闭 WHJ 使能 (disable)
  e       : 使能 WHJ (enable)
  s       : 停止 WHJ 运动 (紧急)
  q       : 退出 (自动关闭 WHJ 使能)
================================================================================
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import time
import math
import struct
import atexit
import threading
import queue
import sys

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
MAX_VEL = 1000.0       # [deg/s] 最大速度
MAX_ACC = 2000.0       # [deg/s^2] 最大加速度
MOTION_UPDATE_RATE = 100  # Hz - 运动更新频率

# 单位转换常量 (mm <-> degree)
MM_PER_DEGREE = 0.018       # [mm/°] 每度对应的毫米数
DEGREE_PER_MM = 1000.0 / 18.0  # [°/mm] 每毫米对应的度数 (~55.556 °/mm)

_global_driver = None
_global_ctrl = None

def cleanup():
    global _global_driver, _global_ctrl
    print("\n[Cleanup] 正在清理...")
    # 先停止 WHJ 使能
    if _global_ctrl:
        try:
            _global_ctrl.whj_disable()
            print("[Cleanup] WHJ 已关闭使能")
        except Exception as e:
            print(f"[Cleanup] 关闭 WHJ 使能失败: {e}")
    # 关闭设备
    if _global_driver:
        try:
            _global_driver.close()
            print("[Cleanup] 设备已关闭")
        except: pass
atexit.register(cleanup)

class HybridController:
    def __init__(self, driver):
        self.driver = driver
        self.whj_id = WHJ_ID
        self.whj_resp_id = WHJ_RESPONSE_ID
        
        # 运动控制线程相关
        self._motion_thread = None
        self._motion_stop_event = threading.Event()
        self._motion_queue = queue.Queue()
        self._is_moving = False
        
        # 输出锁，防止多线程输出混乱
        self._print_lock = threading.Lock()
        self._last_status_len = 0
        
    # ------------------------------------------------------------------------
    # WHJ 部分 (严格过滤模式)
    # ------------------------------------------------------------------------
    def _recv_whj_only(self, timeout_ms=500):
        """
        【核心魔法】只接收 CANFD 帧 (WHJ)，自动丢弃/忽略标准 CAN 帧 (Kinco)。
        """
        start = time.time()
        while (time.time() - start) * 1000 < timeout_ms:
            frame = self.driver.receive_frame_canfd(timeout_ms=0)
            if frame:
                if frame.can_id == self.whj_resp_id:
                    return frame.data
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
        """获取 WHJ 当前位置 [°]"""
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

    def whj_get_pos_mm(self):
        """获取 WHJ 当前位置 [mm]"""
        pos_deg = self.whj_get_pos()
        return pos_deg * MM_PER_DEGREE if pos_deg is not None else None

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

    def whj_get_error(self):
        """读取 WHJ 错误状态"""
        cmd = WHJProtocol.build_read_frame(self.whj_id, Register.SYS_ERROR, 1)
        for _ in range(3):
            resp = self._send_whj_cmd(cmd, timeout_ms=200)
            if resp and len(resp) >= 4:
                error_code = resp[2] | (resp[3] << 8)
                return error_code
            time.sleep(0.05)
        return None

    def whj_iap_handshake(self):
        """IAP 握手 - 启动 WHJ 通信 (必须先完成)"""
        iap_cmd = bytes([0x02, 0x49, 0x00])
        print("  [IAP] 发送 IAP 握手指令...")
        for i in range(3):
            while self.driver.receive_frame_canfd(timeout_ms=0): pass
            self.driver.send_canfd(self.whj_id, iap_cmd, bitrate_switch=True)
            print(f"    IAP 指令 {i+1}/3 已发送")
            time.sleep(0.05)

    def whj_enable(self):
        """使能 WHJ 驱动"""
        self._safe_print("  [WHJ] 使能驱动...")
        en = WHJProtocol.build_write_frame(self.whj_id, Register.SYS_ENABLE_DRIVER, 1)
        if not self._send_whj_cmd(en, timeout_ms=500):
            self._safe_print("  [警告] 使能驱动指令发送失败或无响应")
            return False
        time.sleep(0.1)
        self._safe_print("  [WHJ] 驱动已使能")
        return True

    def _safe_print(self, msg, end='\n', flush=False):
        """线程安全的打印"""
        with self._print_lock:
            # 如果之前有状态行，先清除
            if self._last_status_len > 0 and end == '\n':
                sys.stdout.write('\r' + ' ' * self._last_status_len + '\r')
                self._last_status_len = 0
            print(msg, end=end, flush=flush)

    def _print_status(self, msg):
        """打印单行状态（会被后续输出覆盖）"""
        with self._print_lock:
            # 清除之前的行
            if self._last_status_len > 0:
                sys.stdout.write('\r' + ' ' * self._last_status_len + '\r')
            sys.stdout.write(msg)
            self._last_status_len = len(msg)
            sys.stdout.flush()

    def whj_disable(self):
        """关闭 WHJ 驱动使能"""
        self._safe_print("  [WHJ] 关闭驱动使能...")
        # 先停止任何正在进行的运动
        self.whj_stop_motion()
        dis = WHJProtocol.build_write_frame(self.whj_id, Register.SYS_ENABLE_DRIVER, 0)
        for i in range(3):  # 发送3次确保可靠
            self.driver.send_canfd(self.whj_id, dis, bitrate_switch=True)
            time.sleep(0.02)
        self._safe_print("  [WHJ] 驱动已关闭")

    def whj_init(self):
        """WHJ 初始化 - 带自检流程"""
        print("[Init] WHJ 启动自检...")
        
        # Step 0: IAP 握手
        self.whj_iap_handshake()
        time.sleep(0.1)
        
        # Step 1: 发送 3 次查询位置指令来刷掉缓冲区的错误数据
        print("  [自检1/3] 刷新位置查询...")
        for i in range(3):
            pos = self.whj_get_pos()
            if pos is not None:
                pos_mm = pos * MM_PER_DEGREE
                print(f"    尝试 {i+1}/3: 位置={pos:.2f}[°] ({pos_mm:.3f}[mm])")
            else:
                print(f"    尝试 {i+1}/3: 无响应")
            time.sleep(0.05)
        
        # Step 2: 发送 3 次错误清除指令
        print("  [自检2/3] 清除错误...")
        for i in range(3):
            self.whj_clear_error()
            print(f"    清除指令 {i+1}/3 已发送")
            time.sleep(0.05)
        
        # Step 3: 查询 WHJ 状态，检查错误码
        print("  [自检3/3] 检查错误状态...")
        error_code = self.whj_get_error()
        if error_code is None:
            print("  [警告] 无法读取错误状态，继续初始化...")
        elif error_code != 0:
            from core.protocol import ErrorCode
            errors = ErrorCode.parse(error_code)
            print(f"  [警告] WHJ 仍有错误: 0x{error_code:04X} - {', '.join(errors)}")
            print("  [警告] 尝试再次清除错误...")
            for i in range(3):
                self.whj_clear_error()
                time.sleep(0.05)
            error_code = self.whj_get_error()
            if error_code is not None and error_code != 0:
                print(f"  [错误] 清除失败，错误依然存在: 0x{error_code:04X}")
            else:
                print("  [OK] 错误已清除")
        else:
            print("  [OK] WHJ 状态正常，无错误")
        
        # 使能驱动
        self.whj_enable()
        
        # 设置工作模式为位置模式
        print("  [Init] 设置位置模式...")
        mode = WHJProtocol.build_write_frame(self.whj_id, Register.TAG_WORK_MODE, WorkMode.POSITION_MODE)
        if not self._send_whj_cmd(mode, timeout_ms=500):
            print("  [警告] 设置模式指令发送失败或无响应")
        time.sleep(0.2)
        
        # 最终状态检查
        final_pos = self.whj_get_pos()
        final_error = self.whj_get_error()
        if final_pos is not None:
            final_pos_mm = final_pos * MM_PER_DEGREE
            print(f"[Init] WHJ 初始化完成 | 位置: {final_pos:.2f}[°] ({final_pos_mm:.3f}[mm])", end="")
            if final_error is not None:
                print(f" | 错误码: 0x{final_error:04X}")
            else:
                print("")
        else:
            print("[Init] WHJ 初始化完成 (位置读取失败)")

    # ------------------------------------------------------------------------
    # WHJ 非阻塞运动控制
    # ------------------------------------------------------------------------
    def whj_is_moving(self):
        """检查 WHJ 是否正在运动"""
        return self._is_moving

    def whj_stop_motion(self):
        """停止 WHJ 运动"""
        if self._motion_thread and self._motion_thread.is_alive():
            print("\n  [WHJ] 正在停止运动...")
            self._motion_stop_event.set()
            self._motion_thread.join(timeout=1.0)
            self._is_moving = False
            print("  [WHJ] 运动已停止")

    def _motion_worker(self, target_deg):
        """运动控制工作线程"""
        self._is_moving = True
        self._motion_stop_event.clear()
        
        curr = self.whj_get_pos()
        if curr is None:
            self._safe_print("[Error] Cannot read WHJ pos")
            self._is_moving = False
            return
        
        dist = target_deg - curr
        dist_mm = dist * MM_PER_DEGREE
        if abs(dist) < 0.5:
            self._safe_print("  Already there.")
            self._is_moving = False
            return

        # 梯形规划计算
        t_acc = MAX_VEL / MAX_ACC
        d_acc = 0.5 * MAX_ACC * t_acc * t_acc
        abs_dist = abs(dist)
        abs_dist_mm = abs_dist * MM_PER_DEGREE
        
        if 2 * d_acc >= abs_dist:
            t_total = 2 * math.sqrt(abs_dist / MAX_ACC)
        else:
            t_const = (abs_dist - 2 * d_acc) / MAX_VEL
            t_total = 2 * t_acc + t_const

        self._safe_print(f"  Dist: {abs_dist:.1f}[°] ({abs_dist_mm:.3f}[mm]), Time: {t_total:.2f}[s]")
        
        start_time = time.time()
        direction = 1 if dist > 0 else -1
        
        try:
            while not self._motion_stop_event.is_set():
                t = time.time() - start_time
                if t >= t_total:
                    # 到达目标
                    self.driver.send_canfd(self.whj_id, 
                        WHJProtocol.build_write_frame(self.whj_id, Register.TAG_POSITION_L, 
                            int(target_deg / 0.0001) & 0xFFFF), bitrate_switch=True)
                    self.driver.send_canfd(self.whj_id, 
                        WHJProtocol.build_write_frame(self.whj_id, Register.TAG_POSITION_H, 
                            (int(target_deg / 0.0001) >> 16) & 0xFFFF), bitrate_switch=True)
                    target_mm = target_deg * MM_PER_DEGREE
                    self._safe_print(f"\n[WHJ] Done. Final: {target_deg:.2f}[°] ({target_mm:.3f}[mm])")
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
                
                # 高速发送 (不等待应答)
                self.driver.send_canfd(self.whj_id, 
                    WHJProtocol.build_write_frame(self.whj_id, Register.TAG_POSITION_L, raw & 0xFFFF), 
                    bitrate_switch=True)
                self.driver.send_canfd(self.whj_id, 
                    WHJProtocol.build_write_frame(self.whj_id, Register.TAG_POSITION_H, (raw >> 16) & 0xFFFF), 
                    bitrate_switch=True)
                
                # 每 0.2 秒更新一次状态行（使用 \r 不换行）
                if int(t * 5) % 5 == 0:  # 每秒更新几次
                    current_target_mm = current_target * MM_PER_DEGREE
                    target_mm = target_deg * MM_PER_DEGREE
                    self._print_status(f"  [WHJ Moving] {current_target:.1f}[°] ({current_target_mm:.3f}[mm]) / {target_deg:.1f}[°] ({target_mm:.3f}[mm]) ({100*t/t_total:.0f}%)")
                
                time.sleep(1.0 / MOTION_UPDATE_RATE)
            
            # 最终校准
            time.sleep(0.1)
            final = self.whj_get_pos()
            if final is not None:
                final_mm = final * MM_PER_DEGREE
                err_deg = abs(final - target_deg)
                err_mm = err_deg * MM_PER_DEGREE
                self._safe_print(f"[Check] WHJ Pos: {final:.2f}[°] ({final_mm:.3f}[mm]) (Err: {err_deg:.2f}[°] / {err_mm:.3f}[mm])")
            else:
                self._safe_print("[Check] WHJ Pos: Read Failed")
            
        except Exception as e:
            self._safe_print(f"\n[WHJ Motion Error] {e}")
        finally:
            self._is_moving = False

    def whj_move_smooth_async_deg(self, target_deg):
        """
        启动 WHJ 平滑移动 (非阻塞), 目标位置单位: [°]
        返回 True 表示成功启动，False 表示已有运动在进行中
        """
        if self._is_moving:
            self._safe_print("[Error] WHJ 正在运动中，请先等待或发送 's' 停止")
            return False
        
        target_mm = target_deg * MM_PER_DEGREE
        self._safe_print(f"\n[WHJ] 启动平滑移动至 {target_deg:.2f}[°] ({target_mm:.3f}[mm])")
        self._motion_thread = threading.Thread(target=self._motion_worker, args=(target_deg,))
        self._motion_thread.daemon = True
        self._motion_thread.start()
        return True

    def whj_move_smooth_async_mm(self, target_mm):
        """
        启动 WHJ 平滑移动 (非阻塞), 目标位置单位: [mm]
        返回 True 表示成功启动，False 表示已有运动在进行中
        """
        target_deg = target_mm * DEGREE_PER_MM
        return self.whj_move_smooth_async_deg(target_deg)

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
        data = bytes([0x01, 0x3F, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00])
        self.kinco_send(KINCO_RPDO1_ID, data)
        time.sleep(0.1)
        print("[Init] Kinco OK (Open Loop)")

    def kinco_move(self, deg):
        self._safe_print(f"\n[Kinco] Moving to {deg}[°] (Open Loop)")
        pos_raw = int(deg * KINCO_GEAR_RATIO)
        vel_raw = int(50 * KINCO_RPM_TO_UNITS)
        
        data = struct.pack('<i', pos_raw) + struct.pack('<I', vel_raw)
        
        for i in range(3):
            self.kinco_send(KINCO_RPDO2_ID, data)
            time.sleep(0.05)
        
        self._safe_print(f"[Kinco] Command dispatched. (No feedback check)")


def main():
    global _global_driver, _global_ctrl
    print("="*60)
    print("HYBRID CONTROL V5: Non-blocking WHJ + Kinco")
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
    _global_ctrl = ctrl
    ctrl.whj_init()
    ctrl.kinco_init()
    
    print("\nReady!")
    print("Commands:")
    print("  m <pos> : Move WHJ (Smooth, Non-blocking), unit: [°]")
    print("  mm <pos>: Move WHJ (Smooth, Non-blocking), unit: [mm]")
    print("  k <pos> : Move Kinco (Open Loop), unit: [°]")
    print("  r       : Read WHJ Pos [°] & [mm]")
    print("  c       : Clear WHJ Error")
    print("  d       : Disable WHJ (关闭使能)")
    print("  e       : Enable WHJ (使能)")
    print("  s       : Stop WHJ Motion (停止运动)")
    print("  q       : Quit (自动关闭 WHJ 使能)")

    while True:
        try:
            # 显示提示符前清除状态行
            with ctrl._print_lock:
                if ctrl._last_status_len > 0:
                    sys.stdout.write('\r' + ' ' * ctrl._last_status_len + '\r')
                    ctrl._last_status_len = 0
                prompt = "[MOVING] > " if ctrl.whj_is_moving() else "> "
                sys.stdout.write(prompt)
                sys.stdout.flush()
            
            txt = input().strip()
            if not txt: 
                continue
            
            parts = txt.split()
            cmd = parts[0].lower()
            
            if cmd == 'q': 
                break
            
            elif cmd == 'm' and len(parts) > 1:
                ctrl.whj_move_smooth_async_deg(float(parts[1]))
            
            elif cmd == 'mm' and len(parts) > 1:
                ctrl.whj_move_smooth_async_mm(float(parts[1]))
            
            elif cmd == 'k' and len(parts) > 1:
                ctrl.kinco_move(float(parts[1]))
            
            elif cmd == 'r':
                p = ctrl.whj_get_pos()
                if p is not None:
                    p_mm = p * MM_PER_DEGREE
                    ctrl._safe_print(f"WHJ Pos: {p:.2f}[°] ({p_mm:.3f}[mm])")
                else:
                    ctrl._safe_print("WHJ Pos: Read Failed")
            
            elif cmd == 'c':
                ctrl.whj_clear_error()
            
            elif cmd == 'd':
                ctrl.whj_disable()
            
            elif cmd == 'e':
                ctrl.whj_enable()
            
            elif cmd == 's':
                ctrl.whj_stop_motion()
                
        except KeyboardInterrupt:
            print("\nStopping...")
            ctrl.whj_stop_motion()
            break
        except Exception as e:
            print(f"Err: {e}")
            import traceback
            traceback.print_exc()

    print("Bye! Have a great evening!")

if __name__ == "__main__":
    main()
