#!/usr/bin/env python3
# 哨兵 DR16 串口遥控节点（输入源之一）。
# DR16 接收机 DBUS（裸 18 字节帧，~100Hz）串口直连上位机，本节点解码并统一发布
# cmd_vel 与 gimbal_cmd（经 RemoteCore，对外接口与键盘源完全一致）。
#
# 关键设计:
#   - DBUS 帧无帧头/校验 → 用通道/拨杆合法性判帧 + 滑动重同步
#   - 失联保护：超过 lost_timeout 无有效帧 → 底盘停车 + 告警
#   - 云台遥控(mode 3)：右杆竖直 → pitch 绝对角增量积分（松杆即停保持角度）
#   - 未收到首帧前不发布 gimbal_cmd，避免抢占云台其它模式
#
# 用法示例（真机）:
#   ros2 run spr_remote_control rc_serial_remote --ros-args \
#     -p device:=/dev/ttyUSB0 -p baudrate:=100000 -p parity:=even
import rclpy
from rclpy.node import Node
import serial

from spr_remote_control.remote_core import (
    RemoteCore, GIMBAL_MODE_HOLD, GIMBAL_MODE_AIM, GIMBAL_MODE_REMOTE)
from spr_remote_control.dbus import (
    DBUS_FRAME_LEN, SW_UP, SW_MID, SW_DOWN, decode_dbus_frame, norm_stick)

_PARITY = {'none': serial.PARITY_NONE, 'even': serial.PARITY_EVEN,
           'odd': serial.PARITY_ODD}
# 右拨杆 s1 位置 → 云台模式（下=保持 中=遥控 上=自瞄）
_S1_MODE = {SW_DOWN: GIMBAL_MODE_HOLD,
            SW_MID: GIMBAL_MODE_REMOTE,
            SW_UP: GIMBAL_MODE_AIM}
# 缓冲上限（约 4 帧），防串口粘包/异常涨满
_MAX_BUF = DBUS_FRAME_LEN * 4


class RcSerialRemote(Node):
    """DR16 串口遥控源：读串口 → 解 DBUS → 映射 → 写入 RemoteCore。"""

    def __init__(self):
        super().__init__('rc_serial_remote')
        self._declare_params()
        self._read_params()

        # 串口/缓冲
        self._ser = None
        self._buf = bytearray()
        self._open_warned = False
        self._last_open_attempt = 0.0

        # 遥控状态
        self._snap = None           # 最近一帧解码结果
        self._got_frame = False     # 是否收到过有效帧
        self._last_frame_t = None   # 最近有效帧时刻
        self._lost = False          # 是否处于失联停车状态

        # 核心：publish_hz 与 DBUS 100Hz 对齐，读帧+发布同节拍
        self.core = RemoteCore(self, publish_hz=self._publish_hz)
        self.core.gimbal_enabled = False  # 收到首帧后再开
        self.core.on_tick = self._on_tick

        self.get_logger().info(
            f'DR16 串口遥控就绪: {self._device} @ {self._baudrate}bps, '
            f'{self._parity}, publish={self._publish_hz}Hz')

    # ---------------- 参数 ----------------
    def _declare_params(self):
        self.declare_parameter('device', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 100000)
        self.declare_parameter('parity', 'even')   # none/even/odd
        self.declare_parameter('publish_hz', 100.0)
        # 控制映射
        self.declare_parameter('deadzone', 0.08)
        self.declare_parameter('max_vx', 1.0)      # m/s
        self.declare_parameter('max_vy', 1.0)      # m/s
        self.declare_parameter('max_wz', 2.0)      # rad/s
        self.declare_parameter('pitch_rate', 1.0)  # rad/s @满杆
        # 摇杆方向符号（真机校准：前推/左推为正则填 +1，反了填 -1）
        self.declare_parameter('vx_sign', 1.0)     # 左杆竖直→vx
        self.declare_parameter('vy_sign', 1.0)     # 左杆水平→vy
        self.declare_parameter('wz_sign', 1.0)     # 右杆水平→wz
        self.declare_parameter('pitch_sign', 1.0)  # 右杆竖直→pitch
        # 失联
        self.declare_parameter('lost_timeout', 0.2)  # s

    def _read_params(self):
        g = self.get_parameter
        self._device = g('device').value
        self._baudrate = int(g('baudrate').value)
        self._parity = str(g('parity').value).lower()
        self._publish_hz = float(g('publish_hz').value)
        self._deadzone = float(g('deadzone').value)
        self._max_vx = float(g('max_vx').value)
        self._max_vy = float(g('max_vy').value)
        self._max_wz = float(g('max_wz').value)
        self._pitch_rate = float(g('pitch_rate').value)
        self._vx_sign = float(g('vx_sign').value)
        self._vy_sign = float(g('vy_sign').value)
        self._wz_sign = float(g('wz_sign').value)
        self._pitch_sign = float(g('pitch_sign').value)
        self._lost_timeout = float(g('lost_timeout').value)

    # ---------------- 串口 ----------------
    def _ensure_open(self):
        if self._ser is not None:
            return True
        now = self.get_clock().now().nanoseconds / 1e9
        # 打开失败时最多每 1s 重试一次（支持遥控器接收机热插拔）
        if now - self._last_open_attempt < 1.0:
            return False
        self._last_open_attempt = now
        try:
            self._ser = serial.Serial(
                port=self._device, baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=_PARITY.get(self._parity, serial.PARITY_NONE),
                stopbits=serial.STOPBITS_ONE, timeout=0)
            self._ser.reset_input_buffer()
            self._buf.clear()
            self._open_warned = False
            self.get_logger().info(f'已打开串口 {self._device}')
            return True
        except Exception as e:
            if not self._open_warned:
                self.get_logger().error(
                    f'打开串口失败 {self._device}: {e}（将每 1s 自动重试）')
                self._open_warned = True
            return False

    def _close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            self.get_logger().warn('串口已关闭，等待重连')

    def _read_frames(self):
        """读入可用字节并按 18B 切片解码；非法窗口滑动 1 字节重同步。"""
        try:
            n = self._ser.in_waiting
        except Exception:
            self._close()
            return []
        if n <= 0:
            return []
        try:
            data = self._ser.read(n)
        except Exception:
            self._close()
            return []
        self._buf.extend(data)
        if len(self._buf) > _MAX_BUF:
            del self._buf[:-_MAX_BUF]

        frames = []
        while len(self._buf) >= DBUS_FRAME_LEN:
            cand = bytes(self._buf[:DBUS_FRAME_LEN])
            rc = decode_dbus_frame(cand)
            if rc.valid:
                frames.append(rc)
                del self._buf[:DBUS_FRAME_LEN]
            else:
                del self._buf[:1]  # 滑动重同步
        return frames

    # ---------------- 每拍处理 ----------------
    def _on_tick(self, dt):
        if self._ensure_open():
            for rc in self._read_frames():
                self._handle_frame(rc)
        self._apply(dt)

    def _handle_frame(self, rc):
        self._snap = rc
        now = self.get_clock().now()
        if not self._got_frame:
            self._got_frame = True
            self.core.gimbal_enabled = True
            self.get_logger().info('已收到首帧遥控数据')
        if self._lost:
            self.get_logger().warn('遥控恢复')
            self._lost = False
        self._last_frame_t = now

    def _apply(self, dt):
        core = self.core
        # 未收到首帧：底盘保持 0，不发布云台（gimbal_enabled=False）
        if not self._got_frame:
            return
        # 失联检测：超时无有效帧 → 底盘停车 + 告警（云台保持最后角度）
        age = (self.get_clock().now() - self._last_frame_t).nanoseconds / 1e9
        if age > self._lost_timeout:
            if not self._lost:
                self.get_logger().warn('遥控失联（无帧>%.0fms）→ 底盘停车', age * 1000)
                self._lost = True
            core.set_chassis()
            return
        self._lost = False

        rc = self._snap
        s0, s1 = rc.s[0], rc.s[1]

        # 云台模式：右拨杆 s1 三档
        mode = _S1_MODE.get(s1, core.gimbal_mode)
        core.set_mode(mode)

        # 底盘：左拨杆 s0 非"下"档才允许（下档=急停/使能关）
        if s0 == SW_DOWN:
            core.set_chassis()
        else:
            vx = norm_stick(rc.ch[2], self._deadzone) * self._vx_sign * self._max_vx
            vy = norm_stick(rc.ch[3], self._deadzone) * self._vy_sign * self._max_vy
            wz = norm_stick(rc.ch[0], self._deadzone) * self._wz_sign * self._max_wz
            core.set_chassis(vx, vy, wz)

        # 云台：遥控模式(mode 3)下 右杆竖直 → pitch 绝对角增量积分（松杆停）
        if mode == GIMBAL_MODE_REMOTE:
            pitch = core.angles['pitch']
            pitch += (norm_stick(rc.ch[1], self._deadzone)
                      * self._pitch_sign * self._pitch_rate * dt)
            core.set_angle('pitch', pitch)
            # TODO(IMU/云台稳定)：small_yaw / big_yaw 增量与底盘 wz 的耦合，
            #   待加 IMU 后按"云台绝对角"方案联调。


def main(args=None):
    rclpy.init(args=args)
    node = RcSerialRemote()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
