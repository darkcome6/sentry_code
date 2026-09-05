#!/usr/bin/env python3
# 哨兵键盘遥控（输入源之一）：在无真实遥控器/串口时用于 mujoco / mock 调试。
# 对外接口与本包其它源一致：只经 RemoteCore 统一发布 cmd_vel 与 gimbal_cmd。
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from spr_remote_control.remote_core import RemoteCore


class KeyboardRemote(Node):
    """键盘输入源：离散档位/步进语义，映射后写入 RemoteCore。"""

    def __init__(self):
        super().__init__('keyboard_remote')
        # 键盘离散档位逻辑
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.speed = 0.5       # 线速度档 (m/s)
        self.turn = 0.5        # 角速度档 (rad/s)
        self.angle_step = 0.05  # 每按一次角度步进 (rad)

        self.core = RemoteCore(self, publish_hz=20.0)
        self.print_help()

    # ---------------- 界面 ----------------
    def print_help(self):
        self.get_logger().info(
            '\n' + '=' * 50 +
            '\n 哨兵键盘遥控（键盘源）' +
            '\n 底盘(cmd_vel):' +
            '\n   W/S 前进/后退   A/D 横移   Q/E 旋转' +
            '\n   空格 停止       +/- 调速' +
            '\n 云台(gimbal_cmd):' +
            '\n   0/1/2/3 模式(保持/扫描/自瞄/遥控)' +
            '\n   遥控模式(3)下需按键调角度云台才会动:' +
            '\n   I/K pitch±   J/L small_yaw±   U/O big_yaw±' +
            '\n H 显示帮助    X 退出' +
            '\n' + '=' * 50)
        self.show_status()

    def show_status(self):
        """单行状态栏：用 \\r 覆盖当前行，不换行刷屏。"""
        c = self.core
        sys.stdout.write(
            f"\r[模式={c.gimbal_mode}  pitch={c.angles['pitch']:+.2f}"
            f"  syaw={c.angles['small_yaw']:+.2f}"
            f"  byaw={c.angles['big_yaw']:+.2f}"
            f"  vx={c.vx:+.2f}  vy={c.vy:+.2f}  wz={c.wz:+.2f}]" + ' ' * 8)
        sys.stdout.flush()

    # ---------------- 按键语义 ----------------
    def on_key(self, key):
        c = self.core
        if key == 'h':
            self.print_help()
            return True
        # ---- 底盘 ----
        if key == 'w':
            self.vx = self.speed
        elif key == 's':
            self.vx = -self.speed
        elif key == 'a':
            self.vy = self.speed
        elif key == 'd':
            self.vy = -self.speed
        elif key == 'q':
            self.wz = self.turn
        elif key == 'e':
            self.wz = -self.turn
        elif key == ' ':
            self.vx = self.vy = self.wz = 0.0
        elif key == '+':
            self.speed = min(self.speed + 0.2, 3.0)
        elif key == '-':
            self.speed = max(self.speed - 0.2, 0.1)
        # ---- 云台模式 ----
        elif key in ('0', '1', '2', '3'):
            c.set_mode(int(key))
        # ---- 云台角度微调 ----
        elif key == 'i':
            c.step_angle('pitch', self.angle_step)
        elif key == 'k':
            c.step_angle('pitch', -self.angle_step)
        elif key == 'j':
            c.step_angle('small_yaw', self.angle_step)
        elif key == 'l':
            c.step_angle('small_yaw', -self.angle_step)
        elif key == 'u':
            c.step_angle('big_yaw', self.angle_step)
        elif key == 'o':
            c.step_angle('big_yaw', -self.angle_step)
        elif key in ('x', 'X'):
            return False

        c.set_chassis(self.vx, self.vy, self.wz)
        self.show_status()
        return True


def get_key(timeout=0.1):
    """非阻塞读一个按键，超时返回 None。"""
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardRemote()
    try:
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(0.02)
            if key is not None and not node.on_key(key):
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, sys.stdin.fileno())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
