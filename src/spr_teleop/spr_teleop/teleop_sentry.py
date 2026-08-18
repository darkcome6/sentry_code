#!/usr/bin/env python3
# 哨兵键盘遥控：一键同时控制底盘(cmd_vel)与云台(gimbal_cmd)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from spr_msgs.msg import GimbalCmd

import sys
import select
import termios
import tty


class TeleopSentry(Node):
    def __init__(self):
        super().__init__('teleop_sentry')
        self.cmd_vel_pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gimbal_pub_ = self.create_publisher(GimbalCmd, 'gimbal_controller/gimbal_cmd', 10)

        # 底盘状态
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.speed = 0.5    # 线速度档 (m/s)
        self.turn = 0.5     # 角速度档 (rad/s)

        # 云台状态
        self.mode = 3       # 默认遥控模式
        self.pitch = 0.0
        self.small_yaw = 0.0
        self.big_yaw = 0.0
        self.angle_step = 0.05  # 每按一次角度步进 (rad)

        # 20Hz 持续发布（避免 BEST_EFFORT 首帧丢失 + 遥控模式持续生效）
        self.timer = self.create_timer(0.05, self.publish)

        self.print_help()

    def print_help(self):
        self.get_logger().info(
            '\n' + '=' * 50 +
            '\n 哨兵键盘遥控' +
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
        """单行状态栏：用 \\r 覆盖当前行，不换行刷屏"""
        sys.stdout.write(
            f"\r[模式={self.mode}  pitch={self.pitch:+.2f}  syaw={self.small_yaw:+.2f}"
            f"  byaw={self.big_yaw:+.2f}  vx={self.vx:+.2f}  vy={self.vy:+.2f}"
            f"  wz={self.wz:+.2f}]" + ' ' * 8)
        sys.stdout.flush()

    def publish(self):
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.angular.z = self.wz
        self.cmd_vel_pub_.publish(twist)

        cmd = GimbalCmd()
        cmd.mode = self.mode
        cmd.pitch_angle = self.pitch
        cmd.small_yaw_angle = self.small_yaw
        cmd.big_yaw_angle = self.big_yaw
        self.gimbal_pub_.publish(cmd)

    def on_key(self, key):
        # 按 H 重新显示帮助
        if key == 'h':
            self.print_help()
            return True
        # ---- 底盘 ----
        if key == 'w':
            self.vx = self.speed
        elif key == 's':
            self.vx = -self.speed
        elif key == 'a':
            self.vy = self.speed          # 左移
        elif key == 'd':
            self.vy = -self.speed         # 右移
        elif key == 'q':
            self.wz = self.turn           # 逆时针
        elif key == 'e':
            self.wz = -self.turn          # 顺时针
        elif key == ' ':
            self.vx = self.vy = self.wz = 0.0
        elif key == '+':
            self.speed = min(self.speed + 0.2, 3.0)
        elif key == '-':
            self.speed = max(self.speed - 0.2, 0.1)
        # ---- 云台模式 ----
        elif key == '0':
            self.mode = 0
        elif key == '1':
            self.mode = 1
        elif key == '2':
            self.mode = 2
        elif key == '3':
            self.mode = 3
        # ---- 云台角度微调 ----
        elif key == 'i':
            self.pitch += self.angle_step
        elif key == 'k':
            self.pitch -= self.angle_step
        elif key == 'j':
            self.small_yaw += self.angle_step
        elif key == 'l':
            self.small_yaw -= self.angle_step
        elif key == 'u':
            self.big_yaw += self.angle_step
        elif key == 'o':
            self.big_yaw -= self.angle_step
        elif key in ('x', 'X'):
            return False
        # 单行刷新状态栏（不刷屏）
        self.show_status()
        return True


def get_key(timeout=0.1):
    """非阻塞读一个按键，超时返回 None"""
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None


def main(args=None):
    rclpy.init(args=args)
    node = TeleopSentry()
    try:
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(0.02)
            if key is not None:
                if not node.on_key(key):
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, sys.stdin.fileno())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
