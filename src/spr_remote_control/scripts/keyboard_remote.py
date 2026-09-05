#!/usr/bin/env python3
# 哨兵键盘遥控节点（输入源之一）—— 自包含脚本，随统一遥控包 spr_remote_control
# （C++ 主体）安装。在无真实遥控器/串口时用于 mujoco / mock 调试。
# 对外接口与串口源(rc_serial_remote_cpp)一致，只经本节点状态持续发布：
#   - cmd_vel (geometry_msgs/Twist)：底盘线/角速度
#   - gimbal_controller/gimbal_cmd (spr_msgs/GimbalCmd)：云台模式 + 三轴绝对角
#
# 用法:
#   ros2 run spr_remote_control keyboard_remote
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from spr_msgs.msg import GimbalCmd

# 云台模式（与 spr_sentry_controllers/gimbal_controller 约定一致）
GIMBAL_MODE_HOLD = 0    # 保持
GIMBAL_MODE_SCAN = 1    # 扫描
GIMBAL_MODE_AIM = 2     # 自瞄
GIMBAL_MODE_REMOTE = 3  # 遥控

GIMBAL_JOINTS = ('pitch', 'small_yaw', 'big_yaw')


class KeyboardRemote(Node):
    """键盘输入源：离散档位/步进语义，20Hz 持续发布 cmd_vel + gimbal_cmd。"""

    def __init__(self):
        super().__init__('keyboard_remote')
        # 键盘离散档位逻辑
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.speed = 0.5       # 线速度档 (m/s)
        self.turn = 0.5        # 角速度档 (rad/s)
        self.angle_step = 0.05  # 每按一次角度步进 (rad)

        # 云台目标状态（默认遥控模式 3，三轴绝对角参考）
        self.gimbal_mode = GIMBAL_MODE_REMOTE
        self.angles = {joint: 0.0 for joint in GIMBAL_JOINTS}

        # 持续发布：避免 BEST_EFFORT 首帧丢失，保证遥控模式持续生效
        self.cmd_vel_pub_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gimbal_pub_ = self.create_publisher(
            GimbalCmd, 'gimbal_controller/gimbal_cmd', 10)
        self.create_timer(1.0 / 20.0, self._tick)   # 20Hz
        self.print_help()

    # ---------------- 周期发布 ----------------
    def _tick(self):
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.angular.z = self.wz
        self.cmd_vel_pub_.publish(twist)

        cmd = GimbalCmd()
        cmd.mode = self.gimbal_mode
        cmd.pitch_angle = self.angles['pitch']
        cmd.small_yaw_angle = self.angles['small_yaw']
        cmd.big_yaw_angle = self.angles['big_yaw']
        self.gimbal_pub_.publish(cmd)

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
        sys.stdout.write(
            f"\r[模式={self.gimbal_mode}  pitch={self.angles['pitch']:+.2f}"
            f"  syaw={self.angles['small_yaw']:+.2f}"
            f"  byaw={self.angles['big_yaw']:+.2f}"
            f"  vx={self.vx:+.2f}  vy={self.vy:+.2f}  wz={self.wz:+.2f}]"
            + ' ' * 8)
        sys.stdout.flush()

    # ---------------- 按键语义 ----------------
    def on_key(self, key):
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
            self.gimbal_mode = int(key)
        # ---- 云台角度微调（绝对角参考步进）----
        elif key == 'i':
            self.angles['pitch'] += self.angle_step
        elif key == 'k':
            self.angles['pitch'] -= self.angle_step
        elif key == 'j':
            self.angles['small_yaw'] += self.angle_step
        elif key == 'l':
            self.angles['small_yaw'] -= self.angle_step
        elif key == 'u':
            self.angles['big_yaw'] += self.angle_step
        elif key == 'o':
            self.angles['big_yaw'] -= self.angle_step
        elif key in ('x', 'X'):
            return False

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
