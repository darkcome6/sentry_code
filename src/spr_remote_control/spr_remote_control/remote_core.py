#!/usr/bin/env python3
# 哨兵遥控共享核心：统一发布底盘(cmd_vel)与云台(gimbal_cmd)。
# 键盘 / 串口遥控等输入源只修改目标状态，发布节拍与话题对本包内所有源一致，
# 使各源对底盘/云台控制器呈现完全相同的对外接口（仅输入来源不同）。
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


class RemoteCore:
    """与输入源无关的遥控核心。

    输入源通过本类接口修改目标状态；核心以固定频率发布
    Twist(cmd_vel) 与 GimbalCmd(gimbal_controller/gimbal_cmd)。
    持续发布避免 BEST_EFFORT 首帧丢失，并保证遥控模式持续生效。
    """

    def __init__(self, node: Node, publish_hz: float = 20.0):
        self.node = node
        self.cmd_vel_pub_ = node.create_publisher(Twist, 'cmd_vel', 10)
        self.gimbal_pub_ = node.create_publisher(
            GimbalCmd, 'gimbal_controller/gimbal_cmd', 10)

        # 底盘目标（m/s, rad/s）
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        # 云台目标（绝对角度参考，rad）
        self.gimbal_mode = GIMBAL_MODE_REMOTE
        self.angles = {joint: 0.0 for joint in GIMBAL_JOINTS}
        # 是否发布云台指令：串口源在未收到首帧前保持 False，避免抢占其它模式
        self.gimbal_enabled = True

        # 每拍回调（供串口源做增量积分 / 失联检测），dt = 1/publish_hz
        self.on_tick = None
        self.dt = 1.0 / publish_hz
        node.create_timer(self.dt, self._tick)

    # ---------- 底盘 ----------
    def set_chassis(self, vx=0.0, vy=0.0, wz=0.0):
        self.vx = float(vx)
        self.vy = float(vy)
        self.wz = float(wz)

    # ---------- 云台 ----------
    def set_mode(self, mode: int):
        self.gimbal_mode = int(mode)

    def set_angle(self, joint: str, value: float):
        if joint in self.angles:
            self.angles[joint] = float(value)

    def step_angle(self, joint: str, delta: float):
        """绝对角参考增量步进（键盘离散按键用）。"""
        if joint in self.angles:
            self.angles[joint] += float(delta)

    # ---------- 周期执行 ----------
    def _tick(self):
        if self.on_tick is not None:
            self.on_tick(self.dt)
        self.publish()

    def publish(self):
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.angular.z = self.wz
        self.cmd_vel_pub_.publish(twist)

        if self.gimbal_enabled:
            cmd = GimbalCmd()
            cmd.mode = self.gimbal_mode
            cmd.pitch_angle = self.angles['pitch']
            cmd.small_yaw_angle = self.angles['small_yaw']
            cmd.big_yaw_angle = self.angles['big_yaw']
            self.gimbal_pub_.publish(cmd)
