#!/usr/bin/env python3
# DJI / RoboMaster DR16 遥控器 DBUS 协议（裸 18 字节帧）解码。
# 位运算移植自 DJI 官方 remote_control.c 的 sbus_to_rc（PT_link_en=0 配置）。
#
# DBUS 帧 @~100Hz、无帧头无校验；判帧依赖通道/拨杆合法性
# （对应官方 RC_data_is_error 的检查），供上层滑动重同步使用。
import math
from dataclasses import dataclass, field

DBUS_FRAME_LEN = 18
CH_OFFSET = 1024       # 摇杆原始值中值偏移（RC_CH_VALUE_OFFSET）
CH_VALID_ABS = 700     # 去偏后合法幅值上限（RC_CHANNAL_ERROR_VALUE）
CH_STICK_RANGE = 660   # 去偏后满量程幅值（norm_stick 归一化用）

# 拨杆取值（PT_link_en=0 映射：上=1 中=3 下=2；0=非法/过渡）
SW_UP = 1
SW_MID = 3
SW_DOWN = 2


@dataclass
class RCData:
    """单帧解码结果。ch[0..4] 为去偏后的有符号摇杆值；s[0]/s[1] 为左右拨杆。"""
    ch: list = field(default_factory=lambda: [0, 0, 0, 0, 0])
    s: list = field(default_factory=lambda: [0, 0])
    mouse_x: int = 0
    mouse_y: int = 0
    mouse_z: int = 0
    press_l: int = 0
    press_r: int = 0
    key: int = 0
    valid: bool = False


def decode_dbus_frame(buf) -> RCData:
    """解码一帧 18 字节 DBUS。帧非法时返回 valid=False（字段归零）。"""
    rc = RCData()
    if buf is None or len(buf) < DBUS_FRAME_LEN:
        return rc

    b = buf
    ch = [
        ((b[0] | (b[1] << 8)) & 0x07FF) - CH_OFFSET,           # ch0 右摇杆水平
        (((b[1] >> 3) | (b[2] << 5)) & 0x07FF) - CH_OFFSET,    # ch1 右摇杆竖直
        (((b[2] >> 6) | (b[3] << 2) | (b[4] << 10)) & 0x07FF) - CH_OFFSET,   # ch2 左摇杆竖直
        (((b[4] >> 1) | (b[5] << 7)) & 0x07FF) - CH_OFFSET,    # ch3 左摇杆水平
        ((b[16] | (b[17] << 8)) & 0x07FF) - CH_OFFSET,         # ch4 空/波轮
    ]
    s0 = (b[5] >> 4) & 0x03
    s1 = (b[5] >> 6) & 0x03

    # 合法性判帧：拨杆非 0 且四主通道幅值不越界
    if s0 == 0 or s1 == 0:
        return rc
    if any(abs(c) > CH_VALID_ABS for c in ch[:4]):
        return rc

    rc.ch = ch
    rc.s = [s0, s1]
    rc.mouse_x = b[6] | (b[7] << 8)
    rc.mouse_y = b[8] | (b[9] << 8)
    rc.mouse_z = b[10] | (b[11] << 8)
    rc.press_l = b[12]
    rc.press_r = b[13]
    rc.key = b[14] | (b[15] << 8)
    rc.valid = True
    return rc


def norm_stick(raw: float, deadzone: float = 0.08) -> float:
    """摇杆去偏值 → [-1, 1]：死区内回 0，死区外重新线性化。"""
    v = max(-1.0, min(1.0, raw / CH_STICK_RANGE))
    if abs(v) < deadzone:
        return 0.0
    v = (v - math.copysign(deadzone, v)) / (1.0 - deadzone)
    return max(-1.0, min(1.0, v))
