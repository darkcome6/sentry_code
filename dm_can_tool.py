#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dm_can_tool.py —— 达妙(DM)电机调试小工具

直接读 CAN socket，实时查看电机状态(编码/位置/速度/力矩/温度/err)，
并支持 标零 / 使能 / 失能 / 清错 / 持续维持(hold)。无需 ROS、无需手动解析十六进制。

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
用法（默认 can0；命令帧 ID=电机ID，反馈帧 ID=0x50+电机ID）:
  ./dm_can_tool.py monitor [id]            # 实时监视（不写 id 则显示总线上所有达妙电机）
  ./dm_can_tool.py hold <id>               # ★ 推荐标零姿势：使能并持续发零力矩帧维持
                                           #   （达妙 200ms 收不到控制帧即报 0xD 通讯丢失，
                                           #    纯 enable 发完就停必红灯闪烁）。
                                           #   按键: z=标零  e=使能  d=失能  c=清错  q=退出(自动失能)
  ./dm_can_tool.py setzero <id>            # 标零(多发 0xFE)：把当前输出轴位置存为 0
  ./dm_can_tool.py enable  <id>            # 使能（自动先清错 0xFB，再连发 0xFC x10）
  ./dm_can_tool.py disable <id>            # 失能（失能后可自由手转）
  ./dm_can_tool.py clear   <id>            # 清除错误

可选参数:
  -i/--iface <can口>    默认 can0
  --pm <pos_max>        位置量程(rad)，须与达妙调试助手 PMAX 一致（DM 单圈 ±π=3.14），默认 3.14
  --vm <vel_max>        速度量程(rad/s)，默认 30
  --tm <tor_max>        力矩量程(N·m)，默认 10

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
达妙 CAN 命令（数据段 8 字节，其余 0xFF，帧 ID=电机 CAN ID）:
  0xFB = 清除错误      —— 电机故障锁存(红灯闪烁)时 0xFC 会被忽略，必须先清错
  0xFC = 使能          —— 需连发 ~10+ 帧才可靠；使能后必须持续收控制帧维持
  0xFD = 失能
  0xFE = 保存位置零点  —— 把当前输出轴位置设为 0。⚠️ 不是复位！曾误当复位用把零点写乱

err 状态码（反馈帧 D0 高4位，低4位=电机ID）:
  0=失能(红灯常亮)  1=使能(绿灯常亮)  3/4/5=校准/传感器异常  8=超压  9=欠压
  A=过电流  B=MOS过温  C=线圈过温  D=通讯丢失(红灯闪烁)  E=过载

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
经验 / 坑（2026-09-03 真机验证）:
  1. 电机 200ms 收不到控制帧 → 报 0xD 通讯丢失 → 红灯闪烁。必须持续发帧：
     real 由 write() 维持；纯工具请用 hold 模式。
  2. run_sentry.sh real 运行时，其 write() 自愈(err!=1 时每 100ms 补发 0xFC)
     会接管使能 → 外部 disable 无效。要手转/标零必须先 ./stop_sentry.sh -q。
  3. pos_max 必须与达妙调试助手 PMAX 一致。曾配 12.5 而电机实为 3.14(单圈)，
     位置反馈被放大 ~4 倍 → 位置环猛打。改后须重新 colcon build。
  4. 失能态直接 0xFE 标零可能异常(位置被推到满量程)，推荐在 hold(使能+维持) 里按 z 标零。
  5. 标零后回读编码应变 0x0000 附近、位置≈0；否则零点没写入。

示例：
  ./dm_can_tool.py hold 4       # 使能+持续维持 pitch，按 z 标零，q 退出
  ./dm_can_tool.py monitor 4    # 实时看 pitch 状态
"""

import socket
import struct
import sys
import time

# ---------------- CAN frame ----------------
CAN_FRAME_FMT = "=IB3x8s"
CAN_RAW = socket.CAN_RAW
AF_CAN = socket.AF_CAN

# 达妙命令尾字节（数据段 8 字节：其余 0xFF）
CMD_CLEAR = 0xFB   # 清除错误（红灯闪烁故障锁存时先发这个）
CMD_ENABLE = 0xFC  # 使能
CMD_DISABLE = 0xFD  # 失能
CMD_SET_ZERO = 0xFE  # 保存位置零点（把当前输出轴位置设为 0）

# 反馈帧 = 0x50 + 电机ID
RX_BASE = 0x50

ERR_TEXT = {
    0: "失能(红灯常亮)",
    1: "使能(绿灯常亮)",
    3: "故障:输出轴校准异常",
    4: "故障:传感器输出异常",
    5: "故障:电机编码器校准异常",
    8: "故障:超压",
    9: "故障:欠压",
    10: "故障:过电流",
    11: "故障:MOS过温",
    12: "故障:线圈过温",
    13: "故障:通讯丢失",
    14: "故障:过载",
}


def open_can(iface):
    s = socket.socket(AF_CAN, socket.SOCK_RAW, CAN_RAW)
    try:
        s.bind((iface,))
    except PermissionError:
        print(f"[ERR] 打开 {iface} 权限不足，请用: sudo {sys.argv[0]} ...")
        sys.exit(1)
    except OSError as e:
        print(f"[ERR] 打开 {iface} 失败: {e}（接口是否存在/已 up？用 ./setting.sh {iface} 拉起）")
        sys.exit(1)
    return s


def send_frame(s, can_id, data):
    """发送标准数据帧。can_id 必须 < 0x800（标准帧，不置 EFF 位）。"""
    if len(data) != 8:
        raise ValueError("data must be 8 bytes")
    s.send(struct.pack(CAN_FRAME_FMT, can_id, 8, bytes(data)))


def send_cmd(s, motor_id, cmd_byte):
    send_frame(s, motor_id, [0xFF] * 7 + [cmd_byte])


def send_cmd_repeat(s, motor_id, cmd_byte, count=5, interval=0.06):
    """达妙命令单帧易被忽略（实测需连发 ~10+ 帧才可靠），这里连续多发。"""
    for _ in range(count):
        send_cmd(s, motor_id, cmd_byte)
        time.sleep(interval)


def recv_frame(s, timeout=1.0):
    s.settimeout(timeout)
    try:
        raw = s.recv(16)
    except socket.timeout:
        return None
    can_id, dlc = struct.unpack("=IB", raw[:5])
    data = list(raw[8:16])
    return can_id & 0x1FFFFFFF, dlc, data


def int16(v):
    v &= 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def parse_dm(data, pos_max, vel_max, tor_max):
    """解析达妙 MIT 反馈帧（与 spr_hw_interface decode_dm_feedback 一致）：
    D0: err|id   D1D2: pos16  D3D4[7:4]: v12  D4[3:0]D5: t12  D6:D7 温度
    """
    err = data[0] >> 4
    mid = data[0] & 0x0F
    pos_raw = int16((data[1] << 8) | data[2])
    v_int = ((data[3] << 4) | (data[4] >> 4)) & 0xFFF
    t_int = (((data[4] & 0x0F) << 8) | data[5]) & 0xFFF
    # 12bit 定点→浮点：无符号 0~4095 线性映射到 [-max, +max]
    # （与 spr_hw_interface uint_to_float 一致；勿先转有符号，否则双重偏移）
    pos_rad = pos_raw * pos_max / 32768.0
    vel = v_int * (2 * vel_max) / 4095.0 - vel_max
    tor = t_int * (2 * tor_max) / 4095.0 - tor_max
    return {
        "err": err, "id": mid, "pos_raw": pos_raw,
        "pos_rad": pos_rad, "vel": vel, "tor": tor,
        "mos_t": data[6], "coil_t": data[7],
    }


def fmt_state(m, pos_max):
    et = ERR_TEXT.get(m["err"], f"未知故障码:{m['err']}")
    return (f"[电机{m['id']}] err={m['err']}({et}) "
            f"编码={m['pos_raw']}(0x{m['pos_raw'] & 0xFFFF:04X}) "
            f"位置={m['pos_rad']:+.3f}rad "
            f"速度={m['vel']:+.2f}rad/s "
            f"力矩={m['tor']:+.3f}Nm "
            f"MOS={m['mos_t']}°C 线圈={m['coil_t']}°C")


def cmd_monitor(args):
    s = open_can(args.iface)
    print(f"监视 {args.iface} 达妙电机反馈帧 (Ctrl+C 退出) ...")
    if args.motor_id is not None:
        print(f"只看电机 ID={args.motor_id}（反馈帧 0x{RX_BASE + args.motor_id:03X}）\n")
    while True:
        r = recv_frame(s, 1.0)
        if r is None:
            continue
        can_id, dlc, data = r
        if dlc < 8:
            continue
        # 达妙反馈帧：can_id = 0x50 + 电机ID（标准帧，本工具默认过滤该范围）
        if not (RX_BASE <= can_id <= RX_BASE + 0x0F):
            continue
        m = parse_dm(data, args.pm, args.vm, args.tm)
        if args.motor_id is not None and m["id"] != args.motor_id:
            continue
        line = fmt_state(m, args.pm)
        sys.stdout.write("\r" + line + " " * 20)
        sys.stdout.flush()


def cmd_single(args, name, cmd_byte, hint="", pre_clear=False, count=5):
    s = open_can(args.iface)
    if args.motor_id is None:
        print("[ERR] 需要指定电机 ID，如: ./dm_can_tool.py %s 4" % name)
        sys.exit(1)
    mid = args.motor_id
    if pre_clear:
        # 故障锁存(红灯闪烁)时 0xFC 会被忽略，先 0xFB 清错
        print(f"先发 0xfb 清除错误 x3 ...")
        send_cmd_repeat(s, mid, CMD_CLEAR, 3, 0.05)
        time.sleep(0.1)
    print(f"向电机 ID={mid}（{args.iface}，帧 0x{mid:03X}）发送 {cmd_byte:#04x} x{count} ...")
    send_cmd_repeat(s, mid, cmd_byte, count, 0.06)
    if hint:
        print(hint)
    # 连续读几帧，取该电机最后一帧作为最新回读（避免读到旧帧/别的电机帧）
    last = None
    for _ in range(20):
        r = recv_frame(s, 0.5)
        if r is None:
            break
        can_id, dlc, data = r
        if dlc >= 8 and can_id == RX_BASE + mid:
            last = data
    if last is not None:
        print("回读: " + fmt_state(parse_dm(last, args.pm, args.vm, args.tm), args.pm))


def encode_mit_frame(p, v, kp, kd, t, pos_max, vel_max, tor_max):
    """组装达妙 MIT 控制帧（与 spr_hw_interface encode_mit_frame 一致）：
    布局 p16/v12/kp12/kd12/t12；kp∈[0,500] kd∈[0,5]"""
    def f2u(x, xmin, xmax, bits):
        return int((x - xmin) * ((1 << bits) - 1) / (xmax - xmin))
    p_i = f2u(p, -pos_max, pos_max, 16)
    v_i = f2u(v, -vel_max, vel_max, 12)
    kp_i = f2u(kp, 0.0, 500.0, 12)
    kd_i = f2u(kd, 0.0, 5.0, 12)
    t_i = f2u(t, -tor_max, tor_max, 12)
    f = [0] * 8
    f[0] = (p_i >> 8) & 0xFF; f[1] = p_i & 0xFF
    f[2] = (v_i >> 4) & 0xFF
    f[3] = ((v_i & 0x0F) << 4) | ((kp_i >> 8) & 0x0F)
    f[4] = kp_i & 0xFF
    f[5] = (kd_i >> 4) & 0xFF
    f[6] = ((kd_i & 0x0F) << 4) | ((t_i >> 8) & 0x0F)
    f[7] = t_i & 0xFF
    return f


def cmd_hold(args):
    """使能并持续发零力矩帧维持（达妙 200ms 收不到控制帧即报 0xD 通讯丢失红灯闪烁）。
    交互按键: z=标零(当前位置存0)  e=使能  d=失能  c=清错  q=退出(自动失能)"""
    import select
    import termios
    import tty
    s = open_can(args.iface)
    if args.motor_id is None:
        print("[ERR] hold 需要指定电机 ID，如: ./dm_can_tool.py hold 4")
        sys.exit(1)
    mid = args.motor_id
    print(f"hold: 先清错+使能 ID={mid}，随后持续发零力矩帧维持使能 ...")
    send_cmd_repeat(s, mid, CMD_CLEAR, 3, 0.05)
    time.sleep(0.1)
    send_cmd_repeat(s, mid, CMD_ENABLE, 10, 0.05)
    hold_frame = encode_mit_frame(0.0, 0.0, 0.0, 0.0, 0.0, args.pm, args.vm, args.tm)
    print("按键: z=标零(当前位置存0)  e=使能  d=失能  c=清错  q=退出并失能")
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            # 维持使能：持续发零力矩控制帧（间隔 ~10ms << 电机 200ms 超时）
            send_frame(s, mid, hold_frame)
            if select.select([sys.stdin], [], [], 0)[0]:
                ch = sys.stdin.read(1).lower()
                if ch == "q":
                    break
                elif ch == "z":
                    send_cmd_repeat(s, mid, CMD_SET_ZERO, 8, 0.05)
                    print("\n[标零] 已将当前输出轴位置保存为零点")
                elif ch == "e":
                    send_cmd_repeat(s, mid, CMD_ENABLE, 10, 0.05)
                    print("\n[使能] 已连发 0xFC x10")
                elif ch == "d":
                    send_cmd_repeat(s, mid, CMD_DISABLE, 5, 0.05)
                    print("\n[失能] 可自由手转（此时不再维持，尽快转到位再按 e 或重进 hold）")
                elif ch == "c":
                    send_cmd_repeat(s, mid, CMD_CLEAR, 5, 0.05)
                    print("\n[清错]")
            # 实时显示反馈
            r = recv_frame(s, 0.0)
            if r is not None:
                can_id, dlc, data = r
                if dlc >= 8 and can_id == RX_BASE + mid:
                    m = parse_dm(data, args.pm, args.vm, args.tm)
                    sys.stdout.write("\r" + fmt_state(m, args.pm) + " " * 20)
                    sys.stdout.flush()
            time.sleep(0.01)
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        except Exception:
            pass
        send_cmd_repeat(s, mid, CMD_DISABLE, 5, 0.05)
        print("\n已退出并失能电机。")


def main():
    import argparse
    ap = argparse.ArgumentParser(description="达妙电机调试工具")
    ap.add_argument("cmd", nargs="?",
                    help="monitor / hold / setzero / enable / disable / clear")
    ap.add_argument("motor_id", type=int, nargs="?",
                    help="电机 ID（monitor 可省略=监视全部）")
    ap.add_argument("-i", "--iface", default="can0")
    ap.add_argument("--pm", type=float, default=3.14, help="位置量程 rad (默认3.14, DM单圈±π)")
    ap.add_argument("--vm", type=float, default=30.0, help="速度量程 rad/s (默认30)")
    ap.add_argument("--tm", type=float, default=10.0, help="力矩量程 N·m (默认10)")
    args = ap.parse_args()

    if not args.cmd:
        ap.print_help()
        sys.exit(0)

    if args.cmd == "monitor":
        cmd_monitor(args)
    elif args.cmd == "hold":
        cmd_hold(args)
    elif args.cmd == "setzero":
        cmd_single(args, "setzero", CMD_SET_ZERO,
                   "已将当前输出轴位置保存为零点（多发）。建议先 disable 手转到机械零点再执行。",
                   count=8)
    elif args.cmd == "enable":
        cmd_single(args, "enable", CMD_ENABLE, "电机应变绿(使能)。",
                   pre_clear=True, count=10)
    elif args.cmd == "disable":
        cmd_single(args, "disable", CMD_DISABLE, "电机已失能，可自由转动（标零前建议先失能）。",
                   count=5)
    elif args.cmd == "clear":
        cmd_single(args, "clear", CMD_CLEAR, "已发送清除错误，若电机闪烁红灯请再 enable。",
                   count=5)
    else:
        ap.print_help()
        sys.exit(1)


if __name__ == "__main__":
    main()
