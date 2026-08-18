#!/bin/bash
# ============================================================
# 打开哨兵机器人 CAN 口（USB 转 CAN，SocketCAN 兼容）
# 用法:
#   ./setting.sh            # 打开 can0 + can1 (默认 1000kbps)
#   ./setting.sh all        # 同上
#   ./setting.sh can0       # 只打开指定接口
#   BITRATE=500000 ./setting.sh can0   # 自定义波特率
# ============================================================

BITRATE="${BITRATE:-1000000}"

up_can() {
    local iface="$1"
    sudo ip link set "$iface" up type can bitrate "$BITRATE"
    echo "[OK] $iface up (bitrate=$BITRATE)"
}

if [ "$1" == "all" ] || [ -z "$1" ]; then
    up_can can0
    up_can can1
else
    up_can "$1"
fi
# 监听 CAN 口用 can-utils 里的 candump 工具
# sudo apt install can-utils
# candump can0 -n 10          # 只监听 10 帧后退出
# candump can0 -T 3000        # 3 秒超时退出
# candump can0 -L              # 带时间戳显示
# candump -e can0              # 十六进制 + ASCII 显示（含不可见字符）
# candump -td can0             # 带相对时间戳