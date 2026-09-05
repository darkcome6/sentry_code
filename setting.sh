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

# 判断接口是否存在（系统是否注册了该 CAN 设备）
iface_exists() { ip link show "$1" >/dev/null 2>&1; }

# 判断接口是否处于 UP 状态
iface_is_up() { ip link show "$1" 2>/dev/null | grep -q "state UP"; }

# 判断控制器是否处于异常错误状态（此时无法正常收发）
iface_in_bad_state() {
  ip -details link show "$1" 2>/dev/null | grep -qE "can state (ERROR-PASSIVE|BUS-OFF)"
}

up_can() {
    local iface="$1"
    if ! iface_exists "$iface"; then
        echo "[FAIL] $iface 不存在（系统未注册该 CAN 接口）"
        return 1
    fi
    if iface_is_up "$iface" && ! iface_in_bad_state "$iface"; then
        echo "[OK] $iface 已处于 UP 状态 (bitrate=$BITRATE)"
        return 0
    fi
    # 未 up，或已 up 但控制器处于错误状态 → 先 down 复位再 up
    if iface_is_up "$iface"; then
        echo "[INFO] $iface 控制器处于错误状态，执行 down/up 复位 ..."
        sudo ip link set "$iface" down || true
    fi
    if ! sudo ip link set "$iface" up type can bitrate "$BITRATE" restart-ms 100; then
        echo "[FAIL] $iface 配置失败"
        return 1
    fi
    if iface_in_bad_state "$iface"; then
        echo "[WARN] $iface 仍处于错误状态：请检查 CAN 总线接线 / 对端上电 / 终端电阻"
        echo "[WARN] 验证命令: candump $iface -n 10"
        return 1
    fi
    echo "[OK] $iface up (bitrate=$BITRATE)"
}

FAIL=0
if [ "$1" == "all" ] || [ -z "$1" ]; then
    # 只配置系统真实存在的 CAN 接口（如 can1 未启用则明确跳过，不再假装成功）
    for i in can0 can1; do
        if iface_exists "$i"; then
            up_can "$i" || FAIL=1
        else
            echo "[SKIP] $i 不存在（设备树未启用该路 CAN），跳过"
        fi
    done
else
    up_can "$1" || FAIL=1
fi
exit $FAIL
