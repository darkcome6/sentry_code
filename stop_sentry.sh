#!/bin/bash
# ============================================================
# stop_sentry.sh —— 清理哨兵机器人所有残留 ROS 进程
#
# 为什么需要:
#   重复启动时，上一次 launch 若未正常退出，其 ros2_control_node
#   会一直占用 /controller_manager 服务；新启动的 spawner 会连到
#   旧节点，导致 "Controller already loaded / no controller with
#   this name exists" 之类奇怪报错。启动前先清理即可避免。
#
# 用法:
#   ./stop_sentry.sh         # 清理并显示被杀的进程
#   ./stop_sentry.sh -q      # 安静模式（供 run_sentry.sh 内部调用）
#
# 退出码:
#   0 环境干净；1 仍有 ros2_control_node 残留
# ============================================================

QUIET=0
[ "${1:-}" == "-q" ] && QUIET=1

TOTAL=0

# 按命令行特征收集并杀死进程（自动排除脚本自身和当前终端）
kill_by_pattern() {
  local pat="$1"
  local pids
  # 排除当前脚本 PID($$) 和调用者 PID($PPID)
  pids=$(pgrep -f "$pat" 2>/dev/null | grep -v -e "$$" -e "$PPID" || true)
  [ -z "$pids" ] && return
  for pid in $pids; do
    # 跳过 stop_sentry / run_sentry 自身
    if grep -qE "stop_sentry|run_sentry" "/proc/$pid/cmdline" 2>/dev/null; then
      continue
    fi
    local cmd
    cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null | cut -c1-90)
    if [ "$QUIET" -eq 0 ]; then
      printf '  kill pid=%-6s %s\n' "$pid" "$cmd"
    fi
    if kill -9 "$pid" 2>/dev/null; then
      TOTAL=$((TOTAL + 1))
    fi
  done
}

echo "[stop_sentry] 清理残留进程 ..."
kill_by_pattern "ros2_control_node"          # controller_manager 主节点（最关键）
kill_by_pattern "robot_state_publisher"     # TF / robot_description
kill_by_pattern "sentry_bringup"            # 本项目 launch 进程
kill_by_pattern "controller_manager/spawner" # 控制器 spawner（精确匹配路径，避免误杀）
kill_by_pattern "teleop_sentry"             # 遥控节点

sleep 0.5

if [ "$QUIET" -eq 0 ]; then
  echo "----------------------------------------"
  echo "[stop_sentry] 共清理 $TOTAL 个进程"
fi

# 验证：controller_manager 是否彻底清掉
if pgrep -f "ros2_control_node" >/dev/null 2>&1; then
  echo "[WARN] 仍有 ros2_control_node 残留，请检查:"
  pgrep -af "ros2_control_node"
  exit 1
else
  [ "$QUIET" -eq 0 ] && echo "[OK] 环境干净，可以启动。"
  exit 0
fi

# 若还需清理其它（可选，取消注释）:
# kill_by_pattern "rviz2"
