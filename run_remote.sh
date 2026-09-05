#!/bin/bash
# ============================================================
# run_remote.sh —— 一键启动哨兵遥控节点（键盘 / 串口）
# 说明：统一包 spr_remote_control 提供两个输入源，本脚本封装成短命令。
#
# 用法:
#   ./run_remote.sh                # 串口遥控（真机，默认 /dev/ttyTHS1）
#   ./run_remote.sh serial         # 同上
#   ./run_remote.sh serial /dev/ttyUSB0   # 串口遥控（指定串口）
#   ./run_remote.sh keyboard       # 键盘遥控（需在真实终端里按键，X 退出）
#
# 提示：键盘要单独开一个终端运行；串口模式会随 Ctrl+C 退出。
# ============================================================
set -e
cd "$(dirname "$0")"

MODE="${1:-serial}"
DEV="${2:-/dev/ttyTHS1}"

case "$MODE" in
  serial|keyboard) ;;
  *) echo "[run_remote] 未知模式 '$MODE'（可选 serial / keyboard）"; exit 1 ;;
esac

# 1. 先清理可能残留的遥控节点，避免新旧节点重复发布同一话题
pkill -f 'rc_serial_remote_cpp|keyboard_remote' 2>/dev/null || true

# 2. 加载 ROS 环境
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
else
  echo "[run_remote] 找不到 install/setup.bash，请先 colcon build"
  exit 1
fi

# 3. 启动（exec：Ctrl+C 直接终止节点）
if [ "$MODE" == "keyboard" ]; then
  echo "[run_remote] 键盘遥控 ...（H 帮助 / X 退出）"
  exec ros2 run spr_remote_control keyboard_remote
else
  echo "[run_remote] 串口遥控 ... device=$DEV"
  exec ros2 run spr_remote_control rc_serial_remote_cpp --ros-args \
    -p device:="$DEV" -p baudrate:=100000 -p parity:=even
fi
