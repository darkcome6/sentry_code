#!/bin/bash
# ============================================================
# run_sentry.sh —— 清理残留进程后启动哨兵机器人（一键切换硬件后端）
#
# 用法:
#   ./run_sentry.sh                        # 默认 mock（模拟硬件）
#   ./run_sentry.sh mock                   # 模拟硬件
#   ./run_sentry.sh real                   # 真机（自动拉起 CAN 口）
#   ./run_sentry.sh mujoco                 # MuJoCo 仿真（默认平地场景 flat）
#   ./run_sentry.sh mujoco rough           # MuJoCo + 崎岖地形
#   ./run_sentry.sh mujoco ramp            # MuJoCo + 坡道
#   ./run_sentry.sh mujoco jump            # MuJoCo + 飞坡
#
# 说明:
#   启动前自动执行 stop_sentry.sh 清理上一次残留的进程，
#   避免 controller_manager 服务被旧节点占用导致报错。
# ============================================================
set -e
cd "$(dirname "$0")"

HW="${1:-mock}"
SCENE="${2:-flat}"
case "$HW" in
  mock|real|mujoco) ;;
  *) echo "[run_sentry] 未知硬件类型 '$HW'（可选 mock / real / mujoco）"; exit 1 ;;
esac

# 1. 先清理上一次启动可能残留的进程（残留会阻塞本次启动）
./stop_sentry.sh -q

# 2. 真机模式先拉起 CAN 口
if [ "$HW" == "real" ]; then
  echo "[run_sentry] 拉起 CAN 口 (can0/can1) ..."
  ./setting.sh all || echo "[WARN] CAN 口配置失败，继续尝试启动"
fi

# 3. 加载环境
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
else
  echo "[ERROR] 找不到 install/setup.bash，请先 colcon build"
  exit 1
fi

# 4. 启动（exec：Ctrl+C 直接终止 launch 及其子进程）
echo "[run_sentry] 启动 hardware_type=$HW scene=$SCENE ..."
exec ros2 launch spr_ctrl_bring_up sentry_bringup.launch.py \
  "hardware_type:=$HW" "scene:=$SCENE"
