#!/bin/bash
# ============================================================
# record_bag.sh —— 一键录制关键话题到 ros2 bag，供离线分析
#
# 用法:
#   ./record_bag.sh                 # 保存到 ./bags/sentry_<时间戳>
#   ./record_bag.sh <输出目录>       # 指定保存目录（默认 ./bags）
#
# 说明:
#   自动从当前运行的仿真/真机话题里挑出关心的主题录制，
#   不用手写话题名（避免记错/版本变化）。
#   录完按 Ctrl+C 停止；bag 可直接用 plotjuggler 打开分析。
# ============================================================
set -e
cd "$(dirname "$0")"

OUT_DIR="${1:-bags}"
mkdir -p "$OUT_DIR"

# 过滤出关心的话题：
#   /joint_states              关节状态（位置/速度/力矩，joint_state_broadcaster）
#   /gimbal_controller/...     云台指令/状态 + PID 诊断（含 p_error）
#   /cmd_vel                   底盘速度指令
#   /clock                     仿真时钟（回放必需）
readarray -t TOPICS < <(
  ros2 topic list 2>/dev/null \
    | grep -E '^/(joint_states|cmd_vel|clock|gimbal_controller/|chassis_controller/)' \
    | grep -v '/parameter' \
    | sort -u
)

if [ ${#TOPICS[@]} -eq 0 ]; then
  echo "[record] 没找到要录制的话题，请先启动仿真（./run_sentry.sh mujoco）"
  exit 1
fi

OUT="$OUT_DIR/sentry_$(date +%Y%m%d_%H%M%S)"
echo "[record] 输出: $OUT"
echo "[record] 录制话题:"
printf '  %s\n' "${TOPICS[@]}"
echo "[record] 按 Ctrl+C 停止 ..."
ros2 bag record -o "$OUT" "${TOPICS[@]}"
