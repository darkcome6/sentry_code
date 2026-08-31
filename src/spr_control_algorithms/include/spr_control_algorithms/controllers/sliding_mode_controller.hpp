// 一阶滑模控制器（实验舱）
//   滑模面：s = ė + λ·e
//   控制量：u = u_ff - k·sat(s/φ)   边界层 sat 代替 sign，防抖振
// 纯计算，不依赖 rclcpp。
#pragma once

#include <limits>

namespace spr_control_algorithms
{

struct SlidingModeParams
{
  double lambda = 1.0;    // 滑模面斜率
  double gain = 1.0;      // 切换增益 k
  double boundary = 0.1;  // 边界层厚度 φ（>0，防抖振）
  double output_min = -std::numeric_limits<double>::max();
  double output_max = std::numeric_limits<double>::max();
};

class SlidingModeController
{
public:
  SlidingModeController() = default;
  explicit SlidingModeController(const SlidingModeParams & params);

  void setParams(const SlidingModeParams & params);
  const SlidingModeParams & params() const {return params_;}

  // error: 位置误差；error_dot: 误差导数；feedforward: 前馈（等效控制/补偿）
  double update(double error, double error_dot, double feedforward = 0.0);
  void reset();

  double surface() const {return surface_;}

private:
  SlidingModeParams params_;
  double surface_ = 0.0;
};

}  // namespace spr_control_algorithms
