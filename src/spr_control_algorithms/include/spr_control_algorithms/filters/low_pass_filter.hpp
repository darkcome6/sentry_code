// 一阶低通滤波器（IIR）：y[n] = α·y[n-1] + (1-α)·x[n]
//   常用于信号平滑 / 微分噪声抑制（配合 PID 的 derivative_cutoff 等效）。
// 纯计算，不依赖 rclcpp。
#pragma once

namespace spr_control_algorithms
{

class LowPassFilter
{
public:
  LowPassFilter() = default;

  // 按截止频率配置：hz 截止频率(Hz)，dt 采样周期(s)。无效参数→不过滤（透传）
  void setCutoffFrequency(double hz, double dt);
  // 直接设置平滑系数 α（0~1），α 越大越平滑（滞后越大）
  void setAlpha(double alpha);

  double update(double input);
  void reset(double value = 0.0);

  double filtered() const {return filtered_;}

private:
  double alpha_ = 0.0;
  double filtered_ = 0.0;
  bool initialized_ = false;
};

}  // namespace spr_control_algorithms
