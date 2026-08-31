// 增强版 PID 控制器
//   - 微分低通滤波（解决高频噪声被 D 项放大，500Hz 下的经典坑）
//   - 抗积分饱和（back-calculation 回退 + 积分限幅）
//   - 输出限幅
// 纯计算，不依赖 rclcpp，便于单测/复用。
#pragma once

#include <limits>

namespace spr_control_algorithms
{

struct PidParams
{
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double output_min = -std::numeric_limits<double>::max();
  double output_max = std::numeric_limits<double>::max();
  double integral_min = 0.0;  // 积分项限幅（0 = 不限幅）
  double integral_max = 0.0;
  double derivative_cutoff_hz = 0.0;  // 微分低通截止频率（Hz），0 = 不滤波
  double anti_windup_gain = 1.0;      // 抗饱和回退增益（0 = 关闭回退，只靠积分限幅）
};

class PidController
{
public:
  PidController() = default;
  explicit PidController(const PidParams & params);

  void setParams(const PidParams & params);
  const PidParams & params() const {return params_;}

  // 误差→控制量；dt 采样周期(s)
  double update(double error, double dt);
  void reset();

  double integral() const {return integral_;}
  double filteredDerivative() const {return filtered_derivative_;}

private:
  PidParams params_;
  double integral_ = 0.0;
  double prev_error_ = 0.0;
  double filtered_derivative_ = 0.0;
  bool initialized_ = false;
};

}  // namespace spr_control_algorithms
