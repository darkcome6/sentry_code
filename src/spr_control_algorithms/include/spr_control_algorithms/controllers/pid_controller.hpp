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

// 每一步的分项快照：调试可见 P/I/D/前馈各项输出、目标/反馈/误差与饱和状态
struct PidTrace
{
  double kp = 0.0;               // 当前生效比例增益
  double ki = 0.0;               // 当前生效积分增益
  double kd = 0.0;               // 当前生效微分增益
  double target = 0.0;           // 目标值
  double feedback = 0.0;         // 反馈值
  double error = 0.0;            // error = target - feedback
  double p_term = 0.0;           // P 项输出 = kp * error
  double i_term = 0.0;           // I 项输出（积分累积，含 ki，抗饱和后）
  double d_term = 0.0;           // D 项输出（低通后微分 * kd）
  double feedforward_term = 0.0; // 前馈项（外部注入，如重力补偿）
  double output = 0.0;           // 总输出（含前馈，限幅后）
  double integral = 0.0;         // 积分累积（抗饱和回退后）
  bool saturated = false;        // 是否输出饱和
};

class PidController
{
public:
  PidController() = default;
  explicit PidController(const PidParams & params);

  void setParams(const PidParams & params);
  const PidParams & params() const {return params_;}

  // 目标/反馈/前馈 → 控制量；dt 采样周期(s)。前馈用于重力补偿等模型补偿。
  double update(double target, double feedback, double feedforward, double dt);
  void reset();

  double integral() const {return integral_;}
  double filteredDerivative() const {return filtered_derivative_;}
  const PidTrace & trace() const {return trace_;}

private:
  PidParams params_;
  double integral_ = 0.0;
  double prev_error_ = 0.0;
  double filtered_derivative_ = 0.0;
  bool initialized_ = false;
  PidTrace trace_;
};

}  // namespace spr_control_algorithms
