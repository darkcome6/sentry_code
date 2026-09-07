#include "spr_control_algorithms/controllers/pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace spr_control_algorithms
{

PidController::PidController(const PidParams & params) {setParams(params);}

void PidController::setParams(const PidParams & params) {params_ = params;}

double PidController::update(
  double target, double feedback, double feedforward, double dt)
{
  if (dt <= 0.0) {
    return 0.0;
  }

  const double error = target - feedback;

  // --- P 项 ---
  const double p_out = params_.kp * error;

  // --- I 项（积分 + 积分限幅）---
  integral_ += params_.ki * error * dt;
  if (params_.integral_min < params_.integral_max) {
    integral_ = std::clamp(integral_, params_.integral_min, params_.integral_max);
  }

  // --- D 项（微分 + 一阶低通滤波，抑制高频噪声放大）---
  const double derivative = initialized_ ? (error - prev_error_) / dt : 0.0;
  prev_error_ = error;
  initialized_ = true;
  if (params_.derivative_cutoff_hz > 0.0) {
    const double tau = 1.0 / (2.0 * M_PI * params_.derivative_cutoff_hz);
    const double alpha = tau / (tau + dt);
    filtered_derivative_ = alpha * filtered_derivative_ + (1.0 - alpha) * derivative;
  } else {
    filtered_derivative_ = derivative;
  }
  const double d_out = params_.kd * filtered_derivative_;

  const double out_min = params_.output_min;
  const double out_max = params_.output_max;

  // --- 抗积分饱和(back-calculation)：基于 PID 部分输出限幅回退积分 ---
  double pid_raw = p_out + integral_ + d_out;
  double pid_lim = std::clamp(pid_raw, out_min, out_max);
  bool pid_saturated = (pid_lim != pid_raw);
  if (params_.anti_windup_gain > 0.0 && pid_saturated) {
    // 输出饱和时把超出部分从积分里回退，防止 windup
    integral_ += (pid_lim - pid_raw) * params_.anti_windup_gain;
    if (params_.integral_min < params_.integral_max) {
      integral_ = std::clamp(integral_, params_.integral_min, params_.integral_max);
    }
    pid_raw = p_out + integral_ + d_out;
    pid_lim = std::clamp(pid_raw, out_min, out_max);
    pid_saturated = (pid_lim != pid_raw);
  }

  // --- 总输出 = PID(限幅后) + 前馈，整体再限幅保护 ---
  const double total = pid_lim + feedforward;
  const double out = std::clamp(total, out_min, out_max);

  // --- 诊断快照：增益 + 分项 + 目标/反馈/误差 + 饱和 ---
  trace_.kp = params_.kp;
  trace_.ki = params_.ki;
  trace_.kd = params_.kd;
  trace_.target = target;
  trace_.feedback = feedback;
  trace_.error = error;
  trace_.p_term = p_out;
  trace_.i_term = integral_;
  trace_.d_term = d_out;
  trace_.feedforward_term = feedforward;
  trace_.output = out;
  trace_.integral = integral_;
  trace_.saturated = pid_saturated || (out != total);
  return out;
}

void PidController::reset()
{
  integral_ = 0.0;
  prev_error_ = 0.0;
  filtered_derivative_ = 0.0;
  initialized_ = false;
  trace_ = PidTrace{};
}

}  // namespace spr_control_algorithms
