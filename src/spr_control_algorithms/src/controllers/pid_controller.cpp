#include "spr_control_algorithms/controllers/pid_controller.hpp"

#include <algorithm>
#include <cmath>

namespace spr_control_algorithms
{

PidController::PidController(const PidParams & params) {setParams(params);}

void PidController::setParams(const PidParams & params) {params_ = params;}

double PidController::update(double error, double dt)
{
  if (dt <= 0.0) {
    return 0.0;
  }

  // --- P ---
  const double p_out = params_.kp * error;

  // --- I（积分 + 积分限幅）---
  integral_ += params_.ki * error * dt;
  if (params_.integral_min < params_.integral_max) {
    integral_ = std::clamp(integral_, params_.integral_min, params_.integral_max);
  }

  // --- D（微分 + 一阶低通滤波，抑制高频噪声放大）---
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

  // --- 输出（抗积分饱和 back-calculation + 输出限幅）---
  double out = p_out + integral_ + d_out;
  const double saturated = std::clamp(out, params_.output_min, params_.output_max);
  if (params_.anti_windup_gain > 0.0 && saturated != out) {
    // 输出饱和时把超出部分从积分里回退，防止 windup
    integral_ += (saturated - out) * params_.anti_windup_gain;
    if (params_.integral_min < params_.integral_max) {
      integral_ = std::clamp(integral_, params_.integral_min, params_.integral_max);
    }
    out = saturated;
  }
  out = std::clamp(out, params_.output_min, params_.output_max);
  return out;
}

void PidController::reset()
{
  integral_ = 0.0;
  prev_error_ = 0.0;
  filtered_derivative_ = 0.0;
  initialized_ = false;
}

}  // namespace spr_control_algorithms
