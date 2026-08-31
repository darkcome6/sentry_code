#include "spr_control_algorithms/controllers/sliding_mode_controller.hpp"

#include <algorithm>

namespace spr_control_algorithms
{

SlidingModeController::SlidingModeController(const SlidingModeParams & params)
: params_(params) {}

void SlidingModeController::setParams(const SlidingModeParams & params)
{
  params_ = params;
}

double SlidingModeController::update(double error, double error_dot, double feedforward)
{
  // 滑模面
  surface_ = error_dot + params_.lambda * error;

  // 边界层饱和函数 sat(s/φ)
  double sat = surface_ / params_.boundary;
  sat = std::clamp(sat, -1.0, 1.0);

  const double u = feedforward - params_.gain * sat;
  return std::clamp(u, params_.output_min, params_.output_max);
}

void SlidingModeController::reset() {surface_ = 0.0;}

}  // namespace spr_control_algorithms
