#include "spr_control_algorithms/filters/low_pass_filter.hpp"

#include <algorithm>
#include <cmath>

namespace spr_control_algorithms
{

void LowPassFilter::setCutoffFrequency(double hz, double dt)
{
  if (hz <= 0.0 || dt <= 0.0) {
    alpha_ = 0.0;  // 不过滤（透传）
    return;
  }
  const double tau = 1.0 / (2.0 * M_PI * hz);
  alpha_ = tau / (tau + dt);
}

void LowPassFilter::setAlpha(double alpha) {alpha_ = std::clamp(alpha, 0.0, 1.0);}

double LowPassFilter::update(double input)
{
  if (!initialized_) {
    filtered_ = input;
    initialized_ = true;
    return filtered_;
  }
  filtered_ = alpha_ * filtered_ + (1.0 - alpha_) * input;
  return filtered_;
}

void LowPassFilter::reset(double value)
{
  filtered_ = value;
  initialized_ = true;
}

}  // namespace spr_control_algorithms
