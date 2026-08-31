#include "spr_control_algorithms/controllers/gravity_compensation.hpp"

#include <cmath>

namespace spr_control_algorithms
{

GravityCompensation::GravityCompensation(const GravityCompensationParams & params)
: params_(params) {}

void GravityCompensation::setParams(const GravityCompensationParams & params)
{
  params_ = params;
}

double GravityCompensation::computeTorque(double angle) const
{
  if (!params_.enabled) {
    return 0.0;
  }
  return params_.mass * params_.gravity * params_.arm_length *
         std::cos(angle + params_.angle_offset);
}

}  // namespace spr_control_algorithms
