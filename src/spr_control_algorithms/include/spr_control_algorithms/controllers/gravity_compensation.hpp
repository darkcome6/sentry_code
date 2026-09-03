// 重力补偿前馈
//   典型云台 pitch：τ_ff = m·g·L·cos(θ + offset)
//   角度 0（水平）时力臂最大 → 补偿力矩最大；转到竖直方向 → 趋近 0。
// 纯计算，不依赖 rclcpp。
#pragma once

namespace spr_control_algorithms
{

struct GravityCompensationParams
{
  double mass = 0.0;          // 云台质量（kg）
  double arm_length = 0.0;    // 质心到转轴力臂（m）
  double gravity = 9.81;      // 重力加速度（m/s^2）
  double angle_offset = 0.0;  // 角度零点偏移（rad）
  bool enabled = true;        // 是否启用
};

class GravityCompensation
{
public:
  GravityCompensation() = default;
  explicit GravityCompensation(const GravityCompensationParams & params);

  void setParams(const GravityCompensationParams & params);
  const GravityCompensationParams & params() const {return params_;}

  // 输入当前角度（rad），返回重力补偿前馈力矩
  double computeTorque(double angle) const;

private:
  GravityCompensationParams params_;
};

}  // namespace spr_control_algorithms
