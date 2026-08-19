#include "spr_hw_interface/effort_mock_system.hpp"

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

namespace spr_hw_interface
{
hardware_interface::CallbackReturn EffortMockSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t n = info_.joints.size();
  // 命令接口初始置 NaN（未写入标记），由 read() 按实际写入的接口选择驱动方式
  cmd_positions_.assign(n, std::numeric_limits<double>::quiet_NaN());
  cmd_velocities_.assign(n, std::numeric_limits<double>::quiet_NaN());
  cmd_efforts_.assign(n, std::numeric_limits<double>::quiet_NaN());
  state_positions_.assign(n, 0.0);
  state_velocities_.assign(n, 0.0);
  state_efforts_.assign(n, 0.0);

  auto it = info_.hardware_parameters.find("accel_per_effort");
  if (it != info_.hardware_parameters.end())
  {
    try
    {
      accel_per_effort_ = std::stod(it->second);
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(rclcpp::get_logger("EffortMockSystem"),
        "Failed to parse accel_per_effort '%s', using default %.3f",
        it->second.c_str(), accel_per_effort_);
    }
  }
  it = info_.hardware_parameters.find("max_velocity");
  if (it != info_.hardware_parameters.end())
  {
    try
    {
      max_velocity_ = std::stod(it->second);
    }
    catch (const std::exception & e)
    {
      RCLCPP_WARN(rclcpp::get_logger("EffortMockSystem"),
        "Failed to parse max_velocity '%s', using default %.3f",
        it->second.c_str(), max_velocity_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("EffortMockSystem"),
    "EffortMockSystem initialized with %ld joints, accel_per_effort=%.3f",
    n, accel_per_effort_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EffortMockSystem::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // 命令接口重置为 NaN（未写入），状态清零
  for (auto & v : cmd_positions_) { v = std::numeric_limits<double>::quiet_NaN(); }
  for (auto & v : cmd_velocities_) { v = std::numeric_limits<double>::quiet_NaN(); }
  for (auto & v : cmd_efforts_) { v = std::numeric_limits<double>::quiet_NaN(); }
  std::fill(state_positions_.begin(), state_positions_.end(), 0.0);
  std::fill(state_velocities_.begin(), state_velocities_.end(), 0.0);
  std::fill(state_efforts_.begin(), state_efforts_.end(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EffortMockSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    for (const auto & state_interface : info_.joints[i].state_interfaces)
    {
      if (state_interface.name == "position")
      {
        interfaces.emplace_back(info_.joints[i].name, state_interface.name, &state_positions_[i]);
      }
      else if (state_interface.name == "velocity")
      {
        interfaces.emplace_back(info_.joints[i].name, state_interface.name, &state_velocities_[i]);
      }
      else if (state_interface.name == "effort")
      {
        interfaces.emplace_back(info_.joints[i].name, state_interface.name, &state_efforts_[i]);
      }
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> EffortMockSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    for (const auto & command_interface : info_.joints[i].command_interfaces)
    {
      if (command_interface.name == "position")
      {
        interfaces.emplace_back(
          info_.joints[i].name, command_interface.name, &cmd_positions_[i]);
      }
      else if (command_interface.name == "velocity")
      {
        interfaces.emplace_back(
          info_.joints[i].name, command_interface.name, &cmd_velocities_[i]);
      }
      else if (command_interface.name == "effort")
      {
        interfaces.emplace_back(
          info_.joints[i].name, command_interface.name, &cmd_efforts_[i]);
      }
    }
  }
  return interfaces;
}

hardware_interface::return_type EffortMockSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  const double dt = period.seconds();
  if (dt <= 0.0)
  {
    return hardware_interface::return_type::OK;
  }
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // 按实际被写入的命令接口选择驱动方式（优先级 position > velocity > effort）
    if (!std::isnan(cmd_positions_[i]))
    {
      // 位置模式：一阶低通跟随命令，模拟位置环动态
      const double prev = state_positions_[i];
      state_positions_[i] += (cmd_positions_[i] - state_positions_[i]) * position_gain_;
      state_velocities_[i] = (state_positions_[i] - prev) / dt;
      state_efforts_[i] = 0.0;
    }
    else if (!std::isnan(cmd_velocities_[i]))
    {
      // 速度模式：速度直接跟随命令，位置积分
      state_velocities_[i] = cmd_velocities_[i];
      state_positions_[i] += state_velocities_[i] * dt;
      state_efforts_[i] = 0.0;
    }
    else if (!std::isnan(cmd_efforts_[i]))
    {
      // 力矩模式：effort 驱动（理想电流/力矩环 + 简单惯性与阻尼）
      state_efforts_[i] = cmd_efforts_[i];
      state_velocities_[i] = state_velocities_[i] * damping_
        + accel_per_effort_ * cmd_efforts_[i] * dt;
      // 粘滞摩擦：速度越大阻力越大，增强稳定性
      state_velocities_[i] *= (1.0 - friction_ * dt);
      // 轮速物理上限：防止闭环参数不当导致速度发散
      state_velocities_[i] = std::clamp(state_velocities_[i], -max_velocity_, max_velocity_);
      // 静态摩擦锁：指令力很小且速度很小时直接锁死（静止不滑/不漂移）
      if (std::abs(cmd_efforts_[i]) < effort_deadzone_ &&
          std::abs(state_velocities_[i]) < velocity_deadzone_)
      {
        state_velocities_[i] = 0.0;
      }
      // 速度死区：低速视为停，避免静止时残留速度被里程计放大晃动
      if (std::abs(state_velocities_[i]) < velocity_deadzone_)
      {
        state_velocities_[i] = 0.0;
      }
      state_positions_[i] += state_velocities_[i] * dt;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EffortMockSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  return hardware_interface::return_type::OK;
}
}  // namespace spr_hw_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  spr_hw_interface::EffortMockSystem, hardware_interface::SystemInterface)
