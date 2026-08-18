#include "spr_hw_interface/effort_mock_system.hpp"

#include <algorithm>
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
  cmd_positions_.assign(n, 0.0);
  cmd_velocities_.assign(n, 0.0);
  cmd_efforts_.assign(n, 0.0);
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

  RCLCPP_INFO(rclcpp::get_logger("EffortMockSystem"),
    "EffortMockSystem initialized with %ld joints, accel_per_effort=%.3f",
    n, accel_per_effort_);
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
    // 理想电流/力矩环：反馈力矩 = 命令力矩
    state_efforts_[i] = cmd_efforts_[i];
    // 简单惯性 + 阻尼模型
    state_velocities_[i] = state_velocities_[i] * damping_
      + accel_per_effort_ * cmd_efforts_[i] * dt;
    state_positions_[i] += state_velocities_[i] * dt;
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
