#ifndef SPR_HW_INTERFACE_EFFORT_MOCK_SYSTEM_HPP_
#define SPR_HW_INTERFACE_EFFORT_MOCK_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace spr_hw_interface
{
/// 轻量模拟硬件：按 URDF 声明导出命令接口(position/velocity/effort)与
/// 状态接口(position/velocity/effort)，用于无真机(无CAN)时调试
/// "位置环 -> effort 电流/力矩" 链路。
/// 模拟模型(每关节独立，仅 effort 驱动)：
///   state_effort  = cmd_effort
///   velocity     += accel_per_effort * cmd_effort * dt ;  velocity *= damping
///   position     += velocity * dt
class EffortMockSystem : public hardware_interface::SystemInterface
{
public:
  EffortMockSystem() = default;
  ~EffortMockSystem() override = default;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 命令接口数组（按声明导出）
  std::vector<double> cmd_positions_;
  std::vector<double> cmd_velocities_;
  std::vector<double> cmd_efforts_;
  // 状态接口数组
  std::vector<double> state_positions_;
  std::vector<double> state_velocities_;
  std::vector<double> state_efforts_;

  double accel_per_effort_{ 0.5 };   // 单位 effort 产生的角加速度 (rad/s^2)
  double damping_{ 0.99 };           // 每周期速度阻尼
};
}  // namespace spr_hw_interface
#endif  // SPR_HW_INTERFACE_EFFORT_MOCK_SYSTEM_HPP_
