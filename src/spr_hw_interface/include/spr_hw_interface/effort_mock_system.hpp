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
/// 状态接口(position/velocity/effort)，用于无真机(无CAN)时调试控制链路。
/// read() 按"实际被写入的命令接口"选择驱动方式（优先级 position > velocity > effort）：
///   - position 命令有效：位置一阶低通跟随命令（模拟位置环动态）
///   - velocity 命令有效：速度直接跟随命令，位置积分
///   - effort   命令有效：力矩驱动（velocity += accel*effort*dt, position += vel*dt）
/// 未写入的命令接口保持 NaN，不参与驱动。
class EffortMockSystem : public hardware_interface::SystemInterface
{
public:
  EffortMockSystem() = default;
  ~EffortMockSystem() override = default;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 命令接口数组（按声明导出；未写入时为 NaN）
  std::vector<double> cmd_positions_;
  std::vector<double> cmd_velocities_;
  std::vector<double> cmd_efforts_;
  // 状态接口数组
  std::vector<double> state_positions_;
  std::vector<double> state_velocities_;
  std::vector<double> state_efforts_;

  double accel_per_effort_{ 0.5 };   // 单位 effort 产生的角加速度 (rad/s^2)
  double damping_{ 0.99 };           // 每周期速度阻尼（effort 模式）
  double position_gain_{ 0.2 };      // 每周期向目标位置的接近比例（position 模式 0~1）
};
}  // namespace spr_hw_interface
#endif  // SPR_HW_INTERFACE_EFFORT_MOCK_SYSTEM_HPP_
