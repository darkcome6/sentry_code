#ifndef CHASSIS_CONTROLLER_HPP_
#define CHASSIS_CONTROLLER_HPP_

#include <chassis_controller/chassis_controller_parameters.hpp>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <geometry_msgs/msg/twist.hpp>

#include <control_toolbox/pid_ros.hpp>
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace spr_chassis_controller
{
/// 麦克纳姆轮速控底盘控制器（框架）：
/// 订阅底盘速度指令(cmd_vel) -> 麦克纳姆轮逆运动学 -> 四轮目标速度
/// -> 速度环PID -> 电流/力矩(effort)命令接口 -> 硬件层
class ChassisController : public controller_interface::ControllerInterface
{
public:
  ChassisController();
  ~ChassisController() override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

private:
  // 更新参数
  void update_parameters();
  // 麦克纳姆轮逆运动学：底盘速度(vx,vy,wz) -> 四轮目标速度
  // 返回顺序: {左前, 右前, 左后, 右后}
  std::array<double, 4> inverse_kinematics(double vx, double vy, double wz);
  // 麦克纳姆轮正运动学：四轮速度 -> 车体速度 {vx, vy, wz}
  std::array<double, 4> forward_kinematics(const std::array<double, 4> & wheel_vel);
  // 速度环 PID：轮速误差 -> 电流，限幅后返回
  double compute_velocity_pid(const std::shared_ptr<control_toolbox::PidROS> & pid,
                              double ref, double fb, const rclcpp::Duration & period,
                              double out_min, double out_max);
  // 由轮速反馈更新里程计位姿并发布 odom->base_footprint TF
  void update_odometry(const rclcpp::Time & time, const rclcpp::Duration & period);

  // 参数
  std::shared_ptr<spr_chassis_controller::ParamListener> param_listener_;
  spr_chassis_controller::Params params_;

  // 轮子状态接口（读轮速反馈）与命令接口（写电流）
  static constexpr size_t kWheelCount = 4;  // 左前 右前 左后 右后
  std::array<std::unique_ptr<const hardware_interface::LoanedStateInterface>, kWheelCount>
    wheel_state_interfaces_{ nullptr, nullptr, nullptr, nullptr };
  std::array<std::unique_ptr<hardware_interface::LoanedCommandInterface>, kWheelCount>
    wheel_command_interfaces_{ nullptr, nullptr, nullptr, nullptr };

  // 底盘速度指令订阅（geometry_msgs/Twist），实时缓冲
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_{ nullptr };
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::Twist>> cmd_vel_rt_{ nullptr };

  // 四轮速度环 PID
  std::array<std::shared_ptr<control_toolbox::PidROS>, kWheelCount> wheel_pid_{ nullptr, nullptr, nullptr, nullptr };

  // 四轮命令（电流）与反馈（轮速）
  std::array<double, kWheelCount> wheel_cmd_{ 0.0, 0.0, 0.0, 0.0 };
  std::array<double, kWheelCount> wheel_fb_{ 0.0, 0.0, 0.0, 0.0 };

  // 里程计：位姿积分 + odom->base_footprint TF 发布（让车体在 RViz 中真实移动）
  double odom_x_{ 0.0 };
  double odom_y_{ 0.0 };
  double odom_yaw_{ 0.0 };
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_{ nullptr };
};
}  // namespace spr_chassis_controller
#endif  // CHASSIS_CONTROLLER_HPP_
