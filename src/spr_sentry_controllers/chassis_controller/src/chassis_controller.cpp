#include "chassis_controller/chassis_controller.hpp"
#include <chassis_controller/chassis_controller_parameters.hpp>
#include <control_toolbox/pid_ros.hpp>

#include <algorithm>
#include <array>

namespace spr_chassis_controller
{
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;

ChassisController::ChassisController() : controller_interface::ControllerInterface()
{
}

ChassisController::~ChassisController()
{
}

controller_interface::CallbackReturn ChassisController::on_init()
{
  // 创建参数监听器并读取参数
  param_listener_ = std::make_shared<spr_chassis_controller::ParamListener>(get_node());
  params_ = param_listener_->get_params();

  // TODO(实现): 初始化实时缓冲（默认零速度指令）
  auto cmd = std::make_shared<geometry_msgs::msg::Twist>();
  cmd->linear.x = 0.0;
  cmd->linear.y = 0.0;
  cmd->angular.z = 0.0;
  cmd_vel_rt_.initRT(cmd);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ChassisController::command_interface_configuration() const
{
  // 四轮速度环最终输出电流/力矩参考，通过 effort 命令接口交给硬件层
  std::vector<std::string> joint_names;
  joint_names.push_back(params_.left_wheel_front.joint + "/effort");
  joint_names.push_back(params_.right_wheel_front.joint + "/effort");
  joint_names.push_back(params_.left_wheel_back.joint + "/effort");
  joint_names.push_back(params_.right_wheel_back.joint + "/effort");
  return { interface_configuration_type::INDIVIDUAL, joint_names };
}

controller_interface::InterfaceConfiguration ChassisController::state_interface_configuration() const
{
  // 读四轮轮速反馈，供速度环闭环
  std::vector<std::string> joint_names;
  joint_names.push_back(params_.left_wheel_front.joint + "/velocity");
  joint_names.push_back(params_.right_wheel_front.joint + "/velocity");
  joint_names.push_back(params_.left_wheel_back.joint + "/velocity");
  joint_names.push_back(params_.right_wheel_back.joint + "/velocity");
  return { interface_configuration_type::INDIVIDUAL, joint_names };
}

controller_interface::CallbackReturn ChassisController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  params_ = param_listener_->get_params();

  // 初始化四个轮子的速度环 PID（命名空间: <轮子名>.pid）
  const std::array<std::string, 4> pid_names{
    "left_wheel_front.pid", "right_wheel_front.pid",
    "left_wheel_back.pid", "right_wheel_back.pid" };
  for (size_t i = 0; i < kWheelCount; i++)
  {
    wheel_pid_[i] = std::make_shared<control_toolbox::PidROS>(get_node(), pid_names[i], true);
    if (!wheel_pid_[i]->initPid())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to config the params of PID %s.",
                   pid_names[i].c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  // 底盘速度指令订阅（/cmd_vel），实时写入双缓存
  cmd_vel_sub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
      cmd_vel_rt_.writeFromNonRT(msg);
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ChassisController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  // 1) 更新参数
  update_parameters();

  // 2) 读底盘速度指令（cmd_vel）
  double vx = 0.0, vy = 0.0, wz = 0.0;
  auto cmd = cmd_vel_rt_.readFromRT();
  if (cmd && *cmd)
  {
    vx = (*cmd)->linear.x;
    vy = (*cmd)->linear.y;
    wz = (*cmd)->angular.z;
  }

  // 3) 麦克纳姆轮逆运动学：底盘速度 -> 四轮目标速度（TODO 实现）
  const auto target_vel = inverse_kinematics(vx, vy, wz);

  // 4) 速度环 PID：目标轮速 -> 电流，写 effort 命令接口
  // 左前轮
  wheel_fb_[0] = wheel_state_interfaces_[0]->get_value();
  wheel_cmd_[0] = compute_velocity_pid(wheel_pid_[0], target_vel[0], wheel_fb_[0], period,
    params_.left_wheel_front.pid.output_min, params_.left_wheel_front.pid.output_max);
  wheel_command_interfaces_[0]->set_value(wheel_cmd_[0]);
  // 右前轮
  wheel_fb_[1] = wheel_state_interfaces_[1]->get_value();
  wheel_cmd_[1] = compute_velocity_pid(wheel_pid_[1], target_vel[1], wheel_fb_[1], period,
    params_.right_wheel_front.pid.output_min, params_.right_wheel_front.pid.output_max);
  wheel_command_interfaces_[1]->set_value(wheel_cmd_[1]);
  // 左后轮
  wheel_fb_[2] = wheel_state_interfaces_[2]->get_value();
  wheel_cmd_[2] = compute_velocity_pid(wheel_pid_[2], target_vel[2], wheel_fb_[2], period,
    params_.left_wheel_back.pid.output_min, params_.left_wheel_back.pid.output_max);
  wheel_command_interfaces_[2]->set_value(wheel_cmd_[2]);
  // 右后轮
  wheel_fb_[3] = wheel_state_interfaces_[3]->get_value();
  wheel_cmd_[3] = compute_velocity_pid(wheel_pid_[3], target_vel[3], wheel_fb_[3], period,
    params_.right_wheel_back.pid.output_min, params_.right_wheel_back.pid.output_max);
  wheel_command_interfaces_[3]->set_value(wheel_cmd_[3]);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn ChassisController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  // 借出状态接口（轮速反馈）
  for (size_t i = 0; i < kWheelCount; i++)
  {
    wheel_state_interfaces_[i] =
      std::make_unique<const hardware_interface::LoanedStateInterface>(
        std::move(state_interfaces_[i]));
  }
  // 借出命令接口（effort 电流）
  for (size_t i = 0; i < kWheelCount; i++)
  {
    wheel_command_interfaces_[i] = std::make_unique<hardware_interface::LoanedCommandInterface>(
      std::move(command_interfaces_[i]));
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  for (auto & itf : wheel_state_interfaces_) { itf.reset(); }
  for (auto & itf : wheel_command_interfaces_) { itf.reset(); }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisController::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ChassisController::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return controller_interface::CallbackReturn::SUCCESS;
}

void ChassisController::update_parameters()
{
  if (!param_listener_->is_old(params_)) { return; }
  params_ = param_listener_->get_params();
}

// 麦克纳姆轮逆运动学：底盘速度(vx,vy,wz) -> 四轮目标轮速(rad/s)
// 约定：x 向前、y 向左、wz 逆时针为正（对应 cmd_vel 的 linear.x/linear.y/angular.z）
//   v_LF = (vx - vy - (L+W)/2 * wz) / R
//   v_RF = (vx + vy + (L+W)/2 * wz) / R
//   v_LB = (vx - vy + (L+W)/2 * wz) / R
//   v_RB = (vx + vy - (L+W)/2 * wz) / R
std::array<double, 4> ChassisController::inverse_kinematics(double vx, double vy, double wz)
{
  const double L = params_.wheel_base;     // 轴距 (m)
  const double W = params_.track_width;    // 轮距 (m)
  const double R = params_.wheel_radius;   // 轮半径 (m)
  if (R <= 0.0)
  {
    RCLCPP_WARN(get_node()->get_logger(), "wheel_radius must be > 0, got %.4f", R);
    return { 0.0, 0.0, 0.0, 0.0 };
  }
  const double half = (L + W) / 2.0;
  return {
    ( vx - vy - half * wz) / R,   // 左前轮
    ( vx + vy + half * wz) / R,   // 右前轮
    ( vx - vy + half * wz) / R,   // 左后轮
    ( vx + vy - half * wz) / R,   // 右后轮
  };
}

// 速度环 PID：轮速误差经 PID 得到电流(原始值)，限幅后返回
double ChassisController::compute_velocity_pid(
  const std::shared_ptr<control_toolbox::PidROS> & pid,
  double ref, double fb, const rclcpp::Duration & period,
  double out_min, double out_max)
{
  const double err = ref - fb;
  const double out = pid->computeCommand(err, period);
  return std::clamp(out, out_min, out_max);
}
}  // namespace spr_chassis_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(spr_chassis_controller::ChassisController,
                       controller_interface::ControllerInterface)
