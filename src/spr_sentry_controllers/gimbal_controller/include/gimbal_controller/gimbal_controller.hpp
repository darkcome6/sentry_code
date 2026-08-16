#ifndef GIMBAL_CONTROLLER_HPP_
#define GIMBAL_CONTROLLER_HPP_
#include <gimbal_controller/gimbal_controller_parameters.hpp>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <spr_msgs/msg/gimbal_cmd.hpp>
#include <spr_msgs/msg/gimbal_state.hpp>

#include <control_toolbox/pid_ros.hpp>
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_publisher.hpp"

namespace spr_gimbal_controller
{
class SprGimbalController : public controller_interface::ControllerInterface
{
public:
  SprGimbalController();
  ~SprGimbalController();
  controller_interface::CallbackReturn on_init() override;

  controller_interface::return_type update(const rclcpp::Time& time,const rclcpp::Duration& period) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn
  on_error(const rclcpp_lifecycle::State& previous_state) override;
private:
  // State interfaces
  std::unique_ptr<const hardware_interface::LoanedStateInterface> pitch_state_interface_{ nullptr };
  std::unique_ptr<const hardware_interface::LoanedStateInterface> big_yaw_state_interface_{ nullptr };
  std::unique_ptr<const hardware_interface::LoanedStateInterface> small_yaw_state_interface_{ nullptr };
  // Command interfaces
  std::unique_ptr<hardware_interface::LoanedCommandInterface> pitch_command_interface_{ nullptr };
  std::unique_ptr<hardware_interface::LoanedCommandInterface> big_yaw_command_interface_{ nullptr };
  std::unique_ptr<hardware_interface::LoanedCommandInterface> small_yaw_command_interface_{ nullptr };
  //控制器YAML文件编译文件
  std::shared_ptr<spr_gimbal_controller::ParamListener> param_listener_;
  spr_gimbal_controller::Params params_;
  //更新参数
  void update_parameters();
  //控制器模式
  uint8_t mode_{ 0 };//0 保持不动 1 扫描模式 2自瞄模式 3遥控模式
  uint8_t last_mode_{ 0 };
  //
  std::array<double, 3> scan_mode();
  std::array<double, 3> aim_mode();
  std::array<double, 3> remote_control();
  //临时存储
  double pitch_pos_cmd_{ 0.0 }, small_yaw_pos_cmd_{ 0.0 },big_yaw_pos_cmd_{ 0.0 };
  double pitch_pos_fb_{ 0.0 }, small_yaw_pos_fb_{ 0.0 },big_yaw_pos_fd_{ 0.0 };
  //扫描与目标跟踪状态（上位机只管"平滑轨迹 + 扫↔跟切换"，预测/延迟补偿由电控与视觉负责）
  enum class ScanState : uint8_t { SWEEP = 0, LOCK, TRACK, RECOVER };
  ScanState scan_state_{ ScanState::SWEEP };
  double scan_pos_{ 0.0 };       // 平滑后的扫描目标位置(yaw)
  bool target_valid_{ false };   // 视觉目标是否有效（有有效目标则从扫描切到跟踪）
  double target_pitch_{ 0.0 };   // 当前跟踪目标 pitch
  double target_yaw_{ 0.0 };     // 当前跟踪目标 yaw
  //消息类型别名 命令格式 状态格式
  using CMD = spr_msgs::msg::GimbalCmd;
  using STATE = spr_msgs::msg::GimbalState;
  /// @brief /外部状态订阅器
  rclcpp::Subscription<STATE>::SharedPtr ex_state_sub_ = nullptr;
  /// @brief /外部状态实时缓冲区，存储最新的外部状态
  realtime_tools::RealtimeBuffer<std::shared_ptr<STATE>> ex_state_rt_{ nullptr };
  /// @brief /外部命令订阅器
  rclcpp::Subscription<CMD>::SharedPtr cmd_sub_ = nullptr;
  /// @brief /外部命令实时缓冲区，存储最新的外部命令
  realtime_tools::RealtimeBuffer<std::shared_ptr<CMD>> recv_cmd_ptr_{ nullptr };
  /// @brief  /外部状态发布器（内部封装有双缓冲区 并外加一层锁，防止上一条状态没发完，这条又覆盖上一条）
  std::shared_ptr<realtime_tools::RealtimePublisher<STATE>> rt_gimbal_state_pub_{ nullptr };
  /// @brief 控制器PID参数
   std::shared_ptr<control_toolbox::PidROS> pid_pitch_pos_, pid_small_yaw_pos_,pid_big_yaw_pos_;
  
};
}//namespace spr_gimbal_controller
#endif
