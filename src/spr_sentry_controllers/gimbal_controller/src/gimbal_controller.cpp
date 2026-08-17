#include "gimbal_controller/gimbal_controller.hpp"
#include <gimbal_controller/gimbal_controller_parameters.hpp>
#include <control_toolbox/pid_ros.hpp>

#include <algorithm>
#include <cmath>
#include <array>

namespace spr_gimbal_controller
{
//
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;

  SprGimbalController::SprGimbalController() : controller_interface::ControllerInterface()
  {
  };

  controller_interface::CallbackReturn 
  SprGimbalController::on_init()
  {
    //创建参数监听器
    param_listener_ = std::make_shared<spr_gimbal_controller::ParamListener>(get_node());
    //获取参数
    params_ = param_listener_->get_params();
    
    /// @brief 初始化外部命令实时缓冲区，存储初始零值
    auto cmd= std::make_shared<CMD>();
    cmd->mode = 0;
    cmd->pitch_angle = 0.0;
    cmd->small_yaw_angle = 0.0;
    cmd->big_yaw_angle = 0.0;
    recv_cmd_ptr_.initRT(cmd);
    /// @brief 初始化外部状态实时缓冲区，存储初始零值
    auto state =std::make_shared<STATE>();
    state->mode = 0;
    
    state->pitch_angle_ref = 0.0;
    state->small_yaw_angle_ref = 0.0;
    state->big_yaw_angle_ref = 0.0;

    state->pitch_current_ref = 0.0;
    state->small_yaw_current_ref = 0.0;
    state->big_yaw_current_ref = 0.0;
    ex_state_rt_.initRT(state);
    return controller_interface::CallbackReturn::SUCCESS;
};

  controller_interface::InterfaceConfiguration 
  SprGimbalController::command_interface_configuration() const
  {
    std::vector<std::string> joint_names;
    joint_names.push_back(params_.pitch.joint + "/position");
    joint_names.push_back(params_.small_yaw.joint + "/position");
    joint_names.push_back(params_.big_yaw.joint + "/position");
    //花括号构建初始化参数，INDIVIDUAL类型 
    return { interface_configuration_type::INDIVIDUAL, joint_names };
  };

  controller_interface::InterfaceConfiguration 
  SprGimbalController::state_interface_configuration() const
  {
    std::vector<std::string> joint_names;
    joint_names.push_back(params_.pitch.joint + "/position");
    joint_names.push_back(params_.small_yaw.joint + "/position");
    joint_names.push_back(params_.big_yaw.joint + "/position");
    //花括号构建初始化参数，INDIVIDUAL类型
    return { interface_configuration_type::INDIVIDUAL, joint_names };
  };



  controller_interface::CallbackReturn
  SprGimbalController::on_configure(const rclcpp_lifecycle::State& previous_state)
  {
    (void)previous_state;
    params_ = param_listener_->get_params();
    /// @brief 初始化三个关节的PID参数
    pid_pitch_pos_ = std::make_shared<control_toolbox::PidROS>(get_node(), "pitch.pid", true);
    pid_small_yaw_pos_ = std::make_shared<control_toolbox::PidROS>(get_node(), "small_yaw.pid", true);
    pid_big_yaw_pos_ =std::make_shared<control_toolbox::PidROS>(get_node(),"big_yaw.pid",true);
    //校验是否成功
    bool ok_pitch = pid_pitch_pos_->initPid();
    bool ok_small = pid_small_yaw_pos_->initPid();
    bool ok_big = pid_big_yaw_pos_->initPid();
    if (!ok_pitch || !ok_small || !ok_big)
    {
       RCLCPP_ERROR(get_node()->get_logger(), "Failed to config the params of PIDs.");
       return controller_interface::CallbackReturn::ERROR;
    }
    /// @brief 命令订阅器传入指针，~/gimbal_cmd话题，实时写入双缓存区
    cmd_sub_ = get_node()->create_subscription<CMD>(
        "~/gimbal_cmd", rclcpp::SystemDefaultsQoS(),
        [this](const std::shared_ptr<CMD> msg) -> void {  recv_cmd_ptr_.writeFromNonRT(msg); });
    /// @brief 外部状态订阅器传入指针，~/ex_state_interface ，实时写入双缓存区
    ex_state_sub_ = get_node()->create_subscription<STATE>(
        "~/ex_state_interface", rclcpp::SensorDataQoS(),
        [this](const std::shared_ptr<STATE> msg) -> void { ex_state_rt_.writeFromNonRT(msg); });
    /// @brief  创建外部状态发布器
    auto gimbal_state_pub =
      get_node()->create_publisher<STATE>("~/gimbal_state", rclcpp::SystemDefaultsQoS());
    /// @brief 将 外部状态发布器 包装为 实时安全发布器
    rt_gimbal_state_pub_ =
      std::make_shared<realtime_tools::RealtimePublisher<STATE>>(gimbal_state_pub);
    return controller_interface::CallbackReturn::SUCCESS;
  };

  controller_interface::return_type 
  SprGimbalController::update(const rclcpp::Time& time,const rclcpp::Duration& period)
  {
  (void)time;
  (void)period;
  //更新参数，报文，读取数据,模式
  update_parameters();
  auto cmd = *recv_cmd_ptr_.readFromRT();
  last_mode_ =mode_;
  mode_ =cmd->mode;
  double pitch_fb = 0.0, small_yaw_fb = 0.0,big_yaw_fb=0.0;

  pitch_fb = pitch_state_interface_->get_value();
  small_yaw_fb =small_yaw_state_interface_->get_value();
  big_yaw_fb = big_yaw_state_interface_->get_value();
  
  pitch_pos_fb_=pitch_fb;
  small_yaw_pos_fb_=small_yaw_fb;
  big_yaw_pos_fd_=big_yaw_fb;

  
  switch (mode_)////0 保持不动 1 扫描模式 2自瞄模式 3遥控模式
  {
    case 0:
    {
      // 保持不动模式
      break;
    }
    case 1:
    {
      (void)scan_mode();
      break;
    }
    case 2:
    {
      (void)aim_mode();
      break;
    }
    case 3:
    {
      (void)remote_control();
      break;
    }
    default:
      break;
  }
  return controller_interface::return_type::OK;
};

  controller_interface::CallbackReturn
  SprGimbalController::on_activate(const rclcpp_lifecycle::State& previous_state)
  {
  (void)previous_state;
  //借出状态接口接口
  pitch_state_interface_ = std::make_unique<const hardware_interface::LoanedStateInterface>(
      std::move(state_interfaces_[0]));
  small_yaw_state_interface_ = std::make_unique<const hardware_interface::LoanedStateInterface>(
      std::move(state_interfaces_[1]));
  big_yaw_state_interface_ = std::make_unique<const hardware_interface::LoanedStateInterface>(
      std::move(state_interfaces_[2]));
  //借出命令接口
  pitch_command_interface_ = std::make_unique<hardware_interface::LoanedCommandInterface>(
      std::move(command_interfaces_[0]));
  small_yaw_command_interface_ = std::make_unique<hardware_interface::LoanedCommandInterface>(
      std::move(command_interfaces_[1]));
  big_yaw_command_interface_ = std::make_unique<hardware_interface::LoanedCommandInterface>(
      std::move(command_interfaces_[2]));

  return controller_interface::CallbackReturn::SUCCESS;
  };

  controller_interface::CallbackReturn
  SprGimbalController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
  {
  (void)previous_state;
  //重置状态接口
  pitch_state_interface_.reset();
  big_yaw_state_interface_.reset();
  small_yaw_state_interface_.reset();
  //重置命令接口
  pitch_command_interface_.reset();
  big_yaw_command_interface_.reset();
  small_yaw_command_interface_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
  };

  controller_interface::CallbackReturn
  SprGimbalController::on_cleanup(const rclcpp_lifecycle::State& previous_state)
  {
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
  };

  controller_interface::CallbackReturn
  SprGimbalController::on_error(const rclcpp_lifecycle::State& previous_state)
  {
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
  };
  //更新参数
  void SprGimbalController::update_parameters()
  {
    if (!param_listener_->is_old(params_))
    {return;}
    params_ = param_listener_->get_params();
  }
  /// @brief 扫描模式：时间驱动平滑扫描；一旦视觉给出有效目标，自动切到自瞄跟踪
  std::array<double, 3> SprGimbalController::scan_mode()
  {
    // 1) 目标协同：有有效视觉目标 → 直接转自瞄跟踪（扫↔跟切换）
    if (target_valid_)
    {
      return aim_mode();
    }

    // 2) 时间驱动三角波扫描（与 update 帧率无关，天然无跳变）
    //    相比"每帧加固定步长"，用墙钟时间算扫描角，帧率抖动不影响轨迹
    const double range_min = params_.pitch.scan_range[0];
    const double range_max = params_.pitch.scan_range[1];
    const double period = 4.0;  // 完整往返周期(s)，后续建议做成参数
    const double t = get_node()->now().seconds();

    // 三角波：一个周期内 0→1→0
    const double tri = 2.0 * std::abs(std::fmod(t / period, 1.0) - 0.5);
    const double target = range_min + (range_max - range_min) * tri;

    // 3) 斜坡限速：目标位置一次最多变化 max_step，防位置环猛冲/过冲
    //    scan_add 语义改为"每周期最大角增量"，配合 1000Hz 即最大扫描角速度
    const double max_step = params_.pitch.scan_add;
    scan_pos_ += std::clamp(target - scan_pos_, -max_step, max_step);

    // 4) 软限位保护
    scan_pos_ = std::clamp(scan_pos_, params_.pitch.min, params_.pitch.max);

    // 返回 {pitch目标, small_yaw目标, big_yaw目标}（两个 yaw 一起扫）
    return { pitch_pos_fb_, scan_pos_, scan_pos_ };
  }

  /// @brief 自瞄模式：读取视觉话题(外部状态)目标角度，并记录跟踪状态供扫描切换
  std::array<double, 3> SprGimbalController::aim_mode()
  {
    auto state = *ex_state_rt_.readFromRT();
    // 视觉目标（角度）——话题结构按你的约定：pitch/yaw 目标角 + 目标角速度
    target_pitch_ = state->pitch_angle_ref;
    target_yaw_ = state->small_yaw_angle_ref;
    target_valid_ = true;

    // TODO(速度前馈)：视觉给的目标角速度可在这里读出来，叠加到位置命令上，
    // 用于补偿跟踪滞后（没有 IMU 时这是唯一的前馈来源）
    // double target_yaw_vel = state->xxx;  // 需视觉话题提供角速度字段
    // target_yaw_ += target_yaw_vel * /* 前馈增益 */;

    return { target_pitch_, target_yaw_, target_yaw_ };
  }

  /// @brief 遥控模式：直接采用遥控器下发的角度参考
  std::array<double, 3> SprGimbalController::remote_control()
  {
    auto cmd = *recv_cmd_ptr_.readFromRT();
    return { cmd->pitch_angle, cmd->small_yaw_angle, cmd->big_yaw_angle};
  }

  SprGimbalController::~SprGimbalController(){};
}//namespace spr_gimbal_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(spr_gimbal_controller::SprGimbalController,
                       controller_interface::ControllerInterface)
