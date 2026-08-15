#include "gimbal_controller/gimbal_controller.hpp"
#include "gimbal_controller/gimbal_controller_parameter.hpp"
#include <control_toolbox/pid_ros.hpp>

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
    param_listener_ = std::make_shared<ParamListener>(get_node());
    //获取参数
    params_ = param_listener_->get_params();
    /// @brief 初始化外部命令实时缓冲区，存储初始零值
    auto cmd= std::make_shared<CMD>();
    cmd->mode = 0;
    cmd->pitch_angle_ref = 0.0;
    cmd->small_yaw_angle_ref = 0.0;
    cmd->big_yaw_angle_ref = 0.0;
    recv_cmd_ptr_.init(cmd);
    /// @brief 初始化外部状态实时缓冲区，存储初始零值
    auto state =std::make_shared<STATE>();
    state->mode = 0;
    state->pitch_angle_ref = 0.0;
    state->small_yaw_angle_ref = 0.0;
    state->big_yaw_angle_ref = 0.0;
    state->pitch_current_ref = 0.0;
    state->small_yaw_current_ref = 0.0;
    state->big_yaw_current_ref = 0.0;
    ex_state_rt_.init(state);
    return controller_interface::CallbackReturn::SUCCESS;
};

  controller_interface::InterfaceConfiguration 
  SprGimbalController::command_interface_configuration()
  {
    std::vector<std::string> joint_names;
    joint_names.push_back(params_.pitch.joint + "/position");
    joint_names.push_back(params_.small_yaw.joint+"position");
    joint_names.push_back(params_.big_yaw.joint+"position");
    //花括号构建初始化参数，INDIVIDUAL类型 
    return { interface_configuration_type::INDIVIDUAL, joint_names };
  };

  controller_interface::InterfaceConfiguration 
  SprGimbalController::state_interface_configuration()
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
    params_ = param_listener_->get_params();
    /// @brief 初始化三个关节的PID参数
    pid_pitch_pos_ = std::make_shared<control_toolbox::PidROS>(get_node(), "pitch.pid", true);
    pid_small_yaw_pos_ = std::make_shared<control_toolbox::PidROS>(get_node(), "small_yaw.pid", true);
    pid_big_yaw_pos_ =std::make_shared<control_toolbox::PidROS>(get_node(),"big_yaw.pid",true);
    //校验是否成功
    if (!pid_pitch_pos_->initPid() || !pid_small_yaw_pos_->initPid()||pid_big_yaw_pos_->initPid())
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
  //更新参数，报文，读取数据,模式
  update_parameters();
  auto logger = get_node()->get_logger();
  auto cmd = *recv_cmd_ptr_.readFromRT();
  last_mode_ =mode_;
  mode_ =cmd->mode;
  double pitch_fb = 0.0, small_yaw_fb = 0.0,big_yaw_fb=0.0;
  double pitch_cmd = 0.0, small_yaw_cmd = 0.0,big_yaw_cmd=0.0;

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
      pitch_cmd = cmd->pitch_ref;
      yaw_cmd = cmd->yaw_ref;
      break;
    }
    case 1:
    {
      auto result =scan_mode();
     
      break;
    }
    case 2:
    {
      auto result =aim_mode();

      break;
    }
    case 3:
    {
      auto result =remote_control();
      break;
    }
    default:
      break;
  }
};

  controller_interface::CallbackReturn
  SprGimbalController::on_activate(const rclcpp_lifecycle::State& previous_state)
  {
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
  small_yaw_state_interface_ = std::make_unique<hardware_interface::LoanedCommandInterface>(
      std::move(command_interfaces_[1]));
  big_yaw_state_interface_ = std::make_unique<hardware_interface::LoanedCommandInterface>(
      std::move(command_interfaces_[2]));

  return controller_interface::CallbackReturn::SUCCESS;
  };

  controller_interface::CallbackReturn
  SprGimbalController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
  {
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
    return controller_interface::CallbackReturn::SUCCESS;
  };

  controller_interface::CallbackReturn
  SprGimbalController::on_error(const rclcpp_lifecycle::State& previous_state)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  };
  //更新参数
  void SprGimbalController::update_parameters()
  {
    if (!param_listener_->is_old(params_))
    {return;}
    params_ = param_listener_->get_params();
  }
  /// @brief 扫描模式
  std::array<double, 3> scan_mode()
  {

  };
  /// @brief 自瞄模式
  std::array<double, 3> aim_mode()
  {

  };

  /// @brief 遥控模式
  std::array<double, 3> remote_control()
  {
    
  };

  SprGimbalController::~SprGimbalController(){};
}//namespace spr_gimbal_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(spr_gimbal_controller::SprGimbalController,
                       controller_interface::ControllerInterface)
