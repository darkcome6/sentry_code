#include "gimbal_controller/gimbal_controller.hpp"
#include "gimbal_controller/gimbal_controller_parameter.hpp"

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
  };

  controller_interface::InterfaceConfiguration 
  SprGimbalController::command_interface_configuration()
  {
    std::vector<std::string> joint_names;
    joint_names.push_back(params_.pitch.joint + "/position");
    joint_names.push_back(params_.yaw.joint + "/position");
    //花括号构建初始化参数，INDIVIDUAL类型 
    return { interface_configuration_type::INDIVIDUAL, joint_names };
  };

  controller_interface::InterfaceConfiguration 
  SprGimbalController::state_interface_configuration()
  {
    std::vector<std::string> joint_names;
    joint_names.push_back(params_.pitch.joint + "/position");
    joint_names.push_back(params_.yaw.joint + "/position");
    //花括号构建初始化参数，INDIVIDUAL类型
    return { interface_configuration_type::INDIVIDUAL, joint_names };
  };

  controller_interface::return_type 
  SprGimbalController::update(const rclcpp::Time& time,const rclcpp::Duration& period);

  controller_interface::CallbackReturn
  SprGimbalController::on_configure(const rclcpp_lifecycle::State& previous_state);

  controller_interface::CallbackReturn
  SprGimbalController::on_activate(const rclcpp_lifecycle::State& previous_state)
  ;

  controller_interface::CallbackReturn
  SprGimbalController::on_deactivate(const rclcpp_lifecycle::State& previous_state);

  controller_interface::CallbackReturn
  SprGimbalController::on_cleanup(const rclcpp_lifecycle::State& previous_state);

  controller_interface::CallbackReturn
  SprGimbalController::on_error(const rclcpp_lifecycle::State& previous_state);
  //更新参数
  void SprGimbalController::update_parameters()
  {
    if (!param_listener_->is_old(params_))
    {return;}
    params_ = param_listener_->get_params();
  }
  SprGimbalController::~SprGimbalController(){};
}//namespace spr_gimbal_controller
