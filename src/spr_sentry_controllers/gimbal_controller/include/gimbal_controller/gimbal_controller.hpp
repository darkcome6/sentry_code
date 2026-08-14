#include "gimbal_controller/gimbal_controller.hpp"
namespace spr_gimbal_controller
{
class SprGimbalController : public controller_interface::ControllerInterface
{
public:
  SprGimbalController();
  ~SprGimbalController();
  controller_interface::CallbackReturn SprGimbalController::on_init() override;

  controller_interface::return_type SprGimbalController::update(const rclcpp::Time& time,const rclcpp::Duration& period) override;
 
  controller_interface::InterfaceConfiguration SprGimbalController::command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration SprGimbalController::state_interface_configuration() const override;

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
  std::shared_ptr<ParamListener> param_listener_;
  Params params_;
  //更新参数
  void update_parameters();
  //控制器模式
  uint8_t mode_{ 0 };
  uint8_t last_mode_{ 0 };
  
}
}//namespace spr_gimbal_controller
