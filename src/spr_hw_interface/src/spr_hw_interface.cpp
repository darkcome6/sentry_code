#include "spr_hw_interface.hpp"
#include <string>
namespace spr_hw_interface
{

  CanDevice::CanDevice(const std::string& interface_name, ReceiveCallback callback)
  : interface(interface_name),
    receive_callback_(callback),
    sender(std::make_shared<SocketCanSender>(interface_name)),
    receiver(std::make_shared<SocketCanReceiver>(interface_name)),
    thread_running(std::make_shared<std::atomic<bool>>(true))
  {
    receiver_thread = std::make_shared<std::thread>(&CanDevice::receiveLoop, this);
  };
  CanDevice::~CanDevice()
  {
    if (thread_running)
    {
        thread_running->store(false);
        if (receiver_thread && receiver_thread->joinable())
        {
            receiver_thread->join();
        }
    } 
  }
  void CanDevice::receiveLoop(){
    while (thread_running->load())
    {
        std::array<uint8_t, 8> rx_buff;
        uint32_t rx_id;
        if (receiver->receive(rx_buff, rx_id))
        {
            receive_callback_(rx_buff, rx_id);
        }
    }
  };
// ===== SprHardwareInterface 实现 =====
// 注意：类 SprHardwareInterface 的定义只在头文件 spr_hw_interface.hpp 中，
// 本 .cpp 只写各成员函数的实现，不要重复定义类体。

SprHardwareInterface::SprHardwareInterface() {}

SprHardwareInterface::~SprHardwareInterface() {}

// TODO: 以下虚函数实现待补全
// hardware_interface::CallbackReturn SprHardwareInterface::on_init(const hardware_interface::HardwareInfo& info);
// hardware_interface::CallbackReturn SprHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state);
// hardware_interface::CallbackReturn SprHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state);
// hardware_interface::CallbackReturn SprHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state);
// hardware_interface::CallbackReturn SprHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state);
// std::vector<hardware_interface::StateInterface> SprHardwareInterface::export_state_interfaces();
// std::vector<hardware_interface::CommandInterface> SprHardwareInterface::export_command_interfaces();
// hardware_interface::return_type SprHardwareInterface::read();
// hardware_interface::return_type SprHardwareInterface::write();

}  // namespace spr_hw_interface