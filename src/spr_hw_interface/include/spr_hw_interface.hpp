#ifndef SPR_HARDWARE_INTERFACE_HPP
#define SPR_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "spr_motor.hpp"

#include "socket_can/socket_can_sender.hpp"
#include "socket_can/socket_can_receiver.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace spr_hw_interface
{
class CanDevice
{
public:
  using ReceiveCallback = std::function<void(const std::array<uint8_t, 8>&, uint32_t)>;

  CanDevice(const std::string& interface_name, ReceiveCallback callback);
  ~CanDevice();

  std::string interface;//CAN接口名称
  std::shared_ptr<SocketCanSender> sender;//发送器
  std::shared_ptr<SocketCanReceiver> receiver;//接收器
  std::shared_ptr<std::thread> receiver_thread;//接收线程
  std::shared_ptr<std::atomic<bool>> thread_running;//线程运行标志
  std::array<std::array<uint8_t, 8>, 3> tx_buff{};//发送缓冲区3个
  /*
  tx_buff[0] → [u8, u8, u8, u8, u8, u8, u8, u8]   // 8 字节（帧0）
  tx_buff[1] → [u8, u8, u8, u8, u8, u8, u8, u8]   // 8 字节（帧1）
  tx_buff[2] → [u8, u8, u8, u8, u8, u8, u8, u8]   // 8 字节（帧2）
  */
private:
  void receiveLoop();
  ReceiveCallback receive_callback_;
};


class SprHardwareInterface : public hardware_interface::SystemInterface
{
public:
  SprHardwareInterface();
  ~SprHardwareInterface() override;
  
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

private:
    void configureMotorCan(std::shared_ptr<DJI_Motor> motor);
    bool sendCanFrame(std::shared_ptr<CanDevice> device, const uint8_t* data, size_t len, uint32_t id);
    void stopMotors();
    std::vector<std::shared_ptr<CanDevice>> can_devices_;//CAN设备映射
    std::vector<std::shared_ptr<DJI_Motor>> motors_;//电机映射
    
    size_t joint_count{ 0 };
    size_t can_device_count_{ 0 };
    //数组  命令接口  状态接口
    std::vector<double> cmd_positions_;
    std::vector<double> cmd_velocities_;
    std::vector<double> state_positions_;
    std::vector<double> state_velocities_;
    std::vector<double> state_currents_;
    std::vector<double> state_temperatures_;
};
}// namespace spr_hw_interface
#endif  // SPR_HARDWARE_INTERFACE_HPP
