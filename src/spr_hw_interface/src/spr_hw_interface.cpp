#include "spr_hw_interface.hpp"
#include "spr_motor.hpp"

#include <limits>//std::numeric_limits
#include <string>

#include <rclcpp/clock.hpp>   // rclcpp::Clock
#include <rclcpp/time.hpp>    // rclcpp::Time

namespace spr_hw_interface
{
  CanDevice::CanDevice(const std::string& interface_name, ReceiveCallback callback)
  : interface(interface_name),
    receive_callback_(callback),//将传入的会u到函数赋值给类成员函数
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
void CanDevice::receiveLoop()
{
  std::array<uint8_t, 8> rx_data;
  while (thread_running->load() && rclcpp::ok())
  {
    try
    {//receive 将监听线程收到的数据传给rxdata 返回CANid对象 
      auto rx_frame = receiver->receive(rx_data.data(), std::chrono::milliseconds(1));
      receive_callback_(rx_data, rx_frame.get());
    }
    catch (const SocketCanTimeout&)
    {
      continue;
    }
    catch (const std::exception& e)
    {
      if (thread_running->load())
      {
        RCLCPP_ERROR(rclcpp::get_logger("CanDevice"), "Error in receive loop: %s", e.what());
      }
      break;
    }
  }
}
// ===== SprHardwareInterface 实现 =====
// 注意：类 SprHardwareInterface 的定义只在头文件 spr_hw_interface.hpp 中，
// 本 .cpp 只写各成员函数的实现，不要重复定义类体。

SprHardwareInterface::SprHardwareInterface()
:hardware_interface::SystemInterface()
{
//     // 调用父类构造函数的实现
//     SystemInterface::SystemInterface()
// : logger_(rclcpp::get_logger("hardware_interface")),     // ① 日志句柄
//   node_(std::make_shared<rclcpp::Node>("hardware_interface")), // ② 创建 rclcpp 节点
//   cmd_iface_(), state_iface_(),                          // ③ 命令/状态接口容器（空）
//   transition_cb_map_()                                   // ④ 转换回调映射表（空）
// {
//   // ⑤ 注册生命周期转换回调 —— 核心！
//   register_transition_callback(State::IDLE,        [this](const State&) { return on_configure(State::IDLE); });
//   register_transition_callback(State::INACTIVE,    [this](const State&) { return on_activate(State::INACTIVE); });
//   register_transition_callback(State::ACTIVE,      [this](const State&) { return on_deactivate(State::ACTIVE); });
//   register_transition_callback(State::UNCONFIGURED,[this](const State&) { return on_cleanup(State::UNCONFIGURED); });
//   register_transition_callback(State::FINALIZED,   [this](const State&) { return on_shutdown(State::FINALIZED); });
//   register_transition_callback(State::UNCONFIGURED,[this](const State&) { return on_error(State::UNCONFIGURED); });
// }
}
SprHardwareInterface::~SprHardwareInterface() {
//暂不实现
}

// ===== 生命周期回调函数 =====
hardware_interface::CallbackReturn
SprHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  // 先调用基类 on_init：解析并保存 info_（含 joints / 参数）
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 此时 info_ 已就绪，读取 URDF 中 <ros2_control> 段声明的关节数
  joint_count = info_.joints.size();
  // 初始化状态接口和命令接口的数组大小，并设置初始值为 NaN
  state_positions_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());
  state_velocities_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());
  state_currents_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());
  state_temperatures_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());

  cmd_positions_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());
  cmd_velocities_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());
  // 解析 info_ 中的关节信息，初始化电机配置
  for (const auto& joint : info_.joints)
  {
    Motor_Config_t config;
    config.motor_name = joint.name;

    for (const auto& [key, value] : joint.parameters)
    {
      if (key == "can_bus")
        config.can_bus = value;
      else if (key == "tx_id")
      {
        config.tx_id = std::stoi(value);
      }
      else if (key == "rx_id")
      {
        config.rx_id = std::stoi(value);
      }
      else if (key == "motor_type")
      {
        if (value == "M2006")
        {
          config.motor_type = M2006;
        }
        else if (value == "M3508")
        {
          config.motor_type = M3508;
        }
        else if (value == "GM6020")
        {
          config.motor_type = GM6020;
        }
        else if (value == "VIRTUAL_JOINT")
        {
          config.motor_type = VIRTUAL_JOINT;
        }
        else
        {
          RCLCPP_ERROR(rclcpp::get_logger("TideHardwareInterface"), "Unknown motor type: %s",
                       value.c_str());
        }
      }
      else if (key == "offset")
        config.offset = std::stoi(value);
    }
    // 创建电机对象并存储在 motors_ 映射中
    motors_[config.motor_name] = std::make_shared<DJI_Motor>(config);
    configureMotorCan(motors_[config.motor_name]);
  }
  // 初始化 CAN 设备
    for (size_t i = 0; i < can_device_count_; i++)
  {
    std::string can_device_name = "can" + std::to_string(i);
    const std::string bus_name = can_device_name;
    //传入的回调函数  lambada表达式
    auto receive_callback = [this, bus_name](const std::array<uint8_t, 8>& data, uint32_t can_id) {
      rclcpp::Time current_time = rclcpp::Clock().now();
      for (auto& motor : motors_)
      {
        if (motor->config_.can_bus == bus_name && motor->config_.rx_id == can_id)
        {
          motor->status = MOTOR_OK;
          motor->rx_buff = data;
          motor->decode_feedback();
          motor->update_timestamp(current_time);
          break;
        }
      }
    };
    can_devices_.push_back(std::make_shared<CanDevice>(can_device_name, receive_callback));
  }
  RCLCPP_INFO(rclcpp::get_logger("SprHardwareInterface"), 
            "SprHardwareInterface initialized successfully, loaded %ld joints and %ld CAN devices.", joint_count, can_device_count_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn 
SprHardwareInterface::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  std::fill(state_positions_.begin(), state_positions_.end(), 0.0);
  std::fill(state_velocities_.begin(), state_velocities_.end(), 0.0);
  std::fill(state_currents_.begin(), state_currents_.end(), 0.0);
  std::fill(state_temperatures_.begin(), state_temperatures_.end(), 0.0);
  std::fill(cmd_positions_.begin(), cmd_positions_.end(), 0.0);
  std::fill(cmd_velocities_.begin(), cmd_velocities_.end(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn 
SprHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
   return hardware_interface::CallbackReturn::SUCCESS;   
};
hardware_interface::CallbackReturn 
SprHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    try
  {
    stopMotors();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TideHardwareInterface"), "Error in on_deactivate: %s",
                 e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
};
hardware_interface::CallbackReturn 
SprHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
    try
  {
    stop_thread_ = true;
    can_devices_.clear();
    motors_.clear();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TideHardwareInterface"), "Error in on_cleanup: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
    
};
std::vector<hardware_interface::StateInterface> 
SprHardwareInterface::export_state_interfaces()
{
    
};
std::vector<hardware_interface::CommandInterface> 
SprHardwareInterface::export_command_interfaces()
{
    
};
hardware_interface::return_type 
SprHardwareInterface::read()
{
    
};
hardware_interface::return_type 
SprHardwareInterface::write()
{
    
};

// ===== 配置电机的 CAN ID 和报文标识符 =====
void SprHardwareInterface::configureMotorCan(std::shared_ptr<DJI_Motor> motor)
{
  switch (motor->config_.motor_type)
  {
    case M2006:
        motor->config_.rx_id = 0x200 + motor->config_.tx_id;
        if (motor->config_.tx_id <= 4)
        {
            motor->config_.identifier = 0x200;
        }
        else
        {
            motor->config_.tx_id -= 4;//tx_id -= 4：把"全局电机号 5~8"换算成"该控制帧内的槽位 1~4"，一帧带四个电机
            motor->config_.identifier = 0x1ff;
        }
        break;
    case M3508:
      motor->config_.rx_id = 0x200 + motor->config_.tx_id;
      if (motor->config_.tx_id <= 4)
      {
        motor->config_.identifier = 0x200;
      }
      else
      {
        motor->config_.tx_id -= 4;//tx_id -= 4：把"全局电机号 5~8"换算成"该控制帧内的槽位 1~4"，一帧带四个电机
        motor->config_.identifier = 0x1ff;
      }
      break;
    case GM6020:
      motor->config_.rx_id = 0x204 + motor->config_.tx_id;
      if (motor->config_.tx_id <= 4)
      {
        motor->config_.identifier = 0x1ff;
      }
      else
      {
        motor->config_.tx_id -= 4;//tx_id -= 4：把"全局电机号 5~8"换算成"该控制帧内的槽位 1~4"一帧带四个电机
        motor->config_.identifier = 0x2ff;
      }
      break;
    default:
      return;
  }
}
// ===== 停止所有电机 =====
void SprHardwareInterface::stopMotors()
{
  for (auto& motor : motors_)
  {
    motor->stop();
  }
}
}  // namespace spr_hw_interface