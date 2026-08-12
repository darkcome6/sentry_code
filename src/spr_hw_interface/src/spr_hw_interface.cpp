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
    // 创建电机对象，配置 CAN，存入 motors_ 数组（顺序对应 info_.joints）
    auto motor = std::make_shared<DJI_Motor>(config);
    configureMotorCan(motor);
    motors_.push_back(motor);
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
  std::vector<hardware_interface::StateInterface> interfaces;
  /*
  StateInterface 对象(
    prefix_name_   = "left_wheel",      // info_.joints[i].name
    interface_name_ = "position",       // state_interface.name
    value_ptr_     = &state_positions_[i]  // 指向状态数组第 i 个元素
    );
  */
  for (size_t i = 0; i < joint_count; i++)
  {
    for (const auto& state_interface : info_.joints[i].state_interfaces)
    {
      if (state_interface.name == "position")
      {
        interfaces.emplace_back(info_.joints[i].name, state_interface.name, &state_positions_[i]);
      }
      else if (state_interface.name == "velocity")
      {
        interfaces.emplace_back(info_.joints[i].name, state_interface.name, &state_velocities_[i]);
      }
      else if (state_interface.name == "effort")
      {
        interfaces.emplace_back(info_.joints[i].name, state_interface.name, &state_currents_[i]);
      }
    }
  }
  return interfaces;  
};
std::vector<hardware_interface::CommandInterface> 
SprHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < joint_count; i++)
  {
    for (const auto& command_interface : info_.joints[i].command_interfaces)
    {
      if (command_interface.name == "position")
      {
        interfaces.emplace_back(info_.joints[i].name, command_interface.name, &cmd_positions_[i]);
      }
      else if (command_interface.name == "velocity")
      {
        interfaces.emplace_back(info_.joints[i].name, command_interface.name, &cmd_velocities_[i]);
      }
    }
  }
  return interfaces;
    
};
hardware_interface::return_type 
SprHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  auto current_time = time;
  for (size_t i = 0; i < joint_count; i++){
      auto& motor = motors_[i];  
      motor->check_connection(current_time);
      state_positions_[i] = motor->angle_current;
      state_velocities_[i] = motor->measure.speed_aps;
      state_currents_[i] = motor->measure.real_current;
      state_temperatures_[i] = motor->measure.temperature;
  }
  return hardware_interface::return_type::OK;
};
hardware_interface::return_type 
SprHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{
    //清空每个can设备的发送缓冲区,防止数据残留
    for (auto can_device : can_devices_)
  {
    for (size_t i = 0; i < 3; i++)
    {
      can_device->tx_buff[i].fill(0);
    }
  }
  //根据电机类型和命令接口，填充每个电机的发送缓冲区
  //1.从命令接口获取命令数值
  for (size_t i = 0; i < joint_count; i++)
  {
    double command = 0.0;
    if (!std::isnan(cmd_positions_[i]) && info_.joints[i].command_interfaces[0].name == "position")
    {
      command = (cmd_positions_[i]);
    }
    else if (!std::isnan(cmd_velocities_[i]) &&
             info_.joints[i].command_interfaces[0].name == "velocity")
    {
      command = cmd_velocities_[i];
    }

    auto motor = motors_[i];
    //将命令数值转换为电机输出值，并写入电机对象
    motor->output = static_cast<int16_t>(command);
    //2.检查电机,将命令数值写入缓存区
    auto current_time = time;
    if(motor->check_connection(current_time))
    {
    for (auto can_device : can_devices_)
      {
        if (motor->config_.can_bus == can_device->interface)
        {
           /*根据电机的报文标识符选择发送缓冲区的索引 
           一帧能和控制四个但是这四个电机的标识符必须一样
            但是既可能是0x200也可能是0x1ff也可能是0X2ff
           */
          size_t buff_index = (motor->config_.identifier == 0x200) ? 0 :
                              (motor->config_.identifier == 0x1ff) ? 1 :2;
          
          /*在这帧里的第几个字节大疆的电机一帧带四个电机的电流
                组内槽位 tx_id	占据的字节位置	data_index
                    1	          字节 0、1	        0
                    2	          字节 2、3	        2
                    3	          字节 4、5	        4
                    4	          字节 6、7	        6
          */
          size_t data_index = (motor->config_.tx_id - 1) * 2;

          can_device->tx_buff[buff_index][data_index] = motor->output >> 8;
          can_device->tx_buff[buff_index][data_index + 1] = motor->output & 0xff;

          break;
        }
      }
    }
  }
    for (auto can_device : can_devices_)
  {
    for (size_t i = 0; i < 3; i++)
    {
      /*
        条件	                 结果 id
        i == 0                   成立	0x200
        i == 0                   不成立，且 i == 1 成立	0x1ff
        以上都不成立（i == 2）	  0x2ff
      */
      auto id = (i == 0) ? 0x200 : (i == 1) ? 0x1ff : 0x2ff;
      bool result = sendCanFrame(can_device, can_device->tx_buff[i].data(), 8, id);
    }
  }
  return hardware_interface::return_type::OK;
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
//
bool SprHardwareInterface::sendCanFrame(std::shared_ptr<CanDevice> device, const uint8_t* data, size_t len, uint32_t id)
{
    try
  {//创建CANid对象,设置帧类型为数据帧,标准帧
    CanId send_id(id, 0, FrameType::DATA, StandardFrame);
    device->sender->send(data, len, send_id, std::chrono::milliseconds(1));
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TideHardwareInterface"), "Failed to send CAN frame: %s",
                 e.what());
    return false;
  }
  return true;
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