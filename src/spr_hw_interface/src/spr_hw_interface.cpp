#include "spr_hw_interface.hpp"
#include "spr_motor.hpp"

#include <limits>//std::numeric_limits
#include <string>
#include <array>
#include <cstring>

#include <rclcpp/clock.hpp>   // rclcpp::Clock
#include <rclcpp/time.hpp>    // rclcpp::Time

namespace spr_hw_interface
{
  CanDevice::CanDevice(const std::string& interface_name, ReceiveCallback callback)
  : interface(interface_name),
    sender(std::make_shared<SocketCanSender>(interface_name)),
    receiver(std::make_shared<SocketCanReceiver>(interface_name)),
    thread_running(std::make_shared<std::atomic<bool>>(true)),
    receive_callback_(callback)//将传入的回调函数赋值给类成员函数
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
  /*
  struct HardwareInfo
{
  std::string name;                    // 机器人名，如 "sentry"
  std::string type;                    // "system"（来自 ros2_control type="system"）
  std::string hardware_plugin_name;    // "spr_hw_interface/SprHardwareInterface"
  std::unordered_map<std::string, std::string> hardware_parameters;  // <hardware> 下的参数
  std::vector<JointInfo> joints;       // 每个 <joint>
  std::vector<SensorInfo> sensors;     // <sensor>（你没有，空）
  std::vector<GpioInfo> gpios;         // <gpio>（你没有，空）
  std::vector<TransmissionInfo> transmissions;  // <transmission>（没有，空）
  uint32_t rw_rate;                    // 读写速率
};

struct JointInfo
{
  std::string name;                         // 如 "bigyaw_joint"
  std::string type;                         // URDF 关节类型 "continuous"
  std::vector<InterfaceInfo> state_interfaces;    // <state_interface> 列表
  std::vector<InterfaceInfo> command_interfaces;  // <command_interface> 列表
  std::unordered_map<std::string, std::string> parameters;  // <param> 列表
};

struct InterfaceInfo
{
  std::string name;   // "position" / "velocity" / "effort" ...
  std::string min;
  std::string max;
};

  */
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
  cmd_efforts_.resize(joint_count, std::numeric_limits<double>::quiet_NaN());
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
        else if (value == "DM6006")
        {
          config.motor_type = DM6006;
        }
        else if (value == "DM4310")
        {
          config.motor_type = DM4310;
        }
        else if (value == "VIRTUAL_JOINT")
        {
          config.motor_type = VIRTUAL_JOINT;
        }
        else
        {
          RCLCPP_ERROR(rclcpp::get_logger("SprHardwareInterface"), "Unknown motor type: %s",
                       value.c_str());
        }
      }
      else if (key == "offset")
        config.offset = std::stoi(value);
      else if (key == "pos_max")
        config.pos_max = std::stof(value);
      else if (key == "vel_max")
        config.vel_max = std::stof(value);
      else if (key == "tor_max")
        config.tor_max = std::stof(value);
    }
    // 创建电机对象，配置 CAN，存入 motors_ 数组（顺序对应 info_.joints）
    auto motor = std::make_shared<DJI_Motor>(config);
    configureMotorCan(motor);
    motors_.push_back(motor);
  }
  // 从 URDF <hardware> 参数读取 CAN 设备数量
  auto it = info_.hardware_parameters.find("can_device_count_");
  if (it != info_.hardware_parameters.end())
  {
    can_device_count_ = std::stoul(it->second);
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
          motor->status = MOTOR_ACTIVE;
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
  (void)previous_state;
  std::fill(state_positions_.begin(), state_positions_.end(), 0.0);
  std::fill(state_velocities_.begin(), state_velocities_.end(), 0.0);
  std::fill(state_currents_.begin(), state_currents_.end(), 0.0);
  std::fill(state_temperatures_.begin(), state_temperatures_.end(), 0.0);
  std::fill(cmd_positions_.begin(), cmd_positions_.end(), 0.0);
  std::fill(cmd_velocities_.begin(), cmd_velocities_.end(), 0.0);
  std::fill(cmd_efforts_.begin(), cmd_efforts_.end(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
};

hardware_interface::CallbackReturn 
SprHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
   (void)previous_state;
   // 达妙电机 MIT 模式上电默认失能，激活时发送使能帧
   // TODO(真机确认): 使能字节按达妙调试助手/固件版本核对
   const std::array<uint8_t, 8> dm_enable{ 0xFC, 0xFD, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
   for (const auto& motor : motors_)
   {
     if (motor->config_.motor_type == DM6006 || motor->config_.motor_type == DM4310)
     {
       for (auto can_device : can_devices_)
       {
         if (motor->config_.can_bus == can_device->interface)
         {
           sendCanFrame(can_device, dm_enable.data(), dm_enable.size(), motor->config_.tx_id);
           break;
         }
       }
     }
   }
   return hardware_interface::CallbackReturn::SUCCESS;   
};
hardware_interface::CallbackReturn 
SprHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    try
  {
    stopMotors();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SprHardwareInterface"), "Error in on_deactivate: %s",
                 e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
};
hardware_interface::CallbackReturn 
SprHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& previous_state)
{
    (void)previous_state;
    try
  {
    can_devices_.clear();
    motors_.clear();
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("SprHardwareInterface"), "Error in on_cleanup: %s", e.what());
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
      else if (command_interface.name == "effort")
      {
        interfaces.emplace_back(info_.joints[i].name, command_interface.name, &cmd_efforts_[i]);
      }
    }
  }
  return interfaces;
    
};
hardware_interface::return_type 
SprHardwareInterface::read(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  (void)period;
  auto current_time = time;
  for (size_t i = 0; i < joint_count; i++){
      auto& motor = motors_[i];  
      motor->check_connection(current_time);
      if (motor->config_.motor_type == DM6006 || motor->config_.motor_type == DM4310)
      {
        // 达妙 MIT 回传帧解包（位置/速度/力矩线性映射还原）
        motor->decode_dm_feedback();
        state_positions_[i] = motor->angle_current;
        state_velocities_[i] = motor->measure.speed_aps;
        state_currents_[i] = motor->measure.real_current;
        state_temperatures_[i] = motor->measure.temperature;
        continue;
      }
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
    (void)period;
    //清空每个can设备的发送缓冲区,防止数据残留
    for (auto can_device : can_devices_)
  {
    can_device->tx_buff.clear();
  }
  //根据电机类型和命令接口，填充每个电机的发送缓冲区
  //1.从命令接口获取命令数值（position/velocity/effort）
  for (size_t i = 0; i < joint_count; i++)
  {
    double command = 0.0;
    const std::string& iface_name = info_.joints[i].command_interfaces[0].name;
    if (iface_name == "position" && !std::isnan(cmd_positions_[i]))
    {
      command = cmd_positions_[i];
    }
    else if (iface_name == "velocity" && !std::isnan(cmd_velocities_[i]))
    {
      command = cmd_velocities_[i];
    }
    else if (iface_name == "effort" && !std::isnan(cmd_efforts_[i]))
    {
      command = cmd_efforts_[i];
    }

    auto motor = motors_[i];
    //电机失联或虚拟关节则跳过
    if (!motor->check_connection(time))
    {
      continue;
    }

    switch (motor->config_.motor_type)
    {
      // ---------- DJI 电机：int16 电流帧，一帧带4个电机 ----------
      // 分层约定：控制器统一输出力矩(N·m)，硬件层按电机类型做映射。
      //   DJI 电机（M3508/GM6020/M2006）→ 电流模式：
      //     满量程映射：tor_max(N·m) → I_MAX(电流给定满量程 16384)
      case M2006:
      case M3508:
      case GM6020:
      {
        constexpr int16_t I_MAX = 16384;  // 大疆电机电流给定满量程
        double torque = std::isnan(command) ? 0.0 : command;
        double current = torque / motor->config_.tor_max * static_cast<double>(I_MAX);
        if (current > I_MAX) current = I_MAX;
        else if (current < -I_MAX) current = -I_MAX;
        motor->output = static_cast<int16_t>(current);
        for (auto can_device : can_devices_)
        {
          if (motor->config_.can_bus == can_device->interface)
          {
             /* 一帧带 4 个电机，组内槽位由 tx_id(已换算为组内 1~4) 决定：
                  组内槽位 tx_id	占据的字节位置	data_index
                      1	          字节 0、1	        0
                      2	          字节 2、3	        2
                      3	          字节 4、5	        4
                      4	          字节 6、7	        6
                帧标识符 identifier：M3508/M2006 → 0x200/0x1FF，GM6020 → 0x1FE/0x2FE
             */
            size_t data_index = (motor->config_.tx_id - 1) * 2;

            can_device->tx_buff[motor->config_.identifier][data_index] = motor->output >> 8;
            can_device->tx_buff[motor->config_.identifier][data_index + 1] = motor->output & 0xff;

            break;
          }
        }
        break;
      }
      // ---------- 达妙电机：MIT 力矩模式（p=0,v=0,kp=0,kd=0, t_ff=目标力矩） ----------
      case DM6006:
      case DM4310:
      {
        //命令为力矩(N·m)，按 MIT 帧 12 位线性映射打包，帧ID = 电机 CAN ID
        std::array<uint8_t, 8> dm_frame{ 0 };
        DJI_Motor::encode_mit_frame(dm_frame, 0.0f, 0.0f, 0.0f, 0.0f,
                                    static_cast<float>(command), motor->config_);
        for (auto can_device : can_devices_)
        {
          if (motor->config_.can_bus == can_device->interface)
          {
            sendCanFrame(can_device, dm_frame.data(), dm_frame.size(), motor->config_.tx_id);
            break;
          }
        }
        break;
      }
      default:
        break;
    }
  }
    // 按收集到的帧标识符逐一发送（支持 0x200/0x1FF(M3508) 与 0x1FE/0x2FE(GM6020)）
    for (auto can_device : can_devices_)
  {
    for (const auto& [frame_id, frame_data] : can_device->tx_buff)
    {
      sendCanFrame(can_device, frame_data.data(), frame_data.size(), frame_id);
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
      // GM6020 专用协议：控制帧 0x1FE(电机1-4) / 0x2FE(电机5-7)，反馈 0x204+ID
      motor->config_.rx_id = 0x204 + motor->config_.tx_id;
      if (motor->config_.tx_id <= 4)
      {
        motor->config_.identifier = 0x1fe;
      }
      else
      {
        motor->config_.tx_id -= 4;//tx_id -= 4：把"全局电机号 5~7"换算成"该控制帧内的槽位 1~3"
        motor->config_.identifier = 0x2fe;
      }
      break;
    case DM4310:
      motor->config_.rx_id = 0x100 + motor->config_.tx_id;
      break;
    case DM6006:
      motor->config_.rx_id = 0x100 + motor->config_.tx_id;
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


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(spr_hw_interface::SprHardwareInterface, hardware_interface::SystemInterface)
