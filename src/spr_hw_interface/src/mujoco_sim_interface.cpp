#include "mujoco_sim_interface.hpp"

// MuJoCo 完整头文件（.cpp 中需要完整类型定义，以操作 model_ 和 data_）
#include <mujoco/mujoco.h>

// pluginlib 导出宏
#include <pluginlib/class_list_macros.hpp>

// 标准库
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace spr_hw_interface
{

MujocoInterface::MujocoInterface()
{
  // 初始化日志接口
  RCLCPP_INFO(rclcpp::get_logger("MujocoInterface"), "MujocoInterface constructed.");
}

MujocoInterface::~MujocoInterface()
{
  // MuJoCo 的 model_ 和 data_ 由 unique_ptr 自动管理内存，无需手动释放
  RCLCPP_INFO(rclcpp::get_logger("MujocoInterface"), "MujocoInterface destroyed.");
}

hardware_interface::CallbackReturn MujocoInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
 // 1. 父类初始化（必须保留）
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    joint_count_ = info_.joints.size();
    RCLCPP_INFO(rclcpp::get_logger("MujocoInterface"), 
                "Joint count from URDF: %zu", joint_count_);

    // 2. 读取硬件参数中的模型路径
    auto it = info_.hardware_parameters.find("model_path");
    if (it == info_.hardware_parameters.end())
    {
        RCLCPP_ERROR(rclcpp::get_logger("MujocoInterface"), 
                     "Missing 'model_path' in hardware_parameters!");
        return hardware_interface::CallbackReturn::ERROR;
    }
    model_path_ = it->second;

    // 3. 【关键步骤】真正加载 MuJoCo 模型文件
    //    注意：MuJoCo 3.4 的 mj_loadModel 存在 bug（对 XML 报 "Model missing header ID"），
    //    统一用 mj_loadXML 加载，它同时会返回详细的错误字符串，方便调试。
    char load_error[1024] = { 0 };
    model_.reset(mj_loadXML(model_path_.c_str(), nullptr, load_error, sizeof(load_error)));
    if (!model_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MujocoInterface"), 
                     "Failed to load MuJoCo model from: %s\n  MuJoCo error: %s",
                     model_path_.c_str(), load_error);
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 4. 创建 MuJoCo 数据空间
    data_.reset(mj_makeData(model_.get()));
    if (!data_)
    {
        RCLCPP_ERROR(rclcpp::get_logger("MujocoInterface"), 
                     "Failed to allocate MuJoCo data.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 5. 缓存物理步长（后面时间补偿会用到）
    physics_timestep_ = model_->opt.timestep;
    RCLCPP_INFO(rclcpp::get_logger("MujocoInterface"), 
                "MuJoCo model loaded. Timestep: %.4f s, DOF: %d, Actuators: %d",
                physics_timestep_, model_->nq, model_->nu);

    // 6. 【重要检查】确保 URDF 关节数与 MuJoCo 执行器数匹配
    //    通常我们通过 ROS 2 Control 控制的是执行器（actuators），数量应为 model_->nu。
    //    如果 joint_count_ != model_->nu，需要根据情况调整，否则会数组越界。
    if (joint_count_ != static_cast<size_t>(model_->nu))
    {
        RCLCPP_WARN(rclcpp::get_logger("MujocoInterface"),
                    "Joint count (%zu) differs from MuJoCo actuator count (%d). "
                    "Ensure your URDF joints map to MuJoCo actuators.",
                    joint_count_, model_->nu);
        // 在此处可以选择将 joint_count_ 调整为 model_->nu 并重新 resize 向量，
        // 但更推荐去检查 URDF 和 XML 的映射关系。
    }

    // 7. 初始化状态/命令向量（基于真实数据大小）
    //    如果 joint_count_ 与 model_->nu 不同，优先采用 model_->nu 作为实际控制数。
    //    下面以 model_->nu 为准（因为这是 MuJoCo 实际接受指令的数量）
    size_t actual_ctrl_size = static_cast<size_t>(model_->nu);
    if (joint_count_ != actual_ctrl_size)
    {
        RCLCPP_INFO(rclcpp::get_logger("MujocoInterface"),
                    "Resizing command/state vectors from %zu to %zu (MuJoCo actuators)",
                    joint_count_, actual_ctrl_size);
        joint_count_ = actual_ctrl_size;
    }

    // 命令接口初始为 NaN（未写入标记），write() 据此区分该关节走哪种指令模式；
    // 状态接口初始为 0。
    cmd_positions_.resize(joint_count_, std::numeric_limits<double>::quiet_NaN());
    cmd_velocities_.resize(joint_count_, std::numeric_limits<double>::quiet_NaN());
    cmd_efforts_.resize(joint_count_, std::numeric_limits<double>::quiet_NaN());
    state_positions_.resize(joint_count_, 0.0);
    state_velocities_.resize(joint_count_, 0.0);
    state_efforts_.resize(joint_count_, 0.0);

    RCLCPP_INFO(rclcpp::get_logger("MujocoInterface"), "on_init completed successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

hardware_interface::CallbackReturn MujocoInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  // 重置时间补偿变量（准备开始仿真）
  first_read_ = true;
  accumulated_time_ = 0.0;

  // 建立 URDF 关节名 → MuJoCo 索引 的映射（按名字匹配，不依赖声明顺序；
  // MuJoCo 里底盘有 freejoint 占 qpos/qvel 前几个元素，索引与 URDF 关节序不对齐）。
  joint_qpos_adr_.assign(joint_count_, -1);
  joint_dof_adr_.assign(joint_count_, -1);
  joint_actuator_idx_.assign(joint_count_, -1);
  for (size_t i = 0; i < joint_count_; ++i)
  {
    const std::string& jname = info_.joints[i].name;
    const int jid = mj_name2id(model_.get(), mjOBJ_JOINT, jname.c_str());
    if (jid < 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("MujocoInterface"),
                  "MuJoCo model has no joint named '%s'. This joint will be skipped.",
                  jname.c_str());
      continue;
    }
    joint_qpos_adr_[i] = model_->jnt_qposadr[jid];
    joint_dof_adr_[i] = model_->jnt_dofadr[jid];
    // 找到驱动该关节的执行器（motor 的 actuator_trnid[2*a] == 关节 id）
    for (int a = 0; a < model_->nu; ++a)
    {
      if (model_->actuator_trnid[2 * a] == jid)
      {
        joint_actuator_idx_[i] = a;
        break;
      }
    }
    if (joint_actuator_idx_[i] < 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("MujocoInterface"),
                  "No actuator drives joint '%s' — its commands will be ignored.",
                  jname.c_str());
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("MujocoInterface"), "on_configure completed.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  // 重置物理引擎到初始状态，并做一次正向动力学，保证 data 状态有效
  if (model_ && data_)
  {
    mj_resetData(model_.get(), data_.get());
    mj_forward(model_.get(), data_.get());
  }
  RCLCPP_INFO(rclcpp::get_logger("MujocoInterface"), "on_activate completed.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  // TODO: 停用硬件时的清理工作（通常无需额外操作）
  RCLCPP_INFO(rclcpp::get_logger("MujocoInterface"), "on_deactivate completed.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  // TODO: 清理资源，释放 MuJoCo 模型（unique_ptr 自动处理）
  // model_.reset(nullptr);
  // data_.reset(nullptr);

  RCLCPP_INFO(rclcpp::get_logger("MujocoInterface"), "on_cleanup completed.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// 导出状态和命令接口（Export Interfaces）
// ============================================================================

std::vector<hardware_interface::StateInterface> 
MujocoInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  // 为每个关节导出：位置、速度、力矩
  for (size_t i = 0; i < joint_count_; ++i)
  {
    const std::string& joint_name = info_.joints[i].name;

    interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, 
                            &state_positions_[i]);
    interfaces.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, 
                            &state_velocities_[i]);
    interfaces.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, 
                            &state_efforts_[i]);
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface> 
MujocoInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  // 为每个关节导出：位置/速度/力矩 命令接口
  // 注：实际使用时，根据你的控制模式（如速度控制），只需要其中一种即可。
  // 但全部导出有利于兼容不同的控制器。
  for (size_t i = 0; i < joint_count_; ++i)
  {
    const std::string& joint_name = info_.joints[i].name;

    interfaces.emplace_back(joint_name, hardware_interface::HW_IF_POSITION, 
                            &cmd_positions_[i]);
    interfaces.emplace_back(joint_name, hardware_interface::HW_IF_VELOCITY, 
                            &cmd_velocities_[i]);
    interfaces.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, 
                            &cmd_efforts_[i]);
  }

  return interfaces;
}

// ============================================================================
// 核心读写函数（Core Read/Write）
// ============================================================================

hardware_interface::return_type MujocoInterface::read(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/)
{
  // 如果模型尚未加载，直接返回 OK（避免崩溃）
  if (!model_ || !data_)
  {
    return hardware_interface::return_type::OK;
  }

  // ============ 1. 时间补偿：按真实流逝时间推进物理 ============
  // controller_manager 的调用周期未必是 physics_timestep 的整数倍，
  // 这里用真实时钟累加误差，一次读需要几步就补几步；
  // 单次上限 max_steps_per_read_ 防止掉帧后步数雪崩导致仿真失速。
  const auto now = std::chrono::steady_clock::now();
  if (first_read_)
  {
    first_read_ = false;
    accumulated_time_ = 0.0;
  }
  else
  {
    const double real_dt =
        std::chrono::duration<double>(now - last_real_time_).count();
    accumulated_time_ += real_dt;
  }
  last_real_time_ = now;

  int steps = static_cast<int>(accumulated_time_ / physics_timestep_);
  steps = std::min(steps, max_steps_per_read_);
  for (int s = 0; s < steps; ++s)
  {
    mj_step(model_.get(), data_.get());
  }
  accumulated_time_ -= static_cast<double>(steps) * physics_timestep_;

  // ============ 2. 从 MuJoCo 读取状态数据 ============
  for (size_t i = 0; i < joint_count_; ++i)
  {
    const int qadr = joint_qpos_adr_[i];
    const int dadr = joint_dof_adr_[i];
    if (qadr < 0 || dadr < 0)
    {
      continue;
    }
    state_positions_[i] = data_->qpos[qadr];   // 关节角度
    state_velocities_[i] = data_->qvel[dadr];  // 关节角速度
    state_efforts_[i] = data_->qfrc_actuator[dadr];  // 执行器施加的广义力(=力矩)
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoInterface::write(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/)
{
  // 如果模型尚未加载，直接返回 OK
  if (!model_ || !data_)
  {
    return hardware_interface::return_type::OK;
  }

  // ============ 3. 将命令写入 MuJoCo 控制接口 data->ctrl ============
  // 命令接口同时导出了 position/velocity/effort，这里按"实际被写入的接口"
  // 选择驱动方式（优先级 effort > velocity > position，与 EffortMockSystem 一致）：
  //   - effort   ：直接作为执行器力矩（motor 执行器 ctrl 即力矩）
  //   - velocity ：速度伺服  tau = vel_gain * (cmd_vel - vel)
  //   - position ：位置 PD    tau = kp*(cmd_pos - pos) - kd*vel
  // 未写入(NaN)的关节输出 0 力矩。
  for (size_t i = 0; i < joint_count_; ++i)
  {
    const int aid = joint_actuator_idx_[i];
    if (aid < 0)
    {
      continue;
    }

    double tau = 0.0;
    if (!std::isnan(cmd_efforts_[i]))
    {
      tau = cmd_efforts_[i];
    }
    else if (!std::isnan(cmd_velocities_[i]))
    {
      tau = vel_gain_ * (cmd_velocities_[i] - state_velocities_[i]);
    }
    else if (!std::isnan(cmd_positions_[i]))
    {
      tau = pos_kp_ * (cmd_positions_[i] - state_positions_[i]) -
            pos_kd_ * state_velocities_[i];
    }

    // ============ 4. 执行器限幅：饱和到模型 ctrlrange（电机力矩上下限） ============
    const double lo = model_->actuator_ctrlrange[2 * aid];
    const double hi = model_->actuator_ctrlrange[2 * aid + 1];
    data_->ctrl[aid] = std::max(lo, std::min(hi, tau));
  }

  return hardware_interface::return_type::OK;
}

} // namespace spr_hw_interface

// ============================================================================
// 插件导出宏（必须放在全局作用域）
// ============================================================================

PLUGINLIB_EXPORT_CLASS(spr_hw_interface::MujocoInterface, 
                       hardware_interface::SystemInterface)