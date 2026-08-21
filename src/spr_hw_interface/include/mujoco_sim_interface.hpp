#ifndef MUJOCO_HARDWARE_INTERFACE_HPP
#define MUJOCO_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

// 前向声明 MuJoCo 结构体（避免头文件引入 mujoco.h 造成编译依赖膨胀）
struct mjModel_;
struct mjData_;

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>



namespace spr_hw_interface
{

class MujocoInterface : public hardware_interface::SystemInterface
{
public:
  MujocoInterface();
  ~MujocoInterface() override;
  
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

    // ========== MuJoCo 物理引擎核心 ==========
    std::unique_ptr<mjModel_> model_;
    std::unique_ptr<mjData_> data_;
    std::string model_path_;           // 从 hardware_parameters 读取
    double physics_timestep_{0.002};   // 缓存 model->opt.timestep

    // ========== 时间补偿机制 ==========
    std::chrono::steady_clock::time_point last_real_time_;
    double accumulated_time_{0.0};
    bool first_read_{true};
    int max_steps_per_read_{10};

    size_t joint_count_{ 0 };   // 关节数量（= info_.joints.size()）

    // ========== 关节 ↔ MuJoCo 索引映射（on_configure 建立，按名字匹配） ==========
    std::vector<int> joint_qpos_adr_;     // 每个 URDF 关节在 data->qpos 中的偏移
    std::vector<int> joint_dof_adr_;      // 每个 URDF 关节在 data->qvel / qfrc 中的偏移
    std::vector<int> joint_actuator_idx_; // 驱动每个关节的执行器索引（data->ctrl）

    // ========== 命令→力矩伺服增益（velocity/position 指令模式） ==========
    double vel_gain_{ 10.0 };  // 速度伺服：tau = vel_gain * (cmd_vel - vel)
    double pos_kp_{ 100.0 };   // 位置 PD：比例
    double pos_kd_{ 10.0 };    // 位置 PD：微分


    //数组  命令接口  状态接口
    std::vector<double> cmd_positions_;
    std::vector<double> cmd_velocities_;
    std::vector<double> cmd_efforts_;

    std::vector<double> state_positions_;
    std::vector<double> state_velocities_;
    std::vector<double> state_efforts_;
};
}// namespace spr_hw_interface
#endif  // MUJOCO_HARDWARE_INTERFACE_HPP
