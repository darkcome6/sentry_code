# MuJoCo 硬件接口职责边界讨论总结

## 1. 讨论背景

在完成 `MujocoInterface` 头文件设计后，我们针对**硬件接口层是否应包含运动学映射逻辑**进行了深入讨论。核心分歧在于：

| 观点 | 做法 | 问题 |
| :--- | :--- | :--- |
| ❌ 将运动学塞入硬件接口 | 在 `write()` 中订阅 `/cmd_vel`，内部做麦轮逆运动学，将速度指令映射为4个轮子的转速指令 | 硬件接口变成“麦轮专用”，换构型（如差速、四足）需重写硬件层 |
| ✅ 将运动学放在控制器层 | 硬件接口只接收执行器空间指令（轮子速度/力矩），运动学由上层控制器或独立节点处理 | 硬件接口通用，可适配任何机器人构型 |

**最终结论**：用户的判断是正确的——硬件接口层应保持“干净”，只负责物理引擎读写与时间补偿，不应包含任何与“底盘”、“麦轮”相关的运动学知识。

---

## 2. 正确的职责划分

| 层级 | 职责 | 输入 | 输出 | 示例 |
| :--- | :--- | :--- | :--- | :--- |
| **控制器层** | 运动学映射 / 控制算法 | `/cmd_vel` (Twist) | `joint.command` (执行器指令) | `mecanum_drive_controller` 或自研控制器 |
| **硬件接口层** (`MujocoInterface`) | 纯物理仿真读写 | `cmd_velocities_` (执行器空间) | `state_velocities_` (执行器空间) | 读写 MuJoCo 的 `data->qvel` / `data->ctrl` |
| **物理引擎** (MuJoCo) | 刚体动力学 / 接触求解 | `data->ctrl` | `data->qpos` / `data->qvel` | `mj_step()` 积分推进 |

### 关键原则

> **硬件接口只认“执行器编号”，不认“底盘”、“左前轮”、“麦轮”这些语义概念。**

---

## 3. 修正后的头文件（已去除运动学依赖）

```cpp
#ifndef MUJOCO_INTERFACE_HPP
#define MUJOCO_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"

// 前向声明 MuJoCo 结构体（避免头文件引入 mujoco.h 造成编译依赖膨胀）
struct mjModel_;
struct mjData_;

#include <memory>
#include <string>
#include <vector>
#include <chrono>

namespace spr_hw_interface
{

class MujocoInterface : public hardware_interface::SystemInterface
{
public:
  MujocoInterface();
  ~MujocoInterface() override;

  // ---------- 生命周期回调 ----------
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  // ---------- 接口导出 ----------
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ---------- 核心读写 ----------
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

private:
  // ========== MuJoCo 物理引擎核心 ==========
  std::unique_ptr<mjModel_> model_;
  std::unique_ptr<mjData_> data_;
  std::string model_path_;
  double physics_timestep_{0.002};

  // ========== 时间补偿机制（解决ROS回调抖动） ==========
  std::chrono::steady_clock::time_point last_real_time_;
  double accumulated_time_{0.0};
  bool first_read_{true};
  int max_steps_per_read_{10};

  // ========== 执行器级命令/状态缓存 ==========
  // 注意：这里不包含任何 "底盘"、"轮子"、"运动学" 的概念
  // 它只代表 N 个执行器（joints）的原始指令和状态
  size_t joint_count_{0};

  std::vector<double> cmd_positions_;   // 位置指令
  std::vector<double> cmd_velocities_;  // 速度指令（哨兵推荐模式）
  std::vector<double> cmd_efforts_;     // 力矩指令

  std::vector<double> state_positions_;
  std::vector<double> state_velocities_;
  std::vector<double> state_efforts_;   // 实际输出力矩
};

}  // namespace spr_hw_interface

#endif  // MUJOCO_INTERFACE_HPP