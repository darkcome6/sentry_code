# MuJoCo 仿真后续开发指南（面向运动控制 / 对标下位机电控）

日期：2026-08-21
面向读者：负责运动控制（对标下位机电控）的同学
前提：已迁移到 `mujoco_ros2_control/MujocoSystemInterface` 成熟方案

---

## 1. 先澄清一个关键认知：插件不是"只有力矩接口"

接口类型由**两处**共同决定，不是插件写死的：

| 命令接口 | 需要的条件 | 说明 |
|---|---|---|
| **effort（力矩）** | MJCF 用 `motor` 执行器 | 直接映射到 `data->ctrl`（当前已具备） |
| **velocity（速度）** | ① `motor` + `pids_config_file` 配速度 PID；或 ② MJCF 用 `velocity` 执行器 | 插件/MuJoCo 内部做速度环 |
| **position（位置）** | ① `motor` + `pids_config_file` 配位置 PID；或 ② MJCF 用 `position` 执行器 | 插件/MuJoCo 内部做位置环 |

> 你现在看到"只有 effort"，是因为 `sentry_robot.xml` 全用 `motor` 且没配 `pids_config_file`。
> 日志里的 `Position command interface ... not supported ... without defining the PIDs` 就是这个意思——**配了 PID 就支持**。

**结论：接口层面不存在障碍，三种接口都能有。**

---

## 2. 运动控制仿真应如何分层（对标下位机电控）

把仿真想成真机的一条链路：

```
你的控制器 (gimbal_controller / chassis_controller)   ← 这就是你的"电控层"
        │ 输出 effort / velocity / position 指令
        ▼
ros2_control 命令接口
        ▼
MujocoSystemInterface + MuJoCo 物理   ← 这就是"电机 + 机械"被控对象
        │ 真实动力学响应（惯量/摩擦/接触）
        ▼
状态接口 → 反馈回控制器
```

关键理解：

- **你写的控制器 = 电控**；**MuJoCo = 电机 + 车体机械**。
- 真机上电机有**电流环 / 速度环 / 位置环**三种工作模式，对应 MuJoCo 三种执行器：
  - 电流模式（DM 力矩 / DJI 电流）≈ `motor` 执行器（effort）
  - 速度模式 ≈ `velocity` 执行器 或 `motor`+速度 PID
  - 位置模式 ≈ `position` 执行器 或 `motor`+位置 PID
- **选哪种接口 = 模拟"电机工作在哪个模式"**，取决于你真机电机实际配置。

**建议**：你的控制链是"位置环 → 力矩参考"（串级），底层就是 effort，所以
**当前 effort 接口是对的**——它代表"你直接控电机电流/力矩"（最底层、最接近真实电控）。
速度/位置接口用于**模拟电机自带速度/位置环**的工况（例如某些模式直接下发速度给电机）。

---

## 3. 你的需求对应要做什么

### 3.1 需要速度/位置指令接口 → 配 `pids_config_file`

在 `spr_ctrl_bring_up/config/` 下新建 `mujoco_pid.yaml`：

```yaml
/**:
  ros__parameters:
    pid_gains:
      position:
        bigyaw_joint:
          p: 500.0
          i: 0.0
          d: 20.0
          u_clamp_max: 10.0
          u_clamp_min: -10.0
          i_clamp_max: 10.0
          i_clamp_min: -10.0
        # ... 每个需要 position/velocity 的关节都写一份
      velocity:
        left_wheel_front_joint:
          p: 5.0
          i: 0.0
          d: 0.0
          u_clamp_max: 30.0
          u_clamp_min: -30.0
          i_clamp_max: 30.0
          i_clamp_min: -30.0
```

`Sentry.xacro` mujoco 硬件段加：
```xml
<param name="pids_config_file">$(find spr_ctrl_bring_up)/config/mujoco_pid.yaml</param>
```
（或把部分关节的执行器改成 `position`/`velocity` 类型，见 4.1）

### 3.2 调参/优化控制效果的前提：物理参数真实

**这是你最该关注的部分**——接口再多，物理参数是占位的，调参就没有意义。

| 参数 | 现状 | 影响 |
|---|---|---|
| 质量/惯量 | 占位 | 加速/减速动态失真 → PID 增益无法迁移到真机 |
| 摩擦/阻尼 | MuJoCo 默认 | 稳态误差、死区、低速性能失真 |
| 减速比（transmission） | 无 | 电机轴力矩/速度与关节不匹配（M3508 约 19:1） |
| 电机动态（电流环延迟） | 理想力矩源 | 高频行为失真（影响带宽判断） |

**建议按优先级做动力学标定**：
1. 各部件质量/惯量（称重 + 简单几何估算，或 CAD 提供）
2. 关节摩擦（实测：匀速时电流 vs 转速 → 库仑+粘滞摩擦系数）
3. 减速比（查电机手册 M3508 / DM 减速箱，用 transmission 建模）

### 3.3 调参工具链（数据记录 / 评估 / 自动化）

- **数据记录**：`ros2 bag record /joint_states /gimbal_controller/gimbal_state /cmd_vel` 等，离线分析指令/响应/控制量。
- **性能指标**：超调量、调节时间、稳态误差、带宽、抗扰（加地形/障碍场景）。
- **自动化调参**：用 launch 参数批量跑（不同 PID 增益 → bag 记录 → 脚本算指标），`sim_speed_factor` 可加速。
- **扰动注入**：靠场景（rough/ramp/jump）或临时给模型加外力。

---

## 4. 分阶段路线图

### 阶段 1：接口完备（近期，1~2 天）
- [ ] 建 `mujoco_pid.yaml` + 硬件段加 `pids_config_file` → 支持 position/velocity 指令
- [ ] 云台加 `<camera>`（视觉自瞄仿真）、开 `camera_publish_rate`
- [ ] `start_positions.xml` 初始位姿（坡道/飞坡定向测试）
- [ ] 数据记录脚本（ros2 bag）

### 阶段 2：动力学标定（中期，关键，1~2 周）
- [ ] 填真实质量/惯量（各 link）
- [ ] 填关节摩擦/阻尼（实测拟合）
- [ ] transmission 减速比建模（M3508 / DM）
- [ ] 模型/真机对比验证（同一 PID 在仿真和真机响应一致性）

### 阶段 3：进阶（长期，按需）
- [ ] 电机动态建模（电流环延迟/带宽，可用自定义 actuator 或传递函数近似）
- [ ] 随机扰动/噪声注入（负载扰动、路面不平）
- [ ] 半实物（HIL）：真控制器代码直接驱动仿真
- [ ] 仿真与真机"一键迁移"（同一套参数跑两处）

---

## 5. 常见坑

1. **调参要建立在真实物理参数上**：占位惯量下调出的 PID 增益，到真机上很可能发散或太软。
2. **位置/速度接口 ≠ 控制好**：`pids_config_file` 的 PID 是"模拟电机内部环"，不代表你的控制效果；你的串级位置环才是要调的。
3. **motor 执行器的 ctrlrange 是力矩限幅**，别和 PID 输出限幅混为一谈。
4. **use_sim_time=true 后**，若用 RViz 需开 "Use Sim Time"，否则画面不跟仿真走。
5. **启动瞬间 NaN 警告**：控制器首帧输出 NaN 到 ctrl，可在控制器侧把命令初始化为 0 消除。

---

## 6. 一句话总结

> 接口不是瓶颈（position/velocity/effort 都能有）；
> **瓶颈是物理参数真实性和控制调参工具链**。
> 先把动力学标定做了，你的运动控制调参才有意义、才能迁移到真机。
