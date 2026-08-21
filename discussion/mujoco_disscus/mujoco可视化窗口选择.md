# MuJoCo 可视化窗口选择与仿真架构设计（决策记录）

日期：2026-08-21
状态：已实施（方案 A：迁移到 `mujoco_ros2_control`）

## 1. 背景

原 `MujocoInterface`（自研）把 MuJoCo 物理引擎封装在 ros2_control 硬件接口内部
（`read()` 里 `mj_step`），**无头（headless）运行**——没有 MuJoCo 原生窗口，
只能通过 RViz 看 `joint_states` 渲染的画面。

用户期望有一个 MuJoCo 原生 3D 窗口，且后续要加**场景**（崎岖地形、坡道、飞坡）
辅助调试机器人，要求一开始就把框架设计清楚、可扩展。

## 2. 可视化/仿真架构方案对比

| 方案 | 说明 | 优点 | 缺点 |
|---|---|---|---|
| **A. mujoco_ros2_control（社区成熟方案）** | 仿真独立进程（内嵌 simulate GUI 或 headless），硬件接口 `MujocoSystemInterface` 通过共享内存桥接 ros2_control | ✅ 原生 MuJoCo 窗口 ✅ `use_sim_time` 统一时钟 ✅ 机器人/场景分离天然支持 ✅ 支持 transmission/惯性自动处理 ✅ 社区维护（NASA/roboticsorg） | 需替换自研接口、launch 结构调整、学习成本 |
| B. 保留自研接口 + 场景解耦 | 物理在接口内部，拆 robot/scene 文件，`scene_path` 参数切换 | 改动小 | 无 MuJoCo 原生窗口、真实时钟（接感知要改）、场景手拼 |
| C. simulate GUI 单独跑 | `/opt/ros/.../simulate scene.xml` | 能看到 3D 画面 | **与 ros2_control 不同步**，窗口里是无人控制的模型，无法调试控制 |

## 3. 决策：方案 A（迁移到 mujoco_ros2_control）

已安装：`ros-humble-mujoco_ros2_control` / `mujoco_ros2_control_plugins` / `mujoco_vendor`。

### 架构

```
┌──────────────────────┐  共享内存  ┌───────────────────────────┐
│ mujoco_ros2_control  │ ◄────────► │ ros2_control 节点          │
│  ros2_control_node    │            │  resource_manager          │
│  内嵌 simulate GUI    │  状态←──── │    └─ MujocoSystemInterface│
│  mj_step 推进 + /clock│  ────→指令  │  read()/write()           │
└──────────────────────┘            └───────────────────────────┘
        ↑ use_sim_time=true，所有节点跟随 /clock
```

### 关键实现

- **模型传入**：ros2_control URDF 硬件段声明
  `<plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>` +
  `<param name="mujoco_model">$(find spr_sentry_description)/models/scenes/$(arg scene).xml</param>`
- **launch 分支**：`hardware_type:=mujoco` 时用 `mujoco_ros2_control/ros2_control_node`
  替代 `controller_manager/ros2_control_node`，`use_sim_time=true`；
  其余模式（real/mock）行为不变。Humble 下 remap `~/robot_description`→`/robot_description` 话题。
- **场景切换**：`scene:=flat|rough|ramp|jump` launch 参数 → xacro 选不同场景文件，
  控制器/接口零改动。

### 文件组织（机器人/场景解耦）

```
spr_sentry_description/models/
├── sentry_robot.xml        # 机器人本体（freejoint + 云台 + 4 轮 + 7 motor）
└── scenes/
    ├── flat.xml            # 平地（默认）
    ├── rough.xml           # 崎岖地形（hfield heightfield）
    ├── ramp.xml            # 坡道（倾斜 plank）
    └── jump.xml            # 飞坡（起跳台 + 落点平台）
```
每个场景 = `<include file="../sentry_robot.xml"/>` + 世界几何/灯光/天空盒。
加新场景 = 新增一个 xml，不改任何代码。

## 4. 验证结果（实测）

- `./run_sentry.sh mujoco`（flat）：MuJoCo 窗口弹出、7 执行器注册、
  `use_sim_time` 下 `/clock` 发布、gimbal/chassis/joint_state 三控制器激活。
- 发 `/cmd_vel` 0.4 m/s：四轮转动（~14 rad/3s），机器人真实前进，窗口同步受控。
- `scene:=rough`：崎岖地形（hfield）正常加载运行。场景切换生效。

## 5. 遗留问题 / 后续

1. **position/velocity 命令接口告警**：URDF 云台关节声明了 position/velocity/effort，
   但 motor 执行器下 position/velocity 需要配置 PID（`mujoco_pid.yaml`）才支持；
   当前回退到 effort 控制（与现有控制链一致），日志有 ERROR 但功能正常。
   如需要 position/velocity 仿真控制，可给插件配 PID。
2. **启动瞬间 NaN 警告**：控制器首帧输出 NaN 写入 ctrl（ACTUATOR 3, Time=0），
   一次性、不影响后续稳定；可在控制器侧初始化为 0 消除。
3. **自研 MujocoInterface 保留为备选**：`spr_hw_interface` 中仍可编译，
   与方案 A 互斥（同一 URDF 硬件段二选一），迁移稳定后可按需删除。
4. 模型惯性/摩擦仍为占位值，需按真机参数填充。
5. `use_sim_time=true` 后，RViz 需开启 "Use Sim Time"（若用 RViz 观察）。

## 6. 参考

- mujoco_ros2_control 官方：https://github.com/roboticsorg/mujoco_ros2_control
- 本机示例：`ros2 launch mujoco_ros2_control_demos demo.launch.py`
