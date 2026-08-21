import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('spr_ctrl_bring_up')
    xacro_file = os.path.join(pkg_share, 'description', 'sentry.xacro')
    params_file = os.path.join(pkg_share, 'config', 'sentry.yaml')

    hardware_type = LaunchConfiguration('hardware_type').perform(context)
    scene = LaunchConfiguration('scene').perform(context)
    # use_sim_time 保持为 LaunchConfiguration 替换对象（非 mujoco 模式原样传递）
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 通过 xacro 生成 robot_description（解析 $(find ...) 与 hardware_type / scene 参数）
    robot_description_content = Command(
        ['xacro ', xacro_file, ' hardware_type:=', hardware_type, ' scene:=', scene])
    robot_description = {'robot_description': robot_description_content}

    # ---- 控制器管理节点 ----
    # mujoco 模式：用 mujoco_ros2_control 提供的节点（内嵌 MuJoCo 仿真 + 原生窗口，
    # 通过共享内存桥接 ros2_control），所有节点使用仿真时钟。
    # 其余模式：标准 controller_manager 节点。
    if hardware_type == 'mujoco':
        sim_time = True   # 仿真时钟必须为 bool
        control_node = Node(
            package='mujoco_ros2_control',
            executable='ros2_control_node',
            parameters=[{'use_sim_time': True}, params_file],
            # Humble 下从 /robot_description 话题读取 robot_description
            remappings=[('~/robot_description', '/robot_description')]
            if os.environ.get('ROS_DISTRO') == 'humble' else [],
            output='screen',
            on_exit=Shutdown(),
        )
    else:
        sim_time = use_sim_time
        control_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, params_file, {'use_sim_time': use_sim_time}],
            output='screen',
        )

    def spawn(controller):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[controller, '--controller-manager', '/controller_manager'],
            output='screen',
        )

    # RViz 手动启动（避免与控制器竞争资源）:
    #   source install/setup.bash
    #   rviz2 -d $(ros2 pkg prefix spr_ctrl_bring_up)/share/spr_ctrl_bring_up/config/rviz/sentry.rviz

    return [
        # 机器人模型发布（TF + robot_description）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': sim_time}],
            output='screen',
        ),
        control_node,
        spawn('joint_state_broadcaster'),
        spawn('gimbal_controller'),
        spawn('chassis_controller'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'hardware_type', default_value='mock',
            description='Hardware backend: real (CAN), mock (simulated), or mujoco (MuJoCo)'),
        DeclareLaunchArgument(
            'scene', default_value='flat',
            description='MuJoCo scene: flat / rough / ramp / jump (only for hardware_type:=mujoco)'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulated clock'),
        OpaqueFunction(function=launch_setup),
    ])
