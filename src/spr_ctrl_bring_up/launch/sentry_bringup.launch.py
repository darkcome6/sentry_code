import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('spr_ctrl_bring_up')
    xacro_file = os.path.join(pkg_share, 'description', 'sentry.xacro')
    params_file = os.path.join(pkg_share, 'config', 'sentry.yaml')

    # 通过 xacro 生成 robot_description（会解析 $(find spr_sentry_description)）
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_content}

    use_sim_time = LaunchConfiguration('use_sim_time')

    # 控制器管理节点：加载 sentry.yaml 中的 controller_manager + 控制器参数
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, params_file],
        output='screen',
    )

    # 生成关节状态（/joint_states），供 robot_state_publisher 使用
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 加载并激活云台控制器
    spawn_gimbal_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gimbal_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # 加载并激活底盘控制器
    spawn_chassis_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['chassis_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # RViz 手动启动（避免与控制器竞争资源）:
    #   source install/setup.bash
    #   rviz2 -d $(ros2 pkg prefix spr_ctrl_bring_up)/share/spr_ctrl_bring_up/config/rviz/sentry.rviz

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulated (Gazebo / ros2_control) clock'),

        # 机器人模型发布（TF + robot_description）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description, {'use_sim_time': use_sim_time}],
            output='screen',
        ),

        controller_manager,
        spawn_joint_state_broadcaster,
        spawn_gimbal_controller,
        spawn_chassis_controller,
    ])
