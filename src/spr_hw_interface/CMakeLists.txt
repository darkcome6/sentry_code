cmake_minimum_required(VERSION 3.10)
project(spr_hw_interface)      # 包名（必须和 package.xml 的 <name> 一致）

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()    # 自动 find_package 所有 package.xml 里的 <depend>

# 修复 ros-humble-mujoco_vendor 导出 target 的相对路径 bug：
# mujoco_vendor::mujoco 的 INTERFACE_LINK_LIBRARIES 是 "opt/mujoco_vendor/lib/libmujoco.so"
# （缺少 /opt/ros/humble 前缀），直接链接会报 -lopt/mujoco_vendor/lib/libmujoco.so 找不到。
# 这里用 extras 里提供的绝对路径变量 MUJOCO_LIB_DIR 修正它。
find_package(mujoco_vendor REQUIRED)
if(TARGET mujoco_vendor::mujoco)
  get_target_property(_mujoco_ill mujoco_vendor::mujoco INTERFACE_LINK_LIBRARIES)
  if(_mujoco_ill AND NOT IS_ABSOLUTE "${_mujoco_ill}")
    set_target_properties(mujoco_vendor::mujoco PROPERTIES
      INTERFACE_LINK_LIBRARIES "${MUJOCO_LIB_DIR}/libmujoco.so")
  endif()
endif()

# Build the hardware interface library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/spr_motor.cpp
  src/socket_can/socket_can_common.cpp
  src/socket_can/socket_can_id.cpp
  src/socket_can/socket_can_receiver.cpp
  src/socket_can/socket_can_sender.cpp
  src/spr_hw_interface.cpp
  src/effort_mock_system.cpp
  src/mujoco_sim_interface.cpp
)
# 插件 .so 链接了 libmujoco.so.3.4.0，但 /opt/ros/humble/opt/mujoco_vendor/lib
# 不在系统默认加载路径里，controller_manager dlopen 插件时会找不到库。
# 这里把 rpath 写进插件库本身，运行时即可解析 libmujoco。
target_link_options(${PROJECT_NAME} PRIVATE "-Wl,-rpath,${MUJOCO_LIB_DIR}")
# 硬件接口编译成插件供 ROS2 controller_manager 加载使用
pluginlib_export_plugin_description_file(hardware_interface spr_hw_interface.xml)

# Export dependencies
# ament_export_include_directories(include)   # 暴露 include 目录（别人能 #include 你的头文件）
# ament_export_libraries(${PROJECT_NAME})     # 暴露你的库
# ament_export_dependencies(
#   hardware_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
#   controller_manager
# )
ament_auto_package()                       # 生成 install 目标、把头文件/库装进 install/
