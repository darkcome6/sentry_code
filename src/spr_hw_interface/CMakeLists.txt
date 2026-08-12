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

# Build the hardware interface library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/spr_motor.cpp
  src/socket_can/socket_can_common.cpp
  src/socket_can/socket_can_id.cpp
  src/socket_can/socket_can_receiver.cpp
  src/socket_can/socket_can_sender.cpp
  src/spr_hw_interface.cpp
)
//硬件接口编译成插件供ROS2controller_manager加载使用
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
