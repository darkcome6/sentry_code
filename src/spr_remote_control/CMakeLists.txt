cmake_minimum_required(VERSION 3.10)
project(spr_remote_control)   # 统一遥控包名（原 Python spr_remote_control 与 C++ spr_remote_control_cpp 合并）

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)   # 不用 -Wpedantic（termios2 属 GNU 扩展）
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(spr_msgs REQUIRED)

# ─── 串口驱动库（termios2，支持 DR16 DBUS 的 100000bps 非标准波特率）───
add_library(spr_serial_driver STATIC src/serial_driver.cpp)
target_include_directories(spr_serial_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ─── DBUS 协议解析库（18B 帧解码 + 合法性判帧 + 摇杆归一化）───
add_library(spr_dbus_parser STATIC src/dbus_parser.cpp)
target_include_directories(spr_dbus_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ─── 串口遥控接收主节点（最新版 C++：读线程解码 → 定时发布 cmd_vel + gimbal_cmd）───
#   用法: ros2 run spr_remote_control rc_serial_remote_cpp --ros-args -p device:=/dev/ttyTHS1
add_executable(rc_serial_remote_cpp src/rc_serial_remote_node.cpp)
ament_target_dependencies(rc_serial_remote_cpp rclcpp geometry_msgs spr_msgs)
target_link_libraries(rc_serial_remote_cpp spr_serial_driver spr_dbus_parser)
install(TARGETS rc_serial_remote_cpp DESTINATION lib/${PROJECT_NAME})

# ─── 键盘遥控节点（Python 自包含脚本 → 安装为无扩展名程序）───
#   用法: ros2 run spr_remote_control keyboard_remote
install(PROGRAMS scripts/keyboard_remote.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME keyboard_remote)

ament_package()
