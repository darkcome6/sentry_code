cmake_minimum_required(VERSION 3.8)
project(spr_control_algorithms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 纯计算算法库：不依赖 rclcpp，便于单测/复用
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/controllers/pid_controller.cpp
  src/controllers/sliding_mode_controller.cpp
  src/controllers/gravity_compensation.cpp
  src/filters/low_pass_filter.cpp
)
ament_auto_package()
