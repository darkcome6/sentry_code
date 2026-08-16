cmake_minimum_required(VERSION 3.8)
project(spr_ctrl_bring_up)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装 launch 与 config 目录到 share 目录
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
