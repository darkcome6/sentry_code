cmake_minimum_required(VERSION 3.8)
project(chassis_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# generate_parameter_library：根据 YAML 生成参数库
find_package(generate_parameter_library REQUIRED)

generate_parameter_library(chassis_controller_parameters
  src/chassis_controller_parameter.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/chassis_controller.cpp
)

target_link_libraries(${PROJECT_NAME} chassis_controller_parameters)

# 导出插件描述文件（controller_manager 靠它加载控制器）
pluginlib_export_plugin_description_file(controller_interface chassis_controller.xml)

ament_auto_package()
