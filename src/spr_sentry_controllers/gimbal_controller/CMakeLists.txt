cmake_minimum_required(VERSION 3.8)
project(gimbal_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()   # ① 自动找依赖

# generate_parameter_library：根据 YAML 生成参数库
find_package(generate_parameter_library REQUIRED)

#根据YAML文件创建参数文件 gimbal_controller_parameter.hpp
#gimbal_controller_parameter.cpp
generate_parameter_library(gimbal_controller_parameters
  src/gimbal_controller_parameter.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED    # ② 核心：把源码编译成库
  src/gimbal_controller.cpp
)

# ③ 链接生成的参数库（否则 include gimbal_controller_parameters.hpp 会失败）
target_link_libraries(${PROJECT_NAME} gimbal_controller_parameters)

# ④ 导出插件描述文件（controller_manager 靠它加载控制器，最不能省的一步！）
pluginlib_export_plugin_description_file(controller_interface gimbal_controller.xml)

ament_auto_package()  
