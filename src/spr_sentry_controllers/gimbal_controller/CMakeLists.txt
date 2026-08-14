cmake_minimum_required(VERSION 3.8)
project(gimbal_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
#根据YAML文件创建参数文件 gimbal_controller_parameter.hpp
#gimbal_controller_parameter.cpp
generate_parameter_library(gimbal_controller_parameters
  src/gimbal_controller_parameter.yaml
)



ament_package()
