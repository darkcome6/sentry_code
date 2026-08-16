cmake_minimum_required(VERSION 3.8)
project(spr_sentry_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 安装 URDF/Xacro 描述文件到 share 目录，供 $(find spr_sentry_description) 查找
install(DIRECTORY .
  DESTINATION share/${PROJECT_NAME}
  PATTERN "CMakeLists.txt" EXCLUDE
  PATTERN "package.xml" EXCLUDE
)

ament_package()
