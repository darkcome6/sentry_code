#通用模板
cmake_minimum_required(VERSION 3.10)
project(mypkg)#包名（必须和 package.xml 的 <name> 一致）
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()   # ① 自动找依赖
ament_auto_add_library(${PROJECT_NAME} SHARED    # ② 核心：把源码编译成库（不能省！）
  src/a.cpp
  src/b.cpp
)
ament_auto_package()                   # ③ 安装