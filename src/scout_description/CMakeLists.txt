cmake_minimum_required(VERSION 3.8)
project(scout_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS 2 的构建系统 ament_cmake
find_package(ament_cmake REQUIRED)

# 安装 launch、meshes 和 urdf 文件夹到 ROS 2 包的 share 目录中
install(
  DIRECTORY launch meshes urdf rviz config
  DESTINATION share/${PROJECT_NAME}
)

# 标记为 ament 包
ament_package()
