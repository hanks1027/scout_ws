cmake_minimum_required(VERSION 3.8)
project(scout_description)

# 设置 C++ 标准为 C++17（ROS 2 Jazzy 推荐）
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 查找 ROS 2 及第三方依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(OpenCV REQUIRED)

# 1. 必须寻找依赖包（若已有某些包，在后面叠加即可）
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 2. 添加 C++ 节点编译目标
add_executable(arm_bounding_box_node src/arm_bounding_box_node.cpp)
add_executable(base_footprint_publisher src/base_footprint_publisher.cpp)

# 3. 链接依赖库
ament_target_dependencies(arm_bounding_box_node
  rclcpp
  tf2_ros
  geometry_msgs
  visualization_msgs
  urdf
  OpenCV
)

ament_target_dependencies(base_footprint_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
# 4. 安装编译好的可执行文件到 lib 目录（ros2 run 找的就是这里）
install(
  TARGETS 
    arm_bounding_box_node
    base_footprint_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# 5. 安装资源文件夹（launch, urdf, meshes 等）到 share 目录
install(
  DIRECTORY launch meshes urdf rviz config worlds models src
  DESTINATION share/${PROJECT_NAME}
)

# 标记为 ament 包
ament_package()