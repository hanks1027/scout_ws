cmake_minimum_required(VERSION 3.16)
project(livox_mid360_gz_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(sensor_msgs REQUIRED)

set(GZ_SIM_FOUND FALSE)
find_package(gz-sim8 QUIET
  HINTS ${CMAKE_PREFIX_PATH}/opt/gz_sim_vendor/extra_cmake/lib/cmake
        /opt/ros/${ROS_DISTRO}/opt/gz_sim_vendor/extra_cmake/lib/cmake
)
if(gz-sim8_FOUND)
  set(GZ_SIM_VER 8)
  set(GZ_TARGET gz-sim8::core)
  set(GZ_SIM_FOUND TRUE)
endif()
if(NOT GZ_SIM_FOUND)
  find_package(gz-sim7 QUIET)
  if(gz-sim7_FOUND)
    set(GZ_SIM_VER 7)
    set(GZ_TARGET gz-sim7::core)
    set(GZ_SIM_FOUND TRUE)
  endif()
endif()
if(NOT GZ_SIM_FOUND)
  find_package(ignition-gazebo6 QUIET)
  if(ignition-gazebo6_FOUND)
    set(GZ_SIM_VER 6)
    set(GZ_TARGET ignition-gazebo6::core)
    set(GZ_SIM_FOUND TRUE)
  endif()
endif()

set(GZ_PLUGIN_FOUND FALSE)
find_package(gz-plugin3 QUIET
  HINTS ${CMAKE_PREFIX_PATH}/opt/gz_plugin_vendor/extra_cmake/lib/cmake
        /opt/ros/${ROS_DISTRO}/opt/gz_plugin_vendor/extra_cmake/lib/cmake
)
if(gz-plugin3_FOUND)
  set(GZ_PLUGIN_TARGET gz-plugin3::register)
  set(GZ_PLUGIN_FOUND TRUE)
endif()
if(NOT GZ_PLUGIN_FOUND)
  find_package(gz-plugin2 QUIET)
  if(gz-plugin2_FOUND)
    set(GZ_PLUGIN_TARGET gz-plugin2::register)
    set(GZ_PLUGIN_FOUND TRUE)
  endif()
endif()
if(NOT GZ_PLUGIN_FOUND)
  find_package(ignition-plugin1 QUIET)
  if(ignition-plugin1_FOUND)
    set(GZ_PLUGIN_TARGET ignition-plugin1::register)
    set(GZ_PLUGIN_FOUND TRUE)
  endif()
endif()

# Keep this after Gazebo discovery. The Livox/PCL dependency chain may create
# JsonCpp imported targets that conflict with Ignition Fuel Tools on Humble.
find_package(livox_ros_driver2 REQUIRED)

if(GZ_SIM_FOUND AND GZ_PLUGIN_FOUND)
  add_library(livox_mid360_gz_sim_plugin SHARED src/livox_mid360_gz_plugin.cpp)
  target_include_directories(livox_mid360_gz_sim_plugin PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(livox_mid360_gz_sim_plugin ${GZ_TARGET} ${GZ_PLUGIN_TARGET})
  ament_target_dependencies(livox_mid360_gz_sim_plugin
    livox_ros_driver2 nav_msgs rclcpp sensor_msgs)
  install(TARGETS livox_mid360_gz_sim_plugin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)
  ament_export_libraries(livox_mid360_gz_sim_plugin)
else()
  message(WARNING "Gazebo Sim/plugin development packages not found; Mid-360 plugin will not be built")
endif()

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_livox_scan_timing test/test_livox_scan_timing.cpp)
  target_include_directories(test_livox_scan_timing PRIVATE include)
  add_test(NAME livox_scan_timing COMMAND test_livox_scan_timing)
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch urdf worlds DESTINATION share/${PROJECT_NAME})
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")
ament_export_include_directories(include)
ament_package()
