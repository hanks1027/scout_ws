#!/bin/bash
#启动USB相机模块
gnome-terminal --title="usb_camera" -- bash -c "ros2 run v4l2_camera v4l2_camera_node; exec bash"
sleep 2
#启动底盘控制节点
gnome-terminal --title="scout_base" -- bash -c "ros2 launch scout_base scout_mini_base.launch.py; exec bash"

#延时2秒
sleep 2
#启动激光雷达节点
gnome-terminal --title="livox_ros_driver2" -- bash -c "ros2 launch livox_ros_driver2 msg_MID360s_launch.py; exec bash"

# 延时2秒
#sleep 2

#启动fastlio2
#gnome-terminal --title="fast_lio" -- bash -c "ros2 launch fast_lio mapping.launch.py; exec bash"
sleep 2
#启动回环检测（待改进）
gnome-terminal --title="pgo" -- bash -c "ros2 launch pgo pgo_launch.py; exec bash"

#sleep 2
#绑定map->odom
#gnome-terminal --title="map_to_odom" -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0.21 0 0 0 map odom; exec bash"

sleep 2
gnome-terminal --title="pointcloud_to_laserscan2" -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 body base_link; exec bash"

#绑定base_link->base_footprint
sleep 2
gnome-terminal --title="base_link_to_base_footprint" -- bash -c "ros2 run scout_description base_footprint_publisher; exec bash"

sleep 2
#启动3d点云转2d节点
gnome-terminal --title="pointcloud_to_laserscan" -- bash -c "ros2 launch pointcloud_to_laserscan mid360s_pointcloud_to_laserscan.launch.py; exec bash"
#启动map_server加载地图
sleep 2
gnome-terminal --title="slam_toolbox" -- bash -c "ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/hanks/scout_ws/src/map/mymap2.yaml -p use_sim_time:=false; exec bash"
sleep 2
#激活map_server发布地图
gnome-terminal --title="slam_toolbox" -- bash -c "ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p node_names:="['map_server']" -p autostart:=true; exec bash"
#启动nav2节点
sleep 2
gnome-terminal --title="nav2_bringup" -- bash -c "ros2 launch nav2_bringup bringup_launch.py use_localization:=False map:=/home/hanks/scout_ws/src/map/mymap2.yaml params_file:=/home/hanks/scout_ws/src/scout_description/config/nav2_params.yaml; exec bash"


