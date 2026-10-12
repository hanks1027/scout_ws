#!/bin/bash
#启动USB相机模块
gnome-terminal --title="usb_camera" -- bash -c "sudo ip link set can0 up type can bitrate 500000; exec bash"
sleep 5
#启动底盘控制节点
gnome-terminal --title="scout_base" -- bash -c "ros2 launch scout_base scout_mini_base.launch.py; exec bash"

#延时2秒
sleep 2

#启动激光雷达节点
gnome-terminal --title="livox_ros_driver2" -- bash -c "ros2 launch livox_ros_driver2 msg_MID360s_launch.py; exec bash"

# 延时2秒
sleep 2

#启动fastlio2
gnome-terminal --title="fast_lio" -- bash -c "ros2 launch fastlio2 lio_launch.py; exec bash"
sleep 2
#启动回环检测（待改进）
#gnome-terminal --title="pgo" -- bash -c "ros2 launch pgo pgo_launch.py; exec bash"
sleep 2
#启动3d点云转2d节点
#gnome-terminal --title="pointcloud_to_laserscan" -- bash -c "ros2 launch scan_planner rviz.launch.py; exec bash"

#启动nav2节点
sleep 2
gnome-terminal --title="nav2_bringup" -- bash -c "ros2 launch scan_planner run.launch.py   is_real_world:=true navi_mode:=1 sensor_type:=lidar   controller_mode:=closed_loop use_gpu:=false; exec bash"


