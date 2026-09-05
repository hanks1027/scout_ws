# Scout Mini ROS 2 项目指南

本项目是基于 **ROS 2 Jazzy**（Ubuntu 24.04）开发的 **Scout Mini 移动机器人** 开源部署框架。项目整合了 Scout Mini 底盘驱动与 CAN 通信、机器人 URDF 模型可视化、Gazebo 仿真环境搭建，并深度适配了 **Livox Mid-360S** 激光雷达。同时，项目对 **FAST_LIO2** 3D LiDAR-IMU 紧耦合激光里程计与建图算法进行了 ROS 2 Jazzy 版本的编译适配与调优，提供了一套从硬件通信、实机遥控到高精度 3D SLAM 建图的一站式保姆级部署指南。 

---

## 🛠️ 1. 环境准备与 CAN 通信设置 
在连接机器人硬件前，需配置系统的 CAN 总线波特率并验证通信：


### 设置 CAN0 波特率为 500k 并启动接口
sudo ip link set can0 up type can bitrate 500000

### 监听 CAN0 数据流（用于测试硬件通信是否正常）
candump can0

### 编译工作空间后，在每个新的终端窗口中加载环境变量：
source install/setup.bash

## 📸 2. 模型可视化 (URDF) 
在 RViz 中查看 Scout Mini 的机器人模型结构：
### 查看 URDF 模型
ros2 launch urdf_tutorial display.launch.py model:=/home/hanks/scout_ws/src/scout_description/urdf/scout_mini.urdf.xacro

ros2 launch scout_description display_mini_models.launch.py

## 🚀 3. 实机部署与控制
### 3.1 启动底盘与键盘遥控
分别在不同终端运行以下命令：
####  终端 1：启动 Scout Mini 底盘驱动节点
ros2 launch scout_base scout_mini_base.launch.py

#### 终端 2：启动键盘遥控节点
ros2 run teleop_twist_keyboard teleop_twist_keyboard

### 3.2 机器灯光控制
#### 向 /light_control 话题发送单次控制指令以打开灯光：
ros2 topic pub --once /light_control scout_msgs/msg/ScoutLightCmd "{
  cmd_ctrl_allowed: true,
  front_mode: 1,
  front_custom_value: 0,
  rear_mode: 1,
  rear_custom_value: 0
}"

## 📡 4. 传感器驱动 (Livox MID360)
### 启动 Livox 雷达驱动并同步打开 RViz 可视化
ros2 launch livox_ros_driver2 rviz_MID360s_launch.py

### 仅发布 Livox 雷达自定义消息格式 (CustomMsg)
ros2 launch livox_ros_driver2 msg_MID360s_launch.py

## 🎮 5. Gazebo 仿真
ros2 launch scout_description gazebo.launch.py
## 发布速度
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

## 图形化界面控制
python3 scout_gui_control.py

## 6.fastlio2建图 (已兼容ROS2 jazzy&mid360s)
分别在两个终端输入命令
####  终端 1：启动mid360s节点
ros2 launch livox_ros_driver2 msg_MID360s_launch.py 

#### 终端 2：启动FASTLIO节点
ros2 launch fast_lio mapping.launch.py 

---
### 💡 上传仓库命令：
git add . && git commit -m "日常代码与文档更新" && git push



