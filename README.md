设置can波特率：
sudo ip link set can0 up type can bitrate 500000
candump can0

source install setup.bash

查看scout mini的urdf模型显示：
ros2 launch urdf_tutorial display.launch.py model:=/home/hanks/scout_ws/src/scout_description/urdf/scout_mini.urdf 


实机部署：
ros2 launch scout_base scout_mini_base.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
开灯光：
ros2 topic pub --once /light_control scout_msgs/msg/ScoutLightCmd "{
  cmd_ctrl_allowed: true,
  front_mode: 1,
  front_custom_value: 0,
  rear_mode: 1,
  rear_custom_value: 0
}"


ros2 launch livox_ros_driver2 rviz_MID360s_launch.py

ros2 launch livox_ros_driver2 msg_MID360s_launch.py

ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro scout_mini.xacro)"
ros2 launch ros_gz_sim gz_sim.launch.py gz_arg:="empty.sdf -r"
ros2 run ros_gz_sim create -topic robot_description



