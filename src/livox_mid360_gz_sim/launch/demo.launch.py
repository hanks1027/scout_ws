"""Launch a minimal stationary Mid-360 Gazebo Sim example."""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("livox_mid360_gz_sim")
    world = os.path.join(share, "worlds", "demo.sdf")
    model = os.path.join(share, "urdf", "demo.urdf.xacro")
    simulator = ["gz", "sim"] if shutil.which("gz") else ["ign", "gazebo"]
    description = ParameterValue(Command(["xacro ", model]), value_type=str)

    return LaunchDescription([
        ExecuteProcess(cmd=[*simulator, "-r", world], output="screen"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": description, "use_sim_time": True}],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=["-name", "mid360_demo", "-topic", "robot_description", "-z", "0.2"],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            output="screen",
        ),
    ])
