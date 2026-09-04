import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_path = get_package_share_directory('scout_description')
    
    # 1. 解析 Xacro 模型文件
    xacro_file = os.path.join(pkg_path, 'urdf', 'scout_mini.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # 2. RViz 配置文件路径
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'urdf_config.rviz')

    # 3. 定义 ROS 2 节点
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
