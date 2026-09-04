import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_scout_description = get_package_share_directory('scout_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 1. 动态设置 GZ 资源路径
    install_dir = os.path.dirname(pkg_scout_description)
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[install_dir]
    )

    # 2. XACRO 文件路径与解析
    xacro_file = os.path.join(pkg_scout_description, 'urdf', 'scout_mini.urdf.xacro')
    bridge_config_file = os.path.join(pkg_scout_description, 'config', 'gazebo_bridge.yaml')

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 3. RViz 配置文件路径
    # 注意：请根据实际存放路径修改，假设在 scout_description/rviz/urdf_config.rviz
    rviz_config_file = os.path.join(pkg_scout_description, 'rviz', 'urdf_config.rviz')

    # 4. 启动 Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 5. 在 Gazebo 中生成模型
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name', 'scout_mini',
            '-z', '0.5'
        ],
        output='screen'
    )

    # 6. 发布 /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # # 7. 发布关节状态（可选，解决 RViz 显示 TF 树缺失关节问题）
    # joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     output='screen'
    # )

    # # 8. 发布 TF 静态变换 (可选)
    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     arguments=['--frame-id', 'base_link', '--child-frame-id', 'base_footprint']
    # )

    # 9. ROS 2 <-> Gazebo 桥接节点 (添加了 /joint_states 桥接，方便 RViz 同步关节状态)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen'
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_config_file}'
        ],
        output='screen'
    )

    # 10. 启动 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        spawn_entity,
        robot_state_publisher,
        # joint_state_publisher_gui_node,
        # joint_state_publisher,
        # static_tf,
        bridge,
        rviz_node
    ])