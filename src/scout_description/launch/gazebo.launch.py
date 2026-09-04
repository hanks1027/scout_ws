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

    # 1. 动态设置 GZ 资源路径（自动检索源码 models 目录和系统原有路径）
    install_dir = os.path.dirname(pkg_scout_description)
    models_dir = os.path.expanduser('~/scout_ws/src/scout_description/models')
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    gz_resource_paths = [install_dir, models_dir]
    if existing_gz_path:
        gz_resource_paths.append(existing_gz_path)
        
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join(gz_resource_paths)
    )

    # # # 1. 动态设置 GZ 资源路径
    # install_dir = os.path.dirname(pkg_scout_description)
    # set_gz_resource_path = SetEnvironmentVariable(
    #     name='GZ_SIM_RESOURCE_PATH',
    #     value=[install_dir]
    # )

    # 2. XACRO 文件路径与解析
    xacro_file = os.path.join(pkg_scout_description, 'urdf', 'scout_mini.urdf.xacro')
    bridge_config_file = os.path.join(pkg_scout_description, 'config', 'gazebo_bridge.yaml')
    world_file = os.path.join(pkg_scout_description, 'worlds', 'house.sdf.world')

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 3. RViz 配置文件路径
    # 注意：请根据实际存放路径修改，假设在 scout_description/rviz/urdf_config.rviz
    rviz_config_file = os.path.join(pkg_scout_description, 'rviz', 'urdf_config.rviz')

    # # 4. 启动 Gazebo Sim
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
    #     ),
    #     launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    # )

    # 4. 启动 Gazebo Sim，加载 house.world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 5. 在 Gazebo 中生成模型
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name', 'scout_mini',
            '-x', '2.0',   # 避开中央密集家具
            '-y', '0.0',
            '-z', '1.0'    # 从 1 米高度落下
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


    # 9. ROS 2 <-> Gazebo 桥接节点 (添加了 /joint_states 桥接，方便 RViz 同步关节状态)

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
        bridge,
        rviz_node
    ])