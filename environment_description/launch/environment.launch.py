import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('environment_description')
    env_urdf_path = os.path.join(pkg_share, 'urdf', 'environment.urdf.xacro')

    environment_description = ParameterValue(
        Command(['xacro', ' ', env_urdf_path]),
        value_type=str
    )

    world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")
    world_path = PathJoinSubstitution([
                pkg_share,
                "worlds",
                PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
            ]
        )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
                launch_arguments={
                    "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
                }.items()
             )

    return LaunchDescription([

        # 1. 加载世界模型
        world_name_arg,
        gazebo,

        # 2. 建立 Nav2 的 map 坐标系与 Gazebo world 坐标系的桥梁
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
        ),

        # 3. 发布相机 TF 树
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='environment_state_publisher',
            parameters=[{
                'robot_description': environment_description,
                'use_sim_time': True
            }],
            remappings=[
                ('robot_description',
                 '/environment_description'),
                ('tf', '/tf'),
                ('tf_static', '/tf_static'),
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'environment',
                '-topic', '/environment_description',
                '-x', '0.0', '-y', '0.0', '-z', '0.0',
                '-R', '0.0', '-P', '0.0', '-Y', '0.0'
            ],
            output='screen'
        )
    ])