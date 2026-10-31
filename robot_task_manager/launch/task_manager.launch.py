import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share = get_package_share_directory('robot_task_manager')
    default_task = os.path.join(share, 'tasks', 'pick_and_place.json')
    default_waypoints = os.path.join(share, 'config', 'waypoints.json')

    return LaunchDescription([
        DeclareLaunchArgument('task_file', default_value=default_task),
        DeclareLaunchArgument('waypoints_file', default_value=default_waypoints),
        DeclareLaunchArgument('action_name', default_value='navigate_to_pose'),

        Node(
            package='robot_task_manager', executable='task_manager',
            name='task_manager_node', output='screen',
            parameters=[{
                'task_file': LaunchConfiguration('task_file'),
                'waypoints_file': LaunchConfiguration('waypoints_file'),
            }],
        ),
    ])
