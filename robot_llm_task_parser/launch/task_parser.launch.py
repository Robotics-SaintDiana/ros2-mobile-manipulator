import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parser_share = get_package_share_directory('robot_llm_task_parser')
    task_manager_share = get_package_share_directory('robot_task_manager')

    return LaunchDescription([
        Node(
            package='robot_llm_task_parser',
            executable='task_parser_node',
            name='task_parser_node',
            output='screen',
            parameters=[
                os.path.join(parser_share, 'config', 'llm_params.yaml'),
                {
                    'use_sim_time': True,
                    'waypoints_file': os.path.join(task_manager_share, 'config', 'waypoints.json'),
                },
            ],
        ),
    ])

