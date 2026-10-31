from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    manipulator_server_node = Node(
        package="panda_manipulation",
        executable="manipulator_task_server",
        output="screen",
        parameters=[{
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        manipulator_server_node
    ])