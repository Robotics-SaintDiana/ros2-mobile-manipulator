from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller", "--controller-manager", "/controller_manager"],
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription(
        [
            panda_arm_controller_spawner,
            hand_controller_spawner,
        ]
    )