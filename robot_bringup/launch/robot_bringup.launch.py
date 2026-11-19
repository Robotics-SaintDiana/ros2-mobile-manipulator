import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # ------------------- Gazebo -------------------
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_description"),
                "launch",
                "gazebo.launch.py"
            )
        )
    )

    # ------------------- Ros Gz Bridge -------------------
    # Lidar 的扫描数据的 frame_id 有问题，gazebo自动变更成了 /panda/base_footprint/laser，这里手动修正为 /laser_frame
    fix_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='fix_laser_tf_publisher',
        arguments=[
            '0', '0', '0',   
            '0', '0', '0',   
            'laser_frame',          
            'panda/base_footprint/laser' 
        ]
    )

    # ------------------- Environment Description -------------------
    environment_tf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("environment_description"),
                "launch",
                "environment.launch.py"
            )
        )
    )

    # ------------------- Controllers -------------------
    controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_controller"),
                "launch",
                "panda_controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ------------------- MoveIt Config -------------------
    moveit_config_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_moveit_config"),
                "launch",
                "moveit.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True"}.items()
    )

    # ------------------- Navigation -------------------
    navigation_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_navigation"),
                "launch",
                "navigation.launch.py"
            )
        ),
        launch_arguments={"use_sim_time": "True"}.items()
    )

    # ------------------- Manipulator Server Node -------------------
    manipulator_server_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_manipulation"),
                "launch",
                "manipulator_server.launch.py"
            )
        )
    )

    # ------------------- GraspNet Node -------------------
    graspnet_node = Node(
        package="panda_grasp_detection",
        executable="graspnet_executor_node",
        name="graspnet_executor_node",
        output="screen",
    )

    # ------------------- BT Manager Node -------------------
    bt_manager_node = Node(
        package="robot_bt_manager",
        executable="bt_manager_node",
        name="bt_manager_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # ------------------- Task Manager Node -------------------
    task_manager_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_task_manager"),
                "launch",
                "task_manager.launch.py"
            )
        )
    )

    # ------------------- LLM Task Parser Node -------------------
    # 启动 bringup 的终端需要先 source src/.env
    task_parser_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_llm_task_parser"),
                "launch",
                "task_parser.launch.py"
            )
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        gazebo_node,
        fix_laser_tf_node,
        environment_tf_node,
        controller_node,
        navigation_node,
        moveit_config_node,
        manipulator_server_node,
        graspnet_node,
        bt_manager_node,
        task_manager_node,
        task_parser_node
    ])