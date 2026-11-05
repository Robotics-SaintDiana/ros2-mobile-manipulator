import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    robot_description = get_package_share_directory("robot_description")

    model_arg = DeclareLaunchArgument(
        name="model", default_value=os.path.join(
                robot_description, "urdf", "panda.urdf.xacro"
            ),
        description="Absolute path to robot urdf file"
    )

    model_path = str(Path(robot_description).parent.resolve())
    model_path += pathsep + os.path.join(get_package_share_directory("robot_description"), 'models')

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
        )

    robot_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("model"),
        ]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "panda",
            "-x", "0.0",  
            "-y", "4.7",  
            "-z", "0.15",  
            "-R", "0.0", 
            "-P", "0.0",
            "-Y", "1.57",
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # --- 公共时钟 ---
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

            # --- 底盘关节状态 ---
            "/world/empty_world/model/panda/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model",

            # --- odom TF ---
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",

            # --- 激光雷达 ---
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",

            # --- 速度控制 ---
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",

            # --- 抓取固定相机 (pick_camera) ---
            "/pick_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            '/pick_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            "/pick_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",

            # --- 放置固定相机 (place_camera) ---
            "/place_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            '/place_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            "/place_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",

            # --- 机器人手部固定相机 (hand_camera) ---
            "/hand_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            '/hand_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            "/hand_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
        ],
        remappings=[
            # --- 底盘关节状态 ---
            ("/world/empty_world/model/panda/joint_state", "joint_states"),

            # --- 抓取固定相机重映射 ---
            ("/pick_camera/depth_image", "/pick_camera/depth/image_raw"),
            ("/pick_camera/image", "/pick_camera/color/image_raw"),

            # --- 放置固定相机重映射 ---
            ("/place_camera/depth_image", "/place_camera/depth/image_raw"),
            ("/place_camera/image", "/place_camera/color/image_raw"),

            # --- 手部固定相机重映射 ---
            ("/hand_camera/depth_image", "/hand_camera/depth/image_raw"),
            ("/hand_camera/image", "/hand_camera/color/image_raw"),
        ],
        output='screen'
    )

    point_cloud_node = ComposableNodeContainer(
        name='depth_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # --- 抓取固定相机点云生成节点 ---
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',  
                name='pick_camera_point_cloud_xyzrgb_node',
                parameters=[{
                    'use_sim_time': True,  
                    'exact_sync': False, 
                    'queue_size': 30
                }],
                remappings=[
                    ('rgb/image_rect_color', '/pick_camera/color/image_raw'),
                    ('rgb/camera_info', '/pick_camera/camera_info'),
                    ('depth_registered/image_rect', '/pick_camera/depth/image_raw'),
                    ('points', '/pick_camera/depth/color/points')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # --- 放置固定相机点云生成节点 ---
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',  
                name='place_camera_point_cloud_xyzrgb_node',
                parameters=[{
                    'use_sim_time': True,  
                    'exact_sync': False, 
                    'queue_size': 30
                }],
                remappings=[
                    ('rgb/image_rect_color', '/place_camera/color/image_raw'),
                    ('rgb/camera_info', '/place_camera/camera_info'),
                    ('depth_registered/image_rect', '/place_camera/depth/image_raw'),
                    ('points', '/place_camera/depth/color/points')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),

            # --- 手部固定相机点云生成节点 ---
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',  
                name='hand_camera_point_cloud_xyzrgb_node',
                parameters=[{
                    'use_sim_time': True,  
                    'exact_sync': False, 
                    'queue_size': 30
                }],
                remappings=[
                    ('rgb/image_rect_color', '/hand_camera/color/image_raw'),
                    ('rgb/camera_info', '/hand_camera/camera_info'),
                    ('depth_registered/image_rect', '/hand_camera/depth/image_raw'),
                    ('points', '/hand_camera/depth/color/points')
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
        output='screen',
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher_node,
        gz_spawn_entity,
        gz_ros2_bridge,
        point_cloud_node
    ])