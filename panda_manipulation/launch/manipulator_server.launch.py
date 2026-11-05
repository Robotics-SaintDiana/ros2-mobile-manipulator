import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory('robot_description'), 'urdf', 'panda.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    kinematics_path = os.path.join(
        get_package_share_directory('panda_moveit_config'), 'config', 'kinematics.yaml')
    with open(kinematics_path, 'r') as f:
        kinematics = yaml.safe_load(f)
    
    manipulator_server_node = Node(
        package="panda_manipulation",
        executable="manipulator_task_server",
        output="screen",
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
            'robot_description_kinematics': kinematics,
        }]
    )

    return LaunchDescription([manipulator_server_node])