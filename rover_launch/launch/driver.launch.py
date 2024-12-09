import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    ld = LaunchDescription()
    
    # Nodes
    drive_core = Node(
        package="locomotion_core",
        executable="movebase_kinematics",
        )

    motor_driver = Node(
        package="locomotion_core",
        executable="cmd_roboteq",
    )

    

    ld.add_action(drive_core)
    ld.add_action(motor_driver)

    return ld


