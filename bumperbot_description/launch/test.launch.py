from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os



def generate_launch_description():
    
    robot_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory("bumperbot_description"), "urdf", "bumperbot_urdf.xacro"),
        description="absolute path to robot description"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type = str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2", 
        name="rviz2",
        output = "screen"
    )

    return LaunchDescription([
        robot_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])