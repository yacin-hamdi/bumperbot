from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from pathlib import Path
from os import pathsep
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    bumperbot_description_dir = get_package_share_directory("bumperbot_description")
    ros_distro = os.environ["ROS_DISTRO"]

    is_ignition = "True" if ros_distro == "humble" else "False"
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(bumperbot_description_dir, "urdf", "bumperbot_urdf.xacro"),
        description="absolute path to robot urdf file"
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name", 
        default_value="empty"
    )
    

    world_path = PathJoinSubstitution([
            bumperbot_description_dir, 
            "worlds", 
            PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
        ]
    )
    

    robot_description = ParameterValue(Command([
        "xacro ", LaunchConfiguration("model"),
        " is_ignition:=", is_ignition
        
        ]), value_type=str)

    robot_state_publisher = Node(
        package = "robot_state_publisher", 
        executable = "robot_state_publisher", 
        parameters = [{"robot_description": robot_description}]
    )

    
    model_path = str(Path(bumperbot_description_dir).parent.resolve())
    model_path += pathsep + os.path.join(bumperbot_description_dir, 'models')
    
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", 
        value=[ 
                model_path
            ]
    )
    # print(str(Path(bumperbot_description_dir).parent.resolve()))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
        os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
            launch_arguments={
                "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
            }.items()
                
            
            )
    
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description",
                   "-name", "bumperbot"]
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"
        ],
        remappings=[
            ("/imu", "/imu/out")
        ]
    )
    return LaunchDescription([
        model_arg, 
        world_name_arg,
        robot_state_publisher,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity, gz_ros2_bridge
    ])