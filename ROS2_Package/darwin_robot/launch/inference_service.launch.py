from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    checkpoint_dir = PathJoinSubstitution(
        [FindPackageShare("darwin_robot"), "checkpoint", "policy.pt"]
    )

    inference_node = Node(
        package="darwin_robot",
        executable="inference_service",
        name="inference_service",
        output="screen",
        parameters=[
            {"checkpoint_dir": checkpoint_dir},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(inference_node)
    return ld
