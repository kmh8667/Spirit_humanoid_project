from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from pathlib import Path
from launch.actions import AppendEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="X position")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Y position")
    z_arg = DeclareLaunchArgument("z", default_value="0.35", description="Z position")
    yaw_arg = DeclareLaunchArgument("Y", default_value="0.0", description="Yaw")
    gz_resource_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
            "/usr/share/gazebo-11/models/:",
            str(Path(get_package_share_directory("darwin_robot")).parent.resolve()),
        ],
    )

    joint_state_controller_config = PathJoinSubstitution(
        [FindPackageShare("darwin_robot"), "params", "controller.yaml"]
    )

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("darwin_robot"), "urdf", "darwin.urdf.xacro"]),
            " ",
            "is_classic_gz:=true",
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": robot_description,
            }
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    darwin_robotler_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "-u",  # pause
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
            "--verbose",
        ],
        output="screen",
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_darwin",
        arguments=[
            "-entity",
            "darwin",
            "-topic",
            "robot_description",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
            "-Y",
            LaunchConfiguration("Y"),
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(yaw_arg)
    ld.add_action(gz_resource_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(darwin_robotler_spawner)

    return ld
