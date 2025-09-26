from launch import LaunchContext, LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    checkpoint_dir = PathJoinSubstitution(
        [FindPackageShare("darwin_robot"), "checkpoint", "cuda_dreamwaq_policy2.pt"]
    )
    joint_config_dir = PathJoinSubstitution(
        [FindPackageShare("darwin_robot"), "params", "joint_config.yaml"]
    )

    inference_node = Node(
        package="darwin_robot",
        executable="dreamwaq_inference_service",
        name="inference_service",
        output="screen",
        parameters=[
            {"num_history_steps": 20},
            {"checkpoint_dir": checkpoint_dir},
            {"debug": False},
        ],
    )

    openrb_serial = Node(
        package="darwin_robot",
        executable="openRB_traj.py",
        name="openRB",
        output="screen",
        parameters=[
            {"serial_port": "/dev/ttyACM0"},
            {"serial_baud": 921600},
            {"joint_config": joint_config_dir},
            {"behavior_config_dir": "/home/orin/humble_ws/src/darwin_robot/behaviors"},
            {"action_ratio": 0.1},
            {"zero_action": False},
        ],
    )

    lc = LaunchContext()
    joy_type = EnvironmentVariable("CPR_JOY_TYPE", default_value="ps5")

    filepath_config_joy = PathJoinSubstitution(
        [FindPackageShare("darwin_robot"), "params", ("teleop_" + joy_type.perform(lc) + ".yaml")]
    )

    node_joy = Node(
        package="joy",
        executable="joy_node",
        output="screen",
        name="joy_node",
        parameters=[filepath_config_joy],
    )

    node_teleop_twist_joy = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        output="screen",
        name="teleop_twist_joy_node",
        parameters=[filepath_config_joy],
    )
    ld = LaunchDescription()
    ld.add_action(inference_node)
    ld.add_action(openrb_serial)
    ld.add_action(node_joy)
    ld.add_action(node_teleop_twist_joy)
    return ld
