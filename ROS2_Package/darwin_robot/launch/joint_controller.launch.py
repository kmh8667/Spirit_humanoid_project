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
    urdf_dir = PathJoinSubstitution([FindPackageShare("darwin_robot"), "urdf", "darwin.urdf"])

    inference_node = Node(
        package="darwin_robot",
        executable="torchscript",
        name="joint_controller",
        output="screen",
        parameters=[
            {"checkpoint_dir": checkpoint_dir},
            {"robot_description": urdf_dir},
            {"debug": False},
            {"imu_topic_name": "imu/data_raw"},
            {"cmd_vel_topic_name": "cmd_vel"},
            {"joint_state_topic_name": "joint_states"},
            {"odom_topic_name": "base_pose_ground_truth"},
            {"joint_command_topic_name": "joint_trajectory_controller/joint_trajectory"},
            {"trajectory_time": 0.05},
        ],
    )

    ld = LaunchDescription()
    ld.add_action(inference_node)
    return ld
