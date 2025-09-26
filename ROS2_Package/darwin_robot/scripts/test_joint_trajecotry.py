import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
import time
from builtin_interfaces.msg import Duration


class TestJointTrajectory(Node):
    def __init__(self):
        super().__init__("test_joint_trajectory_command")

        self.pub_trajectory = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        self.client = ActionClient(
            self, FollowJointTrajectory, "/joint_trajectory_controller/follow_joint_trajectory"
        )

    def pub_command(self, val=0.0):
        command = JointTrajectory()
        command.joint_names = [
            "j_pan",
            "j_pelvis_l",
            "j_pelvis_r",
            "j_shoulder_l",
            "j_shoulder_r",
            "j_tilt",
            "j_thigh1_l",
            "j_thigh1_r",
            "j_high_arm_l",
            "j_high_arm_r",
            "j_thigh2_l",
            "j_thigh2_r",
            "j_low_arm_l",
            "j_low_arm_r",
            "j_tibia_l",
            "j_tibia_r",
            "j_gripper_l",
            "j_gripper_r",
            "j_ankle1_l",
            "j_ankle1_r",
            "j_ankle2_l",
            "j_ankle2_r",
        ]
        point = JointTrajectoryPoint()
        point.positions = [0.0 for _ in range(22)]
        point.time_from_start = Duration(nanosec=5000)
        command.points.append(point)

        self.pub_trajectory.publish(command)

    def send_command(self):
        self.client.wait_for_server()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.2]
        point.time_from_start.sec = 2
        goal_msg.trajectory.points.append(point)

        self.get_logger().info("Sending goal...")
        self._send_goal_future = self.client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result received: {result.error_code}")
        rclpy.shutdown()


if __name__ == "__main__":
    rclpy.init()
    node = TestJointTrajectory()
    while rclpy.ok():
        node.pub_command()
        time.sleep(2)
