import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Bool
from hri_humanoid_interfaces.srv import GetActions
import time


class TestInferenceService(Node):
    def __init__(self):
        super().__init__("test_inference_service")

        self.client = self.create_client(GetActions, "get_actions")
        self.timer = self.create_timer(0.02, self.call)
        self.t = time.time()

    def call(self, val=0.0):
        req = GetActions.Request()
        req.observations = [0.2] * 71

        future = self.client.call_async(req)
        future.add_done_callback(self.callback)

    def callback(self, future):
        print(1 / (time.time() - self.t))
        self.t = time.time()


if __name__ == "__main__":
    rclpy.init()
    node = TestInferenceService()
    rclpy.spin(node)

    rclpy.shutdown()
