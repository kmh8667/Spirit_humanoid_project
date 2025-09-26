from isaacsim import SimulationApp

# Launch Isaac Sim in headless=False for GUI; set to True for headless
simulation_app = SimulationApp({"headless": False})

import yaml
import carb
import numpy as np
import time
import re
import omni.graph.core as og
import omni.kit.commands
from pxr import Gf
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.api import World, SimulationContext
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.prims import define_prim, get_prim_at_path
from isaacsim.storage.native import get_assets_root_path
from isaacsim.sensors.physics import IMUSensor

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()


class SteadyRate:
    """Maintains the steady cycle rate provided on initialization by adaptively sleeping an amount
    of time to make up the remaining cycle time after work is done.

    Usage:

    rate = SteadyRate(rate_hz=60.)
    while True:
      app.update() # render/app update call here
      rate.sleep()  # Sleep for the remaining cycle time.

    """

    def __init__(self, rate_hz):
        self.rate_hz = rate_hz
        self.dt = 1.0 / rate_hz
        self.last_sleep_end = time.time()

    def sleep(self):
        work_elapse = time.time() - self.last_sleep_end
        sleep_time = self.dt - work_elapse
        if sleep_time > 0.0:
            time.sleep(sleep_time)
        self.last_sleep_end = time.time()


rate = SteadyRate(1 / 0.005)

import rclpy
from rosgraph_msgs.msg import Clock

rclpy.init()


world = World(physics_dt=0.005, rendering_dt=0.02, stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
set_camera_view(
    eye=[5.0, 0.0, 1.5], target=[0.00, 0.00, 1.00], camera_prim_path="/OmniverseKit_Persp"
)  # set camera view

asset_path = "/home/sungwoo/Documents/darwin_description/urdf/darwin/darwin.usd"

prim = define_prim(prim_path="/World/darwin", prim_type="Xform")
prim.GetReferences().AddReference(asset_path)
darwin = Articulation(prim_paths_expr="/World/darwin", name="darwin")
darwin.set_world_poses(positions=np.array([[0.0, 0.0, 0.4]]) / get_stage_units())

world.reset()
darwin.initialize()

time.sleep(1)

print("Loaded", str(len(darwin.dof_names)), "dofs")
print(darwin.dof_names)
imu = IMUSensor(
    prim_path="/World/darwin/base_link/MP_BODY/IMU_LINK/Imu_Sensor",
    name="Imu_Sensor",
    frequency=50,  # or, dt=1./60
    translation=np.array([0, 0, 0]),  # or, position=np.array([0, 0, 0]),
    orientation=np.array([1, 0, 0, 0]),
    linear_acceleration_filter_size=10,
    angular_velocity_filter_size=10,
    orientation_filter_size=10,
)
imu.initialize()
simulation_app.update()

joint_pos = {
    "j_pan": 0.0,
    "j_tilt": 0.0,
    "j_pelvis_l": 0.0,
    "j_thigh1_l": 0.0,
    "j_thigh2_l": 0.4,
    "j_tibia_l": -0.6,
    "j_ankle1_l": -0.2,
    "j_ankle2_l": 0.2,
    "j_pelvis_r": 0.0,
    "j_thigh1_r": 0.0,
    "j_thigh2_r": -0.4,
    "j_tibia_r": 0.6,
    "j_ankle1_r": 0.0,
    "j_ankle2_r": 0.0,
    "j_shoulder_l": 0.0,
    "j_high_arm_l": 0.7,
    "j_low_arm_l": -1.0,
    "j_shoulder_r": 0.0,
    "j_high_arm_r": 0.7,
    "j_low_arm_r": 1.0,
    "j_gripper_l": -0.99,
    "j_gripper_r": -0.99,
}
stiffness = {
    "j_pelvis_.*": 150.0,
    "j_thigh1_.*": 150.0,
    "j_thigh2_.*": 200.0,
    "j_tibia_.*": 200.0,
    "j_ankle1_.*": 20.0,
    "j_ankle2_.*": 20.0,
    "j_shoulder_.*": 40.0,
    "j_high_arm_.*": 40.0,
    "j_low_arm_.*": 40.0,
    "j_gripper_.*": 40.0,
    "j_pan": 20.0,
    "j_tilt": 20.0,
}

damping = {
    "j_pelvis_.*": 5.0,
    "j_thigh1_.*": 5.0,
    "j_thigh2_.*": 5.0,
    "j_tibia_.*": 5.0,
    "j_ankle1_.*": 4.0,
    "j_ankle2_.*": 4.0,
    "j_shoulder_.*": 10.0,
    "j_high_arm_.*": 10.0,
    "j_low_arm_.*": 10.0,
    "j_gripper_.*": 10.0,
    "j_pan": 4.0,
    "j_tilt": 4.0,
}


def get_param_value(name, param_dict):
    for pattern, value in param_dict.items():
        if re.fullmatch(pattern, name):
            return value
    return 0.0


default_pos_inorder, stiffness_inorder, damping_inorder = [], [], []
for name in darwin.dof_names:
    default_pos_inorder.append(get_param_value(name, joint_pos))
    stiffness_inorder.append(get_param_value(name, stiffness))
    damping_inorder.append(get_param_value(name, damping))

darwin.set_joints_default_state(positions=default_pos_inorder)
print()
world.reset()

time.sleep(1)

# Build Action Graph
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublishRawTFTree", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("PublishTFTree", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
            ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
        ],
        og.Controller.Keys.CONNECT: [
            # Publish Joint State
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("Context.outputs:context", "PublishJointState.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            # Subscibe Joint State
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("Context.outputs:context", "SubscribeJointState.inputs:context"),
            # Subscibe Joint State -> Articulation Controller
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            (
                "SubscribeJointState.outputs:positionCommand",
                "ArticulationController.inputs:positionCommand",
            ),
            ("SubscribeJointState.outputs:execOut", "ArticulationController.inputs:execIn"),
            # Publish Clock
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            # Compute Odometry
            ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            # Publish Odom
            ("Context.outputs:context", "PublishOdom.inputs:context"),
            ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
            ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
            ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
            # Publish Raw TF
            ("Context.outputs:context", "PublishRawTFTree.inputs:context"),
            ("ComputeOdom.outputs:execOut", "PublishRawTFTree.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishRawTFTree.inputs:timeStamp"),
            ("ComputeOdom.outputs:orientation", "PublishRawTFTree.inputs:rotation"),
            ("ComputeOdom.outputs:position", "PublishRawTFTree.inputs:translation"),
            # Publish TF
            ("OnPlaybackTick.outputs:tick", "PublishTFTree.inputs:execIn"),
            ("Context.outputs:context", "PublishTFTree.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishTFTree.inputs:timeStamp"),
            # Read IMU
            ("OnPlaybackTick.outputs:tick", "ReadIMU.inputs:execIn"),
            # Publish IMU
            ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
            ("Context.outputs:context", "PublishIMU.inputs:context"),
            ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
            ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
            ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
            ("ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
        ],
        og.Controller.Keys.SET_VALUES: [
            # Publish Joint State
            ("PublishJointState.inputs:targetPrim", "/World/darwin/base_link"),
            ("PublishJointState.inputs:topicName", "joint_states"),
            # Subscirbe Joint State
            ("SubscribeJointState.inputs:topicName", "joint_command"),
            # Articulation Controller
            ("ArticulationController.inputs:targetPrim", "/World/darwin"),
            # Publish Clock
            ("PublishClock.inputs:topicName", "clock"),
            # Compute Odometry
            ("ComputeOdom.inputs:chassisPrim", "/World/darwin/base_link"),
            # Publish Odometry
            ("PublishOdom.inputs:chassisFrameId", "base_link"),
            ("PublishOdom.inputs:odomFrameId", "odom"),
            ("PublishOdom.inputs:topicName", "odom"),
            # Publish Raw TF
            ("PublishRawTFTree.inputs:childFrameId", "base_link"),
            ("PublishRawTFTree.inputs:parentFrameId", "odom"),
            ("PublishRawTFTree.inputs:topicName", "tf"),
            # Publish TF
            ("PublishTFTree.inputs:parentPrim", "/World/darwin/base_link"),
            ("PublishTFTree.inputs:targetPrims", "/World/darwin"),
            ("PublishTFTree.inputs:topicName", "tf"),
            # Read IMU
            ("ReadIMU.inputs:imuPrim", "/World/darwin/base_link/MP_BODY/IMU_LINK/Imu_Sensor"),
            ("ReadIMU.inputs:readGravity", True),
            # Publish IMU
            ("PublishIMU.inputs:frameId", "IMU_LINK"),
            ("PublishIMU.inputs:topicName", "imu"),
        ],
    },
)


time.sleep(1)
world.reset()

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
