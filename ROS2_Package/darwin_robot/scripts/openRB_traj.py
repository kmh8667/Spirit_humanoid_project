#!/usr/bin/env python3

"""
Combined OpenRB-150 trajectory controller and ROS2 node.
Merges functionality from openRB.py and trajectory_controller.py.
Includes operation mode selection for different behaviors.
"""

import rclpy
import serial
import threading
import queue
import numpy as np
import time
import struct
import os
import sys
import yaml
from collections import deque

# Optional scipy import for cubic splines
try:
    from scipy import interpolate
except ImportError:
    interpolate = None

# Optional matplotlib import for plotting
try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None

from packet_handler import *
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, Bool
from hri_humanoid_interfaces.srv import GetActions


class OpenRB150(Node):
    def __init__(self):
        super().__init__("openrb150_trajectory")

        # Load config
        self.traj_points = [[]]
        self.load_joint_config_from_yaml()

        # Set default positions inorder
        self.default_pos_inorder = [0.0 for _ in range(22)]
        for name in self.sim_joint_names:
            if name in self.motor_index:
                self.default_pos_inorder[self.motor_index[name] - 1] = self.default_joint_pos[name]

        # Create UART connection with OpenRB
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("serial_baud", 921600)
        serial_port = self.get_parameter("serial_port").value
        serial_baud = self.get_parameter("serial_baud").value
        self.port = serial.Serial(serial_port, serial_baud)

        # Create Publishers
        self.pub_imu = self.create_publisher(Imu, "imu", 10)
        self.pub_joint_state = self.create_publisher(JointState, "joint_states", 10)

        # Service interface for ML inference
        self.client_inference = self.create_client(GetActions, "get_actions")
        self.action_log = open("action_log.txt", "w")
        self.obs_log = open("obs_log.txt", "w")
        # Behavior ID setup
        self.behavior_id = 0
        self.sub_behavior_id = self.create_subscription(
            Int8, "/behavior_id", self.cb_behavior_id, 10
        )
        self.num_behavior_id = len(self.traj_id) + 1

        self.declare_parameter(
            "behavior_config_dir", "/home/orin/humble_ws/src/darwin_robot/behaviors/"
        )
        self.behavior_config_dir = self.get_parameter("behavior_config_dir").value
        self.load_trajectories()

        self.declare_parameter("action_ratio", 0.25)
        self.action_ratio = self.get_parameter("action_ratio").value

        self.declare_parameter("zero_action", False)
        self.zero_action = self.get_parameter("zero_action").value
        if self.zero_action:
            self.get_logger().info("Ignoring Inferenced Actions for debug")
        else:
            self.get_logger().info(f"Accepting actions with ratio: {self.action_ratio}")

        # Trajectory control variables
        self.trajectory_active = False
        self.trajectory_waypoints = []
        self.trajectory_interpolated = []
        self.trajectory_start_time = 0
        self.trajectory_dt = 0.02  # 50Hz = 0.02 seconds
        self.interpolation_method = "linear"  # "linear" or "cubic"
        self.transition_time = 1.0

        # Data queues and threads
        serial_thread = threading.Thread(target=self.read_and_publish, daemon=True)
        self.q_pos = queue.Queue()
        self.q_vel = queue.Queue()
        self.q_imu = queue.Queue()

        # Cache for last known values to avoid blocking
        self.last_imu = np.zeros(10)
        self.last_joint_pos = np.zeros(18)
        self.last_joint_vel = np.zeros(18)
        self.current_joint_pos = np.zeros(22)  # Current positions in sim order
        self.current_joint_pos_raw = np.zeros(18)  # Raw motor positions

        # States
        self.torque_enabled = True
        self.sub_torque_enable = self.create_subscription(
            Bool, "/torque_enable", self.cb_torque_enable, 10
        )
        self.op_mod = 0
        self.sub_op_mod = self.create_subscription(Int8, "/op_mod", self.cb_op_mod, 10)

        # Data logging for plotting
        self.actual_positions_log = []
        self.logging_active = False
        self.log_start_time = 0

        # Start threads and timers
        serial_thread.start()
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("OpenRB-150 Trajectory Controller initialized")

    def load_joint_config_from_yaml(self):
        """Load joint configuration from YAML file specified by ROS parameter"""
        # Declare parameter for YAML config file path
        self.declare_parameter("joint_config", "./joint_config.yaml")
        config_file_path = self.get_parameter("joint_config").value

        try:
            with open(config_file_path, "r") as file:
                config = yaml.safe_load(file)

            # Load configurations from YAML
            self.motor_index = config.get("motor_index", {})
            self.sim_joint_names = config.get("sim_joint_names", [])
            self.default_joint_pos = config.get("default_joint_pos", {})

            # Convert joint_pos_clip from list format to tuple format
            joint_pos_clip_dict = config.get("joint_pos_clip", {})
            self.joint_pos_clip = {}
            for joint_name, limits in joint_pos_clip_dict.items():
                if isinstance(limits, list) and len(limits) == 2:
                    self.joint_pos_clip[joint_name] = (limits[0], limits[1])
                else:
                    self.get_logger().warn(f"Invalid joint limits for {joint_name}: {limits}")

            # Load trajectory behaviors
            self.traj_id = config.get("trajectory_behaviors", {})

            # Log loaded configuration
            self.get_logger().info(f"Loaded joint config from: {config_file_path}")
            self.get_logger().info(f"Loaded {len(self.motor_index)} motor indices")
            self.get_logger().info(f"Loaded {len(self.sim_joint_names)} sim joint names")
            self.get_logger().info(f"Loaded {len(self.default_joint_pos)} default positions")
            self.get_logger().info(f"Loaded {len(self.joint_pos_clip)} joint limits")

        except FileNotFoundError:
            self.get_logger().error(f"Joint config file not found: {config_file_path}")
            self.get_logger().error("Using fallback default configuration")
            self._load_fallback_config()
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file {config_file_path}: {e}")
            self.get_logger().error("Using fallback default configuration")
            self._load_fallback_config()
        except Exception as e:
            self.get_logger().error(f"Unexpected error loading config: {e}")
            self.get_logger().error("Using fallback default configuration")
            self._load_fallback_config()

    def _load_fallback_config(self):
        """Load hardcoded fallback configuration if YAML loading fails"""
        self.motor_index = {
            "j_pan": 19,
            "j_pelvis_l": 8,
            "j_pelvis_r": 7,
            "j_shoulder_l": 2,
            "j_shoulder_r": 1,
            "j_tilt": 20,
            "j_thigh1_l": 10,
            "j_thigh1_r": 9,
            "j_high_arm_l": 4,
            "j_high_arm_r": 3,
            "j_thigh2_l": 12,
            "j_thigh2_r": 11,
            "j_low_arm_l": 6,
            "j_low_arm_r": 5,
            "j_tibia_l": 14,
            "j_tibia_r": 13,
            "j_gripper_l": 22,
            "j_gripper_r": 21,
            "j_ankle1_l": 16,
            "j_ankle1_r": 15,
            "j_ankle2_l": 18,
            "j_ankle2_r": 17,
        }

        self.sim_joint_names = [
            "j_pelvis_l",
            "j_pelvis_r",
            "j_shoulder_l",
            "j_shoulder_r",
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
            "j_ankle1_l",
            "j_ankle1_r",
            "j_ankle2_l",
            "j_ankle2_r",
            "j_pan",
            "j_tilt",
            "j_gripper_r",
            "j_gripper_l",
        ]

        self.default_joint_pos = {
            "j_pan": 0.0,
            "j_tilt": 0.0,
            "j_pelvis_l": 0.0,
            "j_thigh1_l": 0.0,
            "j_thigh2_l": 0.4,
            "j_tibia_l": -0.6,
            "j_ankle1_l": -0.2,
            "j_ankle2_l": 0.0,
            "j_pelvis_r": 0.0,
            "j_thigh1_r": 0.0,
            "j_thigh2_r": -0.4,
            "j_tibia_r": 0.6,
            "j_ankle1_r": 0.2,
            "j_ankle2_r": 0.0,
            "j_shoulder_l": 0.0,
            "j_high_arm_l": 0.7,
            "j_low_arm_l": -1.0,
            "j_gripper_l": -0.99,
            "j_shoulder_r": 0.0,
            "j_high_arm_r": 0.7,
            "j_low_arm_r": 1.0,
            "j_gripper_r": -0.99,
        }

        self.joint_pos_clip = {
            "j_pelvis_l": (-0.7900, 2.6200),
            "j_pelvis_r": (-2.6200, 0.7900),
            "j_shoulder_l": (-4.3600, 4.3600),
            "j_shoulder_r": (-4.3600, 4.3600),
            "j_thigh1_l": (-1.0500, 0.0000),
            "j_thigh1_r": (0.0000, 1.0500),
            "j_high_arm_l": (-1.7500, 1.5800),
            "j_high_arm_r": (-1.7500, 1.5800),
            "j_thigh2_l": (-0.5200, 1.7500),
            "j_thigh2_r": (-1.7500, 0.5200),
            "j_low_arm_l": (-2.6180, 0.0000),
            "j_low_arm_r": (0.0, 2.6180),
            "j_tibia_l": (-2.2700, 0.0000),
            "j_tibia_r": (0.0000, 2.2700),
            "j_ankle1_l": (-1.0500, 1.0500),
            "j_ankle1_r": (-1.0500, 1.0500),
            "j_ankle2_l": (-0.5200, 1.0500),
            "j_ankle2_r": (-0.5200, 1.0500),
        }

        self.traj_id = {
            1: "hello",
            2: "crouch",
            3: "stretch",
            4: "greet",
            5: "foot",
            6: "step_back",
            7: "shake_body",
        }

    def load_trajectories(self):
        """Load trajectory files for each behavior"""
        self.get_logger().info("Loading trajectory from: " + self.behavior_config_dir)
        self.get_logger().info(
            f"Found {len([f for f in os.listdir(self.behavior_config_dir) if os.path.isfile(os.path.join(self.behavior_config_dir, f))])} trajectories"
        )
        for i in range(1, len(self.traj_id) + 1):
            filename = self.traj_id[i] + ".txt"
            filepath = os.path.join(self.behavior_config_dir, filename)

            trajectory_waypoints = []

            if os.path.exists(filepath):
                try:
                    with open(filepath, "r") as f:
                        lines = f.readlines()

                    for line in lines:
                        line = line.strip()
                        if line and not line.startswith("#"):
                            parts = line.split()
                            if len(parts) == 23:  # 22 joint positions + time
                                positions = [float(x) for x in parts[:-1]]
                                timestamp = float(parts[-1])
                                trajectory_waypoints.append((timestamp, positions))

                except Exception as e:
                    self.get_logger().error(f"Failed to load trajectory {filename}: {e}")
            else:
                self.get_logger().warn(f"Trajectory file {filepath} not found")

            self.traj_points.append(trajectory_waypoints)

    def cb_behavior_id(self, msg):
        """Callback for behavior"""
        behavior_id = msg.data
        if 0 <= behavior_id < self.num_behavior_id:
            if behavior_id != self.behavior_id:
                self.get_logger().info(f"Behavior changed: {self.behavior_id} -> {behavior_id}")
                if behavior_id == 0:
                    self.get_logger().info("Switching to ML inference mode")
                    self.stop_trajectory()
                else:
                    behavior_name = self.traj_id.get(behavior_id, "unknown")
                    self.get_logger().info(f"Executing trajectory: {behavior_name}")
                self.behavior_id = behavior_id
        else:
            self.get_logger().warn(f"Invalid behavior: {behavior_id}")

    def cb_op_mod(self, msg):
        self.op_mod = msg.data
        if self.op_mod == 0:
            self.get_logger().info("Set Operation Mod: Normal")
            self.toggle_teaching_mode(False)
        elif self.op_mod == 1:
            self.get_logger().info("Set Operation Mod: Teaching")
            self.toggle_teaching_mode(True)

    def cb_torque_enable(self, msg):
        self.toggle_torque(msg.data)

    def read_and_publish(self):
        """Read and parse serial data from OpenRB"""
        buffer = bytearray()

        while True:
            if self.port.in_waiting > 0:
                new_data = self.port.read(self.port.in_waiting)
                buffer.extend(new_data)

                # Process all complete packets in buffer
                while True:
                    header_pos = find_header(buffer)
                    if header_pos == -1:
                        # No header found, keep last 3 bytes in case header is split
                        if len(buffer) > 3:
                            buffer = buffer[-3:]
                        break

                    # Remove data before header
                    if header_pos > 0:
                        buffer = buffer[header_pos:]

                    # Need at least header + length field (2 bytes)
                    if len(buffer) < 6:
                        break

                    # Read packet length (little endian)
                    packet_length = struct.unpack("<H", buffer[4:6])[0]
                    expected_size = 4 + 2 + packet_length  # header + length + data

                    # Wait for complete packet
                    if len(buffer) < expected_size:
                        break

                    # Extract packet data (without header and length)
                    packet_data = buffer[6:expected_size]

                    # Remove byte stuffing
                    destuffed_data = destuff_data(packet_data)

                    # Get packet ID from destuffed data
                    if len(destuffed_data) > 0:
                        packet_id = destuffed_data[0]

                        # Parse encoder packet
                        if packet_id == 0x01:
                            encoder_data = parse_encoder_packet(destuffed_data)
                            if encoder_data:
                                pos = encoder_data["positions"]
                                vel = encoder_data["velocities"]

                                # Update current position data
                                self.current_joint_pos_raw = pos

                                # Convert to sim order for current_joint_pos
                                for i, name in enumerate(self.sim_joint_names):
                                    if name in self.motor_index:
                                        idx = self.motor_index[name] - 1
                                        if idx < 18:
                                            self.current_joint_pos[i] = (
                                                pos[idx] - self.default_pos_inorder[idx]
                                            )

                                # Publish joint state data
                                new_joint_state = JointState()
                                new_joint_state.name = self.sim_joint_names
                                new_joint_state.position = [0.0 for _ in range(22)]
                                new_joint_state.velocity = [0.0 for _ in range(22)]

                                for i, name in enumerate(self.sim_joint_names):
                                    if name in self.motor_index:
                                        idx = self.motor_index[name] - 1
                                        if idx < 18:
                                            new_joint_state.position[i] = (
                                                pos[idx] - self.default_pos_inorder[idx]
                                            )
                                            new_joint_state.velocity[i] = vel[idx]

                                self.q_pos.put(
                                    new_joint_state.position[:18]
                                )  # Only first 18 for ML
                                self.q_vel.put(new_joint_state.velocity[:18])

                                new_joint_state.header.stamp = rclpy.time.Time().to_msg()
                                self.pub_joint_state.publish(new_joint_state)

                        # Parse IMU packet
                        elif packet_id == 0x02:
                            imu_data = parse_imu_packet(destuffed_data)
                            if imu_data:
                                # Convert to list format
                                imu = np.concatenate(
                                    [imu_data["accel"], imu_data["gyro"], imu_data["quat"]]
                                ).tolist()

                                self.q_imu.put(imu)
                                new_imu = Imu()
                                new_imu.linear_acceleration.x = imu[0]
                                new_imu.linear_acceleration.y = imu[1]
                                new_imu.linear_acceleration.z = imu[2]
                                new_imu.angular_velocity.x = imu[3]
                                new_imu.angular_velocity.y = imu[4]
                                new_imu.angular_velocity.z = imu[5]
                                new_imu.orientation.x = imu[6]
                                new_imu.orientation.y = imu[7]
                                new_imu.orientation.z = imu[8]
                                new_imu.orientation.w = imu[9]
                                new_imu.header.stamp = rclpy.time.Time().to_msg()
                                self.pub_imu.publish(new_imu)

                    # Remove processed packet from buffer
                    buffer = buffer[expected_size:]

            time.sleep(0.001)  # Small delay to prevent busy waiting

    def control_loop(self):
        """Main control loop - handles both ML inference and trajectory execution"""
        # Get latest IMU data or use cached value
        if not self.q_imu.empty():
            self.last_imu = np.array(self.q_imu.get(block=False))

        # Get latest joint data or use cached values
        if not self.q_pos.empty():
            self.last_joint_pos = np.array(self.q_pos.get(block=False))
        if not self.q_vel.empty():
            self.last_joint_vel = np.array(self.q_vel.get(block=False))

        if not self.op_mod == 0:
            return

        if self.behavior_id == 0:  # ML Inference mode
            self.inference()
        else:  # Trajectory execution mode
            if not self.trajectory_active and self.behavior_id < len(self.traj_points):
                trajectory_waypoints = self.traj_points[self.behavior_id]
                if trajectory_waypoints:
                    self.trajectory_waypoints = trajectory_waypoints.copy()
                    # Sort by timestamp
                    self.trajectory_waypoints.sort(key=lambda x: x[0])

                    # Add transition from current pose
                    self.add_current_pose_transition()

                    # Interpolate trajectory at 50Hz
                    self.interpolate_trajectory()

                    # Execute trajectory
                    self.execute_trajectory()
            self.behavior_id = 0

    def inference(self):
        """ML inference for autonomous control"""
        imu = self.last_imu

        base_ang_vel = np.array(imu[3:6])
        projected_gravity = np.array(self.quat_rotate_inverse(imu[-4:], [0, 0, -1]))

        # Dummy: Will be added in inference
        command_velocity = np.zeros(3)

        joint_pos = self.last_joint_pos
        joint_vel = self.last_joint_vel

        # Dummy: Will be added in inference
        prev_action = np.zeros(18)
        obs = np.concatenate(
            [
                base_ang_vel,
                projected_gravity,
                command_velocity,
                joint_pos,
                joint_vel,
                prev_action,
            ]
        ).tolist()

        # self.obs_log.writelines(str(obs) + "\n")
        # Inference service call
        req = GetActions.Request()
        req.observations = obs
        future = self.client_inference.call_async(req)
        future.add_done_callback(self.service_callback)

    def quat_rotate_inverse(self, q, v):
        """Quaternion rotation inverse"""
        q_w = q[3]
        q_vec = q[:3]
        a = v * np.array(2.0 * q_w * q_w - 1.0)
        b = np.cross(q_vec, v) * q_w * 2.0
        dot = np.dot(q_vec, v)
        c = q_vec * dot * 2.0
        return a - b + c

    def service_callback(self, future):
        """Handle ML inference service response"""
        actions = future.result().raw_actions
        # self.action_log.writelines(str(list(actions)) + "\n")

        if self.zero_action:
            actions = np.zeros(18)

        target_positions = []
        for i in range(18):
            target_pos = self.default_pos_inorder[i] + actions[i] * self.action_ratio

            # Apply clipping if joint has defined limits
            joint_name = None
            for name, motor_id in self.motor_index.items():
                if motor_id - 1 == i:
                    joint_name = name
                    break

            if joint_name and joint_name in self.joint_pos_clip:
                min_pos, max_pos = self.joint_pos_clip[joint_name]
                target_pos = np.clip(target_pos, min_pos, max_pos)

            target_positions.append(target_pos)

        send_target_positions_binary(self.port, target_positions)

    def add_current_pose_transition(self):
        """Add smooth transition from current pose to trajectory start"""
        if not self.trajectory_waypoints:
            return

        # Skip transition if transition_time is None
        if self.transition_time is None:
            self.get_logger().info("Skipping current pose transition (transition_time is None)")
            return

        # Wait a moment to ensure we have recent data
        time.sleep(0.1)
        current_pose = self.current_joint_pos.copy()

        # Get original first waypoint
        first_time, first_positions = self.trajectory_waypoints[0]

        # Calculate position error between current pose and first trajectory position
        position_error = 0.0
        valid_joints = 0
        for i in range(min(len(current_pose), len(first_positions))):
            if i < 18:  # Only check actual motor joints (18 motors)
                error = abs(current_pose[i] - first_positions[i])
                position_error += error * error
                valid_joints += 1

        if valid_joints > 0:
            rms_error = (position_error / valid_joints) ** 0.5
            if rms_error < 0.1:
                self.get_logger().info(
                    f"Skipping current pose transition (position error {rms_error:.3f} < 0.1 rad)"
                )
                return

        # Shift all existing waypoints forward in time
        for i in range(len(self.trajectory_waypoints)):
            old_time, positions = self.trajectory_waypoints[i]
            self.trajectory_waypoints[i] = (old_time + self.transition_time, positions)

        # Insert current pose at time 0
        self.trajectory_waypoints.insert(0, (0.0, current_pose.tolist()))

        # Insert intermediate transition point at transition_time
        self.trajectory_waypoints.insert(1, (first_time, first_positions))

        # Sort waypoints by time (should already be sorted)
        self.trajectory_waypoints.sort(key=lambda x: x[0])

        self.get_logger().info(
            f"Added {self.transition_time}s transition (position error: {rms_error:.3f} rad)"
        )

    def interpolate_trajectory(self):
        """Interpolate waypoints to create 50Hz trajectory points"""
        if len(self.trajectory_waypoints) < 2:
            self.get_logger().info("Need at least 2 waypoints for interpolation")
            return

        self.trajectory_interpolated = []

        # Get total trajectory time
        start_time = self.trajectory_waypoints[0][0]
        end_time = self.trajectory_waypoints[-1][0]

        if self.interpolation_method == "cubic":
            self._interpolate_cubic_spline(start_time, end_time)
        else:
            self._interpolate_linear(start_time, end_time)

    def _interpolate_linear(self, start_time, end_time):
        """Linear interpolation between waypoints"""
        # Generate time points at 50Hz
        current_time = start_time
        while current_time <= end_time:
            positions = self.interpolate_at_time_linear(current_time)
            if positions is not None:
                self.trajectory_interpolated.append((current_time, positions))
            current_time += self.trajectory_dt

    def _interpolate_cubic_spline(self, start_time, end_time):
        """Cubic spline interpolation between waypoints"""
        if interpolate is None:
            self.get_logger().warn("Scipy not available, falling back to linear interpolation")
            self._interpolate_linear(start_time, end_time)
            return

        if len(self.trajectory_waypoints) < 3:
            self.get_logger().warn(
                "Cubic spline requires at least 3 waypoints, falling back to linear"
            )
            self._interpolate_linear(start_time, end_time)
            return

        try:
            # Extract times and positions
            times = [wp[0] for wp in self.trajectory_waypoints]
            positions = [wp[1] for wp in self.trajectory_waypoints]

            # Ensure strictly increasing timestamps for cubic spline
            cleaned_times = []
            cleaned_positions = []
            min_time_diff = 0.001  # Minimum 1ms between waypoints

            cleaned_times.append(times[0])
            cleaned_positions.append(positions[0])

            for i in range(1, len(times)):
                # Ensure current time is at least min_time_diff after previous time
                if times[i] <= cleaned_times[-1]:
                    adjusted_time = cleaned_times[-1] + min_time_diff
                    cleaned_times.append(adjusted_time)
                    self.get_logger().info(
                        f"Adjusted duplicate timestamp from {times[i]:.3f}s to {adjusted_time:.3f}s"
                    )
                else:
                    cleaned_times.append(times[i])
                cleaned_positions.append(positions[i])

            # Create cubic splines for each joint
            splines = []
            for joint_idx in range(22):
                joint_values = [pos[joint_idx] for pos in cleaned_positions]
                spline = interpolate.CubicSpline(cleaned_times, joint_values, bc_type="natural")
                splines.append(spline)

            # Update end_time if it was adjusted
            actual_end_time = cleaned_times[-1]

            # Generate interpolated points at 50Hz
            current_time = start_time
            while current_time <= actual_end_time:
                interpolated_pos = []
                for joint_idx in range(22):
                    value = float(splines[joint_idx](current_time))
                    interpolated_pos.append(value)

                self.trajectory_interpolated.append((current_time, interpolated_pos))
                current_time += self.trajectory_dt
        except Exception as e:
            self.get_logger().error(f"Cubic spline interpolation failed: {e}")
            self.get_logger().info("Falling back to linear interpolation")
            self._interpolate_linear(start_time, end_time)

    def interpolate_at_time_linear(self, target_time):
        """Linear interpolation of joint positions at a specific time"""
        # Find the two waypoints that bracket target_time
        prev_waypoint = None
        next_waypoint = None

        for waypoint in self.trajectory_waypoints:
            if waypoint[0] <= target_time:
                prev_waypoint = waypoint
            elif waypoint[0] > target_time and next_waypoint is None:
                next_waypoint = waypoint
                break

        if prev_waypoint is None:
            return self.trajectory_waypoints[0][1]  # Before start
        elif next_waypoint is None:
            return prev_waypoint[1]  # After end

        # Linear interpolation
        t1, pos1 = prev_waypoint
        t2, pos2 = next_waypoint

        if t2 == t1:  # Same timestamp
            return pos1

        # Interpolation factor
        alpha = (target_time - t1) / (t2 - t1)
        alpha = np.clip(alpha, 0.0, 1.0)

        # Interpolate each joint
        interpolated_pos = []
        for i in range(22):
            interp_val = pos1[i] + alpha * (pos2[i] - pos1[i])
            interpolated_pos.append(interp_val)

        return interpolated_pos

    def execute_trajectory(self):
        """Execute loaded and interpolated trajectory"""
        if not self.trajectory_interpolated:
            self.get_logger().info("No trajectory loaded or interpolated")
            return

        self.get_logger().info(
            f"Starting trajectory execution... ({len(self.trajectory_interpolated)} points at 50Hz)"
        )
        self.trajectory_active = True
        self.trajectory_start_time = time.time()

        self._trajectory_execution_loop()

    def _trajectory_execution_loop(self):
        """Main trajectory execution loop at 50Hz"""
        trajectory_idx = 0
        last_send_time = -self.trajectory_dt  # Start with negative to allow immediate first command

        # Send initial position immediately to ensure smooth start
        if len(self.trajectory_interpolated) > 0:
            initial_time, initial_positions = self.trajectory_interpolated[0]

            # Convert and send initial position
            motor_positions = [0.0] * 18
            for i, name in enumerate(self.sim_joint_names):
                if name in self.motor_index:
                    motor_idx = self.motor_index[name] - 1
                    if motor_idx < 18:
                        target_pos = initial_positions[i]
                        # Apply joint limits
                        if name in self.joint_pos_clip:
                            min_pos, max_pos = self.joint_pos_clip[name]
                            target_pos = np.clip(target_pos, min_pos, max_pos)
                        motor_positions[motor_idx] = target_pos

            send_target_positions_binary(self.port, motor_positions)
            self.get_logger().info("Sent initial position to ensure smooth trajectory start")

        while self.trajectory_active and trajectory_idx < len(self.trajectory_interpolated):
            current_time = time.time() - self.trajectory_start_time
            target_time, target_positions = self.trajectory_interpolated[trajectory_idx]

            # Send commands at exactly 50Hz (0.02s intervals)
            if (
                current_time >= target_time
                and (current_time - last_send_time) >= self.trajectory_dt
            ):
                # Convert sim order to motor order
                motor_positions = [0.0] * 18
                for i, name in enumerate(self.sim_joint_names):
                    if name in self.motor_index:
                        motor_idx = self.motor_index[name] - 1
                        if motor_idx < 18:
                            target_pos = target_positions[i]

                            # Apply joint limits
                            if name in self.joint_pos_clip:
                                min_pos, max_pos = self.joint_pos_clip[name]
                                target_pos = np.clip(target_pos, min_pos, max_pos)

                            motor_positions[motor_idx] = target_pos

                send_target_positions_binary(self.port, motor_positions)

                # Progress update every 50 points (1 second)
                if trajectory_idx % 50 == 0:
                    progress = (trajectory_idx / len(self.trajectory_interpolated)) * 100
                    self.get_logger().info(
                        f"Trajectory progress: {progress:.1f}% ({current_time:.2f}s)"
                    )

                last_send_time = current_time
                trajectory_idx += 1

            time.sleep(0.005)  # Sleep 5ms, check 4x more frequently than command rate

        self.trajectory_active = False
        self.get_logger().info("Trajectory execution completed")

    def stop_trajectory(self):
        """Stop current trajectory execution"""
        if self.trajectory_active:
            self.trajectory_active = False
            self.get_logger().info("Trajectory stopped")

    def toggle_torque(self, enable):
        """Send torque enable/disable command"""
        self.torque_enabled = send_torque_enable(self.port, enable)
        self.get_logger().info(f"Torque {'ENABLED' if enable else 'DISABLED'}")

    def toggle_teaching_mode(self, enable):
        """Send teaching mode enable/disable command"""
        send_teaching_mode_enable(self.port, enable)
        self.get_logger().info(f"Teaching mode {'ENABLED' if enable else 'DISABLED'}")
        if enable:
            self.get_logger().info("Robot is now compliant for direct teaching")
        else:
            self.get_logger().info("Robot switched back to trajectory control mode")


if __name__ == "__main__":
    rclpy.init()

    try:
        node = OpenRB150()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()
