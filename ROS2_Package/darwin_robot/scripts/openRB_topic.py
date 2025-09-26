import serial
import rclpy
import threading
import queue
import numpy as np
import time
import struct
from collections import deque

from packet_handler import *
from rclpy.node import Node

from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState

# from hri_humanoid_interfaces.srv import GetActions  # Not needed for joint command subscription

# NOTE: Action joint orders follow motor_index
#       while joint orders of positions, and velocities follow sim joint orders
# joint name and real robot motor ids
motor_index = {
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
    "j_gripper_l": 22,  # not real
    "j_gripper_r": 21,  # not real
    "j_ankle1_l": 16,
    "j_ankle1_r": 15,
    "j_ankle2_l": 18,
    "j_ankle2_r": 17,
}

# isaac sim joints
sim_joint_names = [
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
    "j_gripper_r",  # not real
    "j_gripper_l",  # not real
]

default_joint_pos = {
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

joint_pos_clip = {
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


class OpenRB150(Node):
    def __init__(self):
        super().__init__("openrb150_serial")

        # Set default positions inorder
        self.default_pos_inorder = [0.0 for _ in range(22)]
        for name in sim_joint_names:
            self.default_pos_inorder[motor_index[name] - 1] = default_joint_pos[name]

        # Create UART connection with OpenRB
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("serial_baud", 921600)
        serial_port = self.get_parameter("serial_port").value
        serial_baud = self.get_parameter("serial_baud").value
        self.port = serial.Serial(serial_port, serial_baud)

        # Create Publishers
        self.pub_imu = self.create_publisher(Imu, "imu", 10)
        self.pub_joint_state = self.create_publisher(JointState, "joint_states", 10)

        # Joint command subscription (instead of inference service)
        self.subscription_joint_cmd = self.create_subscription(
            JointState, "joint_command", self.joint_command_callback, 10
        )

        serial_thread = threading.Thread(target=self.read_and_publish)

        self.q_pos = queue.Queue()
        self.q_vel = queue.Queue()
        self.q_imu = queue.Queue()

        # Cache for last known values to avoid blocking
        self.last_imu = np.zeros(10)
        self.last_joint_pos = np.zeros(18)
        self.last_joint_vel = np.zeros(18)

        serial_thread.start()
        self.t = time.time()
        self.timer = self.create_timer(0.02, self.inference_and_apply)

        # CRC statistics timer - print every 10 seconds
        self.stats_timer = self.create_timer(10.0, self.print_crc_stats)

    def read_and_publish(self):
        pos = [0.0] * 18
        vel = [0.0] * 18
        imu = [0.0] * 10
        buffer = bytearray()
        header = b"\xff\xff\xfd\x00"

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

                                # Publish data and save them in buffer
                                new_joint_state = JointState()
                                new_joint_state.name = sim_joint_names

                                # current position/velocities are in robot order
                                # need to convert them in sim order
                                new_joint_state.position = [0.0 for _ in range(18)]
                                new_joint_state.velocity = [0.0 for _ in range(18)]

                                for i, name in enumerate(sim_joint_names):
                                    idx = motor_index[name] - 1
                                    if idx < 18:
                                        new_joint_state.position[i] = (
                                            pos[idx] - self.default_pos_inorder[idx]
                                        )
                                        new_joint_state.velocity[i] = vel[idx]

                                self.q_pos.put(new_joint_state.position)
                                self.q_vel.put(new_joint_state.velocity)

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

                        # Parse timing stats packet
                        elif packet_id == 0x54:
                            timing_data = parse_timing_packet(destuffed_data)
                            if timing_data:
                                print_timing_stats(timing_data, self.get_logger())

                    # Remove processed packet from buffer
                    buffer = buffer[expected_size:]

            time.sleep(0.001)  # Small delay to prevent busy waiting

    def inference_and_apply(self):
        # Collect observations
        # we need base_ang_vel, projected_gravity, joint positions/velocities

        # Get latest IMU data or use cached value
        if not self.q_imu.empty():
            self.last_imu = np.array(self.q_imu.get(block=False))
        imu = self.last_imu

        base_ang_vel = np.array(imu[3:6])
        projected_gravity = np.array(self.quat_rotate_inverse(imu[-4:], [0, 0, -1]))

        # Dummy: Will be added in inference
        command_velocity = np.zeros(3)

        # Get latest joint data or use cached values
        if not self.q_pos.empty():
            self.last_joint_pos = np.array(self.q_pos.get(block=False))
        if not self.q_vel.empty():
            self.last_joint_vel = np.array(self.q_vel.get(block=False))

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
        # Joint commands will be received via subscription to 'joint_command' topic
        # No need for explicit inference service call here

    def quat_rotate_inverse(self, q, v):
        q_w = q[3]
        q_vec = q[:3]
        a = v * np.array(2.0 * q_w * q_w - 1.0)
        b = np.cross(q_vec, v) * q_w * 2.0
        dot = np.dot(q_vec, v)
        c = q_vec * dot * 2.0
        return a - b + c

    def joint_command_callback(self, msg):
        """Callback for joint command subscription"""
        # msg.position contains raw actions from inference.cpp
        actions = list(msg.position)
        # self.action_log.writelines(str(actions) + "\n")

        # For safe debug - comment out to enable real actions
        # actions = [0.0] * 18

        target_positions = []
        for i in range(18):
            target_pos = self.default_pos_inorder[i] + actions[i] * 0.5

            # Apply clipping if joint has defined limits
            joint_name = None
            for name, motor_id in motor_index.items():
                if motor_id - 1 == i:
                    joint_name = name
                    break

            if joint_name and joint_name in joint_pos_clip:
                min_pos, max_pos = joint_pos_clip[joint_name]
                target_pos = np.clip(target_pos, min_pos, max_pos)

            target_positions.append(target_pos)

        # Send to OpenRB using binary protocol
        send_target_positions_binary(self.port, target_positions)

        self.t = time.time()

    def print_crc_stats(self):
        """Print CRC error statistics"""
        print_crc_statistics(self.get_logger())


if __name__ == "__main__":

    rclpy.init()

    node = OpenRB150()
    rclpy.spin(node)

    rclpy.shutdown()
