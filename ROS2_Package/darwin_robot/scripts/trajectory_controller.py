import serial
import threading
import queue
import numpy as np
import time
import struct
import sys
import os
from collections import deque
import json

# Windows-compatible imports
if sys.platform == "win32":
    import msvcrt
else:
    import select

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

# Import joint mappings from openRB.py
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
    "j_gripper_r",
    "j_gripper_l",
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


class TrajectoryController:
    def __init__(
        self, serial_port="/dev/ttyACM0", serial_baud=921600, interpolation_method="linear"
    ):
        # Set default positions in motor order
        self.default_pos_inorder = [0.0 for _ in range(22)]
        for name in sim_joint_names:
            if name in motor_index:
                self.default_pos_inorder[motor_index[name] - 1] = default_joint_pos[name]

        # Create UART connection with OpenRB
        self.port = serial.Serial(serial_port, serial_baud)

        # Data queues
        self.q_pos = queue.Queue()
        self.q_vel = queue.Queue()

        # Cache for current positions
        self.current_joint_pos = np.zeros(22)  # In sim order
        self.current_joint_pos_raw = np.zeros(18)  # In motor order

        # Trajectory control
        self.trajectory_active = False
        self.trajectory_waypoints = []  # Original waypoints from file
        self.trajectory_interpolated = []  # Interpolated at 50Hz
        self.trajectory_start_time = 0
        self.trajectory_dt = 0.02  # 50Hz = 0.02 seconds
        self.interpolation_method = interpolation_method  # "linear" or "cubic"
        self.current_trajectory_filename = None  # Store current trajectory filename
        self.transition_time = 1.0  # Default transition time

        # Torque state tracking
        self.torque_enabled = True  # Assume motors start with torque enabled
        
        # Teaching mode state tracking
        self.teaching_mode_enabled = False

        # Data logging for plotting actual positions
        self.actual_positions_log = []  # List of (time, positions) tuples
        self.logging_active = False
        self.log_start_time = 0

        # Start serial reading thread
        self.serial_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.serial_thread.start()

        print(f"Trajectory Controller initialized (interpolation: {self.interpolation_method})")
        print("Commands:")
        print("  'save' - Save current joint positions (trajectory format)")
        print("  'save <time>' - Save with timestamp")
        print(
            "  'load <filename> [time]' - Load and execute trajectory (always starts from current pose)"
        )
        print("  'load_w_interp <filename> <method> [time]' - Load with interpolation method")
        print("  'plot' - Plot loaded trajectory")
        print("  'toggle' - Toggle motor torque enable/disable")
        print("  'teaching' - Toggle direct teaching mode")
        print("  'stop' - Stop current trajectory")
        print("  'quit' - Exit program")

    def read_serial(self):
        """Read and parse serial data from OpenRB"""
        buffer = bytearray()
        header = b"\xff\xff\xfd\x00"

        while True:
            if self.port.in_waiting > 0:
                new_data = self.port.read(self.port.in_waiting)
                buffer.extend(new_data)

                # Process all complete packets in buffer
                while True:
                    header_pos = self.find_header(buffer)
                    if header_pos == -1:
                        if len(buffer) > 3:
                            buffer = buffer[-3:]
                        break

                    if header_pos > 0:
                        buffer = buffer[header_pos:]

                    if len(buffer) < 6:
                        break

                    packet_length = struct.unpack("<H", buffer[4:6])[0]
                    expected_size = 4 + 2 + packet_length

                    if len(buffer) < expected_size:
                        break

                    packet_data = buffer[6:expected_size]
                    destuffed_data = self.destuff_data(packet_data)

                    if len(destuffed_data) > 0:
                        packet_id = destuffed_data[0]

                        if packet_id == 0x01:  # Encoder data
                            encoder_data = self.parse_encoder_packet(destuffed_data)
                            if encoder_data:
                                self.current_joint_pos_raw = encoder_data["positions"]

                                # Convert to sim order
                                for i, name in enumerate(sim_joint_names):
                                    if name in motor_index:
                                        idx = motor_index[name] - 1
                                        if idx < 18:
                                            self.current_joint_pos[i] = (
                                                self.current_joint_pos_raw[idx]
                                                # - self.default_pos_inorder[idx]
                                            )

                                # Log actual positions if logging is active
                                if self.logging_active:
                                    current_time = time.time() - self.log_start_time
                                    self.actual_positions_log.append(
                                        (current_time, self.current_joint_pos.copy())
                                    )

                        elif packet_id == 0x02:  # IMU data (not currently processed)
                            pass  # Could add IMU processing here if needed

                        # Note: packet_id 0x04 torque commands don't send confirmation

                    buffer = buffer[expected_size:]

            time.sleep(0.001)

    def find_header(self, data):
        """Find header position in data, excluding stuffed patterns"""
        header = b"\xff\xff\xfd\x00"
        pos = 0
        while True:
            pos = data.find(header, pos)
            if pos == -1:
                return -1
            if pos + 4 < len(data) and data[pos + 4] == 0xFD:
                pos += 1
                continue
            else:
                return pos

    def destuff_data(self, data):
        """Remove byte stuffing: 0xFF 0xFF 0xFD 0xFD -> 0xFF 0xFF 0xFD"""
        destuffed = bytearray()
        i = 0
        while i < len(data):
            if (
                i <= len(data) - 4
                and data[i] == 0xFF
                and data[i + 1] == 0xFF
                and data[i + 2] == 0xFD
                and data[i + 3] == 0xFD
            ):
                destuffed.extend([0xFF, 0xFF, 0xFD])
                i += 4
            else:
                destuffed.append(data[i])
                i += 1
        return bytes(destuffed)

    def parse_encoder_packet(self, packet_data):
        """Parse encoder packet"""
        if len(packet_data) < 145:
            return None

        packet_id = packet_data[0]
        if packet_id != 0x01:
            return None

        try:
            positions = struct.unpack("<18f", packet_data[1:73])
            velocities = struct.unpack("<18f", packet_data[73:145])

            return {
                "timestamp": time.time(),
                "positions": np.array(positions),
                "velocities": np.array(velocities),
            }
        except struct.error:
            return None

    def stuff_data(self, data):
        """Add byte stuffing: 0xFF 0xFF 0xFD -> 0xFF 0xFF 0xFD 0xFD"""
        stuffed = bytearray()
        i = 0
        while i < len(data):
            stuffed.append(data[i])
            if (
                len(stuffed) >= 3
                and stuffed[-3] == 0xFF
                and stuffed[-2] == 0xFF
                and stuffed[-1] == 0xFD
            ):
                stuffed.append(0xFD)
            i += 1
        return bytes(stuffed)

    def send_target_positions(self, target_positions):
        """Send target positions to robot"""
        packet_data = bytearray()
        packet_data.append(0x03)  # Target position packet ID

        for pos in target_positions:
            packet_data.extend(struct.pack("<f", pos))

        stuffed_data = self.stuff_data(packet_data)
        header = b"\xff\xff\xfd\x00"
        length = struct.pack("<H", len(stuffed_data))
        complete_packet = header + length + stuffed_data

        self.port.write(complete_packet)

    def send_torque_enable(self, enable):
        """Send torque enable/disable command to robot"""
        packet_data = bytearray()
        packet_data.append(0x04)  # Torque enable packet ID
        packet_data.append(1 if enable else 0)  # Enable flag

        stuffed_data = self.stuff_data(packet_data)
        header = b"\xff\xff\xfd\x00"
        length = struct.pack("<H", len(stuffed_data))
        complete_packet = header + length + stuffed_data

        self.port.write(complete_packet)
        # Update local state immediately (no confirmation packet)
        self.torque_enabled = enable
        print(f"Torque {'ENABLED' if enable else 'DISABLED'}")

    def send_teaching_mode_enable(self, enable):
        """Send teaching mode enable/disable command to robot"""
        packet_data = bytearray()
        packet_data.append(0x06)  # Teaching mode packet ID
        packet_data.append(1 if enable else 0)  # Enable flag

        stuffed_data = self.stuff_data(packet_data)
        header = b"\xff\xff\xfd\x00"
        length = struct.pack("<H", len(stuffed_data))
        complete_packet = header + length + stuffed_data

        self.port.write(complete_packet)
        # Update local state immediately (no confirmation packet)
        self.teaching_mode_enabled = enable
        print(f"Teaching mode {'ENABLED' if enable else 'DISABLED'}")
        if enable:
            print("Robot is now compliant for direct teaching")
        else:
            print("Robot switched back to trajectory control mode")

    def save_current_position(self, filename=None, timestamp_value=0.0):
        """Save current joint positions to file in trajectory format"""
        if filename is None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"trajectory_waypoint_{timestamp}.txt"

        # Wait a moment to ensure we have recent data
        time.sleep(0.1)

        with open(filename, "w") as f:
            f.write("# Joint positions saved at {}\n".format(time.strftime("%Y-%m-%d %H:%M:%S")))
            f.write(
                "# Format: 22 joint positions (in sim_joint_names order) followed by time_from_start\n"
            )
            f.write(
                "# j_pelvis_l j_pelvis_r j_shoulder_l j_shoulder_r j_thigh1_l j_thigh1_r j_high_arm_l j_high_arm_r j_thigh2_l j_thigh2_r j_low_arm_l j_low_arm_r j_tibia_l j_tibia_r j_ankle1_l j_ankle1_r j_ankle2_l j_ankle2_r j_pan j_tilt j_gripper_r j_gripper_l time_from_start\n"
            )

            # Write current positions in trajectory format: 22 positions + time
            pos_str = " ".join([f"{self.current_joint_pos[i]:.6f}" for i in range(22)])
            f.write(f"{pos_str} {timestamp_value:.3f}\n")

        print(
            f"Saved current joint positions to {filename} (trajectory format, time={timestamp_value:.3f}s)"
        )
        return filename

    def load_trajectory(self, filename, transition_time=1.0):
        """Load trajectory from text file and interpolate at 50Hz (always starts from current pose)"""
        try:
            with open(filename, "r") as f:
                lines = f.readlines()

            self.trajectory_waypoints = []
            for line in lines:
                line = line.strip()
                if line and not line.startswith("#"):
                    parts = line.split()
                    if len(parts) == 23:  # 22 joint positions + time
                        positions = [float(x) for x in parts[:-1]]
                        timestamp = float(parts[-1])
                        self.trajectory_waypoints.append((timestamp, positions))

            if self.trajectory_waypoints:
                # Sort by timestamp
                self.trajectory_waypoints.sort(key=lambda x: x[0])
                print(f"Loaded {len(self.trajectory_waypoints)} waypoints from {filename}")

                # Store filename for plot naming
                self.current_trajectory_filename = filename

                # Add transition from current pose (if conditions are met)
                self.add_current_pose_transition(transition_time)

                # Interpolate trajectory at 50Hz
                self.interpolate_trajectory()
                print(
                    f"Generated {len(self.trajectory_interpolated)} interpolated points at 50Hz ({self.interpolation_method})"
                )
                return True
            else:
                print(f"No valid trajectory data found in {filename}")
                return False

        except Exception as e:
            print(f"Error loading trajectory: {e}")
            return False

    def create_sample_trajectory(self, filename="sample_trajectory.txt"):
        """Create a sample trajectory file for demonstration"""
        with open(filename, "w") as f:
            f.write("# Sample trajectory: Zero pose -> Default pose -> Zero pose\n")
            f.write(
                "# Format: 22 joint positions (in sim_joint_names order) followed by time_from_start\n"
            )
            f.write(
                "# j_pelvis_l j_pelvis_r j_shoulder_l j_shoulder_r j_thigh1_l j_thigh1_r j_high_arm_l j_high_arm_r j_thigh2_l j_thigh2_r j_low_arm_l j_low_arm_r j_tibia_l j_tibia_r j_ankle1_l j_ankle1_r j_ankle2_l j_ankle2_r j_pan j_tilt j_gripper_r j_gripper_l time_from_start\n"
            )

            # Create default pose positions (in sim_joint_names order)
            default_pose = [0.0] * 22
            for i, name in enumerate(sim_joint_names):
                if name in default_joint_pos:
                    default_pose[i] = default_joint_pos[name]

            # Create waypoints: zero -> default -> zero
            waypoints = [
                (0.0, [0.0] * 22),  # Zero pose (all joints at 0)
                (2.0, default_pose),  # Default standing pose
                (4.0, [0.0] * 22),  # Return to zero pose
            ]

            # Write waypoints to file
            for t, positions in waypoints:
                pos_str = " ".join([f"{p:.6f}" for p in positions])
                f.write(f"{pos_str} {t:.3f}\n")

        print(
            f"Created sample trajectory file: {filename} (Zero->Default->Zero, interpolated to 50Hz)"
        )

    def add_current_pose_transition(self, transition_time):
        """Add smooth transition from current pose to trajectory start"""
        if not self.trajectory_waypoints:
            return

        # Skip interpolation if transition_time is None
        if transition_time is None:
            print("Skipping current pose transition (transition_time is None)")
            return

        # Get current joint positions (fresh capture since torque is enabled)
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
                print(
                    f"Skipping current pose transition (position error {rms_error:.3f} < 0.1 rad)"
                )
                return

        # Shift all existing waypoints forward in time
        for i in range(len(self.trajectory_waypoints)):
            old_time, positions = self.trajectory_waypoints[i]
            self.trajectory_waypoints[i] = (old_time + transition_time, positions)

        # Insert current pose at time 0
        self.trajectory_waypoints.insert(0, (0.0, current_pose.tolist()))

        # Insert intermediate transition point at transition_time
        self.trajectory_waypoints.insert(1, (transition_time, first_positions))

        # Store transition time for later use
        self.transition_time = transition_time

        # Sort waypoints by time (should already be sorted)
        self.trajectory_waypoints.sort(key=lambda x: x[0])

        print(f"Added {transition_time}s transition (position error: {rms_error:.3f} rad)")

    def plot_trajectory(self):
        """Plot trajectory joint positions in 3x6 subplots with motor IDs as titles"""
        if plt is None:
            print("Error: matplotlib is required for plotting")
            print("Install with: pip install matplotlib")
            return

        if not self.trajectory_interpolated:
            print("No trajectory loaded. Load a trajectory first.")
            return

        # Extract interpolated trajectory data
        times = [point[0] for point in self.trajectory_interpolated]
        positions = np.array([point[1] for point in self.trajectory_interpolated])

        # Extract actual position data (if available)
        actual_times = []
        actual_positions = []
        if self.actual_positions_log:
            actual_times = [point[0] for point in self.actual_positions_log]
            actual_positions = np.array([point[1] for point in self.actual_positions_log])

        # Create 3x6 subplot grid
        fig, axes = plt.subplots(3, 6, figsize=(18, 10))
        fig.suptitle("Joint Trajectory Positions", fontsize=16)

        # Plot each joint (18 motors)
        for i in range(18):
            row = i // 6
            col = i % 6
            ax = axes[row, col]

            # Find corresponding sim joint for this motor index
            motor_id = i + 1
            joint_name = None
            for name, idx in motor_index.items():
                if idx == motor_id:
                    joint_name = name
                    break

            if joint_name and joint_name in sim_joint_names:
                sim_idx = sim_joint_names.index(joint_name)

                # Plot interpolated trajectory (solid black line)
                joint_positions = positions[:, sim_idx]
                ax.plot(times, joint_positions, "k-", linewidth=1.5, label="Target")

                # Plot actual positions (dashed black line) if available
                if len(actual_positions) > 0:
                    actual_joint_positions = actual_positions[:, sim_idx]
                    ax.plot(
                        actual_times, actual_joint_positions, "k--", linewidth=1.5, label="Actual"
                    )
            else:
                # If no mapping found, use zeros
                joint_positions = np.zeros(len(times))
                ax.plot(times, joint_positions, "k-", linewidth=1.5, label="Target")

            # Use joint name if found, otherwise motor ID
            title = joint_name if joint_name else f"Motor {motor_id}"
            ax.set_title(title, fontsize=10)
            ax.set_ylim(-2.35, 2.35)
            ax.grid(True, alpha=0.3)
            ax.set_xlabel("Time (s)", fontsize=8)
            ax.set_ylabel("Position (rad)", fontsize=8)

        plt.tight_layout()

        # Add global legend if actual positions are available
        if len(actual_positions) > 0:
            handles, labels = axes[0, 0].get_legend_handles_labels()
            fig.legend(handles, labels, loc="upper right", bbox_to_anchor=(1.0, 1.0), ncol=1)

        # Generate filename with trajectory name, interpolation type and timestamp
        trajectory_name = "unknown"
        if self.current_trajectory_filename:
            # Extract filename without path and extension
            import os

            trajectory_name = os.path.splitext(os.path.basename(self.current_trajectory_filename))[
                0
            ]

        filename = f"trajectory_plot_{trajectory_name}_{self.interpolation_method}.png"
        plt.savefig(filename, dpi=300, bbox_inches="tight")
        plt.close()
        print(f"Plot saved as: {filename}")

    def interpolate_trajectory(self):
        """Interpolate waypoints to create 50Hz trajectory points"""
        if len(self.trajectory_waypoints) < 2:
            print("Need at least 2 waypoints for interpolation")
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
            print("Scipy not available, falling back to linear interpolation")
            self._interpolate_linear(start_time, end_time)
            return

        if len(self.trajectory_waypoints) < 3:
            print("Cubic spline requires at least 3 waypoints, falling back to linear")
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
                    print(
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
            print(f"Cubic spline interpolation failed: {e}")
            print("Falling back to linear interpolation")
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

    def set_interpolation_method(self, method):
        """Change interpolation method: 'linear' or 'cubic'"""
        if method in ["linear", "cubic"]:
            self.interpolation_method = method
            print(f"Interpolation method set to: {method}")
        else:
            print("Invalid method. Use 'linear' or 'cubic'")

    def execute_trajectory(self):
        """Execute loaded and interpolated trajectory"""
        if not self.trajectory_interpolated:
            print("No trajectory loaded or interpolated")
            return

        print(
            f"Starting trajectory execution... ({len(self.trajectory_interpolated)} points at 50Hz)"
        )
        self.trajectory_active = True
        self.trajectory_start_time = time.time()

        # Start logging actual positions
        self.actual_positions_log = []
        self.logging_active = True
        self.log_start_time = self.trajectory_start_time

        trajectory_thread = threading.Thread(target=self._trajectory_execution_loop, daemon=True)
        trajectory_thread.start()

    def _trajectory_execution_loop(self):
        """Main trajectory execution loop at 50Hz"""
        trajectory_idx = 0
        last_send_time = -self.trajectory_dt  # Start with negative to allow immediate first command

        # Send initial position immediately to ensure smooth start
        if len(self.trajectory_interpolated) > 0:
            initial_time, initial_positions = self.trajectory_interpolated[0]

            # Convert and send initial position
            motor_positions = [0.0] * 18
            for i, name in enumerate(sim_joint_names):
                if name in motor_index:
                    motor_idx = motor_index[name] - 1
                    if motor_idx < 18:
                        target_pos = initial_positions[i]
                        # Apply joint limits
                        if name in joint_pos_clip:
                            min_pos, max_pos = joint_pos_clip[name]
                            target_pos = np.clip(target_pos, min_pos, max_pos)
                        motor_positions[motor_idx] = target_pos

            self.send_target_positions(motor_positions)
            print("Sent initial position to ensure smooth trajectory start")

        while self.trajectory_active and trajectory_idx < len(self.trajectory_interpolated):
            current_time = time.time() - self.trajectory_start_time
            target_time, target_positions = self.trajectory_interpolated[trajectory_idx]

            # Send commands at exactly 50Hz (0.02s intervals)
            if (
                current_time >= target_time
                and (current_time - last_send_time) >= self.trajectory_dt
            ):
                # Convert sim order to motor order and add default offsets
                motor_positions = [0.0] * 18
                for i, name in enumerate(sim_joint_names):
                    if name in motor_index:
                        motor_idx = motor_index[name] - 1
                        if motor_idx < 18:
                            # target_pos = self.default_pos_inorder[motor_idx] + target_positions[i]
                            target_pos = target_positions[i]

                            # Apply joint limits
                            if name in joint_pos_clip:
                                min_pos, max_pos = joint_pos_clip[name]
                                target_pos = np.clip(target_pos, min_pos, max_pos)

                            motor_positions[motor_idx] = target_pos

                self.send_target_positions(motor_positions)
                last_send_time = current_time
                trajectory_idx += 1

                # Progress update every 50 points (1 second)
                if trajectory_idx % 50 == 0:
                    progress = (trajectory_idx / len(self.trajectory_interpolated)) * 100
                    print(f"Trajectory progress: {progress:.1f}% ({current_time:.2f}s)")

            time.sleep(0.005)  # Sleep 5ms, check 4x more frequently than command rate

        self.trajectory_active = False
        self.logging_active = False
        print("Trajectory execution completed")
        print(f"Logged {len(self.actual_positions_log)} actual position samples")

    def stop_trajectory(self):
        """Stop current trajectory execution"""
        self.trajectory_active = False
        self.logging_active = False
        print("Trajectory stopped")

    def run_interactive(self):
        """Main interactive loop with Windows compatibility"""
        print("\nReady for commands...")
        print("Type 'help' for available commands")

        try:
            while True:
                if sys.platform == "win32":
                    # Windows: use blocking input
                    try:
                        user_input = input("> ").strip()
                        if not self._process_command(user_input):
                            break
                    except EOFError:
                        break
                else:
                    # Linux/Mac: use select for non-blocking input
                    if sys.stdin in select.select([sys.stdin], [], [], 0.01)[0]:
                        user_input = sys.stdin.readline().strip()
                        if not self._process_command(user_input):
                            break
                    time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nShutting down...")

        self.stop_trajectory()
        self.port.close()

    def _process_command(self, user_input):
        """Process user input commands"""
        if user_input == "" or user_input == "save":
            # Save current position with default timestamp 0.0
            self.save_current_position()

        elif user_input.startswith("save "):
            # Save current position with specified timestamp
            try:
                timestamp_str = user_input[5:].strip()
                timestamp_value = float(timestamp_str)
                self.save_current_position(timestamp_value=timestamp_value)
            except ValueError:
                print("Invalid timestamp. Usage: save <time_value>")
                print("Example: save 1.5")

        elif user_input.startswith("load_w_interp "):
            parts = user_input[14:].strip().split()
            if len(parts) < 2:
                print("Usage: load_w_interp <filename> <interpolation_method> [transition_time]")
                print("Example: load_w_interp demo.txt cubic 1.5")
                print("Example: load_w_interp demo.txt linear none")
            else:
                filename = parts[0]
                interp_method = parts[1]
                transition_time = 1.0

                # Validate interpolation method
                if interp_method not in ["linear", "cubic"]:
                    print("Invalid interpolation method. Use 'linear' or 'cubic'")
                else:
                    # Set interpolation method
                    old_method = self.interpolation_method
                    self.set_interpolation_method(interp_method)

                    # Check for custom transition time
                    if len(parts) > 2:
                        time_str = parts[2].lower()
                        if time_str == "none" or time_str == "null":
                            transition_time = None
                        else:
                            try:
                                transition_time = float(parts[2])
                            except ValueError:
                                print("Invalid transition time, using default 1.0s")

                    if self.load_trajectory(filename, transition_time):
                        self.execute_trajectory()
                    else:
                        # Restore old method if loading failed
                        self.set_interpolation_method(old_method)

        elif user_input.startswith("load "):
            parts = user_input[5:].strip().split()
            filename = parts[0]
            transition_time = 1.0

            # Check for custom transition time
            if len(parts) > 1:
                time_str = parts[1].lower()
                if time_str == "none" or time_str == "null":
                    transition_time = None
                else:
                    try:
                        transition_time = float(parts[1])
                    except ValueError:
                        print("Invalid transition time, using default 1.0s")

            if self.load_trajectory(filename, transition_time):
                self.execute_trajectory()

        elif user_input == "stop":
            self.stop_trajectory()

        elif user_input == "quit" or user_input == "exit":
            return False  # Signal to exit

        elif user_input == "sample":
            self.create_sample_trajectory()

        elif user_input == "plot":
            self.plot_trajectory()

        elif user_input.startswith("interp "):
            method = user_input[7:].strip()
            self.set_interpolation_method(method)

        elif user_input == "toggle":
            # Toggle torque enable/disable
            new_state = not self.torque_enabled
            self.send_torque_enable(new_state)

        elif user_input == "teaching":
            # Toggle teaching mode enable/disable
            new_state = not self.teaching_mode_enabled
            self.send_teaching_mode_enable(new_state)

        elif user_input == "enable":
            # Enable torque
            self.send_torque_enable(True)

        elif user_input == "disable":
            # Disable torque
            self.send_torque_enable(False)

        elif user_input == "help":
            print("Commands:")
            print("  save - Save current joint positions (trajectory format, time=0.0)")
            print("  save <time> - Save current joint positions with specified timestamp")
            print(
                "  load <filename> [time] - Load and execute trajectory starting from current pose"
            )
            print("    Example: load demo.txt 1.5  (1.5s transition from current to trajectory)")
            print("    Example: load demo.txt none  (no transition, direct execution)")
            print("    Default transition time: 1.0s")
            print("    Note: Transition skipped automatically if position error < 0.1 rad")
            print("  load_w_interp <filename> <method> [time] - Load with specific interpolation method")
            print("    Example: load_w_interp demo.txt cubic 1.5  (load with cubic spline)")
            print("    Example: load_w_interp demo.txt linear none  (load with linear, no transition)")
            print("    Methods: 'linear' or 'cubic'")
            print("  stop - Stop current trajectory")
            print("  sample - Create sample trajectory file")
            print("  plot - Plot loaded trajectory (3x6 subplots, motor IDs as titles)")
            print("  interp <method> - Set interpolation: 'linear' or 'cubic'")
            print(f"  Current interpolation: {self.interpolation_method}")
            print("  toggle - Toggle motor torque enable/disable")
            print("  teaching - Toggle direct teaching mode")
            print("  enable - Enable motor torques")
            print("  disable - Disable motor torques")
            print(
                f"  Current torque state: {'ENABLED' if self.torque_enabled else 'DISABLED'} (local tracking)"
            )
            print(
                f"  Current teaching mode: {'ENABLED' if self.teaching_mode_enabled else 'DISABLED'} (local tracking)"
            )
            print("  quit/exit - Exit program")

        else:
            print(f"Unknown command: {user_input}")
            print("Type 'help' for available commands")

        return True


if __name__ == "__main__":
    # Default serial port for Windows
    serial_port = "COM5"  # Change this to match your OpenRB port
    interpolation_method = "linear"  # Default to linear interpolation

    if len(sys.argv) > 1:
        serial_port = sys.argv[1]
    if len(sys.argv) > 2:
        interpolation_method = sys.argv[2]

    try:
        controller = TrajectoryController(
            serial_port=serial_port, interpolation_method=interpolation_method
        )
        controller.run_interactive()
    except ImportError:
        print("Error: scipy is required for cubic spline interpolation")
        print("Install with: pip install scipy")
        print("Falling back to linear interpolation...")
        controller = TrajectoryController(serial_port=serial_port, interpolation_method="linear")
        controller.run_interactive()
    except serial.SerialException as e:
        print(f"Serial connection error: {e}")
        print("Make sure the OpenRB is connected and the correct port is specified")
    except Exception as e:
        print(f"Error: {e}")
