# Darwin Control System Documentation

## Overview

This documentation covers the complete communication protocol and system architecture for the humanoid robotics control system. The system provides real-time control of 18 Dynamixel servo motors with IMU feedback for autonomous behavior, trajectory execution, and compliant teaching modes.

## System Architecture

```
[Python Control Layer] ←→ [Serial 1MHz] ←→ [MCU Firmware] ←→ [18x Dynamixel Motors + IMU]
```

### Main Components

1. **Arduino Firmware**: 
   - `mx28_bulk_teaching.ino`
   
2. **Python Control Systems**:
   - `openRB_traj.py` - ROS2 integration and ML inference with trajectory planning and execution
   
3. **Packet Handling**:
   - `openrb/packet_handler.py` - Centralized CRC16 and protocol utilities

### Hardware Configuration
- **18 Dynamixel MX-28 servo motors** - Joint actuators with position/velocity feedback
- **MPU-9250 IMU** - 9-DOF sensor providing acceleration, gyroscope, and quaternion data
- **OpenRB-150 controller** - MCU (microcontroller unit)
- **Serial communication** at 1MHz baud rate for high-speed data transfer

## Binary Communication Protocol

The system uses a custom binary protocol with byte stuffing for data integrity.

### Packet Structure

All packets follow this format:
```
[Header: 4 bytes] [Length: 2 bytes] [Stuffed(Data + CRC16): variable]
```

#### Header Format
```
0xFF 0xFF 0xFD 0x00
```

#### Length Field
- **Format**: Little-endian 16-bit unsigned integer
- **Content**: Length of stuffed data portion (excluding header and length field)

#### CRC16 Error Detection
- **Algorithm**: CRC16-CCITT (polynomial: 0x1021)
- **Initial Value**: 0x0000
- **Lookup Table**: 256-entry table for fast calculation
- **Calculation**: Over original data payload (before stuffing, excluding CRC itself)
- **Placement**: Appended to data payload before byte stuffing
- **Size**: 2 bytes (little-endian format)
- **Error Detection**: 99.997% detection capability for data corruption

**CRC Calculation Process**:
1. Calculate CRC16 over data payload using lookup table
2. Append CRC16 to data (little-endian: low byte, high byte)
3. Apply byte stuffing to data+CRC16
4. Add header and length

**Lookup Table Implementation**:
- **Arduino**: Stored in PROGMEM for memory efficiency
- **Python**: Standard memory-based lookup table
- **Performance**: ~10-50μs calculation time (negligible at 50Hz)

#### Byte Stuffing Protocol
- **Purpose**: Prevent data from being mistaken for packet headers
- **Rule**: `0xFF 0xFF 0xFD` in data → `0xFF 0xFF 0xFD 0xFD`
- **Reversal**: `0xFF 0xFF 0xFD 0xFD` → `0xFF 0xFF 0xFD` (destuffing)
- **Applied To**: Data payload + CRC16 (after CRC calculation)

### Packet Types

| Packet ID | Direction | Length (bytes) | Data Field Description |
|-----------|-----------|----------------|------------------------|
| 0x01 | MCU → SBC | 147 | 18x Position floats + 18x Velocity floats + CRC16 |
| 0x02 | MCU → SBC | 43 | Linear acceleration (3 floats) + Angular velocity (3 floats) + Quaternion (4 floats) + CRC16 |
| 0x03 | SBC → MCU | 75 | 18x Target position floats + CRC16 |
| 0x04 | SBC → MCU | 4 | Torque enable flag (1 byte) + CRC16 |
| 0x06 | SBC → MCU | 4 | Teaching mode enable flag (1 byte) + CRC16 |

#### 1. Encoder Data Packet (0x01) - MCU → SBC

**Transmission Rate**: 50Hz (every 20ms)

**Structure**:
```
[ID: 0x01] [18x Position floats] [18x Velocity floats] [CRC16: 2 bytes]
```

**Data Layout**:
- **Packet ID**: `0x01` (1 byte)
- **Positions**: 18 × 4-byte floats (little-endian) - joint positions in radians
- **Velocities**: 18 × 4-byte floats (little-endian) - joint velocities in rad/s
- **CRC16**: 2-byte checksum (little-endian)
- **Total Size**: 147 bytes (1 + 72 + 72 + 2)

**Example Creation (MCU)**:
```cpp
struct __attribute__((packed)) {
    uint8_t id = 0x01;
    float positions[18];
    float velocities[18];
} encoderData;

sendWithByteStuffing((uint8_t*)&encoderData, sizeof(encoderData));
```

**Example Parsing (SBC)**:
```python
def parse_encoder_packet(packet_data):
    # Validate packet size and ID
    if len(packet_data) < 147 or packet_data[0] != 0x01:
        return None
    
    # Validate CRC16
    payload_size = len(packet_data) - 2
    received_crc = struct.unpack("<H", packet_data[-2:])[0]
    calculated_crc = calculate_crc16(packet_data[:payload_size])
    
    if received_crc != calculated_crc:
        print(f"Encoder CRC mismatch: received 0x{received_crc:04X}, calculated 0x{calculated_crc:04X}")
        return None
    
    # Parse validated data
    positions = struct.unpack("<18f", packet_data[1:73])
    velocities = struct.unpack("<18f", packet_data[73:145])
    return {"positions": positions, "velocities": velocities}
```

#### 2. IMU Data Packet (0x02) - MCU → SBC

**Transmission Rate**: 50Hz (every 20ms, offset by 10ms from encoder)

**Structure**:
```
[ID: 0x02] [ax,ay,az: 3 floats] [gx,gy,gz: 3 floats] [qx,qy,qz,qw: 4 floats] [CRC16: 2 bytes]
```

**Data Layout**:
- **Packet ID**: `0x02` (1 byte)
- **Linear Acceleration**: 3 × 4-byte floats (ax, ay, az in m/s²)
- **Angular Velocity**: 3 × 4-byte floats (gx, gy, gz in rad/s)
- **Quaternion**: 4 × 4-byte floats (qx, qy, qz, qw)
- **CRC16**: 2-byte checksum (little-endian)
- **Total Size**: 43 bytes (1 + 40 + 2)

**Example Creation (MCU)**:
```cpp
struct __attribute__((packed)) {
    uint8_t id = 0x02;
    float ax, ay, az;
    float gx, gy, gz;
    float qx, qy, qz, qw;
} imuData;

sendWithByteStuffing((uint8_t*)&imuData, sizeof(imuData));
```

#### 3. Target Position Command (0x03) - SBC → MCU

**Purpose**: Send target joint positions to robot

**Structure**:
```
[ID: 0x03] [18x Target Position floats] [CRC16: 2 bytes]
```

**Data Layout**:
- **Packet ID**: `0x03` (1 byte)
- **Target Positions**: 18 × 4-byte floats (little-endian) - target joint positions in radians
- **CRC16**: 2-byte checksum (little-endian)
- **Total Size**: 75 bytes (1 + 72 + 2)

**SBC Send Function**:
```python
def send_target_positions_binary(port, target_positions):
    packet_data = bytearray()
    packet_data.append(0x03)  # Packet ID
    
    # Add 18 target positions (little endian floats)
    for pos in target_positions:
        packet_data.extend(struct.pack("<f", pos))
    
    # Calculate and append CRC16
    crc = calculate_crc16(packet_data)
    packet_data.extend(struct.pack("<H", crc))  # Little endian CRC16
    
    # Apply byte stuffing to data + CRC
    stuffed_data = stuff_data(packet_data)
    header = b"\xff\xff\xfd\x00"
    length = struct.pack("<H", len(stuffed_data))
    complete_packet = header + length + stuffed_data
    
    port.write(complete_packet)
```

**MCU Processing**:
```cpp
void processTargetPositionPacket(uint8_t* destuffed_data, size_t data_size) {
    total_packets_received++;
    
    // Expected: ID(1) + 18 positions(72) + CRC16(2) = 75 bytes
    if (data_size < 75 || destuffed_data[0] != 0x03) {
        crc_error_count++;
        return;
    }
    
    // Validate CRC16
    size_t payload_size = data_size - 2;
    uint16_t received_crc = destuffed_data[data_size-2] | (destuffed_data[data_size-1] << 8);
    uint16_t calculated_crc = calculateCRC16(destuffed_data, payload_size);
    
    if (received_crc != calculated_crc) {
        crc_error_count++;
        return; // Silent drop on CRC mismatch
    }
    
    // Extract validated target positions
    float target_positions[18];
    for (int i = 0; i < 18; i++) {
        memcpy(&target_positions[i], &destuffed_data[1 + i * 4], 4);
    }
    
    // Apply positions to motors with offset and direction compensation
    // ... motor control logic
}
```

#### 4. Torque Enable/Disable (0x04) - SBC → MCU

**Purpose**: Enable or disable motor torque for all joints

**Structure**:
```
[ID: 0x04] [Enable Flag: 1 byte] [CRC16: 2 bytes]
```

**Data Layout**:
- **Packet ID**: `0x04` (1 byte)
- **Enable Flag**: 1 byte (1 = enable, 0 = disable)
- **CRC16**: 2-byte checksum (little-endian)
- **Total Size**: 4 bytes (1 + 1 + 2)

**SBC Send Function**:
```python
def send_torque_enable(port, enable):
    packet_data = bytearray([0x04, 1 if enable else 0])
    # ... packet construction and sending
```

#### 5. Teaching Mode Control (0x06) - SBC → MCU

**Purpose**: Enable/disable compliant teaching mode for direct manipulation

**Structure**:
```
[ID: 0x06] [Enable Flag: 1 byte] [CRC16: 2 bytes]
```

**Data Layout**:
- **Packet ID**: `0x06` (1 byte)  
- **Enable Flag**: 1 byte (1 = enable teaching, 0 = disable teaching)
- **CRC16**: 2-byte checksum (little-endian)
- **Total Size**: 4 bytes (1 + 1 + 2)

**MCU Teaching Mode Logic**:
```cpp
if (teaching) {
    // Impedance control for compliant behavior
    float impedance_torque = -kp_impedance * pos_error - kd_impedance * vel_error;
    float target_pos = current_pos + (impedance_torque * 0.001);
    dxl.setGoalPosition(i+1, (target_pos * 180.0 / 3.141592 / 0.088) + 2048);
}
```

### Teaching Mode Implementation (`mx28_bulk_teaching.ino`)

The teaching mode firmware provides compliant robot control for direct human manipulation and demonstration-based learning.

#### Dual Operating Modes
```cpp
enum RobotOperatingMode {
  NORMAL_MODE,    // Standard trajectory control with full stiffness
  TEACHING_MODE   // Compliant impedance control for direct manipulation
};
```

#### Mode Switching via Packet 0x06
- **Packet Structure**: `[0x06][enable_flag][CRC16]`
- **Mode Transition**: Automatic switching between normal and teaching modes
- **Safety Features**: Smooth parameter initialization during mode changes

#### Impedance Control Parameters
```cpp
float kp_impedance = 15.0;  // Position impedance gain
float kd_impedance = 2.0;   // Velocity damping gain
const unsigned long TEACHING_INTERVAL = 20000; // 50Hz update rate
```

#### Teaching Mode Control Loop
```cpp
void runTeachingMode(unsigned long currentTime) {
    for (int i = 0; i < 18; i++) {
        float current_pos = getCurrentPosition(i+1);
        float current_vel = getCurrentVelocity(i+1);
        
        // Skip stationary joints to prevent jitter
        if (abs(current_vel) < 0.2) {
            prev_positions[i] = current_pos;
            prev_velocities[i] = current_vel;
            continue;
        }
        
        // Calculate impedance control force
        float pos_error = current_pos - prev_positions[i];
        float vel_error = current_vel - prev_velocities[i];
        float impedance_torque = -kp_impedance * pos_error - kd_impedance * vel_error;
        
        // Apply compliant control with reduced stiffness
        float target_pos = current_pos + (impedance_torque * 0.001);
        dxl.setGoalPosition(i+1, (target_pos * 180.0 / 3.141592 / 0.088) + 2048);
        
        prev_positions[i] = current_pos;
        prev_velocities[i] = current_vel;
    }
}
```

#### Key Features
- **Compliant Response**: Reduced motor stiffness allows safe human interaction
- **Velocity Threshold**: Prevents control action on stationary joints (< 0.2 rad/s)
- **Impedance Control**: PD controller with configurable gains for natural feel
- **Real-time Operation**: 50Hz control loop maintains responsive behavior
- **Automatic Initialization**: Previous positions/velocities stored during mode transitions

## CRC Error Handling and Diagnostics

### Error Detection Behavior

#### **MCU Side (Packet Receiver)**:
When a CRC check fails on incoming packets:

```cpp
if (received_crc != calculated_crc) {
    crc_error_count++;
    return; // Silent drop - packet completely ignored
}
```

**Behavior**:
- ✅ **Silent Drop**: Corrupted packet is completely ignored (no dangerous commands executed)
- ✅ **Error Counter**: `crc_error_count` incremented for diagnostics
- ❌ **No Response**: SBC side doesn't know packet was dropped
- ✅ **Safety**: Motors maintain last valid position (no erratic movements)

#### **SBC Side (Packet Receiver)**:
When a CRC check fails on sensor data packets:

```python
if received_crc != calculated_crc:
    print(f"Encoder packet CRC mismatch: received 0x{received_crc:04X}, calculated 0x{calculated_crc:04X}")
    crc_error_count += 1
    return None
```

**Behavior**:
- ✅ **Silent Drop**: Packet parsing returns `None`
- ✅ **Console Logging**: Error message with CRC mismatch details
- ✅ **Error Counter**: Statistics maintained for monitoring
- ✅ **Graceful Degradation**: Uses cached sensor values to continue operation

### Current Limitations

#### **No Retransmission**:
- ❌ Dropped packets are never recovered
- ❌ Critical commands (emergency stop) could be lost silently
- ❌ No acknowledgment system for command verification

#### **No Flow Control**:
- ❌ Sender unaware if packets are being received
- ❌ Could lead to command queuing issues
- ❌ No backpressure mechanism for overwhelmed receiver

#### **Limited Real-time Diagnostics**:
- ❌ No real-time error reporting to ROS2 layer
- ❌ MCU diagnostics only via serial debug (if connected)
- ❌ No automatic recovery mechanisms

### Performance Impact

- **Overhead**: +2 bytes per packet (1.4% for encoder packets)
- **CPU Load**: ~10-50μs CRC calculation time (negligible at 50Hz)
- **Error Detection**: 99.997% capability for data corruption
- **Memory**: ~1KB for lookup tables on both sides
- **Success Rate**: Typically >99.9% in good RF conditions

## Advanced Control Systems

### Trajectory Controller (`trajectory_controller.py`)

The trajectory controller provides advanced trajectory planning and execution capabilities with precise timing control.

#### Key Features
- **Interpolation Methods**: Linear and cubic spline interpolation between waypoints
- **Execution Rate**: 50Hz trajectory execution with sub-millisecond timing precision
- **CRC16 Validation**: Error detection on all encoder data packets
- **Interactive Interface**: Command-line interface for trajectory management
- **Transition Smoothing**: Automatic smooth transitions from current pose to trajectory start
- **Joint Limiting**: Software enforcement of joint position limits during execution

#### Trajectory File Format
```
# Comments start with #
# Format: 22 joint positions (in sim_joint_names order) followed by time_from_start
j_pelvis_l j_pelvis_r j_shoulder_l ... j_gripper_l time_from_start
0.000000   0.000000   0.100000     ... -0.990000   0.000
0.100000   0.000000   0.200000     ... -0.990000   2.000
```

#### Interactive Commands
- `save [time]` - Save current joint positions as trajectory waypoint
- `load <filename> [time]` - Load and execute trajectory with optional transition time
- `load_w_interp <filename> <method> [time]` - Load with specific interpolation method
- `plot` - Generate visualization of loaded trajectory (requires matplotlib)
- `teaching` - Toggle compliant teaching mode for direct manipulation
- `crc` - Display CRC error statistics
- `reset_crc` - Reset CRC error counters

#### CRC Error Monitoring
```python
# Error statistics available in real-time
stats = get_crc_statistics()
print(f"Error rate: {stats['error_rate_percent']:.3f}%")
print(f"Success rate: {stats['success_rate_percent']:.3f}%")
```

### Packet Handler Utilities (`openrb/packet_handler.py`)

Centralized packet processing and CRC16 validation for reliable communication.

#### CRC16 Functions
```python
def calculate_crc16(data):
    """Calculate CRC16-CCITT over data bytes using lookup table"""
    crc = 0x0000
    for byte in data:
        tbl_idx = ((crc >> 8) ^ byte) & 0xFF
        crc = (CRC16_TABLE[tbl_idx] ^ (crc << 8)) & 0xFFFF
    return crc
```

#### Packet Parsing with Validation
- `parse_encoder_packet(packet_data)` - Validates CRC16 and extracts 18 positions + velocities
- `parse_imu_packet(packet_data)` - Validates CRC16 and extracts IMU data
- `parse_timing_packet(packet_data)` - Extracts Arduino timing statistics

#### Command Transmission
- `send_target_positions_binary(port, positions)` - Sends position commands with CRC16
- `send_torque_enable(port, enable)` - Sends torque control with CRC16
- `send_teaching_mode_enable(port, enable)` - Sends teaching mode control with CRC16

#### Error Statistics and Diagnostics
```python
def get_crc_statistics():
    return {
        "total_packets_received": total_packets_received,
        "crc_errors": crc_error_count,
        "error_rate_percent": error_rate,
        "success_rate_percent": 100.0 - error_rate
    }
```

## ROS2 Integration

### Node Information
- **Node Name**: `openrb150_trajectory`
- **Package**: Uses custom `hri_humanoid_interfaces` for services

### Published Topics

#### 1. Joint States (`/joint_states`)
- **Type**: `sensor_msgs/JointState`
- **Rate**: 50Hz (synchronized with encoder data reception)
- **Content**: Joint positions and velocities for all 22 joints

**Message Structure**:
```python
new_joint_state = JointState()
new_joint_state.name = self.sim_joint_names  # 22 joint names
new_joint_state.position = [...]  # Joint positions in radians
new_joint_state.velocity = [...]  # Joint velocities in rad/s
new_joint_state.header.stamp = rclpy.time.Time().to_msg()
```

#### 2. IMU Data (`/imu`)
- **Type**: `sensor_msgs/Imu`
- **Rate**: 50Hz (synchronized with IMU data reception)
- **Content**: Linear acceleration, angular velocity, and orientation quaternion

**Message Structure**:
```python
new_imu = Imu()
new_imu.linear_acceleration.x = imu[0]  # m/s²
new_imu.linear_acceleration.y = imu[1]
new_imu.linear_acceleration.z = imu[2]
new_imu.angular_velocity.x = imu[3]     # rad/s
new_imu.angular_velocity.y = imu[4]
new_imu.angular_velocity.z = imu[5]
new_imu.orientation.x = imu[6]          # quaternion
new_imu.orientation.y = imu[7]
new_imu.orientation.z = imu[8]
new_imu.orientation.w = imu[9]
```

### Subscribed Topics

#### 1. Behavior Control (`/behavior_id`)
- **Type**: `std_msgs/Int8`
- **Purpose**: Switch between ML inference mode and predefined trajectories
- **Values**:
  - `0`: ML inference mode (autonomous control)
  - `1-7`: Predefined trajectory behaviors (hello, crouch, stretch, etc.)

#### 2. Torque Control (`/torque_enable`)
- **Type**: `std_msgs/Bool`
- **Purpose**: Enable/disable motor torque
- **Effect**: Sends torque enable/disable packet to MCU

#### 3. Operation Mode (`/op_mod`)
- **Type**: `std_msgs/Int8`
- **Purpose**: Switch between normal operation and teaching mode
- **Values**:
  - `0`: Normal operation mode
  - `1`: Teaching mode (compliant for direct manipulation)

### Service Clients

#### ML Inference Service (`/get_actions`)
- **Type**: `hri_humanoid_interfaces/GetActions`
- **Purpose**: Get control actions from ML inference pipeline
- **Usage**: Only active when `behavior_id == 0`

**Service Call**:
```python
req = GetActions.Request()
req.observations = obs  # Combined sensor data
future = self.client_inference.call_async(req)
```

**Observation Vector Composition**:
```python
obs = np.concatenate([
    base_ang_vel,      # IMU angular velocity (3 values)
    projected_gravity, # Gravity vector in base frame (3 values) 
    command_velocity,  # Desired velocity command (3 values)
    joint_pos,         # Joint positions (18 values)
    joint_vel,         # Joint velocities (18 values)
    prev_action        # Previous action (18 values)
]).tolist()  # Total: 63 values
```

## Configuration System

### Joint Configuration (YAML)

The system uses a YAML configuration file to define joint mappings and limits:

```yaml
motor_index:
  j_pelvis_l: 8
  j_pelvis_r: 7
  j_shoulder_l: 2
  # ... (maps joint names to motor IDs 1-18)

sim_joint_names:
  - j_pelvis_l
  - j_pelvis_r
  # ... (22 joints including virtual joints 19-22)

default_joint_pos:
  j_pelvis_l: 0.0
  j_thigh2_l: 0.4
  # ... (default positions for each joint)

joint_pos_clip:
  j_pelvis_l: [-0.79, 2.62]  # [min, max] in radians
  # ... (safety limits for each joint)

trajectory_behaviors:
  1: "hello"
  2: "crouch"
  3: "stretch"
  # ... (maps behavior IDs to trajectory file names)
```

### Trajectory Files

Trajectory files are stored as text files with format:
```
# Comments start with #
pos1 pos2 pos3 ... pos22 timestamp
0.0  0.0  0.1  ... 0.0   1.0
0.1  0.0  0.2  ... 0.0   2.0
```

Each line contains:
- 22 joint positions (including virtual joints)
- Timestamp for this waypoint

## Motor Configuration and Calibration

### Direction Inversion
Motors 1 and 3 require direction inversion:
```cpp
if (i == 0 || i == 2) {  // Motors 1 and 3
    // Inverted direction
    encoderData.positions[i] = -(raw_pos - offset[i]);
    encoderData.velocities[i] = -raw_vel;
}
```

### Joint Offsets
Hardware calibration offsets compensate for mechanical assembly:
```cpp
float offset[] = {0.0,0.0,0.78,-0.78,-1.57,1.57,0.0,0.0,0.0,0.0,-0.0,0.0,0.0,-0.0,0.0,-0.0,0.0,0.0,0.0,0.0};
```

### Unit Conversions
- **Position**: Motor units (0-4095) ↔ Radians
- **Velocity**: Motor units ↔ rad/s
- **Acceleration**: g ↔ m/s²
- **Angular velocity**: deg/s ↔ rad/s

```cpp
const float POS_SCALE = 0.088f / 180.0f * PI;  // Motor units to radians
const float VEL_SCALE = 0.11f * 2.0f * PI / 60.0f;  // Motor velocity to rad/s
const double RAD_TO_MOTOR_UNITS = 180.0 / PI / 0.088;  // Radians to motor units
```

## Timing and Performance

### Data Transmission Rates
- **Encoder Data**: 50Hz (every 20ms)
- **IMU Data**: 50Hz (every 20ms, 10ms offset from encoder)
- **Control Commands**: Up to 50Hz (trajectory execution)

### Control Loop Timing
- **Main Control Loop**: 50Hz (0.02s)
- **Serial Processing**: Continuous with 1ms sleep
- **Trajectory Interpolation**: 50Hz points generated from waypoints

## Error Handling and Safety

### Joint Limits
Software joint limits prevent dangerous positions:
```python
if joint_name in self.joint_pos_clip:
    min_pos, max_pos = self.joint_pos_clip[joint_name]
    target_pos = np.clip(target_pos, min_pos, max_pos)
```

### Packet Validation
- Header validation with byte stuffing detection
- Packet size verification
- Data range validation for sensor readings

### Fault Recovery
- Automatic reconnection attempts for serial communication
- Fallback to cached values if data queues are empty
- Teaching mode safety with reduced motor stiffness

## Usage Examples

### Basic Robot Control
```python
# Initialize node
rclpy.init()
node = OpenRB150()

# The node automatically starts:
# 1. Serial communication thread
# 2. 50Hz control loop timer
# 3. ROS2 topic publishers/subscribers

rclpy.spin(node)
```

### Switching to Teaching Mode
```bash
# Enable teaching mode (compliant for direct manipulation)
ros2 topic pub /op_mod std_msgs/Int8 "data: 1"

# Return to normal mode
ros2 topic pub /op_mod std_msgs/Int8 "data: 0"
```

### Executing Predefined Trajectories
```bash
# Execute "hello" trajectory (behavior ID 1)
ros2 topic pub /behavior_id std_msgs/Int8 "data: 1"

# Return to ML inference mode
ros2 topic pub /behavior_id std_msgs/Int8 "data: 0"
```

### Monitoring Robot State
```bash
# View joint states
ros2 topic echo /joint_states

# View IMU data
ros2 topic echo /imu
```


This documentation provides a complete reference for understanding and working with the OpenRB-150 trajectory control system's communication protocol and ROS2 integration.


