import time
import struct
import numpy as np

# fmt: off
# CRC16-CCITT lookup table for fast calculation (polynomial: 0x1021)
CRC16_TABLE = [
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
]
# fmt: on

# CRC error statistics
crc_error_count = 0
total_packets_received = 0


def calculate_crc16(data):
    """Calculate CRC16-CCITT over data bytes"""
    crc = 0x0000  # Initial value for CRC16-CCITT

    for byte in data:
        tbl_idx = ((crc >> 8) ^ byte) & 0xFF
        crc = (CRC16_TABLE[tbl_idx] ^ (crc << 8)) & 0xFFFF

    return crc


def find_header(data):
    """Find header position in data, excluding stuffed patterns"""
    header = b"\xff\xff\xfd\x00"
    pos = 0
    while True:
        pos = data.find(header, pos)
        if pos == -1:
            return -1

        # Check if this is actually stuffed data (0xFF 0xFF 0xFD 0xFD)
        if pos + 4 < len(data) and data[pos + 4] == 0xFD:
            # This is stuffed data, not a real header
            pos += 1
            continue
        else:
            # Found real header
            return pos


def destuff_data(data):
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
            # Remove stuffing: add 0xFF 0xFF 0xFD, skip the extra 0xFD
            destuffed.extend([0xFF, 0xFF, 0xFD])
            i += 4
        else:
            destuffed.append(data[i])
            i += 1
    return bytes(destuffed)


def parse_encoder_packet(packet_data):
    """Parse encoder packet: ID(1) + positions(18*4) + velocities(18*4) + CRC16(2)"""
    global total_packets_received, crc_error_count
    total_packets_received += 1

    # Minimum size: ID(1) + 18 positions(72) + 18 velocities(72) + CRC16(2) = 147 bytes
    if len(packet_data) < 147:
        print(f"Encoder packet too short: {len(packet_data)} < 147")
        crc_error_count += 1
        return None

    packet_id = packet_data[0]
    if packet_id != 0x01:  # Encoder ID
        print(f"Wrong encoder packet ID: expected 0x01, got 0x{packet_id:02X}")
        return None

    # Validate CRC16
    payload_size = len(packet_data) - 2  # Data without CRC
    received_crc = struct.unpack("<H", packet_data[-2:])[0]
    calculated_crc = calculate_crc16(packet_data[:payload_size])

    if received_crc != calculated_crc:
        print(
            f"Encoder packet CRC mismatch: received 0x{received_crc:04X}, calculated 0x{calculated_crc:04X}"
        )
        crc_error_count += 1
        return None

    try:
        # Parse data payload (excluding CRC16)
        payload_data = packet_data[:-2]  # Exclude CRC16

        # Use actual payload length for parsing
        expected_floats = (len(payload_data) - 1) // 4  # -1 for ID byte, /4 for float size
        if expected_floats != 36:  # Should be 18 pos + 18 vel
            print(f"Unexpected number of floats: {expected_floats}, expected 36")
            # Try to parse anyway with available data
            available_floats = min(expected_floats, 36)
            positions_count = min(18, available_floats)
            velocities_count = min(18, available_floats - positions_count)
        else:
            positions_count = 18
            velocities_count = 18

        # Unpack positions
        positions = struct.unpack(f"<{positions_count}f", payload_data[1 : 1 + positions_count * 4])
        # Pad with zeros if needed
        if positions_count < 18:
            positions = positions + (0.0,) * (18 - positions_count)

        # Unpack velocities
        vel_start = 1 + 18 * 4  # After ID and 18 positions
        velocities = struct.unpack(
            f"<{velocities_count}f", payload_data[vel_start : vel_start + velocities_count * 4]
        )
        # Pad with zeros if needed
        if velocities_count < 18:
            velocities = velocities + (0.0,) * (18 - velocities_count)

        # Validate data ranges (positions should be reasonable)
        pos_array = np.array(positions)
        vel_array = np.array(velocities)

        return {
            "timestamp": time.time(),
            "positions": pos_array,
            "velocities": vel_array,
        }
    except struct.error as e:
        print(f"Encoder struct unpack error: {e}")
        return None


def parse_imu_packet(packet_data):
    """Parse IMU packet: ID(1) + ax,ay,az,gx,gy,gz,qx,qy,qz,qw(10*4) + CRC16(2)"""
    global total_packets_received, crc_error_count
    total_packets_received += 1

    # Minimum size: ID(1) + 10 floats(40) + CRC16(2) = 43 bytes
    if len(packet_data) < 43:
        print(f"IMU packet too short: {len(packet_data)} < 43")
        crc_error_count += 1
        return None

    packet_id = packet_data[0]
    if packet_id != 0x02:  # IMU ID
        print(f"Wrong IMU packet ID: expected 0x02, got 0x{packet_id:02X}")
        return None

    # Validate CRC16
    payload_size = len(packet_data) - 2  # Data without CRC
    received_crc = struct.unpack("<H", packet_data[-2:])[0]
    calculated_crc = calculate_crc16(packet_data[:payload_size])

    if received_crc != calculated_crc:
        print(
            f"IMU packet CRC mismatch: received 0x{received_crc:04X}, calculated 0x{calculated_crc:04X}"
        )
        crc_error_count += 1
        return None

    try:
        # Parse data payload (excluding CRC16)
        payload_data = packet_data[:-2]  # Exclude CRC16

        # Use actual payload length for parsing
        available_floats = (len(payload_data) - 1) // 4  # -1 for ID byte
        floats_to_read = min(available_floats, 10)

        # Unpack available floats
        imu_values = struct.unpack(f"<{floats_to_read}f", payload_data[1 : 1 + floats_to_read * 4])

        # Pad with zeros if needed
        if floats_to_read < 10:
            imu_values = imu_values + (0.0,) * (10 - floats_to_read)

        return {
            "timestamp": time.time(),
            "accel": np.array(imu_values[0:3]),  # ax, ay, az
            "gyro": np.array(imu_values[3:6]),  # gx, gy, gz
            "quat": np.array(imu_values[6:10]),  # qx, qy, qz, qw
        }
    except struct.error as e:
        print(f"IMU struct unpack error: {e}")
        return None


def stuff_data(data):
    """Add byte stuffing: 0xFF 0xFF 0xFD -> 0xFF 0xFF 0xFD 0xFD"""
    stuffed = bytearray()
    i = 0
    while i < len(data):
        stuffed.append(data[i])

        # Check if we just completed the pattern 0xFF 0xFF 0xFD
        if (
            len(stuffed) >= 3
            and stuffed[-3] == 0xFF
            and stuffed[-2] == 0xFF
            and stuffed[-1] == 0xFD
        ):
            # Add stuffing byte 0xFD to make it 0xFF 0xFF 0xFD 0xFD
            stuffed.append(0xFD)

        i += 1
    return bytes(stuffed)


def send_target_positions_binary(port, target_positions):
    """Send target positions using binary protocol with packet type 0x03"""
    # Create packet data: ID(1) + 18 target positions(18*4) = 73 bytes
    packet_data = bytearray()
    packet_data.append(0x03)  # Target position packet ID

    # Add 18 float target positions (little endian)
    for pos in target_positions:
        packet_data.extend(struct.pack("<f", pos))

    # Calculate and append CRC16
    crc = calculate_crc16(packet_data)
    packet_data.extend(struct.pack("<H", crc))  # Little endian CRC16

    # Apply byte stuffing to data + CRC
    stuffed_data = stuff_data(packet_data)

    # Create complete packet: header + length + stuffed_data
    header = b"\xff\xff\xfd\x00"
    length = struct.pack("<H", len(stuffed_data))  # Little endian length
    complete_packet = header + length + stuffed_data

    # Send packet
    port.write(complete_packet)


def send_torque_enable(port, enable):
    """Send torque enable/disable command to robot"""
    packet_data = bytearray()
    packet_data.append(0x04)  # Torque enable packet ID
    packet_data.append(1 if enable else 0)  # Enable flag

    # Calculate and append CRC16
    crc = calculate_crc16(packet_data)
    packet_data.extend(struct.pack("<H", crc))  # Little endian CRC16

    stuffed_data = stuff_data(packet_data)
    header = b"\xff\xff\xfd\x00"
    length = struct.pack("<H", len(stuffed_data))
    complete_packet = header + length + stuffed_data

    port.write(complete_packet)
    return enable


def send_teaching_mode_enable(port, enable):
    """Send teaching mode enable/disable command to robot"""
    packet_data = bytearray()
    packet_data.append(0x06)  # Teaching mode packet ID
    packet_data.append(1 if enable else 0)  # Enable flag

    # Calculate and append CRC16
    crc = calculate_crc16(packet_data)
    packet_data.extend(struct.pack("<H", crc))  # Little endian CRC16

    stuffed_data = stuff_data(packet_data)
    header = b"\xff\xff\xfd\x00"
    length = struct.pack("<H", len(stuffed_data))
    complete_packet = header + length + stuffed_data

    port.write(complete_packet)
    return enable


def parse_timing_packet(packet_data):
    """Parse timing stats packet: ID(1) + 3 uint32(12) = 13 bytes"""
    # Expected size: ID(1) + 3 uint32(12) = 13 bytes
    if len(packet_data) < 13:
        return None

    packet_id = packet_data[0]
    if packet_id != 0x54:  # Timing stats ID
        return None

    try:
        # Unpack: 3 uint32 (little endian)
        values = struct.unpack("<3I", packet_data[1:13])

        return {
            "timestamp": time.time(),
            "cmd_process_time_us": values[0],
            "target_apply_time_us": values[1],
            "encoder_read_time_us": values[2],
        }
    except struct.error as e:
        print(f"Timing struct unpack error: {e}")
        return None


def print_timing_stats(timing_data, logger=None):
    """Print timing statistics from Arduino"""
    msg = (
        f"ARDUINO TIMING: CMD={timing_data['cmd_process_time_us']/1e6:.6f}s, "
        f"TARGET={timing_data['target_apply_time_us']/1e6:.6f}s, "
        f"ENCODER={timing_data['encoder_read_time_us']/1e6:.6f}s"
    )

    if logger:
        logger.info(msg)
    else:
        print(msg)


def get_crc_statistics():
    """Get CRC error statistics"""
    global crc_error_count, total_packets_received

    if total_packets_received == 0:
        error_rate = 0.0
    else:
        error_rate = (crc_error_count / total_packets_received) * 100.0

    return {
        "total_packets_received": total_packets_received,
        "crc_errors": crc_error_count,
        "error_rate_percent": error_rate,
        "success_rate_percent": 100.0 - error_rate,
    }


def print_crc_statistics(logger=None):
    """Print CRC error statistics"""
    stats = get_crc_statistics()
    msg = (
        f"CRC STATS: Total={stats['total_packets_received']}, "
        f"Errors={stats['crc_errors']}, "
        f"Error Rate={stats['error_rate_percent']:.3f}%, "
        f"Success Rate={stats['success_rate_percent']:.3f}%"
    )

    if logger:
        logger.info(msg)
    else:
        print(msg)


def reset_crc_statistics():
    """Reset CRC error statistics"""
    global crc_error_count, total_packets_received
    crc_error_count = 0
    total_packets_received = 0
