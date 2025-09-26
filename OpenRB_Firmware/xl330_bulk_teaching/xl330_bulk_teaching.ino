#include <Dynamixel2Arduino.h>
#include <SparkFunMPU9250-DMP.h>

// #if defined(__OPENCR__)
// Dynamixel2Arduino dxl(Serial3, 84);
// #else
Dynamixel2Arduino dxl(Serial1, -1);
// #endif

MPU9250_DMP imu;

// CRC16-CCITT lookup table for fast calculation (polynomial: 0x1021)
const uint16_t CRC16_TABLE[256] PROGMEM = {
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
};

// CRC error statistics
uint32_t crc_error_count = 0;
uint32_t total_packets_received = 0;

// Calculate CRC16-CCITT over data bytes
uint16_t calculateCRC16(const uint8_t* data, size_t length) {
  uint16_t crc = 0x0000;  // Initial value for CRC16-CCITT
  
  for (size_t i = 0; i < length; i++) {
    uint8_t tbl_idx = ((crc >> 8) ^ data[i]) & 0xFF;
    crc = (pgm_read_word(&CRC16_TABLE[tbl_idx]) ^ (crc << 8)) & 0xFFFF;
  }
  
  return crc;
}

// motor ID 1, 3 direction should be inverted
// Offset should be added for target positions, substracted for encoder readings
// motor 3: +0.78 | motor 4: -0.78 | motor 5: -1.57 | motor 6: +1.57 
float zero_positions[18] = {0.0};

// Operating mode control
enum RobotOperatingMode {
  NORMAL_MODE,
  TEACHING_MODE
};
RobotOperatingMode current_mode = NORMAL_MODE;

// Teaching mode parameters
const float MOVE_THRESHOLD = 0.2;
float kp_impedance = 15.0;
float kd_impedance = 2.0;
float prev_positions[18] = {0};
float prev_velocities[18] = {0};
unsigned long lastTeachingTime = 0;
const unsigned long TEACHING_INTERVAL = 20000; // 50Hz for teaching mode

// Timing constants for normal mode (50Hz)
const unsigned long ENCODER_INTERVAL = 20000; // 20ms = 50Hz
const unsigned long IMU_INTERVAL = 20000;      // 20ms = 50Hz
const unsigned long IMU_OFFSET = 0;        // No offset to stagger with encoder

// Timing variables
unsigned long lastEncoderTime = 0;
unsigned long lastIMUTime = IMU_OFFSET; // Start with offset to stagger timing

// Pre-calculated conversion constants for faster processing
const float ACCEL_TO_MS2 = 9.80665f;        // g to m/s²
const float GYRO_DEG_TO_RAD = 3.141592653589793f / 180.0f;  // deg/s to rad/s
const float POS_SCALE = 0.088f / 180.0f * 3.141592653589793f;  // Position conversion: 0.088 * PI/180
const float VEL_SCALE = 0.229f * 2.0f * 3.141592653589793f / 60.0f;  // Velocity conversion
const double RAD_TO_MOTOR_UNITS = 180.0 / 3.141592653589793 / 0.088;  // radians -> motor units conversion

// Serial command processing buffer
uint8_t serialBuffer[512];
size_t bufferIndex = 0;

// Protocol 2.0 Control Table Addresses for XL330-M288
#define ADDR_GOAL_POSITION      116
#define ADDR_PRESENT_POSITION   132
#define ADDR_PRESENT_VELOCITY   128
#define LEN_GOAL_POSITION       4
#define LEN_PRESENT_POSITION    4
#define LEN_PRESENT_VELOCITY    4

// Instructions
ParamForBulkReadInst_t bulk_read_positions_param;
ParamForBulkReadInst_t bulk_read_velocities_param;
RecvInfoFromStatusInst_t positions_read_result;
RecvInfoFromStatusInst_t velocities_read_result;
ParamForSyncWriteInst_t sync_write_param;

typedef struct {
  float ax, ay, az;
  float gx, gy, gz;
  float qx, qy, qz, qw;
} imuData_t;

void sendWithByteStuffing(uint8_t* data, size_t length) {
  const uint8_t header[] = {0xFF, 0xFF, 0xFD, 0x00};
  uint8_t packet[512]; // Buffer for complete packet
  size_t packet_index = 0;
  
  // Add header to packet
  for (int i = 0; i < 4; i++) {
    packet[packet_index++] = header[i];
  }
  
  // Reserve space for length (will be filled later)
  size_t length_pos = packet_index;
  packet[packet_index++] = 0; // Length low byte
  packet[packet_index++] = 0; // Length high byte
  
  // Calculate CRC16 over original data
  uint16_t crc = calculateCRC16(data, length);
  
  // Create data with CRC16 appended
  uint8_t data_with_crc[512];
  memcpy(data_with_crc, data, length);
  data_with_crc[length] = crc & 0xFF;         // CRC low byte
  data_with_crc[length + 1] = (crc >> 8) & 0xFF; // CRC high byte
  size_t total_length = length + 2; // Original data + CRC16
  
  // Add data with CRC and byte stuffing to packet
  size_t data_start = packet_index;
  for (size_t i = 0; i < total_length; i++) {
    packet[packet_index++] = data_with_crc[i];
    
    // Check if we just completed the pattern 0xFF 0xFF 0xFD in the DATA portion only
    // Only check within the data section (after header + length)
    size_t data_pos = packet_index - data_start; // Position within data
    if (data_pos >= 3 && 
        packet[packet_index-3] == 0xFF && 
        packet[packet_index-2] == 0xFF && 
        packet[packet_index-1] == 0xFD) {
      // Add stuffing byte 0xFD to make it 0xFF 0xFF 0xFD 0xFD
      packet[packet_index++] = 0xFD;
    }
  }
  
  // Calculate stuffed data length and insert into packet
  uint16_t stuffed_length = packet_index - data_start;
  
  packet[length_pos] = stuffed_length & 0xFF;     // Low byte
  packet[length_pos + 1] = (stuffed_length >> 8) & 0xFF; // High byte
  
  // Send complete packet at once
  Serial.write(packet, packet_index);
}

bool findHeader(uint8_t* buffer, size_t buffer_size, size_t& header_pos) {
  const uint8_t header[] = {0xFF, 0xFF, 0xFD, 0x00};
  
  for (size_t i = 0; i <= buffer_size - 4; i++) {
    bool found = true;
    for (int j = 0; j < 4; j++) {
      if (buffer[i + j] != header[j]) {
        found = false;
        break;
      }
    }
    
    if (found) {
      // Check if this is actually stuffed data (0xFF 0xFF 0xFD 0xFD)
      if (i + 4 < buffer_size && buffer[i + 4] == 0xFD) {
        // This is stuffed data, not a real header
        continue;
      } else {
        header_pos = i;
        return true;
      }
    }
  }
  return false;
}

void destuffData(uint8_t* data, size_t data_size, uint8_t* output, size_t& output_size) {
  output_size = 0;
  size_t i = 0;
  
  while (i < data_size) {
    if (i <= data_size - 4 && 
        data[i] == 0xFF && data[i+1] == 0xFF && 
        data[i+2] == 0xFD && data[i+3] == 0xFD) {
      // Remove stuffing: add 0xFF 0xFF 0xFD, skip the extra 0xFD
      output[output_size++] = 0xFF;
      output[output_size++] = 0xFF;
      output[output_size++] = 0xFD;
      i += 4;
    } else {
      output[output_size++] = data[i];
      i++;
    }
  }
}

void processTargetPositionPacket(uint8_t* destuffed_data, size_t data_size) {
  total_packets_received++;
  
  // Expected: ID(1) + 18 positions(72) + CRC16(2) = 75 bytes minimum
  if (data_size < 75) {
    crc_error_count++;
    return; // Invalid packet size
  }
  
  uint8_t packet_id = destuffed_data[0];
  if (packet_id != 0x03) {
    return; // Not a target position packet
  }
  
  // Validate CRC16
  size_t payload_size = data_size - 2; // Data without CRC
  uint16_t received_crc = destuffed_data[data_size-2] | (destuffed_data[data_size-1] << 8);
  uint16_t calculated_crc = calculateCRC16(destuffed_data, payload_size);
  
  if (received_crc != calculated_crc) {
    crc_error_count++;
    return; // CRC mismatch
  }
  
  // Extract 18 target positions (float values) from payload
  float target_positions[18];
  for (int i = 0; i < 18; i++) {
    memcpy(&target_positions[i], &destuffed_data[1 + i * 4], 4);
  }
  
  // Apply target positions to motors with offset compensation and direction inversion
  for (int i = 0; i < 18; i++) {
    int motorID = i + 1;
    
    float target_pos = target_positions[i];
    
    // Convert from radians to motor units with proper rounding
    int32_t motor_position = (int32_t)((target_pos * RAD_TO_MOTOR_UNITS) + 2048.5);
    
    // Set target position for motor
    sync_write_param.xel[i].id = motorID;
    memcpy(sync_write_param.xel[i].data, &motor_position, sizeof(motor_position));
  }
  dxl.syncWrite(sync_write_param);
}

void processTorqueEnablePacket(uint8_t* destuffed_data, size_t data_size) {
  total_packets_received++;
  
  // Expected: ID(1) + enable_flag(1) + CRC16(2) = 4 bytes minimum
  if (data_size < 4) {
    crc_error_count++;
    return; // Invalid packet size
  }
  
  uint8_t packet_id = destuffed_data[0];
  if (packet_id != 0x04) {
    return; // Not a torque enable packet
  }
  
  // Validate CRC16
  size_t payload_size = data_size - 2; // Data without CRC
  uint16_t received_crc = destuffed_data[data_size-2] | (destuffed_data[data_size-1] << 8);
  uint16_t calculated_crc = calculateCRC16(destuffed_data, payload_size);
  
  if (received_crc != calculated_crc) {
    crc_error_count++;
    return; // CRC mismatch
  }
  
  uint8_t enable_flag = destuffed_data[1];
  
  // Apply torque enable/disable to all motors
  for (int i = 1; i <= 18; i++) {
    if (enable_flag == 1) {
      dxl.torqueOn(i);
    } else {
      dxl.torqueOff(i);
    }
  }
}

void processTeachingModePacket(uint8_t* destuffed_data, size_t data_size) {
  total_packets_received++;
  
  // Expected: ID(1) + enable_flag(1) + CRC16(2) = 4 bytes minimum
  if (data_size < 4) {
    crc_error_count++;
    return; // Invalid packet size
  }
  
  uint8_t packet_id = destuffed_data[0];
  if (packet_id != 0x06) {
    return; // Not a teaching mode packet
  }
  
  // Validate CRC16
  size_t payload_size = data_size - 2; // Data without CRC
  uint16_t received_crc = destuffed_data[data_size-2] | (destuffed_data[data_size-1] << 8);
  uint16_t calculated_crc = calculateCRC16(destuffed_data, payload_size);
  
  if (received_crc != calculated_crc) {
    crc_error_count++;
    return; // CRC mismatch
  }
  
  uint8_t enable_flag = destuffed_data[1];
  
  if (enable_flag == 1 && current_mode != TEACHING_MODE) {
    // Switch to teaching mode
    current_mode = TEACHING_MODE;
    
    // Initialize teaching mode parameters
    for (int i = 0; i < 18; i++) {
      prev_positions[i] = getCurrentPosition(i+1);
      prev_velocities[i] = getCurrentVelocity(i+1);
    }
    
    Serial.println("Switched to TEACHING MODE - motors are compliant");
    
  } else if (enable_flag == 0 && current_mode != NORMAL_MODE) {
    // Switch to normal mode
    current_mode = NORMAL_MODE;
    Serial.println("Switched to NORMAL MODE - trajectory control active");
  }
}

void processIncomingCommands() {
  
  // Read available serial data into buffer
  while (Serial.available() > 0 && bufferIndex < sizeof(serialBuffer) - 1) {
    serialBuffer[bufferIndex++] = Serial.read();
  }
  
  // Process complete packets in buffer
  while (bufferIndex >= 6) { // Minimum packet size: header(4) + length(2)
    size_t header_pos;
    if (!findHeader(serialBuffer, bufferIndex, header_pos)) {
      // No header found, keep last 3 bytes in case header is split
      if (bufferIndex > 3) {
        memmove(serialBuffer, serialBuffer + bufferIndex - 3, 3);
        bufferIndex = 3;
      }
      break;
    }
    
    // Move header to beginning if not already there
    if (header_pos > 0) {
      memmove(serialBuffer, serialBuffer + header_pos, bufferIndex - header_pos);
      bufferIndex -= header_pos;
    }
    
    // Need at least header + length field
    if (bufferIndex < 6) {
      break;
    }
    
    // Read packet length (little endian)
    uint16_t packet_length = serialBuffer[4] | (serialBuffer[5] << 8);
    size_t expected_size = 4 + 2 + packet_length; // header + length + data
    
    // Wait for complete packet
    if (bufferIndex < expected_size) {
      break;
    }
    
    // Extract and destuff packet data
    uint8_t destuffed_data[256];
    size_t destuffed_size;
    destuffData(serialBuffer + 6, packet_length, destuffed_data, destuffed_size);
    
    // Process packet based on type
    if (destuffed_size > 0) {
      uint8_t packet_id = destuffed_data[0];
      if (packet_id == 0x03) {
        processTargetPositionPacket(destuffed_data, destuffed_size);
      }
      else if (packet_id == 0x04) {
        processTorqueEnablePacket(destuffed_data, destuffed_size);
      }
      else if (packet_id == 0x06) {
        processTeachingModePacket(destuffed_data, destuffed_size);
      }
    }
    
    // Remove processed packet from buffer
    memmove(serialBuffer, serialBuffer + expected_size, bufferIndex - expected_size);
    bufferIndex -= expected_size;
  }
}

void runTeachingMode(unsigned long currentTime) {
  // Only run teaching mode at specified interval
  if (currentTime - lastTeachingTime < TEACHING_INTERVAL) {
    return;
  }
  lastTeachingTime = currentTime;
  
  // Implement impedance control for compliant teaching
  for (int i = 0; i < 18; i++) {
    float current_pos = getCurrentPosition(i+1);
    float current_vel = getCurrentVelocity(i+1);
    if (abs(current_vel) < MOVE_THRESHOLD)
      continue;

    // Calculate force based on position and velocity differences
    float pos_error = current_pos - prev_positions[i];
    float vel_error = current_vel - prev_velocities[i];
    
    // Calculate next position
    float impedance_torque = -kp_impedance * pos_error - kd_impedance * vel_error;
    
    float target_pos = current_pos + (impedance_torque * 0.001); // Small adjustment
    
    // Set goal position
    dxl.setGoalPosition(i+1, target_pos * RAD_TO_MOTOR_UNITS + 2048);
    
    prev_positions[i] = current_pos;
    prev_velocities[i] = current_vel;
  }
}

float getCurrentPosition(int motor_id) {
  return (dxl.getPresentPosition(motor_id) - 2048) * POS_SCALE;
}

float getCurrentVelocity(int motor_id) {
  return dxl.getPresentVelocity(motor_id) * VEL_SCALE;
}

void initBulkReadCommand(){

  bulk_read_positions_param.id_count = 18;
  for (int i=0; i<18; i++){
    bulk_read_positions_param.xel[i].id = i+1;
    bulk_read_positions_param.xel[i].addr = ADDR_PRESENT_POSITION;
    bulk_read_positions_param.xel[i].length = LEN_PRESENT_POSITION;
  }
  
  bulk_read_velocities_param.id_count = 18;
  for (int i=0; i<18; i++){
    bulk_read_velocities_param.xel[i].id = i+1;
    bulk_read_velocities_param.xel[i].addr = ADDR_PRESENT_VELOCITY;
    bulk_read_velocities_param.xel[i].length = LEN_PRESENT_VELOCITY;
  }
}

void initSyncWriteCommand(){
  sync_write_param.addr = ADDR_GOAL_POSITION;
  sync_write_param.length = LEN_GOAL_POSITION;
  sync_write_param.id_count = 18;
}


void setup() {
  
  // Start serial and wait for it
  // TODO: Seperate debug serial and data transfer serial (Serial, Serial2)
  Serial.begin(921600);
  while (!Serial) ;
  dxl.setPortProtocolVersionUsingIndex(2);
  dxl.begin(1000000);
  delay(2000);

  Serial.println("Creating command");
  initBulkReadCommand();
  initSyncWriteCommand();

  Serial.println("Connecting to dynamixels");
  // Wait until all dynamixels are connected
  for (int i=1; i<=18; i++){
    while (!dxl.getTorqueEnableStat(i)){
      dxl.torqueOn(i);
      delay(5);
    }
    dxl.ledOn(i);
  }

  // Turn off all leds for indicating dynamixel connections
  for (int i=1; i<=18; i++){
    dxl.ledOff(i);
  }

  // Initialize teaching mode parameters
  for (int i = 0; i < 18; i++) {
    prev_positions[i] = getCurrentPosition(i+1);
    prev_velocities[i] = getCurrentVelocity(i+1);
  }

  // I2C clock
  Wire.setClock(400000);
  
  // Start IMU in DMP mode
  imu.begin();
  // Initialize the digital motion processor
  imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL | // Send accelerometer data
                DMP_FEATURE_GYRO_CAL       | // Calibrate the gyro data
                DMP_FEATURE_SEND_CAL_GYRO  | // Send calibrated gyro data
                DMP_FEATURE_6X_LP_QUAT     , // Calculate quat's with accel/gyro
                50);  //Sample rate
                
  Serial.println("System Initialized - Default: NORMAL MODE");
  Serial.println("Send packet 0x06 with flag 1 to enable TEACHING MODE");
  Serial.println("CRC16 validation enabled for all incoming packets");
}

void loop() {
  unsigned long currentTime = micros();
  
  // Process incoming commands (always active)
  if (Serial.available() > 0 || bufferIndex >= 6) {
    processIncomingCommands();
  }
  
  if (current_mode == TEACHING_MODE) {
    runTeachingMode(currentTime);
  } else {
    // Normal mode: run trajectory control with IMU and encoder data transmission
    
    // Send encoder data at 50Hz (every 20ms)
    if (currentTime - lastEncoderTime >= ENCODER_INTERVAL) {
      
      //Get positions and velocities
      struct __attribute__((packed)) {
        uint8_t id = 0x01;
        float positions[18];
        float velocities[18];
      } encoderData;
      
      dxl.bulkRead(bulk_read_positions_param, positions_read_result);
      dxl.bulkRead(bulk_read_velocities_param, velocities_read_result);

      for (int i = 0; i < 18; i++) {
        int motorID = i + 1;
        
        int16_t raw_pos_val, raw_vel_val;
        memcpy(&raw_pos_val, positions_read_result.xel[i].data, positions_read_result.xel[i].length);
        memcpy(&raw_vel_val, velocities_read_result.xel[i].data, velocities_read_result.xel[i].length);
        
        float raw_pos = (raw_pos_val - 2048) * POS_SCALE;
        float raw_vel = (raw_vel_val) * VEL_SCALE;
        encoderData.positions[i] = raw_pos;
        encoderData.velocities[i] = raw_vel;
      }
      
      sendWithByteStuffing((uint8_t*)&encoderData, sizeof(encoderData));
      lastEncoderTime = currentTime;
    }
    
    // Send IMU data at 50Hz (every 20ms) with offset from encoder
    if (currentTime - lastIMUTime >= IMU_INTERVAL) {
      // Get IMU data
      if (imu.fifoAvailable() >= 28) { // Check for new data in the FIFO
        if (imu.dmpUpdateFifo() == INV_SUCCESS) {   
          
          struct __attribute__((packed)) {
            uint8_t id = 0x02;
            float ax, ay, az;
            float gx, gy, gz;
            float qx, qy, qz, qw;
          } imuData;
          
          // acceleration: g -> m/s² (using pre-calculated constant)
          imuData.ax = imu.calcAccel(imu.ax) * ACCEL_TO_MS2;
          imuData.ay = imu.calcAccel(imu.ay) * ACCEL_TO_MS2;
          imuData.az = imu.calcAccel(imu.az) * ACCEL_TO_MS2;
          // gyro: deg/s -> rad/s (using pre-calculated constant)
          imuData.gx = imu.calcGyro(imu.gx) * GYRO_DEG_TO_RAD;
          imuData.gy = imu.calcGyro(imu.gy) * GYRO_DEG_TO_RAD;
          imuData.gz = imu.calcGyro(imu.gz) * GYRO_DEG_TO_RAD;
          imuData.qx = imu.calcQuat(imu.qx);
          imuData.qy = imu.calcQuat(imu.qy);
          imuData.qz = imu.calcQuat(imu.qz);
          imuData.qw = imu.calcQuat(imu.qw);

          sendWithByteStuffing((uint8_t*)&imuData, sizeof(imuData));
          lastIMUTime = currentTime;
        }
      }
    }
  }
}