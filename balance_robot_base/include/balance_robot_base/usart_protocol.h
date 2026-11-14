#ifndef BALANCE_PROTOCOL_H
#define BALANCE_PROTOCOL_H

#include <cstdint>

// --- Communication-protocol constants (corresponding to STM32 usart3.h) STM32 usart3.h) ---
#define FRAME_HEADER      0X7B       // Frame header
#define FRAME_TAIL        0X7D       // Frame footer

// Receive-data length (STM32 -> ROS2) (STM32 -> ROS2)
#define RECEIVE_DATA_SIZE 32    

// Send-data length (ROS2 -> STM32) (ROS2 -> STM32)
#define SEND_DATA_SIZE    11        

// Checksum mode
#define CHECK_MODE_RECEIVE 0
#define CHECK_MODE_SEND    1

// --- Physical-parameter definitions ---
#define PI 3.1415926535f

// --- Data-structure definitions ---

// 1. Velocity and position data (used for odometry calculation))
struct VelPosData {
    float x;
    float y;
    float z; // Angle or angular velocity
};

// 2. IMU Raw IMU data structure (corresponding to short-type data sent by STM32))
struct Mpu6050RawData {
    int16_t accel_x; 
    int16_t accel_y; 	
    int16_t accel_z; 
    int16_t gyro_x; 
    int16_t gyro_y; 	
    int16_t gyro_z; 
};

// 3. Receive-data union/structure (STM32 -> ROS2) (STM32 -> ROS2)
// Buffer structure:
// [0]: Header (0x7B)
// [1]: StopFlag (Software-disable flag)
// [2-3]: X_Speed (High, Low)
// [4-5]: Y_Speed (High, Low)
// [6-7]: Z_Speed (High, Low)
// [8-9]: Accel X
// [10-11]: Accel Y
// [12-13]: Accel Z
// [14-15]: Gyro X
// [16-17]: Gyro Y
// [18-19]: Gyro Z
// [20-21]: Voltage
// [22-23]: Pitch
// [24-25]: Roll
// [26-27]: Yaw
// [28]: Checksum
// [29]: Tail (0x7D)
struct ReceivePacket {
    uint8_t rx_buffer[RECEIVE_DATA_SIZE];
    uint8_t flag_stop;
    float x_speed;  
    float y_speed;  
    float z_speed;  
    float power_voltage;
    float pitch;
    float roll;
    float yaw;
};

// 4. Send-data structure (ROS2 -> STM32) (ROS2 -> STM32)
struct SendPacket {
    uint8_t tx_buffer[SEND_DATA_SIZE];
    // Temporarily stores values before conversion
    int16_t x_speed_mm; 
    int16_t y_speed_mm;
    int16_t z_speed_mm;
};

#endif // BALANCE_PROTOCOL_H
