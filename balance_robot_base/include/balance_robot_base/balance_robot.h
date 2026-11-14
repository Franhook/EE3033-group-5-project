#ifndef BALANCE_ROBOT_H
#define BALANCE_ROBOT_H

#include "rclcpp/rclcpp.hpp"
#include <serial/serial.h> // Ensure that the serial library is installed
#include <string>
#include <vector>
#include <chrono>
#include <cstdint>
#include <cstddef>

// ROS 2 message types
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "balance_interface/msg/pid_command.hpp" // <--- Added
#include "balance_interface/msg/pid_params.hpp"  // <--- Added

// Custom header files
#include "balance_robot_base/usart_protocol.h"
#include "balance_robot_base/quaternion_solution.h"

// Gyroscope scale factor
#define GYROSCOPE_RATIO   0.00108397f

// Accelerometer scale factor
// Raw data -> m/s^2: 32768 / 2g (19.6) = 1671.84
#define ACCEL_RATIO 	  1671.84f

class BalanceRobot : public rclcpp::Node
{
public:
    BalanceRobot();
    ~BalanceRobot();

private:
    // --- Core loop ---
    void control_loop(); // Timer callback that replaces the original Control() while loop

    // --- Serial communication and data processing ---
    // Serial-port object
    serial::Serial stm32_serial_;
    // Serial-send function
    void send_velocity_to_stm32(double linear_x, double angular_z);
    bool get_sensor_data(); // Read and parse serial data
    unsigned char check_sum(unsigned char count_number, unsigned char mode); // Checksum calculation
    
    // -- Data-conversion helper functions --
    short imu_trans(uint8_t data_high, uint8_t data_low);
    float odom_trans(uint8_t data_high, uint8_t data_low);

    // --- Subscription callbacks ---
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void pid_cmd_callback(const balance_interface::msg::PIDCommand::SharedPtr msg);

    // --- Topic-publishing functions ---
    void publish_odom();
    void publish_imu();
    void publish_voltage();
    void publish_euler();

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr voltage_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_publisher_;
    rclcpp::Publisher<balance_interface::msg::PIDParams>::SharedPtr pid_publisher_; // Publish the current PID parameters to the GUIGUI

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<balance_interface::msg::PIDCommand>::SharedPtr pid_cmd_subscription_; // Subscribe to PID-modification commands from the GUI

    // TF TF broadcasting
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Algorithm objects
    QuaternionSolution quaternion_solution_;

    // Time calculation
    rclcpp::Time last_time_;
    rclcpp::Time now_time_;

    
    // Velocity command watchdog
    std::chrono::steady_clock::time_point last_cmd_time_;
    double cmd_vel_timeout_sec_;


    // STM32 feedback watchdog
    std::chrono::steady_clock::time_point last_feedback_time_;
    double feedback_timeout_sec_;


    // Data storage
    std::vector<uint8_t> rx_buffer_;        // Dynamic buffers
    ReceivePacket receive_data_; // Receive-data buffer
    SendPacket send_data_;       // Send-data buffer
    
    Mpu6050RawData mpu6050_raw_; // IMU Raw IMU short data
    VelPosData robot_pos_;       // Robot position (X, Y, Theta) (X, Y, Theta)
    VelPosData robot_vel_;       // Robot velocity (VX, VY, VZ) (VX, VY, VZ)
    float power_voltage_;        // Supply voltage

    float stm32_pitch_; // Attitude angles
    float stm32_roll_;
    float stm32_yaw_;

    // --- Parameter variables ---
    double control_loop_freq_;
    std::string usart_port_name_;
    int serial_baud_rate_;
    std::string odom_frame_id_;
    std::string robot_frame_id_;
    std::string gyro_frame_id_;
    // S S-curve parameters and state 
    bool enable_s_curve_;
    double max_accel_x_, max_jerk_x_;
    double max_accel_z_, max_jerk_z_;

    // Dynamic-parameter callback handle
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_subscriber_;
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters);

    // S S-curve smoother
    void apply_s_curve(double &current_v, double &current_a, double target_v, 
                       double dt, double max_a, double max_j);
    
    // Target command (from cmd_vel) cmd_vel)
    double target_cmd_x_;
    double target_cmd_z_;
    // Command actually sent (result of S-curve integration))
    double current_cmd_x_;
    double current_cmd_z_;
    double current_acc_x_; // Current acceleration
    double current_acc_z_;
};

#endif // BALANCE_ROBOT_H
