#ifndef FOLLOWER_MIXED_HPP
#define FOLLOWER_MIXED_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "balance_interface/msg/target_info.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include <deque>
#include <vector>
#include <mutex>

class FollowerMixed : public rclcpp::Node
{
public:
    FollowerMixed();
    ~FollowerMixed();

private:
    // --- ROS communication interfaces ---
    rclcpp::Subscription<balance_interface::msg::TargetInfo>::SharedPtr target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;

    // --- Data caching and thread safety ---
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    balance_interface::msg::TargetInfo::SharedPtr latest_target_;
    
    std::mutex scan_mutex_;
    std::mutex target_mutex_;
    std::mutex yaw_mutex_;
    std::mutex status_mutex_;
    rclcpp::Time last_target_time_;
    rclcpp::Time last_control_time_;
    std::string current_status_ = "IDLE";
    
    // --- Parameter declarations ---
    float camera_fov_deg_;
    float fov_rad_;
    float target_distance_;
    float scan_window_deg_;
    float lost_timeout_;
    
    bool enable_safe_stop_;
    float safe_stop_distance_;

    float max_linear_x_;
    float min_linear_x_;
    float max_angular_z_;

    // PID controller variables
    // X axis (distance loop)
    float kp_x_, ki_x_, kd_x_;
    float prev_error_x_;
    float integral_x_;
    float max_integral_x_;

    // Z axis (steering loop)
    float kp_z_, ki_z_, kd_z_;
    float prev_error_z_;
    float integral_z_;
    float max_integral_z_;

    // Open-loop angle observer
    float current_odom_w_;    // Store the current measured angular velocity
    float virtual_target_angle_rad_;
    // Physical yaw lock
    bool is_aligning_;

    // --- Core callbacks and control loop ---
    void target_callback(const balance_interface::msg::TargetInfo::SharedPtr msg);
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void euler_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void control_loop();

    // Heading-angle history queue
    struct YawRecord {
        rclcpp::Time stamp;
        float yaw;
    };
    std::deque<YawRecord> yaw_history_;
    float current_yaw_;

    // --- Algorithm helper functions ---
    // Extract the cloest valid clustering distance 
    float extract_closest_depth(float target_angle_rad, float window_rad);
    // Return the minimum distance directly ahead
    float get_forward_min_distance();
};

#endif // FOLLOWER_MIXED_HPP
