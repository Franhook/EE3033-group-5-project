#include "balance_follower/follower_mixed.hpp"
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

FollowerMixed::FollowerMixed() : Node("balance_follower_mixed_node")
{
    // --- Parameter declaration and retrieval ---
    this->declare_parameter("camera_fov_deg", 112.0);
    this->declare_parameter("target_distance", 0.40); 
    this->declare_parameter("scan_window_deg", 10.0);
    this->declare_parameter("lost_timeout", 0.5); 
    
    this->declare_parameter("enable_safe_stop", false);
    this->declare_parameter("safe_stop_distance", 0.2); 

    this->declare_parameter("max_linear_x", 0.3); 
    this->declare_parameter("min_linear_x", -0.3); 
    this->declare_parameter("max_angular_z", 2.0);

    // PID parameters
    this->declare_parameter("kp_x", 0.85);
    this->declare_parameter("ki_x", 0.00);
    this->declare_parameter("kd_x", 0.01);
    this->declare_parameter("max_integral_x", 0.5);

    this->declare_parameter("kp_z", 1.40);
    this->declare_parameter("ki_z", 0.00);
    this->declare_parameter("kd_z", 0.005);
    this->declare_parameter("max_integral_z", 1.0);

    camera_fov_deg_ = this->get_parameter("camera_fov_deg").as_double();
    fov_rad_ = camera_fov_deg_ * M_PI / 180.0;
    target_distance_ = this->get_parameter("target_distance").as_double();
    scan_window_deg_ = this->get_parameter("scan_window_deg").as_double();
    lost_timeout_ = this->get_parameter("lost_timeout").as_double();
    
    enable_safe_stop_ = this->get_parameter("enable_safe_stop").as_bool();
    safe_stop_distance_ = this->get_parameter("safe_stop_distance").as_double();

    max_linear_x_ = this->get_parameter("max_linear_x").as_double();
    min_linear_x_ = this->get_parameter("min_linear_x").as_double();
    max_angular_z_ = this->get_parameter("max_angular_z").as_double();

    kp_x_ = this->get_parameter("kp_x").as_double();
    ki_x_ = this->get_parameter("ki_x").as_double();
    kd_x_ = this->get_parameter("kd_x").as_double();
    max_integral_x_ = this->get_parameter("max_integral_x").as_double();

    kp_z_ = this->get_parameter("kp_z").as_double();
    ki_z_ = this->get_parameter("ki_z").as_double();
    kd_z_ = this->get_parameter("kd_z").as_double();
    max_integral_z_ = this->get_parameter("max_integral_z").as_double();

    // Initialize state
    prev_error_x_ = 0.0; integral_x_ = 0.0;
    prev_error_z_ = 0.0; integral_z_ = 0.0;

    // --- 2. Initialize interfaces ---
    // Subscribe to the target-information topic
    target_sub_ = this->create_subscription<balance_interface::msg::TargetInfo>(
        "/detector/target_info", rclcpp::QoS(1).best_effort(), 
        std::bind(&FollowerMixed::target_callback, this, std::placeholders::_1));
    // Subscribe to the LiDAR LaserScan topic
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::QoS(10).best_effort(), 
        std::bind(&FollowerMixed::scan_callback, this, std::placeholders::_1));
    // Subscribe to the RPY-angle topic
    euler_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/robot_euler", rclcpp::QoS(10).best_effort(), 
        std::bind(&FollowerMixed::euler_callback, this, std::placeholders::_1));
    current_yaw_ = 0.0f;
    // Publish the cmd_vel topic
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // Subscribe to the recognition-state topic
    status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/detector/status", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            std::lock_guard<std::mutex> lock(status_mutex_);
            current_status_ = msg->data;
        });

    // Control loop 20Hz (50ms)
    control_timer_ = this->create_wall_timer(
        50ms, std::bind(&FollowerMixed::control_loop, this));

    last_target_time_ = this->get_clock()->now();
    last_control_time_ = this->get_clock()->now();
    
    RCLCPP_INFO(this->get_logger(), "Mixed Follower Node Initialized. Safe Stop: %s", 
                enable_safe_stop_ ? "ON" : "OFF");
}

FollowerMixed::~FollowerMixed() {}

// --- Callbacks ---
void FollowerMixed::target_callback(const balance_interface::msg::TargetInfo::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(target_mutex_);
    latest_target_ = msg;
    last_target_time_ = this->get_clock()->now();

    float tan_fov_half = std::tan(fov_rad_ / 2.0);
    float new_vision_angle = -std::atan(msg->error_x * tan_fov_half);

    // DMP Look up the DMP history buffer and compensate for pure delay
    float vision_delay_s = std::clamp(msg->delay_time, 0.0f, 1.5f); // Image-frame delay
    rclcpp::Time frame_time = last_target_time_ - rclcpp::Duration::from_seconds(vision_delay_s);   // Actual image timestamp corresponding to this targetinfo message
    
    float delta_yaw = 0.0f;
    {
        std::lock_guard<std::mutex> lock(yaw_mutex_);
        float yaw_at_frame = current_yaw_;
        // Search backward for the angle closest to that instant
        for (auto it = yaw_history_.rbegin(); it != yaw_history_.rend(); ++it) {
            if (it->stamp <= frame_time) {
                yaw_at_frame = it->yaw;
                break;
            }
        }
        float raw_diff = current_yaw_ - yaw_at_frame;
        delta_yaw = std::atan2(std::sin(raw_diff), std::cos(raw_diff)); 
    }

    // Current true angle = image angle - angle rotated during the delay
    float compensated_vision_angle = new_vision_angle - delta_yaw;
    // Complementary filter
    virtual_target_angle_rad_ = (virtual_target_angle_rad_ * 0.4f) + (compensated_vision_angle * 0.6f);

    float abs_target_angle_deg = std::abs(virtual_target_angle_rad_) * 180.0 / M_PI;
    if (abs_target_angle_deg > scan_window_deg_ * 2.5) {
        is_aligning_ = true;
    }
    else if (abs_target_angle_deg < scan_window_deg_ * 2.0) {
        is_aligning_ = false;
    }
}

void FollowerMixed::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = msg;
}

void FollowerMixed::euler_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(yaw_mutex_);
    current_yaw_ = msg->vector.z; // Already expressed in radians
    
    rclcpp::Time msg_time = msg->header.stamp;
    yaw_history_.push_back({msg_time, current_yaw_});
    
    auto now = this->get_clock()->now();
    // Keep at most two seconds of history
    while(!yaw_history_.empty() && (now - yaw_history_.front().stamp).seconds() > 2.0) {
        yaw_history_.pop_front();
    }
}

// --- LiDAR depth extraction: nearest-cluster filtering ---
float FollowerMixed::extract_closest_depth(float target_angle_rad, float window_rad)
{
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        if (!latest_scan_) return -1.0;
        scan = latest_scan_;
    }

    std::vector<float> valid_depths;

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float angle = scan->angle_min + i * scan->angle_increment;
        float diff = std::atan2(std::sin(angle - target_angle_rad), std::cos(angle - target_angle_rad));
        
        if (std::abs(diff) <= window_rad) {
            float r = scan->ranges[i];
            if (r > 0.1 && r <= scan->range_max && std::isfinite(r)) { // Ignore LiDAR self-noise within 10 cm
                valid_depths.push_back(r);
            }
        }
    }

    if (valid_depths.empty()) return -1.0;

    // Sort in ascending order
    std::sort(valid_depths.begin(), valid_depths.end());
    // Average the nearest one to three points to represent the object's true front surface
    int points_to_avg = std::min((int)valid_depths.size(), 3);
    float sum = 0.0;
    for (int i = 0; i < points_to_avg; ++i) {
        sum += valid_depths[i];
    }
    return sum / points_to_avg;
}

// --- Obstacle detection: obtain the minimum distance directly ahead ---
float FollowerMixed::get_forward_min_distance()
{
    float check_window = 15.0 * M_PI / 180.0;
    float min_dist = 999.0;
    
    sensor_msgs::msg::LaserScan::SharedPtr scan;
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        if (!latest_scan_) return min_dist; 
        scan = latest_scan_;
    }

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        float angle = scan->angle_min + i * scan->angle_increment;
        float diff = std::atan2(std::sin(angle), std::cos(angle)); 
        
        if (std::abs(diff) <= check_window) {
            float r = scan->ranges[i];
            if (r > 0.05 && r < min_dist) { 
                min_dist = r;
            }
        }
    }
    return min_dist;
}

// --- Core control loop (20Hz) ---
void FollowerMixed::control_loop()
{
    // If the current vision state is SEARCHING, the robot is cruising to find a person
    // This node must yield control of /cmd_vel at this time
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (current_status_ == "SEARCHING") {
            return; 
        }
    }

    geometry_msgs::msg::Twist cmd_vel;
    
    // Get the latest target
    balance_interface::msg::TargetInfo::SharedPtr target;
    {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target = latest_target_;
    }

    // Calculate the actual time interval dt
    auto now = this->get_clock()->now();
    double dt = (now - last_control_time_).seconds();
    last_control_time_ = now;
    if (dt <= 0.001) dt = 0.05; // Prevent division by 0

    double time_since_last_target = (now - last_target_time_).seconds();
    float raw_cmd_x = 0.0;
    float raw_cmd_z = 0.0;

    float fwd_dist = get_forward_min_distance();

    if (enable_safe_stop_ && fwd_dist < safe_stop_distance_) 
    {
        // Emergency repulsion (fallback P control)
        float avoid_error = fwd_dist - safe_stop_distance_; 
        raw_cmd_x = kp_x_ * avoid_error; 
        raw_cmd_x = std::clamp(raw_cmd_x, min_linear_x_, 0.0f);
        raw_cmd_z = 0.0;
        // Clear the integral accumulators
        integral_x_ = 0.0; integral_z_ = 0.0;
        prev_error_x_ = 0.0; prev_error_z_ = 0.0;
    } 
    else if (!target || !target->detected || time_since_last_target > lost_timeout_) 
    {
        // Target lost (brake to a complete stop)
        raw_cmd_x = 0.0; raw_cmd_z = 0.0;
        integral_x_ = 0.0; integral_z_ = 0.0;
        prev_error_x_ = 0.0; prev_error_z_ = 0.0;
    }
    else 
    {
        // PID following mode
        float window_rad = scan_window_deg_ * M_PI / 180.0;
        // Extract LiDAR depth
        float current_depth = extract_closest_depth(virtual_target_angle_rad_, window_rad);
        RCLCPP_INFO(this->get_logger(), "Virtual Angle: %.2f deg, Depth: %.2f m", 
                    virtual_target_angle_rad_ * 180.0 / M_PI, current_depth);

        // --- Z axis (steering loop) ---
        float error_z = virtual_target_angle_rad_;
        // Angle deadband
        if (std::abs(error_z) < 0.05f) {
            error_z = 0.0f;
            integral_z_ = 0.0f; 
        } 
        else {
            // Integral accumulation and limiting
            integral_z_ += error_z * dt;
            integral_z_ = std::clamp(integral_z_, -max_integral_z_, max_integral_z_);
        }

        float d_error_z = (error_z - prev_error_z_) / dt;
        raw_cmd_z = kp_z_ * error_z + ki_z_ * integral_z_ + kd_z_ * d_error_z;
        raw_cmd_z = std::clamp(raw_cmd_z, (float)-max_angular_z_, (float)max_angular_z_);
        prev_error_z_ = error_z;

        // Calculate how many degrees the robot rotated during this control interval dt
        float delta_yaw_loop = 0.0f;
        {
            std::lock_guard<std::mutex> lock(yaw_mutex_);
            static float last_loop_yaw = current_yaw_;
            float raw_diff = current_yaw_ - last_loop_yaw;
            delta_yaw_loop = std::atan2(std::sin(raw_diff), std::cos(raw_diff));
            last_loop_yaw = current_yaw_;
        }
        virtual_target_angle_rad_ -= delta_yaw_loop;

        // --- X axis (distance loop) ---
        if (is_aligning_) {
            // Physical lock active: rotate the robot in place
            raw_cmd_x = 0.0;
            integral_x_ = 0.0;
            prev_error_x_ = 0.0;
        }
        else if (current_depth > 0) {
            // Release the physical lock: the true angle is below 12 degrees, so the distance-loop PID
            float error_x = current_depth - target_distance_;
            
            // Distance deadband 
            if (std::abs(error_x) < 0.1f) {
                error_x = 0.0f;
                integral_x_ = 0.0f; 
            } 
            else {
                integral_x_ += error_x * dt;
                integral_x_ = std::clamp(integral_x_, -max_integral_x_, max_integral_x_);
            }

            float d_error_x = (error_x - prev_error_x_) / dt;
            // cmd_x = Kp * error + Ki * integral + Kd * derivative
            raw_cmd_x = kp_x_ * error_x + ki_x_ * integral_x_ + kd_x_ * d_error_x;
            raw_cmd_x = std::clamp(raw_cmd_x, min_linear_x_, max_linear_x_);    // Forward-speed limit
            prev_error_x_ = error_x;

            // Angle-gated throttle（Angle-Gated Throttle）
            float abs_err_deg = std::abs(virtual_target_angle_rad_) * 180.0 / M_PI;
            float angle_penalty = (scan_window_deg_ - abs_err_deg) / scan_window_deg_; // 12 degrees gives 0; at 0 degrees gives 1
            angle_penalty = std::clamp(angle_penalty, 0.0f, 1.0f);
            raw_cmd_x *= angle_penalty;
        } 
        else
        {
            raw_cmd_x = 0.0; 
            integral_x_ = 0.0;
            prev_error_x_ = 0.0;
        }
    }

    // Send the command
    cmd_vel.linear.x = raw_cmd_x;
    cmd_vel.angular.z = raw_cmd_z;
    cmd_pub_->publish(cmd_vel);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowerMixed>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
