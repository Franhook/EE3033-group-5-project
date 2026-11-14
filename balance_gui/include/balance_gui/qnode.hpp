#ifndef BALANCE_GUI_QNODE_HPP
#define BALANCE_GUI_QNODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <QThread>
#include <QString>
#include <QImage>

// ROS 2 message types
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <balance_interface/srv/set_target.hpp>
#include "balance_interface/msg/pid_params.hpp"
#include "balance_interface/msg/pid_command.hpp"
#include <std_srvs/srv/empty.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/back_up.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class QNode : public QThread
{
    Q_OBJECT
public:
    QNode(int argc, char** argv);
    virtual ~QNode();

    bool init();
    void run() override; // Override QThread::run() to execute ros2 spin

    // Topic-subscription functions
    void sub_camera(QString camera_topic);
    void unsub_camera();
    QStringList getTopicList(); // Get all currently active topics
    // Keyboard/button velocity-command function
    void pub_cmd_vel(float linear_x, float angular_z);
    // Send the physical pixel coordinates of a mouse click
    void pub_click_point(float x, float y);
    // Read a local image and call a ROS 2 service to send the target
    bool send_target_image(QString image_path);
    // Reset the target
    void reset_target();
    // Nav2 related functions
    void cancel_nav2_goals(); // Cancel all active navigation goals
    // Method for sending PID-tuning commands
    void pub_pid_command(uint8_t cmd_type, uint8_t param_id = 0, float param_value = 0.0,
                         float b_kp=0, float b_kd=0, float v_kp=0, float v_ki=0, 
                         float t_kp=0, float t_ki=0, float t_kd=0, float m_ang=0);
    
    // Get the ROS node pointer
    std::shared_ptr<rclcpp::Node> node() { return node_; }

signals:
    // Signal that sends ROS data to the GUI
    void rosShutdown();
    void batteryStateUpdated(double voltage);
    void imageUpdated(QImage img, int byte_size, QString format);
    void odomUpdated(double l_x, double l_y, double a_z, double p_x, double p_y);
    void imuUpdated(double g_x, double g_y, double g_z);
    void attitudeUpdated(double pitch, double roll, double yaw);
    void detectorStatusUpdated(QString status);
    void amclConverged(bool converged);
    void pidParamsUpdated(float b_kp, float b_kd, float v_kp, float v_ki, 
                          float t_kp, float t_ki, float t_kd, float m_ang); // Signal that sends received low-level PID parameters to the GUI

private:
    int init_argc;
    char** init_argv;
    std::shared_ptr<rclcpp::Node> node_;

    // Publishers and subscribers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr click_pub_;
    rclcpp::Publisher<balance_interface::msg::PIDCommand>::SharedPtr pid_cmd_pub_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
    rclcpp::Subscription<balance_interface::msg::PIDParams>::SharedPtr pid_params_sub_;

    // ROS 2 Service clients
    rclcpp::Client<balance_interface::srv::SetTarget>::SharedPtr target_client_;    // Set-target service client
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;         // Reset-target service client
    // Nav2 Action client
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    rclcpp_action::Client<nav2_msgs::action::Spin>::SharedPtr spin_client_;
    rclcpp_action::Client<nav2_msgs::action::BackUp>::SharedPtr backup_client_;

    QString current_camera_topic_;

    // Callbacks
    void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void eulerCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
    void statusCallback(const std_msgs::msg::String::SharedPtr msg);
    void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void pidParamsCallback(const balance_interface::msg::PIDParams::SharedPtr msg);
    // Format conversion
    QImage Mat2QImage(cv::Mat const& src);
};

#endif // BALANCE_GUI_QNODE_HPP