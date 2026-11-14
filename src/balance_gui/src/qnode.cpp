#include "balance_gui/qnode.hpp"
#include <QDebug>


QNode::QNode(int argc, char** argv) : init_argc(argc), init_argv(argv) {}

QNode::~QNode() {
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
    wait();
}

bool QNode::init() {
    rclcpp::init(init_argc, init_argv);
    node_ = std::make_shared<rclcpp::Node>("balance_gui_node");

    if (!rclcpp::ok()) {
        qDebug() << "ROS 2 Init failed!";
        return false;
    }

    // Initialize publishers
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    click_pub_ = node_->create_publisher<geometry_msgs::msg::Point>("/detector/click_point", 10);
    pid_cmd_pub_ = node_->create_publisher<balance_interface::msg::PIDCommand>("/pid_command", 10);

    // Initialize subscribers
    battery_sub_ = node_->create_subscription<std_msgs::msg::Float32>(
        "/power_voltage", 10, std::bind(&QNode::batteryCallback, this, std::placeholders::_1));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&QNode::odomCallback, this, std::placeholders::_1));
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, std::bind(&QNode::imuCallback, this, std::placeholders::_1));
    euler_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/robot_euler", 10, std::bind(&QNode::eulerCallback, this, std::placeholders::_1));
    status_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "/detector/status", 10, std::bind(&QNode::statusCallback, this, std::placeholders::_1));
    amcl_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 10, std::bind(&QNode::amclPoseCallback, this, std::placeholders::_1));
    pid_params_sub_ = node_->create_subscription<balance_interface::msg::PIDParams>(
        "/pid_params", 10, std::bind(&QNode::pidParamsCallback, this, std::placeholders::_1));
    
    // Initialize the Image Target client
    target_client_ = node_->create_client<balance_interface::srv::SetTarget>("/detector/set_target");
    reset_client_ = node_->create_client<std_srvs::srv::Empty>("/detector/reset_target");
    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "navigate_to_pose");
    spin_client_ = rclcpp_action::create_client<nav2_msgs::action::Spin>(node_, "spin");
    backup_client_ = rclcpp_action::create_client<nav2_msgs::action::BackUp>(node_, "backup");

    start(); // Start the QThread QThread
    return true;
}

void QNode::run() {
    rclcpp::spin(node_);
    qDebug() << "ROS shutdown, proceeding to close the gui.";
    Q_EMIT rosShutdown();
}

/***********************************
 * Function implementations
 **********************************/
// Publish a velocity command
 void QNode::pub_cmd_vel(float linear_x, float angular_z) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = linear_x;
    twist.angular.z = angular_z;
    cmd_pub_->publish(twist);
}

// Asynchronously request that the vision node reset the target
void QNode::reset_target()
{
    if (!reset_client_->wait_for_service(std::chrono::seconds(1))) {
        qDebug() << "Service /detector/reset_target unavailable!";
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    reset_client_->async_send_request(request, 
        [this](rclcpp::Client<std_srvs::srv::Empty>::SharedFuture future) {
            qDebug() << "Reset command sent successfully.";
        });
}

// Publish the physical pixel coordinates of a mouse click
void QNode::pub_click_point(float x, float y) {
    geometry_msgs::msg::Point pt;
    pt.x = x;
    pt.y = y;
    pt.z = 0.0;
    click_pub_->publish(pt);
}

// Optionally subscribe to the camera topic (do not initialize when subscription is disabled)
void QNode::sub_camera(QString camera_topic) {
    current_camera_topic_ = camera_topic;
    // Reset the subscriber
    image_sub_ = nullptr;
    compressed_image_sub_ = nullptr;

    // Select the decoder dynamically according to the topic type
    if (camera_topic.contains("compressed")) {
        compressed_image_sub_ = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
            camera_topic.toStdString(), rclcpp::SensorDataQoS(), 
            std::bind(&QNode::compressedImageCallback, this, std::placeholders::_1));
        qDebug() << "Subscribed to COMPRESSED camera topic: " << camera_topic;
    } 
    else {
        image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            camera_topic.toStdString(), rclcpp::SensorDataQoS(), 
            std::bind(&QNode::imageCallback, this, std::placeholders::_1));
        qDebug() << "Subscribed to RAW camera topic: " << camera_topic;
    }
}

void QNode::unsub_camera() {
    image_sub_ = nullptr;
    compressed_image_sub_ = nullptr;
    qDebug() << "Unsubscribed from camera topic.";
}

// Topic-list retrieval function
QStringList QNode::getTopicList() {
    QStringList list;
    auto topics = node_->get_topic_names_and_types();
    for (const auto& topic : topics) {
        list.append(QString::fromStdString(topic.first));
    }
    list.removeDuplicates();
    list.sort();
    return list;
}

// Nav2 Nav2-related: cancel all active navigation goals
void QNode::cancel_nav2_goals() 
{
    // Cancel all Nav2 actions
    if (nav_client_) nav_client_->async_cancel_all_goals();
    if (spin_client_) spin_client_->async_cancel_all_goals();
    if (backup_client_) backup_client_->async_cancel_all_goals();
    // Send zero velocity
    pub_cmd_vel(0.0, 0.0);
}

/***********************************
 * ROS ROS callback functions
 **********************************/
void QNode::batteryCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    Q_EMIT batteryStateUpdated(msg->data);
}

void QNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Col 1 & 4: Extract linear velocity X/Y, angular velocity Z, and position X/Y X/Y
    Q_EMIT odomUpdated(
        msg->twist.twist.linear.x, 
        msg->twist.twist.linear.y, 
        msg->twist.twist.angular.z,
        msg->pose.pose.position.x, 
        msg->pose.pose.position.y
    );
}
void QNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Col 3: Extract three-axis angular velocity (rad/s) (rad/s)
    Q_EMIT imuUpdated(
        msg->angular_velocity.x, 
        msg->angular_velocity.y, 
        msg->angular_velocity.z
    );
}

void QNode::eulerCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
    // Vector3Stamped Vector3Stamped data is stored in msg->vector
    double pitch_deg = msg->vector.y * 180.0 / M_PI;
    double roll_deg  = msg->vector.x * 180.0 / M_PI;
    double yaw_deg   = msg->vector.z * 180.0 / M_PI;

    Q_EMIT attitudeUpdated(pitch_deg, roll_deg, yaw_deg);
}

void QNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    } 
    catch (cv_bridge::Exception& e) {
        qDebug() << "cv_bridge exception: " << e.what();
        return;
    }
    
    QImage im = Mat2QImage(cv_ptr->image);
    int byte_size = msg->data.size(); // Get the actual byte size of this frame
    QString format = QString::fromStdString(msg->encoding); // Get the encoding, such as rgb8 rgb8)
    
    Q_EMIT imageUpdated(im, byte_size, format);
}

void QNode::compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
        // Decode the JPEG byte stream received from ROS 2 directly into an OpenCV BGR image
        cv::Mat cv_image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        
        if (cv_image.empty()) {
            qDebug() << "Failed to decode compressed image!";
            return;
        }
        
        QImage im = Mat2QImage(cv_image);
        int byte_size = msg->data.size(); // Get the compressed byte size
        // Extract the specific format that follows, such as jpeg jpeg)
        QString format = QString::fromStdString(msg->format).split(";").last().trimmed(); 
        
        Q_EMIT imageUpdated(im, byte_size, format);
    } 
    catch (std::exception& e) {
        qDebug() << "Error decoding compressed image: " << e.what();
    }
}

void QNode::statusCallback(const std_msgs::msg::String::SharedPtr msg) {
    Q_EMIT detectorStatusUpdated(QString::fromStdString(msg->data));
}

void QNode::amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    // The covariance is a 6x6 matrix with 36 elements
    // Index 0: XX-axis variance, Index 7: Y-axis variance, Index 35: yaw-angle variance
    double cov_x = msg->pose.covariance[0];
    double cov_y = msg->pose.covariance[7];
    double cov_yaw = msg->pose.covariance[35];
    // X/Y X/Y error below 0.10 and angular error below 0.15 indicate full convergence
    bool converged = (cov_x < 0.10 && cov_y < 0.10 && cov_yaw < 0.15);
    
    Q_EMIT amclConverged(converged);
}

void QNode::pidParamsCallback(const balance_interface::msg::PIDParams::SharedPtr msg) {
    // After receiving parameters from the backend, send them to the front-end UI through a Qt signal UI
    Q_EMIT pidParamsUpdated(msg->balance_kp, msg->balance_kd, 
                            msg->velocity_kp, msg->velocity_ki,
                            msg->turn_kp, msg->turn_ki, msg->turn_kd, 
                            msg->middle_angle);
}


void QNode::pub_pid_command(uint8_t cmd_type, uint8_t param_id, float param_value,
                            float b_kp, float b_kd, float v_kp, float v_ki, 
                            float t_kp, float t_ki, float t_kd, float m_ang) 
{
    balance_interface::msg::PIDCommand cmd_msg;
    cmd_msg.command_type = cmd_type;
    cmd_msg.param_id = param_id;
    cmd_msg.param_value = param_value;
    
    cmd_msg.balance_kp = b_kp;
    cmd_msg.balance_kd = b_kd;
    cmd_msg.velocity_kp = v_kp;
    cmd_msg.velocity_ki = v_ki;
    cmd_msg.turn_kp = t_kp;
    cmd_msg.turn_ki = t_ki;
    cmd_msg.turn_kd = t_kd;
    cmd_msg.middle_angle = m_ang;

    pid_cmd_pub_->publish(cmd_msg);
}

/***********************************
 * Helper functions
 ***********************************/
// OpenCV Mat Convert OpenCV Mat to QImage QImage
QImage QNode::Mat2QImage(cv::Mat const& src) {
    QImage dest(src.cols, src.rows, QImage::Format_ARGB32);
    if (src.channels() == 3) {
        for (int i = 0; i < src.rows; ++i) {
            for (int j = 0; j < src.cols; ++j) {
                cv::Vec3b bgr = src.at<cv::Vec3b>(i, j);
                dest.setPixel(j, i, qRgb(bgr[2], bgr[1], bgr[0])); // BGR to RGB
            }
        }
    }
    return dest;
}

bool QNode::send_target_image(QString image_path)
{
    if (!target_client_->wait_for_service(std::chrono::seconds(1))) {
        qDebug() << "视觉节点 Service /detector/set_target 不可用！";
        return false;
    }

    // Read a local image with OpenCV
    cv::Mat img = cv::imread(image_path.toStdString(), cv::IMREAD_COLOR);
    if (img.empty()) {
        qDebug() << "无法加载图像文件: " << image_path;
        return false;
    }

    // Convert OpenCV Mat to ROS 2 sensor_msgs::msg::Image ROS 2 sensor_msgs::msg::Image
    std_msgs::msg::Header header;
    header.frame_id = "camera_link";
    header.stamp = node_->get_clock()->now();
    cv_bridge::CvImage cv_image(header, "bgr8", img);
    sensor_msgs::msg::Image::SharedPtr ros_img_msg = cv_image.toImageMsg();

    // Build the Service request
    auto request = std::make_shared<balance_interface::srv::SetTarget::Request>();
    request->target_image = *ros_img_msg;

    // Send the request asynchronously
    auto result_future = target_client_->async_send_request(request,
        [this](rclcpp::Client<balance_interface::srv::SetTarget>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                qDebug() << "目标设定成功: " << QString::fromStdString(response->message);
            } else {
                qDebug() << "目标设定失败: " << QString::fromStdString(response->message);
            }
        });

    return true;
}
