#include "balance_gui/main_window.hpp"
#include "ui_main_window.h"

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , qnode(argc, argv)
{
    proc_base_ = nullptr;
    proc_lidar_ = nullptr;
    proc_camera_ = nullptr;
    proc_vision_ = nullptr;
    proc_follower_ = nullptr;
    proc_topic_echo_ = nullptr;
    proc_rqt_ = nullptr;
    proc_build_ = nullptr;
    proc_mapping_ = nullptr;
    proc_rviz_ = nullptr;
    proc_tftree_ = nullptr;
    proc_localization_ = nullptr;
    proc_patrol_ = nullptr;
    proc_nav_ = nullptr;
    proc_explore_ = nullptr;

    is_nfs_mounted_ = false;
    is_docker_on_ = false;
    is_online_ = false;

    ui->setupUi(this);

    // 1. Initialize QNode (ROS 2 backend)
    if (!qnode.init()) {
        QMessageBox::critical(this, "Error", "ROS 2 initialization failed!");
        close();
    }

    // Read configuration
    auto node = qnode.node();
    node->declare_parameter("use_ssh", true);
    node->declare_parameter("remote_ip", "192.168.1.30");
    node->declare_parameter("remote_user", "balance");
    rcl_interfaces::msg::ParameterDescriptor pwd_desc;
    pwd_desc.dynamic_typing = true;
    node->declare_parameter("remote_pwd", rclcpp::ParameterValue(""), pwd_desc);
    node->declare_parameter("use_docker", true);
    node->declare_parameter("docker_name", "ros2_humble");
    node->declare_parameter("local_pwd", "dongguan");

    use_ssh_ = node->get_parameter("use_ssh").as_bool();
    remote_ip_ = QString::fromStdString(node->get_parameter("remote_ip").as_string());
    remote_user_ = QString::fromStdString(node->get_parameter("remote_user").as_string());
    QString raw_pwd = QString::fromStdString(node->get_parameter("remote_pwd").value_to_string());
    remote_pwd_ = raw_pwd.remove("PWD_").replace("'", "").replace("\"", "");
    use_docker_ = node->get_parameter("use_docker").as_bool();
    docker_name_ = QString::fromStdString(node->get_parameter("docker_name").as_string());
    local_pwd_ = QString::fromStdString(node->get_parameter("local_pwd").as_string());

    // 2. Connect QNode signals to UI slots
    // Note: QImage must be registered as a metatype when passed across threads
    qRegisterMetaType<QImage>("QImage");
    
    connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(slotRosShutdown()));
    connect(&qnode, SIGNAL(batteryStateUpdated(double)), this, SLOT(slotUpdateBattery(double)));
    connect(&qnode, SIGNAL(imageUpdated(QImage,int,QString)), this, SLOT(slotUpdateImage(QImage,int,QString)));
    connect(&qnode, SIGNAL(odomUpdated(double,double,double,double,double)), 
            this, SLOT(slotUpdateOdom(double,double,double,double,double)));
    connect(&qnode, SIGNAL(imuUpdated(double,double,double)), 
            this, SLOT(slotUpdateImu(double,double,double)));
    connect(&qnode, SIGNAL(attitudeUpdated(double,double,double)), 
            this, SLOT(slotUpdateAttitude(double,double,double)));
    connect(&qnode, SIGNAL(detectorStatusUpdated(QString)), this, SLOT(slotDetectorStatusUpdated(QString)));     
    connect(&qnode, SIGNAL(amclConverged(bool)), this, SLOT(slotAmclConverged(bool)));
    connect(&qnode, SIGNAL(pidParamsUpdated(float,float,float,float,float,float,float,float)), 
            this, SLOT(slotUpdatePIDParams(float,float,float,float,float,float,float,float)));

    // 3. UI initialization settings
    initUI();
    // Initialize the teleoperation module
    initTeleop();
    // Initialize the SSH module
    initSSH();
    // At startup, disable all buttons on the right
    setControlButtonsEnabled(false);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initUI()
{
    // Force the image widget to be centered and install a mouse-event filter
    ui->label_video->setAlignment(Qt::AlignCenter);
    ui->label_video->installEventFilter(this);
    // Show the initial state as offline
    ui->label_status_text->setText("Offline");
    ui->label_status_text->setStyleSheet("color: red; font-weight: bold;");
    // Connect the close button of the tabbed terminal
    connect(ui->tabWidget_terminals, &QTabWidget::tabCloseRequested, this, &MainWindow::slotCloseTerminalTab);
    // Keep at most 1,000 lines in the log text box
    ui->textEdit_logs->document()->setMaximumBlockCount(1000);
    // Show the video page by default in the stacked widget (index 0)
    ui->stackedWidget_display->setCurrentIndex(0); 
    // Initial UI state for autonomous mapping
    connect(ui->comboBox_map_method, &QComboBox::currentTextChanged, this, &MainWindow::on_comboBox_map_method_currentTextChanged);
    // Disable the PID-tuning panel by default until serial communication starts
    ui->tab_pid->setEnabled(false);
}

// --- Enable or disable control buttons ---
void MainWindow::setControlButtonsEnabled(bool enabled)
{
    // Enable or disable the right-side control buttons in bulk according to connection status
    ui->btn_open_base->setEnabled(enabled);
    ui->btn_open_lidar->setEnabled(enabled);
    ui->btn_open_usb_camera->setEnabled(enabled);
    ui->btn_open_vision->setEnabled(enabled);
    ui->btn_open_mixed_follower->setEnabled(enabled);
    ui->btn_open_map->setEnabled(enabled);
    ui->btn_open_localization->setEnabled(enabled);
    ui->btn_map_save->setEnabled(enabled);
    ui->btn_open_nav->setEnabled(enabled);
    ui->btn_map_explore->setEnabled(enabled && ui->comboBox_map_method->currentText().contains("Auto"));
    ui->btn_open_patrol->setEnabled(enabled);
    ui->btn_close_all->setEnabled(enabled);

    ui->btn_cmd_docker->setEnabled(enabled);
    ui->btn_cmd_build->setEnabled(enabled);
    ui->btn_new_terminal->setEnabled(enabled);
    ui->btn_cmd_build_pkg->setEnabled(enabled);
    ui->btn_sub_topic->setEnabled(enabled);
    ui->checkBox_teleop->setEnabled(enabled);
    ui->checkBox_sacc->setEnabled(enabled);
    if (!enabled) {
        ui->btn_pub_target->setEnabled(false);
        ui->btn_reset_target->setEnabled(false);
    }
}

// --- Log-printing helper functions ---
void MainWindow::appendLog(const QString& msg, bool is_error)
{
    QString color = is_error ? "red" : "black";
    QString html_msg = QString("<font color=\"%1\">%2</font>").arg(color, msg);
    ui->textEdit_logs->append(html_msg);
}


// --- ROS 2 data-update slots ---
void MainWindow::slotUpdateBattery(double voltage)
{
    // Update the voltage text
    ui->label_voltage->setText(QString::number(voltage, 'f', 2) + " V");

    // Update the progress bar (fully charged: 12.3 V; depleted: 11.0V)
    double max_v = 12.3;
    double min_v = 11.0;
    int percent = 0;

    if (voltage >= max_v) {
        percent = 100;
    } else if (voltage <= min_v) {
        percent = 0;
    } else {
        percent = static_cast<int>((voltage - min_v) / (max_v - min_v) * 100);
    }

    ui->progressBar_battery->setValue(percent);
}

void MainWindow::slotUpdateOdom(double l_x, double l_y, double a_z, double p_x, double p_y)
{
    // Col 1: Odometry velocity
    ui->label_linear_x->setText(QString::number(l_x, 'f', 3) + " m/s");
    ui->label_linear_y->setText(QString::number(l_y, 'f', 3) + " m/s");
    ui->label_angular_z->setText(QString::number(a_z, 'f', 3) + " rad/s");
    // Col 4: Odometry position
    ui->label_pos_x->setText(QString::number(p_x, 'f', 2) + " m");
    ui->label_pos_y->setText(QString::number(p_y, 'f', 2) + " m");
}

void MainWindow::slotUpdateImu(double g_x, double g_y, double g_z)
{
    // Col 3: IMU angular velocity (the ROS backend usesrad/s)
    ui->label_gyro_x->setText(QString::number(g_x, 'f', 2) + " rad/s");
    ui->label_gyro_y->setText(QString::number(g_y, 'f', 2) + " rad/s");
    ui->label_gyro_z->setText(QString::number(g_z, 'f', 2) + " rad/s");
}

void MainWindow::slotUpdateAttitude(double pitch, double roll, double yaw)
{
    // Col 2: STM32 native attitude angles
    ui->label_pose_pitch->setText(QString::number(pitch, 'f', 2) + " deg");
    ui->label_pose_roll->setText(QString::number(roll, 'f', 2) + " deg");
    ui->label_pose_yaw->setText(QString::number(yaw, 'f', 2) + " deg");
}

void MainWindow::slotUpdateImage(QImage img, int byte_size, QString format)
{
    latest_image_ = img;

    if (ui->stackedWidget_display->currentIndex() == 0 && ui->radio_img->isChecked()) {
        
        // Accumulate data for this frame
        frame_count_++;
        bytes_accumulated_ += byte_size;
        
        qint64 current_time = QDateTime::currentMSecsSinceEpoch();
        
        if (last_calc_time_ == 0) {
            last_calc_time_ = current_time;
        } 
        else {
            double dt = (current_time - last_calc_time_) / 1000.0; 
            if (dt >= 1.0) { 
                // Calculate the true average over the previous second
                double real_fps = frame_count_ / dt;
                double real_bw_kb = (bytes_accumulated_ / 1024.0) / dt;
                
                ui->label_video_fps->setText(QString("FPS: %1 Hz").arg(real_fps, 0, 'f', 1));
                
                if (real_bw_kb > 1024.0) {
                    ui->label_video_bw->setText(QString("BW: %1 MB/s").arg(real_bw_kb / 1024.0, 0, 'f', 2));
                } else {
                    ui->label_video_bw->setText(QString("BW: %1 KB/s").arg(real_bw_kb, 0, 'f', 0));
                }
                // Clear the accumulators and begin the next one-second interval
                frame_count_ = 0;
                bytes_accumulated_ = 0;
                last_calc_time_ = current_time;
            }
        }

        // Update the resolution and format Label
        ui->label_video_resol->setText(QString("Resolution: %1x%2").arg(img.width()).arg(img.height()));
        ui->label_video_format->setText(QString("Format: %1").arg(format));
        // Render the image
        ui->label_video->setPixmap(QPixmap::fromImage(img).scaled(
            ui->label_video->width(), ui->label_video->height(), 
            Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void MainWindow::slotDetectorStatusUpdated(QString status)
{
    // Enable or disable buttons according to the state of the low-level vision node
    if (status == "IDLE") {
        ui->btn_pub_target->setEnabled(true);
        ui->btn_reset_target->setEnabled(false);
    } 
    else {
        ui->btn_pub_target->setEnabled(true);
        ui->btn_reset_target->setEnabled(true);
    }

    if (!ui->checkBox_follower_cruise->isChecked()) return;             // Follow-and-patrol mode is not enabled
    if (ui->btn_open_mixed_follower->text().contains("Open")) return;   // The follower node is not enabled
    if (!proc_follower_ || proc_follower_->state() == QProcess::NotRunning) return; // The follower node is not running
    if (!proc_localization_ || proc_localization_->state() == QProcess::NotRunning) return; // The localization node is not running

    if (status == "SEARCHING") {
        if (ui->btn_open_patrol->text().contains("Open")) {
            appendLog("<b>[FOLLOWER]</b> Target lost! Triggering PATROL node to search...");
            ui->comboBox_patrol_strategy->setCurrentText("Random");
            on_btn_open_patrol_clicked();
        }
    }
    else if (status == "TRACKING") {
        if (ui->btn_open_patrol->text().contains("Close")) {
            appendLog("<b>[FOLLOWER]</b> Target FOUND! Terminating PATROL node natively...");
            on_btn_open_patrol_clicked(); 
        }
    }
}

void MainWindow::slotRosShutdown()
{
    ui->label_status_text->setText("Offline");
    ui->label_status_text->setStyleSheet("color: red; font-weight: bold;");
    appendLog("<b>[ROS]</b> ROS 2 Connection Lost.", true);
}

// --- GUI page switching ---
void MainWindow::updateSubButtonUI(bool is_subbed) {
    if (is_subbed) {
        ui->btn_sub_topic->setText("Unsubscribe");
        ui->btn_sub_topic->setStyleSheet("background-color: #f44336; color: white; font-weight: bold;");
    } else {
        ui->btn_sub_topic->setText("Subscribe");
        ui->btn_sub_topic->setStyleSheet("");
    }
}

void MainWindow::on_radio_img_toggled(bool checked)
{
    if (checked) {
        ui->stackedWidget_display->setCurrentIndex(0); 
        updateSubButtonUI(is_img_sub_); // Synchronize the image subscription state when switching
    }
}

void MainWindow::on_radio_txt_toggled(bool checked)
{
    if (checked) {
        ui->stackedWidget_display->setCurrentIndex(1); 
        updateSubButtonUI(is_txt_sub_); // Synchronize the text subscription state when switching
    }
}

// --- Subscribe to the currently selected topic ---
void MainWindow::on_btn_sub_topic_clicked()
{
    QString topic = ui->comboBox_topic->currentText();
    if (topic.isEmpty()) return;

    if (ui->radio_img->isChecked()) {
        // Image-stream subscribe/unsubscribe logic
        if (is_img_sub_) {
            // Unsubscribe
            qnode.unsub_camera();
            last_calc_time_ = 0; // Reset the speed meter
            ui->label_video->setText("Waiting for subscribing image...");
            ui->label_video->setPixmap(QPixmap());
            // Clear the Label display
            ui->label_video_name->setText("Name: ");
            ui->label_video_resol->setText("Resolution: ");
            ui->label_video_fps->setText("FPS: 0 Hz");
            ui->label_video_bw->setText("BW: 0 KB/s");
            ui->label_video_format->setText("Format: None");
            
            is_img_sub_ = false;
            appendLog("<b>[TOPIC]</b> Unsubscribed from image topic.");
        } 
        else {
            // Start subscribing
            qnode.sub_camera(topic);
            ui->label_video_name->setText("Name: " + topic);
            is_img_sub_ = true;
            appendLog("<b>[TOPIC]</b> Subscribed to image topic: " + topic);
        }
        updateSubButtonUI(is_img_sub_);
    } 
    else {
        // Plain-text stream subscribe/unsubscribe logic
        if (is_txt_sub_) {
            // Unsubscribe
            if (proc_topic_echo_ && proc_topic_echo_->state() != QProcess::NotRunning) {
                proc_topic_echo_->kill();
                proc_topic_echo_->waitForFinished(500);
            }
            ui->textEdit_topic_data->clear();
            ui->textEdit_topic_data->append("Waiting for subscribing topic...");
            
            is_txt_sub_ = false;
            appendLog("<b>[TOPIC]</b> Unsubscribed from text topic.");
        } 
        else {
            // Start subscribing
            if (proc_topic_echo_ && proc_topic_echo_->state() != QProcess::NotRunning) {
                proc_topic_echo_->kill();
                proc_topic_echo_->waitForFinished(500);
            }
            if (!proc_topic_echo_) {
                proc_topic_echo_ = new QProcess(this);
                connect(proc_topic_echo_, &QProcess::readyReadStandardOutput, this, [this]() {
                    QString data = QString::fromLocal8Bit(proc_topic_echo_->readAllStandardOutput()).trimmed();
                    if (!data.isEmpty()) {
                        ui->textEdit_topic_data->append(data);
                        ui->textEdit_topic_data->document()->setMaximumBlockCount(1000);
                    }
                });
            }
            ui->textEdit_topic_data->clear();
            QString cmd = buildCommand("ros2 topic echo " + topic);
            proc_topic_echo_->start("sh", QStringList() << "-c" << cmd);
            
            is_txt_sub_ = true;
            appendLog("<b>[TOPIC]</b> Subscribed to text topic: " + topic);
        }
        updateSubButtonUI(is_txt_sub_);
    }
}

// --- AMCL Update convergence status ---
void MainWindow::slotAmclConverged(bool converged) {
    if (!proc_localization_ || proc_localization_->state() == QProcess::NotRunning) return;

    QString current_text = ui->btn_amcl_relocate->text();
    
    if (!converged) {
        if (ui->btn_amcl_relocate->text() != "Unconverged") {
            ui->btn_amcl_relocate->setStyleSheet("background-color: #F44336; color: white; font-weight: bold;"); 
            ui->btn_amcl_relocate->setText("Unconverged");
            appendLog("<b>[LOCALIZATION]</b> AMCL Tracking Lost! Covariance too high.");
        }
    } 
    else {
        if (ui->btn_amcl_relocate->text() != "Localized") {
            ui->btn_amcl_relocate->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;"); 
            ui->btn_amcl_relocate->setText("Localized");
            
            // Automatically stop the localization-assistance Spin
            if (proc_patrol_ && proc_patrol_->state() != QProcess::NotRunning && ui->comboBox_patrol_strategy->currentText().contains("Spin")) {
                appendLog("<b>[LOCALIZATION]</b> Auto-converged successfully! Stopping Spin Patrol.");
                on_btn_open_patrol_clicked(); 
            }
        }
    }
}
// --- Refresh the topic list ---
void MainWindow::on_btn_refresh_topics_clicked()
{
    ui->comboBox_topic->clear();
    QStringList topics = qnode.getTopicList();
    if (topics.isEmpty()) {
        appendLog("<b>[TOPIC]</b> No topics found. Please check ROS 2 connection.", true);
    } else {
        ui->comboBox_topic->addItems(topics);
        appendLog("<b>[TOPIC]</b> Topic list refreshed.");
    }
}

// --- Select and publish the target image ---
void MainWindow::on_btn_browse_img_clicked()
{
    // Open a file-selection dialog that allows only common image formats
    QString fileName = QFileDialog::getOpenFileName(this, 
        tr("Select Target Image"), "", tr("Images (*.png *.jpg *.jpeg *.bmp)"));
    
    if (!fileName.isEmpty()) {
        ui->lineEdit_target_img->setText(fileName);
        appendLog("<b>[VISION]</b> Target image selected: " + fileName);
    }
}

void MainWindow::on_btn_pub_target_clicked()
{
    QString path = ui->lineEdit_target_img->text();
    if (path.isEmpty()) {
        QMessageBox::warning(this, "Warning", "Please browse and select a target image first!");
        return;
    }
    
    // Use QNode to read a local image and send it to the detector node
    if (qnode.send_target_image(path)) {
        appendLog("<b>[VISION]</b> Target image sent, waiting for feature extraction...");
    } else {
        appendLog("<b>[VISION]</b> Failed to send target image. Please check the file path or node status!", true);
    }
}

void MainWindow::on_btn_reset_target_clicked()
{
    qnode.reset_target();
    appendLog("<b>[VISION]</b> Sent target reset command. Waiting for IDLE state...");
}

// --- Snapshot button function ---
void MainWindow::on_btn_get_image_clicked()
{
    if (latest_image_.isNull()) {
        QMessageBox::warning(this, "Warning", "No image stream available! Please subscribe to a camera topic first.");
        return;
    }
    
    QString targetDir = "/home/ros2_ws/src/balance_gui/images";
    QDir dir;
    if (!dir.exists(targetDir)) {
        dir.mkpath(targetDir); // Force creation of an absolute path
    }
    
    QString defaultName = "target_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss") + ".jpg";
    QString defaultPath = targetDir + "/" + defaultName; // Construct the complete default path
    QString savePath = QFileDialog::getSaveFileName(this, 
        tr("Save Captured Target Image"), defaultPath, tr("Images (*.jpg *.png *.bmp)"));
        
    if (!savePath.isEmpty()) {
        if (latest_image_.save(savePath)) {
            ui->lineEdit_target_img->setText(savePath);
            appendLog("<b>[VISION]</b> Image successfully captured and saved to: " + savePath);
        } else {
            QMessageBox::critical(this, "Error", "Failed to save the image to the specified path.");
        }
    }
}

// --- Odometry reset button ---
void MainWindow::on_btn_reset_odom_clicked()
{
    if (!ui->btn_open_base->text().contains("Close")) {
        QMessageBox::warning(this, "Warning", "Base Node is not running!");
        return;
    }

    QString cmd = buildCommand("ros2 param set /balance_robot_node reset_odom true");
    QProcess::startDetached("sh", QStringList() << "-c" << cmd);
    appendLog("<b>[BASE]</b> Odometry reset command sent.");
}

// --- Window close event ---
void MainWindow::closeEvent(QCloseEvent *event)
{
    // Create the dialog
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("Exit Confirmation");
    msgBox.setText("Please select the action to take for the remote environment on exit:");
    msgBox.setIcon(QMessageBox::Question);
    // Add three buttons
    QPushButton *btn_close_all = msgBox.addButton("Close All (Docker+Nodes)", QMessageBox::ActionRole);
    QPushButton *btn_close_nodes = msgBox.addButton("Close Nodes Only", QMessageBox::ActionRole);
    QPushButton *btn_keep_all = msgBox.addButton("Keep All", QMessageBox::ActionRole);
    QPushButton *btn_cancel = msgBox.addButton("Cancel", QMessageBox::RejectRole);

    // If Docker management is disabled, disable "Close All”
    if (!use_docker_) {
        btn_close_all->setEnabled(false);
        btn_close_all->setToolTip("Current Docker management is not enabled, cannot perform this action.");
    }
    if (!is_online_) {
        btn_close_all->setEnabled(false);
        btn_close_nodes->setEnabled(false);
        btn_close_all->setToolTip("Current host is offline, cannot perform this action.");
        btn_close_nodes->setToolTip("Current host is offline, cannot perform this action.");
    }
    msgBox.exec();

    // Decision logic
    if (msgBox.clickedButton() == btn_close_all) {
        appendLog("<b>[EXIT]</b> Executing global resource cleanup (closing nodes and stopping Docker)...", true);
        on_btn_close_all_clicked();
        if (use_docker_ && is_docker_on_) {
            QString stop_docker = QString("docker stop %1").arg(docker_name_);
            if (use_ssh_) {
                stop_docker = QString("sshpass -p '%1' ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null %2@%3 '%4'")
                                .arg(remote_pwd_, remote_user_, remote_ip_, stop_docker);
            }
            QProcess::execute("sh", QStringList() << "-c" << stop_docker);
        }
        event->accept();
    } 
    else if (msgBox.clickedButton() == btn_close_nodes) {
        appendLog("<b>[EXIT]</b> Only closing running nodes, keeping Docker in awakened state...", true);
        on_btn_close_all_clicked();
        event->accept();
    } 
    else if (msgBox.clickedButton() == btn_keep_all) {
        appendLog("<b>[EXIT]</b> Keeping all remote process states, exiting control panel directly.");
        event->accept();
    } 
    else if (msgBox.clickedButton() == btn_cancel) {
        // The user clicked Cancel or closed the dialog
        event->ignore(); 
    }
}

// Mouse click on the video display (GUI -> native image-coordinate conversion)
bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
    // If the user left-clicks label_video
    if (watched == ui->label_video && event->type() == QEvent::MouseButtonPress) {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        
        if (mouseEvent->button() == Qt::LeftButton) {
            // Ensure that an image stream is currently active
            if (!latest_image_.isNull()) {
                
                // Get the widget size and original image size
                int lw = ui->label_video->width();
                int lh = ui->label_video->height();
                int iw = latest_image_.width();
                int ih = latest_image_.height();

                QSize scaledSize = latest_image_.size().scaled(lw, lh, Qt::KeepAspectRatio);
                
                // Calculate the top/bottom or left/right padding
                int dx = (lw - scaledSize.width()) / 2;
                int dy = (lh - scaledSize.height()) / 2;

                int click_x = mouseEvent->x();
                int click_y = mouseEvent->y();

                // Check that the click actually lies inside the rendered image rather than on a black border）
                if (click_x >= dx && click_x <= dx + scaledSize.width() &&
                    click_y >= dy && click_y <= dy + scaledSize.height()) {
                    // Map it proportionally back to the original image pixel coordinates
                    float real_x = (click_x - dx) * (float)iw / scaledSize.width();
                    float real_y = (click_y - dy) * (float)ih / scaledSize.height();
                    // Publish to the ROS 2 vision node in the backend
                    qnode.pub_click_point(real_x, real_y);
                    appendLog(QString("<b>[GUI]</b> Clicked target at (%1, %2), commanding VISION node...").arg(real_x, 0, 'f', 1).arg(real_y, 0, 'f', 1));
                }
            }
        }
    }
    return QMainWindow::eventFilter(watched, event);
}
