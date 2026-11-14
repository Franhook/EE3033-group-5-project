#include "balance_gui/main_window.hpp"
#include "ui_main_window.h"

void MainWindow::initSSH()
{
    // Create a QProcess for testing the SSH connection: QProcess
    ssh_test_proc_ = new QProcess(this);
    // Capture standard error from the lower-level SSH probe
    connect(ssh_test_proc_, &QProcess::readyReadStandardError, this, [this]() {
        QString err_log = QString::fromLocal8Bit(ssh_test_proc_->readAllStandardError()).trimmed();
        if (err_log.contains("setlocale")) return; // Ignore unrelated error messages
        if (err_log.contains("Permanently added")) return;
        if (!err_log.isEmpty()) {
            QString prefix = use_ssh_ ? "<b>[SSH]</b>" : (use_docker_ ? "<b>[DOCKER]</b>" : "<b>[LOCAL]</b>");
            appendLog(QString("%1: %2").arg(prefix, err_log), true);
            ui->textEdit_logs->verticalScrollBar()->setValue(ui->textEdit_logs->verticalScrollBar()->maximum());
        }
    });
    connect(ssh_test_proc_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &MainWindow::slotConnectionTestFinished);
    
    connection_timer_ = new QTimer(this);
    connect(connection_timer_, &QTimer::timeout, this, &MainWindow::slotCheckConnection);
    connection_timer_->start(5000); 
    slotCheckConnection();          // Initial handshake

    // Initialize the status-synchronization timer
    monitor_timer_ = new QTimer(this);
    connect(monitor_timer_, &QTimer::timeout, this, &MainWindow::syncRemoteState);
}

// =========================================================
// sshConnection heartbeat detection and status synchronization
// =========================================================
void MainWindow::slotCheckConnection()
{
    if (ssh_test_proc_->state() != QProcess::NotRunning) return;

    if (use_ssh_) {
        // Scenario 1: remotely control the Raspberry Pi from a virtual machine
        QString test_cmd = QString("sshpass -p '%1' ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=5 %2@%3 'echo connected'")
                            .arg(remote_pwd_, remote_user_, remote_ip_);
        ssh_test_proc_->start("sh", QStringList() << "-c" << test_cmd);
    } else if (use_docker_) {
        // Scenario 2: run on the Raspberry Pi host and control Docker
        QString test_cmd = QString("docker ps -q -f name=%1").arg(docker_name_);
        ssh_test_proc_->start("sh", QStringList() << "-c" << test_cmd);
    } else {
        // Scenario 3: run inside Raspberry Pi Docker (fully local environment)
        ssh_test_proc_->start("sh", QStringList() << "-c" << "echo connected");
    }
}

void MainWindow::slotConnectionTestFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    QString prefix = use_ssh_ ? "<b>[SSH]</b>" : (use_docker_ ? "<b>[DOCKER]</b>" : "<b>[LOCAL]</b>");

    if (exitStatus == QProcess::NormalExit && exitCode == 0) {
        QString output = QString::fromLocal8Bit(ssh_test_proc_->readAllStandardOutput()).trimmed();
        
        // Scenario 2 outputs the container ID; scenarios 1 and 3 output "connected"
        if (output == "connected" || (!use_ssh_ && !output.isEmpty())) {
            if (!is_online_) {
                is_online_ = true;
                ui->label_status_text->setText("Online");
                ui->label_status_text->setStyleSheet("color: green; font-weight: bold;");
                setControlButtonsEnabled(true);
                appendLog(QString("%1 Connection established!").arg(prefix));
                // Synchronize status
                syncRemoteState();
                // After connection succeeds, adjust the heartbeat frequency and start the status synchronizer
                connection_timer_->setInterval(10000); 
                monitor_timer_->start(5000);
            }
            return;
        }
    }
    
    // Connection-failure handling
    if (is_online_) {
        is_online_ = false;
        ui->label_status_text->setText("Offline");
        ui->label_status_text->setStyleSheet("color: red; font-weight: bold;");
        setControlButtonsEnabled(false);
        appendLog(QString("%1 Connection to target environment lost, attempting to reconnect...").arg(prefix), true);
        connection_timer_->setInterval(3000);
        monitor_timer_->stop(); // Stop polling while offline
    }
}

void MainWindow::syncRemoteState()
{
    // Synchronize Docker status
    if (use_docker_) {
        QString check_cmd = QString("docker ps -q -f name=%1").arg(docker_name_);
        if (use_ssh_) {
            check_cmd = QString("sshpass -p '%1' ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null %2@%3 '%4'")
                        .arg(remote_pwd_, remote_user_, remote_ip_, check_cmd);
        }
        
        QProcess *doc_proc = new QProcess(this);
        connect(doc_proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, [this, doc_proc](int exitCode, QProcess::ExitStatus exitStatus) {

            QString output = QString::fromLocal8Bit(doc_proc->readAllStandardOutput()).trimmed();
            bool current_state = !output.isEmpty();
            // Update the UI and print a log only when the status changes
            if (current_state != is_docker_on_) {
                is_docker_on_ = current_state;
                if (is_docker_on_) {
                    ui->btn_cmd_docker->setText("Stop Docker");
                    ui->btn_cmd_docker->setStyleSheet("background-color: #f44336; color: white; font-weight: bold;");
                    appendLog("<b>[DOCKER]</b> Container is now RUNNING.");
                } 
                else {
                    ui->btn_cmd_docker->setText("Start Docker");
                    ui->btn_cmd_docker->setStyleSheet("");
                    appendLog("<b>[DOCKER]</b> Container is now STOPPED.");
                }
            }
            doc_proc->deleteLater();
            // Docker After confirming it is running, check the node processes inside
            if (is_docker_on_) checkRemoteProcesses();
        });
        doc_proc->start("sh", QStringList() << "-c" << check_cmd);
    } 
    else {
        // When Docker is not used, check the host processes directly
        checkRemoteProcesses();
    }
}

void MainWindow::checkRemoteProcesses()
{
    // ps aux List all processes for subsequent keyword matching
    QString exec_cmd = buildCommand("ps aux");
    
    QProcess *ps_proc = new QProcess(this);
    connect(ps_proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this, ps_proc](int exitCode, QProcess::ExitStatus exitStatus) {
        QString output = QString::fromLocal8Bit(ps_proc->readAllStandardOutput());
        
        auto sync_btn = [this, output](QPushButton* btn, const QString& keyword, const QString& node_name) {
            bool is_running = output.contains(keyword);
            QString text = btn->text();

            // Case 1: the node is running but the button is the white "Open" state; turn it green and take control。
            if (is_running && text.contains("Open")) {
                btn->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;");
                QString text = btn->text();
                text.replace("Open", "Close");
                btn->setText(text);
                btn->setEnabled(true);
                appendLog(QString("<b>[SYNC]</b> Status update: [%1] is running.").arg(node_name));
            }
            // Case 2: the node has stopped and the button shows "Close"; return it to white。
            else if (!is_running && text.contains("Close")) {
                btn->setStyleSheet("");
                QString text = btn->text();
                text.replace("Close", "Open");
                btn->setText(text);
                btn->setEnabled(true);
                appendLog(QString("<b>[SYNC]</b> Status update: [%1] stopped.").arg(node_name));
            }
            // If the process really stopped, the button is white and says Open but is disabled
            else if (!is_running && text.contains("Open") && !btn->isEnabled()) {
                btn->setEnabled(true);
            }
        };
        
        // Match using the same keywords as startup (generic synchronization scan)
        sync_btn(ui->btn_open_base, "balance_robot_base.launch.py", "BASE");
        sync_btn(ui->btn_open_lidar, "lslidar_launch.py", "LiDAR");
        sync_btn(ui->btn_open_usb_camera, "usb_cam.launch.py", "USBCam");
        sync_btn(ui->btn_open_vision, "balance_object_detector.launch.py", "VISION");
        sync_btn(ui->btn_open_mixed_follower, "balance_mixed_follower.launch.py", "FOLLOWER");
        sync_btn(ui->btn_open_map, "balance_mapping_slamtoolbox", "MAP"); // slam Match prefix
        sync_btn(ui->btn_open_nav, "balance_nav2_core.launch.py", "NAV2");
        sync_btn(ui->btn_open_localization, "balance_amcl_localization.launch.py", "LOCALIZATION");
        sync_btn(ui->btn_map_explore, "balance_explorer_lite.launch.py", "EXPLORER");
        sync_btn(ui->btn_open_patrol, "balance_patrol.launch.py", "PATROL");
        // Synchronization scan for a specific button
        if (ui->btn_open_localization->text().contains("Close")) {
            if (!ui->btn_amcl_relocate->isEnabled()) {
                ui->btn_amcl_relocate->setEnabled(true);
                ui->btn_amcl_relocate->setStyleSheet(""); 
                ui->btn_amcl_relocate->setText("Relocate");
            }
        } 
        else {
            ui->btn_amcl_relocate->setEnabled(false);
            ui->btn_amcl_relocate->setStyleSheet(""); 
            ui->btn_amcl_relocate->setText("Relocate");
        }
        bool is_base_active = ui->btn_open_base->text().contains("Close");
        ui->tab_pid->setEnabled(is_base_active);
        
        ps_proc->deleteLater();
    });
    ps_proc->start("sh", QStringList() << "-c" << exec_cmd);
}

// =========================================================
// Command-generation engine
// =========================================================
QString MainWindow::buildCommand(const QString& ros2_cmd)
{
    QString final_cmd = ros2_cmd;
    // Docker Environment passthrough
    if (use_docker_) {
        final_cmd = QString("docker exec %1 bash -c \"source ~/.bashrc && cd /home/ros2_ws && source install/setup.bash && %2\"")
                    .arg(docker_name_, ros2_cmd);
    }
    // SSH Remote passthrough
    if (use_ssh_) {
        QString escaped_cmd = final_cmd;
        escaped_cmd.replace("\"", "\\\"");
        final_cmd = QString("sshpass -p '%1' ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null %2@%3 \"%4\"")
                    .arg(remote_pwd_, remote_user_, remote_ip_, escaped_cmd);
    }
    return final_cmd + "\n";
}

QString MainWindow::buildKillCommand(const QString& keyword)
{
    QString remote_kill = QString("pkill -2 -f '%1'").arg(keyword);
    if (use_docker_) {
        remote_kill = QString("docker exec %1 sh -c \"%2\"").arg(docker_name_, remote_kill);
    }
    if (use_ssh_) {
        QString escaped_cmd = remote_kill;
        escaped_cmd.replace("\"", "\\\"");
        remote_kill = QString("sshpass -p '%1' ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null %2@%3 \"%4\"")
                      .arg(remote_pwd_, remote_user_, remote_ip_, escaped_cmd);
    }
    return remote_kill;
}

// =========================================================
// Node startup and control logic
// =========================================================
bool MainWindow::toggleNodeProcess(QProcess *&proc, QPushButton *btn, const QString &ros2_cmd, 
                                   const QString &node_name, const QString &kill_keyword, 
                                   std::function<void(const QString&)> log_callback)
{
    // State A: prepare to close the node
    if (btn->text().contains("Close")) {
        appendLog(QString("<b>[NODE]</b> Stopping node [%1]...").arg(node_name));
        btn->setStyleSheet("");
        QString text = btn->text();
        text.replace("Close", "Open");
        btn->setText(text);
        btn->setEnabled(false);

        // Send pkill -2 to stop it
        QString kill_cmd = buildKillCommand(kill_keyword);
        QProcess::execute("sh", QStringList() << "-c" << kill_cmd); 
        if (proc) {
            proc->disconnect(); // Disconnect the signal to prevent QProcess from crashing
            proc->kill(); 
            proc->deleteLater(); 
            proc = nullptr;
        }
        // 1.5 seconds later, verify the process status
        QTimer::singleShot(1500, this, &MainWindow::checkRemoteProcesses);
        return false; 
    }
    // State B: prepare to start the node
    else {
        if (!proc) {
            proc = new QProcess(this);
            // Connect standard output
            connect(proc, &QProcess::readyReadStandardOutput, this, [this, proc, node_name, log_callback]() {
                QString log = QString::fromLocal8Bit(proc->readAllStandardOutput()).trimmed();
                if(!log.isEmpty()) appendLog(QString("<b>[%1]</b>: %2").arg(node_name, log));
                ui->textEdit_logs->verticalScrollBar()->setValue(ui->textEdit_logs->verticalScrollBar()->maximum());
                if (log_callback) log_callback(log);
            });
            // Connect standard error
            connect(proc, &QProcess::readyReadStandardError, this, [this, proc, node_name, log_callback]() {
                QString log = QString::fromLocal8Bit(proc->readAllStandardError()).trimmed();
                if (log.contains("setlocale") || log.contains("Permanently added")) return;
                if (log.contains("[INFO]") || log.contains("[WARN]")) {
                    appendLog(QString("<b>[%1]</b>: <font color=\"#2196f3\">%2</font>").arg(node_name, log));
                } 
                else if (!log.isEmpty()) {
                    appendLog(QString("<b>[%1]</b>: %2").arg(node_name, log), true);
                }
                ui->textEdit_logs->verticalScrollBar()->setValue(ui->textEdit_logs->verticalScrollBar()->maximum());
                if (log_callback) log_callback(log);
            });
            
            // Monitor unexpected or requested process termination and safely reset the button
            QProcess **proc_ptr = &proc; 
            connect(proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                    this, [this, btn, node_name, proc_ptr](int exitCode, QProcess::ExitStatus exitStatus) {
                // Restore the button UI
                btn->setStyleSheet("");
                QString text = btn->text();
                text.replace("Close", "Open");
                btn->setText(text);
                
                if (exitStatus == QProcess::CrashExit || exitCode != 0) {
                    appendLog(QString("<b>[NODE]</b> [%1] node has been stopped/crashed (ExitCode: %2)").arg(node_name).arg(exitCode), true);
                } 
                else {
                    appendLog(QString("<b>[NODE]</b> [%1] node has been stopped.").arg(node_name));
                }
                (*proc_ptr)->deleteLater();
                *proc_ptr = nullptr;
            });
        }

        QString cmd = buildCommand(ros2_cmd);
        proc->start("sh", QStringList() << "-c" << cmd);
        
        // Change the button UI to the running state (Open -> Close)
        btn->setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;");
        QString current_text = btn->text();
        current_text.replace("Open", "Close");
        btn->setText(current_text);
        
        appendLog(QString("<b>[NODE]</b> Starting node [%1]...").arg(node_name));
        return true; // Return true to indicate that it is currently running
    }
}

// =========================================================
// Functions Shortcut functions
// --- SSH Control-area button slot functions ---
// =========================================================
void MainWindow::on_btn_open_base_clicked()
{
    bool is_running = toggleNodeProcess(proc_base_, ui->btn_open_base, 
                      "ros2 launch balance_robot_base balance_robot_base.launch.py", "BASE", "balance_robot_base.launch.py");

    if (is_running) {
        // After the chassis starts successfully, enable the checkbox
        ui->checkBox_sacc->setEnabled(true);
        ui->checkBox_teleop->setEnabled(true);
        ui->tab_pid->setEnabled(true);
        // On startup, immediately synchronize the current checkbox state with the Raspberry Pi backend
        QTimer::singleShot(1000, this, [this]() {
            on_checkBox_sacc_stateChanged(ui->checkBox_sacc->checkState());
        });
        // Whenever the chassis process terminates
        connect(proc_base_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, [this]() { 
                    ui->checkBox_sacc->setEnabled(false); 
                    ui->tab_pid->setEnabled(false);
                }, Qt::UniqueConnection);
    } 
    else {
        // When the chassis is stopped, disable the checkbox
        ui->checkBox_sacc->setEnabled(false);
        ui->tab_pid->setEnabled(false);
        ui->checkBox_teleop->setEnabled(false);
    }
}

// S S-curve smooth-acceleration control
void MainWindow::on_checkBox_sacc_stateChanged(int state)
{
    if (!proc_base_ || proc_base_->state() == QProcess::NotRunning) {
        return;
    }
    QString is_enable = (state == Qt::Checked) ? "true" : "false";
    // Build the ROS 2 parameter command
    QString raw_cmd = QString("ros2 param set /balance_robot_node enable_s_curve %1").arg(is_enable);
    QString cmd = buildCommand(raw_cmd);
    QProcess::startDetached("sh", QStringList() << "-c" << cmd);
    // Print a message in the log panel on the right side of the GUI
    QString status_text = (state == Qt::Checked) ? "<font color='green'>Enabled</font>" : "<font color='red'>Disabled</font>";
    appendLog(QString("<b>[BASE]</b> S-Curve Acceleration: %1").arg(status_text));
}

void MainWindow::on_btn_open_lidar_clicked()
{
    toggleNodeProcess(proc_lidar_, ui->btn_open_lidar, 
                      "ros2 launch lslidar_driver_n10p lslidar_launch.py", "LiDAR", "lslidar_launch.py");
}

void MainWindow::on_btn_open_usb_camera_clicked()
{
    // If the vision node is using the camera, prevent the user from stopping or restarting the camera separately
    if (proc_vision_ && proc_vision_->state() != QProcess::NotRunning) {
        appendLog("<b>[CAMERA]</b> Camera is currently managed by the VISION Node. Please shut down the VISION Node first.", true);
        return;
    }
    // Start the camera independently and pass mode:=default
    toggleNodeProcess(proc_camera_, ui->btn_open_usb_camera, 
                      "ros2 launch balance_usb_cam usb_cam.launch.py mode:=default", 
                      "USBCam", "usb_cam.launch.py");
}

void MainWindow::on_btn_open_vision_clicked()
{
    // --- State A: prepare to start the vision node ---
    if (!proc_vision_ || proc_vision_->state() == QProcess::NotRunning) {
        
        // If the standalone camera node is running in default mode, stop it first to release the camera
        if (proc_camera_ && proc_camera_->state() != QProcess::NotRunning) {
            appendLog("<b>[CAMERA]</b> Restarting camera with 'detector' configuration...", true);
            // Passing an empty command is equivalent to executing the stop logic
            toggleNodeProcess(proc_camera_, ui->btn_open_usb_camera, "", "USBCam", "usb_cam.launch.py"); 
        }
        // Automatically start the camera with mode:=detector and load the distortion-correction intrinsics
        toggleNodeProcess(proc_camera_, ui->btn_open_usb_camera, 
                          "ros2 launch balance_usb_cam usb_cam.launch.py mode:=detector", 
                          "USBCam", "usb_cam.launch.py");
    } 
    // --- State B: prepare to stop the vision node ---
    else {
        // Also stop the camera that was started above
        if (proc_camera_ && proc_camera_->state() != QProcess::NotRunning) {
            appendLog("<b>[CAMERA]</b> VISION node stopping, shutting down camera synchronously...");
            toggleNodeProcess(proc_camera_, ui->btn_open_usb_camera, "", "USBCam", "usb_cam.launch.py"); 
        }
    }

    // Toggle the state of the vision algorithm node itself
    bool is_running = toggleNodeProcess(proc_vision_, ui->btn_open_vision, 
                      "ros2 launch balance_object_detect balance_object_detector.launch.py", 
                      "VISION", "balance_object_detector.launch.py");
                      
    if (is_running && proc_vision_) {
        // Connect the process termination signal and lock the button
        connect(proc_vision_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), 
            this, [this]() {
                ui->btn_pub_target->setEnabled(false);
                ui->btn_reset_target->setEnabled(false);
        }, Qt::UniqueConnection);
    } 
    else {
        // The user explicitly requested shutdown, so lock the button directly
        ui->btn_pub_target->setEnabled(false);
        ui->btn_reset_target->setEnabled(false);
    }
}

// -------------------------------------------------
// Start the core Nav2 nodes
void MainWindow::on_btn_open_nav_clicked()
{
    if (ui->btn_open_nav->text().contains("Open")) {
        if (!ui->btn_open_base->text().contains("Close")) on_btn_open_base_clicked();
        if (!ui->btn_open_lidar->text().contains("Close")) on_btn_open_lidar_clicked();
    }

    toggleNodeProcess(proc_nav_, ui->btn_open_nav, 
                      "ros2 launch balance_navigation balance_nav2_core.launch.py", 
                      "NAV2", "balance_nav2_core.launch.py");
}

// ------------------------------------------------
// Start the fused-following node
void MainWindow::on_btn_open_mixed_follower_clicked()
{
    // If the system is locked while waiting for AMCL convergence, allow the user to cancel
    if (ui->btn_open_mixed_follower->text() == "Cancel Wait") {
        ui->btn_open_mixed_follower->setText("Open Mixed Follower");
        ui->btn_open_mixed_follower->setStyleSheet("");
        appendLog("<b>[FOLLOWER]</b> Auto-start aborted by user.");
        return;
    }

    if (ui->btn_open_mixed_follower->text().contains("Open")) {
        // If AMCL is relocalizing or diverging, forcibly block the request and queue it
        if (ui->btn_amcl_relocate->text() == "Unconverged") {
            appendLog("<b>[FOLLOWER]</b> Map is unaligned. Queueing Follower to start after convergence...");
            ui->btn_open_mixed_follower->setText("Cancel Wait");
            ui->btn_open_mixed_follower->setStyleSheet("background-color: #FF9800; color: white; font-weight: bold;");
            QTimer* wait_timer = new QTimer(this);
            connect(wait_timer, &QTimer::timeout, this, [this, wait_timer]() {
                if (ui->btn_open_mixed_follower->text() != "Cancel Wait") {
                    wait_timer->stop();
                    wait_timer->deleteLater();
                    return;
                }
                if (ui->btn_amcl_relocate->text() == "Localized") {
                    wait_timer->stop();
                    wait_timer->deleteLater();
                    ui->btn_open_mixed_follower->setText("Open Mixed Follower");
                    ui->btn_open_mixed_follower->setStyleSheet("");
                    appendLog("<b>[FOLLOWER]</b> Localization converged! Launching queued FOLLOWER node...");
                    bool is_running = toggleNodeProcess(proc_follower_, ui->btn_open_mixed_follower, 
                                    "ros2 launch balance_follower balance_mixed_follower.launch.py", "FOLLOWER", "balance_mixed_follower.launch.py");
                    ui->checkBox_follower_cruise->setEnabled(!is_running);
                }
            });
            wait_timer->start(500); 
            return; 
        }

        // Start the hardware perception layer
        if (!ui->btn_open_base->text().contains("Close")) {
            appendLog("<b>[FOLLOWER]</b> Starting [BASE]...");
            on_btn_open_base_clicked();
        }
        if (!ui->btn_open_lidar->text().contains("Close")) {
            appendLog("<b>[FOLLOWER]</b> Starting [LiDAR]...");
            on_btn_open_lidar_clicked();
        }
        if (!ui->btn_open_vision->text().contains("Close")) {
            appendLog("<b>[FOLLOWER]</b> Starting [VISION]...");
            on_btn_open_vision_clicked();
        }

        // Check whether lost-target patrol is enabled
        if (ui->checkBox_follower_cruise->isChecked()) {
            // Automatically start the localization node
            if (!ui->btn_open_localization->text().contains("Close")) {
                appendLog("<b>[FOLLOWER]</b> Auto-starting [LOCALIZATION] for cruise capability...");
                on_btn_open_localization_clicked();
            }
            // As long as localization is not green, suspend startup of the following node
            if (ui->btn_amcl_relocate->text() != "Localized") {
                appendLog("<b>[FOLLOWER]</b> Waiting for AMCL Localization to turn Green...");
                ui->btn_open_mixed_follower->setText("Cancel Wait");
                ui->btn_open_mixed_follower->setStyleSheet("background-color: #FF9800; color: white; font-weight: bold;");
                
                QTimer* wait_timer = new QTimer(this);
                connect(wait_timer, &QTimer::timeout, this, [this, wait_timer]() {
                    if (ui->btn_open_mixed_follower->text() != "Cancel Wait") {
                        wait_timer->stop(); wait_timer->deleteLater(); return;
                    }
                    if (ui->btn_amcl_relocate->text() == "Localized") {
                        wait_timer->stop(); wait_timer->deleteLater();
                        ui->btn_open_mixed_follower->setText("Open Mixed Follower");
                        ui->btn_open_mixed_follower->setStyleSheet("");
                        appendLog("<b>[FOLLOWER]</b> Localization turned Green! Launching FOLLOWER node now...");
                        bool is_running = toggleNodeProcess(proc_follower_, ui->btn_open_mixed_follower, 
                                          "ros2 launch balance_follower balance_mixed_follower.launch.py", "FOLLOWER", "balance_mixed_follower.launch.py");
                        ui->checkBox_follower_cruise->setEnabled(!is_running);
                    }
                });
                wait_timer->start(500); 
                return; 
            }
        }
    }

    // Normal start/stop logic (when patrol is not involved or localization has converged）
    bool is_running = toggleNodeProcess(proc_follower_, ui->btn_open_mixed_follower, 
                      "ros2 launch balance_follower balance_mixed_follower.launch.py", "FOLLOWER", "balance_mixed_follower.launch.py");
    ui->checkBox_follower_cruise->setEnabled(!is_running);              
}

// Whether automatic patrol is allowed after the target is lost
void MainWindow::on_checkBox_follower_cruise_stateChanged(int state)
{
    if (state == Qt::Checked) {
        // If enabled without a selected image, force a switch to image selection
        if (ui->lineEdit_select_map->text().trimmed().isEmpty()) {
            QMessageBox::information(this, "Select Map", "Please select a map for Auto-Cruising first.");
            on_btn_browse_map_clicked(); // Invoke the image-selection slot
            
            // If the image is still empty after returning (the user clicked Cancel), clear the checkbox
            if (ui->lineEdit_select_map->text().trimmed().isEmpty()) {
                ui->checkBox_follower_cruise->setChecked(false);
            }
        }
        if (ui->checkBox_follower_cruise->isChecked()) {
            appendLog("<b>[FOLLOWER]</b> Target-loss Auto-Cruising feature is ENABLED.");
        }
    } else {
        appendLog("<b>[FOLLOWER]</b> Target-loss Auto-Cruising feature is DISABLED.");
    }
}

// ------------------------------------------------
// SLAM Mapping
void MainWindow::on_btn_open_map_clicked()
{
    if (ui->btn_open_map->text().contains("Open")){
        if (!ui->btn_open_base->text().contains("Close")) on_btn_open_base_clicked();
        if (!ui->btn_open_lidar->text().contains("Close")) on_btn_open_lidar_clicked();
    }

    QString method = ui->comboBox_map_method->currentText();
    bool is_auto = method.contains("Auto");

    // State A: prepare to start mapping
    if (ui->btn_open_map->text().contains("Open")) {
        if (is_auto) {
            // Auto mode requires the Nav2 core
            if (!ui->btn_open_nav->text().contains("Close")) {
                appendLog("<b>[MAPPING]</b> Auto-starting [NAV2] for autonomous exploration...");
                on_btn_open_nav_clicked();
            }
        }
    }
    // State B: prepare to stop mapping
    else {
        // When stopping mapping, also stop the explorer if its button is active
        if (ui->btn_map_explore->text().contains("Close")) {
            appendLog("<b>[MAPPING]</b> Stopping exploration node synchronously...");
            on_btn_map_explore_clicked();
        }
    }

    if (is_auto) {
        ui->lineEdit_select_map->clear();
    }
    // In either mode, the backend actually starts the base SLAM
    QString launch_file = method.contains("Sync") ? "balance_mapping_slamtoolbox_sync.launch.py" : "balance_mapping_slamtoolbox_async.launch.py";
    QString cmd = QString("ros2 launch balance_mapping %1").arg(launch_file);
    
    bool is_running = toggleNodeProcess(proc_mapping_, ui->btn_open_map, cmd, "MAP", "balance_mapping_slamtoolbox");

    if (is_running) {
        ui->comboBox_map_method->setEnabled(false);
        ui->btn_map_save->setEnabled(true);
        connect(proc_mapping_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, [this]() { ui->btn_map_save->setEnabled(false); }, Qt::UniqueConnection);
                
        // In Auto mode, start the explorer after a 10-second delay to stagger startup）
        if (is_auto) {
            appendLog("<b>[MAPPING]</b> Auto mode selected. Exploration node will start in 10 seconds...");
            QTimer::singleShot(10000, this, [this]() {
                if (ui->btn_open_map->text().contains("Close")) {
                    if (!ui->btn_map_explore->text().contains("Close")) {
                        on_btn_map_explore_clicked(); // Start explore
                    }
                }
            });
        }
    } 
    else {
        ui->comboBox_map_method->setEnabled(true);
        ui->btn_map_save->setEnabled(false);
    }
}

// Explorer-node toggle logic
void MainWindow::on_btn_map_explore_clicked() 
{
    bool is_running = toggleNodeProcess(proc_explore_, ui->btn_map_explore, 
        "ros2 launch balance_mapping balance_explorer_lite.launch.py",
        "EXPLORER", "balance_explorer_lite.launch.py",
        // Log-analysis logic
        [this](const QString& log) {
            if (log.contains("All frontiers traversed") || log.contains("Exploration stopped")) {
                if (ui->btn_map_explore->text().contains("Close")) { 
                    appendLog("<b>[EXPLORER]</b> Map completed! Auto-returning to origin (0,0)...");
                    
                    QString raw_cmd = "ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "
                                      "\"{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}\"";
                    QProcess::startDetached("sh", QStringList() << "-c" << buildCommand(raw_cmd));
                    
                    // Use a single-shot timer to stop asynchronously, avoiding a crash caused by killing the process from inside the log-parser call stack
                    QTimer::singleShot(0, this, &MainWindow::on_btn_map_explore_clicked);
                }
            }
        });
    
    if (!is_running) {
        appendLog("<b>[EXPLORER]</b> Canceling active Nav2 exploration goals...");
        qnode.cancel_nav2_goals();
    }
}

void MainWindow::on_btn_map_save_clicked()
{
    // Generate a default name
    QString default_name = "map_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    
    // Open a dialog for the user to edit or confirm the name
    bool ok;
    QString map_name = QInputDialog::getText(this, "Save Map",
                                             "Enter map name (will be saved on Raspberry Pi):",
                                             QLineEdit::Normal, default_name, &ok);
    if (!ok || map_name.isEmpty()) return;
    if (map_name.endsWith(".yaml") || map_name.endsWith(".pgm")) {
        map_name = map_name.split(".")[0];
    }
    appendLog(QString("<b>[MAPPING]</b> Saving map as [%1] on target device...").arg(map_name));

    // Build the map-saving command to execute on the Raspberry Pi
    QString raw_cmd = QString("mkdir -p /home/ros2_ws/src/balance_mapping/maps && "
                              "cd /home/ros2_ws/src/balance_mapping/maps && "
                              "ros2 run nav2_map_server map_saver_cli -f %1").arg(map_name);
    
    QString cmd = buildCommand(raw_cmd);

    // Execute the command and capture the result
    QProcess *proc_save = new QProcess(this);
    connect(proc_save, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this, proc_save](int exitCode, QProcess::ExitStatus exitStatus) {
        if (exitCode == 0) {
            appendLog("<b>[MAPPING]</b> Map saved successfully!");
        } else {
            appendLog("<b>[MAPPING]</b> Failed to save map.", true);
        }
        proc_save->deleteLater();
    });
    proc_save->start("sh", QStringList() << "-c" << cmd);
}

void MainWindow::on_comboBox_map_method_currentTextChanged(const QString &text)
{
    if (text.contains("Auto")) {
        ui->btn_map_explore->setEnabled(true);
    } else {
        ui->btn_map_explore->setEnabled(false);
        // When leaving Auto mode, automatically stop explore if it is still running
        if (proc_explore_ && proc_explore_->state() != QProcess::NotRunning) {
            on_btn_map_explore_clicked();
        }
    }
}

// ------------------------------------------------
// Select a static map
// ------------------------------------------------
// Select a map (supports remote and local modes)
void MainWindow::on_btn_browse_map_clicked()
{
    // Absolute path used to store maps on the Raspberry Pi
    QString default_dir = "/home/ros2_ws/src/balance_mapping/maps";

    if (use_ssh_ || use_docker_) {
        // Build a query command that lists all YAML files in the directory
        QString raw_cmd = QString("ls -1 %1/*.yaml 2>/dev/null").arg(default_dir);
        QString cmd = buildCommand(raw_cmd);

        QProcess proc;
        proc.start("sh", QStringList() << "-c" << cmd);
        
        // Wait synchronously for at most 3 seconds
        if (!proc.waitForFinished(3000)) {
            QMessageBox::warning(this, "Timeout", "Failed to fetch map list. Connection timed out.");
            return;
        }

        // Read the terminal output
        QString output = QString::fromLocal8Bit(proc.readAllStandardOutput()).trimmed();
        QStringList map_files = output.split('\n', Qt::SkipEmptyParts);
        if (map_files.isEmpty()) {
            QMessageBox::warning(this, "Not Found", "No map files (.yaml) found in target directory:\n" + default_dir);
            return;
        }

        // Extract plain filenames for display in the combo box
        QStringList map_names;
        for (const QString& path : map_files) {
            QString clean_path = path.trimmed();
            // Ensure the filename ends in yaml
            if (clean_path.endsWith(".yaml")) {
                map_names << clean_path.split('/').last();
            }
        }
        if (map_names.isEmpty()) {
            QMessageBox::warning(this, "Error", "Failed to parse remote map files.");
            return;
        }
        // Open a dialog with a drop-down list for selection
        bool ok;
        QString selected_name = QInputDialog::getItem(this, "Select Remote Map",
                                                      "Available maps on Target Device:",
                                                      map_names, 0, false, &ok);
        if (ok && !selected_name.isEmpty()) {
            // Reconstruct the absolute path required by the remote side
            QString full_path = default_dir + "/" + selected_name;
            ui->lineEdit_select_map->setText(full_path);
            appendLog("<b>[LOCALIZATION]</b> Selected map: " + full_path);
        }
    }
    else {
        // When running directly on the Raspberry Pi without Docker, open the system file chooser
        QString fileName = QFileDialog::getOpenFileName(this, 
            tr("Select Map"), default_dir, tr("Map Files (*.yaml)"));
        
        if (!fileName.isEmpty()) {
            ui->lineEdit_select_map->setText(fileName);
            appendLog("<b>[LOCALIZATION]</b> Selected local map: " + fileName);
        }
    }
}

// AMCLAMCL localization node
void MainWindow::on_btn_open_localization_clicked()
{
    if (ui->btn_open_localization->text().contains("Open")) {
        if (!ui->btn_open_base->text().contains("Close")) on_btn_open_base_clicked();
        if (!ui->btn_open_lidar->text().contains("Close")) on_btn_open_lidar_clicked();

        QString map_path = ui->lineEdit_select_map->text().trimmed();
        if (map_path.isEmpty()) {
            QMessageBox::warning(this, "Warning", "Please select a map file (.yaml) first!");
            return;
        }
        if (!ui->btn_open_nav->text().contains("Close")) {
            appendLog("<b>[LOCALIZATION]</b> Auto-starting [NAV2]...");
            on_btn_open_nav_clicked();
        }
    }

    QString map_path = ui->lineEdit_select_map->text().trimmed();
    QString raw_cmd = QString("ros2 launch balance_navigation balance_amcl_localization.launch.py map:=%1").arg(map_path);
    bool is_running = toggleNodeProcess(proc_localization_, ui->btn_open_localization, raw_cmd, "LOCALIZATION", "balance_amcl_localization.launch.py");

    if (is_running) {
        ui->btn_browse_map->setEnabled(false);
        ui->lineEdit_select_map->setEnabled(false);
        
        // When starting localization, initialize it to the white idle state and enable it
        ui->btn_amcl_relocate->setEnabled(true);
        ui->btn_amcl_relocate->setStyleSheet("background-color: #F44336; color: white; font-weight: bold;");
        ui->btn_amcl_relocate->setText("Unconverged");

        // After a delay, automatically trigger global relocalization once
        // QTimer::singleShot(3000, this, [this]() {
        //     if (ui->btn_open_localization->text().contains("Close")) {
        //         if (ui->btn_amcl_relocate->text() != "Localized") {
        //             appendLog("<b>[LOCALIZATION]</b> Auto-triggering initial relocate...");
        //             on_btn_amcl_relocate_clicked();
        //         }
        //     }
        // });

        connect(proc_localization_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, [this]() {
                    ui->btn_browse_map->setEnabled(true);
                    ui->lineEdit_select_map->setEnabled(true);
                    // When stopping localization, disable it and restore the white state
                    ui->btn_amcl_relocate->setEnabled(false);
                    ui->btn_amcl_relocate->setStyleSheet(""); 
                    ui->btn_amcl_relocate->setText("Relocate");
                }, Qt::UniqueConnection);
        appendLog("<b>[LOCALIZATION]</b> AMCL Localization ENABLED.");
    }
    else {
        ui->btn_browse_map->setEnabled(true);
        ui->lineEdit_select_map->setEnabled(true);
        ui->btn_amcl_relocate->setEnabled(false);
        ui->btn_amcl_relocate->setStyleSheet(""); 
        ui->btn_amcl_relocate->setText("Relocate");
    }
}

// AMCL Global relocalization
void MainWindow::on_btn_amcl_relocate_clicked()
{
    if (!proc_localization_ || proc_localization_->state() == QProcess::NotRunning) {
        QMessageBox::warning(this, "Warning", "Please Start Localization first!");
        return;
    }

    // Absolute preemption: stop nodes that may be competing for /cmd_vel control
    bool resume_follower = false;
    if (ui->btn_open_mixed_follower->text().contains("Close")) {
        appendLog("<b>[LOCALIZATION]</b> Temporarily suspending Follower for Relocation...");
        on_btn_open_mixed_follower_clicked(); // Simulate clicking the button to stop the following node
        resume_follower = true;
    }
    if (ui->btn_open_patrol->text().contains("Close")) {
        on_btn_open_patrol_clicked(); // Force-stop any active patrol action
    }

    // Set the red-light state
    ui->btn_amcl_relocate->setStyleSheet("background-color: #F44336; color: white; font-weight: bold;");
    ui->btn_amcl_relocate->setText("Unconverged");

    appendLog("<b>[LOCALIZATION]</b> Triggering Global Relocalization...");
    QString raw_cmd = "ros2 service call /reinitialize_global_localization std_srvs/srv/Empty {}";
    QProcess::startDetached("sh", QStringList() << "-c" << buildCommand(raw_cmd));

    QTimer::singleShot(800, this, [this, resume_follower]() {
        appendLog("<b>[LOCALIZATION]</b> Starting Auto-Spin to converge particles...");
        ui->comboBox_patrol_strategy->setCurrentText("Spin");
        if (ui->btn_open_patrol->text().contains("Open")) {
            on_btn_open_patrol_clicked();
        }
        
        // If following was suspended, issue another start command; it will be intercepted and held at "Cancel Wait" until the light turns green
        if (resume_follower) {
            QTimer::singleShot(500, this, [this]() {
                 if (ui->btn_open_mixed_follower->text().contains("Open")) {
                     on_btn_open_mixed_follower_clicked();
                 }
            });
        }
    });
}

// ------------------------------------------------
// Autonomous patrol node
void MainWindow::on_btn_open_patrol_clicked()
{
    // Patrol depends on all lower-level navigation components
    if (ui->btn_open_patrol->text().contains("Open")) {
        if (!ui->btn_open_nav->text().contains("Close")) {
            QMessageBox::warning(this, "Warning", "Please Start NAV2 Core first!");
            return;
        }
        if (!ui->btn_open_localization->text().contains("Close")) {
            QMessageBox::warning(this, "Warning", "Please Start AMCL Localization first!");
            return;
        }
    }

    // Get the selected patrol strategy
    QString strategy_text = ui->comboBox_patrol_strategy->currentText();
    QString strategy_arg = strategy_text.contains("Spin") ? "spin" : 
                       (strategy_text.contains("Mixed") ? "mixed" : 
                       (strategy_text.contains("Wander") ? "wander" : "random"));
    QString raw_cmd = QString("ros2 launch balance_navigation balance_patrol.launch.py strategy:=%1").arg(strategy_arg);

    // Toggle the process state
    bool is_running = toggleNodeProcess(proc_patrol_, ui->btn_open_patrol, raw_cmd, "PATROL", "balance_patrol.launch.py");

    // If is_running is false after toggling, the operation just performed was a stop
    if (!is_running) {
        appendLog("<b>[PATROL]</b> Canceling active Nav2 goals and braking...");
        qnode.cancel_nav2_goals();
    }
}

// Close all nodes
void MainWindow::on_btn_close_all_clicked() {
    appendLog("<b>[NODE]</b> Closing all nodes gracefully...");

    // Attempt to stop every running node with SIGINT (-2)
    if (ui->btn_open_mixed_follower->text().contains("Close") || ui->btn_open_mixed_follower->text() == "Cancel Wait") {
        on_btn_open_mixed_follower_clicked();
    }
    if (ui->btn_open_vision->text().contains("Close")) on_btn_open_vision_clicked();
    if (ui->btn_open_usb_camera->text().contains("Close")) on_btn_open_usb_camera_clicked();
    if (ui->btn_open_lidar->text().contains("Close")) on_btn_open_lidar_clicked();
    if (ui->btn_open_base->text().contains("Close")) on_btn_open_base_clicked();
    if (ui->btn_open_map->text().contains("Close")) on_btn_open_map_clicked();
    if (ui->btn_open_localization->text().contains("Close")) on_btn_open_localization_clicked();
    if (ui->btn_map_explore->text().contains("Close")) on_btn_map_explore_clicked();
    if (ui->btn_open_nav->text().contains("Close")) on_btn_open_nav_clicked();
    if (ui->btn_open_patrol->text().contains("Close")) on_btn_open_patrol_clicked();

    // Fallback cleanup (SIGKILL -9)
    QTimer::singleShot(4000, this, [this]() {
        QString kill_logic = "ps aux | egrep 'ros2|nav2|balance_|slam_toolbox|explore|lslidar|usb_cam|tf2_ros' | grep -v balance_gui | grep -v grep | awk '{print $2}' | xargs -r kill -9";
        QString kill_cmd = buildCommand(kill_logic);
        QProcess::startDetached("sh", QStringList() << "-c" << kill_cmd);
        appendLog("<b>[NODE]</b> Graceful period ended. Any remaining remote ROS 2 nodes have been FORCE KILLED.");
        // After force-killing processes, explicitly synchronize status once
        checkRemoteProcesses();
    });
}

// =========================================================
// Commands Shortcut command area
// Implementation of command-button slot functions
// =========================================================
void MainWindow::on_btn_cmd_docker_clicked() {
    if (!use_docker_) return;

    QString docker_cmd = QString("docker %1 %2").arg(is_docker_on_ ? "stop" : "start", docker_name_);
    if (use_ssh_) {
        docker_cmd = QString("sshpass -p '%1' ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null %2@%3 '%4'")
                        .arg(remote_pwd_, remote_user_, remote_ip_, docker_cmd);
    }

    if (is_docker_on_) {
        // Execute stop
        on_btn_close_all_clicked(); 
        appendLog("<b>[DOCKER]</b> Stopping Docker...");
        QProcess::execute("sh", QStringList() << "-c" << docker_cmd);

        ui->btn_cmd_docker->setText("Start Docker");
        ui->btn_cmd_docker->setStyleSheet("");
        is_docker_on_ = false;
        appendLog("<b>[DOCKER]</b> Docker container has been stopped.");
    } 
    else {
        // Execute start
        appendLog("<b>[DOCKER]</b> Starting Docker...");
        QProcess::execute("sh", QStringList() << "-c" << docker_cmd);

        ui->btn_cmd_docker->setText("Stop Docker");
        ui->btn_cmd_docker->setStyleSheet("background-color: #f44336; color: white; font-weight: bold;");
        is_docker_on_ = true;
        appendLog("<b>[DOCKER]</b> Docker container has been started.");
    }
}

void MainWindow::on_btn_cmd_nfs_clicked() {
    if (!use_ssh_) {
        appendLog("<b>[NFS]</b> NFS mounting is only available in SSH remote connection mode (local execution will be ignored).", true);
        return;
    }

    if (!is_nfs_mounted_) {
        appendLog("<b>[NFS]</b> Mounting NFS...");
        QString mount_cmd = QString("echo '%1' | sudo -S sh -c 'mkdir -p /mnt/pi5_ws && mount -t nfs %2:/home/balance/ros2_ws /mnt/pi5_ws'")
                            .arg(local_pwd_, remote_ip_);
        int ret = QProcess::execute("sh", QStringList() << "-c" << mount_cmd);
        if (ret == 0) {
            is_nfs_mounted_ = true;
            ui->btn_cmd_nfs->setText("NFS Umount");
            ui->btn_cmd_nfs->setStyleSheet("background-color: #2196F3; color: white; font-weight: bold;");
            appendLog("<b>[NFS]</b> NFS Mounted (Path: /mnt/pi5_ws)");
        } 
        else {
            appendLog("<b>[NFS]</b> NFS mounting failed. Please check the password or network configuration.", true);
        }
    } 
    else {
        appendLog("<b>[NFS]</b> Unmounting NFS...");
        QString umount_cmd = QString("echo '%1' | sudo -S umount /mnt/pi5_ws").arg(local_pwd_);
        QProcess::execute("sh", QStringList() << "-c" << umount_cmd);
        is_nfs_mounted_ = false;
        ui->btn_cmd_nfs->setText("NFS Mount");
        ui->btn_cmd_nfs->setStyleSheet("");
        appendLog("<b>[NFS]</b> NFS has been unmounted.");
    }
}

void MainWindow::on_btn_cmd_build_clicked() {
    // If a build is already running, terminate it first
    if (proc_build_ && proc_build_->state() != QProcess::NotRunning) {
        proc_build_->kill();
        proc_build_->waitForFinished(500);
    }

    if (!proc_build_) {
        proc_build_ = new QProcess(this);
        connect(proc_build_, &QProcess::readyReadStandardOutput, this, [this]() {
            QString log = QString::fromLocal8Bit(proc_build_->readAllStandardOutput()).trimmed();
            if(!log.isEmpty()) appendLog(QString("<font color=\"#607D8B\">[Build]: %1</font>").arg(log));
            ui->textEdit_logs->verticalScrollBar()->setValue(ui->textEdit_logs->verticalScrollBar()->maximum());
        });
        connect(proc_build_, &QProcess::readyReadStandardError, this, [this]() {
            QString log = QString::fromLocal8Bit(proc_build_->readAllStandardError()).trimmed();
            if (log.contains("setlocale") || log.contains("Permanently added")) return;
            if(!log.isEmpty()) appendLog(QString("<b>[Build]</b>: %1").arg(log), true);
            ui->textEdit_logs->verticalScrollBar()->setValue(ui->textEdit_logs->verticalScrollBar()->maximum());
        });
    }
    QString cmd = buildCommand("colcon build --symlink-install");
    proc_build_->start("sh", QStringList() << "-c" << cmd);
    appendLog("<b>[Build]</b> Compiling all packages...");
}

void MainWindow::on_btn_cmd_rqt_clicked()
{
    if (!proc_rqt_) {
        proc_rqt_ = new QProcess(this);
        connect(proc_rqt_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, [this]() { appendLog("<b>[RQT]</b> RQT tool has been closed."); });
    }
    if (proc_rqt_->state() == QProcess::NotRunning) {
        appendLog("<b>[RQT]</b> Starting local RQT tool...");
        proc_rqt_->start("rqt", QStringList());
    } 
    else {
        appendLog("<b>[RQT]</b> RQT is already running.", true);
    }
}

void MainWindow::on_btn_cmd_build_pkg_clicked()
{
    // Read the package name from the input box and trim whitespace
    QString target_pkg = ui->comboBox_build_pkg->currentText().trimmed();
    if (target_pkg.isEmpty()) {
        appendLog("<b>[Build]</b> Package name cannot be empty!", true);
        return;
    }

    // Reuse the build process
    if (proc_build_ && proc_build_->state() != QProcess::NotRunning) {
        proc_build_->kill();
        proc_build_->waitForFinished(500);
    }

    if (!proc_build_) {
        proc_build_ = new QProcess(this);
        connect(proc_build_, &QProcess::readyReadStandardOutput, this, [this]() {
            QString log = QString::fromLocal8Bit(proc_build_->readAllStandardOutput()).trimmed();
            if(!log.isEmpty()) appendLog(QString("<font color=\"#607D8B\">[Build]: %1</font>").arg(log));
            ui->textEdit_logs->verticalScrollBar()->setValue(ui->textEdit_logs->verticalScrollBar()->maximum());
        });
        connect(proc_build_, &QProcess::readyReadStandardError, this, [this]() {
            QString log = QString::fromLocal8Bit(proc_build_->readAllStandardError()).trimmed();
            if (log.contains("setlocale") || log.contains("Permanently added")) return;
            if(!log.isEmpty()) appendLog(QString("<b>[Build]</b>: %1").arg(log), true);
            ui->textEdit_logs->verticalScrollBar()->setValue(ui->textEdit_logs->verticalScrollBar()->maximum());
        });
    }
    QString raw_cmd = QString("colcon build --packages-select %1 --symlink-install").arg(target_pkg);
    QString cmd = buildCommand(raw_cmd);
    proc_build_->start("sh", QStringList() << "-c" << cmd);
    appendLog(QString("<b>[Build]</b> Compiling package: [%1]...").arg(target_pkg));
}

void MainWindow::on_btn_cmd_rviz_clicked()
{
    if (!proc_rviz_) {
        proc_rviz_ = new QProcess(this);
        connect(proc_rviz_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, [this]() { appendLog("<b>[RVIZ]</b> Local RViz2 has been closed."); });
    }
    if (proc_rviz_->state() == QProcess::NotRunning) {
        appendLog("<b>[RVIZ]</b> Starting local RViz2...");
        // Try to load the configuration file; if it is absent, open the default interface
        proc_rviz_->start("sh", QStringList() << "-c" << "rviz2 -d mapping.rviz || rviz2");
    } 
    else {
        appendLog("<b>[RVIZ]</b> RViz2 is already running.", true);
    }
}

void MainWindow::on_btn_cmd_tftree_clicked()
{
    if (proc_tftree_ && proc_tftree_->state() != QProcess::NotRunning) {
        appendLog("<b>[TF]</b> TF Tree generation is already in progress...", true);
        return;
    }

    proc_tftree_ = new QProcess(this);
    appendLog("<b>[TF]</b> Generating TF tree (listening for 5 seconds over DDS)...");

    connect(proc_tftree_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this]() {
        appendLog("<b>[TF]</b> TF tree generated successfully! Opening the newest PDF...");
        // Find the most recently timestamped frames*.pdf in the current directory and open it automatically
        QProcess::startDetached("sh", QStringList() << "-c" << "ls -t frames*.pdf | head -n 1 | xargs xdg-open");
        proc_tftree_->deleteLater();
        proc_tftree_ = nullptr;
    });

    // Run the TF listener in the local environment
    proc_tftree_->start("sh", QStringList() << "-c" << "ros2 run tf2_tools view_frames");
}

// =========================================================
// Multi-tab terminal emulator
// =========================================================
void MainWindow::on_btn_new_terminal_clicked()
{
    if (!use_ssh_ || !is_online_) {
        appendLog("<b>[TERMINAL]</b> Cannot open terminal: SSH mode disabled or host is offline.", true);
        return;
    }

    // 1. Create UI components
    QWidget *tab_widget = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout(tab_widget);
    layout->setContentsMargins(0, 0, 0, 0);

    // Output panel
    QTextEdit *term_output = new QTextEdit();
    term_output->setReadOnly(true);
    term_output->setStyleSheet("background-color: #1E1E1E; color: #CCCCCC; font-family: 'Ubuntu Mono', monospace; font-size: 10pt;");

    // Bottom input-area layout
    QHBoxLayout *input_layout = new QHBoxLayout();
    // Input box
    QLineEdit *term_input = new QLineEdit();
    term_input->setPlaceholderText("Type command...");
    term_input->setStyleSheet("background-color: #2D2D2D; color: white; border: 1px solid #3E3E3E;");
    // Ctrl+C Button
    QPushButton *btn_ctrl_c = new QPushButton("Ctrl+C");
    btn_ctrl_c->setStyleSheet("background-color: #d32f2f; color: white; font-weight: bold; padding: 3px;");
    btn_ctrl_c->setFixedWidth(60);

    input_layout->addWidget(term_input);
    input_layout->addWidget(btn_ctrl_c);
    layout->addWidget(term_output);
    layout->addLayout(input_layout);

    QString tab_name = QString("Term-%1").arg(terminal_count_++);
    int tab_index = ui->tabWidget_terminals->addTab(tab_widget, tab_name);
    ui->tabWidget_terminals->setCurrentIndex(tab_index);

    // 2. Create a persistent QProcessQProcess
    QProcess *proc = new QProcess(tab_widget);
    term_output->insertHtml("<i><font color=\"#888888\">Establishing SSH connection, please wait...</font></i><br>");
    // Add -tt to force pseudo-terminal allocation
    QString ssh_cmd = QString("sshpass -p '%1' ssh -tt -q -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR %2@%3")
                      .arg(remote_pwd_, remote_user_, remote_ip_);
    // After the process starts successfully, wait for
    connect(proc, &QProcess::started, [proc]() {
        QTimer::singleShot(1000, proc, [proc]() {
            if (proc->state() == QProcess::Running) {
                proc->write("\n");
            }
        });
    });

    // 3. Process standard output
    connect(proc, &QProcess::readyReadStandardOutput, [proc, term_output]() {
        QString output = QString::fromLocal8Bit(proc->readAllStandardOutput());
        // --- Clean the data and filter garbled text ---
        // Remove Bracketed Paste Mode control sequences
        output.replace(QRegExp("\\x1B\\[\\?2004[hl]"), ""); 
        // Remove terminal-title sequences and other non-color control codes
        output.replace(QRegExp("\\x1B\\][0-9];[^\x07]+\x07"), "");
        // Filter setlocale system warnings
        if (output.contains("setlocale")) return;

        // --- HTML Escape as HTML ---
        output.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
        output.replace("\r\n", "<br>").replace("\n", "<br>");
        output.replace(" ", "&nbsp;");

        // --- Convert native ANSI color codes to HTML ---
        // Match ANSI color sequences such as \\x1B[1;32m or \\x1B[0m
        QRegExp colorRegex("\\x1B\\[([0-9;]*)m");
        int pos = 0;
        while ((pos = colorRegex.indexIn(output, pos)) != -1) {
            QString codes = colorRegex.cap(1);
            QString htmlTag = "";

            if (codes.isEmpty() || codes == "0" || codes == "00") {
                htmlTag = "</font>"; // ANSI Reset code -> close the font tag
            } 
            else {
                // Color-palette mapping
                if (codes.contains("31")) htmlTag = "<font color=\"#FF5555\">";      // Red
                else if (codes.contains("32")) htmlTag = "<font color=\"#55FF55\">"; // Green
                else if (codes.contains("33")) htmlTag = "<font color=\"#FFFF55\">"; // Yellow
                else if (codes.contains("34")) htmlTag = "<font color=\"#5555FF\">"; // Blue
                else if (codes.contains("35")) htmlTag = "<font color=\"#FF55FF\">"; // Magenta
                else if (codes.contains("36")) htmlTag = "<font color=\"#55FFFF\">"; // Cyan
                else if (codes.contains("37") || codes.contains("39")) htmlTag = "<font color=\"white\">";
                
                if (htmlTag.isEmpty()) htmlTag = "</font>"; // For an unknown code, close the tag by default
            }
            // Replace ANSI codes in place with HTML tags
            output.replace(pos, colorRegex.matchedLength(), htmlTag);
            pos += htmlTag.length();
        }

        // --- Write to the terminal ---
        term_output->moveCursor(QTextCursor::End);
        term_output->insertHtml(output);
        term_output->verticalScrollBar()->setValue(term_output->verticalScrollBar()->maximum());
    });

    // 4. Bind the Enter key to write commands to the process
    connect(term_input, &QLineEdit::returnPressed, [proc, term_input]() {
        QString cmd = term_input->text();
        if (cmd.isEmpty()) return;
        term_input->clear();
        // Write commands directly into the persistent SSH session instead of repeatedly disconnecting and reconnecting
        proc->write((cmd + "\n").toLocal8Bit());
    });

    // 5. Ctrl+C Implement Ctrl+C
    connect(btn_ctrl_c, &QPushButton::clicked, [proc]() {
        // Send ASCII 0x03 (ETX), namely Ctrl+C
        proc->write("\x03");
    });

    // 6. Add exception handling for unexpected disconnections
    // Monitor process termination (normal exit or termination caused by a network disconnect)
    connect(proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), 
            [proc, term_output, term_input, btn_ctrl_c](int exitCode, QProcess::ExitStatus exitStatus) {
        
        term_output->moveCursor(QTextCursor::End);
        if (exitStatus == QProcess::CrashExit) {
            term_output->insertHtml("<br><br><i><b><font color=\"#FF5555\">[SSH Process Crashed]</font></b></i><br>");
        } 
        else {
            term_output->insertHtml(QString("<br><br><i><font color=\"#888888\">[SSH Connection Closed (ExitCode: %1)]</font></i><br>").arg(exitCode));
        }
        term_output->verticalScrollBar()->setValue(term_output->verticalScrollBar()->maximum());
        
        // After disconnection, immediately disable the input box and Ctrl+C button to prevent input from causing a crash
        term_input->setEnabled(false);
        term_input->setPlaceholderText("Connection closed.");
        btn_ctrl_c->setEnabled(false);
        
        proc->deleteLater();
    });

    // Monitor startup-level errors (for example, sshpass is missing or the system has no network connection)
    connect(proc, &QProcess::errorOccurred, [term_output, term_input, btn_ctrl_c](QProcess::ProcessError error) {
        term_output->moveCursor(QTextCursor::End);
        term_output->insertHtml(QString("<br><i><b><font color=\"#FF5555\">[Process Failed to Start: Error Code %1]</font></b></i><br>").arg(error));
        term_output->verticalScrollBar()->setValue(term_output->verticalScrollBar()->maximum());
        
        term_input->setEnabled(false);
        term_input->setPlaceholderText("Execution failed.");
        btn_ctrl_c->setEnabled(false);
    });

    // Start the terminal engine
    proc->start("sh", QStringList() << "-c" << ssh_cmd);
}

void MainWindow::slotCloseTerminalTab(int index)
{
    // Get this page's Widget
    QWidget *tab_widget = ui->tabWidget_terminals->widget(index);
    
    // Find the QProcess attached to this Tab page QProcess
    QProcess *proc = tab_widget->findChild<QProcess *>();
    if (proc) {
        // Disconnect all signals to prevent readyRead or finished callbacks from accessing dangling pointers during destruction UI
        proc->disconnect(); 
        // Terminate the underlying SSH process
        if (proc->state() != QProcess::NotRunning) {
            proc->kill();
            proc->waitForFinished(500); // Wait synchronously
        }
    }
    // Remove the Tab page from the UI framework
    ui->tabWidget_terminals->removeTab(index);
    // Safely release the UI memory
    tab_widget->deleteLater();
}
