#include "balance_gui/main_window.hpp"
#include "ui_main_window.h"

// =========================================================
// Remote-control module (keyboard + 3x3 UI button pad)
// =========================================================

void MainWindow::initTeleop()
{
    current_cmd_x_ = 0.0;
    current_cmd_z_ = 0.0;
    
    // Read the target speeds from the sliders during initialization
    target_linear_vel_ = ui->slider_linear->value() / 1000.0;
    target_angular_vel_ = ui->slider_angular->value() / 1000.0;

    // Create the continuous-publishing timer (20Hz = 50ms)
    pub_cmd_timer_ = new QTimer(this);
    connect(pub_cmd_timer_, &QTimer::timeout, this, &MainWindow::slotPubCmdVel);

    ui->checkBox_sacc->setChecked(true);
    // Initially disable all nine control buttons
    setTeleopButtonsEnabled(false);

    // --- Bind UI button press/release events to the same logic as keyboard input ---
    // Forward (W)
    connect(ui->btn_key_W, &QPushButton::pressed, this, [this](){ handleTeleopInput(Qt::Key_W, true); });
    connect(ui->btn_key_W, &QPushButton::released, this, [this](){ handleTeleopInput(Qt::Key_W, false); });
    // Backward (X)
    connect(ui->btn_key_X, &QPushButton::pressed, this, [this](){ handleTeleopInput(Qt::Key_X, true); });
    connect(ui->btn_key_X, &QPushButton::released, this, [this](){ handleTeleopInput(Qt::Key_X, false); });
    // Turn left (A)
    connect(ui->btn_key_A, &QPushButton::pressed, this, [this](){ handleTeleopInput(Qt::Key_A, true); });
    connect(ui->btn_key_A, &QPushButton::released, this, [this](){ handleTeleopInput(Qt::Key_A, false); });
    // Turn right (D)
    connect(ui->btn_key_D, &QPushButton::pressed, this, [this](){ handleTeleopInput(Qt::Key_D, true); });
    connect(ui->btn_key_D, &QPushButton::released, this, [this](){ handleTeleopInput(Qt::Key_D, false); });
    // Brake (S)
    connect(ui->btn_key_S, &QPushButton::pressed, this, [this](){ handleTeleopInput(Qt::Key_S, true); });
    connect(ui->btn_key_S, &QPushButton::released, this, [this](){ handleTeleopInput(Qt::Key_S, false); });
    // Forward-left (Q)
    connect(ui->btn_key_Q, &QPushButton::pressed, this, [this](){ handleTeleopInput(Qt::Key_Q, true); });
    connect(ui->btn_key_Q, &QPushButton::released, this, [this](){ handleTeleopInput(Qt::Key_Q, false); });
    // Forward-right (E)
    connect(ui->btn_key_E, &QPushButton::pressed, this, [this](){ handleTeleopInput(Qt::Key_E, true); });
    connect(ui->btn_key_E, &QPushButton::released, this, [this](){ handleTeleopInput(Qt::Key_E, false); });
    // Backward-left (Z)
    connect(ui->btn_key_Z, &QPushButton::pressed, this, [this](){ handleTeleopInput(Qt::Key_Z, true); });
    connect(ui->btn_key_Z, &QPushButton::released, this, [this](){ handleTeleopInput(Qt::Key_Z, false); });
    // Backward-right (C)
    connect(ui->btn_key_C, &QPushButton::pressed, this, [this](){ handleTeleopInput(Qt::Key_C, true); });
    connect(ui->btn_key_C, &QPushButton::released, this, [this](){ handleTeleopInput(Qt::Key_C, false); });
}

void MainWindow::setTeleopButtonsEnabled(bool enabled)
{
    // Enable or disable the nine directional buttons in a batch
    ui->btn_key_Q->setEnabled(enabled);
    ui->btn_key_W->setEnabled(enabled);
    ui->btn_key_E->setEnabled(enabled);
    ui->btn_key_A->setEnabled(enabled);
    ui->btn_key_S->setEnabled(enabled);
    ui->btn_key_D->setEnabled(enabled);
    ui->btn_key_Z->setEnabled(enabled);
    ui->btn_key_X->setEnabled(enabled);
    ui->btn_key_C->setEnabled(enabled);
}

void MainWindow::on_checkBox_teleop_stateChanged(int arg1)
{
    if (arg1 == Qt::Checked) {
        appendLog("<b>[TELEOP]</b> Teleoperation mode enabled. Use WASD or the buttons to control the robot.");
        setTeleopButtonsEnabled(true);
        pub_cmd_timer_->start(50); // Start the 20 Hz continuous-publishing timer
        this->setFocus(); 
    } 
    else {
        appendLog("<b>[TELEOP]</b> Teleoperation mode disabled.");
        setTeleopButtonsEnabled(false);
        pub_cmd_timer_->stop();
        current_cmd_x_ = 0.0;
        current_cmd_z_ = 0.0;
        qnode.pub_cmd_vel(0.0, 0.0); // Stop the timer and send one brake command
    }
}

// Whether input comes from the keyboard or buttons, handle the target speed here
void MainWindow::handleTeleopInput(int key_code, bool is_pressed)
{
    if (!ui->checkBox_teleop->isChecked()) return;

    // If any movement key is released, immediately set the speed to zero (brake)
    if (!is_pressed) {
        current_cmd_x_ = 0.0;
        current_cmd_z_ = 0.0;
        return;
    }

    // Assign the current speed according to the key
    switch(key_code) {
        case Qt::Key_W: current_cmd_x_ = target_linear_vel_;  current_cmd_z_ = 0.0; break;
        case Qt::Key_X: current_cmd_x_ = -target_linear_vel_; current_cmd_z_ = 0.0; break;
        case Qt::Key_A: current_cmd_x_ = 0.0; current_cmd_z_ = target_angular_vel_; break;
        case Qt::Key_D: current_cmd_x_ = 0.0; current_cmd_z_ = -target_angular_vel_; break;
        case Qt::Key_Q: current_cmd_x_ = target_linear_vel_;  current_cmd_z_ = target_angular_vel_; break;
        case Qt::Key_E: current_cmd_x_ = target_linear_vel_;  current_cmd_z_ = -target_angular_vel_; break;
        case Qt::Key_Z: current_cmd_x_ = -target_linear_vel_; current_cmd_z_ = -target_angular_vel_; break;
        case Qt::Key_C: current_cmd_x_ = -target_linear_vel_; current_cmd_z_ = target_angular_vel_; break;
        case Qt::Key_S: current_cmd_x_ = 0.0; current_cmd_z_ = 0.0; break;
        default: break;
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    // Ignore operating-system key-repeat events caused by holding a key
    if (!ui->checkBox_teleop->isChecked() || event->isAutoRepeat()) return;
    handleTeleopInput(event->key(), true);
}

void MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    if (!ui->checkBox_teleop->isChecked() || event->isAutoRepeat()) return;
    handleTeleopInput(event->key(), false);
}

void MainWindow::slotPubCmdVel()
{
    // Whenever the timer fires while remote control is enabled, continuously feed the current target speed to the backend
    qnode.pub_cmd_vel(current_cmd_x_, current_cmd_z_);
}

void MainWindow::on_slider_linear_valueChanged(int value)
{
    target_linear_vel_ = value / 1000.0;
    ui->label_v_val->setText(QString::number(target_linear_vel_, 'f', 2));
}

void MainWindow::on_slider_angular_valueChanged(int value)
{
    target_angular_vel_ = value / 1000.0;
    ui->label_w_val->setText(QString::number(target_angular_vel_, 'f', 2));
}
