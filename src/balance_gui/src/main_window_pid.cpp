#include "balance_gui/main_window.hpp"
#include "ui_main_window.h"

// Receive parameters from the lower-level node and update the UI SpinBox values
void MainWindow::slotUpdatePIDParams(float b_kp, float b_kd, float v_kp, float v_ki, 
                                     float t_kp, float t_ki, float t_kd, float m_ang)
{
    // Balance loop
    ui->spin_bal_kp->setValue(b_kp);
    ui->spin_bal_kd->setValue(b_kd);
    ui->spin_bal_mid->setValue(m_ang);
    // Velocity loop
    ui->spin_vel_kp->setValue(v_kp);
    ui->spin_vel_ki->setValue(v_ki);
    // Steering loop
    ui->spin_turn_kp->setValue(t_kp);
    ui->spin_turn_ki->setValue(t_ki);
    ui->spin_turn_kd->setValue(t_kd);

    appendLog("<b>[PID]</b> Parameter synchronization from hardware successful!");
}

// ---------------------------------------------------------
// Global button operations
// ---------------------------------------------------------
void MainWindow::on_btn_pid_get_clicked()
{
    // command_type: 2 means CMD_GET
    qnode.pub_pid_command(2);
    appendLog("<b>[PID]</b> Requesting parameters...");
}

void MainWindow::on_btn_pid_save_clicked()
{
    // command_type: 1 means CMD_SAVE
    qnode.pub_pid_command(1);
    appendLog("<b>[PID]</b> Save current parameters to Flash.", true);
}

void MainWindow::on_btn_pid_set_all_clicked()
{
    // Read the current values from all SpinBox widgets
    float b_kp = ui->spin_bal_kp->value();
    float b_kd = ui->spin_bal_kd->value();
    float m_ang = ui->spin_bal_mid->value();

    float v_kp = ui->spin_vel_kp->value();
    float v_ki = ui->spin_vel_ki->value();

    float t_kp = ui->spin_turn_kp->value();
    float t_ki = ui->spin_turn_ki->value();
    float t_kd = ui->spin_turn_kd->value();

    // command_type: 3 means CMD_SET_ALL
    qnode.pub_pid_command(3, 0, 0.0f, b_kp, b_kd, v_kp, v_ki, t_kp, t_ki, t_kd, m_ang);
    appendLog("<b>[PID]</b> Sending ALL parameters...");
}

// ---------------------------------------------------------
// Modify a single parameter
// ---------------------------------------------------------
// Generic helper for sending one parameter
void sendSingleParam(QNode& qnode, uint8_t param_id, double val) {
    // command_type: 4 means CMD_SET_SINGLE
    qnode.pub_pid_command(4, param_id, static_cast<float>(val));
}

// Balance loop (1, 2, 8)
void MainWindow::on_btn_set_bal_kp_clicked() {
    sendSingleParam(qnode, 1, ui->spin_bal_kp->value());
    appendLog(QString("<b>[PID]</b> Balance Kp set to: %1").arg(ui->spin_bal_kp->value()));
}

void MainWindow::on_btn_set_bal_kd_clicked() {
    sendSingleParam(qnode, 2, ui->spin_bal_kd->value());
    appendLog(QString("<b>[PID]</b> Balance Kd set to: %1").arg(ui->spin_bal_kd->value()));
}

void MainWindow::on_btn_set_bal_mid_clicked() {
    sendSingleParam(qnode, 8, ui->spin_bal_mid->value());
    appendLog(QString("<b>[PID]</b> Middle Angle set to: %1").arg(ui->spin_bal_mid->value()));
}

// Velocity loop (3, 4)
void MainWindow::on_btn_set_vel_kp_clicked() {
    sendSingleParam(qnode, 3, ui->spin_vel_kp->value());
    appendLog(QString("<b>[PID]</b> Velocity Kp set to: %1").arg(ui->spin_vel_kp->value()));
}

void MainWindow::on_btn_set_vel_ki_clicked() {
    sendSingleParam(qnode, 4, ui->spin_vel_ki->value());
    appendLog(QString("<b>[PID]</b> Velocity Ki set to: %1").arg(ui->spin_vel_ki->value()));
}

// Steering loop (5, 6, 7)
void MainWindow::on_btn_set_turn_kp_clicked() {
    sendSingleParam(qnode, 5, ui->spin_turn_kp->value());
    appendLog(QString("<b>[PID]</b> Turn Kp set to: %1").arg(ui->spin_turn_kp->value()));
}

void MainWindow::on_btn_set_turn_ki_clicked() {
    sendSingleParam(qnode, 6, ui->spin_turn_ki->value());
    appendLog(QString("<b>[PID]</b> Turn Ki set to: %1").arg(ui->spin_turn_ki->value()));
}

void MainWindow::on_btn_set_turn_kd_clicked() {
    sendSingleParam(qnode, 7, ui->spin_turn_kd->value());
    appendLog(QString("<b>[PID]</b> Turn Kd set to: %1").arg(ui->spin_turn_kd->value()));
}
