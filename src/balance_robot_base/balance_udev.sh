#!/bin/bash

echo "配置 Balance Robot 硬件设备别名..."
# ==========================================================
# 1. STM32 STM32 controller (balance_controller) (balance_controller)
# Corresponds to Serial: 0002 Serial: 0002
# ==========================================================
echo "配置 STM32 控制器 -> /dev/balance_controller"
# Rule 1: match a CP2102 chip
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="balance_controller"' >/etc/udev/rules.d/balance_controller.rules
# Rule 2: match a CH9102/CH343 chip
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="balance_controller"' >>/etc/udev/rules.d/balance_controller.rules
# Rule 3: match ACM mode (with some drivers))
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="balance_controller"' >>/etc/udev/rules.d/balance_controller.rules

# ==========================================================
# 2. LiDAR (balance_lidar) (balance_lidar)
# Corresponds to Serial: 0001 Serial: 0001
# ==========================================================
echo "配置 激光雷达 -> /dev/balance_lidar"
# Rule1: CP2102
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="balance_lidar"' >/etc/udev/rules.d/balance_lidar.rules
# Rule2: CH9102/CH343
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="balance_lidar"' >>/etc/udev/rules.d/balance_lidar.rules
# Rule3: ACM
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="balance_lidar"' >>/etc/udev/rules.d/balance_lidar.rules

# ==========================================================
# 3. USB USB camera (balance_cam) (balance_cam)
# When using Pi Camera Module 3, this section is normally unnecessary because it defaults to /dev/video0 /dev/video0
# For a USB camera, enter its idVendor and idProduct idProduct
# ==========================================================
echo "配置 摄像头 -> /dev/balance_cam"

# --- Automatically detect the currently connected USB camera (generic method)) ---
# This rule creates the balance_cam symbolic link for the first video device detected by the system balance_cam
# echo 'KERNEL=="video0", MODE:="0777", SYMLINK+="balance_cam"' >/etc/udev/rules.d/balance_camera.rules
# ---  Bind a specific USB-camera ID ID ---
# WheelTec C100 Camera
echo 'KERNEL=="video*", ATTRS{idVendor}=="2ce3", ATTRS{idProduct}=="0010", MODE:="0777", SYMLINK+="balance_cam"' >>/etc/udev/rules.d/balance_camera.rules

# ==========================================================
# 4. Restart the udev service to apply the configuration
# ==========================================================
echo "正在重启 udev 服务..."
service udev reload
sleep 2
service udev restart
echo "配置完成"
