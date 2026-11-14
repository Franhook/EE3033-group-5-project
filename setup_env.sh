#!/bin/bash
# =================================================================
# Balance Robot Project - Environment Setup Script
# Target OS: Ubuntu 22.04 LTS
# ROS Version: ROS 2 Humble Hawksbill
# =================================================================
set -e

echo "========= [1/3] Updating System Apt Repositories ========="
sudo apt update && sudo apt upgrade -y

echo "========= [2/3] Installing System Tools & GUI Dependencies ========="
sudo apt install -y \
    sshpass \
    qtbase5-dev \
    qtchooser \
    qt5-qmake \
    qtbase5-dev-tools \
    libserial-dev \
    xdg-utils

echo "========= [3/3] Installing ROS 2 Navigation & Mapping Ecosystem ========="
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-slam-toolbox \
    ros-humble-explore-lite \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-tf2-tools

echo "================================================================="
echo " Environment setup completed successfully"
echo "================================================================="
