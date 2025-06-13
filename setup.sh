#!/bin/bash

if [ "$EUID" -ne 0 ]; then
  echo "Script must be run as root. Use sudo."
  exit 1
fi

if [ ! -f /etc/os-release ]; then
  echo "This script is intended for Ubuntu 24.04. Exiting."
  exit 1
fi

apt update -y

# ==========================================
# Configurazione Locales
# ==========================================
apt install locales -y
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ==========================================
# Installazione ROS Jazzy
# ==========================================
apt install software-properties-common -y
add-apt-repository universe -y

apt update && apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
apt install /tmp/ros2-apt-source.deb

apt update && apt upgrade -y
apt install ros-jazzy-desktop -y
rm -rf /var/lib/apt/lists/*

echo 'source /opt/ros/jazzy/setup.bash' >> /home/$USER/.bashrc
echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc

# ==========================================
# Installazione Nav2, Gazebo, e altri pacchetti ROS
# ==========================================
apt-get update -y
apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-slam-toolbox \
    ros-jazzy-tf-transformations \
    ros-jazzy-ros-gz
rm -rf /var/lib/apt/lists/*

# ==========================================
# Installazione TurtleBot4
# ==========================================
apt update -y
apt install -y \
    ros-jazzy-turtlebot4-description \
    ros-jazzy-turtlebot4-msgs \
    ros-jazzy-turtlebot4-navigation \
    ros-jazzy-turtlebot4-node \
    ros-jazzy-turtlebot4-simulator/*
rm -rf /var/lib/apt/lists/*