#!/bin/bash

if [ "$EUID" -ne 0 ]; then
  echo "Script must be run as root. Use sudo."
  exit 1
fi

if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$ID" = "ubuntu" ] && [ "$VERSION_ID" != "22.04" ]; then
        echo "This script is intended for Ubuntu 22.04. Exiting."
        exit 1
    fi
else
    echo "Cannot verify OS. This script is intended for Ubuntu 22.04. Exiting."
    exit 1
fi

apt update -y

# ==========================================
# Configures locale
# ==========================================
apt install locales -y
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ==========================================
# Setup for installing ROS 2 Humble
# ==========================================
apt install software-properties-common -y
add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo apt install /tmp/ros2-apt-source.deb

# ==========================================
# Install required packages
# ==========================================
sudo apt-get update -y && sudo apt upgrade -y

sudo apt-get install -y \
    build-essential \
    ros-humble-ros-core \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-tf-transformations \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot4-description \
    ros-humble-turtlebot4-msgs \
    ros-humble-turtlebot4-navigation \
    ros-humble-turtlebot4-node \
    ros-humble-turtlebot4-simulator
    
rm -rf /var/lib/apt/lists/*