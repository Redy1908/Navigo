# Navigo

# Requirements

- Ubuntu 24.04 (Noble) [link](https://releases.ubuntu.com/noble/)

# Setup

1. Install Ros2 Jazzy following the instructions from the [official documentation](https://docs.ros.org/en/jazzy/Installation.html). After the installation run:
   ```bash
   echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && source ~/.bashrc
   ```

2. Install Gazebo Harmonic following the instructions from the [official documentation](https://gazebosim.org/docs/harmonic/ros_installation/).
   ```bash
   sudo apt-get install ros-${ROS_DISTRO}-ros-gz
   ```
   ```bash
   source ~/.bashrc
   ```

3. Create and activate a python virtual environment:
   ```bash
   $ python3 -m venv .venv
   $ source .venv/bin/activate
   ```

4. Install the required python packages:
   ```bash
   pip install -r requirements.txt
   ```
