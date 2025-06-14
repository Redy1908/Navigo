# Navigo

# Requirements

- Ubuntu 22.04 (jammy) [link](https://releases.ubuntu.com/jammy/)

# Setup

1. Install Ros2 Humble, Gazebo and required dependencies:
   ```bash
   sudo ./install.sh
   ```

2. Create a python virtual environment and install the required packages:
   ```bash
   ./setup.sh
   ```

3. Source the ROS and Gazebo setup files in your `~/.bashrc`:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
   echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc && source ~/.bashrc
   ```

4. Set the Turtlebot3 model in your `~/.bashrc`:
   ```bash
    echo "export TURTLEBOT3_MODEL=waffle " >> ~/.bashrc && source ~/.bashrc
   ```

5. Build and run (from the root directory of the project):
   ```bash
   source .venv/bin/activate
   cd ros_ws
   python3 -m colcon build
   source install/setup.sh
   ros2 launch aws_robomaker_small_house_world small_house.launch.py gui:=true
   ```
