# Navigo

# Requirements

- Ubuntu 22.04 (jammy) [link](https://releases.ubuntu.com/jammy/)

# Setup

1. Install Ros2 Humble, Gazebo and required dependencies:
   ```bash
   sudo ./install.sh
   ```

2. Source the ROS and Gazebo setup files in your `~/.bashrc`:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
   echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc && source ~/.bashrc
   echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/<PATH-TO-NAVIGO>/ros_ws/src/litterbug/models" && source ~/.bashrc
   echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc && source ~/.bashrc
   ```

3. Set the Turtlebot3 model in your `~/.bashrc`:
   ```bash
   echo "export TURTLEBOT3_MODEL=waffle " >> ~/.bashrc && source ~/.bashrc
   ```

4. Create and activate a Python virtual environment in the root directory of the project:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   ```

5. Install the required Python packages:
   ```bash
   pip install -r requirements.txt
   ```

6. Build the ROS workspace:
   ```bash
   cd ros_ws
   python3 -m colcon build
   source install/setup.sh
   ```

# Mapping

(Assuming all the previous steps have been completed)

There is an already built map of the environment in the `ros_ws/src/aws-robomaker-small-house-world-ros2/maps` directory.

If you want to create a new map:
```bash
cd ros_ws
ros2 launch aws_robomaker_small_house_world house_mapping.launch.py 
```