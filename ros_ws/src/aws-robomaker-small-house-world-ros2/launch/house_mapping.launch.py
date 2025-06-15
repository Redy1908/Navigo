import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("explorer"), "config")
    rviz_config = os.path.join(config_dir, "nav.rviz")
    
    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")
    pkg_slam_toolbox = get_package_share_directory("slam_toolbox")
    pkg_nav2_map_server = get_package_share_directory("nav2_map_server")

    house_world_path = os.path.join(
        get_package_share_directory("aws_robomaker_small_house_world"),
        "worlds",
        "house.world",
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": house_world_path}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "publish_period": "0.0",
        }.items(),
    )

    rviz2 = Node(
        package="rviz2",
        output="screen",
        executable="rviz2",
        name="rviz2_node",
        arguments=["-d", rviz_config],
    )

    explorer = Node(
        package="explorer",
        executable="explorer",
        name="explorer_node",
        output="screen",
        parameters=[],
    )

    map_saver_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_map_server, "launch", "map_saver_server.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    sim_group = GroupAction(actions=[gzserver_cmd, gzclient_cmd, robot_state_publisher_cmd, spawn_turtlebot_cmd])
    nav_group = GroupAction(actions=[nav2_cmd, slam_cmd, map_saver_server])
    ui_group = GroupAction(actions=[rviz2])
    explorer_group = GroupAction(actions=[explorer])

    ld = LaunchDescription()

    ld.add_action(sim_group)
    ld.add_action(nav_group)
    ld.add_action(ui_group)
    ld.add_action(explorer_group)

    return ld