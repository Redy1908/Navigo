import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition # Per gzclient opzionale

def generate_launch_description():
    pkg_world_path_provider = get_package_share_directory("aws_robomaker_small_house_world")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_turtlebot3_gazebo = get_package_share_directory("turtlebot3_gazebo")

    world_file_name = "small_house.world"
    house_world_path = os.path.join(pkg_world_path_provider, "worlds", world_file_name)

    # Argomenti di lancio
    use_sim_time = LaunchConfiguration("use_sim_time")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    yaw_pose = LaunchConfiguration("yaw_pose")
    turtlebot3_model = LaunchConfiguration("turtlebot3_model")
    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")

    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=house_world_path,
        description="Percorso completo al file del mondo SDF."
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Usa il tempo di simulazione (Gazebo)."
    )

    declare_x_pose_arg = DeclareLaunchArgument(
        "x_pose",
        default_value="0.0",
        description="Posizione X iniziale del robot."
    )

    declare_y_pose_arg = DeclareLaunchArgument(
        "y_pose",
        default_value="0.0",
        description="Posizione Y iniziale del robot."
    )

    declare_z_pose_arg = DeclareLaunchArgument(
        "z_pose",
        default_value="0.01",
        description="Posizione Z iniziale del robot."
    )

    declare_yaw_pose_arg = DeclareLaunchArgument(
        "yaw_pose",
        default_value="0.0",
        description="Orientamento Yaw iniziale del robot."
    )

    declare_turtlebot3_model_arg = DeclareLaunchArgument(
        "turtlebot3_model",
        default_value="burger",
        description="Modello di TurtleBot3 da usare."
    )

    declare_gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Avvia l'interfaccia grafica di Gazebo (gzclient)."
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world, "verbose": "false"}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        ),
        condition=IfCondition(gui)
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "turtlebot3_model": turtlebot3_model
        }.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, "launch", "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={
            "x_pose": x_pose,
            "y_pose": y_pose,
            "z_pose": z_pose,
            "yaw": yaw_pose,
            "turtlebot3_model": turtlebot3_model
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_arg)
    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_x_pose_arg)
    ld.add_action(declare_y_pose_arg)
    ld.add_action(declare_z_pose_arg)
    ld.add_action(declare_yaw_pose_arg)
    ld.add_action(declare_turtlebot3_model_arg)
    ld.add_action(declare_gui_arg)

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld