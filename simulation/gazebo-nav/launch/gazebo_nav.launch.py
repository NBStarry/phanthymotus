import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("phanthymotus_sim_nav"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    world = str(share / "worlds" / "synthetic_room.sdf")
    map_yaml = str(share / "maps" / "synthetic_room.yaml")
    params = str(share / "config" / "nav2_params.yaml")
    localization_mode = os.environ.get("LOCALIZATION_MODE", "ground_truth")
    if localization_mode not in {"ground_truth", "amcl"}:
        raise RuntimeError(f"unsupported LOCALIZATION_MODE: {localization_mode}")

    gazebo = ExecuteProcess(cmd=["ign", "gazebo", "-r", "-s", "-v", "3", world], output="screen")
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/phanthymotus_sim_nav/gz_cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/phanthymotus_sim_nav/gz_odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/phanthymotus_sim_nav/gz_scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/world/synthetic_room/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        remappings=[
            ("/phanthymotus_sim_nav/gz_cmd_vel", "/cmd_vel"),
            ("/phanthymotus_sim_nav/gz_odom", "/phanthymotus_sim_nav/gz_odom_raw"),
            ("/phanthymotus_sim_nav/gz_scan", "/phanthymotus_sim_nav/gz_scan_raw"),
        ],
        output="screen",
    )
    adapter = Node(package="phanthymotus_sim_nav", executable="navigation_node", output="screen")
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params, {"use_sim_time": True, "yaml_filename": map_yaml}],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": True, "autostart": True, "node_names": ["map_server"]}
        ],
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / "launch" / "localization_launch.py")),
        launch_arguments={
            "map": map_yaml,
            "params_file": params,
            "use_sim_time": "True",
            "autostart": "True",
            "use_composition": "False",
        }.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(nav2_share / "launch" / "navigation_launch.py")),
        launch_arguments={
            "params_file": params,
            "use_sim_time": "True",
            "autostart": "True",
            "use_composition": "False",
        }.items(),
    )
    map_actions = [localization] if localization_mode == "amcl" else [map_server, map_lifecycle]
    return LaunchDescription([
        gazebo,
        bridge,
        adapter,
        TimerAction(period=3.0, actions=map_actions),
        TimerAction(period=6.0 if localization_mode == "amcl" else 5.0, actions=[nav2]),
    ])
