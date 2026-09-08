"""One owned process group, one map->odom authority, no recovery driving."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def stack(context):
    mode = LaunchConfiguration("mode").perform(context)
    ns = LaunchConfiguration("namespace").perform(context)
    params = LaunchConfiguration("params").perform(context)
    tf = [("/tf", "/tf"), ("/tf_static", "/tf_static")]
    if mode == "mapping":
        return [Node(package="slam_toolbox", executable="async_slam_toolbox_node",
                     name="slam_toolbox", namespace=ns, parameters=[params],
                     remappings=tf, output="screen")]
    nodes = [
        ("nav2_map_server", "map_server"),
        ("nav2_amcl", "amcl"),
        ("nav2_planner", "planner_server"),
        ("nav2_controller", "controller_server"),
        ("nav2_bt_navigator", "bt_navigator"),
    ]
    actions = [Node(package=pkg, executable=name, name=name, namespace=ns,
                    parameters=[params], output="screen",
                    remappings=tf + [("cmd_vel", "controller_cmd_vel")])
               for pkg, name in nodes]
    actions.append(Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager",
        name="lifecycle_manager", namespace=ns,
        parameters=[{"autostart": True, "bond_timeout": 4.0,
                     "node_names": [name for _, name in nodes]}], output="screen"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode"), DeclareLaunchArgument("namespace"),
        DeclareLaunchArgument("params"), OpaqueFunction(function=stack)])
