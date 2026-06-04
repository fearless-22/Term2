import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("custom_slam")
    nav2_dir = get_package_share_directory("nav2_bringup")

    nav2_params = LaunchConfiguration("nav2_params")
    exploration_params = LaunchConfiguration("exploration_params")
    system_params = LaunchConfiguration("system_params")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "autostart": autostart,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Web bridge uses wall-clock ROS time unless /clock is provided",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically activate Nav2 lifecycle nodes",
            ),
            DeclareLaunchArgument(
                "nav2_params",
                default_value=os.path.join(pkg_dir, "config", "nav2_web_params.yaml"),
                description="Nav2 parameter file",
            ),
            DeclareLaunchArgument(
                "exploration_params",
                default_value=os.path.join(pkg_dir, "config", "exploration_params.yaml"),
                description="Frontier exploration parameter file",
            ),
            DeclareLaunchArgument(
                "system_params",
                default_value=os.path.join(pkg_dir, "config", "system_params.yaml"),
                description="State manager and map evaluator parameter file",
            ),
            navigation_launch,
            Node(
                package="custom_slam",
                executable="map_evaluator.py",
                name="map_evaluator",
                output="screen",
                parameters=[system_params],
            ),
            Node(
                package="custom_slam",
                executable="exploration_node.py",
                name="exploration_node",
                output="screen",
                parameters=[exploration_params],
            ),
            Node(
                package="custom_slam",
                executable="state_manager.py",
                name="state_manager",
                output="screen",
                parameters=[system_params],
            ),
        ]
    )
