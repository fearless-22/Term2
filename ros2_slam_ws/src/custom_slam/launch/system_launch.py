import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory("custom_slam")
    launch_dir = os.path.join(pkg_dir, "launch")

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "slam_web_launch.py"))
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "nav2_web_launch.py"))
    )

    return LaunchDescription(
        [
            slam,
            TimerAction(period=2.0, actions=[nav2]),
        ]
    )
