import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取参数文件路径
    slam_params_file = os.path.join(
        get_package_share_directory('custom_slam'),
        'config',
        'slam_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node', # 建图节点
            name='slam_toolbox',
            output='screen', # 关键配置：开启屏幕输出以便查看报错日志
            parameters=[slam_params_file]
        )
    ])
