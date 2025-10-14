from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():
    # 获取包的路径
    yaml_path = get_package_share_directory('feature_tracker')

    # 获取参数文件路径
    params_file = os.path.join(yaml_path, 'config', 'params.yaml')

    # 确保参数文件存在
    if not os.path.exists(params_file):
        raise FileNotFoundError(f"Config file not found: {params_file}")
    
    return LaunchDescription([
        Node(
            package='video_player',
            executable="video_player_node",
            name="video_player",
            parameters=[params_file],
        ),
        Node(
            package='feature_tracker',
            executable ="feature_tracker_node",
            name="feature_tracker",
            parameters=[params_file],
        ),
        Node(
            package = "vins_estimator",
            executable = "vins_estimator_node",
            name="vins_estimator",
            parameters=[params_file],
        )
    ])
