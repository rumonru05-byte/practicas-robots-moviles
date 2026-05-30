import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Config file path
    config_file = os.path.join(
        get_package_share_directory('seg_tray'),
        'config',
        'corr_params.yaml'
    )

    print(f"Loading config from: {config_file}")  # Debug statement

    return LaunchDescription([
        Node(
            package='seg_tray',
            executable='corr_nav_wall',
            name='corr_nav_wall_node',
            output='screen',
            parameters=[config_file]
        ),
    ])