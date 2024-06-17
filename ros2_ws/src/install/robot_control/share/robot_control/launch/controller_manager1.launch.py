import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    controller_config = os.path.join(
        get_package_share_directory('robot_control'),
        'config',
        'controller_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_config],
            output='screen'
        )
    ])
