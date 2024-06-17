
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    pkg_share2 = get_package_share_directory('robot_control')

    default_model_path = os.path.join(pkg_share, 'urdf/my_robot_description.urdf.xacro')

    controller_params_file = os.path.join(pkg_share2, 'config/my_controllers.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(name='robot_description', default_value=default_model_path, description='Absolute path to robot urdf file'),
        

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[controller_params_file],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'), 'robot_description': LaunchConfiguration('robot_description')}],
            output='screen'),

        Node(
            package='joint_state_broadcaster',
            executable='joint_state_broadcaster_spawner',
            parameters=[controller_params_file],
            output='screen'),

        Node(
            package='diff_drive_controller',
            executable='diff_drive_controller_spawner',
            parameters=[controller_params_file],
            output='screen'),
    ])
