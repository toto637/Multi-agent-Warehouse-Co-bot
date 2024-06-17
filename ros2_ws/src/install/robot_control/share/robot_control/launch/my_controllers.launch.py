from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    pkg_share2 = get_package_share_directory('robot_control')

    default_model_path = os.path.join(pkg_share, 'urdf/my_robot_description.urdf.xacro')

    controller_params_file = os.path.join(pkg_share2, 'config/my_controllers.yaml')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_broad",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", 
                   "--controller-manager", 
                   "/controller_manager"
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(name='use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
            DeclareLaunchArgument(name='robot_description', default_value=default_model_path, description='Absolute path to robot urdf file'),
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
        ]
    )