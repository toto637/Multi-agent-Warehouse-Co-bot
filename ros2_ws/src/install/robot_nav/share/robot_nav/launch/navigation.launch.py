import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    
    package_name = "robot_nav"
    
    nav2_yaml = os.path.join(get_package_share_directory(package_name), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory(package_name), 'config', 'map1.yaml')
    rviz_config_file = os.path.join(get_package_share_directory(package_name), "rviz", "auton-nav-final-1.rviz")
    controller_yaml = os.path.join(get_package_share_directory('nav'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('nav'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('nav'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('nav'), 'config', 'recovery.yaml')
    ekf_yaml = os.path.join(get_package_share_directory('nav'), 'config', 'ekf.yaml')  # Path to EKF YAML file
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)                   # EKF


    use_rviz = LaunchConfiguration("rviz", default=True)
    
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file}]
        ),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator']}]),
        #----------------------------
        Node(
            package='robot_localization',  # Replace 'ekf_package_name' with the actual package name containing your EKF node
            executable='ekf_node',  # Replace 'ekf_node_executable' with the actual executable name of your EKF node
            name='ekf_filter_node',  # Name of the EKF node
            output='screen',
            parameters=[ekf_yaml, 
            {'use_sim_time': use_sim_time}]),

        #-----------------------------    

        Node(
        package= "rviz2",
        executable= "rviz2",
        arguments=["-d", rviz_config_file],
        output= "screen",
        condition=IfCondition(use_rviz)),
    ])
    