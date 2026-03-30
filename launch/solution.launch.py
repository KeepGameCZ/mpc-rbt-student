import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'config.rviz')

    localization_cmd = Node(
        package='mpc_rbt_student',
        executable='localization_node',
        name='localization_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    static_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=[
            '--x', '-0.5', 
            '--y', '0.0', 
            '--z', '0.0', 
            '--yaw', '0.0',
            '--pitch', '0.0', 
            '--roll', '0.0', 
            '--frame-id', 'map', 
            '--child-frame-id', 'odom'
        ]
    )
    
    planning_cmd = Node(
        package='mpc_rbt_student',
        executable='planning_node',
        name='planning_node',
    )

    return LaunchDescription([
        localization_cmd, rviz_cmd, static_tf_cmd, planning_cmd
    ])
