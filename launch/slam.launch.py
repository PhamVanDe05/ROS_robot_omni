import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = '/home/pham-van-de/ros2_ws/src/robot_omni/config'

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'omni_base.lua'
        ],
        remappings=[
            ('/scan_1', '/scan_front_raw'), 
            ('/scan_2', '/scan_rear_raw'),  
            # CẤP ODOM ĐÃ LỌC TỪ EKF CHO CARTOGRAPHER
            ('/odom', '/odometry/filtered'), 
            ('/imu', '/base_imu')
        ]
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
        rviz_node
    ])