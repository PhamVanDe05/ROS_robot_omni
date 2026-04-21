import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_omni')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Đường dẫn file config và map
    param_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_dir, 'maps', 'hospital_map.yaml') 
    
    # ĐƯỜNG DẪN ĐẾN FILE CẤU HÌNH BỘ LỌC C++ BẠN VỪA TẠO
    laser_filter_config = os.path.join(pkg_dir, 'config', 'scan_filter.yaml')
    
    # File RViz mặc định của Nav2
    rviz_config_file = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    return LaunchDescription([

        # 1.1 Node lọc Lidar TRƯỚC (Đã có sẵn của bạn)
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_front_filter',
            output='screen',
            parameters=[laser_filter_config],
            remappings=[
                ('scan', '/scan_front_raw'),      
                ('scan_filtered', '/scan')        # Lidar trước xuất ra /scan
            ]
        ),

        # 1.2 Node lọc Lidar TRƯỚC (Đã có sẵn của bạn)
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_rear_filter',
            output='screen',
            parameters=[laser_filter_config],
            remappings=[
                ('scan', '/scan_rear_raw'),       # Hút dữ liệu thô Lidar sau
                ('scan_filtered', '/scan_rear')   # TẠO TOPIC MỚI: Lidar sau xuất ra /scan_rear
            ]
        ),
        # 2. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[param_file, {'yaml_filename': map_file}]
        ),

        # 3. AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[param_file]
        ),

        # 4. Khối Planner (Chứa cả Global Costmap)
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[param_file]
        ),

        # 5. Khối Controller (Tài xế + Local Costmap)
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[param_file],
            # CẮM VÀO ĐÚNG CỔNG REFERENCE CỦA ROS 2 JAZZY CONTROL
            remappings=[('/cmd_vel', '/mobile_base_controller/reference')] 
        ),

        # 6. Khối Behavior (Xử lý sự cố kẹt)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[param_file],
            # Tương tự cho khối gỡ kẹt
            remappings=[('/cmd_vel', '/mobile_base_controller/reference')]
        ),

        # 7. Khối Nhạc trưởng (BT Navigator)
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[param_file]
        ),

        # 8. QUẢN GIA LIFECYCLE (Quản lý 6 khối)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': [
                    'map_server', 
                    'amcl', 
                    'planner_server', 
                    'controller_server', 
                    'behavior_server', 
                    'bt_navigator',
                    'waypoint_follower',
                    'filter_mask_server',        
                    'costmap_filter_info_server'
                ]} 
            ]
        ),

        # 9. RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
        
        # 10. Khối Waypoint Follower (Quản lý mảng tọa độ)
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[param_file]
        ),
        # 11. Node đọc file ảnh Vùng Cấm
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            parameters=[param_file]
        ),

        # 12. Node phiên dịch ảnh thành lệnh cấm
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            parameters=[param_file]
        ),
    ])