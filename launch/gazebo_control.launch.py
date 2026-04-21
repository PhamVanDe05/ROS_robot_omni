
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg = get_package_share_directory('robot_omni')
    
    urdf_file = os.path.join(pkg, 'urdf', 'omni_base.urdf')
    # Dùng hospital_full.world (Map đã được chứng minh là load thành công)
    world_file = os.path.join(pkg, 'worlds', 'hospital_full.world')
    # world_file = os.path.join(pkg, 'worlds', 'empty.sdf')
    bridge_config = os.path.join(pkg, 'config', 'bridge_config.yaml')

    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml') # <--- THÊM DÒNG NÀY

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ==========================================
    # 1. CÁC BIẾN MÔI TRƯỜNG BẮT BUỘC (Từ file chạy thành công)
    # ==========================================
    model_path = os.path.expanduser('~/ros2_ws/src/my_robot_gazebo/models')
    
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f"{os.path.dirname(pkg)}:{model_path}"
    )
    
    # BẮT BUỘC: Chỉ đường cho Gazebo tìm Plugin ROS 2 Control
    set_gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/jazzy/lib'
    )

    set_ros_args = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_ARGS',
        value=f'--ros-args -p robot_description:="{robot_description}"'
    )

    # ==========================================
    # 2. KHỞI ĐỘNG GAZEBO & ROBOT STATE
    # ==========================================
    # Mở Gazebo Sim bằng ExecuteProcess cho độ ổn định cao nhất
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': robot_description, 'use_sim_time': True}
        ]
    )

    # ==========================================
    # 3. THẢ XE (SPAWN) - Delay 3 giây 
    # ==========================================
    delayed_spawn = TimerAction(
        period=20.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                # Dùng -file thay vì -topic để chống văng Gazebo
                arguments=['-name', 'robot_omni',
                           '-file', urdf_file,
                           '-x', '0.0', '-y', '15.0', '-z', '1.0', '-Y', '-1.5'],
                output='screen'
            )
        ]
    )

    # ==========================================
    # 4. CẦU NỐI ROS-GZ (BRIDGE) - Delay 5 giây
    # ==========================================
    delayed_bridge = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
                output='screen'
            )
        ]
    )

    # ==========================================
    # 5. BỘ ĐIỀU KHIỂN (CONTROLLERS) - Delay 8 giây
    # ==========================================
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    mobile_base_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mobile_base_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    delayed_controllers = TimerAction(
        period=30.0,
        actions=[
            joint_state_broadcaster,
            mobile_base_controller
        ]
    )

# ==========================================
    # 6. BỘ LỌC EKF (Trộn Odom + IMU)
    # ==========================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}]
    )



    # TRẢ VỀ TOÀN BỘ HỆ THỐNG
    return LaunchDescription([
        set_gz_resource_path,
        set_gz_plugin_path,
        set_ros_args,
        gz_sim,
        robot_state_publisher,
        delayed_spawn,
        delayed_bridge,
        delayed_controllers,
        ekf_node
    ])