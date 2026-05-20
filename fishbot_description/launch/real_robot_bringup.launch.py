import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    fishbot_dir = get_package_share_directory('fishbot_description')
    ekf_param_path = os.path.join(fishbot_dir, 'config', 'ekf_real.yaml')
    urdf_path = os.path.join(fishbot_dir, 'urdf', 'real_robot.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock (false for real robot)'),

        # 1. TF 发布
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'use_sim_time': use_sim_time}],
        ),

        # 2. IMU 驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('hipnuc_imu'),
                '/launch', '/imu_spec_msg.launch.py']),
        ),

        # 3. 激光雷达驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('lslidar_driver'),
                '/launch', '/lsn10p_launch.py']),
        ),

        # 4. micro-ROS agent: 串口桥接到 ESP32（/odom ←→ /cmd_vel）
        launch_ros.actions.Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial','--dev','/dev/fishbot'],
            output='screen'
        ),
        # 5. EKF: 融合 /odom + /imu/data_raw → /odometry/local
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_param_path,
                        {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/local')],
        ),
    ])
