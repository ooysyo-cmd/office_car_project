import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    fishbot_description_dir = get_package_share_directory('fishbot_description')
    ekf_param_path = os.path.join(fishbot_description_dir, 'config', 'ekf_real.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock (false for real robot)'),

        # map 坐标系 EKF：融合 odom + IMU + GPS
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_param_path,
                        {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/global')],
        ),

        # GPS → UTM
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_param_path,
                        {'use_sim_time': use_sim_time}],
            remappings=[('imu/data', 'imu/data_raw'),
                        ('gps/fix', 'gps/fix'),
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/global')],
        ),
    ])
