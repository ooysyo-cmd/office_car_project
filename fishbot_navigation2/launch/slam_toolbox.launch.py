import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    fishbot_nav_dir = get_package_share_directory('fishbot_navigation2')
    slam_param_path = os.path.join(fishbot_nav_dir, 'config', 'slam_toolbox_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),

        launch_ros.actions.Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_param_path,
                        {'use_sim_time': use_sim_time}],
        ),
    ])
