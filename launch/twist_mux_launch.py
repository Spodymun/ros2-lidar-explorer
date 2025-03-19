from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    params_file_default = os.path.join(
        os.getenv('HOME'), "ws_lidar/src/ros2-lidar-explorer/config/twist_mux.yaml"
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params_file_default, description='Path to the parameters YAML file'),
        DeclareLaunchArgument('cmd_vel_out', default_value='/cmd_vel', description='cmd vel output topic'),

        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', LaunchConfiguration('cmd_vel_out'))},
            parameters=[LaunchConfiguration('params_file')]
        )
    ])
