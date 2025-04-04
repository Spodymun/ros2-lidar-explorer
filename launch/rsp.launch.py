import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    esp_ip = LaunchConfiguration('esp_ip')

    # Locate the xacro file
    pkg_path = get_package_share_directory('ros2-lidar-explorer')
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Generate the robot_description parameter using xacro command
    robot_description = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=', use_sim_time,
        ' esp_ip:=', esp_ip
    ])

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control interface if true'),
        DeclareLaunchArgument(
            'esp_ip',
            description='IP address of the ESP32-based microcontroller'),

        robot_state_publisher_node
    ])
