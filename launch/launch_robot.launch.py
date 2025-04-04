import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    SetEnvironmentVariable
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Workaround: Disable FastRTPS shared memory transport (Fixes SHM error)
    shm_fix = SetEnvironmentVariable(
        name='RMW_FASTRTPS_USE_SHM',
        value='0'
    )

    # Launch argument for ESP IP
    esp_ip_arg = DeclareLaunchArgument(
        'esp_ip',
        description='IP-Adresse des ESP'
    )
    esp_ip = LaunchConfiguration('esp_ip')

    # Package path
    package_name = 'ros2-lidar-explorer'
    pkg_path = get_package_share_directory(package_name)

    # Include rsp.launch.py (robot state publisher)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={  
            'use_sim_time': 'false',
            'use_ros2_control': 'false',
            'esp_ip': esp_ip
        }.items()
    )

    return LaunchDescription([
        shm_fix,
        esp_ip_arg,
        rsp,
    ])
