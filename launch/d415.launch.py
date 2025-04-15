from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[
                {'enable_color': True},
                {'rgb_camera.color_profile': '424x240x15'}, # u can also use '848x480x30'
                {'rgb_camera.color_format': 'RGB8'},
                {'enable_depth': True},
                {'align_depth.enable': True},
                {'pointcloud.enable': False}, # change this to true if you need those 
                {'pointcloud.texture_stream': 'RS2_STREAM_COLOR'},
                {'initial_reset': True}
            ]
        )
    ])